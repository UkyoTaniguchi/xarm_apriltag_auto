#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU監視 + ICPズレ検出 + Depth表示 + 折れ線グラフ可視化（ROI可視対応・動的基準更新）
- ROI受信(/monitor/roi)後にIMU・ICP・グラフ更新を開始
- IMUの傾き or ICPズレがしきい値を超えたら /recalibration/run を自動コール
"""

import math, time, threading, signal
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import cv2
import open3d as o3d
import rospy
from threading import Lock
from collections import deque
from sensor_msgs.msg import Imu, Image, CameraInfo
from std_msgs.msg import Float32MultiArray, Float32
from cv_bridge import CvBridge
from std_srvs.srv import Trigger, TriggerResponse


class ImuIcpMonitor:
    def __init__(self):
        rospy.init_node("imu_icp_monitor")

        # ---- パラメータ ----
        self.tilt_threshold_deg = rospy.get_param("~tilt_threshold_deg", 1.0)
        self.icp_threshold = rospy.get_param("~icp_threshold", 0.05)
        self.alpha = rospy.get_param("~alpha", 0.1)
        self.update_rate = rospy.get_param("~update_rate", 10.0)
        self.window_len = int(rospy.get_param("~window_len", 500))
        self.depth_topic = rospy.get_param("~depth_topic", "/cam_2/depth/image_rect_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/cam_2/depth/camera_info")
        self.imu_topic = rospy.get_param("~imu_topic", "/cam_2/imu")
        self.roi_topic = rospy.get_param("~roi_topic", "/monitor/roi")

        # 再キャリブレーション関連
        self.recalib_cooldown_sec = rospy.get_param("~recalib_cooldown_sec", 60.0)

        # ROI（正規化座標）
        self.roi_xmin = 0.0
        self.roi_xmax = 1.0
        self.roi_ymin = 0.0
        self.roi_ymax = 1.0

        # 点群生成・ICP関連
        self.downsample_step = 4
        self.voxel_size = 0.02
        self.icp_rate = 2.0
        self.icp_max_corr = 0.1
        self.depth_trunc = 1.5
        self.normal_radius = 0.05
        self.normal_max_nn = 30

        # ==========================
        # 内部状態
        # ==========================
        self.filtered_gravity_dir = None   # フィルタ後の重力ベクトル
        self.reference_dir = None          # 基準姿勢の重力ベクトル
        self.last_update_time = 0.0        # IMU更新タイムスタンプ
        self.last_icp_time = 0.0           # ICP更新タイムスタンプ

        self.buffer_lock = Lock()
        self.initial_pcd = None            # ICPの基準点群
        self.latest_depth_vis = None       # Depth表示用カラー画像
        self.K = None                      # カメラ行列 (fx, fy, cx, cy, width, height)
        self.has_camera_info = False
        self.roi_received = False          # ROI受信済みフラグ

        # 再キャリブレーション状態
        self.recalib_running = False
        self.last_recalib_time = 0.0

        # グラフ用バッファ
        N = self.window_len
        self.dx_buf = deque(maxlen=N)
        self.dy_buf = deque(maxlen=N)
        self.dz_buf = deque(maxlen=N)
        self.tilt_buf = deque(maxlen=N)
        self.frame_index = 0

        self.bridge = CvBridge()

        # パブリッシャ
        self.pub_icp = rospy.Publisher("/monitor/icp_trans_norm", Float32, queue_size=1)
        self.pub_tilt = rospy.Publisher("/monitor/tilt_deg", Float32, queue_size=1)

        # 再キャリブレーションサービスクライアント
        self.recalib_client = rospy.ServiceProxy("/recalibration/run", Trigger)

        # ==========================
        # サブスクライバ
        # ==========================
        rospy.Subscriber(self.imu_topic, Imu, self.imu_callback, queue_size=50)
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback, queue_size=1)
        rospy.Subscriber(self.depth_topic, Image, self.depth_callback, queue_size=1)
        rospy.Subscriber(self.roi_topic, Float32MultiArray, self.roi_callback, queue_size=1)

        rospy.loginfo("IMU + ICP + Depth Monitor ready.")

        # 基準リセットサービス
        self.reset_srv = rospy.Service("/monitor/reset_reference", Trigger, self.reset_reference_callback)

        # ==========================
        # Depth表示スレッド
        # ==========================
        threading.Thread(target=self._depth_display_loop, daemon=True).start()

        # ==========================
        # 折れ線グラフ
        # ==========================
        self._init_plot()
        self.anim = animation.FuncAnimation(
            self.fig, self.update_plot,
            interval=int(1000 / self.update_rate),
            cache_frame_data=False
        )

        signal.signal(signal.SIGINT, self._sigint_handler)
        self.spin()

    # ---------------------------------------------------------------------
    # 基準リセット処理
    # ---------------------------------------------------------------------
    def reset_reference(self):
        """
        IMU/ICPの基準をリセットする。
        - 次のDepthで ICP 基準点群を取り直す
        - 現在のフィルタ済み重力ベクトルを IMU 基準とする
        """
        self.initial_pcd = None

        if self.filtered_gravity_dir is not None:
            self.reference_dir = self.filtered_gravity_dir.copy()

        rospy.loginfo("Reference reset: IMU & ICP baseline updated.")

    def reset_reference_callback(self, _req):
        self.reset_reference()
        return TriggerResponse(success=True, message="Reference reset.")

    # ---------------------------------------------------------------------
    # 再キャリブレーション自動トリガ判定
    # ---------------------------------------------------------------------
    def check_recalib_condition(self, tilt_deg=None, icp_norm=None):
        """
        tilt_deg: IMU の傾き角度 [deg]
        icp_norm: ICP の平行移動量のノルム [m]
        → どちらかがしきい値を超えたら /recalibration/run を呼び出す
        """
        now = time.time()

        # クールダウン中は何もしない
        if now - self.last_recalib_time < self.recalib_cooldown_sec:
            return

        # すでに再キャリブ呼び出し中なら何もしない
        if self.recalib_running:
            return

        tilt_exceeded = (tilt_deg is not None and tilt_deg > self.tilt_threshold_deg)
        icp_exceeded = (icp_norm is not None and icp_norm > self.icp_threshold)

        if not (tilt_exceeded or icp_exceeded):
            return

        rospy.logwarn("=== しきい値超過検出 → /recalibration/run を呼び出します ===")
        rospy.logwarn(f"  tilt={tilt_deg}, icp_norm={icp_norm}, "
                      f"tilt_th={self.tilt_threshold_deg}, icp_th={self.icp_threshold}")

        self.recalib_running = True
        self.last_recalib_time = now

        # サービス呼び出しはブロッキングなので別スレッドで実行
        threading.Thread(target=self._call_recalib_service, daemon=True).start()

    def _call_recalib_service(self):
        """実際に /recalibration/run を呼び出すスレッド用関数"""
        try:
            rospy.loginfo("/recalibration/run を呼び出し中...")
            res = self.recalib_client()
            if res.success:
                rospy.loginfo(f"/recalibration/run 成功: {res.message}")
            else:
                rospy.logerr(f"/recalibration/run 失敗: {res.message}")
        except Exception as e:
            rospy.logerr(f"/recalibration/run 呼び出しエラー: {e}")
        finally:
            # 再度トリガ可能状態に戻す（クールダウン時間は check 側で管理）
            self.recalib_running = False

    # ---------------------------------------------------------------------
    def roi_callback(self, msg):
        """ROIを外部トピックから更新"""
        if len(msg.data) == 4:
            self.roi_xmin, self.roi_ymin, self.roi_xmax, self.roi_ymax = msg.data
            self.roi_received = True
            rospy.loginfo(f"ROI updated to {msg.data}")

    # ---------------------------------------------------------------------
    def camera_info_callback(self, msg):
        fx, fy, cx, cy = msg.K[0], msg.K[4], msg.K[2], msg.K[5]
        self.K = (fx, fy, cx, cy, msg.width, msg.height)
        self.has_camera_info = True

    # ---------------------------------------------------------------------
    def _init_plot(self):
        self.fig, (self.ax_icp, self.ax_tilt) = plt.subplots(2, 1, figsize=(10, 6))
        self.fig.suptitle("ICP Δ[m] + IMU Tilt [deg]", fontsize=13)

        (self.line_dx,) = self.ax_icp.plot([], [], label="Δx [m]")
        (self.line_dy,) = self.ax_icp.plot([], [], label="Δy [m]")
        (self.line_dz,) = self.ax_icp.plot([], [], label="Δz [m]")

        self.ax_icp.set_ylim(-0.2, 0.2)
        self.ax_icp.set_ylabel("ICP Δ[m]")
        self.ax_icp.grid(True)
        self.ax_icp.legend(loc="center left", bbox_to_anchor=(1.02, 0.5))

        (self.line_tilt,) = self.ax_tilt.plot([], [], label="Tilt [deg]")
        self.ax_tilt.set_ylim(0, 10)
        self.ax_tilt.set_ylabel("Tilt [deg]")
        self.ax_tilt.set_xlabel("Frame Index")
        self.ax_tilt.grid(True)
        self.ax_tilt.legend(loc="center left", bbox_to_anchor=(1.02, 0.5))

        self.fig.tight_layout(rect=[0, 0, 0.85, 1])

    # ---------------------------------------------------------------------
    def imu_callback(self, msg):
        """IMUから重力ベクトルを推定し、傾き角度を計算して監視"""
        if not self.roi_received:
            return

        now = time.time()
        if now - self.last_update_time < 1.0 / self.update_rate:
            return
        self.last_update_time = now

        acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z,
        ])
        norm_acc = np.linalg.norm(acc)
        if norm_acc < 1e-6:
            return

        gravity_dir = acc / norm_acc

        if self.filtered_gravity_dir is None:
            self.filtered_gravity_dir = gravity_dir.copy()
        if self.reference_dir is None:
            self.reference_dir = gravity_dir.copy()

        # 一次フィルタ
        self.filtered_gravity_dir = (1 - self.alpha) * self.filtered_gravity_dir + self.alpha * gravity_dir
        self.filtered_gravity_dir /= np.linalg.norm(self.filtered_gravity_dir)

        # 基準姿勢との角度
        cos_angle = np.clip(np.dot(self.filtered_gravity_dir, self.reference_dir), -1.0, 1.0)
        tilt_deg = math.degrees(math.acos(cos_angle))

        self.pub_tilt.publish(Float32(data=tilt_deg))

        with self.buffer_lock:
            self.tilt_buf.append(tilt_deg)

        # ★ しきい値チェック（IMU側）
        self.check_recalib_condition(tilt_deg=tilt_deg, icp_norm=None)

    # ---------------------------------------------------------------------
    def depth_callback(self, msg):
        """DepthからROI点群を生成し、ICPで初期点群との差分を計算"""
        if not self.roi_received:
            return
        if not self.has_camera_info:
            return

        now = time.time()
        if now - self.last_icp_time < 1.0 / self.icp_rate:
            return
        self.last_icp_time = now

        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) / 1000.0
            elif depth.dtype != np.float32:
                return

            H_full, W_full = depth.shape

            # ROI → ピクセル座標
            xmin = int(self.roi_xmin * W_full)
            xmax = int(self.roi_xmax * W_full)
            ymin = int(self.roi_ymin * H_full)
            ymax = int(self.roi_ymax * H_full)

            step = self.downsample_step
            depth_ds = depth[::step, ::step]

            fx, fy, cx, cy, W0, H0 = self.K
            fx, fy, cx, cy = fx / step, fy / step, cx / step, cy / step

            H, W = depth_ds.shape

            xmin //= step
            xmax //= step
            ymin //= step
            ymax //= step

            mask = np.zeros_like(depth_ds, dtype=np.uint8)
            mask[ymin:ymax, xmin:xmax] = 1

            depth_masked = np.where(mask, depth_ds, 0)
            depth_masked = np.clip(depth_masked, 0, self.depth_trunc)

            # Depth可視化用
            depth_vis = np.clip(np.nan_to_num(depth_ds), 0, self.depth_trunc)
            depth_norm = (depth_vis / self.depth_trunc * 255).astype(np.uint8)
            depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
            cv2.rectangle(depth_color, (xmin, ymin), (xmax, ymax), (0, 0, 255), 2)
            self.latest_depth_vis = depth_color

            # Open3D 点群生成
            intr = o3d.camera.PinholeCameraIntrinsic(W, H, fx, fy, cx, cy)
            depth_o3d = o3d.geometry.Image(depth_masked)
            pcd = o3d.geometry.PointCloud.create_from_depth_image(
                depth_o3d, intr, depth_scale=1.0, depth_trunc=self.depth_trunc
            )
            pcd = pcd.voxel_down_sample(self.voxel_size)

            if np.asarray(pcd.points).shape[0] < 3:
                return

            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=self.normal_radius,
                    max_nn=self.normal_max_nn
                )
            )

            # 初回は基準点群を記録
            if self.initial_pcd is None:
                self.initial_pcd = pcd
                rospy.loginfo("Initial ICP reference captured (ROI).")
                return

            # ICP 登録
            reg = o3d.pipelines.registration.registration_icp(
                pcd,
                self.initial_pcd,
                max_correspondence_distance=self.icp_max_corr,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
            )
            T = reg.transformation
            dx, dy, dz = T[0, 3], T[1, 3], T[2, 3]

            with self.buffer_lock:
                self.dx_buf.append(dx)
                self.dy_buf.append(dy)
                self.dz_buf.append(dz)

            icp_norm = float(math.sqrt(dx * dx + dy * dy + dz * dz))
            self.pub_icp.publish(Float32(data=icp_norm))

            # ★ しきい値チェック（ICP側）
            self.check_recalib_condition(tilt_deg=None, icp_norm=icp_norm)

        except Exception as e:
            rospy.logwarn_throttle(1.0, f"Depth ICP failed: {e}")

    # ---------------------------------------------------------------------
    def _depth_display_loop(self):
        cv2.namedWindow("Depth", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Depth", 960, 720)
        while not rospy.is_shutdown():
            if self.latest_depth_vis is not None:
                cv2.imshow("Depth", self.latest_depth_vis)
                cv2.waitKey(1)
            time.sleep(0.05)
        cv2.destroyAllWindows()

    # ---------------------------------------------------------------------
    def update_plot(self, _frame):
        if not self.roi_received:
            return []

        with self.buffer_lock:
            dx = np.array(self.dx_buf)
            dy = np.array(self.dy_buf)
            dz = np.array(self.dz_buf)
            tilt = np.array(self.tilt_buf)

        m = max(len(dx), len(dy), len(dz), len(tilt), 1)
        x = np.arange(self.frame_index - m + 1, self.frame_index + 1)

        def pad(arr):
            buf = np.zeros(m)
            if len(arr) > 0:
                buf[-len(arr):] = arr
            return buf

        dx = pad(dx)
        dy = pad(dy)
        dz = pad(dz)
        tilt = pad(tilt)

        self.line_dx.set_data(x, dx)
        self.line_dy.set_data(x, dy)
        self.line_dz.set_data(x, dz)
        self.line_tilt.set_data(x, tilt)

        if self.frame_index > 1:
            xmin = max(0, self.frame_index - self.window_len)
            xmax = self.frame_index if xmin < self.frame_index else xmin + 1
            self.ax_icp.set_xlim(xmin, xmax)
            self.ax_tilt.set_xlim(xmin, xmax)

        self.frame_index += 1
        return [self.line_dx, self.line_dy, self.line_dz, self.line_tilt]

    # ---------------------------------------------------------------------
    def spin(self):
        threading.Thread(target=rospy.spin, daemon=True).start()
        plt.show()
        cv2.destroyAllWindows()

    # ---------------------------------------------------------------------
    def _sigint_handler(self, _sig, _frame):
        plt.close("all")
        cv2.destroyAllWindows()
        rospy.signal_shutdown("SIGINT")


if __name__ == "__main__":
    ImuIcpMonitor()
