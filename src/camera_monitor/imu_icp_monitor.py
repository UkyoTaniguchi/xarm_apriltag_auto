#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU監視 + ICPズレ検出 + Depth表示 + 折れ線グラフ可視化（ROI対応・動的基準更新）
- ROI(/monitor/roi) 受信後に IMU・ICP を開始
- 傾き or ICPズレがしきい値超過 → /recalibration/run を自動コール
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


# ============================================================
# メインクラス
# ============================================================
class ImuIcpMonitor:
    """
    - ROIで限定されたDepth点群からICP差分を計算
    - IMUから傾き角度を算出
    - グラフ・可視化・異常検知
    - しきい値超過で /recalibration/run を自動実行
    """

    def __init__(self):
        rospy.init_node("imu_icp_monitor")

        # --------------------------------------------------
        # パラメータ
        # --------------------------------------------------
        # 閾値
        self.max_tilt_deg = rospy.get_param("~tilt_threshold_deg", 1.0)
        self.max_icp_norm = rospy.get_param("~icp_threshold", 0.05)
        self.gravity_filter_alpha = rospy.get_param("~alpha", 0.1)

        # 更新周期
        self.imu_rate = rospy.get_param("~update_rate", 10.0)
        self.icp_rate = 2.0

        # Depth/Camera
        self.depth_topic = rospy.get_param("~depth_topic", "/cam_2/depth/image_rect_raw")
        self.caminfo_topic = rospy.get_param("~camera_info_topic", "/cam_2/depth/camera_info")
        self.imu_topic = rospy.get_param("~imu_topic", "/cam_2/imu")
        self.roi_topic = rospy.get_param("~roi_topic", "/monitor/roi")

        # 再キャリブレーションクールダウン
        self.recalib_cooldown_sec = rospy.get_param("~recalib_cooldown_sec", 60.0)

        # Depth領域設定（正規化ROI）
        self.roi_xmin, self.roi_ymin = 0.0, 0.0
        self.roi_xmax, self.roi_ymax = 1.0, 1.0
        self.roi_active = False

        # ICP関連
        self.downsample_step = 4
        self.voxel_size = 0.02
        self.icp_max_corr = 0.1
        self.depth_trunc = 1.5
        self.normal_radius = 0.05
        self.normal_max_nn = 30

        # カメラ行列情報
        self.K = None
        self.has_camera_info = False

        # 内部状態（IMU）
        self.filtered_gravity = None
        self.reference_gravity = None
        self.last_imu_time = 0.0

        # 内部状態（ICP）
        self.initial_pcd = None
        self.last_icp_time = 0.0

        # 再キャリブ状態
        self.recalib_active = False
        self.last_recalib_time = 0.0

        # Depth表示
        self.depth_vis = None

        # グラフバッファ
        buffer_cap = int(rospy.get_param("~window_len", 500))
        self.dx_hist = deque(maxlen=buffer_cap)
        self.dy_hist = deque(maxlen=buffer_cap)
        self.dz_hist = deque(maxlen=buffer_cap)
        self.tilt_hist = deque(maxlen=buffer_cap)

        self.frame_idx = 0
        self.buffer_lock = Lock()
        self.bridge = CvBridge()

        # --------------------------------------------------
        # Publisher
        # --------------------------------------------------
        self.pub_icp = rospy.Publisher("/monitor/icp_trans_norm", Float32, queue_size=1)
        self.pub_tilt = rospy.Publisher("/monitor/tilt_deg", Float32, queue_size=1)

        # 再キャリブ client
        self.recalib_client = rospy.ServiceProxy("/recalibration/run", Trigger)

        # --------------------------------------------------
        # Subscriber
        # --------------------------------------------------
        rospy.Subscriber(self.imu_topic, Imu, self.on_imu)
        rospy.Subscriber(self.caminfo_topic, CameraInfo, self.on_camera_info)
        rospy.Subscriber(self.depth_topic, Image, self.on_depth)
        rospy.Subscriber(self.roi_topic, Float32MultiArray, self.on_roi)

        # --------------------------------------------------
        # Reference reset service
        # --------------------------------------------------
        self.reset_srv = rospy.Service(
            "/monitor/reset_reference",
            Trigger,
            self._reset_reference_srv
        )

        rospy.loginfo("IMU+ICP Monitor initialized.")

        # Depth Viewer
        # threading.Thread(target=self._depth_view_loop, daemon=True).start()

        # グラフ初期化
        self._init_plot()
        self.anim = animation.FuncAnimation(
            self.fig,
            self.update_plot,
            interval=int(1000 / self.imu_rate),
            cache_frame_data=False
        )

        # Ctrl+C対応
        signal.signal(signal.SIGINT, self._sigint_handler)

        self.run()

    # ============================================================
    # 基準リセット
    # ============================================================
    def reset_reference(self):
        """IMU/ICP の基準をリセットする."""
        self.initial_pcd = None

        if self.filtered_gravity is not None:
            self.reference_gravity = self.filtered_gravity.copy()

        rospy.loginfo("Reference reset for IMU & ICP.")

    def _reset_reference_srv(self, _req):
        self.reset_reference()
        return TriggerResponse(success=True, message="Reference reset completed.")

    # ============================================================
    # ROI
    # ============================================================
    def on_roi(self, msg):
        if len(msg.data) == 4:
            self.roi_xmin, self.roi_ymin, self.roi_xmax, self.roi_ymax = msg.data
            self.roi_active = True
            rospy.loginfo(f"ROI updated: {msg.data}")

    # ============================================================
    # CameraInfo
    # ============================================================
    def on_camera_info(self, msg):
        fx, fy, cx, cy = msg.K[0], msg.K[4], msg.K[2], msg.K[5]
        self.K = (fx, fy, cx, cy, msg.width, msg.height)
        self.has_camera_info = True

    # ============================================================
    # IMU処理
    # ============================================================
    def on_imu(self, msg):
        """重力ベクトルから傾き角度を推定し、監視"""
        if not self.roi_active:
            return

        now = time.time()
        if now - self.last_imu_time < 1.0 / self.imu_rate:
            return
        self.last_imu_time = now

        acc = np.array([msg.linear_acceleration.x,
                        msg.linear_acceleration.y,
                        msg.linear_acceleration.z])
        norm = np.linalg.norm(acc)
        if norm < 1e-6:
            return
        gravity = acc / norm

        if self.filtered_gravity is None:
            self.filtered_gravity = gravity.copy()
        if self.reference_gravity is None:
            self.reference_gravity = gravity.copy()

        # LPF
        self.filtered_gravity = \
            (1 - self.gravity_filter_alpha) * self.filtered_gravity + \
            self.gravity_filter_alpha * gravity
        self.filtered_gravity /= np.linalg.norm(self.filtered_gravity)

        # 角度差
        dot = np.clip(np.dot(self.filtered_gravity, self.reference_gravity), -1.0, 1.0)
        tilt_deg = math.degrees(math.acos(dot))

        self.pub_tilt.publish(Float32(tilt_deg))

        with self.buffer_lock:
            self.tilt_hist.append(tilt_deg)

        # しきい値判定（IMU）
        self._check_recalib_trigger(tilt=tilt_deg, icp=None)

    # ============================================================
    # ICP処理
    # ============================================================
    def on_depth(self, msg):
        if not (self.roi_active and self.has_camera_info):
            return

        now = time.time()
        if now - self.last_icp_time < 1.0 / self.icp_rate:
            return
        self.last_icp_time = now

        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            # 単位変換
            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) / 1000.0
            elif depth.dtype != np.float32:
                return

            H_full, W_full = depth.shape

            # ROI → pixel
            xmin = int(self.roi_xmin * W_full)
            xmax = int(self.roi_xmax * W_full)
            ymin = int(self.roi_ymin * H_full)
            ymax = int(self.roi_ymax * H_full)

            # ダウンサンプル
            step = self.downsample_step
            depth_ds = depth[::step, ::step]

            fx, fy, cx, cy, W0, H0 = self.K
            fx /= step; fy /= step; cx /= step; cy /= step
            H, W = depth_ds.shape

            xmin //= step; xmax //= step
            ymin //= step; ymax //= step

            # ROI Mask
            mask = np.zeros_like(depth_ds, dtype=np.uint8)
            mask[ymin:ymax, xmin:xmax] = 1
            depth_masked = np.where(mask, depth_ds, 0)
            depth_masked = np.clip(depth_masked, 0, self.depth_trunc)

            # Depth可視化
            depth_vis = np.clip(np.nan_to_num(depth_ds), 0, self.depth_trunc)
            depth_norm = (depth_vis / self.depth_trunc * 255).astype(np.uint8)
            depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
            cv2.rectangle(depth_color, (xmin, ymin), (xmax, ymax), (0, 0, 255), 2)
            self.depth_vis = depth_color

            # 点群生成
            intr = o3d.camera.PinholeCameraIntrinsic(W, H, fx, fy, cx, cy)
            depth_o3d = o3d.geometry.Image(depth_masked)

            pcd = o3d.geometry.PointCloud.create_from_depth_image(
                depth_o3d, intr, depth_scale=1.0, depth_trunc=self.depth_trunc
            )
            pcd = pcd.voxel_down_sample(self.voxel_size)

            if len(pcd.points) < 3:
                return

            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=self.normal_radius,
                    max_nn=self.normal_max_nn
                )
            )

            # 初回 → 基準点群として保存
            if self.initial_pcd is None:
                self.initial_pcd = pcd
                rospy.loginfo("Initial ICP reference saved.")
                return

            # ICP計算
            reg = o3d.pipelines.registration.registration_icp(
                pcd, self.initial_pcd,
                self.icp_max_corr,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
            )
            T = reg.transformation
            dx, dy, dz = T[0, 3], T[1, 3], T[2, 3]

            with self.buffer_lock:
                self.dx_hist.append(dx)
                self.dy_hist.append(dy)
                self.dz_hist.append(dz)

            icp_norm = float(math.sqrt(dx*dx + dy*dy + dz*dz))
            self.pub_icp.publish(Float32(icp_norm))

            # しきい値チェック（ICP）
            self._check_recalib_trigger(tilt=None, icp=icp_norm)

        except Exception as e:
            rospy.logwarn_throttle(1.0, f"ICP failed: {e}")

    # ============================================================
    # Depth Viewer
    # ============================================================
    def _depth_view_loop(self):
        cv2.namedWindow("Depth", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Depth", 960, 720)
        while not rospy.is_shutdown():
            if self.depth_vis is not None:
                cv2.imshow("Depth", self.depth_vis)
                cv2.waitKey(1)
            time.sleep(0.05)
        cv2.destroyAllWindows()

    # ============================================================
    # 再キャリブレーショントリガ判定
    # ============================================================
    def _check_recalib_trigger(self, tilt=None, icp=None):
        now = time.time()

        # cooldown
        if now - self.last_recalib_time < self.recalib_cooldown_sec:
            return
        if self.recalib_active:
            return

        tilt_flag = (tilt is not None and tilt > self.max_tilt_deg)
        icp_flag = (icp is not None and icp > self.max_icp_norm)

        if not (tilt_flag or icp_flag):
            return

        rospy.logwarn("=== Recalibration Triggered ===")
        rospy.logwarn(f" tilt={tilt}, icp={icp}")

        self.recalib_active = True
        self.last_recalib_time = now

        threading.Thread(target=self._call_recalib_service, daemon=True).start()

    def _call_recalib_service(self):
        try:
            rospy.loginfo("Calling /recalibration/run ...")
            res = self.recalib_client()
            if res.success:
                rospy.loginfo("Recalibration OK.")
            else:
                rospy.logerr("Recalibration FAILED.")
        except Exception as e:
            rospy.logerr(f"Recalibration service error: {e}")
        finally:
            self.recalib_active = False

    # ============================================================
    # グラフ描画
    # ============================================================
    def _init_plot(self):
        self.fig, (self.ax_icp, self.ax_tilt) = plt.subplots(2, 1, figsize=(4, 2))
        self.fig.suptitle("ICP Δ[m] + IMU Tilt [deg]")

        (self.line_dx,) = self.ax_icp.plot([], [], label="Δx [m]")
        (self.line_dy,) = self.ax_icp.plot([], [], label="Δy [m]")
        (self.line_dz,) = self.ax_icp.plot([], [], label="Δz [m]")

        self.ax_icp.set_ylim(-0.2, 0.2)
        self.ax_icp.set_ylabel("ICP Δ[m]")
        self.ax_icp.grid(True)
        self.ax_icp.legend(loc="center left", bbox_to_anchor=(1.02, 0.5))

        (self.line_tilt,) = self.ax_tilt.plot([], [], label="tilt [deg]")
        self.ax_tilt.set_ylim(0, 10)
        self.ax_tilt.set_xlabel("Frame Index")
        self.ax_tilt.grid(True)
        self.ax_tilt.legend(loc="center left", bbox_to_anchor=(1.02, 0.5))

        self.fig.tight_layout(rect=[0, 0, 0.85, 1])

    def update_plot(self, _):
        if not self.roi_active:
            return []

        with self.buffer_lock:
            dx = np.array(self.dx_hist)
            dy = np.array(self.dy_hist)
            dz = np.array(self.dz_hist)
            tilt = np.array(self.tilt_hist)

        max_len = max(len(dx), len(dy), len(dz), len(tilt), 1)
        x = np.arange(self.frame_idx - max_len + 1, self.frame_idx + 1)

        # パディング
        def pad(a):
            buf = np.zeros(max_len)
            if len(a) > 0:
                buf[-len(a):] = a
            return buf

        dx = pad(dx)
        dy = pad(dy)
        dz = pad(dz)
        tilt = pad(tilt)

        self.line_dx.set_data(x, dx)
        self.line_dy.set_data(x, dy)
        self.line_dz.set_data(x, dz)
        self.line_tilt.set_data(x, tilt)

        if self.frame_idx > 1:
            xmin = max(0, self.frame_idx - len(dx))
            xmax = self.frame_idx
            self.ax_icp.set_xlim(xmin, xmax)
            self.ax_tilt.set_xlim(xmin, xmax)

        self.frame_idx += 1
        return [self.line_dx, self.line_dy, self.line_dz, self.line_tilt]

    # ============================================================
    def run(self):
        threading.Thread(target=rospy.spin, daemon=True).start()
        plt.show()
        cv2.destroyAllWindows()

    # ============================================================
    def _sigint_handler(self, *_):
        plt.close("all")
        cv2.destroyAllWindows()
        rospy.signal_shutdown("SIGINT")


if __name__ == "__main__":
    ImuIcpMonitor()
