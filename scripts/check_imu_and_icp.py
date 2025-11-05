#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU監視 + ICPズレ検出 + Depth表示 + 折れ線グラフ可視化（ROI可視対応・動的基準更新）
- 上段: ICP平行移動(Δx,Δy,Δz) [m]
- 下段: IMU傾き角(Tilt) [deg]
- Depth表示は全体表示＋赤枠ROI描画
- ICPはROI内のみを対象に計算
- 再キャリブレーション完了時にIMU・ICP基準を更新
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
from std_srvs.srv import Trigger
from cv_bridge import CvBridge


class ImuIcpMonitor:
    def __init__(self):
        rospy.init_node("imu_icp_monitor")

        # ===== パラメータ =====
        self.tilt_threshold_deg = rospy.get_param("~tilt_threshold_deg", 1.0)
        self.icp_threshold = rospy.get_param("~icp_threshold", 0.05)
        self.alpha = rospy.get_param("~alpha", 0.1)
        self.update_rate = rospy.get_param("~update_rate", 10.0)
        self.window_len = int(rospy.get_param("~window_len", 500))

        self.depth_topic = rospy.get_param("~depth_topic", "/camera/camera/depth/image_rect_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/camera/depth/camera_info")
        self.imu_topic = rospy.get_param("~imu_topic", "/camera/camera/imu")

        # ROI設定（0〜1の割合指定）
        self.roi_xmin = rospy.get_param("~roi_xmin", 0.25)
        self.roi_xmax = rospy.get_param("~roi_xmax", 0.4)
        self.roi_ymin = rospy.get_param("~roi_ymin", 0.2)
        self.roi_ymax = rospy.get_param("~roi_ymax", 0.5)

        # 軽量化設定
        self.downsample_step = 4
        self.voxel_size = 0.02
        self.icp_rate = 2.0
        self.icp_max_corr = 0.1
        self.depth_trunc = 1.5
        self.normal_radius = 0.05
        self.normal_max_nn = 30

        # ===== 状態変数 =====
        self.filtered_gravity_dir = None
        self.reference_dir = None
        self.last_update_time = 0.0
        self.last_icp_time = 0.0
        self.buffer_lock = Lock()
        self.is_recalibrating = False

        self.initial_pcd = None
        self.latest_pcd = None
        self.latest_depth_vis = None

        N = self.window_len
        self.dx_buf, self.dy_buf, self.dz_buf = deque(maxlen=N), deque(maxlen=N), deque(maxlen=N)
        self.tilt_buf = deque(maxlen=N)

        self.frame_index = 0  # 横軸用カウンタ

        self.K = None
        self.has_camera_info = False
        self.bridge = CvBridge()

        # ===== サービス =====
        try:
            rospy.wait_for_service("/recalibration/run", timeout=3.0)
            self.recalibration_client = rospy.ServiceProxy("/recalibration/run", Trigger)
        except rospy.ROSException:
            self.recalibration_client = None
            rospy.logwarn("/recalibration/run が見つからないため、スキップ")

        # ===== Subscriber =====
        rospy.Subscriber(self.imu_topic, Imu, self.imu_callback, queue_size=50)
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback, queue_size=1)
        rospy.Subscriber(self.depth_topic, Image, self.depth_callback, queue_size=1)
        rospy.loginfo("IMU + ICP + Depth Monitor ready.")

        # ===== Depth表示スレッド =====
        threading.Thread(target=self._depth_display_loop, daemon=True).start()

        # ===== プロット =====
        self._init_plot()
        self.anim = animation.FuncAnimation(
            self.fig, self.update_plot,
            interval=int(1000 / self.update_rate),
            cache_frame_data=False
        )
        signal.signal(signal.SIGINT, self._sigint_handler)
        self.spin()

    # =========================================================
    def camera_info_callback(self, msg: CameraInfo):
        fx = msg.K[0]; fy = msg.K[4]; cx = msg.K[2]; cy = msg.K[5]
        self.K = (fx, fy, cx, cy, msg.width, msg.height)
        self.has_camera_info = True

    # =========================================================
    def _init_plot(self):
        self.fig, (self.ax_icp, self.ax_tilt) = plt.subplots(2, 1, figsize=(10, 6))
        self.fig.suptitle("ICP Δ[m] + IMU Tilt [deg]", fontsize=13)
        (self.line_dx,) = self.ax_icp.plot([], [], label="Δx [m]")
        (self.line_dy,) = self.ax_icp.plot([], [], label="Δy [m]")
        (self.line_dz,) = self.ax_icp.plot([], [], label="Δz [m]")
        self.ax_icp.set_ylim(-0.2, 0.2  )
        self.ax_icp.set_ylabel("ICP Δ[m]")
        self.ax_icp.grid(True)
        # ---- 凡例を右外に出す ----
        self.ax_icp.legend(loc="center left", bbox_to_anchor=(1.02, 0.5))

        (self.line_tilt,) = self.ax_tilt.plot([], [], label="Tilt [deg]")
        self.ax_tilt.set_ylim(0, 10)
        self.ax_tilt.set_ylabel("Tilt [deg]")
        self.ax_tilt.set_xlabel("Frame Index")
        self.ax_tilt.grid(True)
        # ---- 下段も外側に ----
        self.ax_tilt.legend(loc="center left", bbox_to_anchor=(1.02, 0.5))

        self.fig.tight_layout(rect=[0, 0, 0.85, 1])  # 凡例分だけ右に余白

    # =========================================================
    def imu_callback(self, msg: Imu):
        now = time.time()
        if now - self.last_update_time < 1.0 / self.update_rate:
            return
        self.last_update_time = now
        a = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        n = np.linalg.norm(a)
        if n < 1e-6:
            return
        g = a / n
        if self.filtered_gravity_dir is None:
            self.filtered_gravity_dir = g.copy()
        if self.reference_dir is None:
            self.reference_dir = g.copy()
        self.filtered_gravity_dir = (1 - self.alpha) * self.filtered_gravity_dir + self.alpha * g
        self.filtered_gravity_dir /= np.linalg.norm(self.filtered_gravity_dir)
        tilt = math.degrees(math.acos(np.clip(np.dot(self.filtered_gravity_dir, self.reference_dir), -1, 1)))
        with self.buffer_lock:
            self.tilt_buf.append(tilt)
        if tilt > self.tilt_threshold_deg and not self.is_recalibrating:
            self._start_recalibration(f"Tilt {tilt:.2f}°")

    # =========================================================
    def depth_callback(self, msg: Image):
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

            # === ROI設定 ===
            H_full, W_full = depth.shape
            xmin = int(self.roi_xmin * W_full)
            xmax = int(self.roi_xmax * W_full)
            ymin = int(self.roi_ymin * H_full)
            ymax = int(self.roi_ymax * H_full)

            # ダウンサンプリング
            step = self.downsample_step
            depth_ds = depth[::step, ::step]
            fx, fy, cx, cy, W0, H0 = self.K
            fx, fy, cx, cy = fx/step, fy/step, cx/step, cy/step
            H, W = depth_ds.shape

            # ROIスケール変換
            xmin //= step; xmax //= step
            ymin //= step; ymax //= step

            # ICP対象のみマスク
            mask = np.zeros_like(depth_ds, dtype=np.uint8)
            mask[ymin:ymax, xmin:xmax] = 1
            depth_masked = np.where(mask, depth_ds, 0)
            depth_masked = np.clip(depth_masked, 0, self.depth_trunc)

            # ---- Depth可視化: 全体表示＋赤枠ROI描画 ----
            depth_vis = np.clip(np.nan_to_num(depth_ds), 0, self.depth_trunc)
            depth_norm = (depth_vis / self.depth_trunc * 255).astype(np.uint8)
            depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
            cv2.rectangle(depth_color, (xmin, ymin), (xmax, ymax), (0, 0, 255), 2)
            self.latest_depth_vis = depth_color

            # ---- Open3D点群生成（ROI内のみ）----
            intr = o3d.camera.PinholeCameraIntrinsic(W, H, fx, fy, cx, cy)
            depth_o3d = o3d.geometry.Image(depth_masked)
            pcd = o3d.geometry.PointCloud.create_from_depth_image(
                depth_o3d, intr, depth_scale=1.0, depth_trunc=self.depth_trunc
            )
            pcd = pcd.voxel_down_sample(self.voxel_size)
            if np.asarray(pcd.points).shape[0] < 10:
                return
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=self.normal_radius, max_nn=self.normal_max_nn
                )
            )

            self.latest_pcd = pcd

            if self.initial_pcd is None:
                self.initial_pcd = pcd
                rospy.loginfo("Initial ICP reference captured (ROI).")
                return

            reg = o3d.pipelines.registration.registration_icp(
                pcd, self.initial_pcd,
                max_correspondence_distance=self.icp_max_corr,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane()
            )
            T = reg.transformation
            dx, dy, dz = T[0, 3], T[1, 3], T[2, 3]

            with self.buffer_lock:
                self.dx_buf.append(dx)
                self.dy_buf.append(dy)
                self.dz_buf.append(dz)

            norm = math.sqrt(dx*dx + dy*dy + dz*dz)
            if norm > self.icp_threshold and not self.is_recalibrating:
                self._start_recalibration(f"ICP {norm:.3f} m")

        except Exception as e:
            rospy.logwarn_throttle(1.0, f"Depth ICP failed: {e}")

    # =========================================================
    def _depth_display_loop(self):
        cv2.namedWindow("Depth", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Depth", 960, 720)
        while not rospy.is_shutdown():
            if self.latest_depth_vis is not None:
                cv2.imshow("Depth", self.latest_depth_vis)
                cv2.waitKey(1)
            time.sleep(0.05)
        cv2.destroyAllWindows()

    # =========================================================
    def _start_recalibration(self, reason):
        self.is_recalibrating = True
        rospy.loginfo(f"[RECALIBRATION TRIGGER] reason={reason}")
        threading.Thread(target=self.trigger_recalibration, daemon=True).start()

    def trigger_recalibration(self):
        try:
            if self.recalibration_client:
                resp = self.recalibration_client()
                rospy.loginfo(f"Recalibration result: {resp.message}")
                if resp.success:
                    if self.filtered_gravity_dir is not None:
                        self.reference_dir = self.filtered_gravity_dir.copy()
                    if self.latest_pcd is not None:
                        self.initial_pcd = self.latest_pcd
                        rospy.loginfo("ICP reference updated after recalibration.")
        except Exception as e:
            rospy.logwarn(f"Failed to call /recalibration/run: {e}")
        finally:
            self.is_recalibrating = False

    # =========================================================
    def update_plot(self, frame):
        with self.buffer_lock:
            dx = np.array(self.dx_buf)
            dy = np.array(self.dy_buf)
            dz = np.array(self.dz_buf)
            tilt = np.array(self.tilt_buf)
        m = max(len(dx), len(dy), len(dz), len(tilt), 1)
        x = np.arange(self.frame_index - m + 1, self.frame_index + 1)

        def pad(a):
            b = np.zeros(m)
            if len(a) > 0:
                b[-len(a):] = a
            return b

        dx, dy, dz, tilt = pad(dx), pad(dy), pad(dz), pad(tilt)
        self.line_dx.set_data(x, dx)
        self.line_dy.set_data(x, dy)
        self.line_dz.set_data(x, dz)
        self.line_tilt.set_data(x, tilt)

        # --- 警告回避: フレームが2以上のときのみ軸を更新 ---
        if self.frame_index > 1:
            xmin = max(0, self.frame_index - self.window_len)
            xmax = self.frame_index
            if xmin == xmax:
                xmax += 1  # ゼロ幅防止
            self.ax_icp.set_xlim(xmin, xmax)
            self.ax_tilt.set_xlim(xmin, xmax)

        self.frame_index += 1
        return [self.line_dx, self.line_dy, self.line_dz, self.line_tilt]

    # =========================================================
    def spin(self):
        threading.Thread(target=rospy.spin, daemon=True).start()
        plt.show()
        cv2.destroyAllWindows()

    def _sigint_handler(self, sig, frame):
        plt.close("all")
        cv2.destroyAllWindows()
        rospy.signal_shutdown("SIGINT")


if __name__ == "__main__":
    ImuIcpMonitor()
