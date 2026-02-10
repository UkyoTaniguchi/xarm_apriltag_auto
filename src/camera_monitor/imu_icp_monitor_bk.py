#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU監視 + ICPズレ検出 + Depth表示（ROI対応・動的基準更新）
- ROI(/monitor/roi) 受信後に IMU・ICP を開始
- 傾き or ICPズレがしきい値超過 → /recalibration/run を自動コール
- 再キャリブ直前・直後の平均に加えて，
  「ズレ検知から再キャリブ完了までの平均誤差(DURING)」も summary.txt に出力
"""

import os
import math
import time
import threading
import signal
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt  # 今はグラフ未使用だが将来用に残す
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
    - 異常検知 → 再キャリブレーション実行
    - BEFORE/AFTER/DURING の平均を summary.txt に書き出す
      (DURING: ズレ検知〜再キャリブ完了まで)
    """

    def __init__(self):
        rospy.init_node("imu_icp_monitor")

        # --------------------------------------------------
        # 評価ファイル出力ディレクトリの作成
        # --------------------------------------------------
        base_dir = os.path.expanduser("~/imu_icp_eval")
        ts = time.strftime("%Y%m%d_%H%M%S")
        self.exp_dir = os.path.join(base_dir, ts)
        os.makedirs(self.exp_dir, exist_ok=True)

        # CSV 出力ファイル
        self.csv_path = os.path.join(self.exp_dir, "raw.csv")
        self.csv_fp = open(self.csv_path, "w")
        self.csv_fp.write("time,rmse_mm,centroid_mm,dx,dy,dz,rot_deg,tilt_deg,tag\n")

        # --------------------------------------------------
        # パラメータ
        # --------------------------------------------------
        self.max_tilt_deg = rospy.get_param("~tilt_threshold_deg", 1.0)
        self.max_icp_norm = rospy.get_param("~icp_threshold", 0.05)
        self.gravity_filter_alpha = rospy.get_param("~alpha", 0.1)
        self.max_icp_rot_deg = rospy.get_param("~icp_rot_threshold_deg", 1.0)

        self.imu_rate = rospy.get_param("~update_rate", 30.0)
        self.icp_rate = 10.0

        self.depth_topic = rospy.get_param("~depth_topic", "/cam_2/depth/image_rect_raw")
        self.caminfo_topic = rospy.get_param("~camera_info_topic", "/cam_2/depth/camera_info")
        self.imu_topic = rospy.get_param("~imu_topic", "/cam_2/imu")
        self.roi_topic = rospy.get_param("~roi_topic", "/monitor/roi")

        self.recalib_cooldown_sec = rospy.get_param("~recalib_cooldown_sec", 60.0)

        # ROI
        self.roi_xmin = self.roi_ymin = 0.0
        self.roi_xmax = self.roi_ymax = 1.0
        self.roi_active = False

        # ICP 関連
        self.downsample_step = 2
        self.depth_trunc = 1.5
        self.icp_max_corr = 0.1

        # Depth 可視化
        self.depth_vis = None

        # カメラ内部パラ
        self.K = None
        self.has_camera_info = False

        # IMU 状態
        self.filtered_gravity = None
        self.reference_gravity = None

        # ICP 基準点群
        self.initial_pcd = None
        self.ref_centroid = None

        # 時間管理
        self.frame_idx = 0
        self.last_imu_time = 0.0
        self.last_icp_time = 0.0

        # 再キャリブ状態
        self.recalib_active = False
        self.last_recalib_time = 0.0

        self.buffer_lock = Lock()
        self.bridge = CvBridge()

        buffer_cap = rospy.get_param("~window_len", 500)
        self.dx_hist = deque(maxlen=buffer_cap)
        self.dy_hist = deque(maxlen=buffer_cap)
        self.dz_hist = deque(maxlen=buffer_cap)
        self.tilt_hist = deque(maxlen=buffer_cap)
        self.icp_rot_hist = deque(maxlen=buffer_cap)

        self.camera_name = rospy.get_param("~camera_name", "")

        # --------------------------------------------------
        # BEFORE / DURING / AFTER 評価用バッファ
        # --------------------------------------------------
        self.before_buf = []            # 起動直後〜初回ドリフト検出まで
        self.during_buf = []            # ドリフト検知〜再キャリブ完了まで
        self.after_buf = []             # 再キャリブ後 2 秒間だけ

        self.collect_before = True
        self.collect_during = False
        self.collect_after = False
        self.after_collect_start = 0.0

        self.before_mean = None
        self.during_mean = None
        self.after_mean = None

        # --------------------------------------------------
        # Publisher, Subscriber, Service
        # --------------------------------------------------
        self.pub_icp = rospy.Publisher("/monitor/icp_trans_norm", Float32, queue_size=1)
        self.pub_tilt = rospy.Publisher("/monitor/tilt_deg", Float32, queue_size=1)

        self.recalib_client = rospy.ServiceProxy("/recalibration/run", Trigger)

        rospy.Subscriber(self.imu_topic, Imu, self.on_imu)
        rospy.Subscriber(self.caminfo_topic, CameraInfo, self.on_camera_info)
        rospy.Subscriber(self.depth_topic, Image, self.on_depth)
        rospy.Subscriber(self.roi_topic, Float32MultiArray, self.on_roi)

        # 外部ノード（再キャリブノード）が呼ぶ reset サービス
        self.reset_srv = rospy.Service(
            "/monitor/reset_reference", Trigger, self._reset_reference_srv
        )

        rospy.loginfo("IMU+ICP Monitor initialized.")

        # 深度ビューアスレッド
        threading.Thread(target=self._depth_view_loop, daemon=True).start()

        self._init_plot()  # 今は何もしない

        # Ctrl+C対応
        signal.signal(signal.SIGINT, self._sigint_handler)

        self.run()

    # ============================================================
    # /monitor/reset_reference: 外部からの基準リセット
    #  → ICP/IMUの基準だけリセット（評価バッファは触らない）
    # ============================================================
    def _reset_reference_srv(self, _req):
        self.initial_pcd = None
        self.ref_centroid = None
        self.filtered_gravity = None
        self.reference_gravity = None

        # 履歴もクリアしておく（グラフ・ログ用）
        with self.buffer_lock:
            self.dx_hist.clear()
            self.dy_hist.clear()
            self.dz_hist.clear()
            self.tilt_hist.clear()
            self.icp_rot_hist.clear()

        rospy.loginfo("IMU/ICP reference reset by /monitor/reset_reference.")
        return TriggerResponse(success=True, message="Reference reset completed.")

    # ============================================================
    # 点群評価 (RMSE + centroid 誤差)
    # ============================================================
    def compute_pcd_metrics(self, pcd):
        if self.initial_pcd is None:
            return None, None

        pts0 = np.asarray(self.initial_pcd.points)
        pts1 = np.asarray(pcd.points)
        if len(pts0) == 0 or len(pts1) == 0:
            return None, None

        # KDTree 最近傍 RMSE
        tree = o3d.geometry.KDTreeFlann(self.initial_pcd)
        dists = []
        for p in pts1:
            _, idx, _ = tree.search_knn_vector_3d(p, 1)
            q = pts0[idx[0]]
            dists.append(np.linalg.norm(p - q))
        rmse = float(np.sqrt(np.mean(np.square(dists))))

        # 重心誤差
        if self.ref_centroid is None:
            return rmse, None
        c1 = pts1.mean(axis=0)
        centroid_err = float(np.linalg.norm(c1 - self.ref_centroid))

        return rmse, centroid_err

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
        if not self.roi_active:
            return

        now = time.time()
        if now - self.last_imu_time < 1.0 / self.imu_rate:
            return
        self.last_imu_time = now

        acc = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        norm = np.linalg.norm(acc)
        if norm < 1e-6:
            return
        gravity = acc / norm

        if self.filtered_gravity is None:
            self.filtered_gravity = gravity.copy()
        if self.reference_gravity is None:
            self.reference_gravity = gravity.copy()

        a = self.gravity_filter_alpha
        self.filtered_gravity = (1 - a) * self.filtered_gravity + a * gravity
        self.filtered_gravity /= np.linalg.norm(self.filtered_gravity)

        dot = np.clip(np.dot(self.filtered_gravity, self.reference_gravity), -1.0, 1.0)
        tilt_deg = math.degrees(math.acos(dot))

        self.pub_tilt.publish(Float32(tilt_deg))

        with self.buffer_lock:
            self.tilt_hist.append(tilt_deg)

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

            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) / 1000.0

            H_full, W_full = depth.shape

            # ROI
            xmin = int(self.roi_xmin * W_full)
            xmax = int(self.roi_xmax * W_full)
            ymin = int(self.roi_ymin * H_full)
            ymax = int(self.roi_ymax * H_full)

            # ダウンサンプリング
            step = self.downsample_step
            depth_ds = depth[::step, ::step]

            fx, fy, cx, cy, W0, H0 = self.K
            fx /= step
            fy /= step
            cx /= step
            cy /= step
            H, W = depth_ds.shape

            xmin //= step
            xmax //= step
            ymin //= step
            ymax //= step

            mask = np.zeros_like(depth_ds, dtype=np.uint8)
            mask[ymin:ymax, xmin:xmax] = 1
            depth_masked = np.where(mask, depth_ds, 0)
            depth_masked = np.clip(depth_masked, 0, self.depth_trunc)

            # Depth 表示
            depth_vis = np.clip(np.nan_to_num(depth_ds), 0, self.depth_trunc)
            depth_norm = (depth_vis / self.depth_trunc * 255).astype(np.uint8)
            depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
            cv2.rectangle(depth_color, (xmin, ymin), (xmax, ymax), (0, 0, 255), 2)
            self.depth_vis = depth_color

            # PointCloud 生成
            intr = o3d.camera.PinholeCameraIntrinsic(W, H, fx, fy, cx, cy)
            depth_o3d = o3d.geometry.Image(depth_masked)

            pcd = o3d.geometry.PointCloud.create_from_depth_image(
                depth_o3d, intr, depth_scale=1.0, depth_trunc=self.depth_trunc
            )

            if len(pcd.points) < 3:
                return

            pcd.estimate_normals(
                o3d.geometry.KDTreeSearchParamHybrid(radius=0.05, max_nn=30)
            )

            # 初期点群（基準）の保存
            if self.initial_pcd is None:
                self.initial_pcd = pcd
                pts = np.asarray(pcd.points)
                if len(pts) > 0:
                    self.ref_centroid = pts.mean(axis=0)
                rospy.loginfo("Initial ICP reference saved.")
                return

            # ICP
            reg = o3d.pipelines.registration.registration_icp(
                pcd,
                self.initial_pcd,
                self.icp_max_corr,
                np.eye(4),
                o3d.pipelines.registration.TransformationEstimationPointToPoint(),
                o3d.pipelines.registration.ICPConvergenceCriteria(
                    relative_fitness=1e-6,
                    relative_rmse=1e-6,
                    max_iteration=30,
                ),
            )

            T = reg.transformation
            dx, dy, dz = T[0, 3], T[1, 3], T[2, 3]

            Rm = T[:3, :3]
            trace = np.trace(Rm)
            val = np.clip((trace - 1.0) / 2.0, -1.0, 1.0)
            theta_deg = math.degrees(math.acos(val))

            rmse, centroid_err = self.compute_pcd_metrics(pcd)
            rmse_mm = rmse * 1000.0 if rmse is not None else 0.0
            cent_mm = centroid_err * 1000.0 if centroid_err is not None else 0.0

            tilt_latest = self.tilt_hist[-1] if len(self.tilt_hist) > 0 else 0.0

            # CSV ログ（NORMAL）
            self.csv_fp.write(
                f"{time.time()},{rmse_mm:.2f},{cent_mm:.2f},"
                f"{dx},{dy},{dz},{theta_deg},{tilt_latest},NORMAL\n"
            )
            self.csv_fp.flush()

            # 履歴
            with self.buffer_lock:
                self.dx_hist.append(dx)
                self.dy_hist.append(dy)
                self.dz_hist.append(dz)
                self.icp_rot_hist.append(theta_deg)

            # 評価用レコード
            item = (rmse_mm, cent_mm, dx, dy, dz, theta_deg, tilt_latest)

            # BEFORE 収集中
            if self.collect_before:
                self.before_buf.append(item)

            # DURING 収集中（ズレ検知〜再キャリブが完了するまで）
            if self.collect_during:
                self.during_buf.append(item)

            # AFTER 収集中（再キャリブ後 2 秒間）
            if self.collect_after:
                if time.time() - self.after_collect_start < 2.0:
                    self.after_buf.append(item)
                else:
                    self.collect_after = False
                    self._output_after_mean()

            icp_norm = math.sqrt(dx * dx + dy * dy + dz * dz)
            self.pub_icp.publish(Float32(icp_norm))

            self._check_recalib_trigger(tilt_latest, icp_norm, theta_deg)

        except Exception as e:
            rospy.logwarn_throttle(1.0, f"ICP failed: {e}")

    # ============================================================
    # BEFORE 平均出力
    # ============================================================
    def _output_before_mean(self):
        if len(self.before_buf) == 0:
            return

        arr = np.array(self.before_buf)
        mean = arr.mean(axis=0)
        rmse, cent, dx, dy, dz, rot, tilt = mean

        self.csv_fp.write(
            f"{time.time()},{rmse:.2f},{cent:.2f},{dx},{dy},{dz},{rot},{tilt},BEFORE_RECALIB_MEAN\n"
        )
        self.csv_fp.flush()

        self.before_mean = {
            "rmse": rmse,
            "centroid": cent,
            "dx": dx,
            "dy": dy,
            "dz": dz,
            "rot": rot,
            "tilt": tilt,
        }

        self.before_buf.clear()

    # ============================================================
    # DURING 平均出力（ズレ検知〜再キャリブ完了まで）
    # ============================================================
    def _output_during_mean(self):
        if len(self.during_buf) == 0:
            return

        arr = np.array(self.during_buf)
        mean = arr.mean(axis=0)
        rmse, cent, dx, dy, dz, rot, tilt = mean

        self.csv_fp.write(
            f"{time.time()},{rmse:.2f},{cent:.2f},{dx},{dy},{dz},{rot},{tilt},DURING_RECALIB_MEAN\n"
        )
        self.csv_fp.flush()

        self.during_mean = {
            "rmse": rmse,
            "centroid": cent,
            "dx": dx,
            "dy": dy,
            "dz": dz,
            "rot": rot,
            "tilt": tilt,
        }

        self.during_buf.clear()

    # ============================================================
    # AFTER 平均出力
    # ============================================================
    def _output_after_mean(self):
        if len(self.after_buf) == 0:
            return

        arr = np.array(self.after_buf)
        mean = arr.mean(axis=0)
        rmse, cent, dx, dy, dz, rot, tilt = mean

        self.csv_fp.write(
            f"{time.time()},{rmse:.2f},{cent:.2f},{dx},{dy},{dz},{rot},{tilt},AFTER_RECALIB_MEAN\n"
        )
        self.csv_fp.flush()

        self.after_mean = {
            "rmse": rmse,
            "centroid": cent,
            "dx": dx,
            "dy": dy,
            "dz": dz,
            "rot": rot,
            "tilt": tilt,
        }

        self.after_buf.clear()

        # summary.txt を作成
        if self.before_mean is not None:
            self._write_summary_txt()

    # ============================================================
    # summary.txt 作成
    # ============================================================
    def _write_summary_txt(self):
        def safe_improve(b, a):
            if abs(b) < 1e-6:
                return 0.0
            return (1.0 - a / b) * 100.0

        b = self.before_mean
        a = self.after_mean
        d = self.during_mean

        path = os.path.join(self.exp_dir, "summary.txt")
        with open(path, "w") as f:
            f.write("=== IMU+ICP Recalibration Evaluation Summary ===\n")
            f.write(f"Experiment Time: {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")

            f.write("-- BEFORE Recalibration Mean --\n")
            f.write(f"RMSE: {b['rmse']:.2f} mm\n")
            f.write(f"Centroid Error: {b['centroid']:.2f} mm\n")
            f.write(f"ICP Δ: dx={b['dx']:.3f}, dy={b['dy']:.3f}, dz={b['dz']:.3f}\n")
            f.write(f"ICP Rotation: {b['rot']:.3f} deg\n")
            f.write(f"Tilt: {b['tilt']:.3f} deg\n\n")

            if d is not None:
                f.write("-- DURING (Detection→Recalibration) Mean --\n")
                f.write(f"RMSE: {d['rmse']:.2f} mm\n")
                f.write(f"Centroid Error: {d['centroid']:.2f} mm\n")
                f.write(f"ICP Δ: dx={d['dx']:.3f}, dy={d['dy']:.3f}, dz={d['dz']:.3f}\n")
                f.write(f"ICP Rotation: {d['rot']:.3f} deg\n")
                f.write(f"Tilt: {d['tilt']:.3f} deg\n\n")

            f.write("-- AFTER Recalibration Mean --\n")
            f.write(f"RMSE: {a['rmse']:.2f} mm\n")
            f.write(f"Centroid Error: {a['centroid']:.2f} mm\n")
            f.write(f"ICP Δ: dx={a['dx']:.3f}, dy={a['dy']:.3f}, dz={a['dz']:.3f}\n")
            f.write(f"ICP Rotation: {a['rot']:.3f} deg\n")
            f.write(f"Tilt: {a['tilt']:.3f} deg\n\n")

            f.write("-- Improvement (BEFORE→AFTER) --\n")
            f.write(
                f"RMSE improved by {safe_improve(b['rmse'], a['rmse']):.1f} %\n"
            )
            f.write(
                f"Centroid improved by {safe_improve(b['centroid'], a['centroid']):.1f} %\n"
            )
            f.write(
                f"Tilt improved by {safe_improve(b['tilt'], a['tilt']):.1f} %\n"
            )

            if d is not None:
                f.write("\n-- Improvement (DURING→AFTER) --\n")
                f.write(
                    f"RMSE improved by {safe_improve(d['rmse'], a['rmse']):.1f} %\n"
                )
                f.write(
                    f"Centroid improved by {safe_improve(d['centroid'], a['centroid']):.1f} %\n"
                )
                f.write(
                    f"Tilt improved by {safe_improve(d['tilt'], a['tilt']):.1f} %\n"
                )

        rospy.loginfo(f"summary.txt created at: {path}")

    # ============================================================
    # 再キャリブレーショントリガ判定
    # ============================================================
    def _check_recalib_trigger(self, tilt, icp, icp_rot):
        now = time.time()

        if now - self.last_recalib_time < self.recalib_cooldown_sec:
            return
        if self.recalib_active:
            return

        tilt_flag = tilt > self.max_tilt_deg
        icp_flag = icp > self.max_icp_norm
        rot_flag = icp_rot > self.max_icp_rot_deg

        if not (tilt_flag or icp_flag or rot_flag):
            return

        rospy.logwarn("=== Recalibration Triggered ===")
        rospy.logwarn(f" tilt={tilt}, icp={icp}, icp_rot={icp_rot}")

        # BEFORE の平均確定
        self.collect_before = False
        self._output_before_mean()

        # DURING の記録開始（このタイミング以降のフレーム）
        self.collect_during = True
        self.during_buf.clear()

        # BEFORE タグ
        self.csv_fp.write(f"{time.time()},0,0,0,0,0,0,0,BEFORE_RECALIB\n")
        self.csv_fp.flush()

        self.recalib_active = True
        self.last_recalib_time = now

        threading.Thread(target=self._call_recalib_service, daemon=True).start()

    # ============================================================
    # 再キャリブサービス呼び出し
    # ============================================================
    def _call_recalib_service(self):
        try:
            if self.camera_name:
                rospy.set_param("/recalibration/target_camera", self.camera_name)

            res = self.recalib_client()
            if res.success:
                rospy.loginfo("Recalibration OK.")
                self.csv_fp.write(
                    f"{time.time()},0,0,0,0,0,0,0,AFTER_RECALIB\n"
                )
                self.csv_fp.flush()

                # DURING の平均を確定
                self.collect_during = False
                self._output_during_mean()

                # ★ 再キャリブ後は基準点群を取り直す
                self.initial_pcd = None
                self.ref_centroid = None

                # AFTER の記録開始
                self.collect_after = True
                self.after_collect_start = time.time()
            else:
                rospy.logerr("Recalibration FAILED.")
        finally:
            self.recalib_active = False

    # ============================================================
    def _init_plot(self):
        # 今はグラフを使っていないので何もしない
        pass

    # ============================================================
    def _depth_view_loop(self):
        cv2.namedWindow("Depth", cv2.WINDOW_NORMAL)
        while not rospy.is_shutdown():
            if self.depth_vis is not None:
                cv2.imshow("Depth", self.depth_vis)
                cv2.waitKey(1)
            time.sleep(0.03)
        cv2.destroyAllWindows()

    # ============================================================
    def run(self):
        rospy.spin()

    # ============================================================
    def _sigint_handler(self, *_):
        try:
            self.csv_fp.close()
        except Exception:
            pass
        rospy.signal_shutdown("SIGINT")


if __name__ == "__main__":
    ImuIcpMonitor()
