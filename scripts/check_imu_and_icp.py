#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU監視 + ICPズレ検出 + Depth表示 + 折れ線グラフ可視化（ROI可視対応・動的基準更新）
- 上段: ICP平行移動(Δx,Δy,Δz) [m]
- 下段: IMU傾き角(Tilt) [deg]
- Depth表示は全体表示＋赤枠ROI描画（ROI外も可視化）
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
        # ROSノード初期化
        rospy.init_node("imu_icp_monitor")

        # ==========================
        # パラメータ（rosparam 経由で上書き可）
        # ==========================
        # しきい値系
        self.tilt_threshold_deg = rospy.get_param("~tilt_threshold_deg", 1.0)  # 傾き角での発火[deg]
        self.icp_threshold = rospy.get_param("~icp_threshold", 0.05)           # ICPノルムでの発火[m]

        # 平滑化と描画更新
        self.alpha = rospy.get_param("~alpha", 0.1)                             # EMA係数
        self.update_rate = rospy.get_param("~update_rate", 10.0)                # グラフ更新Hz
        self.window_len = int(rospy.get_param("~window_len", 500))              # 表示リングバッファ長

        # サブスクライブするトピック名
        self.depth_topic = rospy.get_param("~depth_topic", "/camera/camera/depth/image_rect_raw")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/camera/camera/depth/camera_info")
        self.imu_topic = rospy.get_param("~imu_topic", "/camera/camera/imu")

        # ROI（0〜1の割合で指定：フル解像度基準）
        # ※ ROI外は可視化に含めるが、ICP計算には含めない
        self.roi_xmin = rospy.get_param("~roi_xmin", 0.25)
        self.roi_xmax = rospy.get_param("~roi_xmax", 0.4)
        self.roi_ymin = rospy.get_param("~roi_ymin", 0.2)
        self.roi_ymax = rospy.get_param("~roi_ymax", 0.5)

        # 軽量化・前処理パラメータ
        self.downsample_step = 4      # 画像間引きステップ（整数）
        self.voxel_size = 0.02        # 点群ボクセルDS [m]
        self.icp_rate = 2.0           # ICP 実行レート [Hz]
        self.icp_max_corr = 0.1       # ICP 対応点の最大距離 [m]
        self.depth_trunc = 1.5        # 深度の上限クリップ [m]
        self.normal_radius = 0.05     # 法線推定の近傍半径 [m]
        self.normal_max_nn = 30       # 法線推定の最大近傍点数

        # ==========================
        # 内部状態（IMU）
        # ==========================
        self.filtered_gravity_dir = None   # EMA後の重力方向ベクトル
        self.reference_dir = None          # 再キャリブ後に更新される参照重力方向
        self.last_update_time = 0.0        # IMUグラフ更新スロットリング
        self.last_icp_time = 0.0           # ICPスロットリング
        self.buffer_lock = Lock()          # バッファ保護
        self.is_recalibrating = False      # 再キャリ中フラグ

        # ==========================
        # 内部状態（ICP/可視化）
        # ==========================
        self.initial_pcd = None       # ICPの基準点群（初回/再キャリ成功時に更新）
        self.latest_pcd = None        # 直近フレームのROI点群
        self.latest_depth_vis = None  # Depth表示用カラー画像（ROI枠込み）

        # グラフ用リングバッファ（固定長）
        N = self.window_len
        self.dx_buf, self.dy_buf, self.dz_buf = deque(maxlen=N), deque(maxlen=N), deque(maxlen=N)
        self.tilt_buf = deque(maxlen=N)

        # フレーム番号（横軸を実時間に従って単調増加させるためのカウンタ）
        self.frame_index = 0

        # カメラ内部パラメータ（CameraInfo受領後にセット）
        self.K = None        # (fx, fy, cx, cy, width, height)
        self.has_camera_info = False

        # ROS <-> OpenCV 変換
        self.bridge = CvBridge()

        # ==========================
        # サービスクライアント（任意）
        # ==========================
        try:
            rospy.wait_for_service("/recalibration/run", timeout=3.0)
            self.recalibration_client = rospy.ServiceProxy("/recalibration/run", Trigger)
        except rospy.ROSException:
            # サービスが存在しない場合は None にしておき、呼び出し時にスキップ
            self.recalibration_client = None
            rospy.logwarn("/recalibration/run が見つからないため、スキップ")

        # ==========================
        # サブスクライバ登録
        # ==========================
        rospy.Subscriber(self.imu_topic, Imu, self.imu_callback, queue_size=50)
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_callback, queue_size=1)
        rospy.Subscriber(self.depth_topic, Image, self.depth_callback, queue_size=1)
        rospy.loginfo("IMU + ICP + Depth Monitor ready.")

        # ==========================
        # Depth表示スレッド（メインスレでimshowし続けるためのプロセス分離）
        # ==========================
        threading.Thread(target=self._depth_display_loop, daemon=True).start()

        # ==========================
        # 折れ線グラフ初期化
        # ==========================
        self._init_plot()
        # Matplotlib アニメーション（一定間隔で update_plot を呼ぶ）
        self.anim = animation.FuncAnimation(
            self.fig, self.update_plot,
            interval=int(1000 / self.update_rate),
            cache_frame_data=False
        )

        # Ctrl+Cで安全終了
        signal.signal(signal.SIGINT, self._sigint_handler)

        # 可視化ループ開始
        self.spin()

    # ---------------------------------------------------------------------
    # CameraInfo 受信: 内部パラメータを保存
    # ---------------------------------------------------------------------
    def camera_info_callback(self, msg: CameraInfo):
        fx = msg.K[0]; fy = msg.K[4]; cx = msg.K[2]; cy = msg.K[5]
        self.K = (fx, fy, cx, cy, msg.width, msg.height)
        self.has_camera_info = True

    # ---------------------------------------------------------------------
    # グラフ初期化: 2段構成（上: ICP Δ, 下: Tilt）
    # 凡例は右外側に配置してプロットと重ならないようにする
    # ---------------------------------------------------------------------
    def _init_plot(self):
        self.fig, (self.ax_icp, self.ax_tilt) = plt.subplots(2, 1, figsize=(10, 6))
        self.fig.suptitle("ICP Δ[m] + IMU Tilt [deg]", fontsize=13)

        # 上段：ICP Δ
        (self.line_dx,) = self.ax_icp.plot([], [], label="Δx [m]")
        (self.line_dy,) = self.ax_icp.plot([], [], label="Δy [m]")
        (self.line_dz,) = self.ax_icp.plot([], [], label="Δz [m]")
        self.ax_icp.set_ylim(-0.2, 0.2)              # 既定の表示レンジ（必要に応じてrosparamで変更推奨）
        self.ax_icp.set_ylabel("ICP Δ[m]")
        self.ax_icp.grid(True)
        # 凡例は右外（描画領域の外）へ
        self.ax_icp.legend(loc="center left", bbox_to_anchor=(1.02, 0.5))

        # 下段：Tilt
        (self.line_tilt,) = self.ax_tilt.plot([], [], label="Tilt [deg]")
        self.ax_tilt.set_ylim(0, 10)
        self.ax_tilt.set_ylabel("Tilt [deg]")
        self.ax_tilt.set_xlabel("Frame Index")
        self.ax_tilt.grid(True)
        # 凡例は右外
        self.ax_tilt.legend(loc="center left", bbox_to_anchor=(1.02, 0.5))

        # 右に凡例分の余白を確保
        self.fig.tight_layout(rect=[0, 0, 0.85, 1])

    # ---------------------------------------------------------------------
    # IMUコールバック: 重力方向をEMAで平滑化して tilt 角を算出
    # tilt が閾値超えなら再キャリブレーションをトリガ
    # ---------------------------------------------------------------------
    def imu_callback(self, msg: Imu):
        now = time.time()
        # 可視化更新レートに合わせたスロットリング
        if now - self.last_update_time < 1.0 / self.update_rate:
            return
        self.last_update_time = now

        # 加速度ベクトル（重力方向推定）
        a = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        n = np.linalg.norm(a)
        if n < 1e-6:
            return
        g = a / n

        # 初期化（初回のみ）
        if self.filtered_gravity_dir is None:
            self.filtered_gravity_dir = g.copy()
        if self.reference_dir is None:
            self.reference_dir = g.copy()

        # EMA で重力方向を更新
        self.filtered_gravity_dir = (1 - self.alpha) * self.filtered_gravity_dir + self.alpha * g
        self.filtered_gravity_dir /= np.linalg.norm(self.filtered_gravity_dir)

        # tilt 角（現在の重力方向 vs 基準重力方向）
        tilt = math.degrees(math.acos(np.clip(np.dot(self.filtered_gravity_dir, self.reference_dir), -1, 1)))

        # バッファへ追記
        with self.buffer_lock:
            self.tilt_buf.append(tilt)

        # しきい値超過で再キャリブレーション開始
        if tilt > self.tilt_threshold_deg and not self.is_recalibrating:
            self._start_recalibration(f"Tilt {tilt:.2f}°")

    # ---------------------------------------------------------------------
    # Depthコールバック:
    #  1) ROIを反映して点群生成
    #  2) 初回は initial_pcd を登録
    #  3) 以降は ICP で Δ を取得してプロット用バッファへ積む
    #  4) ノルムがしきい値超なら再キャリトリガ
    # ---------------------------------------------------------------------
    def depth_callback(self, msg: Image):
        if not self.has_camera_info:
            return

        # ICP 実行レート制御（高頻度で重くならないよう抑制）
        now = time.time()
        if now - self.last_icp_time < 1.0 / self.icp_rate:
            return
        self.last_icp_time = now

        try:
            # ROS Image → NumPy
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            # 16UC1 は [mm] 前提のため [m] に変換。32FC1 は [m] のまま。
            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) / 1000.0
            elif depth.dtype != np.float32:
                # 想定外フォーマットはスキップ
                return

            # ---- ROI をフル解像度座標で算出 ----
            H_full, W_full = depth.shape
            xmin = int(self.roi_xmin * W_full)
            xmax = int(self.roi_xmax * W_full)
            ymin = int(self.roi_ymin * H_full)
            ymax = int(self.roi_ymax * H_full)

            # ---- ダウンサンプリング（可視化と点群生成の両方に同じ step を適用）----
            step = self.downsample_step
            depth_ds = depth[::step, ::step]

            # CameraInfo から内部パラメータを取得し、downsample に応じてスケール
            fx, fy, cx, cy, W0, H0 = self.K
            fx, fy, cx, cy = fx / step, fy / step, cx / step, cy / step
            H, W = depth_ds.shape

            # ROI をダウンサンプル解像度へ射影
            xmin //= step; xmax //= step
            ymin //= step; ymax //= step

            # ---- ICP 用の ROI マスク作成（ROI外は 0 = 無効深度）----
            mask = np.zeros_like(depth_ds, dtype=np.uint8)
            mask[ymin:ymax, xmin:xmax] = 1
            depth_masked = np.where(mask, depth_ds, 0)
            depth_masked = np.clip(depth_masked, 0, self.depth_trunc)

            # ---- Depth 可視化（全体の擬似カラー）＋ ROI 枠描画 ----
            depth_vis = np.clip(np.nan_to_num(depth_ds), 0, self.depth_trunc)
            depth_norm = (depth_vis / self.depth_trunc * 255).astype(np.uint8)
            depth_color = cv2.applyColorMap(depth_norm, cv2.COLORMAP_JET)
            cv2.rectangle(depth_color, (xmin, ymin), (xmax, ymax), (0, 0, 255), 2)  # ROI枠（赤）
            self.latest_depth_vis = depth_color

            # ---- Open3D 点群生成（ROI内のみ有効）----
            intr = o3d.camera.PinholeCameraIntrinsic(W, H, fx, fy, cx, cy)
            depth_o3d = o3d.geometry.Image(depth_masked)
            pcd = o3d.geometry.PointCloud.create_from_depth_image(
                depth_o3d, intr, depth_scale=1.0, depth_trunc=self.depth_trunc
            )
            # ボクセルダウンサンプリングで点数を減らし計算安定化
            pcd = pcd.voxel_down_sample(self.voxel_size)

            # 点が極端に少ない場合はスキップ（ノイズ/ROI外など）
            if np.asarray(pcd.points).shape[0] < 10:
                return

            # 法線推定（Point-to-Plane ICP を使うために必要）
            pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamHybrid(
                    radius=self.normal_radius, max_nn=self.normal_max_nn
                )
            )
            self.latest_pcd = pcd

            # 初回は基準点群として登録して終わり
            if self.initial_pcd is None:
                self.initial_pcd = pcd
                rospy.loginfo("Initial ICP reference captured (ROI).")
                return

            # ---- ICP（現フレーム pcd を initial_pcd に合わせる）----
            reg = o3d.pipelines.registration.registration_icp(
                pcd, self.initial_pcd,
                max_correspondence_distance=self.icp_max_corr,
                # point to plane
                # estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane()
                # point to pointの場合はこちら
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
            )
            T = reg.transformation  # 4x4 同次変換
            dx, dy, dz = T[0, 3], T[1, 3], T[2, 3]

            # 平行移動成分をリングバッファへ
            with self.buffer_lock:
                self.dx_buf.append(dx)
                self.dy_buf.append(dy)
                self.dz_buf.append(dz)

            # ICPノルムで再キャリトリガ判定
            norm = math.sqrt(dx*dx + dy*dy + dz*dz)
            if norm > self.icp_threshold and not self.is_recalibrating:
                self._start_recalibration(f"ICP {norm:.3f} m")

        except Exception as e:
            # 頻出時にスパム化しないよう throttle
            rospy.logwarn_throttle(1.0, f"Depth ICP failed: {e}")

    # ---------------------------------------------------------------------
    # Depth表示スレッド：最新フレームを常時表示
    # cv2.imshow はメインスレッドでの実行が安定だが、ここでは短間隔のループで更新
    # ---------------------------------------------------------------------
    def _depth_display_loop(self):
        win_name = "Depth"
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win_name, 960, 720)

        # 画面幅取得
        try:
            import tkinter as tk
            root = tk.Tk()
            root.withdraw()
            screen_w = root.winfo_screenwidth()
            root.destroy()
        except Exception:
            screen_w = 1920

        target_x = screen_w - 960
        target_y = 0

        move_counter = 0          # ★ 最初の数フレームだけ強制移動
        MOVE_TRIES = 20           # ★ これが重要（10〜30推奨）

        while not rospy.is_shutdown():
            if self.latest_depth_vis is not None:
                cv2.imshow(win_name, self.latest_depth_vis)
                cv2.waitKey(1)

                # ★ ウィンドウマネージャに勝つための強制再配置
                if move_counter < MOVE_TRIES:
                    cv2.moveWindow(win_name, target_x, target_y)
                    move_counter += 1

            time.sleep(0.05)

        cv2.destroyWindow(win_name)


    # ---------------------------------------------------------------------
    # 再キャリブレーション開始（非同期でサービス呼び出し）
    # ---------------------------------------------------------------------
    def _start_recalibration(self, reason):
        self.is_recalibrating = True
        rospy.loginfo(f"[RECALIBRATION TRIGGER] reason={reason}")
        threading.Thread(target=self.trigger_recalibration, daemon=True).start()

    # ---------------------------------------------------------------------
    # 再キャリブレーション本体：
    #  成功したら IMU の参照方向と ICP の基準点群を更新する
    # ---------------------------------------------------------------------
    def trigger_recalibration(self):
        try:
            if self.recalibration_client:
                resp = self.recalibration_client()
                rospy.loginfo(f"Recalibration result: {resp.message}")
                if resp.success:
                    # IMU基準更新
                    if self.filtered_gravity_dir is not None:
                        self.reference_dir = self.filtered_gravity_dir.copy()
                    # ICP基準更新（直近のROI点群を新しい initial_pcd とする）
                    if self.latest_pcd is not None:
                        self.initial_pcd = self.latest_pcd
                        rospy.loginfo("ICP reference updated after recalibration.")
        except Exception as e:
            rospy.logwarn(f"Failed to call /recalibration/run: {e}")
        finally:
            self.is_recalibrating = False

    # ---------------------------------------------------------------------
    # グラフ更新（FuncAnimation から周期呼び出し）
    #  リングバッファ長に合わせて右スクロールし、横軸は frame_index で単調増加
    # ---------------------------------------------------------------------
    def update_plot(self, frame):
        # バッファをスレッドセーフに取り出し
        with self.buffer_lock:
            dx = np.array(self.dx_buf)
            dy = np.array(self.dy_buf)
            dz = np.array(self.dz_buf)
            tilt = np.array(self.tilt_buf)

        # 現在のサンプル数（各系列のうち最大長）
        m = max(len(dx), len(dy), len(dz), len(tilt), 1)

        # 横軸は「現在のフレーム番号を右端」とし、m個ぶん遡って描く
        x = np.arange(self.frame_index - m + 1, self.frame_index + 1)

        # 系列ごとに長さを m に合わせて右詰め（左側をゼロ埋め）
        def pad(a):
            b = np.zeros(m)
            if len(a) > 0:
                b[-len(a):] = a
            return b
        dx, dy, dz, tilt = pad(dx), pad(dy), pad(dz), pad(tilt)

        # データ反映
        self.line_dx.set_data(x, dx)
        self.line_dy.set_data(x, dy)
        self.line_dz.set_data(x, dz)
        self.line_tilt.set_data(x, tilt)

        # 初回のゼロ幅設定による警告回避。2フレーム目以降で xlim 更新。
        if self.frame_index > 1:
            xmin = max(0, self.frame_index - self.window_len)
            xmax = self.frame_index
            if xmin == xmax:
                xmax += 1  # ゼロ幅防止
            self.ax_icp.set_xlim(xmin, xmax)
            self.ax_tilt.set_xlim(xmin, xmax)

        # フレームカウンタを進める（横軸が単調増加する）
        self.frame_index += 1

        # アニメーションが再描画すべき Line オブジェクト群を返す
        return [self.line_dx, self.line_dy, self.line_dz, self.line_tilt]

    # ---------------------------------------------------------------------
    # 可視化ループ開始（Matplotlibのブロッキング表示）
    # ---------------------------------------------------------------------
    def spin(self):
        threading.Thread(target=rospy.spin, daemon=True).start()
        plt.show()
        cv2.destroyAllWindows()

    # ---------------------------------------------------------------------
    # Ctrl+C ハンドラ
    # ---------------------------------------------------------------------
    def _sigint_handler(self, sig, frame):
        plt.close("all")
        cv2.destroyAllWindows()
        rospy.signal_shutdown("SIGINT")


if __name__ == "__main__":
    ImuIcpMonitor()
