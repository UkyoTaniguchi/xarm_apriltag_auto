#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU監視ノード + 折れ線グラフ可視化 + ロボット再キャリブレーション呼び出し

機能:
  - /camera/imu を購読し、角速度・加速度を監視
  - 傾き角(Tilt)、角速度、加速度のスパイクを検出
  - 閾値を超えると /recalibration/run (std_srvs/Trigger) サービスを呼び出し
  - MoveIt側ノードが再キャリブレーション動作を実行
  - 再キャリブレーション完了後に現在のIMU姿勢を新たな基準(ref_dir)として更新
  - Matplotlibでリアルタイム折れ線グラフを表示
"""

import rospy
import math, time, threading, numpy as np
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from threading import Lock


class ImuMonitorTriggerWithPlot:
    """IMUの姿勢変化を監視してロボット再キャリブレーションをトリガーするクラス"""

    def __init__(self):
        # =========================
        # ROSノード初期化
        # =========================
        rospy.init_node("imu_monitor_trigger_plot")

        # ---- パラメータ ----
        self.tilt_threshold_deg   = rospy.get_param("~tilt_threshold_deg", 1.0)    # 傾き角の閾値[deg]
        self.omega_threshold      = rospy.get_param("~omega_threshold", 1.0)       # 角速度の閾値[rad/s]
        self.acc_spike_threshold  = rospy.get_param("~acc_spike_threshold", 1.0)   # 加速度スパイク閾値[m/s^2]
        self.alpha                = rospy.get_param("~alpha", 0.1)                 # 平滑化係数(EMA)
        self.update_rate          = rospy.get_param("~update_rate", 30.0)          # 処理周期[Hz]
        self.window_len           = rospy.get_param("~window_len", 500)            # グラフ表示範囲
        self.decimate_after       = rospy.get_param("~decimate_after", 100000)     # メモリ対策: バッファ間引き閾値

        # =========================
        # 内部状態変数
        # =========================
        self.filtered_gravity_dir = None   # EMAで平滑化された重力方向ベクトル
        self.reference_dir        = None   # 基準重力方向ベクトル（再キャリブレーション後に更新）
        self.last_update_time     = time.time()
        self.is_recalibrating     = False
        self.buffer_lock          = Lock()

        # =========================
        # データバッファ
        # =========================
        self.gx_buf, self.gy_buf, self.gz_buf = [], [], []
        self.tilt_buf = []

        # =========================
        # 再キャリブレーションサービス
        # =========================
        rospy.wait_for_service("/recalibration/run")
        self.recalibration_client = rospy.ServiceProxy("/recalibration/run", Trigger)

        # =========================
        # IMU購読開始
        # =========================
        rospy.Subscriber("/camera/camera/imu", Imu, self.imu_callback)
        rospy.loginfo("IMU Monitor + Plot + Recalibration Trigger Ready.")

        # =========================
        # Matplotlib初期設定
        # =========================
        self._init_plot()

        # アニメーション起動
        self.anim = animation.FuncAnimation(
            self.fig, self.update_plot,
            interval=int(1000 / self.update_rate),
            cache_frame_data=False
        )

        self.spin()

    # =========================================================
    # Matplotlib初期化
    # =========================================================
    def _init_plot(self):
        self.fig, (self.ax_gyro, self.ax_tilt) = plt.subplots(2, 1, figsize=(10, 6))
        self.fig.suptitle("IMU Monitor + xArm Trigger", fontsize=14)

        # ---- 角速度グラフ ----
        (self.line_gx,) = self.ax_gyro.plot([], [], label="ωx [rad/s]", color="r")
        (self.line_gy,) = self.ax_gyro.plot([], [], label="ωy [rad/s]", color="g")
        (self.line_gz,) = self.ax_gyro.plot([], [], label="ωz [rad/s]", color="b")
        self.ax_gyro.set_ylim(-3, 3)
        self.ax_gyro.set_ylabel("Angular Velocity [rad/s]")
        self.ax_gyro.legend(loc="upper right")
        self.ax_gyro.grid(True)

        # ---- 傾き角グラフ ----
        (self.line_tilt,) = self.ax_tilt.plot([], [], label="Tilt [deg]", color="orange")
        self.ax_tilt.set_ylim(0, 5)
        self.ax_tilt.set_xlabel("Sample Index")
        self.ax_tilt.set_ylabel("Tilt Angle [deg]")
        self.ax_tilt.legend(loc="upper right")
        self.ax_tilt.grid(True)

    # =========================================================
    # IMUコールバック
    # =========================================================
    def imu_callback(self, msg):
        now = time.time()
        if now - self.last_update_time < 1.0 / self.update_rate:
            return
        self.last_update_time = now

        # ---- センサ値取得 ----
        gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z

        # ---- ベクトル計算 ----
        omega_norm = math.sqrt(gx**2 + gy**2 + gz**2)
        accel_vec = np.array([ax, ay, az], dtype=float)
        accel_norm = float(np.linalg.norm(accel_vec))
        if accel_norm < 1e-6:
            return
        gravity_dir = accel_vec / accel_norm  # 単位ベクトル化

        # ---- EMA平滑化 ----
        if self.filtered_gravity_dir is None:
            self.filtered_gravity_dir = gravity_dir.copy()
        if self.reference_dir is None:
            self.reference_dir = gravity_dir.copy()
        self.filtered_gravity_dir = (
            (1 - self.alpha) * self.filtered_gravity_dir + self.alpha * gravity_dir
        )
        self.filtered_gravity_dir /= np.linalg.norm(self.filtered_gravity_dir)

        # ---- 傾き角θ = acos(a・ref) ----
        tilt_deg = math.degrees(
            math.acos(np.clip(np.dot(self.filtered_gravity_dir, self.reference_dir), -1.0, 1.0))
        )
        acc_spike = abs(accel_norm - 9.81)

        # ---- バッファ更新 ----
        with self.buffer_lock:
            self.gx_buf.append(gx)
            self.gy_buf.append(gy)
            self.gz_buf.append(gz)
            self.tilt_buf.append(tilt_deg)
            if self.decimate_after and len(self.gx_buf) > self.decimate_after:
                self.gx_buf = self.gx_buf[::2]
                self.gy_buf = self.gy_buf[::2]
                self.gz_buf = self.gz_buf[::2]
                self.tilt_buf = self.tilt_buf[::2]
                rospy.loginfo("Data buffers decimated to reduce memory.")

        # ---- 閾値判定 ----
        impulse_detected = (omega_norm > self.omega_threshold) or (acc_spike > self.acc_spike_threshold)
        tilt_exceeded = tilt_deg > self.tilt_threshold_deg

        if (impulse_detected or tilt_exceeded) and not self.is_recalibrating:
            self.is_recalibrating = True
            rospy.loginfo(f"[MOTION DETECTED] Tilt={tilt_deg:.2f}° |ω|={omega_norm:.2f}")
            threading.Thread(target=self.trigger_recalibration, daemon=True).start()

    # =========================================================
    # 再キャリブレーション呼び出し
    # =========================================================
    def trigger_recalibration(self):
        """再キャリブレーションサービスを呼び出し、成功したら基準姿勢を更新"""
        try:
            resp = self.recalibration_client()
            rospy.loginfo(f"Recalibration result: {resp.message}")

            # 再キャリブレーション成功時に基準姿勢を更新
            if resp.success and self.filtered_gravity_dir is not None:
                self.reference_dir = self.filtered_gravity_dir.copy()
                rospy.loginfo("基準姿勢(ref_dir)を現在のIMU重力方向に更新しました。")

        except Exception as e:
            rospy.logwarn(f"Failed to call /recalibration/run: {e}")
        finally:
            self.is_recalibrating = False

    # =========================================================
    # グラフ更新関数（FuncAnimation用）
    # =========================================================
    def update_plot(self, frame):
        with self.buffer_lock:
            n = len(self.tilt_buf)
            if n == 0:
                return []
            x = np.arange(n)
            gx, gy, gz = np.array(self.gx_buf), np.array(self.gy_buf), np.array(self.gz_buf)
            tilt = np.array(self.tilt_buf)

            if self.window_len and n > self.window_len:
                x = x[-self.window_len:]
                gx, gy, gz, tilt = gx[-self.window_len:], gy[-self.window_len:], gz[-self.window_len:], tilt[-self.window_len:]

        # ---- 折れ線更新 ----
        self.line_gx.set_data(x, gx)
        self.line_gy.set_data(x, gy)
        self.line_gz.set_data(x, gz)
        self.line_tilt.set_data(x, tilt)

        # ---- 軸範囲更新 ----
        xmin, xmax = x[0], x[-1]
        self.ax_gyro.set_xlim(xmin, xmax)
        self.ax_tilt.set_xlim(xmin, xmax)
        return [self.line_gx, self.line_gy, self.line_gz, self.line_tilt]

    # =========================================================
    # スピン＋グラフ表示ループ
    # =========================================================
    def spin(self):
        threading.Thread(target=rospy.spin, daemon=True).start()
        plt.show()


if __name__ == "__main__":
    ImuMonitorTriggerWithPlot()
