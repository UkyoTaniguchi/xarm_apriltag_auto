#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU監視ノード + 折れ線グラフ可視化 + ロボット再キャリブレーション呼び出し
＋ 水平方向変位(Δx, Δy)の生データ積分表示（平滑化なし）
"""

import rospy
import math, time, threading, numpy as np
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from threading import Lock


class ImuMonitorTriggerWithPlot:
    def __init__(self):
        rospy.init_node("imu_monitor_trigger_plot")

        # ---- パラメータ ----
        self.tilt_threshold_deg   = rospy.get_param("~tilt_threshold_deg", 1.0)
        self.omega_threshold      = rospy.get_param("~omega_threshold", 1.0)
        self.acc_spike_threshold  = rospy.get_param("~acc_spike_threshold", 1.0)
        self.alpha                = rospy.get_param("~alpha", 0.1)
        self.update_rate          = rospy.get_param("~update_rate", 30.0)
        self.window_len           = rospy.get_param("~window_len", 500)
        self.decimate_after       = rospy.get_param("~decimate_after", 100000)

        # ---- 状態変数 ----
        self.filtered_gravity_dir = None
        self.reference_dir = None
        self.last_update_time = time.time()
        self.is_recalibrating = False
        self.buffer_lock = Lock()

        # ---- データバッファ ----
        self.gx_buf, self.gy_buf, self.gz_buf = [], [], []
        self.tilt_buf = []
        self.dx_buf, self.dy_buf = [], []

        # ---- 位置推定用 ----
        self.vel = np.zeros(3)
        self.pos = np.zeros(3)
        self.last_time = None

        # ---- 再キャリブレーションサービス ----
        rospy.wait_for_service("/recalibration/run")
        self.recalibration_client = rospy.ServiceProxy("/recalibration/run", Trigger)

        # ---- IMU購読 ----
        rospy.Subscriber("/camera/camera/imu", Imu, self.imu_callback)
        rospy.loginfo("IMU Monitor + Plot + Recalibration Trigger Ready.")

        # ---- Matplotlib初期化 ----
        self._init_plot()

        # ---- アニメーション起動 ----
        self.anim = animation.FuncAnimation(
            self.fig, self.update_plot,
            interval=int(1000 / self.update_rate),
            cache_frame_data=False
        )

        self.spin()

    # =========================================================
    def _init_plot(self):
        self.fig, (self.ax_pos, self.ax_gyro, self.ax_tilt) = plt.subplots(3, 1, figsize=(10, 8))
        self.fig.suptitle("IMU Position + Angular Velocity + Tilt", fontsize=14)

        # ---- 上段：水平変位 ----
        (self.line_dx,) = self.ax_pos.plot([], [], label="Δx [m]", color="r")
        (self.line_dy,) = self.ax_pos.plot([], [], label="Δy [m]", color="g")
        self.ax_pos.set_ylabel("Displacement [m]")
        self.ax_pos.set_ylim(-0.5, 0.5)
        self.ax_pos.legend(loc="upper right")
        self.ax_pos.grid(True)

        # ---- 中段：角速度 ----
        (self.line_gx,) = self.ax_gyro.plot([], [], label="ωx [rad/s]", color="r")
        (self.line_gy,) = self.ax_gyro.plot([], [], label="ωy [rad/s]", color="g")
        (self.line_gz,) = self.ax_gyro.plot([], [], label="ωz [rad/s]", color="b")
        self.ax_gyro.set_ylim(-3, 3)
        self.ax_gyro.set_ylabel("Angular Velocity [rad/s]")
        self.ax_gyro.legend(loc="upper right")
        self.ax_gyro.grid(True)

        # ---- 下段：傾き角 ----
        (self.line_tilt,) = self.ax_tilt.plot([], [], label="Tilt [deg]", color="orange")
        self.ax_tilt.set_ylim(0, 5)
        self.ax_tilt.set_xlabel("Sample Index")
        self.ax_tilt.set_ylabel("Tilt Angle [deg]")
        self.ax_tilt.legend(loc="upper right")
        self.ax_tilt.grid(True)

    # =========================================================
    def imu_callback(self, msg):
        now = time.time()
        if self.last_time is None:
            self.last_time = now
            return
        dt = now - self.last_time
        if dt <= 0:
            return
        self.last_time = now

        # ---- IMUセンサ値 ----
        gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        accel = np.array([ax, ay, az], dtype=float)

        # ---- 重力方向を推定 ----
        accel_norm = np.linalg.norm(accel)
        if accel_norm < 1e-6:
            return
        gravity_dir = accel / accel_norm
        if self.filtered_gravity_dir is None:
            self.filtered_gravity_dir = gravity_dir.copy()
        if self.reference_dir is None:
            self.reference_dir = gravity_dir.copy()
        self.filtered_gravity_dir = (1 - self.alpha) * self.filtered_gravity_dir + self.alpha * gravity_dir
        self.filtered_gravity_dir /= np.linalg.norm(self.filtered_gravity_dir)

        # ---- 加速度から重力成分を除去（生データ利用）----
        g_vec = 9.81 * self.filtered_gravity_dir
        accel_nograv = accel - g_vec

        # ---- 生の積分で位置更新 ----
        self.vel += accel_nograv * dt
        self.pos += self.vel * dt
        dx, dy = self.pos[0], self.pos[1]

        # ---- 傾き角計算（元のまま）----
        tilt_deg = math.degrees(
            math.acos(np.clip(np.dot(self.filtered_gravity_dir, self.reference_dir), -1.0, 1.0))
        )
        acc_spike = abs(accel_norm - 9.81)

        # ---- バッファ更新 ----
        with self.buffer_lock:
            self.dx_buf.append(dx)
            self.dy_buf.append(dy)
            self.gx_buf.append(gx)
            self.gy_buf.append(gy)
            self.gz_buf.append(gz)
            self.tilt_buf.append(tilt_deg)
            if self.decimate_after and len(self.gx_buf) > self.decimate_after:
                for b in [self.dx_buf, self.dy_buf, self.gx_buf, self.gy_buf, self.gz_buf, self.tilt_buf]:
                    b[:] = b[::2]
                rospy.loginfo("Data buffers decimated to reduce memory.")

        # ---- 閾値判定（元のまま）----
        omega_norm = math.sqrt(gx**2 + gy**2 + gz**2)
        impulse_detected = (omega_norm > self.omega_threshold) or (acc_spike > self.acc_spike_threshold)
        tilt_exceeded = tilt_deg > self.tilt_threshold_deg
        if (impulse_detected or tilt_exceeded) and not self.is_recalibrating:
            self.is_recalibrating = True
            rospy.loginfo(f"[MOTION DETECTED] Tilt={tilt_deg:.2f}° |ω|={omega_norm:.2f}")
            threading.Thread(target=self.trigger_recalibration, daemon=True).start()

    # =========================================================
    def trigger_recalibration(self):
        try:
            resp = self.recalibration_client()
            rospy.loginfo(f"Recalibration result: {resp.message}")
            if resp.success and self.filtered_gravity_dir is not None:
                self.reference_dir = self.filtered_gravity_dir.copy()
                rospy.loginfo("基準姿勢(ref_dir)を現在のIMU重力方向に更新しました。")
        except Exception as e:
            rospy.logwarn(f"Failed to call /recalibration/run: {e}")
        finally:
            self.is_recalibrating = False

    # =========================================================
    def update_plot(self, frame):
        with self.buffer_lock:
            n = len(self.tilt_buf)
            if n == 0:
                return []
            x = np.arange(n)
            gx, gy, gz = np.array(self.gx_buf), np.array(self.gy_buf), np.array(self.gz_buf)
            tilt = np.array(self.tilt_buf)
            dx, dy = np.array(self.dx_buf), np.array(self.dy_buf)
            if self.window_len and n > self.window_len:
                x = x[-self.window_len:]
                gx, gy, gz, tilt, dx, dy = gx[-self.window_len:], gy[-self.window_len:], gz[-self.window_len:], tilt[-self.window_len:], dx[-self.window_len:], dy[-self.window_len:]

        # ---- 上段 Δx, Δy ----
        self.line_dx.set_data(x, dx)
        self.line_dy.set_data(x, dy)
        # ---- 中段 角速度 ----
        self.line_gx.set_data(x, gx)
        self.line_gy.set_data(x, gy)
        self.line_gz.set_data(x, gz)
        # ---- 下段 傾き ----
        self.line_tilt.set_data(x, tilt)

        xmin, xmax = x[0], x[-1]
        for ax in [self.ax_pos, self.ax_gyro, self.ax_tilt]:
            ax.set_xlim(xmin, xmax)
        return [self.line_dx, self.line_dy, self.line_gx, self.line_gy, self.line_gz, self.line_tilt]

    # =========================================================
    def spin(self):
        threading.Thread(target=rospy.spin, daemon=True).start()
        plt.show()


if __name__ == "__main__":
    ImuMonitorTriggerWithPlot()
