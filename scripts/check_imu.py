#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np
import time
import threading
import math


class ImuMonitor:
    def __init__(self):
        rospy.init_node("imu_monitor", anonymous=True)

        # ===== Parameters =====
        # thresholds
        self.tilt_threshold_deg = 3.0       # deg: 持続傾き判定
        self.omega_threshold = 1.0          # rad/s: 瞬間回転
        self.acc_spike_threshold = 2.0      # m/s^2: |a|-g 逸脱
        # timing
        self.hold_time = 0.7                # s: 傾き継続必要時間
        self.impulse_window = 0.03          # s: 衝撃の最小持続
        self.cooldown_time = 0.2            # s: アラート解除までの安定時間
        # smoothing
        self.alpha = 0.1                    # EMA 係数(重力方向の平滑化)

        # plot/system
        self.max_len = 1000
        self.update_rate = 30.0
        self.last_update = time.time()

        # ===== Buffers =====
        self.timestamps = deque(maxlen=self.max_len)
        self.omega_norm_buf = deque(maxlen=self.max_len)
        self.acc_spike_buf = deque(maxlen=self.max_len)
        self.tilt_deg_buf = deque(maxlen=self.max_len)

        # ===== States =====
        self.prev_accel_dir = None
        self.a_ema = None                   # 平滑化重力方向(単位ベクトル)
        self.ref_dir = None                 # 基準姿勢(単位ベクトル)
        self.tilt_exceed_since = None
        self.impulse_exceed_since = None

        self.alert_active = False
        self.last_alert_time = 0.0

        # ===== Subscriber =====
        rospy.Subscriber("/camera/imu", Imu, self.imu_callback)
        rospy.loginfo_throttle(1.0, f"acc=({msg.linear_acceleration.x:.2f},{msg.linear_acceleration.y:.2f},{msg.linear_acceleration.z:.2f})  "
                         f"gyro=({msg.angular_velocity.x:.2f},{msg.angular_velocity.y:.2f},{msg.angular_velocity.z:.2f})")


        # ===== Plot =====
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 6))
        self.fig.suptitle("IMU Monitor", fontsize=14)

        (self.line_omega,) = self.ax1.plot([], [], label="|ω| [rad/s]")
        # (self.line_accspike,) = self.ax1.plot([], [], label="|a|-g [m/s²]")
        self.ax1.set_ylim(0, 2.0)
        self.ax1.set_ylabel("Angular velocity magnitude [rad/s]")
        self.ax1.legend(loc="upper right")
        self.ax1.grid(True)

        (self.line_tilt,) = self.ax2.plot([], [], label="Gravity direction change [deg]", color="orange")
        self.ax2.set_ylim(0, 5)
        self.ax2.set_xlabel("Time [s]")
        self.ax2.set_ylabel("Gravity Direction Change [deg]")
        self.ax2.legend(loc="upper right")
        self.ax2.grid(True)

        # ===== Start animation =====
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=int(1000 / self.update_rate), cache_frame_data=False
        )

    def imu_callback(self, msg):
        now = time.time()
        if now - self.last_update < 1.0 / self.update_rate:
            return
        self.last_update = now
        self.timestamps.append(now)

        # ---- Gyro ----
        gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        omega_norm = math.sqrt(gx * gx + gy * gy + gz * gz)
        self.omega_norm_buf.append(omega_norm)

        # ---- Accel ----
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        accel_vec = np.array([ax, ay, az], dtype=float)
        norm_a = float(np.linalg.norm(accel_vec))
        if norm_a < 1e-6:
            # invalid sample
            self.acc_spike_buf.append(0.0)
            self.tilt_deg_buf.append(0.0)
            return

        acc_spike = abs(norm_a - 9.81)
        self.acc_spike_buf.append(acc_spike)
        accel_dir = accel_vec / norm_a

        # 初期化
        if self.a_ema is None:
            self.a_ema = accel_dir.copy()
        if self.ref_dir is None:
            self.ref_dir = accel_dir.copy()

        # 平滑化（EMA）
        self.a_ema = (1.0 - self.alpha) * self.a_ema + self.alpha * accel_dir
        self.a_ema = self.a_ema / np.linalg.norm(self.a_ema)

        # 基準との角度
        dot = float(np.clip(np.dot(self.a_ema, self.ref_dir), -1.0, 1.0))
        theta_deg = math.degrees(math.acos(dot))
        self.tilt_deg_buf.append(theta_deg)

        # ---- Triggers ----
        if omega_norm > self.omega_threshold or acc_spike > self.acc_spike_threshold:
            if self.impulse_exceed_since is None:
                self.impulse_exceed_since = now
        else:
            self.impulse_exceed_since = None
        impulse_hit = (self.impulse_exceed_since is not None) and ((now - self.impulse_exceed_since) >= self.impulse_window)

        if theta_deg > self.tilt_threshold_deg:
            if self.tilt_exceed_since is None:
                self.tilt_exceed_since = now
        else:
            self.tilt_exceed_since = None
        tilt_hit = (self.tilt_exceed_since is not None) and ((now - self.tilt_exceed_since) >= self.hold_time)

        # 総合判定
        if (impulse_hit or tilt_hit) and not self.alert_active:
            self.alert_active = True
            self.last_alert_time = now
            rospy.loginfo(
                f"[CAMERA MOTION] impulse={bool(impulse_hit)} tilt={bool(tilt_hit)}  "
                f"Δθ={theta_deg:.2f}°  |ω|={omega_norm:.2f}  |a|-g={acc_spike:.2f}"
            )
            threading.Thread(target=self.say_alert, daemon=True).start()

        # 解除ヒステリシス
        if self.alert_active:
            stable = (
                theta_deg < 0.7 * self.tilt_threshold_deg and
                omega_norm < 0.7 * self.omega_threshold and
                acc_spike < 0.7 * self.acc_spike_threshold
            )
            if stable and (now - self.last_alert_time) > self.cooldown_time:
                self.alert_active = False

        self.prev_accel_dir = accel_dir

    def say_alert(self):
        print("固定カメラの動きを検出。再キャリブレーションを評価してください。")

    def update_plot(self, frame):
        if not self.timestamps:
            return []
        t0 = self.timestamps[0]
        t = np.array(self.timestamps) - t0

        self.line_omega.set_data(t, self.omega_norm_buf)
        # self.line_accspike.set_data(t, self.acc_spike_buf)
        self.line_tilt.set_data(t, self.tilt_deg_buf)

        xmax = t[-1]
        xmin = max(0.0, xmax - self.max_len / self.update_rate)
        self.ax1.set_xlim(xmin, xmax)
        self.ax2.set_xlim(xmin, xmax)

        return [self.line_omega, self.line_tilt]

    def spin(self):
        """rospy.spin()相当: Matplotlibループと両立"""
        spin_thread = threading.Thread(target=rospy.spin, daemon=True)
        spin_thread.start()
        plt.show()


if __name__ == "__main__":
    node = ImuMonitor()
    node.spin()
