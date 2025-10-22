#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import numpy as np
import time


class ImuPlotter:
    def __init__(self):
        rospy.init_node("imu_plotter", anonymous=True)

        # ===== 閾値を先に定義 =====
        self.gyro_threshold = 0.01  # rad/s
        self.accel_threshold = 0.1  # m/s²

        # ===== 設定 =====
        self.max_len = 1000
        self.update_rate = 30.0  # Hz
        self.last_update = time.time()

        # ===== データバッファ =====
        self.timestamps = deque(maxlen=self.max_len)
        self.accel_x, self.accel_y, self.accel_z = deque(maxlen=self.max_len), deque(maxlen=self.max_len), deque(maxlen=self.max_len)
        self.gyro_x, self.gyro_y, self.gyro_z = deque(maxlen=self.max_len), deque(maxlen=self.max_len), deque(maxlen=self.max_len)

        # ===== グラフ設定 =====
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(9, 6))
        self.fig.suptitle("RealSense IMU  (/camera/imu)", fontsize=14)

        # 加速度
        self.lines_accel = [self.ax1.plot([], [], label="accel_x")[0],
                            self.ax1.plot([], [], label="accel_y")[0],
                            self.ax1.plot([], [], label="accel_z")[0]]
        self.ax1.set_ylim(-10, 10)
        self.ax1.set_ylabel("Acceleration [m/s²]")
        self.ax1.legend(loc="upper right")
        self.ax1.grid(True)

        # 角速度
        self.lines_gyro = [self.ax2.plot([], [], label="gyro_x")[0],
                           self.ax2.plot([], [], label="gyro_y")[0],
                           self.ax2.plot([], [], label="gyro_z")[0]]
        self.ax2.set_ylim(-3, 3)
        self.ax2.set_xlabel("Time [s]")
        self.ax2.set_ylabel("Angular Velocity [rad/s]")
        self.ax2.legend(loc="upper right")
        self.ax2.grid(True)

        # ===== Subscriber は最後に登録 =====
        rospy.Subscriber("/camera/imu", Imu, self.imu_callback)

        # ===== アニメーション設定 =====
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=int(1000 / self.update_rate), cache_frame_data=False
        )
        plt.show()

    def imu_callback(self, msg):
        """IMUトピックからデータ受信"""
        now = time.time()
        if now - self.last_update >= 1.0 / self.update_rate:
            self.last_update = now
            self.timestamps.append(now)
            self.accel_x.append(msg.linear_acceleration.x)
            self.accel_y.append(msg.linear_acceleration.y)
            self.accel_z.append(msg.linear_acceleration.z)
            self.gyro_x.append(msg.angular_velocity.x)
            self.gyro_y.append(msg.angular_velocity.y)
            self.gyro_z.append(msg.angular_velocity.z)
            self.detect_camera_motion(msg)

    def detect_camera_motion(self, msg):
        """固定カメラの動きを検知"""
        gyro = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
        accel = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

        gyro_mean = np.mean(np.abs(gyro))
        accel_mag = np.linalg.norm(accel) - 9.8  # 重力補正

        moved = False
        reason = ""

        if gyro_mean > self.gyro_threshold:
            moved = True
            reason = f"角速度: {gyro_mean:.4f} rad/s"
        elif abs(accel_mag) > self.accel_threshold:
            moved = True
            reason = f"加速度: {accel_mag:.4f} m/s²"

        if moved:
            rospy.logwarn_throttle(1.0, f"[CAMERA MOTION DETECTED] {reason}")

    def update_plot(self, frame):
        """リアルタイムグラフ更新"""
        if not self.timestamps:
            return []
        t0 = self.timestamps[0]
        t = np.array(self.timestamps) - t0
        for line, data in zip(self.lines_accel, [self.accel_x, self.accel_y, self.accel_z]):
            line.set_data(t, data)
        for line, data in zip(self.lines_gyro, [self.gyro_x, self.gyro_y, self.gyro_z]):
            line.set_data(t, data)
        for ax in [self.ax1, self.ax2]:
            ax.set_xlim(max(0, t[-1] - self.max_len / self.update_rate), t[-1])
        return self.lines_accel + self.lines_gyro


if __name__ == "__main__":
    ImuPlotter()
