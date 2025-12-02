#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Imu
import matplotlib.pyplot as plt
from collections import deque
import time
from tf.transformations import euler_from_quaternion  # ★追加

class ImuDataPlotter:
    def __init__(self):
        rospy.init_node('imu_data_plotter', anonymous=True)

        self.window_size = rospy.get_param('~window_size', 200)
        self.imu_topic = rospy.get_param('~imu_topic', 'camera/imu')

        # 時間 & 各値
        self.time_data = deque(maxlen=self.window_size)

        self.accel_x = deque(maxlen=self.window_size)
        self.accel_y = deque(maxlen=self.window_size)
        self.accel_z = deque(maxlen=self.window_size)

        self.gyro_x = deque(maxlen=self.window_size)
        self.gyro_y = deque(maxlen=self.window_size)
        self.gyro_z = deque(maxlen=self.window_size)

        # ★ RPY 追加
        self.roll = deque(maxlen=self.window_size)
        self.pitch = deque(maxlen=self.window_size)
        self.yaw = deque(maxlen=self.window_size)

        rospy.Subscriber(self.imu_topic, Imu, self.imu_callback)

        # Plot setup
        plt.ion()
        self.fig, (self.ax_acc, self.ax_gyro, self.ax_rpy) = plt.subplots(3, 1, figsize=(10, 12))
        plt.show(block=False)

        self.last_plot_time = time.time()

    def imu_callback(self, msg):
        t = rospy.get_time()
        self.time_data.append(t)

        # --- Accel ---
        self.accel_x.append(msg.linear_acceleration.x)
        self.accel_y.append(msg.linear_acceleration.y)
        self.accel_z.append(msg.linear_acceleration.z)

        # --- Gyro ---
        self.gyro_x.append(msg.angular_velocity.x)
        self.gyro_y.append(msg.angular_velocity.y)
        self.gyro_z.append(msg.angular_velocity.z)

        # --- Quaternion → RPY ---
        q = msg.orientation
        quat = [q.x, q.y, q.z, q.w]
        r, p, y = euler_from_quaternion(quat)

        # rad → deg にして見やすく
        self.roll.append(r * 180.0 / 3.141592)
        self.pitch.append(p * 180.0 / 3.141592)
        self.yaw.append(y * 180.0 / 3.141592)

    def update_plot(self):
        now = time.time()
        if now - self.last_plot_time < 1.0:
            return
        self.last_plot_time = now

        # --- Acceleration plot ---
        self.ax_acc.clear()
        self.ax_acc.plot(self.time_data, self.accel_x, label='Accel X')
        self.ax_acc.plot(self.time_data, self.accel_y, label='Accel Y')
        self.ax_acc.plot(self.time_data, self.accel_z, label='Accel Z')
        self.ax_acc.set_title('IMU Linear Acceleration')
        self.ax_acc.set_xlabel('Time [s]')
        self.ax_acc.set_ylabel('Acceleration [m/s²]')
        self.ax_acc.legend()
        self.ax_acc.grid(True)

        # --- Gyro plot ---
        self.ax_gyro.clear()
        self.ax_gyro.plot(self.time_data, self.gyro_x, label='Gyro X')
        self.ax_gyro.plot(self.time_data, self.gyro_y, label='Gyro Y')
        self.ax_gyro.plot(self.time_data, self.gyro_z, label='Gyro Z')
        self.ax_gyro.set_title('IMU Angular Velocity')
        self.ax_gyro.set_xlabel('Time [s]')
        self.ax_gyro.set_ylabel('Angular Velocity [rad/s]')
        self.ax_gyro.legend()
        self.ax_gyro.grid(True)

        # --- RPY plot ---
        self.ax_rpy.clear()
        self.ax_rpy.plot(self.time_data, self.roll,  label='Roll [deg]')
        self.ax_rpy.plot(self.time_data, self.pitch, label='Pitch [deg]')
        self.ax_rpy.plot(self.time_data, self.yaw,   label='Yaw [deg]')
        self.ax_rpy.set_title('IMU Orientation (RPY)')
        self.ax_rpy.set_xlabel('Time [s]')
        self.ax_rpy.set_ylabel('Angle [deg]')
        self.ax_rpy.legend()
        self.ax_rpy.grid(True)

        self.fig.tight_layout()
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


if __name__ == '__main__':
    plotter = ImuDataPlotter()
    rate = rospy.Rate(200)
    while not rospy.is_shutdown():
        plotter.update_plot()
        rate.sleep()
