#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math, time, threading, numpy as np
from threading import Lock
from sensor_msgs.msg import Imu
import moveit_commander
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
from tf.transformations import quaternion_from_euler

class ImuTriggerRecalibrationWithPlot:
    def __init__(self):
        rospy.init_node("imu_trigger_recalibration_plot")

        # ===== 判定・時定数 =====
        self.tilt_threshold_deg = 1.0       # deg
        self.omega_threshold = 1.0          # rad/s
        self.acc_spike_threshold = 1.0      # m/s^2
        self.alpha = 0.1                    # EMA
        self.hold_time = 0.7
        self.impulse_window = 0.03
        self.cooldown_time = 0.2

        # ===== 状態 =====
        self.a_ema = None
        self.ref_dir = None
        self.alert_active = False
        self.last_alert_time = 0.0
        self.home_pose = None

        # ===== MoveIt =====
        moveit_commander.roscpp_initialize([])
        self.arm = moveit_commander.MoveGroupCommander("xarm6")
        self.arm.set_max_velocity_scaling_factor(1.0)
        self.arm.set_max_acceleration_scaling_factor(1.0)
        self.arm.set_planning_time(1.0)
        rospy.sleep(1.0)
        self.home_pose = self.arm.get_current_pose().pose

        # ===== グラフ用バッファ =====
        self.max_len = 1000
        self.update_rate = 30.0
        self.timestamps = deque(maxlen=self.max_len)
        self.gx_buf, self.gy_buf, self.gz_buf = (
            deque(maxlen=self.max_len), deque(maxlen=self.max_len), deque(maxlen=self.max_len)
        )
        self.tilt_buf = deque(maxlen=self.max_len)  # ← 下段は傾き角を維持
        self.last_update = time.time()
        self.buf_lock = Lock()

        # ===== Subscriber =====
        rospy.Subscriber("/camera/imu", Imu, self.imu_callback)
        rospy.loginfo("IMU monitor + MoveIt ready.")

        # ===== Matplotlib =====
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 6))
        self.fig.suptitle("IMU Monitor + xArm Trigger", fontsize=14)

        # 上段：角速度各軸
        (self.line_gx,) = self.ax1.plot([], [], label="ωx [rad/s]", color="r")
        (self.line_gy,) = self.ax1.plot([], [], label="ωy [rad/s]", color="g")
        (self.line_gz,) = self.ax1.plot([], [], label="ωz [rad/s]", color="b")
        self.ax1.set_ylim(-3, 3)
        self.ax1.set_ylabel("Angular velocity [rad/s]")
        self.ax1.legend(loc="upper right")
        self.ax1.grid(True)

        # 下段：重力方向の変化角
        (self.line_tilt,) = self.ax2.plot([], [], label="Tilt [deg]", color="orange")
        self.ax2.set_ylim(0, 5)
        self.ax2.set_xlabel("Time [s]")
        self.ax2.set_ylabel("Tilt angle [deg]")
        self.ax2.legend(loc="upper right")
        self.ax2.grid(True)

        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=int(1000 / self.update_rate), cache_frame_data=False
        )

        self.spin()

    def imu_callback(self, msg: Imu):
        now = time.time()
        if now - self.last_update < 1.0 / self.update_rate:
            return
        self.last_update = now

        gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z

        omega_norm = math.sqrt(gx**2 + gy**2 + gz**2)
        accel_vec = np.array([ax, ay, az], dtype=float)
        norm_a = np.linalg.norm(accel_vec)
        if norm_a < 1e-6:
            return
        accel_dir = accel_vec / norm_a

        if self.a_ema is None:
            self.a_ema = accel_dir.copy()
        if self.ref_dir is None:
            self.ref_dir = accel_dir.copy()

        self.a_ema = (1 - self.alpha) * self.a_ema + self.alpha * accel_dir
        self.a_ema /= np.linalg.norm(self.a_ema)

        dot = float(np.clip(np.dot(self.a_ema, self.ref_dir), -1.0, 1.0))
        theta_deg = math.degrees(math.acos(dot))
        acc_spike = abs(norm_a - 9.81)

        with self.buf_lock:
            self.timestamps.append(now)
            self.gx_buf.append(gx)
            self.gy_buf.append(gy)
            self.gz_buf.append(gz)
            self.tilt_buf.append(theta_deg)  # ← 下段は傾き角を入れる

        # --- トリガー判定 ---
        impulse_hit = omega_norm > self.omega_threshold or acc_spike > self.acc_spike_threshold
        tilt_hit = theta_deg > self.tilt_threshold_deg
        if (impulse_hit or tilt_hit) and not self.alert_active:
            self.alert_active = True
            rospy.loginfo(f"[CAMERA MOTION] tilt={theta_deg:.2f}° |ω|={omega_norm:.2f} |a|-g={acc_spike:.2f}")
            threading.Thread(target=self.run_recalibration_sequence, daemon=True).start()

    def run_recalibration_sequence(self):
        try:
            rospy.loginfo("開始: 再キャリブレーション動作")

            target_pose = self.arm.get_current_pose().pose
            target_pose.position.x = 0.0
            target_pose.position.y = -0.50
            target_pose.position.z = 0.30
            # 下向き→横向き（ピッチ +90°）
            q = quaternion_from_euler(math.radians(90), 0, 0)
            target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w = q

            self.arm.set_pose_target(target_pose)
            success, plan, _, _ = self.arm.plan()
            if success:
                self.arm.execute(plan, wait=True)
                rospy.loginfo("再キャリブレーション位置へ移動完了。")
            else:
                rospy.loginfo("再キャリブレーションプラン生成失敗。")
                return

            time.sleep(2.0)
            rospy.loginfo("元の位置に戻ります。")
            self.arm.set_pose_target(self.home_pose)
            success, plan, _, _ = self.arm.plan()
            if success:
                self.arm.execute(plan, wait=True)
                rospy.loginfo("復帰動作完了。")
            else:
                rospy.loginfo("復帰プラン生成失敗。")

        finally:
            if self.a_ema is not None:
                self.ref_dir = self.a_ema.copy()
                rospy.loginfo("基準姿勢を更新。")
            self.alert_active = False
            rospy.loginfo("監視を再開。")

    def update_plot(self, frame):
        with self.buf_lock:
            if not self.timestamps:
                return []
            t0 = self.timestamps[0]
            t = np.array(self.timestamps, dtype=float) - t0
            n = min(len(t), len(self.gx_buf), len(self.gy_buf), len(self.gz_buf), len(self.tilt_buf))
            if n == 0:
                return []
            t = t[-n:]
            gx = np.array(self.gx_buf)[-n:]
            gy = np.array(self.gy_buf)[-n:]
            gz = np.array(self.gz_buf)[-n:]
            tilt = np.array(self.tilt_buf)[-n:]

        self.line_gx.set_data(t, gx)
        self.line_gy.set_data(t, gy)
        self.line_gz.set_data(t, gz)
        self.line_tilt.set_data(t, tilt)

        xmax = t[-1]
        xmin = max(0.0, xmax - self.max_len / self.update_rate)
        self.ax1.set_xlim(xmin, xmax)
        self.ax2.set_xlim(xmin, xmax)
        return [self.line_gx, self.line_gy, self.line_gz, self.line_tilt]

    def spin(self):
        threading.Thread(target=rospy.spin, daemon=True).start()
        plt.show()


if __name__ == "__main__":
    ImuTriggerRecalibrationWithPlot()
