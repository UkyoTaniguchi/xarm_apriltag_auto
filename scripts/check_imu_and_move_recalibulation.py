#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import math, time, threading, numpy as np
from threading import Lock
from sensor_msgs.msg import Imu
import moveit_commander
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from tf.transformations import quaternion_from_euler


class ImuTriggerRecalibrationWithPlot:
    """
    IMUの姿勢変化を監視し、しきい値超過でxArm6を再キャリブレーション位置へ移動→復帰。
    上段: 角速度(ωx, ωy, ωz)。下段: 重力方向の変化角(Tilt[deg])。
    横軸はサンプル番号。バッファは蓄積型。必要に応じて間引き。
    """

    def __init__(self):
        rospy.init_node("imu_trigger_recalibration_plot")

        # ---- 判定・描画設定 ----
        self.tilt_threshold_deg   = rospy.get_param("~tilt_threshold_deg", 1.0)   # [deg]
        self.omega_threshold      = rospy.get_param("~omega_threshold", 1.0)      # [rad/s]
        self.acc_spike_threshold  = rospy.get_param("~acc_spike_threshold", 1.0)  # [m/s^2]
        self.alpha                = rospy.get_param("~alpha", 0.1)                # EMA係数
        self.update_rate          = rospy.get_param("~update_rate", 30.0)         # [Hz]
        # 表示窓。None or 負値: 全履歴。正値: そのサンプル数のみ表示。
        self.window_len           = rospy.get_param("~window_len", None)
        if isinstance(self.window_len, (int, float)) and self.window_len is not None:
            self.window_len = int(self.window_len)
            if self.window_len <= 0:
                self.window_len = None

        # 長時間対策: 総サンプルが閾値超過で1/2間引き（Noneで無効）
        self.decimate_after       = rospy.get_param("~decimate_after", 100000)

        # ---- 状態変数 ----
        self.a_ema = None                 # 平滑化重力方向(単位ベクトル)
        self.ref_dir = None               # 基準重力方向(単位ベクトル)
        self.alert_active = False         # 動作中フラグ
        self.home_pose = None             # 復帰用ポーズ
        self.last_update = time.time()
        self.buf_lock = Lock()

        # ---- MoveIt 初期化 ----
        moveit_commander.roscpp_initialize([])
        self.arm = moveit_commander.MoveGroupCommander("xarm6")
        self.arm.set_max_velocity_scaling_factor(1.0)
        self.arm.set_max_acceleration_scaling_factor(1.0)
        self.arm.set_planning_time(1.0)
        rospy.sleep(1.0)
        self.home_pose = self.arm.get_current_pose().pose

        # ---- バッファ（蓄積型リスト）----
        self.gx_buf, self.gy_buf, self.gz_buf = [], [], []
        self.tilt_buf = []

        # ---- 購読とログ ----
        rospy.Subscriber("/camera/imu", Imu, self.imu_callback)
        rospy.loginfo("IMU monitor + MoveIt + Plot ready.")

        # ---- Matplotlib ----
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 6))
        self.fig.suptitle("IMU Monitor + xArm Trigger", fontsize=14)

        # 上段: 角速度3軸
        (self.line_gx,) = self.ax1.plot([], [], label="ωx [rad/s]", color="r")
        (self.line_gy,) = self.ax1.plot([], [], label="ωy [rad/s]", color="g")
        (self.line_gz,) = self.ax1.plot([], [], label="ωz [rad/s]", color="b")
        self.ax1.set_ylim(-3, 3)
        self.ax1.set_ylabel("Angular velocity [rad/s]")
        self.ax1.legend(loc="upper right")
        self.ax1.grid(True)

        # 下段: 傾き角
        (self.line_tilt,) = self.ax2.plot([], [], label="Tilt [deg]", color="orange")
        self.ax2.set_ylim(0, 5)
        self.ax2.set_xlabel("Sample index")
        self.ax2.set_ylabel("Tilt angle [deg]")
        self.ax2.legend(loc="upper right")
        self.ax2.grid(True)

        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=int(1000 / self.update_rate), cache_frame_data=False
        )
        self.spin()

    # -------------------- XYZ+RPY指定 --------------------
    def move_to_xyzrpy(self, x, y, z, rx, ry, rz):
        pose = self.arm.get_current_pose().pose
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        qx, qy, qz, qw = quaternion_from_euler(rx, ry, rz)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw
        self.arm.set_pose_target(pose)
        ok, plan, _, _ = self.arm.plan()
        if ok:
            self.arm.execute(plan, wait=True)
            return True
        return False

    # -------------------- IMU --------------------
    def imu_callback(self, msg: Imu):
        # 更新レート制限
        now = time.time()
        if now - self.last_update < 1.0 / self.update_rate:
            return
        self.last_update = now

        # 角速度・加速度
        gx, gy, gz = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z

        omega_norm = math.sqrt(gx**2 + gy**2 + gz**2)
        a_vec = np.array([ax, ay, az], dtype=float)
        a_norm = float(np.linalg.norm(a_vec))
        if a_norm < 1e-6:
            return
        a_dir = a_vec / a_norm  # 重力方向（単位ベクトル）

        # EMAで重力方向を平滑化
        if self.a_ema is None:
            self.a_ema = a_dir.copy()
        if self.ref_dir is None:
            self.ref_dir = a_dir.copy()
        self.a_ema = (1.0 - self.alpha) * self.a_ema + self.alpha * a_dir
        self.a_ema /= np.linalg.norm(self.a_ema)

        # 基準との角度 θ = acos(a·ref)
        dot = float(np.clip(np.dot(self.a_ema, self.ref_dir), -1.0, 1.0))
        theta_deg = math.degrees(math.acos(dot))
        acc_spike = abs(a_norm - 9.81)

        # バッファ追加（蓄積）
        with self.buf_lock:
            self.gx_buf.append(gx)
            self.gy_buf.append(gy)
            self.gz_buf.append(gz)
            self.tilt_buf.append(theta_deg)

            # 長時間対策の間引き
            if self.decimate_after and len(self.gx_buf) > self.decimate_after:
                self.gx_buf = self.gx_buf[::2]
                self.gy_buf = self.gy_buf[::2]
                self.gz_buf = self.gz_buf[::2]
                self.tilt_buf = self.tilt_buf[::2]
                rospy.loginfo("Buffers decimated by 2 to limit memory.")

        # トリガー判定
        impulse_hit = (omega_norm > self.omega_threshold) or (acc_spike > self.acc_spike_threshold)
        tilt_hit = theta_deg > self.tilt_threshold_deg
        if (impulse_hit or tilt_hit) and not self.alert_active:
            self.alert_active = True
            rospy.loginfo(f"[MOTION DETECTED] tilt={theta_deg:.2f}° |ω|={omega_norm:.2f} |a|-g={acc_spike:.2f}")
            threading.Thread(target=self.run_recalibration_sequence, daemon=True).start()

    # -------------------- xArm sequence --------------------
    def run_recalibration_sequence(self):
        try:
            rospy.loginfo("開始: 再キャリブレーション動作")

            # 例: (x,y,z,rx,ry,rz) = (0.0, -0.50, 0.30, 90°, 0°, 0°)
            ok = self.move_to_xyzrpy(
                0.0, -0.50, 0.30,
                math.radians(90), 0.0, 0.0
            )
            if not ok:
                rospy.loginfo("再キャリブレーションプラン生成失敗。")
                return
            rospy.loginfo("再キャリブレーション位置へ移動完了。")

            time.sleep(2.0)

            rospy.loginfo("元の位置に戻ります。")
            self.arm.set_pose_target(self.home_pose)
            ok, plan, _, _ = self.arm.plan()
            if ok:
                self.arm.execute(plan, wait=True)
                rospy.loginfo("復帰動作完了。")
            else:
                rospy.loginfo("復帰プラン生成失敗。")

        finally:
            if self.a_ema is not None:
                self.ref_dir = self.a_ema.copy()  # 新しい基準に更新
                rospy.loginfo("基準姿勢を更新。")
            self.alert_active = False
            rospy.loginfo("監視を再開。")

    # -------------------- plot --------------------
    def update_plot(self, frame):
        with self.buf_lock:
            n = min(len(self.gx_buf), len(self.gy_buf), len(self.gz_buf), len(self.tilt_buf))
            if n == 0:
                return []
            x_all = np.arange(n)
            gx = np.asarray(self.gx_buf)
            gy = np.asarray(self.gy_buf)
            gz = np.asarray(self.gz_buf)
            tilt = np.asarray(self.tilt_buf)

            # 表示範囲の決定
            if self.window_len is None:
                xmin, xmax = 0, n
                x, gx_v, gy_v, gz_v, tilt_v = x_all, gx, gy, gz, tilt
            else:
                xmin = max(0, n - self.window_len)
                xmax = n
                x = x_all[xmin:xmax]
                gx_v = gx[xmin:xmax]
                gy_v = gy[xmin:xmax]
                gz_v = gz[xmin:xmax]
                tilt_v = tilt[xmin:xmax]

        self.line_gx.set_data(x, gx_v)
        self.line_gy.set_data(x, gy_v)
        self.line_gz.set_data(x, gz_v)
        self.line_tilt.set_data(x, tilt_v)

        self.ax1.set_xlim(xmin, xmax)
        self.ax2.set_xlim(xmin, xmax)
        return [self.line_gx, self.line_gy, self.line_gz, self.line_tilt]

    # -------------------- spin --------------------
    def spin(self):
        threading.Thread(target=rospy.spin, daemon=True).start()
        plt.show()


if __name__ == "__main__":
    ImuTriggerRecalibrationWithPlot()
