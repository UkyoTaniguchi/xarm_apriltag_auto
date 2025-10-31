#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
xArm6 再キャリブレーションサーバノード
------------------------------------
/recalibration/run (std_srvs/Trigger) サービスを提供。

呼び出されると：
  1. 再キャリブレーション用姿勢へ移動
  2. 2秒待機
  3. ホーム姿勢に復帰

IMU監視ノード（imu_monitor_trigger_with_plot.py）から呼び出される。
"""

import rospy
import math, time
import moveit_commander
from tf.transformations import quaternion_from_euler
from std_srvs.srv import Trigger, TriggerResponse


class XarmRecalibrationServer:
    """xArm6の再キャリブレーション動作を担当するサービスサーバ"""

    def __init__(self):
        # =========================
        # ROSノード初期化
        # =========================
        rospy.init_node("xarm_recalibration_server")
        rospy.loginfo("Initializing xArm recalibration server...")

        # MoveIt初期化
        moveit_commander.roscpp_initialize([])
        self.arm = moveit_commander.MoveGroupCommander("xarm6")
        self.arm.set_max_velocity_scaling_factor(1.0)
        self.arm.set_max_acceleration_scaling_factor(1.0)
        self.arm.set_planning_time(1.0)
        rospy.sleep(1.0)

        # 現在姿勢をホームポーズとして保持
        self.home_pose = self.arm.get_current_pose().pose
        self.is_running = False

        # サービス登録
        self.srv = rospy.Service("/recalibration/run", Trigger, self.handle_recalibration)
        rospy.loginfo("Service [/recalibration/run] ready.")
        rospy.spin()

    # =========================================================
    # サービスコールバック
    # =========================================================
    def handle_recalibration(self, req):
        """
        /recalibration/run が呼ばれたときに実行される。
        実行中であれば拒否。
        """
        if self.is_running:
            return TriggerResponse(success=False, message="Recalibration already running.")
        self.is_running = True

        try:
            rospy.loginfo("Recalibration sequence start.")

            # ---- Step 1: 再キャリブレーション姿勢へ ----
            success = self.move_to_xyzrpy(
                x=0.0, y=-0.50, z=0.30,
                rx=math.radians(90), ry=0.0, rz=0.0
            )
            if not success:
                rospy.logwarn("Failed to plan path to recalibration pose.")
                return TriggerResponse(success=False, message="Failed to plan recalibration pose.")

            rospy.loginfo("Moved to recalibration pose. Holding for 2 seconds.")
            time.sleep(2.0)

            # ---- Step 2: ホーム姿勢に戻す ----
            rospy.loginfo("Returning to home pose.")
            self.arm.set_pose_target(self.home_pose)
            ok, plan, _, _ = self.arm.plan()
            if ok:
                self.arm.execute(plan, wait=True)
                rospy.loginfo("Returned to home pose successfully.")
                return TriggerResponse(success=True, message="Recalibration completed successfully.")
            else:
                rospy.logwarn("Failed to plan return to home pose.")
                return TriggerResponse(success=False, message="Failed to plan home pose.")

        except Exception as e:
            rospy.logerr(f"Recalibration failed: {e}")
            return TriggerResponse(success=False, message=str(e))

        finally:
            self.is_running = False
            rospy.loginfo("Recalibration process finished (server ready).")

    # =========================================================
    # 汎用関数: XYZ + RPY指定で姿勢移動
    # =========================================================
    def move_to_xyzrpy(self, x, y, z, rx, ry, rz):
        """
        引数: 座標(x, y, z)[m], 姿勢(rx, ry, rz)[rad]
        現在姿勢から指定姿勢へMoveIt!経由で移動。
        成功すればTrueを返す。
        """
        pose = self.arm.get_current_pose().pose
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        qx, qy, qz, qw = quaternion_from_euler(rx, ry, rz)
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw

        self.arm.set_pose_target(pose)
        ok, plan, _, _ = self.arm.plan()
        if ok:
            self.arm.execute(plan, wait=True)
            return True
        return False


# =========================================================
# メイン処理
# =========================================================
if __name__ == "__main__":
    XarmRecalibrationServer()
