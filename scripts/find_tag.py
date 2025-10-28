#!/usr/bin/env python3
import moveit_commander
import rospy
import os
import tf
import geometry_msgs.msg
import math
from tf.transformations import quaternion_from_euler

# moveit library
robot = moveit_commander.RobotCommander()
xarm = moveit_commander.MoveGroupCommander("xarm6")

def ArmInitialization():
    robot = moveit_commander.RobotCommander()

    print("=" * 10, " Robot Groups:")
    print(robot.get_group_names())
    print("=" * 10, " Printing robot state")
    print(robot.get_current_state())
    print("=" * 15, " xarm ", "=" * 15)
    print("=" * 10, " Reference frame: %s" %
          xarm.get_planning_frame())
    print("=" * 10, " Reference frame: %s" %
          xarm.get_end_effector_link())

    xarm_initial_pose = xarm.get_current_pose().pose
    print("=" * 10, " Printing Xarm initial pose: ")
    print(xarm_initial_pose)

    xarm_initial_rpy = xarm.get_current_rpy()
    print("=" * 10, " Printing Xarm initial rpy: ")
    print(xarm_initial_rpy)

    xarm.set_max_velocity_scaling_factor(0.2)
    xarm.set_max_acceleration_scaling_factor(0.2)


def go_to_predefined_pose():
    """
    起動直後に指定座標へ移動する関数
    """
    q = quaternion_from_euler(0, math.pi/2, 0)

    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.7
    target_pose.position.y = 0.10
    target_pose.position.z = 0.30
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]

    xarm.set_max_velocity_scaling_factor(0.2)
    xarm.set_max_acceleration_scaling_factor(0.2)
    xarm.set_pose_target(target_pose)
    success = xarm.go(wait=True)
    xarm.stop()
    xarm.clear_pose_targets()

    if success:
        print("[INFO] Move to predefined pose succeeded.")
    else:
        print("[WARN] Move to predefined pose failed.")


if __name__ == '__main__':
    rospy.init_node("xArm_auto_move")
    ArmInitialization()
    go_to_predefined_pose()
    print("[INFO] Task complete. Program finished.")
