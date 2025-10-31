#!/usr/bin/env python3
import math
import rospy
from moveit_commander import MoveGroupCommander, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler

def move_xarm6(arm):
    arm.set_max_velocity_scaling_factor(0.5)
    arm.set_max_acceleration_scaling_factor(0.5)
    arm.set_planning_time(10.0)  # タイムアウトを延長

    # ---- 目標Pose ----
    q = quaternion_from_euler(0, math.pi/2, 0)  # roll, pitch, yaw (rad)
    target_pose = Pose()
    target_pose.position.x = 0.7
    target_pose.position.y = -0.10
    target_pose.position.z = 0.30
    target_pose.orientation.x = q[0]
    target_pose.orientation.y = q[1]
    target_pose.orientation.z = q[2]
    target_pose.orientation.w = q[3]

    arm.set_start_state_to_current_state()
    arm.set_pose_target(target_pose)

    print("\n--- Target Pose ---")
    print(f"Position : x={target_pose.position.x}, y={target_pose.position.y}, z={target_pose.position.z}")
    print(f"Orientation : x={target_pose.orientation.x}, y={target_pose.orientation.y}, z={target_pose.orientation.z}, w={target_pose.orientation.w}")
    input("\nPress [Enter] to plan and move xArm6...")

    success, plan, _, _ = arm.plan()
    if success:
        print("Plan found. Executing...")
        arm.execute(plan, wait=True)
        print("xArm6 moved to target pose.")
    else:
        print("No valid plan found (IK failed or timed out).")

def main():
    rospy.init_node("xarm6_move_and_log", anonymous=True)
    roscpp_initialize([])

    arm = MoveGroupCommander("xarm6")
    rospy.loginfo("MoveGroupCommander initialized for xarm6")

    move_xarm6(arm)

    rate = rospy.Rate(2.0)
    while not rospy.is_shutdown():
        pose = arm.get_current_pose().pose
        rospy.loginfo(
            "\n[Current Pose]\n"
            f"  Position: x={pose.position.x:.4f}, y={pose.position.y:.4f}, z={pose.position.z:.4f}\n"
            f"  Orientation: x={pose.orientation.x:.4f}, "
            f"y={pose.orientation.y:.4f}, z={pose.orientation.z:.4f}, w={pose.orientation.w:.4f}"
        )
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        roscpp_shutdown()
