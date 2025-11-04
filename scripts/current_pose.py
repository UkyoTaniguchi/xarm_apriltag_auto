#!/usr/bin/env python3
import rospy
from moveit_commander import MoveGroupCommander, roscpp_initialize, roscpp_shutdown

def main():
    rospy.init_node("xarm6_pose_logger", anonymous=True)
    roscpp_initialize([])

    arm = MoveGroupCommander("xarm6")
    rospy.loginfo("MoveGroupCommander initialized for xarm6")

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
