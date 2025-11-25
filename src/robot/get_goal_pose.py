#!/usr/bin/env python3
import rospy
from moveit_commander import MoveGroupCommander, roscpp_initialize, roscpp_shutdown
from sensor_msgs.msg import JointState

current_joint_state = None

def joint_state_callback(msg):
    global current_joint_state
    current_joint_state = msg

def main():
    rospy.init_node("xarm_fk_getter")
    roscpp_initialize([])

    # MoveIt commander
    arm = MoveGroupCommander("xarm6")

    # Subscribe to joint states
    rospy.Subscriber("/joint_states", JointState, joint_state_callback)

    rospy.loginfo("Waiting for /joint_states...")
    rospy.wait_for_message("/joint_states", JointState)

    rospy.sleep(0.5)

    # 1. 現在の joint_state を MoveIt の start state に設定
    arm.set_start_state_to_current_state()

    # 2. 現在姿勢を FK で取得
    pose = arm.get_current_pose().pose

    # 3. YAML に書くための形式で出力
    print("\n=== FK Result Pose ===")
    print(f"[{pose.position.x:.6f}, {pose.position.y:.6f}, {pose.position.z:.6f}, "
          f"{pose.orientation.x:.6f}, {pose.orientation.y:.6f}, "
          f"{pose.orientation.z:.6f}, {pose.orientation.w:.6f}]")

    roscpp_shutdown()

if __name__ == "__main__":
    main()
