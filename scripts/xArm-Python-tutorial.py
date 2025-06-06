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
# psi = moveit_commander.PlanningSceneInterface()

if os.name == 'nt':
    import msvcrt

    def getch():
        return msvcrt.getch().decode()
else:
    import sys
    import tty
    import termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

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


def Go_homeposition():
    home_pose = geometry_msgs.msg.Pose()
    home_pose.position.x = 0.1989471831501947
    home_pose.position.y = -4.213686537985398e-05
    home_pose.position.z = 0.033605173050147194#+0.750
    home_pose.orientation.x = -0.9999072837831338
    home_pose.orientation.y = -4.859141964351074e-05
    home_pose.orientation.z = 0.013613290340705797
    home_pose.orientation.w = 0.00031591519361944936

    xarm.set_max_velocity_scaling_factor(0.2)
    xarm.set_max_acceleration_scaling_factor(0.2)
    xarm.set_pose_target(home_pose)
    xarm.go()

# debug
def armpose_checker():
    pose=xarm.get_current_pose().pose
    print("=" * 10, " Printing Xarm current pose: ")
    print(pose)
    rpy = xarm.get_current_rpy()
    print("=" * 10, " Printing Xarm current rpy: ","="*10)
    print(rpy)

def test_pose():
    target_pose = geometry_msgs.msg.Pose()
    target_pose.position.x = 0.300
    target_pose.position.y = 0.400
    target_pose.position.z = 0.200
    target_pose.orientation.x = -0.9999072837831338
    target_pose.orientation.y = -4.859141964351074e-05
    target_pose.orientation.z = 0.013613290340705797
    target_pose.orientation.w = 0.00031591519361944936
    
    xarm.set_max_velocity_scaling_factor(0.2)
    xarm.set_max_acceleration_scaling_factor(0.2)
    xarm.set_pose_target(target_pose)
    xarm.go()

def apriltag_towoard_pose():
    q = quaternion_from_euler(0, math.pi/2, 0)  # x方向を向く
    
    target_pose = geometry_msgs.msg.Pose()
    # target_pose.position.x = x
    # target_pose.position.y = y
    # target_pose.position.z = z
    # target_pose.orientation.x = q[0]
    # target_pose.orientation.y = q[1]
    # target_pose.orientation.z = q[2]
    # target_pose.orientation.w = q[3]
    target_pose.position.x = 0.8462385436134898
    target_pose.position.y = 0.11094014777770038
    target_pose.position.z = 0.3361595531386302
    target_pose.orientation.x = 0.7178595194441207
    target_pose.orientation.y = -0.0007155137903498503
    target_pose.orientation.z = 0.6961876143794062
    target_pose.orientation.w = 6.299351872424036e-05

    xarm.set_max_velocity_scaling_factor(0.2)
    xarm.set_max_acceleration_scaling_factor(0.2)
    xarm.set_pose_target(target_pose)
    xarm.go()

"""
課題にある動作を実現する関数を作ってみよう.
なお、必ずGazeboシミュレーション上で動作を確認してから実機でチェックを行うこと.
moveitの関数の使い方の説明等は以下のURLを見れば分かります.
URL:https://robo-marc.github.io/moveit_documents/moveit_commander.html


gazeboでシミュレーションする方法:
ターミナル
$ roslaunch xarm_gazebo xarm6_beside_table.launch
別ターミナル
$ roslaunch xarm6_moveit_config xarm6_moveit_gazebo.launch

実機で動作させる場合:
$ roslaunch xarm6_moveit_config realMove_exec.launch robot_ip:=192.168.1.217 

"""


if __name__ == '__main__':
    # connect robot arm (xArm6)
    rospy.init_node("xArm_tutorial")
    ArmInitialization()

    while True:
        print("************* Command ***************")
        print("c:アームの状態取得")
        print("q:終了")
        print("t:テストポーズ")
        print("h:ホームポジションに戻る")
        print("a:apriltag認識ポーズ")
        mode = input("mode select>>")
        print("You select mode : %s " % mode)

        if mode == "c":
            armpose_checker()

        elif mode == "q":
            quit()

        elif mode=="t":
            test_pose()

        elif mode=="h":
            Go_homeposition()

        elif mode == "a":
            # x = float(input("x方向を入力"))
            # y = float(input("y方向を入力"))
            # z = float(input("z方向を入力"))
            apriltag_towoard_pose()

        else:
            print("there isn't such mode")


