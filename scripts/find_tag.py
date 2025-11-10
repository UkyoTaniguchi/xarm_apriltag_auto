#!/usr/bin/env python3
import moveit_commander
import rospy
import os
import tf
import geometry_msgs.msg
import math
from tf.transformations import quaternion_from_euler
import copy
from scipy.spatial.transform import Rotation
import numpy as np

# moveit library
robot = moveit_commander.RobotCommander()
xarm = moveit_commander.MoveGroupCommander("xarm6")

# None の場合はポーズ未受信
TARGET_POSE_STAMPED = None 

# サブスクライブするトピック名
TARGET_POSE_TOPIC = "/ar_tag/id0/pose_in_world"

FIXED_QUATERNION = quaternion_from_euler(math.pi/2, -math.pi/2, math.pi/2)

MOVEMENT_ATTEMPTED = False
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


def pose_callback(msg):
    """
    指定トピックから geometry_msgs/PoseStamped メッセージを受信し、
    グローバル変数 TARGET_POSE_STAMPED を更新します。
    """
    global TARGET_POSE_STAMPED, MOVEMENT_ATTEMPTED

    if MOVEMENT_ATTEMPTED:
        return
    
    MOVEMENT_ATTEMPTED = True
    TARGET_POSE_STAMPED = msg
    rospy.loginfo(f"[{rospy.get_name()}] 目標ポーズを受信しました。処理を開始します。")
    
    # コピーすることで、オリジナルのメッセージを変更することを防ぎます
    modified_pose = copy.deepcopy(msg.pose)
   
    # --- 2. 姿勢の固定 (事前に計算したクォータニオンで上書き) ---
    orientation_quat = np.array([
        modified_pose.orientation.x,
        modified_pose.orientation.y,
        modified_pose.orientation.z,
        modified_pose.orientation.w
    ])
    
    # ScipyのRotationオブジェクトを作成
    r_current = Rotation.from_quat(orientation_quat)

    local_offset = np.array([0.0, 0.0, 0.25]) 

    # ローカル移動ベクトルをワールド座標系に変換
    world_offset = r_current.apply(local_offset)
    
    # ワールド座標系の位置を更新
    modified_pose.position.x += world_offset[0]
    modified_pose.position.y += world_offset[1]
    modified_pose.position.z += world_offset[2]
    

    rotation_angle_x_deg = 180
    # Rotation.from_rotvec(軸ベクトル * ラジアン角) で180度回転を表すクォータニオンを作成
    r_90_x = Rotation.from_rotvec(np.array([1, 0, 0]) * np.deg2rad(rotation_angle_x_deg))

    rotation_angle_z_deg = -90
    r_90_z = Rotation.from_rotvec(np.array([0, 0, 1]) * np.deg2rad(rotation_angle_z_deg))

    # --- 3. 新しい姿勢の計算 (乗算) ---
    r_new = r_current * r_90_x* r_90_z

    # 新しい姿勢のクォータニオン成分を取得 (x, y, z, w)
    q_new_xyz_w = r_new.as_quat()
    modified_pose.orientation.x = q_new_xyz_w[0]
    modified_pose.orientation.y = q_new_xyz_w[1]
    modified_pose.orientation.z = q_new_xyz_w[2]
    modified_pose.orientation.w = q_new_xyz_w[3]
    
    # トピックを受信したら、移動処理を呼び出す
    move_to_subscribed_pose(modified_pose)


# --- 新規追加: 受信したポーズに移動する関数 ---
def move_to_subscribed_pose(target_pose):
    """
    MoveIt! を使用して指定された geometry_msgs/Pose に移動
    """
    rospy.loginfo("--- xArm 移動開始 ---")

    xarm.set_planning_time(10.0)
    xarm.set_num_planning_attempts(5)
    
    xarm.set_max_velocity_scaling_factor(0.1)
    xarm.set_max_acceleration_scaling_factor(0.1)
    
    # 目標ポーズを設定
    xarm.set_pose_target(target_pose)
    
    # プランニングと実行
    success = xarm.go(wait=True)
    
    # 停止と目標クリア
    xarm.stop()
    xarm.clear_pose_targets()

    if success:
        rospy.loginfo("[INFO] Move to subscribed pose succeeded.")
    else:
        rospy.logwarn("[WARN] Move to subscribed pose failed. Check constraints or planning space.")
    
    rospy.signal_shutdown("Movement attempt completed.")

if __name__ == '__main__':
    rospy.init_node("xArm_auto_move")
    ArmInitialization()
    # --- サブスクライバーの作成 ---
    rospy.Subscriber(TARGET_POSE_TOPIC, geometry_msgs.msg.PoseStamped, pose_callback)
    rospy.loginfo(f"[{rospy.get_name()}] トピック {TARGET_POSE_TOPIC} のポーズ情報を待機中...")
    
    # ROSノードを終了させずに、コールバック関数による移動指示を待ち続けます
    rospy.spin()
    # go_to_predefined_pose()

    print("[INFO] Task complete. Program finished.")
