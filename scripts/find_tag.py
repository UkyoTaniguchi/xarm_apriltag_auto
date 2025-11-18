#!/usr/bin/env python3
import moveit_commander
import rospy
import copy
import math
import numpy as np
import geometry_msgs.msg
from scipy.spatial.transform import Rotation
from tf.transformations import quaternion_from_euler
# --- 新規追加: TFとファイル操作のためのライブラリ ---
import tf2_ros
import tf
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped

# --- グローバル変数と初期設定 ---
# MoveIt! Library
robot = moveit_commander.RobotCommander()
xarm = moveit_commander.MoveGroupCommander("xarm6")

# サブスクライブするトピック名
TARGET_POSE_TOPIC_ID0 = "/ar_tag/id0/pose_in_world"
TARGET_POSE_TOPIC_ID1 = "/ar_tag/id1/pose_in_world"
TAG_DETECTIONS_TOPIC = "/tag_detections" # ARタグ検出情報のトピック

# 動作ステータスを追跡するためのフラグ
# ステータス: "WAITING_FOR_ID0", "MOVING_TO_ID0", "WAITING_FOR_ID1", "DONE"
TARGET_STATUS = "WAITING_FOR_ID0"

# サブスクライバーオブジェクト (後で解除するために保持)
id0_subscriber = None
id1_subscriber = None


def ArmInitialization():
    """MoveIt! の初期設定とアームの状態を出力します。"""
    global xarm
    
    # 速度と加速度を設定
    xarm.set_max_velocity_scaling_factor(0.2)
    xarm.set_max_acceleration_scaling_factor(0.2)
    
    # 初期状態の情報を出力
    rospy.loginfo("=" * 10 + " Robot Initialization Complete " + "=" * 10)
    rospy.loginfo(f"Reference frame: {xarm.get_planning_frame()}")
    rospy.loginfo(f"End effector link: {xarm.get_end_effector_link()}")


# --- 新規追加: ポーズ情報を保存する関数 ---
def save_tag_and_camera_pose_tf(tag_id_list, file_suffix):
    """
    指定されたタグIDとカメラポーズの情報をTFから取得し、ファイルに保存します。
    :param tag_id_list: 保存対象のタグIDのリスト (例: [0], [1], [0, 1])
    :param file_suffix: 保存ファイル名の接尾辞 (例: "_id0_done.txt")
    """
    # データを一時的に保持するための辞書
    current_tag_info = {}
    
    # Tag detectionsトピックから最新のメッセージを一度だけ取得するためのSubscriber
    detection_msg = rospy.wait_for_message(TAG_DETECTIONS_TOPIC, AprilTagDetectionArray, timeout=5.0)

    for detection in detection_msg.detections:
        tag_id = detection.id[0]
        if tag_id in tag_id_list:
            current_tag_info[tag_id] = detection.pose.pose.pose

    # ファイルに書き込む情報リスト
    tag_info_lines = []
    
    # 検出されたタグ情報をフォーマット
    for tag_id in sorted(current_tag_info.keys()):
        pose = current_tag_info[tag_id]
        position = pose.position
        orientation = pose.orientation
        
        tag_info_lines.append(f"--- Tag ID: {tag_id} ---")
        tag_info_lines.append(f"Position: x={position.x:.6f}, y={position.y:.6f}, z={position.z:.6f}")
        tag_info_lines.append(f"Orientation: x={orientation.x:.6f}, y={orientation.y:.6f}, z={orientation.z:.6f}, w={orientation.w:.6f}")
        tag_info_lines.append("")

    # カメラの位置姿勢をTFから取得
    try:
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        rospy.sleep(0.5)  # TFが準備されるまで少し待つ

        transform: TransformStamped = tf_buffer.lookup_transform(
            target_frame='world',
            source_frame='cam_1_color_optical_frame',
            time=rospy.Time(0),
            timeout=rospy.Duration(2.0)
        )

        t = transform.transform.translation
        r = transform.transform.rotation

        tag_info_lines.append("--- cam_1_color_optical_frame w.r.t world ---")
        tag_info_lines.append(f"Position: x={t.x:.6f}, y={t.y:.6f}, z={t.z:.6f}")
        tag_info_lines.append(f"Orientation: x={r.x:.6f}, y={r.y:.6f}, z={r.z:.6f}, w={r.w:.6f}")

        # 指定されたパスに保存
        file_path = f"/home/ukyo/catkin_ws/src/xarm_apriltag_demo/public/accurate_tag_pose{file_suffix}"
        with open(file_path, "w") as f:
            f.write("\n".join(tag_info_lines))

        rospy.loginfo(f"[INFO] Pose information saved to {file_path}")

    except Exception as e:
        rospy.logerr(f"[ERROR] Failed to get transform or write file: {e}")
        
    return 


def calculate_target_pose(pose_msg, offset_distance):
    """
    受信したポーズにオフセットを適用し、グリッパーの姿勢を修正した新しいポーズを計算します。
    """
    modified_pose = copy.deepcopy(pose_msg.pose)
    
    # --- 1. ローカルオフセットの適用 ---
    orientation_quat = np.array([
        modified_pose.orientation.x,
        modified_pose.orientation.y,
        modified_pose.orientation.z,
        modified_pose.orientation.w
    ])
    
    # ScipyのRotationオブジェクトを作成
    r_current = Rotation.from_quat(orientation_quat)

    # ターゲットからZ軸方向に offset_distance 後退した位置を目標とする
    local_offset = np.array([0.0, 0.0, offset_distance]) 

    # ローカル移動ベクトルをワールド座標系に変換
    world_offset = r_current.apply(local_offset)
    
    # ワールド座標系の位置を更新 (後退)
    modified_pose.position.x += world_offset[0]
    modified_pose.position.y += world_offset[1]
    modified_pose.position.z += world_offset[2]
    
    # --- 2. 姿勢の修正 (グリッパーが下を向き、ターゲットを向くように修正) ---
    
    # ターゲットポーズのX軸周りに180度回転 (グリッパーが下を向くように)
    rotation_angle_x_deg = 180
    r_180_x = Rotation.from_rotvec(np.array([1, 0, 0]) * np.deg2rad(rotation_angle_x_deg))

    # ターゲットポーズのZ軸周りに-90度回転 (グリッパーの向きを調整)
    rotation_angle_z_deg = -90
    r_m90_z = Rotation.from_rotvec(np.array([0, 0, 1]) * np.deg2rad(rotation_angle_z_deg))

    # 新しい姿勢の計算 (現在の姿勢 * 180度X回転 * -90度Z回転)
    r_new = r_current * r_180_x * r_m90_z

    # 新しい姿勢のクォータニオン成分を取得 (x, y, z, w)
    q_new_xyz_w = r_new.as_quat()
    modified_pose.orientation.x = q_new_xyz_w[0]
    modified_pose.orientation.y = q_new_xyz_w[1]
    modified_pose.orientation.z = q_new_xyz_w[2]
    modified_pose.orientation.w = q_new_xyz_w[3]
    
    return modified_pose


def pose_callback_id0(msg):
    """
    ID0のポーズを受信した際に呼び出され、アームをID0へ移動させます。
    """
    global TARGET_STATUS, id0_subscriber
    
    if TARGET_STATUS != "WAITING_FOR_ID0":
        return

    rospy.loginfo(f"[{rospy.get_name()}] ID0のポーズを受信しました。ID0への移動を開始します。")
    TARGET_STATUS = "MOVING_TO_ID0"
    
    # ID0への移動を実行
    success = move_to_subscribed_pose(msg)
    
    if success:
        rospy.loginfo("[INFO] ID0への移動に成功しました。")
        # --- 移動後のポーズ保存 (ID0) ---
        save_tag_and_camera_pose_tf(tag_id_list=[0], file_suffix="_id0_done.txt")
        # --------------------------------
    else:
        rospy.logwarn("[WARN] ID0への移動に失敗しました。")
        
    # ID0の購読を解除し、ID1の購読を開始
    if id0_subscriber is not None:
        id0_subscriber.unregister()
    
    start_id1_phase() # ID1の待ち受けフェーズへ移行


def pose_callback_id1(msg):
    """
    ID1のポーズを受信した際に呼び出され、アームをID1へ移動させます。
    """
    global TARGET_STATUS, id1_subscriber

    if TARGET_STATUS != "WAITING_FOR_ID1":
        return

    rospy.loginfo(f"[{rospy.get_name()}] ID1のポーズを受信しました。ID1への移動を開始します。")
    TARGET_STATUS = "DONE"
    
    # ID1への移動を実行
    success = move_to_subscribed_pose(msg)
    
    if success:
        rospy.loginfo("[INFO] ID1への移動に成功しました。")
        # --- 移動後のポーズ保存 (ID1) ---
        save_tag_and_camera_pose_tf(tag_id_list=[1], file_suffix="_id1_done.txt")
        # --------------------------------
    else:
        rospy.logwarn("[WARN] ID1への移動に失敗しました。")
    
    # ID1の購読を解除し、プログラムを終了
    if id1_subscriber is not None:
        id1_subscriber.unregister()
        
    rospy.signal_shutdown("Sequential movement task completed.")


def move_to_subscribed_pose(pose_stamped_msg):
    """
    MoveIt! を使用して指定されたポーズに移動します。
    移動が失敗した場合、ARマーカからの距離を遠くして再試行します。
    """
    global xarm
    rospy.loginfo("--- xArm 移動開始（再試行ロジック付き）---")

    # 再試行のパラメータ
    initial_offset = 0.25  # 初期距離 
    max_offset = 0.50      # 最大距離 
    step_size = 0.05       # 増加ステップ
    
    current_offset = initial_offset
    success = False

    while current_offset <= max_offset:
        # 1. 目標ポーズを計算
        target_pose = calculate_target_pose(pose_stamped_msg, current_offset)

        rospy.loginfo(f"試行: オフセット距離 {current_offset:.2f}m")

        # 2. 目標ポーズを設定
        xarm.set_pose_target(target_pose)
        
        # 3. プランニングを実行し、結果を正しく受け取る
        plan_success, plan, _, _ = xarm.plan()
        
        # 4. プランニング結果の評価
        if plan_success and len(plan.joint_trajectory.points) > 0:
            rospy.loginfo(f"オフセット {current_offset:.2f}m でプランニングに成功しました。実行します。")
            
            # 5. 実行
            success = xarm.execute(plan, wait=True)
            if success:
                break # 成功したのでループを抜ける
            else:
                rospy.logwarn("プランニングは成功しましたが、実行に失敗しました。次のオフセットを試行します。")
        else:
            rospy.logwarn(f"オフセット {current_offset:.2f}m でプランニングに失敗しました。次のオフセットを試行します。")
            
        # 停止と目標クリア (次の試行のために)
        xarm.stop()
        xarm.clear_pose_targets()
        
        # 次のオフセットへ
        current_offset += step_size

    # 最終的な停止と目標クリア
    xarm.stop()
    xarm.clear_pose_targets()

    if success:
        rospy.loginfo("[INFO] Move to subscribed pose succeeded.")
    else:
        rospy.logerr(f"[ERROR] Max offset {max_offset:.2f}m まで試行しましたが、移動に失敗しました。")
        
    return success

def start_id1_phase():
    """ID1のポーズ待ち受けフェーズへ移行し、サブスクライバを設定します。"""
    global TARGET_STATUS, id1_subscriber
    
    TARGET_STATUS = "WAITING_FOR_ID1"
    # ID1のトピック購読を開始
    id1_subscriber = rospy.Subscriber(
        TARGET_POSE_TOPIC_ID1, 
        geometry_msgs.msg.PoseStamped, 
        pose_callback_id1
    )
    rospy.loginfo(f"[{rospy.get_name()}] ID0への移動完了。トピック {TARGET_POSE_TOPIC_ID1} のポーズ情報を待機中...")


if __name__ == '__main__':
    rospy.init_node("xArm_auto_move_sequential")
    ArmInitialization()

    # --- ID0のサブスクライバーを作成 (最初に待機) ---
    id0_subscriber = rospy.Subscriber(
        TARGET_POSE_TOPIC_ID0, 
        geometry_msgs.msg.PoseStamped, 
        pose_callback_id0
    )
    rospy.loginfo(f"[{rospy.get_name()}] トピック {TARGET_POSE_TOPIC_ID0} のポーズ情報を待機中...")
    
    # ROSノードを終了させずに、コールバック関数による移動指示を待ち続けます
    rospy.spin()

    rospy.loginfo("[INFO] Sequential movement task finished.")