#!/usr/bin/env python3
import moveit_commander
import rospy
import copy
import math
import numpy as np
import geometry_msgs.msg
from scipy.spatial.transform import Rotation
from tf.transformations import quaternion_from_euler, quaternion_from_matrix, quaternion_multiply
import tf2_ros
import tf
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped

# --- グローバル変数と初期設定 ---
tf_buffer = None # グローバル変数として初期化
tf_listener = None
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
    xarm.set_max_velocity_scaling_factor(0.1)
    xarm.set_max_acceleration_scaling_factor(0.1)
    
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

    global tf_buffer

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
        
        tag_info_lines.append(f"Tag ID: {tag_id}")
        tag_info_lines.append(f"Position: x={position.x:.6f}, y={position.y:.6f}, z={position.z:.6f}")
        tag_info_lines.append(f"Orientation: x={orientation.x:.6f}, y={orientation.y:.6f}, z={orientation.z:.6f}, w={orientation.w:.6f}")
        tag_info_lines.append("")

    # カメラの位置姿勢をTFから取得
    try:

        rospy.sleep(0.5)  # TFが準備されるまで少し待つ

        transform: TransformStamped = tf_buffer.lookup_transform

        transform: TransformStamped = tf_buffer.lookup_transform(
            target_frame='world',
            source_frame='cam_1_color_optical_frame',
            time=rospy.Time(0),
            timeout=rospy.Duration(2.0)
        )

        t = transform.transform.translation
        r = transform.transform.rotation

        tag_info_lines.append("cam_1_color_optical_frame w.r.t world")
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

def calculate_fallback_pose(pose_stamped_msg, offset_distance=0.25):
    """
    ARマーカと base_link を結ぶ直線上で、ARマーカから手前25cmの位置を目標とします。
    アーム先端のZ軸（ツール方向）がARマーカを向く姿勢を採用します。
    """
    rospy.loginfo("[INFO] フォールバックポーズの計算を開始します。")
    
    # 1. ARタグと base_link の位置を取得
    p_tag_msg = pose_stamped_msg.pose.position
    p_tag = np.array([p_tag_msg.x, p_tag_msg.y, p_tag_msg.z])
    
    try:
        # base_link の位置を world フレームに対して取得
        transform: TransformStamped = tf_buffer.lookup_transform(
            pose_stamped_msg.header.frame_id, # 'world'
            'link_base', # アームの根元
            rospy.Time(0),
            rospy.Duration(1.0)
        )
        p_base = np.array([transform.transform.translation.x, 
                           transform.transform.translation.y, 
                           transform.transform.translation.z])
    except Exception as e:
        rospy.logerr(f"[ERROR] link_base のTF取得に失敗しました: {e}. base_linkを原点(0,0,0)と仮定します。")
        p_base = np.array([0.0, 0.0, 0.0])

    # 2. ARタグ -> base_link への方向ベクトル (V_base)
    # フォールバック位置 P_fall は P_tag + (0.25m * 単位V_base)
    v_base = p_base - p_tag
    v_base_norm = np.linalg.norm(v_base)
    
    if v_base_norm < 1e-6:
        rospy.logerr("[ERROR] ARタグと base_link の位置が近すぎます。フォールバック計算を中止します。")
        return None

    v_base_unit = v_base / v_base_norm
    
    # 3. 目標位置 P_fall を計算 (ARタグからベースに向かって offset_distance)
    p_fall = p_tag + v_base_unit * offset_distance

    # 4. 目標姿勢 (アーム先端のZ軸がARタグを向く)
    
    # Z-axis: ツール先端から ARタグの中心を指す方向
    v_z_fwd = p_tag - p_fall
    v_z_unit = v_z_fwd / np.linalg.norm(v_z_fwd)
    
    # Y-axis (垂直方向の制約): ツール先端のY軸がワールドZ軸に垂直になるように調整
    world_z = np.array([0, 0, 1])
    
    # X-axis (右方向): ワールドZとZ軸の外積から横方向を計算
    v_x_right = np.cross(v_z_unit, world_z)
    v_x_right_norm = np.linalg.norm(v_x_right)
    
    if v_x_right_norm < 1e-6:
        # ターゲットがワールドZ軸上にある場合（非常に稀）
        v_x_right = np.array([1, 0, 0]) # X軸をデフォルトに設定
    else:
        v_x_right = v_x_right / v_x_right_norm
    
    # Y-axis (上方向): 右方向と前方向の外積
    v_y_up = np.cross(v_z_unit, v_x_right)
    
    # 回転行列 R = [V_x | V_y | V_z] (MoveIt!の慣例に基づく)
    R_matrix = np.column_stack([v_x_right, v_y_up, v_z_unit])

    T_matrix_4x4 = np.identity(4)
    T_matrix_4x4[:3, :3] = R_matrix

    q_original = quaternion_from_matrix(T_matrix_4x4)  # (x, y, z, w)

    # Z軸周りに -90度の回転 (グリッパーの向きの調整)
    rotation_angle_z_deg = -90
    q_rot = quaternion_from_euler(0, 0, np.deg2rad(rotation_angle_z_deg))

    q_new = quaternion_multiply(q_original, q_rot)

    # 回転行列をクォータニオンに変換
    # q_new = quaternion_from_matrix(T_matrix_4x4)

    # 5. geometry_msgs/Pose オブジェクトの作成
    fallback_pose = geometry_msgs.msg.Pose()
    fallback_pose.position.x = p_fall[0]
    fallback_pose.position.y = p_fall[1]
    fallback_pose.position.z = p_fall[2]
    fallback_pose.orientation.x = q_new[0]
    fallback_pose.orientation.y = q_new[1]
    fallback_pose.orientation.z = q_new[2]
    fallback_pose.orientation.w = q_new[3]
    
    return fallback_pose

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
    失敗した場合、ARマーカからの距離を遠くして再試行し、
    それでも失敗した場合はフォールバックポーズで最終試行を行います。
    """
    global xarm
    rospy.loginfo("--- xArm 移動開始（再試行ロジック付き）---")

    # 再試行のパラメータ (0.25m, 0.30m, 0.35m, 0.40m, 0.45m, 0.50m を試行)
    initial_offset = 0.25 
    max_offset = 0.50     
    step_size = 0.05      
    
    current_offset = initial_offset
    success = False

    # --- 1. 標準的なZ軸後退と姿勢固定の再試行ループ ---
    while current_offset <= max_offset:
        target_pose = calculate_target_pose(pose_stamped_msg, current_offset)

        rospy.loginfo(f"試行 (標準): オフセット距離 {current_offset:.2f}m")

        xarm.set_pose_target(target_pose)
        
        plan_success, plan, _, _ = xarm.plan()
        
        if plan_success and len(plan.joint_trajectory.points) > 0:
            rospy.loginfo(f"オフセット {current_offset:.2f}m でプランニングに成功しました。実行します。")
            success = xarm.execute(plan, wait=True)
            if success:
                break 
            else:
                rospy.logwarn("プランニングは成功しましたが、実行に失敗しました。次のオフセットを試行します。")
        else:
            rospy.logwarn(f"オフセット {current_offset:.2f}m でプランニングに失敗しました。次のオフセットを試行します。")
            
        xarm.stop()
        xarm.clear_pose_targets()
        current_offset += step_size

    # --- 2. フォールバックポーズでの最終試行 ---
    if not success:
        rospy.logwarn("標準的なZ軸後退の再試行がすべて失敗しました。フォールバックポーズで距離を変えて試行します。")
        
        # 再試行のパラメータを定義
        fallback_initial_offset = 0.25 
        fallback_max_offset = 0.50     
        fallback_step_size = 0.05      
        
        current_fallback_offset = fallback_initial_offset

        while current_fallback_offset <= fallback_max_offset:
            rospy.loginfo(f"試行 (フォールバック): base_link方向 オフセット距離 {current_fallback_offset:.2f}m")
            
            # フォールバックポーズを計算 (ARタグからbase_linkに向かって current_fallback_offset)
            fallback_pose = calculate_fallback_pose(pose_stamped_msg, offset_distance=current_fallback_offset)
            
            if fallback_pose:
                xarm.set_pose_target(fallback_pose)
                
                plan_success, plan, _, _ = xarm.plan()
                
                if plan_success and len(plan.joint_trajectory.points) > 0:
                    rospy.loginfo(f"フォールバックポーズ (オフセット {current_fallback_offset:.2f}m) でプランニングに成功しました。実行します。")
                    success = xarm.execute(plan, wait=True)
                    if success:
                        break # 成功したらループを抜ける
                    else:
                        rospy.logwarn("フォールバックポーズでのプランニングは成功しましたが、実行に失敗しました。次のオフセットを試行します。")
                else:
                    rospy.logwarn(f"フォールバックポーズ (オフセット {current_fallback_offset:.2f}m) でプランニングに失敗しました。次のオフセットを試行します。")
            else:
                rospy.logerr("フォールバックポーズの計算ができませんでした。これ以上の試行を中止します。")
                break # 計算自体に失敗したら中止
                
            xarm.stop()
            xarm.clear_pose_targets()
            current_fallback_offset += fallback_step_size

    # 最終的な停止と目標クリア
    xarm.stop()
    xarm.clear_pose_targets()

    if success:
        rospy.loginfo("[INFO] Move to subscribed pose succeeded.")
    else:
        rospy.logerr(f"[ERROR] すべての移動試行（標準再試行およびフォールバック）に失敗しました。")
        
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
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1.0) # TFの準備を待つ
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