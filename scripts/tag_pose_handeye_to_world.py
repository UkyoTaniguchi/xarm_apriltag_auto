#!/usr/bin/env python3
# world 座標系が親

import rospy
import tf
from geometry_msgs.msg import TransformStamped
import tf.transformations as tft
import math
import numpy as np
import re
import os
import numpy as np
from tf.transformations import quaternion_matrix, quaternion_multiply, translation_matrix, concatenate_matrices, inverse_matrix
import geometry_msgs.msg

TAG_FILE_DEFAULT = "/home/ukyo/catkin_ws/src/xarm_apriltag_auto/public/tag_pose.txt"

_pos_re = re.compile(r"Position:\s*x=([-\d\.eE]+),\s*y=([-\d\.eE]+),\s*z=([-\d\.eE]+)")
_ori_re = re.compile(r"Orientation:\s*x=([-\d\.eE]+),\s*y=([-\d\.eE]+),\s*z=([-\d\.eE]+),\s*w=([-\d\.eE]+)")

def _pick_first_match(lines, start_idx):
    """start_idx 以降で Position/Orientation を（この順で）拾って返す"""
    pos = ori = None
    for i in range(start_idx, len(lines)):
        if pos is None:
            m = _pos_re.search(lines[i])
            if m:
                pos = tuple(float(m.group(k)) for k in (1,2,3))
                continue
        if pos is not None and ori is None:
            m = _ori_re.search(lines[i])
            if m:
                ori = tuple(float(m.group(k)) for k in (1,2,3,4))  # (x,y,z,w)
                break
    return pos, ori

def load_from_tag_pose_file(path, wanted_tag_id=0):
    """
    tag_pose.txt から
      1) 指定 Tag ID の Position/Orientation
      2) cam_1_color_optical_frame w.r.t world の Position/Orientation
    を読み取って返す。
    戻り値:
      translation(tuple3), raw_quat(tuple4), T_w_c1_pos(np.array3), T_w_c1_quat(tuple4)
    """
    if not os.path.exists(path):
        raise FileNotFoundError(f"{path} が見つかりません。")

    with open(path, "r") as f:
        lines = [ln.strip() for ln in f.readlines()]

    # --- Tag ID ブロックを探す ---
    tag_idx = None
    for i, ln in enumerate(lines):
        if ln.startswith("Tag ID:"):
            try:
                tid = int(ln.split(":")[1].strip())
                if tid == wanted_tag_id:
                    tag_idx = i
                    break
            except Exception:
                pass
    if tag_idx is None:
        raise ValueError(f"Tag ID: {wanted_tag_id} の記述が見つかりません。")

    translation, raw_quat = _pick_first_match(lines, tag_idx)
    if translation is None or raw_quat is None:
        raise ValueError("Tag の Position/Orientation を解釈できませんでした。")

    # --- cam_1_color_optical_frame ブロックを探す ---
    cam1_idx = None
    for i, ln in enumerate(lines):
        if ln.startswith("cam_1_color_optical_frame w.r.t world:"):
            cam1_idx = i
            break
    if cam1_idx is None:
        raise ValueError("'cam_1_color_optical_frame w.r.t world:' の行が見つかりません。")

    cam1_pos, cam1_quat = _pick_first_match(lines, cam1_idx)
    if cam1_pos is None or cam1_quat is None:
        raise ValueError("cam_1 の Position/Orientation を解釈できませんでした。")

    return translation, raw_quat, cam1_pos, cam1_quat

def calculate_tag_pose_in_world():
    # パラメータ（必要なら rosparam で上書き可能）
    tag_file = rospy.get_param("~tag_file", TAG_FILE_DEFAULT)
    tag_id   = rospy.get_param("~tag_id", 0)

    # --- 1) ファイルから数値を取り込む ---
    translation, raw_quat, T_w_c1_pos, T_w_c1_quat = load_from_tag_pose_file(tag_file, tag_id)
    print(T_w_c1_quat)

    # --- 1. カメラ (cam_1_color_optical_frame) のポーズ (World w.r.t) の変換行列 (T_world_cam) を作成 ---
    R_world_cam = quaternion_matrix(T_w_c1_quat)
    T_trans_world_cam = translation_matrix(T_w_c1_pos)
    
    # 回転と並進を結合し、同次変換行列 T_world_cam を作成
    T_world_cam = concatenate_matrices(T_trans_world_cam, R_world_cam)
    
    # --- 2. ARマーカのポーズ (Camera w.r.t) の変換行列 (T_cam_tag) を作成 ---
    R_cam_tag = quaternion_matrix(raw_quat)
    T_trans_cam_tag = translation_matrix(translation)
    
    # 回転と並進を結合し、同次変換行列 T_cam_tag を作成
    T_cam_tag = concatenate_matrices(T_trans_cam_tag, R_cam_tag)
    
    # --- 3. 変換行列の乗算 (T_world_tag = T_world_cam * T_cam_tag) ---
    
    # ワールド座標系から見たARマーカのポーズを表す同次変換行列を計算
    T_world_tag = np.dot(T_world_cam, T_cam_tag)
    
    # --- 4. 結果を geometry_msgs.msg.Pose 形式で抽出して返す ---
    
    # 位置 (Position) を抽出
    x = T_world_tag[0, 3]
    y = T_world_tag[1, 3]
    z = T_world_tag[2, 3]
    
    # 姿勢 (Orientation) を抽出 (回転行列からクォータニオンに変換)
    from tf.transformations import quaternion_from_matrix
    q_world_tag = quaternion_from_matrix(T_world_tag)
    
    pose_stamped = geometry_msgs.msg.PoseStamped()
    
    # ヘッダー情報を設定
    pose_stamped.header.stamp = rospy.Time.now()
    pose_stamped.header.frame_id = "world" # 親座標系を設定
    
    # ポーズ情報を設定
    pose_stamped.pose.position.x = x
    pose_stamped.pose.position.y = y
    pose_stamped.pose.position.z = z
    pose_stamped.pose.orientation.x = q_world_tag[0]
    pose_stamped.pose.orientation.y = q_world_tag[1]
    pose_stamped.pose.orientation.z = q_world_tag[2]
    pose_stamped.pose.orientation.w = q_world_tag[3]
    
    return pose_stamped

if __name__ == '__main__':
    rospy.init_node("tag_pose_to_world_publisher")
    
    # パブリッシャーを初期化
    pose_pub = rospy.Publisher("/ar_tag/pose_in_world", geometry_msgs.msg.PoseStamped, queue_size=1)
    
    # ファイルは一度読み込むだけで、リアルタイム更新はしない想定（必要に応じてループ処理で定期的に読み直すことも可能）
    tag_pose_world_stamped = calculate_tag_pose_in_world()
    
    if tag_pose_world_stamped is not None:
        # 結果を表示
        print("✨ World座標系から見たARマーカ (ID: 0) のポーズ:")
        print("-" * 40)
        print(tag_pose_world_stamped.pose)
        
        # ROSが実行中である限り、計算したポーズをパブリッシュし続ける
        # 他のノードが起動するのを待つため、1秒間隔でパブリッシュ
        rate = rospy.Rate(1) # 1Hz
        while not rospy.is_shutdown():
            tag_pose_world_stamped.header.stamp = rospy.Time.now() # タイムスタンプを更新
            pose_pub.publish(tag_pose_world_stamped)
            rate.sleep()
    else:
        rospy.logerr("計算されたポーズが存在しないため、パブリッシュをスキップしました。")
