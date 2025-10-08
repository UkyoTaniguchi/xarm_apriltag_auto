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

# デフォルトのタグファイルパス
TAG_FILE_DEFAULT = "/home/robot/nishidalab_ws/src/3_utils/xarm_apriltag_auto/public/tag_pose.txt"

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

    return translation, raw_quat, np.array(cam1_pos), tuple(cam1_quat)

def broadcast_transform():
    rospy.init_node('camera_to_world_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(15.0)

    # パラメータ（必要なら rosparam で上書き可能）
    tag_file = rospy.get_param("~tag_file", TAG_FILE_DEFAULT)
    tag_id   = rospy.get_param("~tag_id", 0)

    # --- 1) ファイルから数値を取り込む ---
    translation, raw_quat, T_w_c1_pos, T_w_c1_quat = load_from_tag_pose_file(tag_file, tag_id)

    # --- 2) あなたの既存ロジック：姿勢補正など ---
    # 座標系補正（順番：Yaw=0°, Pitch=-90°, Roll=90°）← すべてローカル軸回転
    q_pitch = tft.quaternion_from_euler(0, math.radians(-90), 0)
    q_roll  = tft.quaternion_from_euler(math.radians(-90), 0, 0)

    # q_correction = q_pitch → q_roll（右から適用）
    q_correction = tft.quaternion_multiply(q_pitch, q_roll)

    # 最終姿勢 = もとの姿勢 × 補正姿勢
    corrected_quat = tft.quaternion_multiply(raw_quat, q_correction)

    local_offset = np.array([0.0042, 0.0175, -0.0525])  # cam_2_link基準の平行移動

    # corrected_quat の回転行列でオフセットを変換
    rot_matrix = tft.quaternion_matrix(corrected_quat)[:3, :3]
    offset_global = rot_matrix @ local_offset

    # 移動後の位置（型を合わせる）
    final_translation = np.array(translation) + offset_global

    # cam_2_link ← cam_1_color_optical_frame の変換
    T_c1_c2_pos  = final_translation
    T_c1_c2_quat = corrected_quat

    # 回転の合成（クォータニオン積）
    T_w_c2_quat = tft.quaternion_multiply(T_w_c1_quat, T_c1_c2_quat)

    # 平行移動の変換：cam_1→cam_2 のベクトルを world の回転で変換
    R_w_c1 = tft.quaternion_matrix(T_w_c1_quat)[:3, :3]
    T_c2_offset_world = R_w_c1 @ T_c1_c2_pos

    # 最終的な平行移動（world→cam_2）
    T_w_c2_pos = T_w_c1_pos + T_c2_offset_world

    rospy.loginfo(f"[OK] Loaded from {tag_file} (Tag ID: {tag_id})")
    rospy.loginfo(f"T_w_c2_pos = {T_w_c2_pos}")
    rospy.loginfo(f"T_w_c2_quat = {T_w_c2_quat}")

    while not rospy.is_shutdown():
        br.sendTransform(
            T_w_c2_pos,
            T_w_c2_quat,
            rospy.Time.now(),
            "cam_2_link",
            "world"
        )
        rate.sleep()

if __name__ == '__main__':
    try:
        broadcast_transform()
    except rospy.ROSInterruptException:
        pass
