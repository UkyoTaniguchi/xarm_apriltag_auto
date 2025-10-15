#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# world 座標系を親として、tag_id=0,1,2 の各AprilTagに対応するカメラ2〜4の姿勢をpublish

import rospy
import tf
import math
import numpy as np
import re
import os
import tf.transformations as tft

TAG_FILE_DEFAULT = "/home/robot/nishidalab_ws/src/3_utils/xarm_apriltag_auto/public/tag_pose.txt"

_pos_re = re.compile(r"Position:\s*x=([-\d\.eE]+),\s*y=([-\d\.eE]+),\s*z=([-\d\.eE]+)")
_ori_re = re.compile(r"Orientation:\s*x=([-\d\.eE]+),\s*y=([-\d\.eE]+),\s*z=([-\d\.eE]+),\s*w=([-\d\.eE]+)")
# 「(Tag N)」の有無を両方許容
_cam1_hdr_re = re.compile(r"^cam_1_color_optical_frame(?:\s*\(Tag\s*(\d+)\))?\s*w\.r\.t\s*world:")

# 位置・姿勢の変化を検出するための閾値
def _pick_first_match(text_lines, start_index):
    # Position/Orientation の最初のマッチを探す
    position = orientation = None
    # 各行を順にチェック
    for line_index in range(start_index, len(text_lines)):
        # position 未取得なら探す
        if position is None:
            position_match = _pos_re.search(text_lines[line_index])
            if position_match:
                position = tuple(float(position_match.group(group_id)) for group_id in (1, 2, 3))
                continue
        if position is not None and orientation is None:
            orientation_match = _ori_re.search(text_lines[line_index])
            if orientation_match:
                orientation = tuple(float(orientation_match.group(group_id)) for group_id in (1, 2, 3, 4))
                break
    return position, orientation

def _normalize_quaternion(quaternion):
    """クォータニオンを正規化"""
    quaternion_array = np.asarray(quaternion, dtype=float)
    norm_value = np.linalg.norm(quaternion_array)
    if not np.isfinite(norm_value) or norm_value < 1e-8:
        raise ValueError("Invalid quaternion (norm ~ 0 or NaN)")
    return tuple((quaternion_array / norm_value).tolist())


def _assert_finite_values(array_name, array_values):
    """NaNやinfが含まれていないか確認"""
    numeric_array = np.asarray(array_values, dtype=float)
    if not np.all(np.isfinite(numeric_array)):
        raise ValueError(f"{array_name} contains non-finite values: {numeric_array}")


def load_from_tag_pose_file(file_path, target_tag_id):
    """tag_pose.txtから指定Tag IDとcam_1の姿勢を読み取る"""
    if not os.path.exists(file_path):
        raise FileNotFoundError(f"{file_path} が見つかりません。")

    with open(file_path, "r") as file:
        all_lines = [line.strip() for line in file.readlines()]

    # --- Tag ID ブロックを検索 ---
    tag_line_index = None
    for line_index, current_line in enumerate(all_lines):
        if current_line.startswith("Tag ID:"):
            try:
                tag_id_value = int(current_line.split(":")[1].strip())
                if tag_id_value == target_tag_id:
                    tag_line_index = line_index
                    break
            except Exception:
                pass
    if tag_line_index is None:
        raise ValueError(f"Tag ID: {target_tag_id} の記述が見つかりません。")

    # Tagの位置・姿勢抽出
    tag_translation, tag_quaternion_raw = _pick_first_match(all_lines, tag_line_index)
    if tag_translation is None or tag_quaternion_raw is None:
        raise ValueError("Tag の Position/Orientation を解釈できません。")

    # --- cam_1_color_optical_frame のブロックを検索 ---
    cam1_line_index = None
    for line_index, current_line in enumerate(all_lines):
        cam1_match = _cam1_hdr_re.match(current_line)
        if not cam1_match:
            continue
        # group(1) が None の場合は無条件採用
        # group(1) に値があれば target_tag_id と一致したときのみ採用
        if cam1_match.group(1) is None or int(cam1_match.group(1)) == target_tag_id:
            cam1_line_index = line_index
            break
    if cam1_line_index is None:
        raise ValueError("'cam_1_color_optical_frame ... w.r.t world:' の行が見つかりません。")

    # cam_1 の位置・姿勢抽出
    cam1_translation, cam1_quaternion = _pick_first_match(all_lines, cam1_line_index)
    if cam1_translation is None or cam1_quaternion is None:
        raise ValueError("cam_1 の Position/Orientation を解釈できません。")

    return (
        tag_translation,
        tag_quaternion_raw,
        np.array(cam1_translation),
        tuple(cam1_quaternion),
    )


def compute_transform(tag_file, tag_id):
    translation, raw_quat, T_w_c1_pos, T_w_c1_quat = load_from_tag_pose_file(tag_file, tag_id)

    # 回転補正（ローカル軸：Yaw=0°, Pitch=-90°, Roll=90°）
    q_pitch = tft.quaternion_from_euler(0, math.radians(-90), 0)
    q_roll  = tft.quaternion_from_euler(math.radians(-90), 0, 0)
    q_correction = tft.quaternion_multiply(q_pitch, q_roll)
    corrected_quat = tft.quaternion_multiply(_normalize_quaternion(raw_quat), _normalize_quaternion(q_correction))
    corrected_quat = _normalize_quaternion(corrected_quat)

    # オフセット（必要に応じて個別化）
    offsets = {
        0: np.array([0.0042, 0.0175, -0.0525]),
        1: np.array([0.0042, 0.0175, -0.0525]),
        2: np.array([0.0042, 0.0175, -0.0525]),
    }
    local_offset = offsets.get(tag_id, np.zeros(3))

    rot_matrix = tft.quaternion_matrix(corrected_quat)[:3, :3]
    offset_global = rot_matrix @ local_offset
    final_translation = np.array(translation) + offset_global

    # cam_i_link ← cam_1_color_optical_frame
    T_c1_ci_pos  = final_translation
    T_c1_ci_quat = corrected_quat

    # world → cam_i
    T_w_c1_quat = _normalize_quaternion(T_w_c1_quat)
    T_w_ci_quat = tft.quaternion_multiply(T_w_c1_quat, T_c1_ci_quat)
    T_w_ci_quat = _normalize_quaternion(T_w_ci_quat)

    R_w_c1 = tft.quaternion_matrix(T_w_c1_quat)[:3, :3]
    T_ci_offset_world = R_w_c1 @ T_c1_ci_pos
    T_w_ci_pos = T_w_c1_pos + T_ci_offset_world

    _assert_finite_values("T_w_ci_pos", T_w_ci_pos)
    _assert_finite_values("T_w_ci_quat", T_w_ci_quat)

    return T_w_ci_pos, T_w_ci_quat

def broadcast_transform():
    rospy.init_node("multi_camera_broadcaster")
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(15.0)

    tag_file = rospy.get_param("~tag_file", TAG_FILE_DEFAULT)
    tag_ids  = rospy.get_param("~tag_ids", [0, 1, 2])

    rospy.loginfo(f"[INFO] Broadcasting {len(tag_ids)} camera transforms")

    transforms = {}
    for tag_id in tag_ids:
        try:
            T_pos, T_quat = compute_transform(tag_file, tag_id)
            ci = tag_id + 2  # Tag0→cam2, Tag1→cam3, Tag2→cam4
            transforms[ci] = (T_pos, T_quat)
            rospy.loginfo(f"[OK] Tag {tag_id} → cam_{ci}_link  pos={T_pos}, quat={T_quat}")
        except Exception as e:
            rospy.logwarn(f"[WARN] Tag {tag_id}: {e}")

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        for ci, (pos, quat) in transforms.items():
            br.sendTransform(pos, quat, now, f"cam_{ci}_link", "world")
        rate.sleep()

if __name__ == "__main__":
    try:
        broadcast_transform()
    except rospy.ROSInterruptException:
        pass
