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

def _pick_first_match(lines, start_idx):
    pos = ori = None
    for i in range(start_idx, len(lines)):
        if pos is None:
            m = _pos_re.search(lines[i])
            if m:
                pos = tuple(float(m.group(k)) for k in (1, 2, 3))
                continue
        if pos is not None and ori is None:
            m = _ori_re.search(lines[i])
            if m:
                ori = tuple(float(m.group(k)) for k in (1, 2, 3, 4))
                break
    return pos, ori

def _normalize_quat(q):
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if not np.isfinite(n) or n < 1e-8:
        raise ValueError("Invalid quaternion (norm ~ 0 or NaN)")
    return tuple((q / n).tolist())

def _assert_finite(name, arr):
    a = np.asarray(arr, dtype=float)
    if not np.all(np.isfinite(a)):
        raise ValueError(f"{name} contains non-finite values: {a}")

def load_from_tag_pose_file(path, wanted_tag_id):
    if not os.path.exists(path):
        raise FileNotFoundError(f"{path} が見つかりません。")
    with open(path, "r") as f:
        lines = [ln.strip() for ln in f.readlines()]

    # --- Tag ID ブロック ---
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
        raise ValueError("Tag の Position/Orientation を解釈できません。")

    # --- cam_1_color_optical_frame ヘッダ（Tag有無どちらも許容）---
    cam1_idx = None
    for i, ln in enumerate(lines):
        m = _cam1_hdr_re.match(ln)
        if not m:
            continue
        # m.group(1) が None（= Tag表記なし）の場合は採用
        # ある場合は wanted_tag_id と一致したときのみ採用
        if m.group(1) is None or int(m.group(1)) == wanted_tag_id:
            cam1_idx = i
            break
    if cam1_idx is None:
        raise ValueError(f"'cam_1_color_optical_frame ... w.r.t world:' の行が見つかりません。")

    cam1_pos, cam1_quat = _pick_first_match(lines, cam1_idx)
    if cam1_pos is None or cam1_quat is None:
        raise ValueError("cam_1 の Position/Orientation を解釈できません。")

    return translation, raw_quat, np.array(cam1_pos), tuple(cam1_quat)

def compute_transform(tag_file, tag_id):
    translation, raw_quat, T_w_c1_pos, T_w_c1_quat = load_from_tag_pose_file(tag_file, tag_id)

    # 回転補正（ローカル軸：Yaw=0°, Pitch=-90°, Roll=90°）
    q_pitch = tft.quaternion_from_euler(0, math.radians(-90), 0)
    q_roll  = tft.quaternion_from_euler(math.radians(-90), 0, 0)
    q_correction = tft.quaternion_multiply(q_pitch, q_roll)
    corrected_quat = tft.quaternion_multiply(_normalize_quat(raw_quat), _normalize_quat(q_correction))
    corrected_quat = _normalize_quat(corrected_quat)

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
    T_w_c1_quat = _normalize_quat(T_w_c1_quat)
    T_w_ci_quat = tft.quaternion_multiply(T_w_c1_quat, T_c1_ci_quat)
    T_w_ci_quat = _normalize_quat(T_w_ci_quat)

    R_w_c1 = tft.quaternion_matrix(T_w_c1_quat)[:3, :3]
    T_ci_offset_world = R_w_c1 @ T_c1_ci_pos
    T_w_ci_pos = T_w_c1_pos + T_ci_offset_world

    _assert_finite("T_w_ci_pos", T_w_ci_pos)
    _assert_finite("T_w_ci_quat", T_w_ci_quat)

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
