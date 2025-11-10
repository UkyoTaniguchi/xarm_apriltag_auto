#!/usr/bin/env python3
# world座標系から見たARマーカ（ID=0/1）のポーズを2本出力するノード

import rospy
import os
import re
import numpy as np
import geometry_msgs.msg
from tf.transformations import (
    quaternion_matrix, quaternion_from_matrix,
    translation_matrix, concatenate_matrices
)

TAG_FILE_DEFAULT = "/home/ukyo/catkin_ws/src/xarm_apriltag_demo/public/tag_pose.txt"

_pos_re = re.compile(r"(?:^|\s)Position:\s*x=([-\d\.eE]+),\s*y=([-\d\.eE]+),\s*z=([-\d\.eE]+)")
_ori_re = re.compile(r"(?:^|\s)Orientation:\s*x=([-\d\.eE]+),\s*y=([-\d\.eE]+),\s*z=([-\d\.eE]+),\s*w=([-\d\.eE]+)")
_section_sep_re = re.compile(r"^-{5,}$")
_cam_header_re = re.compile(r"^cam_1_color_optical_frame w\.r\.t world:\s*$")
_id_header_re  = re.compile(r"^\[ID\s+(\d+)\]\s+detection\s*$")
_tagid_line_re = re.compile(r"^Tag ID:\s*(\d+)\s*$")

def _pick_first_pose(lines, start_idx):
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
                ori = tuple(float(m.group(k)) for k in (1,2,3,4))
                break
    return pos, ori

def _find_block_for_tag(lines, wanted_tag_id):
    i = 0
    while i < len(lines):
        if _section_sep_re.match(lines[i]):
            i += 1
            if i < len(lines):
                m = _id_header_re.match(lines[i])
                if m and int(m.group(1)) == wanted_tag_id:
                    return i
        m = _id_header_re.match(lines[i])
        if m and int(m.group(1)) == wanted_tag_id:
            return i
        i += 1
    return None

def _find_next(lines, start_idx, pred, stop_pred=None):
    for i in range(start_idx, len(lines)):
        if stop_pred and stop_pred(lines[i]):
            return None
        if pred(lines[i]):
            return i
    return None

def load_from_tag_pose_file(path, wanted_tag_id):
    if not os.path.exists(path):
        raise FileNotFoundError(path)
    with open(path, "r") as f:
        lines = [ln.rstrip() for ln in f.readlines()]

    id_idx = _find_block_for_tag(lines, wanted_tag_id)
    if id_idx is None:
        # 後方互換（Tag ID: Xのみ）
        tag_idx = next((i for i,ln in enumerate(lines)
                        if _tagid_line_re.match(ln) and int(_tagid_line_re.match(ln).group(1))==wanted_tag_id), None)
        if tag_idx is None:
            return None
        tag_pos, tag_quat = _pick_first_pose(lines, tag_idx)
        if not tag_pos or not tag_quat: return None
        cam_hdr_idx = _find_next(lines, tag_idx, lambda s: _cam_header_re.match(s) is not None)
        if cam_hdr_idx is None: return None
        cam_pos, cam_quat = _pick_first_pose(lines, cam_hdr_idx)
        if not cam_pos or not cam_quat: return None
        return tag_pos, tag_quat, cam_pos, cam_quat

    # 新形式
    tag_block_start = _find_next(lines, id_idx, lambda s: s.strip().startswith("tag w.r.t camera:"))
    if tag_block_start is None: tag_block_start = id_idx
    tag_pos, tag_quat = _pick_first_pose(lines, tag_block_start)
    if not tag_pos or not tag_quat: return None

    cam_hdr_idx = _find_next(
        lines, id_idx,
        pred=lambda s: _cam_header_re.match(s) is not None,
        stop_pred=lambda s: _section_sep_re.match(s) is not None
    )
    if cam_hdr_idx is None:
        cam_hdr_idx = _find_next(lines, id_idx, lambda s: _cam_header_re.match(s) is not None)
        if cam_hdr_idx is None: return None
    cam_pos, cam_quat = _pick_first_pose(lines, cam_hdr_idx)
    if not cam_pos or not cam_quat: return None
    return tag_pos, tag_quat, cam_pos, cam_quat

def calc_world_pose(tag_pos, tag_quat, cam_pos, cam_quat):
    # world -> camera
    T_world_cam = concatenate_matrices(translation_matrix(cam_pos), quaternion_matrix(cam_quat))
    # camera -> tag
    T_cam_tag   = concatenate_matrices(translation_matrix(tag_pos), quaternion_matrix(tag_quat))
    # world -> tag
    T_world_tag = np.dot(T_world_cam, T_cam_tag)
    q = quaternion_from_matrix(T_world_tag)
    p = T_world_tag[:3, 3]
    ps = geometry_msgs.msg.PoseStamped()
    ps.header.stamp = rospy.Time.now()
    ps.header.frame_id = "world"
    ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, p)
    ps.pose.orientation.x, ps.pose.orientation.y, ps.pose.orientation.z, ps.pose.orientation.w = map(float, q)
    return ps

if __name__ == '__main__':
    rospy.init_node("tag_pose_to_world_publisher_dual")

    tag_file = rospy.get_param("~tag_file", TAG_FILE_DEFAULT)
    ids = rospy.get_param("~tag_ids", [0, 1])  # 必要なら変更可

    pub0 = rospy.Publisher("/ar_tag/id0/pose_in_world", geometry_msgs.msg.PoseStamped, queue_size=1)
    pub1 = rospy.Publisher("/ar_tag/id1/pose_in_world", geometry_msgs.msg.PoseStamped, queue_size=1)

    # 読み込み（1回）
    parsed = {}
    for tid in ids:
        res = load_from_tag_pose_file(tag_file, tid)
        if res is None:
            rospy.logwarn(f"ID {tid}: tag_pose.txt から必要情報を取得できませんでした。")
            continue
        tag_pos, tag_quat, cam_pos, cam_quat = res
        ps = calc_world_pose(tag_pos, tag_quat, cam_pos, cam_quat)
        parsed[tid] = ps
        print(f"[ID {tid}] world pose:")
        print(f"  position: x={ps.pose.position.x:.6f}, y={ps.pose.position.y:.6f}, z={ps.pose.position.z:.6f}")
        print(f"  orientation: x={ps.pose.orientation.x:.6f}, y={ps.pose.orientation.y:.6f}, z={ps.pose.orientation.z:.6f}, w={ps.pose.orientation.w:.6f}")

    if not parsed:
        rospy.logerr("有効なIDのポーズが取得できなかったため、終了します。")
        raise SystemExit(1)

    rate = rospy.Rate(1) 
    while not rospy.is_shutdown():
        now = rospy.Time.now()
        if 0 in parsed:
            parsed[0].header.stamp = now
            pub0.publish(parsed[0])
        if 1 in parsed:
            parsed[1].header.stamp = now
            pub1.publish(parsed[1])
        rate.sleep()
