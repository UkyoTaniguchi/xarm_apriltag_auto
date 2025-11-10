#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
import tf2_ros
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped

SAVE_PATH = "/home/ukyo/catkin_ws/src/xarm_apriltag_demo/public/tag_pose.txt"
TARGET_FRAME = "world"
CAM_FRAME = "cam_1_color_optical_frame"   # 必要なら param で上書き可

# 検出ごとの保持領域
seen = set()   # {0,1}
stash = {
    # 0: {"tag_lines":[...], "cam_lines":[...], "time": rospy.Time}
    # 1: {...}
}
written = False

# tf2 は1つだけ
tf_buffer = None
tf_listener = None

def ensure_dir(path):
    d = os.path.dirname(path)
    if d and not os.path.exists(d):
        os.makedirs(d, exist_ok=True)

def fmt_pose(prefix, p, q):
    return [
        f"{prefix} Position: x={p.x:.9f}, y={p.y:.9f}, z={p.z:.9f}",
        f"{prefix} Orientation: x={q.x:.9f}, y={q.y:.9f}, z={q.z:.9f}, w={q.w:.9f}",
    ]

def lookup_cam_pose_at(stamp, target_frame, cam_frame):
    """stamp 時刻の world→camera を取りに行き、だめなら最新にフォールバック"""
    try:
        tr: TransformStamped = tf_buffer.lookup_transform(
            target_frame=target_frame,
            source_frame=cam_frame,
            time=stamp,
            timeout=rospy.Duration(0.5)
        )
        return tr
    except Exception as e1:
        rospy.logwarn_throttle(2.0, f"TF({target_frame}<-{cam_frame}) at {stamp.to_sec():.3f} failed: {e1}. fallback to latest.")
        try:
            tr: TransformStamped = tf_buffer.lookup_transform(
                target_frame=target_frame,
                source_frame=cam_frame,
                time=rospy.Time(0),
                timeout=rospy.Duration(0.5)
            )
            return tr
        except Exception as e2:
            rospy.logwarn_throttle(2.0, f"TF latest fallback failed: {e2}")
            return None

def cb(msg: AprilTagDetectionArray):
    global written
    if written:
        return
    if not msg.detections:
        return

    target_frame = rospy.get_param("~target_frame", TARGET_FRAME)
    cam_frame = rospy.get_param("~camera_frame", CAM_FRAME)
    stamp = msg.header.stamp

    # このメッセージ時刻での camera pose（world基準）を取得
    cam_tr = lookup_cam_pose_at(stamp, target_frame, cam_frame)
    cam_lines = []
    if cam_tr is not None:
        t = cam_tr.transform.translation
        r = cam_tr.transform.rotation
        cam_lines.append(f"{cam_frame} w.r.t {target_frame}:")
        cam_lines += fmt_pose("camera:", t, r)
        cam_lines.append("")

    # 今回のメッセージで 0/1 を初検出したものだけ保存
    for det in msg.detections:
        if not det.id:
            continue
        tid = det.id[0]
        if tid not in (0, 1):
            continue
        if tid in seen:
            continue  # 既に確保済み

        pose = det.pose.pose.pose
        p, q = pose.position, pose.orientation

        tag_lines = [
            f"Tag ID: {tid}",
            f"frame_id: {msg.header.frame_id}",
            f"time_sec: {stamp.secs}.{stamp.nsecs:09d}",
            f"tag w.r.t camera:",
            f"  Position: x={p.x:.9f}, y={p.y:.9f}, z={p.z:.9f}",
            f"  Orientation: x={q.x:.9f}, y={q.y:.9f}, z={q.z:.9f}, w={q.w:.9f}",
            ""
        ]

        stash[tid] = {
            "tag_lines": tag_lines,
            "cam_lines": cam_lines[:] if cam_lines else ["(camera pose unavailable)\n"],
            "time": stamp
        }
        seen.add(tid)

    # 2つそろったら書き出し（1回だけ）
    if (0 in seen) and (1 in seen) and not written:
        lines = []
        lines.append("=" * 60)
        lines.append("First detection snapshot for IDs 0 and 1")
        lines.append(f"target_frame: {target_frame}")
        lines.append(f"camera_frame: {cam_frame}")
        lines.append("")

        # ID順で出力
        for tid in (0, 1):
            lines.append("-" * 40)
            lines.append(f"[ID {tid}] detection")
            lines += stash[tid]["tag_lines"]
            lines += stash[tid]["cam_lines"]

        ensure_dir(SAVE_PATH)
        with open(SAVE_PATH, "w") as f:
            f.write("\n".join(lines) + "\n")

        written = True
        rospy.loginfo(f"Pose information saved to {SAVE_PATH}")

def main():
    global tf_buffer, tf_listener
    rospy.init_node("apriltag_pair_with_cam_tf", anonymous=True)

    # tf 初期化
    tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # 少し待機してから購読開始（tfツリー起動猶予）
    rospy.sleep(0.5)

    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, cb, queue_size=10)
    rospy.loginfo("apriltag_pair_with_cam_tf started.")
    rospy.spin()

if __name__ == "__main__":
    main()
