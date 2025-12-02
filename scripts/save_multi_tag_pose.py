#!/usr/bin/env python3
import rospy
import rospkg
import os
import tf2_ros
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
from handeye_to_multi_tag_broadcast import broadcast_transform

tag_pose = {}
seen_tags = set()  # すでに broadcast 済みの Tag ID を記録
tf_buffer = None
listener = None
EPS = 1e-4

def is_pose_changed(old_pose, new_pose):
    for old, new in zip(old_pose, new_pose):
        if abs(old - new) > EPS:
            return True
    return False


def save_tag_and_camera_pose(detections_msg):
    global tag_pose, tf_buffer, seen_tags
    if len(detections_msg.detections) == 0:
        return

    updated = False
    tag_info_lines = []
    new_detected_tags = set()

    for detection in detections_msg.detections:
        tag_id = detection.id[0]
        pose = detection.pose.pose.pose
        p = pose.position
        q = pose.orientation
        new_tag_pose = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)

        # --- カメラ座標取得 ---
        try:
            transform: TransformStamped = tf_buffer.lookup_transform(
                target_frame='world',
                source_frame='cam_1_color_optical_frame',
                time=rospy.Time(0),
                timeout=rospy.Duration(2.0)
            )
            t = transform.transform.translation
            r = transform.transform.rotation
            new_cam_pose = (t.x, t.y, t.z, r.x, r.y, r.z, r.w)
        except Exception as e:
            rospy.logwarn(f"Transform not found for Tag ID {tag_id}: {e}")
            continue

        if tag_id not in tag_pose:
            rospy.loginfo(f"New Tag ID {tag_id} detected, saving.")
            tag_pose[tag_id] = {"tag_pose": new_tag_pose, "camera_pose": new_cam_pose}
            updated = True
            new_detected_tags.add(tag_id)
        else:
            old_tag_pose = tag_pose[tag_id]["tag_pose"]
            old_cam_pose = tag_pose[tag_id]["camera_pose"]
            if is_pose_changed(old_tag_pose, new_tag_pose) or is_pose_changed(old_cam_pose, new_cam_pose):
                tag_pose[tag_id]["tag_pose"] = new_tag_pose
                tag_pose[tag_id]["camera_pose"] = new_cam_pose
                updated = True

    if not updated:
        return

    # --- ファイル保存 ---
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path("xarm_apriltag_auto")  # ← 実際のパッケージ名に置換
    save_path = os.path.join(pkg_path, "public", "tag_pose.txt")

    os.makedirs(os.path.dirname(save_path), exist_ok=True)  # 念のため
    for tag_id, data in tag_pose.items():
        px, py, pz, qx, qy, qz, qw = data["tag_pose"]
        cx, cy, cz, rx, ry, rz, rw = data["camera_pose"]
        tag_info_lines += [
            f"Tag ID: {tag_id}",
            f"Position: x={px}, y={py}, z={pz}",
            f"Orientation: x={qx}, y={qy}, z={qz}, w={qw}",
            "",
            f"cam_1_color_optical_frame (Tag {tag_id}) w.r.t world:",
            f"Position: x={cx}, y={cy}, z={cz}",
            f"Orientation: x={rx}, y={ry}, z={rz}, w={rw}",
            "",
        ]

    with open(save_path, "w") as f:
        f.write("\n".join(tag_info_lines))

    # print(f"Pose information saved to {save_path}, total tags: {len(tag_pose)}")

    # # --- 新規タグを検出したときのみ TF ブロードキャスト ---
    # unseen = new_detected_tags - seen_tags
    # if unseen:
    #     rospy.loginfo(f"Broadcasting for newly detected tags: {sorted(list(unseen))}")
    #     broadcast_transform()
    #     seen_tags.update(unseen)


def listener():
    global tf_buffer, listener
    rospy.init_node('apriltag_listener', anonymous=True)
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, save_tag_and_camera_pose)
    rospy.spin()


if __name__ == '__main__':
    listener()
