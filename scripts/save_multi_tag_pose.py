#!/usr/bin/env python3
import rospy
import tf2_ros
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped
from handeye_to_multi_tag_broadcast import broadcast_transform
import math

tag_pose = {}
tf_buffer = None
listener = None

# --- 浮動小数比較の許容誤差 ---
EPS = 1e-4

def is_pose_changed(old_pose, new_pose):
    """位置・姿勢が有意に変化しているか判定"""
    for old, new in zip(old_pose, new_pose):
        if abs(old - new) > EPS:
            return True
    return False

def save_tag_and_camera_pose(detections_msg):
    global tag_pose, tf_buffer
    if len(detections_msg.detections) == 0:
        return

    updated = False
    tag_info_lines = []

    # --- 各タグについて処理 ---
    for detection in detections_msg.detections:
        # -- タグの位置・姿勢取得 ---
        tag_id = detection.id[0]
        pose = detection.pose.pose.pose
        p = pose.position
        q = pose.orientation
        new_tag_pose = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)

        # --- カメラ座標取得 ---
        try:
            # tf2_rosを使って座標変換を取得
            transform: TransformStamped = tf_buffer.lookup_transform(
                # カメラ座標系からワールド座標系へ変換
                target_frame='world',
                source_frame='cam_1_color_optical_frame', # カメラ1の座標系
                # 時間指定
                time=rospy.Time(0),
                # タイムアウト設定
                timeout=rospy.Duration(2.0)
            )
            # 座標変換の結果を取得
            t = transform.transform.translation
            r = transform.transform.rotation
            # カメラの位置・姿勢を取得
            new_cam_pose = (t.x, t.y, t.z, r.x, r.y, r.z, r.w)
        except Exception as e:
            rospy.logwarn(f"Transform not found for Tag ID {tag_id}: {e}")
            continue

        # --- 新規または変化があったときのみ更新 ---
        if tag_id not in tag_pose:
            # 新規タグの情報を保存
            rospy.loginfo(f"New Tag ID {tag_id} detected, saving.")
            # タグの位置・姿勢を保存
            tag_pose[tag_id] = {
                "tag_pose": new_tag_pose,
                "camera_pose": new_cam_pose,
            }
            updated = True
        else:
            # 既存タグの位置・姿勢が変化しているか確認
            old_tag_pose = tag_pose[tag_id]["tag_pose"]
            old_cam_pose = tag_pose[tag_id]["camera_pose"]
            # tag_poseまたはcamera_poseが変化していたら更新
            if is_pose_changed(old_tag_pose, new_tag_pose) or is_pose_changed(old_cam_pose, new_cam_pose):
                # rospy.loginfo(f"Tag ID {tag_id} pose changed, updating.")
                tag_pose[tag_id]["tag_pose"] = new_tag_pose
                tag_pose[tag_id]["camera_pose"] = new_cam_pose
                updated = True
            else:
                # rospy.loginfo(f"Tag ID {tag_id} unchanged, skipping update.")
                continue

    # --- 変化なしなら保存しない ---
    if not updated:
        return

    # --- ファイル出力 ---
    for tag_id, data in tag_pose.items():
        # タグの位置・姿勢とカメラの位置・姿勢を取得
        px, py, pz, qx, qy, qz, qw = data["tag_pose"]
        cx, cy, cz, rx, ry, rz, rw = data["camera_pose"]

        # --- ファイルに書き込む内容を作成 ---
        tag_info_lines.append(f"Tag ID: {tag_id}")
        tag_info_lines.append(f"Position: x={px}, y={py}, z={pz}")
        tag_info_lines.append(f"Orientation: x={qx}, y={qy}, z={qz}, w={qw}")
        tag_info_lines.append("")
        tag_info_lines.append(f"cam_1_color_optical_frame (Tag {tag_id}) w.r.t world:")
        tag_info_lines.append(f"Position: x={cx}, y={cy}, z={cz}")
        tag_info_lines.append(f"Orientation: x={rx}, y={ry}, z={rz}, w={rw}")
        tag_info_lines.append("")

    # --- ファイルに保存 ---
    save_path = "src/3_utils/xarm_apriltag_auto/public/tag_pose.txt"
    with open(save_path, "w") as f:
        f.write("\n".join(tag_info_lines))

    # rospy.loginfo(f"Pose information updated. Total tags: {len(tag_pose_dict)}")
    #　各カメラもworld座標系に変換
    broadcast_transform()
    print(f"Pose information saved to {save_path}, total tags: {len(tag_pose)}")
    



def listener():
    # --- ROSノード初期化 ---
    global tf_buffer, listener
    rospy.init_node('apriltag_listener', anonymous=True)
    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, save_tag_and_camera_pose)
    rospy.spin()

if __name__ == '__main__':
    listener()
