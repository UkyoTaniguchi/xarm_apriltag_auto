#!/usr/bin/env python3
import rospy
import tf2_ros
from apriltag_ros.msg import AprilTagDetectionArray
from std_srvs.srv import Trigger, TriggerResponse

def get_save_path(tag_id):
    return f"/home/robot/nishidalab_ws/src/3_utils/xarm_apriltag_auto/public/tag_{tag_id}_pose.txt"

def handle_capture(req):
    rospy.loginfo("=== Capturing AprilTag + Camera Pose ===")

    try:
        msg = rospy.wait_for_message("/tag_detections", AprilTagDetectionArray, timeout=5.0)
    except Exception:
        return TriggerResponse(success=False, message="タグ検出メッセージを取得できません")

    if len(msg.detections) == 0:
        return TriggerResponse(success=False, message="AprilTag が見つかりません")

    lines = []

    # Tag の姿勢を記録
    for det in msg.detections:
        tag_id = det.id[0]
        pose = det.pose.pose.pose
        p = pose.position
        o = pose.orientation
        lines.append(f"Tag ID: {tag_id}")
        lines.append(f"Position: x={p.x}, y={p.y}, z={p.z}")
        lines.append(f"Orientation: x={o.x}, y={o.y}, z={o.z}, w={o.w}")
        lines.append("")

    # Camera TF を取得
    try:
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)
        rospy.sleep(0.5)

        trans = tf_buffer.lookup_transform(
            "world",
            "cam_1_color_optical_frame",
            rospy.Time(0),
            rospy.Duration(2.0)
        )

        t = trans.transform.translation
        r = trans.transform.rotation
        lines.append("cam_1_color_optical_frame w.r.t world:")
        lines.append(f"Position: x={t.x}, y={t.y}, z={t.z}")
        lines.append(f"Orientation: x={r.x}, y={r.y}, z={r.z}, w={r.w}")

    except Exception as e:
        return TriggerResponse(success=False, message=f"TF取得失敗: {e}")

    save_path = get_save_path(tag_id)

    # 保存
    with open(save_path, "w") as f:
        f.write("\n".join(lines))
    # ★★★ 保存した内容をログに出す（追加部分） ★★★
    rospy.loginfo("==== Saved Content ====")
    for ln in lines:
        rospy.loginfo(ln)
    rospy.loginfo("==== End Content ====")
    rospy.loginfo(f"{save_path} にタグとカメラの姿勢を保存しました。")

    return TriggerResponse(success=True, message="Tag & Camera pose saved")

def main():
    rospy.init_node("capture_tag_pose_server")
    srv = rospy.Service("/capture_tag_pose", Trigger, handle_capture)
    rospy.loginfo("Service /capture_tag_pose ready.")
    rospy.spin()

if __name__ == "__main__":
    main()
