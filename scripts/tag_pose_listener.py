#!/usr/bin/env python3

import rospy
import tf2_ros
from apriltag_ros.msg import AprilTagDetectionArray
from geometry_msgs.msg import TransformStamped

saved = False  # 1回だけ保存するためのフラグ

def save_tag_and_camera_pose(detections_msg):
    global saved
    if saved or len(detections_msg.detections) == 0:
        return

    try:
        tag_info_lines = []
        for detection in detections_msg.detections:
            tag_id = detection.id[0]
            pose = detection.pose.pose.pose
            position = pose.position
            orientation = pose.orientation

            tag_info_lines.append(f"Tag ID: {tag_id}")
            tag_info_lines.append(f"Position: x={position.x}, y={position.y}, z={position.z}")
            tag_info_lines.append(f"Orientation: x={orientation.x}, y={orientation.y}, z={orientation.z}, w={orientation.w}")
            tag_info_lines.append("")

        # カメラの位置姿勢をTFから取得
        tf_buffer = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(tf_buffer)

        rospy.sleep(1.0)  # TFが準備されるまで少し待つ

        transform: TransformStamped = tf_buffer.lookup_transform(
            target_frame='world',
            source_frame='cam_1_color_optical_frame',
            time=rospy.Time(0),
            timeout=rospy.Duration(2.0)
        )

        t = transform.transform.translation
        r = transform.transform.rotation

        tag_info_lines.append("cam_1_color_optical_frame w.r.t world:")
        tag_info_lines.append(f"Position: x={t.x}, y={t.y}, z={t.z}")
        tag_info_lines.append(f"Orientation: x={r.x}, y={r.y}, z={r.z}, w={r.w}")

        with open("/home/ukyo/catkin_ws/src/xarm_apriltag_demo/public/tag_pose.txt", "w") as f:
            f.write("\n".join(tag_info_lines))

        rospy.loginfo("Pose information saved to /tmp/tag_pose.txt")
        saved = True

    except Exception as e:
        rospy.logerr(f"Failed to get transform or write file: {e}")

def listener():
    rospy.init_node('apriltag_listener', anonymous=True)
    rospy.Subscriber('/tag_detections', AprilTagDetectionArray, save_tag_and_camera_pose)
    rospy.spin()

if __name__ == '__main__':
    listener()
