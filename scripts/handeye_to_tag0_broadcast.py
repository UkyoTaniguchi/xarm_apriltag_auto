#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import TransformStamped
import tf.transformations as tft
import math

def broadcast_transform():
    rospy.init_node('camera_to_world_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(15.0)

    # AprilTagから得られた変換情報（cam_2_linkの位置と姿勢）
    translation = (0.04638254214635267, -0.02213720827257412, 0.3361446136878856)
    raw_quat = (0.9881675615207981, -0.04851030912901147, -0.055268871968521996, -0.13459930184653457)

    # 座標系補正（順番：Yaw=0°, Pitch=-90°, Roll=90°）← すべてローカル軸回転
    q_pitch = tft.quaternion_from_euler(0, math.radians(-90), 0)
    q_roll  = tft.quaternion_from_euler(math.radians(-90), 0, 0)

    # 順に乗算： q_correction = q_pitch → q_roll
    q_correction = tft.quaternion_multiply(q_pitch, q_roll)

    # 最終姿勢 = もとの姿勢 × 補正姿勢
    corrected_quat = tft.quaternion_multiply(raw_quat, q_correction)

    while not rospy.is_shutdown():
        br.sendTransform(
            translation,
            corrected_quat,
            rospy.Time.now(),
            "cam_2_link",
            "cam_1_color_optical_frame"
        )
        rate.sleep()

if __name__ == '__main__':
    try:
        broadcast_transform()
    except rospy.ROSInterruptException:
        pass
