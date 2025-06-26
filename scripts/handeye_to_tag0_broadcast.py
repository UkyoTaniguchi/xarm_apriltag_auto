#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import TransformStamped
import tf.transformations as tft
import math
import numpy as np

def broadcast_transform():
    rospy.init_node('camera_to_world_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(15.0)

    # AprilTagから得られた変換情報（cam_2_linkの位置と姿勢）
    translation = (0.04613161491029728, -0.06653980868776793, 0.4563454475593456)
    raw_quat = (0.9961804529033943, -0.02621040542367576, -0.06041047494973308, -0.05734190803472116)

    # 座標系補正（順番：Yaw=0°, Pitch=-90°, Roll=90°）← すべてローカル軸回転
    q_pitch = tft.quaternion_from_euler(0, math.radians(-90), 0)
    q_roll  = tft.quaternion_from_euler(math.radians(-90), 0, 0)

    # 順に乗算： q_correction = q_pitch → q_roll
    q_correction = tft.quaternion_multiply(q_pitch, q_roll)

    # 最終姿勢 = もとの姿勢 × 補正姿勢
    corrected_quat = tft.quaternion_multiply(raw_quat, q_correction)

    local_offset = np.array([-0.0042, 0.0175, -0.0525])  # ← cam_2_link基準の平行移動

    # corrected_quat の回転行列でオフセットを変換
    rot_matrix = tft.quaternion_matrix(corrected_quat)[:3, :3]
    offset_global = rot_matrix @ local_offset

    # 移動後の位置
    final_translation = translation + offset_global

    while not rospy.is_shutdown():
        br.sendTransform(
            final_translation,
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