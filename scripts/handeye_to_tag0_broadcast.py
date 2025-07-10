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

    # cam_1_color_optical_frame の world からの位置・姿勢（クォータニオン）
    T_w_c1_pos = np.array([0.7011397474192116, 0.1434457767138928, 0.39845546874782145])
    T_w_c1_quat = [-0.5186274359877653, 0.5049423237374071, -0.49123082288991715, 0.4845111598688641]

    # cam_2_link ← cam_1_color_optical_frame の変換
    T_c1_c2_pos = final_translation
    T_c1_c2_quat = corrected_quat

    # 回転の合成（クォータニオン積）
    T_w_c2_quat = tft.quaternion_multiply(T_w_c1_quat, T_c1_c2_quat)

    # 平行移動の変換：cam_1→cam_2 のベクトルを world の回転で変換
    R_w_c1 = tft.quaternion_matrix(T_w_c1_quat)[:3, :3]
    T_c2_offset_world = R_w_c1 @ T_c1_c2_pos

    # 最終的な平行移動（world→cam_2）
    T_w_c2_pos = T_w_c1_pos + T_c2_offset_world


    while not rospy.is_shutdown():
        br.sendTransform(
            T_w_c2_pos,
            T_w_c2_quat,
            rospy.Time.now(),
            "cam_2_link",
            "world"
        )
        rate.sleep()

if __name__ == '__main__':
    try:
        broadcast_transform()
    except rospy.ROSInterruptException:
        pass