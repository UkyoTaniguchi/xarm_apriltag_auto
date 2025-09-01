#!/usr/bin/env python3

import rospy
import tf
import math
from geometry_msgs.msg import TransformStamped

def broadcast_transform():
    rospy.init_node('camera_to_world_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(15.0)

    # roll=0°, pitch=-90°, yaw=180° を“厳密に”指定（ラジアン）
    roll  = 0.0
    pitch = -math.pi / 2.0      # -90°
    yaw   =  math.pi            # 180°
    quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    while not rospy.is_shutdown():
        br.sendTransform(
            (0.06746, -0.0175, 0.0237),   # 平行移動 (x, y, z)
            quat,                         # 回転（クォータニオン）
            rospy.Time.now(),
            "cam_1_link",                 # 子フレーム
            "link_eef"                    # 親フレーム
        )
        rate.sleep()

# テスト用(500mm手前)
def broadcast_transform2():
    rospy.init_node('camera_to_world_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(15.0)

    # roll=0°, pitch=-90°, yaw=180° を“厳密に”指定（ラジアン）
    roll  = 0.0
    pitch = 0.0     # -90°
    yaw   = 0.0     # 180°
    quat = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    while not rospy.is_shutdown():
        br.sendTransform(
            (0.344484, 0.0675, -0.0855),   # 平行移動 (x, y, z)
            quat,                         # 回転（クォータニオン）
            rospy.Time.now(),
            "cam_1_link",                 # 子フレーム
            "world"                    # 親フレーム
        )
        rate.sleep()

if __name__ == '__main__':
    try:
        broadcast_transform2()
    except rospy.ROSInterruptException:
        pass
