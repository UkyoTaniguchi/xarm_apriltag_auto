#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Vector3
import tf
import tf.transformations as tft
import math

class EEFCorrectionBroadcaster:
    def __init__(self):
        # パラメータ
        self.parent_frame = rospy.get_param("~parent_frame", "link_eef")
        self.child_frame  = rospy.get_param("~child_frame",  "link_eef_fixed")
        xyz_list          = rospy.get_param("~xyz", [0.0, 0.0, 0.0])        # [m]
        rpy_deg_list      = rospy.get_param("~rpy_deg", [0.0, 0.0, 3.000916])    # [deg]
        self.rate_hz      = rospy.get_param("~rate", 30)

        # 状態（内部保持）
        self.tx, self.ty, self.tz = xyz_list
        self.roll_deg, self.pitch_deg, self.yaw_deg = rpy_deg_list

        self.br = tf.TransformBroadcaster()

        # 角度・平行移動をライブ更新するトピック
        rospy.Subscriber("~xyz", Vector3, self._on_xyz)            # [m]
        rospy.Subscriber("~rpy_deg", Vector3, self._on_rpy_deg)    # [deg]

        rospy.loginfo("EEF correction broadcaster started.")
        rospy.loginfo("parent=%s, child=%s", self.parent_frame, self.child_frame)
        rospy.loginfo("init xyz=%.4f, %.4f, %.4f [m]", self.tx, self.ty, self.tz)
        rospy.loginfo("init rpy=%.3f°, %.3f°, %.3f°", self.roll_deg, self.pitch_deg, self.yaw_deg)

    def _on_xyz(self, msg: Vector3):
        self.tx, self.ty, self.tz = msg.x, msg.y, msg.z
        rospy.loginfo("Update xyz -> [%.4f, %.4f, %.4f] m", self.tx, self.ty, self.tz)

    def _on_rpy_deg(self, msg: Vector3):
        self.roll_deg, self.pitch_deg, self.yaw_deg = msg.x, msg.y, msg.z
        rospy.loginfo("Update rpy(deg) -> [R=%.3f, P=%.3f, Y=%.3f]°",
                      self.roll_deg, self.pitch_deg, self.yaw_deg)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            # deg -> rad
            r = math.radians(self.roll_deg)
            p = math.radians(self.pitch_deg)
            y = math.radians(self.yaw_deg)

            # ROS慣習: roll=x, pitch=y, yaw=z の順でローカル固定軸回り
            qx, qy, qz, qw = tft.quaternion_from_euler(r, p, y)

            # parent -> child の固定変換を送信
            self.br.sendTransform((self.tx, self.ty, self.tz),
                                  (qx, qy, qz, qw),
                                  rospy.Time.now(),
                                  self.child_frame,
                                  self.parent_frame)
            rate.sleep()

if __name__ == "__main__":
    rospy.init_node("eef_correction_broadcaster")
    node = EEFCorrectionBroadcaster()
    node.spin()
