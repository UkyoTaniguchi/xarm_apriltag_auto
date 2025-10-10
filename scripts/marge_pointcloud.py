#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import std_msgs.msg
import tf2_ros
import tf2_sensor_msgs.tf2_sensor_msgs as tf2_sm

class PointCloudMerger:
    def __init__(self):
        self.pub = rospy.Publisher('/merged/points', PointCloud2, queue_size=1)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # 最新の点群を保存
        self.pc1_latest = None
        self.pc2_latest = None
        self.pc3_latest = None
        self.pc4_latest = None

        rospy.Subscriber('/cam_1/depth/color/points', PointCloud2, self.cb1)
        rospy.Subscriber('/cam_2/depth/color/points', PointCloud2, self.cb2)
        rospy.Subscriber('/cam_3/depth/color/points', PointCloud2, self.cb3)
        rospy.Subscriber('/cam_4/depth/color/points', PointCloud2, self.cb4)

    def cb1(self, msg):
        self.pc1_latest = msg
        self.try_merge()

    def cb2(self, msg):
        self.pc2_latest = msg
        self.try_merge()

    def cb3(self, msg):
        self.pc3_latest = msg
        self.try_merge()

    def cb4(self, msg):
        self.pc4_latest = msg
        self.try_merge()

    def try_merge(self):
        if self.pc1_latest is None or self.pc2_latest is None or self.pc3_latest is None or self.pc4_latest is None:
            return
        try:
            # TF変換
            t1 = self.tf_buffer.lookup_transform("world", self.pc1_latest.header.frame_id, rospy.Time(0), rospy.Duration(0.1))
            t2 = self.tf_buffer.lookup_transform("world", self.pc2_latest.header.frame_id, rospy.Time(0), rospy.Duration(0.1))
            t3 = self.tf_buffer.lookup_transform("world", self.pc3_latest.header.frame_id, rospy.Time(0), rospy.Duration(0.1))
            t4 = self.tf_buffer.lookup_transform("world", self.pc4_latest.header.frame_id, rospy.Time(0), rospy.Duration(0.1))
            pc1_world = tf2_sm.do_transform_cloud(self.pc1_latest, t1)
            pc2_world = tf2_sm.do_transform_cloud(self.pc2_latest, t2)
            pc3_world = tf2_sm.do_transform_cloud(self.pc3_latest, t3)
            pc4_world = tf2_sm.do_transform_cloud(self.pc4_latest, t4)
            # 点群を結合
            # 結合
            points = list(pc2.read_points(pc1_world, skip_nans=True)) + \
                     list(pc2.read_points(pc2_world, skip_nans=True)) + \
                     list(pc2.read_points(pc3_world, skip_nans=True)) + \
                     list(pc2.read_points(pc4_world, skip_nans=True))

            header = std_msgs.msg.Header()
            header.stamp = rospy.Time.now()
            header.frame_id = "world"

            merged_msg = pc2.create_cloud(pc1_world.header, pc1_world.fields, points)
            merged_msg.header = header
            self.pub.publish(merged_msg)

        except Exception as e:
            rospy.logwarn("Transform/merge failed: %s" % str(e))

if __name__ == '__main__':
    rospy.init_node('pointcloud_merger')
    PointCloudMerger()
    rospy.spin()
