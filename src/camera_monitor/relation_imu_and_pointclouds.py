#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
from sensor_msgs.msg import PointCloud2, PointField, Imu
import sensor_msgs.point_cloud2 as pc2
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String
import open3d as o3d


class SimpleTiltRMSE:
    def __init__(self):
        rospy.init_node("simple_tilt_rmse")

        self.mode = rospy.get_param("~rmse_mode", "theory")  # theory / icp
        self.pc_topic = rospy.get_param("~pointcloud_topic",
                                        "/camera/depth/color/points")

        self.tilt_deg = 0.0                     # theory: キー操作, icp: IMU差分
        self.rot = np.eye(3)                    # theory用回転行列

        self.ref_points = None                  # ICP用基準点群
        self.g_ref = None                       # IMU基準重力方向

        self.pub_rot = rospy.Publisher("/tilt/rotated", PointCloud2, queue_size=1)

        rospy.Subscriber(self.pc_topic, PointCloud2, self.pc_callback)
        rospy.Subscriber("key_input", String, self.key_callback)

        if self.mode == "icp":
            rospy.Subscriber("/camera/imu", Imu, self.imu_callback)
            rospy.loginfo("IMU subscriber enabled (/camera/imu)")

        rospy.loginfo("SimpleTiltRMSE started. mode=%s", self.mode)
        rospy.spin()

    # ---------------------- IMU ----------------------
    def imu_callback(self, msg):
        ax, ay, az = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z
        g = np.array([ax, ay, az], dtype=float)
        n = np.linalg.norm(g)
        if n < 1e-6:
            return

        g = g / n  # 単位ベクトル

        if self.g_ref is None:
            self.g_ref = g
            self.tilt_deg = 0.0
            rospy.loginfo("[IMU] g_ref set to (%.3f, %.3f, %.3f)", *g)
            return

        dot = float(np.clip(np.dot(self.g_ref, g), -1.0, 1.0))
        self.tilt_deg = math.degrees(math.acos(dot))

    # ---------------------- key input ----------------------
    def key_callback(self, msg):
        if self.mode != "theory":
            return

        if msg.data == "t":
            self.tilt_deg += 0.5
        elif msg.data == "g":
            self.tilt_deg -= 0.5
        else:
            return

        self.update_rotation()
        rospy.loginfo("[KEY] tilt_deg=%.2f deg", self.tilt_deg)

    def update_rotation(self):
        self.rot = R.from_euler("x", np.deg2rad(self.tilt_deg)).as_matrix()

    # ---------------------- pointcloud ----------------------
    def pc_callback(self, msg):
        points = np.array([[p[0], p[1], p[2]]
                           for p in pc2.read_points(msg, skip_nans=True)], dtype=float)

        if len(points) == 0:
            return

        if self.mode == "theory":
            self._run_theory(points, msg)
        else:
            self._run_icp(points, msg)

    # ---------------------- theory mode ----------------------
    def _run_theory(self, points, msg):
        rotated = (self.rot @ points.T).T
        d = np.linalg.norm(rotated - points, axis=1)

        self.mean_d = float(np.mean(d))

        rospy.loginfo("[THEORY] tilt=%.2f deg | mean_distance=%.6f m",
                      self.tilt_deg, self.mean_d)

        self.publish(rotated, msg.header.frame_id)

    # ---------------------- ICP mode ----------------------
    def _run_icp(self, points, msg):

        if self.g_ref is None:
            rospy.loginfo_throttle(2.0, "[ICP] Waiting for IMU (g_ref not set)...")
            return

        if self.ref_points is None:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            self.ref_points = pcd.voxel_down_sample(0.02)
            rospy.loginfo("[ICP] Saved reference cloud. N=%d", len(self.ref_points.points))
            return

        pcd_cur = o3d.geometry.PointCloud()
        pcd_cur.points = o3d.utility.Vector3dVector(points)
        cur_ds = pcd_cur.voxel_down_sample(0.02)

        if len(cur_ds.points) == 0:
            return

        # 最近傍
        pcd_ref = o3d.geometry.PointCloud()
        pcd_ref.points = self.ref_points.points
        kdtree = o3d.geometry.KDTreeFlann(pcd_ref)

        dists = []
        for p in np.asarray(cur_ds.points):
            _, idx, d = kdtree.search_knn_vector_3d(p, 1)
            dists.append(math.sqrt(d[0]))

        dists = np.array(dists)
        self.mean_d = float(np.mean(dists))
        max_d = float(np.max(dists))
        self.imu_diff = self.tilt_deg

        rospy.loginfo("[ICP] IMU_diff=%.2f deg | mean=%.4f | max=%.4f",
                      self.imu_diff, self.mean_d, max_d)

        self.publish(points, msg.header.frame_id)

    # ---------------------- publish & graph ----------------------
    def publish(self, points, frame):
        rgb = (255 << 16)
        rgb_f = np.frombuffer(np.uint32(rgb).tobytes(), dtype=np.float32)[0]

        pts = [[p[0], p[1], p[2], rgb_f] for p in points]

        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("rgb", 12, PointField.FLOAT32, 1),
        ]

        header = rospy.Header(frame_id=frame)
        header.stamp = rospy.Time.now()

        pc2_msg = pc2.create_cloud(header, fields, pts)
        self.pub_rot.publish(pc2_msg)

        # グラフ描画
        if self.mode == "icp":
            self.plot_graph(self.imu_diff, self.mean_d)
        else:
            self.plot_graph(self.tilt_deg, self.mean_d)

    def plot_graph(self, tilt_or_imu, mean_val):
        if not hasattr(self, 'fig'):
            import matplotlib.pyplot as plt
            self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(8, 6))
            plt.ion()
            plt.show()
            self.tilt_hist = []
            self.mean_hist = []

        self.tilt_hist.append(tilt_or_imu)
        self.mean_hist.append(mean_val)

        self.ax1.cla()
        self.ax2.cla()

        self.ax1.plot(self.tilt_hist, label='IMU / tilt (deg)')
        self.ax1.set_ylabel("Tilt / IMU diff [deg]")
        self.ax1.legend()

        self.ax2.plot(self.mean_hist, label='Mean distance (m)')
        self.ax2.set_ylabel("Mean distance [m]")
        self.ax2.legend()

        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


if __name__ == "__main__":
    try:
        SimpleTiltRMSE()
    except rospy.ROSInterruptException:
        pass
