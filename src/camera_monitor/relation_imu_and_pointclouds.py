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

        # theory: キー操作で使う tilt
        # icp: IMU から計算した「初期重力方向からの傾き角」
        self.tilt_deg = 0.0

        # theory 用回転
        self.rot = np.eye(3)

        # icp 用
        self.ref_points = None         # 最初の点群（voxel 減）
        self.g_ref = None              # 初期の重力方向（単位ベクトル）

        # publisher
        self.pub_rot = rospy.Publisher("/tilt/rotated", PointCloud2, queue_size=1)

        # subscribers
        rospy.Subscriber(self.pc_topic, PointCloud2, self.pc_callback)
        rospy.Subscriber("key_input", String, self.key_callback)

        if self.mode == "icp":
            rospy.Subscriber("/camera/imu", Imu, self.imu_callback)
            rospy.loginfo("IMU subscriber enabled (/camera/imu)")

        rospy.loginfo("SimpleTiltRMSE started. mode=%s", self.mode)
        rospy.spin()

    # -------------------------------------------------------
    # IMU コールバック（icp モード専用）
    #   linear_acceleration から「初期重力方向との差の角度」を tilt_deg とする
    # -------------------------------------------------------
    def imu_callback(self, msg):
        # 加速度ベクトル（重力 + 慣性）
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        g = np.array([ax, ay, az], dtype=float)
        n = np.linalg.norm(g)
        if n < 1e-6:
            # ノイズ or 無効
            return

        g = g / n  # 単位ベクトル

        # 初回: 基準重力方向を保存
        if self.g_ref is None:
            self.g_ref = g
            self.tilt_deg = 0.0
            rospy.loginfo("[IMU] g_ref set to (%.3f, %.3f, %.3f)", g[0], g[1], g[2])
            return

        # 現在の tilt = g_ref と g のなす角
        dot = float(np.clip(np.dot(self.g_ref, g), -1.0, 1.0))
        angle_rad = math.acos(dot)
        self.tilt_deg = math.degrees(angle_rad)

    # -------------------------------------------------------
    # theory: キー入力で tilt_deg を変える
    # -------------------------------------------------------
    def key_callback(self, msg):
        if self.mode != "theory":
            # icp モードではキー入力で tilt を変えない
            return

        if msg.data == "t":
            self.tilt_deg += 0.5
        elif msg.data == "g":
            self.tilt_deg -= 0.5
        else:
            return

        self.update_rotation()

        rospy.loginfo("[KEY] tilt_deg=%.2f deg", self.tilt_deg)

    # -------------------------------------------------------
    # tilt → rotation（theory 専用）
    # -------------------------------------------------------
    def update_rotation(self):
        self.rot = R.from_euler("x", np.deg2rad(self.tilt_deg)).as_matrix()

    # -------------------------------------------------------
    # 点群コールバック
    # -------------------------------------------------------
    def pc_callback(self, msg):
        points = np.array([[p[0], p[1], p[2]]
                           for p in pc2.read_points(msg, skip_nans=True)],
                          dtype=float)

        if len(points) == 0:
            return

        if self.mode == "theory":
            self._run_theory(points, msg)
        else:
            self._run_icp(points, msg)

    # -------------------------------------------------------
    # theory: 回転前 vs 回転後
    # -------------------------------------------------------
    def _run_theory(self, points, msg):
        rotated = (self.rot @ points.T).T
        disp = np.linalg.norm(rotated - points, axis=1)
        rmse = np.sqrt(np.mean(disp ** 2))

        rospy.loginfo("[THEORY] tilt=%.2f deg | RMSE=%.6f m",
                      self.tilt_deg, rmse)

        self.publish(rotated, msg.header.frame_id)

    # -------------------------------------------------------
    # icp: ref(初回) vs points（実観測）を比較
    #      tilt_deg は IMU からの「初期重力方向からの角度」
    # -------------------------------------------------------
    def _run_icp(self, points, msg):

        # IMU まだ初期化されていない場合
        if self.g_ref is None:
            rospy.loginfo_throttle(2.0, "[ICP] Waiting for IMU (g_ref not set)...")
            return

        # 1. 最初の点群を ref として保存（voxel 減）
        if self.ref_points is None:
            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(points)
            self.ref_points = pcd.voxel_down_sample(0.02)
            rospy.loginfo("[ICP] Saved reference cloud. N=%d", len(self.ref_points.points))
            return

        # 2. 現在点群をダウンサンプル
        pcd_cur = o3d.geometry.PointCloud()
        pcd_cur.points = o3d.utility.Vector3dVector(points)
        cur_ds = pcd_cur.voxel_down_sample(0.02)

        if len(cur_ds.points) == 0:
            return

        # 3. 最近傍距離（ref: self.ref_points）
        pcd_ref = o3d.geometry.PointCloud()
        pcd_ref.points = self.ref_points.points
        kdtree = o3d.geometry.KDTreeFlann(pcd_ref)

        dists = []
        for p in np.asarray(cur_ds.points):
            _, idx, d = kdtree.search_knn_vector_3d(p, 1)
            dists.append(math.sqrt(d[0]))

        dists = np.array(dists)
        rmse = float(np.sqrt(np.mean(dists ** 2)))
        mean_d = float(np.mean(dists))
        max_d = float(np.max(dists))

        # tilt_deg はそのまま「IMU_diff」として解釈してよい
        imu_diff = self.tilt_deg

        rospy.loginfo("[ICP] IMU_diff=%.2f deg | RMSE=%.4f | mean=%.4f | max=%.4f",
                      imu_diff, rmse, mean_d, max_d)

        # 4. 現在の点群を赤で表示
        self.publish(points, msg.header.frame_id)

    # -------------------------------------------------------
    # publish (always red)
    # -------------------------------------------------------
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


if __name__ == "__main__":
    try:
        SimpleTiltRMSE()
    except rospy.ROSInterruptException:
        pass
