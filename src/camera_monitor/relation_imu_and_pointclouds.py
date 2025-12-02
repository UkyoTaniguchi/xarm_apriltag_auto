#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
IMU tilt（重力方向のズレ）を軸回転として 3D 点群へ適用し、
理論 RMSE / 実観測 RMSE (ICP) を切り替えて評価するノード
"""

import numpy as np
import rospy
import math
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import String
import open3d as o3d


class TiltBasedPointCloudRotation:
    def __init__(self):
        rospy.init_node("tilt_pointcloud_rotation")

        # --- Parameters ---
        self.pc_topic = rospy.get_param("~pointcloud_topic",
                                        "/camera/depth/color/points")

        # rmse_mode: "theory" or "icp"
        self.rmse_mode = rospy.get_param("~rmse_mode", "icp")

        # ROI
        self.roi_u_min = rospy.get_param("~roi_u_min", 200)
        self.roi_u_max = rospy.get_param("~roi_u_max", 500)
        self.roi_v_min = rospy.get_param("~roi_v_min", 50)
        self.roi_v_max = rospy.get_param("~roi_v_max", 400)

        # Camera intrinsics
        self.fx = rospy.get_param("~fx", 615.0)
        self.fy = rospy.get_param("~fy", 615.0)
        self.cx = rospy.get_param("~cx", 320.0)
        self.cy = rospy.get_param("~cy", 240.0)

        # tilt angle (キーで操作)
        self.tilt_deg = 0.0
        self.ref_gravity = np.array([0, 0, -1], dtype=float)
        self.rot = np.eye(3)

        # ICP 用パラメータ
        self.icp_max_corr = rospy.get_param("~icp_max_corr", 0.05)
        self.icp_max_iter = rospy.get_param("~icp_max_iter", 50)
        self.voxel_size = rospy.get_param("~icp_voxel_size", 0.01)

        # ICP 用参照点群
        self.ref_pcd = None  # open3d.geometry.PointCloud

        # Publisher
        self.pub_rotated = rospy.Publisher("/imu_pc/rotated",
                                           PointCloud2, queue_size=1)

        # Subscriber
        rospy.Subscriber(self.pc_topic, PointCloud2, self.pc_callback)
        rospy.Subscriber("key_input", String, self.key_callback)

        rospy.loginfo(
            "TiltBasedPointCloudRotation started. rmse_mode=%s", self.rmse_mode
        )
        rospy.spin()

    # -------------------------------------------
    # tilt をキーで操作（理論モードの回転更新）
    # -------------------------------------------
    def key_callback(self, msg):
        key = msg.data

        if key == "t":     # tilt +
            self.tilt_deg += 0.5
        elif key == "g":   # tilt -
            self.tilt_deg -= 0.5
        else:
            return

        if self.rmse_mode == "theory":
            self.update_rotation_from_tilt()

        rospy.loginfo("[KEY] tilt=%.2f deg", self.tilt_deg)

    # -------------------------------------------
    # 理論 tilt → 回転行列
    # -------------------------------------------
    def update_rotation_from_tilt(self):
        tilt_rad = np.deg2rad(self.tilt_deg)

        # X軸周りに tilt だけ回したと仮定
        rot_sim = R.from_euler('x', tilt_rad).as_matrix()
        cur_gravity = rot_sim @ self.ref_gravity

        axis = np.cross(self.ref_gravity, cur_gravity)
        n = np.linalg.norm(axis)
        if n < 1e-6:
            self.rot = np.eye(3)
            return

        axis /= n
        self.rot = R.from_rotvec(axis * tilt_rad).as_matrix()

        rospy.loginfo(
            "[THEORY] tilt=%.2f deg, axis=%s", self.tilt_deg, axis
        )

    # -------------------------------------------
    # PointCloud callback
    # -------------------------------------------
    def pc_callback(self, msg):
        # 点群を読み出し & ROI でフィルタ
        pts_in = pc2.read_points(
            msg, skip_nans=True,
            field_names=("x", "y", "z", "rgb")
        )

        roi_xyz = []
        for x, y, z, rgb in pts_in:
            if z <= 0:
                continue

            u = int(self.fx * x / z + self.cx)
            v = int(self.fy * y / z + self.cy)

            if (self.roi_u_min <= u <= self.roi_u_max and
                self.roi_v_min <= v <= self.roi_v_max):
                roi_xyz.append([x, y, z])

        if len(roi_xyz) == 0:
            return

        points = np.asarray(roi_xyz, dtype=np.float64)
        mean_z = np.mean(points[:, 2])

        if self.rmse_mode == "theory":
            self._process_theory_mode(points, mean_z, msg)
        elif self.rmse_mode == "icp":
            self._process_icp_mode(points, mean_z, msg)
        else:
            rospy.logwarn_throttle(
                5.0, "Unknown rmse_mode=%s", self.rmse_mode
            )

    # -------------------------------------------
    # 理論モード: rot @ p と p の RMSE
    # -------------------------------------------
    def _process_theory_mode(self, points, mean_z, msg):
        N = points.shape[0]

        rotated_points = (self.rot @ points.T).T

        diff = rotated_points - points
        disp = np.linalg.norm(diff, axis=1)

        rmse = np.sqrt(np.mean(disp * disp))
        mean_disp = float(np.mean(disp))
        max_disp = float(np.max(disp))

        rospy.loginfo(
            "[THEORY] tilt=%.2f deg | N=%d mean_z=%.3fm | "
            "RMSE=%.6fm mean=%.6fm max=%.6fm",
            self.tilt_deg, N, mean_z, rmse, mean_disp, max_disp
        )

        pc_rot = self.make_pc2_red(rotated_points, msg.header.frame_id)
        self.pub_rotated.publish(pc_rot)

    # -------------------------------------------
    # 実観測モード: ICP (point-to-point) で ref vs 現在 を比較
    # -------------------------------------------
    def _process_icp_mode(self, points, mean_z, msg):
        # Open3D point cloud に変換
        pcd_cur = o3d.geometry.PointCloud()
        pcd_cur.points = o3d.utility.Vector3dVector(points)

        # 参照点群がまだ無い場合 → 保存して終了
        if self.ref_pcd is None:
            self.ref_pcd = pcd_cur.voxel_down_sample(self.voxel_size)
            rospy.loginfo(
                "[ICP] Reference cloud saved: N_ref=%d mean_z=%.3fm",
                len(self.ref_pcd.points), mean_z
            )
            return

        # 下層化
        src = pcd_cur.voxel_down_sample(self.voxel_size)
        tgt = self.ref_pcd

        if len(src.points) < 10 or len(tgt.points) < 10:
            rospy.logwarn_throttle(
                2.0, "[ICP] Not enough points: src=%d tgt=%d",
                len(src.points), len(tgt.points)
            )
            return

        # Point-to-point ICP
        reg = o3d.pipelines.registration.registration_icp(
            src, tgt,
            self.icp_max_corr,
            np.eye(4),
            o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            o3d.pipelines.registration.ICPConvergenceCriteria(
                max_iteration=self.icp_max_iter
            )
        )
        T = reg.transformation

        # 整列後の点群
        src_aligned = src.transform(T.copy())

        # 参照点群との距離を計算（最近傍距離）
        dists = np.asarray(
            src_aligned.compute_point_cloud_distance(tgt),
            dtype=np.float64
        )

        if dists.size == 0:
            return

        rmse = float(np.sqrt(np.mean(dists * dists)))
        mean_disp = float(np.mean(dists))
        max_disp = float(np.max(dists))

        trans = T[:3, 3]
        trans_norm = float(np.linalg.norm(trans))

        rospy.loginfo(
            "[ICP] tilt_label=%.2f deg | "
            "N_src=%d N_ref=%d | "
            "trans_norm=%.6fm | "
            "RMSE=%.6fm mean=%.6fm max=%.6fm",
            self.tilt_deg,
            len(src.points), len(tgt.points),
            trans_norm, rmse, mean_disp, max_disp
        )

        # 可視化用に整列後点群を publish（赤）
        aligned_points = np.asarray(src_aligned.points)
        pc_rot = self.make_pc2_red(aligned_points, msg.header.frame_id)
        self.pub_rotated.publish(pc_rot)

    # -------------------------------------------
    # Utility: PointCloud2 生成
    # -------------------------------------------
    def make_pc2_red(self, points, frame):
        rgb = (255 << 16)  # 赤
        rgb_float = np.frombuffer(
            np.uint32(rgb).tobytes(), dtype=np.float32
        )[0]

        cloud_points = [[p[0], p[1], p[2], rgb_float] for p in points]

        fields = [
            PointField('x',   0,  PointField.FLOAT32, 1),
            PointField('y',   4,  PointField.FLOAT32, 1),
            PointField('z',   8,  PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.FLOAT32, 1),
        ]

        header = rospy.Header(frame_id=frame)
        header.stamp = rospy.Time.now()

        return pc2.create_cloud(header, fields, cloud_points)


if __name__ == "__main__":
    try:
        TiltBasedPointCloudRotation()
    except rospy.ROSInterruptException:
        pass
