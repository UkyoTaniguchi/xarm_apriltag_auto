#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
xArm6 Pick & Place + Auto Recalibration Node
============================================

機能概要:
  - YAMLファイル(config/pick_and_place_pose.yaml)からピック&プレースの目標姿勢を読み込む
  - 順にピック&プレース動作を繰り返す
  - /recalibration/run (std_srvs/Trigger) を受け取ると再キャリブレーション姿勢へ移動
      → 2秒静止 → ホーム復帰

構成推奨:
  your_pkg/
    scripts/xarm_recalibration_server.py   ← 本ファイル
    config/pick_and_place_pose.yaml        ← 各姿勢定義
    launch/xarm_recalibration.launch       ← (任意) rosparam load 用
"""

import os
import math
import threading
import yaml
import rospy
import rospkg
import moveit_commander
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
from std_srvs.srv import Trigger, TriggerResponse


# =========================================================
# YAML Utility Functions
# =========================================================
def make_pose(values):
    """7要素[x,y,z,qx,qy,qz,qw]のリストからgeometry_msgs/Poseを生成"""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = values[0:3]
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = values[3:7]
    return pose


def load_poses_from_yaml(package_name: str, rel_path: str, root_key: str):
    """
    YAMLを読み込んで姿勢dictを返す。
    優先順位:
      1. 環境変数 XARM_POSES_YAML に絶対パス指定がある場合
      2. rospack.get_path(package_name)/rel_path
    """
    # 1. 環境変数で絶対パス指定されている場合
    yaml_path = os.environ.get("XARM_POSES_YAML")
    if not yaml_path:
        # 2. パッケージのconfigから相対パスで検索
        rospack = rospkg.RosPack()
        yaml_path = os.path.join(rospack.get_path(package_name), rel_path)

    if not os.path.isfile(yaml_path):
        raise FileNotFoundError(f"YAMLファイルが見つかりません: {yaml_path}")

    # YAML読み込み
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f)

    if root_key not in data:
        raise KeyError(f"ルートキー '{root_key}' が {yaml_path} に存在しません")

    raw_dict = data[root_key]
    poses = {name: make_pose(values) for name, values in raw_dict.items()}
    return poses


# =========================================================
# メインクラス
# =========================================================
class XarmRecalibrationServer:
    """xArm6のピック&プレース動作と再キャリブレーション制御を担当するROSノード"""

    def __init__(self):
        # ------------------------------------------
        # ROS Node 初期化
        # ------------------------------------------
        rospy.init_node("xarm_recalibration_server")
        moveit_commander.roscpp_initialize([])

        # --- パラメータ ---
        self.pkg_name = rospy.get_param("~package_name", "xarm_apriltag_demo")
        self.yaml_rel_path = rospy.get_param("~poses_yaml_rel", "config/pick_and_place_pose.yaml")
        self.yaml_root_key = rospy.get_param("~poses_root_key", "pick_and_place")

        # --- MoveIt初期化 ---
        self.arm = moveit_commander.MoveGroupCommander("xarm6")
        self.arm.set_max_velocity_scaling_factor(rospy.get_param("~vel_scale", 1.0))
        self.arm.set_max_acceleration_scaling_factor(rospy.get_param("~acc_scale", 1.0))
        self.arm.set_planning_time(rospy.get_param("~planning_time", 5.0))

        rospy.sleep(0.5)
        self.home_pose = self.arm.get_current_pose().pose  # 現在姿勢をホームとして記録
        self.is_running = False
        self.shutdown_flag = False
        self.lock = threading.Lock()

        # --- YAML読み込み ---
        self.poses = load_poses_from_yaml(self.pkg_name, self.yaml_rel_path, self.yaml_root_key)

        # --- 動作シーケンス定義 ---
        self.sequence = rospy.get_param(
            "~sequence", ["target1", "target2", "target1", "target3", "target4", "target3"]
        )

        # --- 再キャリブレーションサービス登録 ---
        self.srv = rospy.Service("/recalibration/run", Trigger, self.handle_recalibration)
        rospy.loginfo("Service [/recalibration/run] ready.")

        # --- ピックアンドプレーススレッド起動 ---
        self.pick_thread = threading.Thread(target=self.pick_and_place_loop, daemon=True)
        self.pick_thread.start()

        rospy.on_shutdown(self._on_shutdown)
        rospy.spin()

    # =========================================================
    # Pick & Place Loop
    # =========================================================
    def pick_and_place_loop(self):
        """定期的にピック&プレース動作を繰り返すループ"""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and not self.shutdown_flag:
            if self.is_running:  # 再キャリブ中は動作停止
                rate.sleep()
                continue
            try:
                with self.lock:
                    rospy.loginfo("=== Pick & Place サイクル開始 ===")

                    for pose_name in self.sequence:
                        if pose_name not in self.poses:
                            rospy.logwarn(f"YAML内に {pose_name} が存在しません。スキップします。")
                            continue

                        target_pose = self.poses[pose_name]
                        self.arm.stop()
                        self.arm.clear_pose_targets()
                        self.arm.set_start_state_to_current_state()
                        self.arm.set_pose_target(target_pose)

                        success = self.arm.go(wait=True)
                        if not success:
                            rospy.logwarn(f"姿勢 {pose_name} への移動に失敗しました。")
                            break

                        rospy.sleep(0.1)

            except Exception as e:
                rospy.logwarn(f"Pick & Place ループエラー: {e}")

            rate.sleep()

    # =========================================================
    # Recalibration Handler
    # =========================================================
    def handle_recalibration(self, _req):
        """/recalibration/run サービスを処理"""
        if self.is_running:
            return TriggerResponse(success=False, message="既に再キャリブレーション中です。")

        self.is_running = True
        try:
            with self.lock:
                # 現在の動作を安全に停止
                self.arm.stop()
                self.arm.clear_pose_targets()
                self.arm.set_start_state_to_current_state()

                rospy.loginfo("=== 再キャリブレーション開始 ===")

                # 再キャリブ姿勢へ移動
                success = self.move_to_xyzrpy(
                    x=0.0, y=-0.50, z=0.30,
                    rx=math.radians(90.0), ry=0.0, rz=0.0
                )
                if not success:
                    return TriggerResponse(success=False, message="再キャリブレーション姿勢への移動に失敗しました。")

                rospy.sleep(2.0)  # 静止状態を確保（再キャリブ実施中）

                rospy.loginfo("ホーム姿勢へ復帰中...")
                self.arm.stop()
                self.arm.clear_pose_targets()
                self.arm.set_start_state_to_current_state()
                self.arm.set_pose_target(self.home_pose)
                ok = self.arm.go(wait=True)

                if not ok:
                    return TriggerResponse(success=False, message="ホーム復帰に失敗しました。")

                return TriggerResponse(success=True, message="再キャリブレーション完了。")

        except Exception as e:
            rospy.logerr(f"Recalibration failed: {e}")
            return TriggerResponse(success=False, message=str(e))

        finally:
            self.is_running = False
            rospy.loginfo("=== 再キャリブレーション終了 ===")

    # =========================================================
    # Move Helper: XYZ + RPY
    # =========================================================
    def move_to_xyzrpy(self, x, y, z, rx, ry, rz):
        """位置(x,y,z)[m]と姿勢(rx,ry,rz)[rad]を指定してMoveItで移動"""
        pose = self.arm.get_current_pose().pose
        pose.position.x, pose.position.y, pose.position.z = x, y, z

        qx, qy, qz, qw = quaternion_from_euler(rx, ry, rz)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = qx, qy, qz, qw

        self.arm.stop()
        self.arm.clear_pose_targets()
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(pose)
        return bool(self.arm.go(wait=True))

    # =========================================================
    # Shutdown Handler
    # =========================================================
    def _on_shutdown(self):
        """ノード終了時の安全停止"""
        self.shutdown_flag = True
        try:
            self.arm.stop()
            self.arm.clear_pose_targets()
        except Exception:
            pass
        moveit_commander.roscpp_shutdown()


# =========================================================
# メイン処理
# =========================================================
if __name__ == "__main__":
    XarmRecalibrationServer()
