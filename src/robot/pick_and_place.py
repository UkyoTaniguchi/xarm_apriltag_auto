#!/usr/bin/env python3
#pick_and_placeの童話を行うためのモジュール
#config/pick_and_place.yamlのパラメータを使用する
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

MOTION_LOCK_PARAM = "/motion_lock"   # ← 他ノードと共有する排他パラメータ

#=========================================================
# YAMLを読み込んで姿勢dictを返す
#=========================================================
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
#=========================================================
# YAML Utility Functions
#=========================================================
def make_pose(values):
    """7要素[x,y,z,qx,qy,qz,qw]のリストからgeometry_msgs/Poseを生成"""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = values[0:3]
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = values[3:7]
    return pose

class PickAndPlace:
    def __init__(self):
        rospy.init_node("pick_and_place_node")
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
            # --- 他ノードによる動作ロックを確認 ---
            if rospy.get_param(MOTION_LOCK_PARAM, False):
                rospy.loginfo_throttle(5.0, "他ノード(再キャリブ等)が動作中のため待機中...")
                rate.sleep()
                continue

            if self.is_running:  # 再キャリブ中は動作停止
                rate.sleep()
                continue
            try:
                with self.lock:
                    rospy.loginfo("=== Pick & Place サイクル開始 ===")

                    for pose_name in self.sequence:
                        # --- 再キャリブ開始時に中断 ---
                        if rospy.get_param(MOTION_LOCK_PARAM, False):
                            rospy.loginfo("外部ロック検出。ピック&プレースを一時停止。")
                            break

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
    PickAndPlace()
