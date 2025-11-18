#!/usr/bin/env python3
import os
import math
import threading
import yaml
import rospy
import rospkg
import moveit_commander
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler
from std_srvs.srv import Trigger, TriggerResponse     # ← 既存

MOTION_LOCK_PARAM = "/motion_lock"   # ← 他ノードと共有する排他パラメータ
MOTION_BUSY_PARAM = "/motion_busy"   # ← Pick&Placeの動作中状態

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

def make_pose(values):
    """7要素[x,y,z,qx,qy,qz,qw]のリストからgeometry_msgs/Poseを生成"""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = values[0:3]
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = values[3:7]
    return pose


class RecalibrationServer:
    def __init__(self):
        rospy.init_node("xarm_recalibration_server")
        moveit_commander.roscpp_initialize([])
        ns = "" if rospy.get_param("~use_root_ns", False) else rospy.get_namespace()
        rospy.loginfo(f"Using MoveIt namespace: '{ns}'")
        self.arm = moveit_commander.MoveGroupCommander("xarm6", ns="")


        # --- 起動ディレイ（競合防止用） ---
        wait_before = rospy.get_param("~wait_before_start", 0.0)
        if wait_before > 0:
            rospy.loginfo(f"Delaying start by {wait_before} sec to avoid MoveIt conflict...")
            rospy.sleep(wait_before)

        # --- パラメータ ---
        self.pkg_name = rospy.get_param("~package_name", "xarm_apriltag_demo")
        self.yaml_rel_path = rospy.get_param("~poses_yaml_rel", "config/recalib_poses.yaml")
        self.yaml_root_key = rospy.get_param("~poses_root_key", "recalib_pose")

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

        # --- ★追加：IMU/ICP基準リセットサービスクライアント ---
        self.reset_reference_client = rospy.ServiceProxy("/monitor/reset_reference", Trigger)

        # --- 再キャリブレーションサービス登録 ---
        self.srv = rospy.Service("/recalibration/run", Trigger, self.handle_recalibration)
        rospy.loginfo("Service [/recalibration/run] ready.")

        rospy.spin()

    # =========================================================
    # Recalibration Handler
    # =========================================================
    def handle_recalibration(self, _req):
        """/recalibration/run サービスを処理"""
        if self.is_running:
            return TriggerResponse(success=False, message="既に再キャリブレーション中です。")

        # --- Pick&Placeノードを停止させる ---
        rospy.set_param(MOTION_LOCK_PARAM, True)
        rospy.loginfo("Pick&Placeノードに動作停止を要求中...")

        # --- Pick&Placeのサイクル完了を待機 ---
        while rospy.get_param(MOTION_BUSY_PARAM, False):
            rospy.loginfo_throttle(5.0, "Pick&Place完了待機中...")
            rospy.sleep(1.0)

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

                # 再キャリブ姿勢に移動できたのか確認
                if success:
                    rospy.loginfo("現在の姿勢を確認中...")
                    #現在の姿勢を取得
                    current_pose = self.arm.get_current_pose().pose
                    rospy.loginfo(f"Current Pose: {current_pose}")
                    rospy.loginfo("再キャリブレーション姿勢に到達しました。")
                    rospy.loginfo("カメラキャリブレーションを実行します")
                    # カメラキャリブレーション処理サービスの呼び出し
                    try:
                        recalib_client = rospy.ServiceProxy("/capture_tag_pose", Trigger)
                        recalib_response = recalib_client()
                        if recalib_response.success:
                            rospy.loginfo("カメラキャリブレーションが成功しました。")
                        else:
                            rospy.logerr(f"カメラキャリブレーションに失敗しました: {recalib_response.message}")
                            rospy.set_param(MOTION_LOCK_PARAM, False)
                            return TriggerResponse(success=False, message="カメラキャリブレーションに失敗しました。")
                    except Exception as e:
                        rospy.logerr(f"カメラキャリブレーションサービスの呼び出しに失敗しました: {e}")
                        rospy.set_param(MOTION_LOCK_PARAM, False)
                        return TriggerResponse(success=False, message="カメラキャリブレーションサービスの呼び出しに失敗しました。")
                else:
                    rospy.set_param(MOTION_LOCK_PARAM, False)
                    return TriggerResponse(success=False, message="再キャリブレーション姿勢に到達できませんでした。")
                # ホーム姿勢へ復帰
                rospy.loginfo("ホーム姿勢へ復帰中...")
                self.arm.stop()
                self.arm.clear_pose_targets()
                self.arm.set_start_state_to_current_state()
                self.arm.set_pose_target(self.home_pose)
                ok = self.arm.go(wait=True)

                if not ok:
                    rospy.set_param(MOTION_LOCK_PARAM, False)
                    return TriggerResponse(success=False, message="ホーム復帰に失敗しました。")

                rospy.loginfo("=== 再キャリブレーション完了 ===")

                # --- ★追加：IMU/ICP基準リセットを呼び出す ---
                try:
                    res = self.reset_reference_client()
                    rospy.loginfo(f"IMU/ICP reference reset: {res.message}")
                except Exception as e:
                    rospy.logerr(f"Failed to reset IMU/ICP reference: {e}")

                return TriggerResponse(success=True, message="再キャリブレーション完了。")

        except Exception as e:
            rospy.logerr(f"Recalibration failed: {e}")
            return TriggerResponse(success=False, message=str(e))

        finally:
            # --- Pick&Placeノード再開許可 ---
            rospy.set_param(MOTION_LOCK_PARAM, False)
            self.is_running = False
            rospy.loginfo("Pick&Placeノードの動作再開を許可しました。")
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


if __name__ == "__main__":
    RecalibrationServer()
