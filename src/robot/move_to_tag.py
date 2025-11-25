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
from std_srvs.srv import Trigger, TriggerResponse
from std_msgs.msg import Empty


# =========================================================
# パラメータ（他ノードと排他制御に使用）
# =========================================================
MOTION_LOCK_PARAM = "/motion_lock"     # Pick&Place ノードに「動くな」と指示する
MOTION_BUSY_PARAM = "/motion_busy"     # Pick&Place ノードの動作中フラグ


# =========================================================
# YAML Utility
# =========================================================
def load_poses_from_yaml(package_name: str, rel_path: str, root_key: str):
    """
    YAML から Pose 群を読み込んで dict にして返す。
    読み込み優先順位:
      1. 環境変数 XARM_POSES_YAML に絶対パス指定がある場合
      2. rospack パッケージルートから相対パスで検索
    """
    yaml_path = os.environ.get("XARM_POSES_YAML")

    if not yaml_path:
        rospack = rospkg.RosPack()
        yaml_path = os.path.join(rospack.get_path(package_name), rel_path)

    if not os.path.isfile(yaml_path):
        raise FileNotFoundError(f"YAMLファイルが見つかりません: {yaml_path}")

    with open(yaml_path, "r") as file_handle:
        yaml_data = yaml.safe_load(file_handle)

    if root_key not in yaml_data:
        raise KeyError(f"ルートキー '{root_key}' が {yaml_path} に存在しません")

    raw_dict = yaml_data[root_key]
    return {name: convert_list_to_pose(values) for name, values in raw_dict.items()}


def convert_list_to_pose(values):
    """7要素 [x, y, z, qx, qy, qz, qw] → geometry_msgs/Pose"""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = values[0:3]
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = values[3:7]
    return pose


# =========================================================
# Main Class
# =========================================================
class RecalibrationServer:
    """
    ロボットを再キャリブレーション姿勢に移動 → AprilTag で姿勢計測
    → TF 更新ノードへ通知 → IMU/ICP 基準リセット
    の一連処理を提供するサービスノード。
    """

    def __init__(self):
        rospy.init_node("xarm_recalibration_server")
        moveit_commander.roscpp_initialize([])

        # -----------------------------------------------------
        # MoveIt 初期化
        # -----------------------------------------------------
        namespace_used = "" if rospy.get_param("~use_root_ns", False) else rospy.get_namespace()
        rospy.loginfo(f"Using MoveIt namespace: '{namespace_used}'")

        self.arm = moveit_commander.MoveGroupCommander("xarm6", ns="")

        # -----------------------------------------------------
        # TF再読み込み通知パブリッシャ
        # -----------------------------------------------------
        self.reload_tf_publisher = rospy.Publisher("/recalib/reload_tf", Empty, queue_size=1)

        # -----------------------------------------------------
        # 起動ディレイ（起動衝突回避）
        # -----------------------------------------------------
        startup_delay = rospy.get_param("~wait_before_start", 0.0)
        if startup_delay > 0:
            rospy.loginfo(f"Delaying start by {startup_delay} sec to avoid MoveIt conflict...")
            rospy.sleep(startup_delay)

        # -----------------------------------------------------
        # パラメータ読み込み
        # -----------------------------------------------------
        self.package_name = rospy.get_param("~package_name", "xarm_apriltag_demo")
        self.yaml_rel_path = rospy.get_param("~poses_yaml_rel", "config/recalib_poses.yaml")
        self.yaml_root_key = rospy.get_param("~poses_root_key", "recalib_pose")

        # MoveIt 設定
        self.arm.set_max_velocity_scaling_factor(rospy.get_param("~vel_scale", 0.1))
        self.arm.set_max_acceleration_scaling_factor(rospy.get_param("~acc_scale", 0.1))
        self.arm.set_planning_time(rospy.get_param("~planning_time", 5.0))

        rospy.sleep(0.5)
        self.home_pose = self.arm.get_current_pose().pose

        # -----------------------------------------------------
        # 内部ステート
        # -----------------------------------------------------
        self.is_processing = False
        self.shutdown_flag = False
        self.thread_lock = threading.Lock()

        # -----------------------------------------------------
        # YAML から Recalibration Pose 読み込み
        # -----------------------------------------------------
        self.pose_table = load_poses_from_yaml(
            self.package_name,
            self.yaml_rel_path,
            self.yaml_root_key
        )

        # -----------------------------------------------------
        # IMU/ICP基準リセットクライアント
        # -----------------------------------------------------
        self.reset_reference_client = rospy.ServiceProxy("/monitor/reset_reference", Trigger)

        # -----------------------------------------------------
        # 再キャリブレーション実行サービス
        # -----------------------------------------------------
        self.service_handle = rospy.Service("/recalibration/run", Trigger, self.handle_recalibration)
        rospy.loginfo("Service [/recalibration/run] ready.")

        rospy.spin()

    # =========================================================
    # Recalibration Handler
    # =========================================================
    def handle_recalibration(self, _req):
        """
        /recalibration/run サービスの処理本体。
        - Pick&Place の停止
        - 再キャリブレーション姿勢への移動
        - AprilTag ベースの再キャリブ実行
        - TF更新通知
        - IMU/ICP Reference Reset
        """
        if self.is_processing:
            return TriggerResponse(success=False, message="既に再キャリブレーション処理中です。")

        # -----------------------------------------------------
        # Pick&Place ノードへ「動くな」指示
        # -----------------------------------------------------
        rospy.set_param(MOTION_LOCK_PARAM, True)
        rospy.loginfo("Pick&Place ノードの動作停止を要求中...")

        # Pick&Place が動作完了するまで待機
        while rospy.get_param(MOTION_BUSY_PARAM, False):
            rospy.loginfo_throttle(5.0, "Pick&Place のサイクル終了待ち中...")
            rospy.sleep(1.0)

        self.is_processing = True

        try:
            with self.thread_lock:
                # -----------------------------------------------------
                # MoveIt：安全停止 → 状態更新
                # -----------------------------------------------------
                self.arm.stop()
                self.arm.clear_pose_targets()
                self.arm.set_start_state_to_current_state()

                rospy.loginfo("=== 再キャリブレーション開始 ===")

                # -----------------------------------------------------
                # 再キャリブ姿勢へ移動
                # -----------------------------------------------------
                recalib_pose = self.pose_table["pose"]
                self.arm.set_pose_target(recalib_pose)
                moved = self.arm.go(wait=True)

                if moved:
                    rospy.loginfo("再キャリブレーション姿勢に到達しました。")
                    rospy.loginfo("現在姿勢を取得中...")
                    current_pose = self.arm.get_current_pose().pose
                    rospy.loginfo(f"Current Pose: {current_pose}")

                    # -----------------------------------------------------
                    # AprilTag キャプチャサービス呼び出し
                    # -----------------------------------------------------
                    rospy.loginfo("カメラキャリブレーション実行中...")
                    try:
                        capture_tag_service = rospy.ServiceProxy("/capture_tag_pose", Trigger)
                        result = capture_tag_service()

                        if result.success:
                            rospy.loginfo("カメラキャリブレーション成功。")

                            # TF更新ノードへ通知
                            self.reload_tf_publisher.publish()
                            rospy.loginfo("TF更新要求を送信しました。")

                        else:
                            rospy.logerr(f"キャリブレーション失敗: {result.message}")
                            return TriggerResponse(success=False, message="キャリブレーション失敗。")

                    except Exception as e:
                        rospy.logerr(f"capture_tag_pose の呼び出し失敗: {e}")
                        return TriggerResponse(success=False, message="capture_tag_pose 呼び出しに失敗。")

                else:
                    return TriggerResponse(success=False, message="再キャリブレーション姿勢に移動できませんでした。")

                # -----------------------------------------------------
                # ホーム姿勢へ復帰
                # -----------------------------------------------------
                rospy.loginfo("ホーム姿勢へ復帰中...")
                self.arm.stop()
                self.arm.clear_pose_targets()
                self.arm.set_start_state_to_current_state()
                self.arm.set_pose_target(self.home_pose)

                home_ok = self.arm.go(wait=True)
                if not home_ok:
                    return TriggerResponse(success=False, message="ホーム復帰に失敗しました。")

                rospy.loginfo("=== 再キャリブレーション完了 ===")

                # -----------------------------------------------------
                # IMU/ICP Reference をリセット
                # -----------------------------------------------------
                try:
                    reset_res = self.reset_reference_client()
                    rospy.loginfo(f"IMU/ICP reference reset: {reset_res.message}")
                except Exception as e:
                    rospy.logerr(f"IMU/ICP reference reset失敗: {e}")

                return TriggerResponse(success=True, message="再キャリブレーション完了。")

        except Exception as e:
            rospy.logerr(f"Recalibration failed: {e}")
            return TriggerResponse(success=False, message=str(e))

        finally:
            # Pick&Place ノードの再開許可
            rospy.set_param(MOTION_LOCK_PARAM, False)
            self.is_processing = False
            rospy.loginfo("Pick&Place ノードの動作再開を許可しました。")
            rospy.loginfo("=== 再キャリブレーション処理終了 ===")

    # =========================================================
    # Move Helper
    # =========================================================
    def move_to_xyzrpy(self, x, y, z, rx, ry, rz):
        """
        (xyz + rpy) で MoveIt 移動する簡易ユーティリティ
        """
        current_pose = self.arm.get_current_pose().pose

        current_pose.position.x = x
        current_pose.position.y = y
        current_pose.position.z = z

        qx, qy, qz, qw = quaternion_from_euler(rx, ry, rz)
        current_pose.orientation.x = qx
        current_pose.orientation.y = qy
        current_pose.orientation.z = qz
        current_pose.orientation.w = qw

        self.arm.stop()
        self.arm.clear_pose_targets()
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(current_pose)

        return bool(self.arm.go(wait=True))

    # =========================================================
    # Shutdown Handler
    # =========================================================
    def _on_shutdown(self):
        """ノード終了時の安全停止処理"""
        self.shutdown_flag = True
        try:
            self.arm.stop()
            self.arm.clear_pose_targets()
        except Exception:
            pass
        moveit_commander.roscpp_shutdown()


# =========================================================
# Main
# =========================================================
if __name__ == "__main__":
    RecalibrationServer()
