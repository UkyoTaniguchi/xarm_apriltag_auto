#!/usr/bin/env python3
import os
import threading
import yaml
import rospy
import rospkg
import moveit_commander
from geometry_msgs.msg import Pose
from std_srvs.srv import Trigger, TriggerResponse


# ====================================================================
# グローバルパラメータ名（他ノードとの同期用）
# ====================================================================
MOTION_LOCK_PARAM = "/motion_lock"    # 他ノードがロボットを占有中
MOTION_BUSY_PARAM = "/motion_busy"    # このノードがロボットを占有中


# ====================================================================
# YAML Utility
# ====================================================================
def load_poses_from_yaml(package_name: str, rel_path: str, root_key: str):
    """
    YAMLから Pose をまとめて読み込む。
    XARM_POSES_YAML が設定されていればそちらを優先。
    """
    yaml_file_path = os.environ.get("XARM_POSES_YAML")

    if not yaml_file_path:
        rospack = rospkg.RosPack()
        yaml_file_path = os.path.join(rospack.get_path(package_name), rel_path)

    if not os.path.isfile(yaml_file_path):
        raise FileNotFoundError(f"YAMLファイルが見つかりません: {yaml_file_path}")

    with open(yaml_file_path, "r") as file_handle:
        yaml_data = yaml.safe_load(file_handle)

    if root_key not in yaml_data:
        raise KeyError(f"ルートキー '{root_key}' が {yaml_file_path} に存在しません")

    pose_dict_raw = yaml_data[root_key]
    pose_dict = {name: make_pose(values) for name, values in pose_dict_raw.items()}
    return pose_dict


def make_pose(values):
    """list → geometry_msgs/Pose に変換"""
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = values[0:3]
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = values[3:7]
    return pose


# ====================================================================
# Main Class
# ====================================================================
class PickAndPlace:
    """
    YAMLで定義された複数の Pose へ MoveIt を用いて順番に移動する
    Pick & Place 制御ノード。
    """

    def __init__(self):
        rospy.init_node("pick_and_place_node")
        moveit_commander.roscpp_initialize([])

        # ------------------------------------------------------------
        # MoveIt 初期化
        # ------------------------------------------------------------
        namespace_option = "" if rospy.get_param("~use_root_ns", False) else rospy.get_namespace()
        rospy.loginfo(f"Using MoveIt namespace: '{namespace_option}'")

        self.arm = moveit_commander.MoveGroupCommander("xarm6", ns="")
        self.arm.set_max_velocity_scaling_factor(rospy.get_param("~vel_scale", 0.1))
        self.arm.set_max_acceleration_scaling_factor(rospy.get_param("~acc_scale", 0.1))
        self.arm.set_planning_time(rospy.get_param("~planning_time", 10.0))

        rospy.sleep(0.5)
        self.home_pose = self.arm.get_current_pose().pose  # 現在の姿勢を保存（使っていないが保持は妥当）

        # ------------------------------------------------------------
        # 内部状態
        # ------------------------------------------------------------
        self.is_running = False
        self.shutdown_flag = False
        self.thread_lock = threading.Lock()

        # ------------------------------------------------------------
        # YAML Pose 読み込み
        # ------------------------------------------------------------
        self.package_name = rospy.get_param("~package_name", "xarm_apriltag_auto")
        self.yaml_relative_path = rospy.get_param("~poses_yaml_rel", "config/pick_and_place_pose.yaml")
        self.yaml_root_key = rospy.get_param("~poses_root_key", "pick_and_place")

        self.pose_table = load_poses_from_yaml(
            self.package_name,
            self.yaml_relative_path,
            self.yaml_root_key
        )

        # ------------------------------------------------------------
        # 実行シーケンス
        # ------------------------------------------------------------
        self.pose_sequence = rospy.get_param("~sequence", ["target1", "target2"])
        self.wait_time_per_pose = rospy.get_param("~wait_time_per_action", 3.0)

        # ------------------------------------------------------------
        # Pick & Place 動作スレッド開始
        # ------------------------------------------------------------
        self.worker_thread = threading.Thread(target=self.pick_and_place_loop, daemon=True)
        self.worker_thread.start()

        rospy.on_shutdown(self._on_shutdown)
        rospy.spin()

    # ====================================================================
    # MoveIt Utility (plan → execute)
    # ====================================================================
    def plan_and_execute(self, target_pose: Pose):
        """
        MoveIt plan → execute を安全パターンで実行
        """
        self.arm.stop()
        self.arm.clear_pose_targets()
        self.arm.set_start_state_to_current_state()
        self.arm.set_pose_target(target_pose)

        rospy.loginfo("[P&P] Planning...")
        plan_result = self.arm.plan()

        # MoveIt のバージョン差による返り値形式の吸収
        if isinstance(plan_result, tuple):
            success_flag, trajectory = plan_result[0], plan_result[1]
        else:
            success_flag = plan_result is not None
            trajectory = plan_result

        if not success_flag or trajectory is None:
            rospy.logwarn("[P&P] Planning failed.")
            return False

        rospy.loginfo("[P&P] Executing...")
        self.arm.execute(trajectory, wait=True)
        return True

    # ====================================================================
    # Main Loop
    # ====================================================================
    def pick_and_place_loop(self):
        """
        Pick & Place のメインループ。
        YAMLで指定された pose_sequence の順番に MoveIt 移動を実行。
        """

        # ---- 最初の1回だけ手動 Enter ----
        try:
            input("ピックアンドプレース動作を開始するには Enter を押してください...")
        except EOFError:
            rospy.logwarn("標準入力が利用できないため、自動的に開始します。")

        rospy.loginfo("Pick & Place loop started.")

        loop_rate = rospy.Rate(10)

        while not rospy.is_shutdown() and not self.shutdown_flag:

            # 他ノードによるロック
            if rospy.get_param(MOTION_LOCK_PARAM, False):
                rospy.loginfo_throttle(5.0, "他ノードがロボットを占有中のため待機中...")
                loop_rate.sleep()
                continue

            try:
                with self.thread_lock:
                    rospy.set_param(MOTION_BUSY_PARAM, True)
                    rospy.loginfo("=== Pick & Place サイクル開始 ===")

                    for pose_name in self.pose_sequence:

                        # 動作中にロックが入ったら中断
                        if rospy.get_param(MOTION_LOCK_PARAM, False):
                            rospy.loginfo("外部ロック検出。中断します。")
                            break

                        if pose_name not in self.pose_table:
                            rospy.logwarn(f"YAML に '{pose_name}' が存在しません。スキップします。")
                            continue

                        target_pose = self.pose_table[pose_name]
                        rospy.loginfo(f"[P&P] Move to '{pose_name}': {target_pose}")

                        moved = self.plan_and_execute(target_pose)
                        if not moved:
                            rospy.logwarn(f"[P&P] '{pose_name}' の移動に失敗。")
                            break

                        rospy.sleep(self.wait_time_per_pose)

                    rospy.set_param(MOTION_BUSY_PARAM, False)

            except Exception as error:
                rospy.logwarn(f"[P&P] ループ処理中にエラー発生: {error}")
                rospy.set_param(MOTION_BUSY_PARAM, False)

            loop_rate.sleep()

    # ====================================================================
    # Shutdown
    # ====================================================================
    def _on_shutdown(self):
        """
        シャットダウン処理：ロボット動作を安全停止
        """
        self.shutdown_flag = True

        try:
            self.arm.stop()
            self.arm.clear_pose_targets()
        except Exception:
            pass

        moveit_commander.roscpp_shutdown()


# ====================================================================
# Main
# ====================================================================
if __name__ == "__main__":
    PickAndPlace()
