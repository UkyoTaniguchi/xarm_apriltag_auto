#!/usr/bin/env python3
import rospy
import tf
import tf.transformations as tft
import numpy as np
import math
import re
import os

from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Empty


# ============================================================
#  ファイルパス関連ユーティリティ
# ============================================================

def get_default_tag_file(tag_id):
    """Tag姿勢ファイルのデフォルト保存パス"""
    return f"/home/robot/nishidalab_ws/src/3_utils/xarm_apriltag_auto/public/tag_{tag_id}_pose.txt"


# Position/Orientation の行検出用パターン
_POS_PATTERN = re.compile(r"Position:\s*x=([-\d\.eE]+),\s*y=([-\d\.eE]+),\s*z=([-\d\.eE]+)")
_ORI_PATTERN = re.compile(r"Orientation:\s*x=([-\d\.eE]+),\s*y=([-\d\.eE]+),\s*z=([-\d\.eE]+),\s*w=([-\d\.eE]+)")

# ============================================================
#  tag_pose ファイル内の姿勢抽出ユーティリティ
# ============================================================

def _extract_pose_from_lines(lines, start_idx):
    """
    指定開始行以降から Position → Orientation を1回だけ検出して返す。
    """
    position = None
    orientation = None

    for line_index in range(start_idx, len(lines)):
        line = lines[line_index]

        # Position
        if position is None:
            match = _POS_PATTERN.search(line)
            if match:
                position = tuple(float(match.group(k)) for k in (1, 2, 3))
                continue

        # Orientation
        if position is not None and orientation is None:
            match = _ORI_PATTERN.search(line)
            if match:
                orientation = tuple(float(match.group(k)) for k in (1, 2, 3, 4))
                break

    return position, orientation


# ============================================================
#  tag_pose ファイル読み込み
# ============================================================

def load_tag_and_camera_pose(file_path, tag_id, ref_camera_frame):
    """
    tag_pose.txt から以下を取得:
      1) Tag ID に対応する Tag→Camera の姿勢 (translation, quaternion)
      2) world→ref_camera_frame の姿勢 (position, quaternion)
    """

    if not os.path.exists(file_path):
        raise FileNotFoundError(f"{file_path} が見つかりません。")

    # ----------------------------------------------
    # ファイル読み込み
    # ----------------------------------------------
    with open(file_path, "r") as file_handle:
        lines = [line.strip() for line in file_handle.readlines()]

    # ----------------------------------------------
    # Tag ID ブロックの探索
    # ----------------------------------------------
    tag_block_index = None

    for line_index, line in enumerate(lines):
        if line.startswith("Tag ID:"):
            try:
                parsed_tag_id = int(line.split(":")[1].strip())
                if parsed_tag_id == tag_id:
                    tag_block_index = line_index
                    break
            except Exception:
                continue

    if tag_block_index is None:
        raise ValueError(f"Tag ID {tag_id} の記述が見つかりません。")

    # Tag → Camera の姿勢抽出
    tag_position, tag_quaternion_raw = _extract_pose_from_lines(lines, tag_block_index)
    if tag_position is None or tag_quaternion_raw is None:
        raise ValueError("Tag の Position/Orientation を解釈できませんでした。")

    # ----------------------------------------------
    # 親カメラ(ref_camera_frame) の姿勢ブロック探索
    # ----------------------------------------------
    search_key = f"{ref_camera_frame} w.r.t world"
    camera_block_index = None

    for line_index, line in enumerate(lines):
        if search_key in line:
            camera_block_index = line_index
            break

    if camera_block_index is None:
        raise ValueError(f"'{search_key}' の行が見つかりません。")

    # 親カメラの姿勢抽出
    camera_position, camera_quaternion = _extract_pose_from_lines(lines, camera_block_index)
    if camera_position is None or camera_quaternion is None:
        raise ValueError("親カメラの Position/Orientation を解釈できませんでした。")

    # ----------------------------------------------
    # ログ
    # ----------------------------------------------
    rospy.loginfo("=== [Loaded Tag Pose] ===")
    rospy.loginfo(f"Tag ID {tag_id}: position={tag_position}, quaternion={tag_quaternion_raw}")

    # ----------------------------------------------
    # numpy化して返却
    # ----------------------------------------------
    return (
        np.array(tag_position),
        tuple(tag_quaternion_raw),
        np.array(camera_position),
        tuple(camera_quaternion)
    )



# ============================================================
#  カメラTFの計算（元ロジックを完全維持）
# ============================================================

def compute_world_to_camera_tf(tag_position_in_camera,
                               tag_quaternion_raw,
                               world_to_reference_position,
                               world_to_reference_quaternion):
    """
    Tag→Camera の観測値を元に、world → target_camera_frame の姿勢を計算する。
    ※計算ロジックは元コードを完全に維持。
    """

    # ----------------------------------------------------------
    # 1) RealSense座標系補正（pitch -90°, roll -90°）
    # ----------------------------------------------------------
    pitch_correction = tft.quaternion_from_euler(0, math.radians(-90), 0)
    roll_correction  = tft.quaternion_from_euler(math.radians(-90), 0, 0)

    correction_quaternion = tft.quaternion_multiply(pitch_correction, roll_correction)

    # TagがCamera座標から見たときの補正後クォータニオン
    tag_quaternion_corrected = tft.quaternion_multiply(
        tag_quaternion_raw,
        correction_quaternion
    )

    # ----------------------------------------------------------
    # 2) Camera座標系における実機オフセットの反映
    # ----------------------------------------------------------
    camera_local_offset = np.array([-0.03, 0.0175, -0.0525])

    rotation_matrix_camera = tft.quaternion_matrix(tag_quaternion_corrected)[:3, :3]
    offset_in_camera_world = rotation_matrix_camera @ camera_local_offset

    # Tag位置（Camera目線）にオフセットを加えたCamera位置
    camera_position_relative = np.array(tag_position_in_camera) + offset_in_camera_world

    # ----------------------------------------------------------
    # 3) world → target_camera_frame の計算
    # ----------------------------------------------------------

    # 回転成分：world→reference_camera の後に camera_local→target_camera 変換を適用
    world_to_target_quaternion = tft.quaternion_multiply(
        world_to_reference_quaternion,
        tag_quaternion_corrected
    )

    # 並進成分：reference_camera の回転を使って offset を world 座標へ写す
    rotation_matrix_reference = tft.quaternion_matrix(world_to_reference_quaternion)[:3, :3]
    camera_offset_in_world = rotation_matrix_reference @ camera_position_relative

    world_to_target_position = world_to_reference_position + camera_offset_in_world

    return world_to_target_position, world_to_target_quaternion


# ============================================================
#  TF ブロードキャスト本体
# ============================================================

def run_tf_broadcaster():
    rospy.init_node('camera_to_world_broadcaster')

    transform_broadcaster = tf.TransformBroadcaster()
    publish_rate = rospy.Rate(15.0)

    # ----------------------------------------------------------
    # パラメータ取得
    # ----------------------------------------------------------
    reference_camera_frame = rospy.get_param("~parent_frame", "cam_1_color_optical_frame")
    target_camera_frame    = rospy.get_param("~output_frame", "cam_3_link")
    target_tag_id          = rospy.get_param("~tag_id", 1)
    tag_pose_file_path     = rospy.get_param(
        "~tag_file",
        get_default_tag_file(target_tag_id)
    )

    # ----------------------------------------------------------
    # 初回の TF 計算
    # ----------------------------------------------------------
    tag_position, tag_quaternion_raw, ref_cam_position, ref_cam_quaternion = \
        load_tag_and_camera_pose(
            tag_pose_file_path,
            target_tag_id,
            reference_camera_frame
        )

    world_to_target_position, world_to_target_quaternion = \
        compute_world_to_camera_tf(
            tag_position,
            tag_quaternion_raw,
            ref_cam_position,
            ref_cam_quaternion
        )

    rospy.loginfo(f"[Loaded] {tag_pose_file_path} (Tag ID={target_tag_id})")
    rospy.loginfo(
        f"world → {target_camera_frame}: "
        f"position={world_to_target_position}, quaternion={world_to_target_quaternion}"
    )

    # ----------------------------------------------------------
    # TF再読み込み要求を受けたら再計算 → ノード再起動（respawn）へ
    # ----------------------------------------------------------
    reload_requested = False

    def handle_tf_reload(_msg):
        nonlocal world_to_target_position, world_to_target_quaternion, reload_requested

        rospy.loginfo("=== /recalib/reload_tf を受信。tag_pose を再読み込みします ===")

        try:
            new_tag_position, new_tag_quaternion_raw, new_ref_position, new_ref_quaternion = \
                load_tag_and_camera_pose(
                    tag_pose_file_path,
                    target_tag_id,
                    reference_camera_frame
                )

            new_world_position, new_world_quaternion = compute_world_to_camera_tf(
                new_tag_position,
                new_tag_quaternion_raw,
                new_ref_position,
                new_ref_quaternion
            )

            world_to_target_position = new_world_position
            world_to_target_quaternion = new_world_quaternion

            rospy.loginfo("=== TF 更新完了 ===")
            reload_requested = True

        except Exception as error:
            rospy.logerr(f"TF再計算に失敗: {error}")

    rospy.Subscriber("/recalib/reload_tf", Empty, handle_tf_reload)

    # ----------------------------------------------------------
    # TFブロードキャストループ
    # ----------------------------------------------------------
    while not rospy.is_shutdown():
        if reload_requested:
            rospy.loginfo("=== TF更新後、ノードを終了します（respawn=true を想定） ===")
            return  # ノード終了 → launch の respawn が自動再起動させる

        transform_broadcaster.sendTransform(
            world_to_target_position,
            world_to_target_quaternion,
            rospy.Time.now(),
            target_camera_frame,
            "world"
        )

        publish_rate.sleep()



if __name__ == '__main__':
    try:
        run_tf_broadcaster()
    except rospy.ROSInterruptException:
        pass
