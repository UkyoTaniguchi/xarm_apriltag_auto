#!/usr/bin/env python3
import rospy
import moveit_commander
import geometry_msgs.msg
import copy
import math
import tf2_ros
from tf.transformations import quaternion_from_euler
# ARタグのメッセージ型をインポート
from apriltag_ros.msg import AprilTagDetectionArray 
from geometry_msgs.msg import TransformStamped

# ARタグ検出トピック
TAG_DETECTIONS_TOPIC = "/tag_detections"
# アームのプランニングフレーム（ワールド座標系として使用）
PLANNING_FRAME = "world" # または "link_base"

# アーム先端が真下を向く姿勢 (Z軸マイナス方向) を定義 (RPY = 180度, 0度, 90度)
q_fixed = quaternion_from_euler(math.pi, 0.0, math.pi / 2.0) 

def move_to_tag0_fixed_orientation():
    """
    ARタグID:0の位置をtfでワールド座標系に変換し、link_tcpの姿勢を垂直下向きに固定して移動させます。
    """
    try:
        # 1. ノードの初期化
        rospy.init_node('move_to_tag0_fixed_orientation_node', anonymous=True)
        rospy.loginfo("MoveIt!, TF, and Tag Target Node Initialized.")

        # 2. TFリスナーの初期化
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        # 3. MoveIt! の初期化
        robot = moveit_commander.RobotCommander()
        move_group = moveit_commander.MoveGroupCommander("xarm6") 
        move_group.set_max_velocity_scaling_factor(0.1)
        move_group.set_max_acceleration_scaling_factor(0.1)
        rospy.loginfo(f"Planning frame set to: {PLANNING_FRAME}")
        
        
        # 4. Tag Detections メッセージの取得 (5秒間待機)
        rospy.loginfo(f"Waiting for message on {TAG_DETECTIONS_TOPIC}...")
        # メッセージ型を AprilTagDetectionArray に変更
        detection_array_msg = rospy.wait_for_message(
            TAG_DETECTIONS_TOPIC, 
            AprilTagDetectionArray, 
            timeout=5.0
        )
        rospy.loginfo("AprilTagDetectionArray received.")

        # 5. ID 0 のタグ情報を抽出
        tag0_detection = None
        for detection in detection_array_msg.detections:
            if 0 in detection.id:
                tag0_detection = detection
                break
        
        if tag0_detection is None:
            rospy.logerr("ERROR: ID 0 のタグが検出メッセージに含まれていませんでした。")
            return

        # 6. タグのポーズを PoseStamped 形式で取得 (カメラ座標系)
        # detection.pose は PoseWithCovarianceStamped なので、その中の PoseStamped を取り出す
        tag_pose_cam_frame = geometry_msgs.msg.PoseStamped()
        tag_pose_cam_frame.header.frame_id = tag0_detection.pose.header.frame_id
        tag_pose_cam_frame.header.stamp = rospy.Time.now()
        tag_pose_cam_frame.pose = tag0_detection.pose.pose.pose
        
        # 7. 座標変換 (Camera Frame -> World Frame)
        rospy.loginfo(f"Transforming pose from {tag_pose_cam_frame.header.frame_id} to {PLANNING_FRAME}...")
        
        try:
            # tag_pose_cam_frame のポーズを PLANNING_FRAME (例: world) に変換
            # tf_buffer.transform() が PoseStamped を受け付けるため、これを使用
            target_pose_world_stamped = tf_buffer.transform(
                tag_pose_cam_frame,
                PLANNING_FRAME,
                rospy.Duration(1.0) # 変換を1秒間待機
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"ERROR: TF座標変換に失敗しました: {e}")
            return


        # 8. 目標ポーズの計算
        target_pose = target_pose_world_stamped.pose
        
        # **目標姿勢**: ワールドZ軸マイナス方向（垂直下向き）に固定 (位置はタグ中心)
        target_pose.orientation.x = q_fixed[0]
        target_pose.orientation.y = q_fixed[1]
        target_pose.orientation.z = q_fixed[2]
        target_pose.orientation.w = q_fixed[3]
        
        # ポーズ情報の確認
        rospy.loginfo(f"Target Position (World): X={target_pose.position.x:.4f}, Y={target_pose.position.y:.4f}, Z={target_pose.position.z:.4f}")
        rospy.loginfo("Target Orientation: Fixed to World Z-axis Negative (Vertical Down)")

        # 9. MoveIt! で移動実行
        # ... (移動のロジックは省略せずにそのまま残します)
        rospy.loginfo("Setting target pose and planning...")
        move_group.set_pose_target(target_pose)
        
        plan_success, plan, _, _ = move_group.plan()
        
        if plan_success and len(plan.joint_trajectory.points) > 0:
            rospy.loginfo("Planning successful. Executing move...")
            move_group.execute(plan, wait=True)
            rospy.loginfo("Movement completed successfully.")
        else:
            rospy.logerr("Planning failed. Could not find a valid path to the target pose.")
            
        move_group.stop()
        move_group.clear_pose_targets()


    except rospy.ROSException as e:
        if "timeout exceeded" in str(e):
            rospy.logerr(f"AR Tag Pose Timeout: {TAG_DETECTIONS_TOPIC} から 5.0 秒以内にメッセージを受信できませんでした。検出ノードが動作しているか確認してください。")
        else:
            rospy.logerr(f"ROS Error: {e}")
    except Exception as e:
        rospy.logerr(f"An unexpected error occurred: {e}")
        
    finally:
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == '__main__':
    try:
        move_to_tag0_fixed_orientation()
    except rospy.ROSInterruptException:
        pass