#!/usr/bin/env python3
import rospy, cv2, numpy as np, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray


class RoiStaticFixer:
    """
    Depth画像を一定期間収集し、動きが最も少ない領域を推定して
    ROI（正規化座標）として固定出力するノード。
    """

    def __init__(self):
        rospy.init_node("roi_static_fixer")

        # ------------------------------------------------------
        # 初期化
        # ------------------------------------------------------
        self.bridge = CvBridge()
        self.collected_frames = []
        self.collection_duration = rospy.get_param("~duration", 3.0)

        # 終了フラグ
        self.is_roi_fixed = False
        self.collection_start_time = time.time()

        # 入出力トピック
        self.roi_topic_out = rospy.get_param("~roi_output_topic", "/monitor/roi")
        self.depth_topic_in = rospy.get_param("~depth_topic", "/cam_2/depth/image_rect_raw")

        self.roi_publisher = rospy.Publisher(self.roi_topic_out, Float32MultiArray, queue_size=1)

        # ★追加：初期ROI（例: "0.3 0.3 0.6 0.6"）
        # initial_roi_str = rospy.get_param("~initial_roi", None)
        initial_roi_str = rospy.get_param("~initial_roi", "0.2 0.7 0.4 0.9")

        # 初期ROIが与えられていれば即適用
        if initial_roi_str is not None:
            parts = initial_roi_str.split()
            if len(parts) == 4:
                xmin, ymin, xmax, ymax = map(float, parts)
                roi_msg = Float32MultiArray(data=[xmin, ymin, xmax, ymax])

                # ★subscriber を待つ
                while self.roi_publisher.get_num_connections() == 0 and not rospy.is_shutdown():
                    rospy.loginfo("Waiting subscriber for /monitor/roi ...")
                    rospy.sleep(0.1)

                self.roi_publisher.publish(roi_msg)
                rospy.loginfo(
                    f"Initial ROI applied (normalized): "
                    f"{xmin:.3f}, {ymin:.3f}, {xmax:.3f}, {ymax:.3f}"
                )
                self.is_roi_fixed = True
                self.collected_frames.clear()

        rospy.Subscriber(self.depth_topic_in, Image, self._depth_callback)

        rospy.loginfo(f"Collecting depth frames for {self.collection_duration:.1f} sec...")
        rospy.spin()

    # ==========================================================
    # Depth画像コールバック
    # ==========================================================
    def _depth_callback(self, depth_msg):
        """
        depth画像を一定時間分収集し、動きが最小の領域（静的領域）をROI化する。
        """
        if self.is_roi_fixed:
            return

        depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1")
        if depth_image is None:
            return

        # フレームをバッファへ追加
        self.collected_frames.append(depth_image.copy())

        # 収集期間が過ぎていない場合はここで終了
        if time.time() - self.collection_start_time < self.collection_duration:
            return

        # ------------------------------------------------------
        # ROI 推定処理
        # ------------------------------------------------------
        rospy.loginfo("Analyzing static region based on temporal depth differences...")

        depth_diffs = [
            cv2.absdiff(self.collected_frames[idx], self.collected_frames[idx - 1])
            for idx in range(1, len(self.collected_frames))
        ]
        max_motion_map = np.max(np.stack(depth_diffs), axis=0)
        max_motion_map[np.isnan(max_motion_map)] = 0

        motion_map_norm = cv2.normalize(max_motion_map, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        img_height, img_width = motion_map_norm.shape

        target_roi_ratio = 0.05  # 最終ROIは画像全体の15%
        total_pixels = img_height * img_width
        target_pixel_count = int(total_pixels * target_roi_ratio)

        sorted_motion_vals = np.sort(motion_map_norm.flatten())
        threshold_index = min(len(sorted_motion_vals) - 1, target_pixel_count)
        threshold_value = sorted_motion_vals[threshold_index]

        _, static_mask = cv2.threshold(
            motion_map_norm, threshold_value, 255, cv2.THRESH_BINARY_INV
        )
        static_mask = cv2.medianBlur(static_mask, 5)

        # ------------------------------------------------------
        # 最大連結成分を ROI とする
        # ------------------------------------------------------
        contours, _ = cv2.findContours(static_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            rospy.logwarn("No stable region detected.")
            return

        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        current_area_ratio = (w * h) / (img_width * img_height)

        if current_area_ratio > 0.2:
            adjust_scale = (target_roi_ratio / current_area_ratio) ** 0.5
            new_w = int(w * adjust_scale)
            new_h = int(h * adjust_scale)

            x = max(0, x + (w - new_w) // 2)
            y = max(0, y + (h - new_h) // 2)
            w, h = new_w, new_h

        # ------------------------------------------------------
        # ROI を正規化座標で表現
        # ------------------------------------------------------
        xmin = x / img_width
        ymin = y / img_height
        xmax = (x + w) / img_width
        ymax = (y + h) / img_height

        # ------------------------------------------------------
        # 出力と固定
        # ------------------------------------------------------
        roi_msg = Float32MultiArray(data=[xmin, ymin, xmax, ymax])
        self.roi_publisher.publish(roi_msg)

        rospy.set_param("/roi/fixed", [xmin, ymin, xmax, ymax])
        # rospy.set_param("/roi/fixed", {"xmin": xmin, "ymin": ymin, "xmax": xmax, "ymax": ymax})
        rospy.loginfo(
            f"ROI fixed (normalized): "
            f"{xmin:.3f}, {ymin:.3f}, {xmax:.3f}, {ymax:.3f}"
        )

        # 状態更新
        self.is_roi_fixed = True
        self.collected_frames.clear()


if __name__ == "__main__":
    RoiStaticFixer()
