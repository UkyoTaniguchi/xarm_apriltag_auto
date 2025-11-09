#!/usr/bin/env python3
import rospy, cv2, numpy as np, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray

class RoiStaticFixer:
    def __init__(self):
        rospy.init_node("roi_static_fixer")
        self.bridge = CvBridge()
        self.buffer = []
        self.duration = rospy.get_param("~duration", 3.0)
        self.done = False
        self.start_time = time.time()

        self.pub_roi = rospy.Publisher("/monitor/roi", Float32MultiArray, queue_size=1)
        rospy.Subscriber("/camera/camera/depth/image_rect_raw", Image, self.depth_cb)
        rospy.loginfo(f"Collecting depth frames for {self.duration:.1f} sec...")
        rospy.spin()

    def depth_cb(self, msg):
        if self.done:
            return

        depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        if depth is None:
            return
        self.buffer.append(depth.copy())

        if time.time() - self.start_time < self.duration:
            return

        rospy.loginfo("Analyzing static region...")
        diffs = [cv2.absdiff(self.buffer[i], self.buffer[i - 1]) for i in range(1, len(self.buffer))]
        M = np.max(np.stack(diffs), axis=0)
        M[np.isnan(M)] = 0
        M_norm = cv2.normalize(M, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
        H, W = M_norm.shape

        # ---- 動かなかった領域 = 下位 p% を抽出 ----
        # 目標ROI面積 ≈ 全体の5%
        target_ratio = 0.05
        num_pixels = H * W
        target_pixels = int(num_pixels * target_ratio)

        # 差分分布に基づく動きの少ない画素を閾値で抽出
        sorted_vals = np.sort(M_norm.flatten())
        thresh_idx = min(len(sorted_vals) - 1, target_pixels)
        thresh_val = sorted_vals[thresh_idx]
        _, mask = cv2.threshold(M_norm, thresh_val, 255, cv2.THRESH_BINARY_INV)
        mask = cv2.medianBlur(mask, 5)

        # ---- 最大連結成分をROIに ----
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            rospy.logwarn("No stable region detected.")
            return

        # 最大面積の連結成分を選択
        c = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(c)

        # ROI矩形を15%以内に調整（必要に応じて拡大縮小）
        current_ratio = (w * h) / (W * H)
        if current_ratio > 0.2:
            scale = (target_ratio / current_ratio) ** 0.5
            new_w, new_h = int(w * scale), int(h * scale)
            x = max(0, x + (w - new_w) // 2)
            y = max(0, y + (h - new_h) // 2)
            w, h = new_w, new_h

        # ---- 正規化座標に変換 ----
        xmin = x / W
        ymin = y / H
        xmax = (x + w) / W
        ymax = (y + h) / H

        # ---- 公開・固定 ----
        msg_roi = Float32MultiArray(data=[xmin, ymin, xmax, ymax])
        self.pub_roi.publish(msg_roi)
        rospy.set_param("/roi/fixed", [xmin, ymin, xmax, ymax])
        rospy.loginfo(f"ROI fixed (normalized): {xmin:.3f}, {ymin:.3f}, {xmax:.3f}, {ymax:.3f}")

        self.done = True
        self.buffer.clear()

if __name__ == "__main__":
    RoiStaticFixer()
