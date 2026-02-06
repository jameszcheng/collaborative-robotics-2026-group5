#!/usr/bin/env python3
"""
Banana Perception Node (Real Hardware).

Detects banana-like yellow regions in RGB images and estimates a 3D position
using depth + camera intrinsics.

Publishes:
  - /perception/banana_pose (geometry_msgs/PoseStamped)
  - /perception/banana_found (std_msgs/Bool)
  - /perception/banana_debug_image (sensor_msgs/Image, optional)

Subscribes:
  - /camera/color/image_raw (sensor_msgs/Image)
  - /camera/depth/image_raw (sensor_msgs/Image)
  - /camera/color/camera_info (sensor_msgs/CameraInfo)

Usage:
  ros2 run tidybot_bringup detect_banana_real.py
"""

import math
import time
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool


class BananaDetectorNode(Node):
    """Perceive banana-like objects from RGB + depth."""

    def __init__(self):
        super().__init__("banana_detector")

        self.declare_parameter("rgb_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("min_contour_area_px", 700.0)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("max_depth_m", 2.0)

        self.rgb_topic = self.get_parameter("rgb_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.camera_info_topic = self.get_parameter("camera_info_topic").value
        self.min_contour_area_px = float(self.get_parameter("min_contour_area_px").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.max_depth_m = float(self.get_parameter("max_depth_m").value)

        self.bridge = CvBridge()
        self.latest_depth: Optional[np.ndarray] = None
        self.latest_depth_stamp = None
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.camera_frame = "camera_color_optical_frame"
        self.last_found_state = False
        self.last_log_time = 0.0

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, self.rgb_topic, self.rgb_cb, qos)
        self.create_subscription(Image, self.depth_topic, self.depth_cb, qos)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.camera_info_cb, 10)

        self.pose_pub = self.create_publisher(PoseStamped, "/perception/banana_pose", 10)
        self.found_pub = self.create_publisher(Bool, "/perception/banana_found", 10)
        self.debug_pub = self.create_publisher(Image, "/perception/banana_debug_image", qos)

        self.get_logger().info("Banana detector started")
        self.get_logger().info(f"  RGB: {self.rgb_topic}")
        self.get_logger().info(f"  Depth: {self.depth_topic}")
        self.get_logger().info(f"  CameraInfo: {self.camera_info_topic}")

    def camera_info_cb(self, msg: CameraInfo):
        self.fx = msg.k[0]
        self.fy = msg.k[4]
        self.cx = msg.k[2]
        self.cy = msg.k[5]
        self.camera_frame = msg.header.frame_id or self.camera_frame

    def depth_cb(self, msg: Image):
        try:
            if msg.encoding == "16UC1":
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="16UC1")
                self.latest_depth = depth.astype(np.float32) / 1000.0
            elif msg.encoding == "32FC1":
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC1")
                self.latest_depth = depth.astype(np.float32)
            else:
                depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                self.latest_depth = depth.astype(np.float32)
            self.latest_depth_stamp = msg.header.stamp
        except Exception as exc:
            self.get_logger().warn(f"Depth conversion failed: {exc}")

    def rgb_cb(self, msg: Image):
        if self.fx is None or self.latest_depth is None:
            self.publish_found(False)
            return

        try:
            rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        except Exception as exc:
            self.get_logger().warn(f"RGB conversion failed: {exc}")
            self.publish_found(False)
            return

        found, center_xy, contour, mask = self.detect_banana(rgb)
        if not found:
            self.publish_found(False)
            if self.publish_debug_image:
                self.publish_debug(rgb, None, None, mask)
            return

        xyz = self.estimate_3d(center_xy)
        if xyz is None:
            self.publish_found(False)
            if self.publish_debug_image:
                self.publish_debug(rgb, center_xy, contour, mask)
            return

        self.publish_pose(msg, xyz)
        self.publish_found(True)
        if self.publish_debug_image:
            self.publish_debug(rgb, center_xy, contour, mask)

        now = time.time()
        if now - self.last_log_time > 1.0:
            self.last_log_time = now
            self.get_logger().info(
                f"Banana at (x={xyz[0]:.3f}, y={xyz[1]:.3f}, z={xyz[2]:.3f}) in {self.camera_frame}"
            )

    def detect_banana(self, rgb: np.ndarray) -> Tuple[bool, Optional[Tuple[int, int]], Optional[np.ndarray], np.ndarray]:
        """Detect the largest banana-like yellow contour."""
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        # Wide yellow window for bananas under varied lighting.
        lower = np.array([15, 60, 60], dtype=np.uint8)
        upper = np.array([40, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lower, upper)

        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return False, None, None, mask

        best = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(best)
        if area < self.min_contour_area_px:
            return False, None, None, mask

        x, y, w, h = cv2.boundingRect(best)
        aspect = max(float(w), float(h)) / max(1.0, min(float(w), float(h)))
        if aspect < 1.15:
            return False, None, None, mask

        cx = int(x + w / 2)
        cy = int(y + h / 2)
        return True, (cx, cy), best, mask

    def estimate_3d(self, center_xy: Tuple[int, int]) -> Optional[Tuple[float, float, float]]:
        """Estimate 3D point from center pixel using depth + intrinsics."""
        if self.latest_depth is None:
            return None

        u, v = center_xy
        h, w = self.latest_depth.shape[:2]
        if u < 0 or v < 0 or u >= w or v >= h:
            return None

        patch = self.latest_depth[max(0, v - 3):min(h, v + 4), max(0, u - 3):min(w, u + 4)]
        valid = patch[np.isfinite(patch)]
        valid = valid[(valid > 0.05) & (valid < self.max_depth_m)]
        if valid.size == 0:
            return None

        z = float(np.median(valid))
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        if any(map(lambda val: math.isnan(val) or math.isinf(val), [x, y, z])):
            return None
        return x, y, z

    def publish_pose(self, rgb_msg: Image, xyz: Tuple[float, float, float]):
        pose = PoseStamped()
        pose.header.stamp = rgb_msg.header.stamp
        pose.header.frame_id = self.camera_frame
        pose.pose.position.x = float(xyz[0])
        pose.pose.position.y = float(xyz[1])
        pose.pose.position.z = float(xyz[2])
        pose.pose.orientation.w = 1.0
        self.pose_pub.publish(pose)

    def publish_found(self, found: bool):
        if found == self.last_found_state:
            return
        self.last_found_state = found
        msg = Bool()
        msg.data = found
        self.found_pub.publish(msg)

    def publish_debug(
        self,
        rgb: np.ndarray,
        center_xy: Optional[Tuple[int, int]],
        contour: Optional[np.ndarray],
        mask: np.ndarray,
    ):
        vis = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        if contour is not None:
            cv2.drawContours(vis, [contour], -1, (0, 255, 0), 2)
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(vis, (x, y), (x + w, y + h), (255, 180, 0), 2)
        if center_xy is not None:
            cv2.circle(vis, center_xy, 5, (0, 0, 255), -1)

        mask_small = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        mask_small = cv2.resize(mask_small, (vis.shape[1] // 3, vis.shape[0] // 3))
        vis[:mask_small.shape[0], :mask_small.shape[1]] = mask_small

        debug_msg = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
        debug_msg.header.stamp = self.get_clock().now().to_msg()
        debug_msg.header.frame_id = self.camera_frame
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BananaDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
