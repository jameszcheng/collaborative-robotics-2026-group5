#!/usr/bin/env python3
"""
Object Detection Node (Real Hardware, 2D only).

Runs YOLO on RGB images and publishes whether a target object is found
plus a 2D bounding box.

Publishes:
  - /perception/object_found (std_msgs/Bool)
  - /perception/object_label (std_msgs/String)
  - /perception/object_bbox (std_msgs/Int32MultiArray, [x, y, w, h])
  - /perception/object_debug_image (sensor_msgs/Image, optional)

Subscribes:
  - /camera/color/image_raw (sensor_msgs/Image)
  - /perception/target_label (std_msgs/String) [optional runtime target override]

Usage:
  ros2 run tidybot_bringup detect_object_real.py
"""

import time
from typing import List, Optional, Tuple

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32MultiArray, String


class ObjectDetectorNode(Node):
    """Detect target object class from RGB stream using YOLO."""

    def __init__(self):
        super().__init__("object_detector")

        self.declare_parameter("rgb_topic", "/camera/color/image_raw")
        self.declare_parameter("target_label", "apple")
        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("conf_threshold", 0.35)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("publish_debug_image", True)

        self.rgb_topic = self.get_parameter("rgb_topic").value
        self.target_label = str(self.get_parameter("target_label").value).strip().lower()
        self.model_path = self.get_parameter("model_path").value
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)

        self.bridge = CvBridge()
        self.last_found_state = False
        self.last_log_time = 0.0

        self.model = None
        self.model_names = {}
        self._init_model()

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, self.rgb_topic, self.rgb_cb, qos)
        self.create_subscription(String, "/perception/target_label", self.target_label_cb, 10)

        self.found_pub = self.create_publisher(Bool, "/perception/object_found", 10)
        self.label_pub = self.create_publisher(String, "/perception/object_label", 10)
        self.bbox_pub = self.create_publisher(Int32MultiArray, "/perception/object_bbox", 10)
        self.debug_pub = self.create_publisher(Image, "/perception/object_debug_image", qos)

        self.get_logger().info("Object detector started")
        self.get_logger().info(f"  RGB: {self.rgb_topic}")
        self.get_logger().info(f"  Target label: {self.target_label}")
        self.get_logger().info(f"  Model: {self.model_path}")

    def target_label_cb(self, msg: String):
        """Update target class at runtime from NLP or other planners."""
        new_label = str(msg.data).strip().lower()
        if not new_label:
            return
        if new_label == self.target_label:
            return
        self.target_label = new_label
        self.last_found_state = False
        self.get_logger().info(f"Updated target label: {self.target_label}")

    def _init_model(self):
        try:
            from ultralytics import YOLO
        except Exception as exc:
            self.get_logger().error(
                "Failed to import ultralytics. Install it with: uv add ultralytics"
            )
            self.get_logger().error(f"Import error: {exc}")
            return

        try:
            self.model = YOLO(self.model_path)
            names = getattr(self.model, "names", {})
            self.model_names = {int(k): str(v).strip().lower() for k, v in names.items()}
        except Exception as exc:
            self.get_logger().error(f"Failed to load model '{self.model_path}': {exc}")
            self.model = None
            self.model_names = {}

    def rgb_cb(self, msg: Image):
        if self.model is None:
            self.publish_found(False)
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"RGB conversion failed: {exc}")
            self.publish_found(False)
            return

        detection = self.detect_target(bgr)
        if detection is None:
            self.publish_found(False)
            if self.publish_debug_image:
                self.publish_debug(bgr, None)
            return

        x, y, w, h, conf, label = detection
        self.publish_found(True)
        self.publish_label(label)
        self.publish_bbox((x, y, w, h))
        if self.publish_debug_image:
            self.publish_debug(bgr, detection)

        now = time.time()
        if now - self.last_log_time > 1.0:
            self.last_log_time = now
            self.get_logger().info(
                f"Detected '{label}' conf={conf:.2f} bbox=({x}, {y}, {w}, {h})"
            )

    def detect_target(self, bgr: "cv2.typing.MatLike") -> Optional[Tuple[int, int, int, int, float, str]]:
        """Return highest-confidence target detection as (x, y, w, h, conf, label)."""
        try:
            results = self.model.predict(
                source=bgr,
                conf=self.conf_threshold,
                imgsz=self.imgsz,
                verbose=False,
            )
        except Exception as exc:
            self.get_logger().warn(f"YOLO inference failed: {exc}")
            return None

        if not results:
            return None

        result = results[0]
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return None

        best = None
        best_conf = -1.0

        xyxy_list: List[List[float]] = boxes.xyxy.cpu().numpy().tolist()
        conf_list: List[float] = boxes.conf.cpu().numpy().tolist()
        cls_list: List[float] = boxes.cls.cpu().numpy().tolist()

        for xyxy, conf, cls_id in zip(xyxy_list, conf_list, cls_list):
            class_name = self.model_names.get(int(cls_id), str(int(cls_id)))
            if class_name != self.target_label:
                continue

            if conf > best_conf:
                x1, y1, x2, y2 = xyxy
                x = int(round(x1))
                y = int(round(y1))
                w = max(0, int(round(x2 - x1)))
                h = max(0, int(round(y2 - y1)))
                best = (x, y, w, h, float(conf), class_name)
                best_conf = conf

        return best

    def publish_found(self, found: bool):
        if found == self.last_found_state:
            return
        self.last_found_state = found
        msg = Bool()
        msg.data = found
        self.found_pub.publish(msg)

    def publish_label(self, label: str):
        msg = String()
        msg.data = label
        self.label_pub.publish(msg)

    def publish_bbox(self, bbox: Tuple[int, int, int, int]):
        msg = Int32MultiArray()
        msg.data = [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])]
        self.bbox_pub.publish(msg)

    def publish_debug(
        self,
        bgr: "cv2.typing.MatLike",
        detection: Optional[Tuple[int, int, int, int, float, str]],
    ):
        vis = bgr.copy()
        if detection is not None:
            x, y, w, h, conf, label = detection
            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 220, 0), 2)
            text = f"{label} {conf:.2f}"
            cv2.putText(vis, text, (x, max(25, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 0), 2)

        debug_msg = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
        debug_msg.header.stamp = self.get_clock().now().to_msg()
        debug_msg.header.frame_id = "camera_color_optical_frame"
        self.debug_pub.publish(debug_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
