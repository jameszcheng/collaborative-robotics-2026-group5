#!/usr/bin/env python3
"""
Object Detection Node (Real Hardware, 2D + optional 3D pose).

Runs YOLO on RGB images and publishes whether a target object is found
plus a 2D bounding box. If depth + CameraInfo + TF are available, also
publishes a 3D point pose in a world frame.

Publishes:
  - /perception/object_found (std_msgs/Bool)
  - /perception/object_label (std_msgs/String)
  - /perception/object_confidence (std_msgs/Float32)
  - /perception/object_bbox (std_msgs/Int32MultiArray, [x, y, w, h])
  - /perception/object_debug_image (sensor_msgs/Image, optional)
  - /perception/object_pose (geometry_msgs/PoseStamped, optional)

Subscribes:
  - /camera/color/image_raw (sensor_msgs/Image)
  - /camera/aligned_depth_to_color/image_raw (sensor_msgs/Image)
  - /camera/color/camera_info (sensor_msgs/CameraInfo)
  - /perception/target_label (std_msgs/String) [optional runtime target override]

Usage:
  ros2 run tidybot_bringup detect_object_real.py
"""

import os
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32, Int32MultiArray, String
import tf2_ros


def deproject_uvz_to_camera_xyz(
    u: float,
    v: float,
    z_m: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
) -> Tuple[float, float, float]:
    """Back-project pixel/depth to camera optical coordinates."""
    x_cam = (u - cx) * z_m / fx
    y_cam = (v - cy) * z_m / fy
    return x_cam, y_cam, z_m


def quat_to_rot_matrix(qx: float, qy: float, qz: float, qw: float):
    """Convert quaternion to a 3x3 rotation matrix."""
    xx = qx * qx
    yy = qy * qy
    zz = qz * qz
    xy = qx * qy
    xz = qx * qz
    yz = qy * qz
    wx = qw * qx
    wy = qw * qy
    wz = qw * qz

    r00 = 1.0 - 2.0 * (yy + zz)
    r01 = 2.0 * (xy - wz)
    r02 = 2.0 * (xz + wy)
    r10 = 2.0 * (xy + wz)
    r11 = 1.0 - 2.0 * (xx + zz)
    r12 = 2.0 * (yz - wx)
    r20 = 2.0 * (xz - wy)
    r21 = 2.0 * (yz + wx)
    r22 = 1.0 - 2.0 * (xx + yy)
    return (
        (r00, r01, r02),
        (r10, r11, r12),
        (r20, r21, r22),
    )


def transform_point(tf_msg, point_xyz: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Apply rigid transform to a 3D point."""
    x, y, z = point_xyz
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    r = quat_to_rot_matrix(q.x, q.y, q.z, q.w)
    x_w = r[0][0] * x + r[0][1] * y + r[0][2] * z + t.x
    y_w = r[1][0] * x + r[1][1] * y + r[1][2] * z + t.y
    z_w = r[2][0] * x + r[2][1] * y + r[2][2] * z + t.z
    return x_w, y_w, z_w


class ObjectDetectorNode(Node):
    """Detect target object class from RGB stream using YOLO."""

    def __init__(self):
        super().__init__("object_detector")

        self.declare_parameter("rgb_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("world_frame", "odom")
        self.declare_parameter("target_label", "apple")
        self.declare_parameter("model_path", "yolov8n.pt")
        self.declare_parameter("conf_threshold", 0.35)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("save_debug_image", True)
        self.declare_parameter(
            "debug_image_path",
            os.path.join(os.getcwd(), "captures", "latest_detection.jpg"),
        )

        self.rgb_topic = self.get_parameter("rgb_topic").value
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.target_label = str(self.get_parameter("target_label").value).strip().lower()
        self.model_path = self.get_parameter("model_path").value
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.save_debug_image = bool(self.get_parameter("save_debug_image").value)
        self.debug_image_path = str(self.get_parameter("debug_image_path").value)
        if self.save_debug_image:
            os.makedirs(os.path.dirname(os.path.abspath(self.debug_image_path)), exist_ok=True)

        self.bridge = CvBridge()
        self.last_found_state = None
        self.last_log_time = 0.0
        self.last_pose_warn_time = 0.0

        self.latest_depth = None
        self.depth_encoding = ""
        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.model = None
        self.model_names = {}
        self._init_model()

        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, self.rgb_topic, self.rgb_cb, qos)
        self.create_subscription(Image, self.depth_topic, self.depth_cb, qos)
        self._camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_cb, 10)
        self.create_subscription(String, "/perception/target_label", self.target_label_cb, 10)

        self.found_pub = self.create_publisher(Bool, "/perception/object_found", 10)
        self.label_pub = self.create_publisher(String, "/perception/object_label", 10)
        self.conf_pub = self.create_publisher(Float32, "/perception/object_confidence", 10)
        self.bbox_pub = self.create_publisher(Int32MultiArray, "/perception/object_bbox", 10)
        self.debug_pub = self.create_publisher(Image, "/perception/object_debug_image", qos)
        self.pose_pub = self.create_publisher(PoseStamped, "/perception/object_pose", 10)

        self.get_logger().info("Object detector started")
        self.get_logger().info(f"  RGB: {self.rgb_topic}")
        self.get_logger().info(f"  Depth: {self.depth_topic}")
        self.get_logger().info(f"  CameraInfo: {self.camera_info_topic}")
        self.get_logger().info(f"  Pose frame: {self.world_frame}")
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

    def depth_cb(self, msg: Image):
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.depth_encoding = str(msg.encoding).lower()
        except Exception as exc:
            self.latest_depth = None
            self._warn_pose_throttled(f"Depth conversion failed: {exc}")

    def camera_info_cb(self, msg: CameraInfo):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        self.get_logger().info(
            f'Camera intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} '
            f'cx={self.cx:.1f} cy={self.cy:.1f}'
        )
        self.destroy_subscription(self._camera_info_sub)

    def _warn_pose_throttled(self, text: str):
        now = time.time()
        if now - self.last_pose_warn_time > 1.0:
            self.last_pose_warn_time = now
            self.get_logger().warn(text)

    def _depth_at_uv_m(self, u: int, v: int, radius: int = 10) -> Optional[float]:
        if self.latest_depth is None:
            return None

        h, w = self.latest_depth.shape[:2]
        u0 = max(0, u - radius)
        u1 = min(w, u + radius + 1)
        v0 = max(0, v - radius)
        v1 = min(h, v + radius + 1)

        patch = self.latest_depth[v0:v1, u0:u1].astype(np.float32)
        valid = patch[(patch > 0) & np.isfinite(patch)]
        if valid.size == 0:
            return None

        z = float(np.min(valid))  # nearest valid depth in patch
        # RealSense depth is usually 16UC1 in millimeters.
        if self.depth_encoding in ("16uc1", "mono16"):
            z = z / 1000.0
        return z

    def _publish_pose(self, x: int, y: int, w: int, h: int, image_header):
        if any(k is None for k in [self.fx, self.fy, self.cx, self.cy]):
            self._warn_pose_throttled("No camera intrinsics yet; skipping /perception/object_pose.")
            return

        u = int(round(x + 0.5 * w))
        v = int(round(y + 0.5 * h))
        z_m = self._depth_at_uv_m(u, v)
        if z_m is None:
            self._warn_pose_throttled("Invalid depth at bbox center; skipping /perception/object_pose.")
            return

        x_cam, y_cam, z_cam = deproject_uvz_to_camera_xyz(
            float(u), float(v), z_m, self.fx, self.fy, self.cx, self.cy
        )

        try:
            tf_msg = self.tf_buffer.lookup_transform(
                self.world_frame,
                self.camera_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0),
            )
        except Exception as exc:
            self._warn_pose_throttled(
                f"TF lookup failed ({self.world_frame} <- {self.camera_frame}): {exc}"
            )
            return

        x_w, y_w, z_w = transform_point(tf_msg, (x_cam, y_cam, z_cam))

        pose_msg = PoseStamped()
        pose_msg.header.stamp = image_header.stamp
        pose_msg.header.frame_id = self.world_frame
        pose_msg.pose.position.x = float(x_w)
        pose_msg.pose.position.y = float(y_w)
        pose_msg.pose.position.z = float(z_w)
        pose_msg.pose.orientation.w = 1.0
        self.pose_pub.publish(pose_msg)

    def rgb_cb(self, msg: Image):
        if self.model is None:
            self.publish_found(False)
            self.publish_label("")
            self.publish_confidence(0.0)
            self.publish_bbox((0, 0, 0, 0))
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"RGB conversion failed: {exc}")
            self.publish_found(False)
            self.publish_label("")
            self.publish_confidence(0.0)
            self.publish_bbox((0, 0, 0, 0))
            return

        detection = self.detect_target(bgr)
        if detection is None:
            self.publish_found(False)
            self.publish_label("")
            self.publish_confidence(0.0)
            self.publish_bbox((0, 0, 0, 0))
            if self.publish_debug_image:
                self.publish_debug(bgr, None, msg.header)
            return

        x, y, w, h, conf, label = detection
        self.publish_found(True)
        self.publish_label(label)
        self.publish_confidence(conf)
        self.publish_bbox((x, y, w, h))
        self._publish_pose(x, y, w, h, msg.header)
        if self.publish_debug_image:
            self.publish_debug(bgr, detection, msg.header)

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
        msg = Bool()
        msg.data = found
        self.found_pub.publish(msg)
        self.last_found_state = found

    def publish_label(self, label: str):
        msg = String()
        msg.data = label
        self.label_pub.publish(msg)

    def publish_confidence(self, confidence: float):
        msg = Float32()
        msg.data = float(confidence)
        self.conf_pub.publish(msg)

    def publish_bbox(self, bbox: Tuple[int, int, int, int]):
        msg = Int32MultiArray()
        msg.data = [int(bbox[0]), int(bbox[1]), int(bbox[2]), int(bbox[3])]
        self.bbox_pub.publish(msg)

    def publish_debug(
        self,
        bgr: "cv2.typing.MatLike",
        detection: Optional[Tuple[int, int, int, int, float, str]],
        image_header,
    ):
        vis = bgr.copy()
        if detection is not None:
            x, y, w, h, conf, label = detection
            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 220, 0), 2)
            text = f"{label} {conf:.2f}"
            cv2.putText(vis, text, (x, max(25, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 0), 2)

        debug_msg = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
        debug_msg.header = image_header
        self.debug_pub.publish(debug_msg)

        if self.save_debug_image:
            cv2.imwrite(self.debug_image_path, vis)


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
