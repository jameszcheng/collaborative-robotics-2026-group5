#!/usr/bin/env python3
"""
Object Detection Node — HSV Color Segmentation (red pillow, no YOLO).

Drop-in replacement for detect_object_real.py.  Uses HSV masking instead of
YOLO to detect a red pillow on a white/neutral background.

Red spans two hue ranges in HSV (hue wraps at 180 in OpenCV):
  - Lower red: hue 0–10
  - Upper red: hue 170–180
Both masks are OR-ed together.  The largest red contour above `min_area_px`
is taken as the detection.  Confidence is the red-pixel fill ratio of the
bounding box (0–1).

Same topic interface as detect_object_real.py — a drop-in replacement.

Publishes:
  - /perception/object_found        (std_msgs/Bool)
  - /perception/object_label        (std_msgs/String)   always "pillow"
  - /perception/object_confidence   (std_msgs/Float32)  fill ratio 0–1
  - /perception/object_bbox         (std_msgs/Int32MultiArray, [x, y, w, h])
  - /perception/object_debug_image  (sensor_msgs/Image, optional)
  - /perception/object_pose         (geometry_msgs/PoseStamped, optional)

Subscribes:
  - /camera/color/image_raw                    (sensor_msgs/Image)
  - /camera/aligned_depth_to_color/image_raw   (sensor_msgs/Image)
  - /camera/color/camera_info                  (sensor_msgs/CameraInfo)
  - /perception/target_label                   (std_msgs/String) [ignored, kept for compat]

Parameters:
  hue_low1   (int, default 0)    lower red hue band — low
  hue_high1  (int, default 10)   lower red hue band — high
  hue_low2   (int, default 170)  upper red hue band — low
  hue_high2  (int, default 180)  upper red hue band — high
  sat_min    (int, default 80)   minimum saturation (0–255)
  val_min    (int, default 60)   minimum value / brightness (0–255)
  min_area_px (int, default 500) minimum contour area in pixels to count

Usage:
  ros2 run tidybot_bringup detect_object_real_2.py
  ros2 run tidybot_bringup detect_object_real_2.py --ros-args -p hue_low1:=0 -p hue_high1:=12
"""

import os
import time
from typing import Optional, Tuple

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


# ---------------------------------------------------------------------------
# Geometry helpers (copied verbatim from detect_object_real.py)
# ---------------------------------------------------------------------------

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


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class ObjectDetectorNode(Node):
    """Detect a red pillow via HSV color segmentation (no YOLO)."""

    LABEL = "pillow"

    def __init__(self):
        super().__init__("object_detector")

        # -- Camera / TF parameters (same as detect_object_real.py) ----------
        self.declare_parameter("rgb_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/aligned_depth_to_color/image_raw")
        self.declare_parameter("depth_fallback_topic", "/camera/depth/image_raw")
        self.declare_parameter("depth_fallback_timeout", 3.0)
        self.declare_parameter("camera_info_topic", "/camera/color/camera_info")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("world_frame", "base_link")
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("save_debug_image", True)
        self.declare_parameter(
            "debug_image_path",
            os.path.join(os.getcwd(), "captures", "latest_detection.jpg"),
        )

        # -- HSV color segmentation parameters --------------------------------
        self.declare_parameter("hue_low1", 0)      # lower red band: 0–10
        self.declare_parameter("hue_high1", 10)
        self.declare_parameter("hue_low2", 170)    # upper red band: 170–180
        self.declare_parameter("hue_high2", 180)
        self.declare_parameter("sat_min", 80)      # minimum saturation (0–255)
        self.declare_parameter("val_min", 60)      # minimum value / brightness (0–255)
        self.declare_parameter("min_area_px", 500) # minimum contour area in pixels

        # -- Hardcoded pose fallback ------------------------------------------
        # When use_hardcoded_pose=True the node skips depth estimation entirely
        # and publishes a fixed pose in world_frame.  Useful for close-up pickup
        # where the depth camera is unreliable.
        # Defaults: pillow at (x=0.0, y=0.2, z=0.15) in base_link frame.
        self.declare_parameter("use_hardcoded_pose", True)
        self.declare_parameter("hardcoded_x", 0.0)   # m — centre left/right
        self.declare_parameter("hardcoded_y", 0.2)   # m — forward from base
        self.declare_parameter("hardcoded_z", 0.15)  # m — height above ground

        # Retrieve camera / TF params
        self.rgb_topic = str(self.get_parameter("rgb_topic").value)
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.depth_fallback_topic = str(self.get_parameter("depth_fallback_topic").value)
        self.depth_fallback_timeout = float(self.get_parameter("depth_fallback_timeout").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.world_frame = str(self.get_parameter("world_frame").value)
        self.publish_debug_image = bool(self.get_parameter("publish_debug_image").value)
        self.save_debug_image = bool(self.get_parameter("save_debug_image").value)
        self.debug_image_path = str(self.get_parameter("debug_image_path").value)
        if self.save_debug_image:
            os.makedirs(os.path.dirname(os.path.abspath(self.debug_image_path)), exist_ok=True)

        # Retrieve HSV params
        self.hue_low1 = int(self.get_parameter("hue_low1").value)
        self.hue_high1 = int(self.get_parameter("hue_high1").value)
        self.hue_low2 = int(self.get_parameter("hue_low2").value)
        self.hue_high2 = int(self.get_parameter("hue_high2").value)
        self.sat_min = int(self.get_parameter("sat_min").value)
        self.val_min = int(self.get_parameter("val_min").value)
        self.min_area_px = int(self.get_parameter("min_area_px").value)

        self.use_hardcoded_pose = bool(self.get_parameter("use_hardcoded_pose").value)
        self.hardcoded_x = float(self.get_parameter("hardcoded_x").value)
        self.hardcoded_y = float(self.get_parameter("hardcoded_y").value)
        self.hardcoded_z = float(self.get_parameter("hardcoded_z").value)

        # -- State ------------------------------------------------------------
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

        # -- Subscriptions ----------------------------------------------------
        qos_best_effort = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Image, self.rgb_topic, self.rgb_cb, qos_best_effort)

        self._depth_received = False
        self._depth_fallback_active = False
        self._depth_sub = self.create_subscription(
            Image, self.depth_topic, self.depth_cb, qos_best_effort)
        self._depth_fallback_sub = self.create_subscription(
            Image, self.depth_fallback_topic, self._depth_fallback_cb,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE))
        self._depth_start_time = time.time()

        self._camera_info_sub = self.create_subscription(
            CameraInfo, self.camera_info_topic, self.camera_info_cb, 10)

        # Kept for interface compatibility (ignored — we always detect "pillow")
        self.create_subscription(String, "/perception/target_label", self.target_label_cb, 10)

        # -- Publishers -------------------------------------------------------
        self.found_pub = self.create_publisher(Bool, "/perception/object_found", 10)
        self.label_pub = self.create_publisher(String, "/perception/object_label", 10)
        self.conf_pub = self.create_publisher(Float32, "/perception/object_confidence", 10)
        self.bbox_pub = self.create_publisher(Int32MultiArray, "/perception/object_bbox", 10)
        self.debug_pub = self.create_publisher(Image, "/perception/object_debug_image", qos_best_effort)
        self.pose_pub = self.create_publisher(PoseStamped, "/perception/object_pose", 10)

        self.get_logger().info("Object detector (HSV red-pillow) started")
        self.get_logger().info(f"  RGB: {self.rgb_topic}")
        self.get_logger().info(f"  Depth: {self.depth_topic}")
        self.get_logger().info(f"  CameraInfo: {self.camera_info_topic}")
        self.get_logger().info(f"  Pose frame: {self.world_frame}")
        self.get_logger().info(
            f"  HSV red masks: [{self.hue_low1}–{self.hue_high1}] OR "
            f"[{self.hue_low2}–{self.hue_high2}], "
            f"sat>={self.sat_min}, val>={self.val_min}"
        )
        self.get_logger().info(f"  Min contour area: {self.min_area_px} px")
        if self.use_hardcoded_pose:
            self.get_logger().warn(
                f"  HARDCODED POSE MODE — depth estimation disabled. "
                f"Publishing fixed pose: ({self.hardcoded_x}, {self.hardcoded_y}, {self.hardcoded_z}) "
                f"in {self.world_frame}"
            )
        else:
            self.get_logger().info("  Pose mode: depth estimation (camera back-projection)")

    # ------------------------------------------------------------------
    # Subscription callbacks (verbatim from detect_object_real.py)
    # ------------------------------------------------------------------

    def target_label_cb(self, msg: String):
        """Kept for topic compatibility; this node always detects 'pillow'."""
        label = str(msg.data).strip().lower()
        if label and label != self.LABEL:
            self.get_logger().warn(
                f"target_label '{label}' received but this node is HSV-locked to 'pillow'. Ignoring."
            )

    def depth_cb(self, msg: Image):
        """Callback for primary (aligned) depth topic."""
        if not self._depth_received:
            self._depth_received = True
            self._depth_fallback_active = False
            self.get_logger().info(f"Receiving depth from primary topic: {self.depth_topic}")
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.depth_encoding = str(msg.encoding).lower()
        except Exception as exc:
            self.latest_depth = None
            self._warn_pose_throttled(f"Depth conversion failed: {exc}")

    def _depth_fallback_cb(self, msg: Image):
        """Fallback depth — only used if aligned depth never arrives."""
        if self._depth_received:
            return
        elapsed = time.time() - self._depth_start_time
        if elapsed < self.depth_fallback_timeout:
            return
        if not self._depth_fallback_active:
            self._depth_fallback_active = True
            self.get_logger().warn(
                f"No data from {self.depth_topic} after {self.depth_fallback_timeout:.0f}s, "
                f"falling back to {self.depth_fallback_topic}"
            )
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.depth_encoding = str(msg.encoding).lower()
        except Exception as exc:
            self.latest_depth = None
            self._warn_pose_throttled(f"Depth fallback conversion failed: {exc}")

    def camera_info_cb(self, msg: CameraInfo):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        self.get_logger().info(
            f"Camera intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} "
            f"cx={self.cx:.1f} cy={self.cy:.1f}"
        )
        self.destroy_subscription(self._camera_info_sub)

    # ------------------------------------------------------------------
    # Depth / TF helpers (verbatim from detect_object_real.py)
    # ------------------------------------------------------------------

    def _warn_pose_throttled(self, text: str):
        now = time.time()
        if now - self.last_pose_warn_time > 1.0:
            self.last_pose_warn_time = now
            self.get_logger().warn(text)

    def _depth_from_mask(self, mask: np.ndarray) -> Optional[float]:
        """Return 20th-percentile depth of pixels that are inside the HSV mask.

        Using the 20th percentile (instead of median) selects the front surface
        of the object and ignores background pixels that leaked into the mask.
        Only pixels with mask > 0 AND valid depth are considered.
        Falls back to a centre-crop median if the depth image is a different
        resolution than the colour image (unaligned fallback topic).
        """
        if self.latest_depth is None:
            return None

        depth = self.latest_depth.astype(np.float32)

        # If depth and mask have different shapes (fallback unaligned topic),
        # resize mask to match depth so indexing still works.
        if depth.shape[:2] != mask.shape[:2]:
            mask = cv2.resize(
                mask, (depth.shape[1], depth.shape[0]), interpolation=cv2.INTER_NEAREST
            )

        valid = depth[(mask > 0) & (depth > 0) & np.isfinite(depth)]
        if valid.size == 0:
            return None

        z = float(np.percentile(valid, 20))
        if self.depth_encoding in ("16uc1", "mono16"):
            z = z / 1000.0
        return z

    def _publish_pose(
        self,
        cx_px: int,
        cy_px: int,
        mask: np.ndarray,
        image_header,
    ) -> Optional[Tuple[float, float, float]]:
        """Publish 3D pose and return (x, y, z) in world frame, or None.

        cx_px, cy_px — contour centroid in image pixels (from cv2.moments).
        mask         — binary HSV mask used to sample depth only over the object.
        """
        if self.use_hardcoded_pose:
            x_w, y_w, z_w = self.hardcoded_x, self.hardcoded_y, self.hardcoded_z
            self.get_logger().info(
                f"[HARDCODED POSE] publishing ({x_w:.3f}, {y_w:.3f}, {z_w:.3f}) "
                f"in {self.world_frame} — depth estimation bypassed",
                throttle_duration_sec=2.0,
            )
        else:
            if any(k is None for k in [self.fx, self.fy, self.cx, self.cy]):
                self._warn_pose_throttled("No camera intrinsics yet; skipping /perception/object_pose.")
                return None

            u, v = cx_px, cy_px
            z_m = self._depth_from_mask(mask)
            if z_m is None:
                self._warn_pose_throttled("Invalid depth from mask; skipping /perception/object_pose.")
                return None

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
                return None

            x_w, y_w, z_w = transform_point(tf_msg, (x_cam, y_cam, z_cam))

        pose_msg = PoseStamped()
        pose_msg.header.stamp = image_header.stamp
        pose_msg.header.frame_id = self.world_frame
        pose_msg.pose.position.x = float(x_w)
        pose_msg.pose.position.y = float(y_w)
        pose_msg.pose.position.z = float(z_w)
        pose_msg.pose.orientation.w = 1.0
        self.pose_pub.publish(pose_msg)
        return (x_w, y_w, z_w)

    # ------------------------------------------------------------------
    # HSV detection
    # ------------------------------------------------------------------

    def detect_target(self, bgr: np.ndarray) -> Optional[Tuple[int, int, int, int, float, str]]:
        """Return largest red contour as (x, y, w, h, conf, label), or None.

        conf = fraction of bounding-box pixels that are red (0.0–1.0).
        """
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        # Lower red band (hue wraps near 0)
        mask1 = cv2.inRange(
            hsv,
            np.array([self.hue_low1, self.sat_min, self.val_min]),
            np.array([self.hue_high1, 255, 255]),
        )
        # Upper red band (hue wraps near 180)
        mask2 = cv2.inRange(
            hsv,
            np.array([self.hue_low2, self.sat_min, self.val_min]),
            np.array([self.hue_high2, 255, 255]),
        )
        mask = cv2.bitwise_or(mask1, mask2)

        # Morphological cleanup — remove small noise blobs
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None

        # Largest contour by area
        best_cnt = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(best_cnt)
        if area < self.min_area_px:
            return None

        rx, ry, rw, rh = cv2.boundingRect(best_cnt)

        # Confidence = red-pixel fill ratio inside the bounding box
        bbox_mask = mask[ry:ry + rh, rx:rx + rw]
        red_pixels = int(np.count_nonzero(bbox_mask))
        total_pixels = rw * rh
        conf = float(red_pixels) / float(max(total_pixels, 1))

        # Contour centroid via moments — more accurate than bbox midpoint
        M = cv2.moments(best_cnt)
        if M['m00'] > 0:
            cx_m = int(round(M['m10'] / M['m00']))
            cy_m = int(round(M['m01'] / M['m00']))
        else:
            cx_m = rx + rw // 2
            cy_m = ry + rh // 2

        return (rx, ry, rw, rh, conf, self.LABEL, cx_m, cy_m, mask)

    # ------------------------------------------------------------------
    # Main RGB callback
    # ------------------------------------------------------------------

    def rgb_cb(self, msg: Image):
        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"RGB conversion failed: {exc}")
            self._publish_not_found()
            return

        detection = self.detect_target(bgr)
        if detection is None:
            self._publish_not_found()
            if self.publish_debug_image:
                self.publish_debug(bgr, None, msg.header)
            return

        x, y, w, h, conf, label, cx_m, cy_m, mask = detection
        self.publish_found(True)
        self.publish_label(label)
        self.publish_confidence(conf)
        self.publish_bbox((x, y, w, h))
        pose_xyz = self._publish_pose(cx_m, cy_m, mask, msg.header)
        if self.publish_debug_image:
            self.publish_debug(bgr, detection, msg.header, pose_xyz)

        now = time.time()
        if now - self.last_log_time > 1.0:
            self.last_log_time = now
            pose_str = ""
            if pose_xyz is not None:
                pose_str = (
                    f" pose=({pose_xyz[0]:.3f}, {pose_xyz[1]:.3f}, {pose_xyz[2]:.3f}) in {self.world_frame}"
                )
            fallback_str = " [RAW DEPTH - positions may be inaccurate!]" if self._depth_fallback_active else ""
            self.get_logger().info(
                f"Detected '{label}' conf={conf:.2f} bbox=({x}, {y}, {w}, {h}){pose_str}{fallback_str}"
            )

    # ------------------------------------------------------------------
    # Publishing helpers
    # ------------------------------------------------------------------

    def _publish_not_found(self):
        self.publish_found(False)
        self.publish_label("")
        self.publish_confidence(0.0)
        self.publish_bbox((0, 0, 0, 0))

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
        bgr: np.ndarray,
        detection: Optional[Tuple[int, int, int, int, float, str]],
        image_header,
        pose_xyz: Optional[Tuple[float, float, float]] = None,
    ):
        vis = bgr.copy()
        if detection is not None:
            x, y, w, h, conf, label, cx_m, cy_m, _mask = detection
            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 220, 0), 2)
            text = f"{label} {conf:.2f}"
            cv2.putText(vis, text, (x, max(25, y - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 220, 0), 2)
            # Draw contour centroid (used for 3D back-projection)
            cv2.circle(vis, (cx_m, cy_m), 6, (0, 0, 255), -1)
            if pose_xyz is not None:
                pose_text = f"({pose_xyz[0]:.2f}, {pose_xyz[1]:.2f}, {pose_xyz[2]:.2f})"
                cv2.putText(vis, pose_text, (x, y + h + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 180, 255), 2)

        debug_msg = self.bridge.cv2_to_imgmsg(vis, encoding="bgr8")
        debug_msg.header = image_header
        self.debug_pub.publish(debug_msg)

        if self.save_debug_image:
            cv2.imwrite(self.debug_image_path, vis)


# ---------------------------------------------------------------------------

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
