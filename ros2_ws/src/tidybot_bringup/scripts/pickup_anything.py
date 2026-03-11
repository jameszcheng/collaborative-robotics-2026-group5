#!/usr/bin/env python3
"""
Pickup Anything - Language-driven 6-DoF grasp detection using GraspAnything-6D.

Instead of a hardcoded fingers-down orientation, this script uses GraspAnything-6D
(https://airvlab.github.io/grasp-anything/docs/grasp-anything-6d/) to propose a
6-DoF grasp pose from an RGB-D image and a language prompt, then executes it with
the TidyBot IK motion planner.

Pipeline:
    Camera (RGB + aligned depth) + language label
            ↓
      GraspAnythingClient
      (point cloud + prompt → 6-DoF grasp proposals)
            ↓
      Best grasp pose (position + orientation) transformed to base_link
            ↓
      TidyBot IK (plan and execute)
            ↓
      Arm executes approach → descend → grasp → lift

Setup (one time):
    # Clone the ECCV 2024 implementation
    git clone https://github.com/Fsoft-AIC/Language-Driven-6-DoF-Grasp-Detection-Using-Negative-Prompt-Guidance
    cd Language-Driven-6-DoF-Grasp-Detection-Using-Negative-Prompt-Guidance
    pip install -r requirements.txt
    # Download checkpoint and set GRASP_ANYTHING_CKPT env var:
    export GRASP_ANYTHING_CKPT=/path/to/checkpoint.pth
    export GRASP_ANYTHING_ROOT=/path/to/Language-Driven-6-DoF-...

Usage:
    # Terminal 1
    ros2 launch tidybot_bringup real.launch.py use_planner:=true

    # Terminal 2 (optional - for object detection / camera search)
    ros2 run tidybot_bringup detect_object_real.py --ros-args -p target_label:=banana

    # Terminal 3
    ros2 run tidybot_bringup pickup_anything.py --ros-args -p target_label:=banana
"""

import os
import sys
import time
import traceback
from dataclasses import dataclass
from enum import Enum, auto
from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseStamped
from interbotix_xs_msgs.msg import JointGroupCommand, JointSingleCommand
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32, Float64MultiArray, String
from tidybot_msgs.srv import PlanToTarget
from sensor_msgs.msg import JointState
import tf2_ros


# ---------------------------------------------------------------------------
# GraspAnything-6D client
# ---------------------------------------------------------------------------

@dataclass
class GraspProposal:
    """A single 6-DoF grasp candidate in camera optical frame."""
    position: np.ndarray        # (3,) xyz in meters, camera optical frame
    rotation_matrix: np.ndarray # (3, 3) rotation matrix, camera optical frame
    score: float                # grasp quality score (higher = better)

    @property
    def quaternion(self) -> Tuple[float, float, float, float]:
        """Return (qw, qx, qy, qz) from the rotation matrix."""
        R = self.rotation_matrix
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            qw = 0.25 / s
            qx = (R[2, 1] - R[1, 2]) * s
            qy = (R[0, 2] - R[2, 0]) * s
            qz = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            qw = (R[2, 1] - R[1, 2]) / s
            qx = 0.25 * s
            qy = (R[0, 1] + R[1, 0]) / s
            qz = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            qw = (R[0, 2] - R[2, 0]) / s
            qx = (R[0, 1] + R[1, 0]) / s
            qy = 0.25 * s
            qz = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            qw = (R[1, 0] - R[0, 1]) / s
            qx = (R[0, 2] + R[2, 0]) / s
            qy = (R[1, 2] + R[2, 1]) / s
            qz = 0.25 * s
        return float(qw), float(qx), float(qy), float(qz)


class GraspAnythingClient:
    """
    Wraps the GraspAnything-6D model for language-driven 6-DoF grasp proposal.

    Inference interface:
        proposals = client.propose_grasps(
            rgb,          # (H, W, 3) uint8 BGR (OpenCV convention)
            depth_m,      # (H, W) float32, depth in metres
            fx, fy, cx, cy,  # camera intrinsics
            language_prompt,   # e.g. "banana"
            negative_prompts,  # e.g. ["table", "background"]  (optional)
            max_grasps,        # how many candidates to return
        )

    Returns a list of GraspProposal in camera_color_optical_frame, sorted by
    score descending.

    GraspAnything-6D paper (ECCV 2024):
        "Language-Driven 6-DoF Grasp Detection Using Negative Prompt Guidance"
        https://github.com/Fsoft-AIC/Language-Driven-6-DoF-Grasp-Detection-Using-Negative-Prompt-Guidance
    """

    def __init__(self, checkpoint_path: str, model_root: str):
        """
        Load the GraspAnything-6D model.

        Args:
            checkpoint_path: Path to the pretrained .pth checkpoint.
            model_root: Path to the cloned repository root (added to sys.path).
        """
        self._model = None
        self._load_model(checkpoint_path, model_root)

    def _load_model(self, checkpoint_path: str, model_root: str):
        """Load model weights. Sets self._model = None on failure."""
        if model_root and model_root not in sys.path:
            sys.path.insert(0, model_root)
        try:
            # ----------------------------------------------------------------
            # GraspAnything-6D model import.
            # Adjust the import path to match the actual module structure of the
            # cloned repo (Fsoft-AIC/Language-Driven-6-DoF-...).
            # ----------------------------------------------------------------
            from models.lgrasp6d import LGrasp6D  # type: ignore
            import torch

            self._model = LGrasp6D.from_pretrained(checkpoint_path)
            self._model.eval()
            self._device = 'cuda' if torch.cuda.is_available() else 'cpu'
            self._model.to(self._device)
            print(f"[GraspAnythingClient] Model loaded on {self._device}.")
        except ImportError as exc:
            print(
                f"[GraspAnythingClient] Could not import GraspAnything-6D model: {exc}\n"
                "  Make sure GRASP_ANYTHING_ROOT points to the cloned repo and "
                "dependencies are installed."
            )
        except Exception as exc:
            print(f"[GraspAnythingClient] Model load failed: {exc}")

    @property
    def available(self) -> bool:
        return self._model is not None

    def _depth_to_point_cloud(
        self,
        depth_m: np.ndarray,
        fx: float, fy: float, cx: float, cy: float,
        max_depth: float = 2.0,
    ) -> np.ndarray:
        """Convert a depth image to an (N, 3) point cloud in camera frame."""
        h, w = depth_m.shape
        u, v = np.meshgrid(np.arange(w), np.arange(h))
        z = depth_m.astype(np.float32)
        mask = (z > 0) & (z < max_depth) & np.isfinite(z)
        z = z[mask]
        x = (u[mask] - cx) * z / fx
        y = (v[mask] - cy) * z / fy
        return np.stack([x, y, z], axis=-1)  # (N, 3)

    def propose_grasps(
        self,
        rgb: np.ndarray,
        depth_m: np.ndarray,
        fx: float, fy: float, cx: float, cy: float,
        language_prompt: str,
        negative_prompts: Optional[List[str]] = None,
        max_grasps: int = 5,
    ) -> List[GraspProposal]:
        """
        Run GraspAnything-6D inference and return ranked grasp proposals.

        Args:
            rgb: (H, W, 3) uint8, BGR (OpenCV).
            depth_m: (H, W) float32, depth in metres.
            fx, fy, cx, cy: Camera intrinsics.
            language_prompt: Object to grasp, e.g. "banana".
            negative_prompts: Objects to avoid grasping, e.g. ["table"].
            max_grasps: Maximum number of proposals to return.

        Returns:
            List of GraspProposal sorted by score descending (best first).
            Empty list if model unavailable or inference fails.
        """
        if not self.available:
            return []

        try:
            import torch

            point_cloud = self._depth_to_point_cloud(depth_m, fx, fy, cx, cy)
            rgb_tensor = torch.from_numpy(
                cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB).astype(np.float32) / 255.0
            ).permute(2, 0, 1).unsqueeze(0).to(self._device)
            pc_tensor = torch.from_numpy(point_cloud).unsqueeze(0).to(self._device)

            neg = negative_prompts or []

            with torch.no_grad():
                # ----------------------------------------------------------------
                # GraspAnything-6D forward pass.
                # The exact call signature depends on the model implementation.
                # Based on the ECCV 2024 paper / Fsoft-AIC repo convention:
                #   output is a dict with 'translations', 'rotations', 'scores'
                #   each of shape (1, K, ...).
                # Adjust if the repo uses a different interface.
                # ----------------------------------------------------------------
                output = self._model(
                    rgb=rgb_tensor,
                    point_cloud=pc_tensor,
                    positive_prompt=language_prompt,
                    negative_prompts=neg,
                )

            translations = output['translations'][0].cpu().numpy()  # (K, 3)
            rotations    = output['rotations'][0].cpu().numpy()     # (K, 3, 3)
            scores       = output['scores'][0].cpu().numpy()        # (K,)

            proposals = [
                GraspProposal(
                    position=translations[i],
                    rotation_matrix=rotations[i],
                    score=float(scores[i]),
                )
                for i in range(len(scores))
            ]
            proposals.sort(key=lambda g: g.score, reverse=True)
            return proposals[:max_grasps]

        except Exception as exc:
            print(f"[GraspAnythingClient] Inference failed: {exc}")
            traceback.print_exc()
            return []


# ---------------------------------------------------------------------------
# TF helpers (same as detect_object_real.py)
# ---------------------------------------------------------------------------

def quat_to_rot_matrix(qx, qy, qz, qw):
    xx, yy, zz = qx*qx, qy*qy, qz*qz
    return (
        (1-2*(yy+zz),   2*(qx*qy-qw*qz), 2*(qx*qz+qw*qy)),
        (2*(qx*qy+qw*qz), 1-2*(xx+zz),   2*(qy*qz-qw*qx)),
        (2*(qx*qz-qw*qy), 2*(qy*qz+qw*qx), 1-2*(xx+yy)),
    )


def transform_point(tf_msg, point_xyz):
    x, y, z = point_xyz
    t = tf_msg.transform.translation
    q = tf_msg.transform.rotation
    r = quat_to_rot_matrix(q.x, q.y, q.z, q.w)
    return (
        r[0][0]*x + r[0][1]*y + r[0][2]*z + t.x,
        r[1][0]*x + r[1][1]*y + r[1][2]*z + t.y,
        r[2][0]*x + r[2][1]*y + r[2][2]*z + t.z,
    )


def transform_rotation(tf_msg, R_cam: np.ndarray) -> np.ndarray:
    """Rotate a 3x3 rotation matrix from camera frame to world frame."""
    q = tf_msg.transform.rotation
    R_world_cam = np.array(quat_to_rot_matrix(q.x, q.y, q.z, q.w))
    return R_world_cam @ R_cam


def rot_matrix_to_quat(R: np.ndarray) -> Tuple[float, float, float, float]:
    """Return (qw, qx, qy, qz) from a 3x3 rotation matrix."""
    g = GraspProposal(position=np.zeros(3), rotation_matrix=R, score=0.0)
    return g.quaternion


# ---------------------------------------------------------------------------
# State machine
# ---------------------------------------------------------------------------

class PickupState(Enum):
    APPROACH = auto()
    DESCEND  = auto()
    GRASP    = auto()
    LIFT     = auto()
    SLEEP    = auto()
    DONE     = auto()


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class PickupAnythingNode(Node):
    """
    Pick up an object specified by language using GraspAnything-6D grasp proposals.

    Differences from pickup.py:
    - No hardcoded ORIENT_FINGERS_DOWN — grasp orientation comes from GraspAnythingClient.
    - Subscribes to RGB + aligned depth + camera_info to feed the grasp model.
    - Falls back to fingers-down orientation if GraspAnything is unavailable or returns
      no proposals (so the script still works without the model installed).
    """

    # Fingers-down fallback quaternion (same as pickup.py)
    ORIENT_FINGERS_DOWN = (0.5, 0.5, 0.5, -0.5)  # (qw, qx, qy, qz)
    SLEEP_POSE = [0.0, -1.80, 1.55, 0.0, 0.8, 0.0]

    APPROACH_HEIGHT = 0.10
    GRASP_HEIGHT    = 0.01
    LIFT_HEIGHT     = 0.25
    Y_OFFSET        = -0.05

    ARM_NAME        = 'right'

    def __init__(self):
        super().__init__('pickup_anything')

        # Parameters
        self.declare_parameter('target_label', 'apple')
        self.declare_parameter('arm_name', self.ARM_NAME)
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/aligned_depth_to_color/image_raw')
        self.declare_parameter('depth_fallback_topic', '/camera/depth/image_raw')
        self.declare_parameter('depth_fallback_timeout', 3.0)
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('world_frame', 'base_link')
        self.declare_parameter('grasp_anything_ckpt',
                               os.environ.get('GRASP_ANYTHING_CKPT', ''))
        self.declare_parameter('grasp_anything_root',
                               os.environ.get('GRASP_ANYTHING_ROOT', ''))
        self.declare_parameter('max_grasp_proposals', 5)
        self.declare_parameter('require_perception', True)

        self.target_label    = self.get_parameter('target_label').value.strip().lower()
        self.arm_name        = self.get_parameter('arm_name').value
        self.camera_frame    = self.get_parameter('camera_frame').value
        self.world_frame     = self.get_parameter('world_frame').value
        self.require_perception = bool(self.get_parameter('require_perception').value)

        # GraspAnything client
        ckpt = self.get_parameter('grasp_anything_ckpt').value
        root = self.get_parameter('grasp_anything_root').value
        self._grasp_client = GraspAnythingClient(ckpt, root)
        if self._grasp_client.available:
            self.get_logger().info('GraspAnything-6D model loaded.')
        else:
            self.get_logger().warn(
                'GraspAnything-6D model NOT loaded — will fall back to fingers-down orientation. '
                'Set GRASP_ANYTHING_CKPT and GRASP_ANYTHING_ROOT env vars to enable it.'
            )

        self.bridge = CvBridge()

        # Camera data
        self.latest_rgb   = None
        self.latest_depth = None
        self.depth_encoding = ''
        self.fx = self.fy = self.cx = self.cy = None

        self._depth_received      = False
        self._depth_fallback_active = False
        self._depth_start_time    = time.time()
        self._depth_fallback_timeout = float(
            self.get_parameter('depth_fallback_timeout').value)

        # TF
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Perception (from detect_object_real.py)
        self.object_found      = False
        self.object_pose       = None
        self.object_confidence = 0.0
        self.object_label_det  = ''

        # Joint states
        self.joint_states_received = False
        self._latest_positions     = {}

        # IK client
        self.plan_client = self.create_client(PlanToTarget, '/plan_to_target')

        # Publishers
        self.arm_group_pub = self.create_publisher(
            JointGroupCommand, f'/{self.arm_name}_arm/commands/joint_group', 10)
        self.gripper_pub = self.create_publisher(
            JointSingleCommand, f'/{self.arm_name}_arm/commands/joint_single', 10)
        self.pan_tilt_pub = self.create_publisher(
            Float64MultiArray, '/camera/pan_tilt_cmd', 10)

        # Subscriptions
        qos_be = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_re = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)

        rgb_topic   = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        depth_fb    = self.get_parameter('depth_fallback_topic').value
        ci_topic    = self.get_parameter('camera_info_topic').value

        self.create_subscription(Image, rgb_topic,   self._rgb_cb,   qos_be)
        self.create_subscription(Image, depth_topic, self._depth_cb, qos_be)
        self.create_subscription(Image, depth_fb,    self._depth_fallback_cb, qos_re)
        self._camera_info_sub = self.create_subscription(
            CameraInfo, ci_topic, self._camera_info_cb, 10)

        self.create_subscription(JointState,   '/joint_states',              self._joint_state_cb,       10)
        self.create_subscription(Bool,         '/perception/object_found',   self._object_found_cb,      10)
        self.create_subscription(PoseStamped,  '/perception/object_pose',    self._object_pose_cb,       10)
        self.create_subscription(Float32,      '/perception/object_confidence', self._object_confidence_cb, 10)
        self.create_subscription(String,       '/perception/object_label',   self._object_label_cb,      10)

        self.state = PickupState.APPROACH

        self.get_logger().info('=' * 60)
        self.get_logger().info('PickupAnything — Language-driven 6-DoF grasp')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'  Target: {self.target_label}')
        self.get_logger().info(f'  Arm:    {self.arm_name}')

        self.get_logger().info('Waiting for /plan_to_target service...')
        if not self.plan_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('Planning service not available. '
                               'Launch with: ros2 launch tidybot_bringup real.launch.py use_planner:=true')
        self.get_logger().info('Service connected.')

    # ------------------------------------------------------------------
    # Camera callbacks
    # ------------------------------------------------------------------

    def _rgb_cb(self, msg: Image):
        try:
            self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            self.latest_rgb = None

    def _depth_cb(self, msg: Image):
        if not self._depth_received:
            self._depth_received = True
            self.get_logger().info(f'Receiving aligned depth from primary topic.')
        try:
            self.latest_depth   = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_encoding = str(msg.encoding).lower()
        except Exception:
            self.latest_depth = None

    def _depth_fallback_cb(self, msg: Image):
        if self._depth_received:
            return
        elapsed = time.time() - self._depth_start_time
        if elapsed < self._depth_fallback_timeout:
            return
        if not self._depth_fallback_active:
            self._depth_fallback_active = True
            self.get_logger().warn(
                f'No aligned depth after {self._depth_fallback_timeout:.0f}s — '
                'falling back to raw depth. Grasp poses may be less accurate.')
        try:
            self.latest_depth   = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.depth_encoding = str(msg.encoding).lower()
        except Exception:
            self.latest_depth = None

    def _camera_info_cb(self, msg: CameraInfo):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])
        self.get_logger().info(
            f'Camera intrinsics: fx={self.fx:.1f} fy={self.fy:.1f} '
            f'cx={self.cx:.1f} cy={self.cy:.1f}')
        self.destroy_subscription(self._camera_info_sub)

    # ------------------------------------------------------------------
    # Other callbacks
    # ------------------------------------------------------------------

    def _joint_state_cb(self, msg: JointState):
        self.joint_states_received = True
        for name, pos in zip(msg.name, msg.position):
            self._latest_positions[name] = pos

    def _object_found_cb(self, msg: Bool):
        self.object_found = msg.data

    def _object_pose_cb(self, msg: PoseStamped):
        self.object_pose = msg

    def _object_confidence_cb(self, msg: Float32):
        self.object_confidence = msg.data

    def _object_label_cb(self, msg: String):
        self.object_label_det = msg.data

    # ------------------------------------------------------------------
    # Grasp proposal
    # ------------------------------------------------------------------

    def _depth_to_metres(self) -> Optional[np.ndarray]:
        """Return depth image in metres, or None if unavailable."""
        if self.latest_depth is None:
            return None
        d = self.latest_depth.astype(np.float32)
        if self.depth_encoding in ('16uc1', 'mono16'):
            d /= 1000.0
        return d

    def get_grasp_pose_in_world(
        self, language_prompt: str
    ) -> Optional[Tuple[float, float, float, float, float, float, float]]:
        """
        Run GraspAnything-6D to get the best grasp pose in world frame.

        Returns (x, y, z, qw, qx, qy, qz) in base_link frame, or None on failure.
        Falls back to the fingers-down orientation using the YOLO-detected object
        position if the model is unavailable or returns no proposals.
        """
        # Collect a fresh snapshot (spin briefly for latest messages)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

        intrinsics_ready = all(v is not None for v in [self.fx, self.fy, self.cx, self.cy])

        # ---- Try GraspAnything-6D ----------------------------------------
        if (self._grasp_client.available
                and self.latest_rgb is not None
                and self.latest_depth is not None
                and intrinsics_ready):

            depth_m = self._depth_to_metres()
            neg_prompts = ['table', 'floor', 'background', 'shelf']

            self.get_logger().info(
                f'Running GraspAnything-6D for "{language_prompt}"...')
            proposals = self._grasp_client.propose_grasps(
                rgb=self.latest_rgb,
                depth_m=depth_m,
                fx=self.fx, fy=self.fy, cx=self.cx, cy=self.cy,
                language_prompt=language_prompt,
                negative_prompts=neg_prompts,
                max_grasps=int(self.get_parameter('max_grasp_proposals').value),
            )

            if proposals:
                best = proposals[0]
                self.get_logger().info(
                    f'GraspAnything: {len(proposals)} proposals, best score={best.score:.3f}')

                # Transform position and orientation to world frame via TF
                try:
                    tf_msg = self.tf_buffer.lookup_transform(
                        self.world_frame, self.camera_frame,
                        rclpy.time.Time(), timeout=Duration(seconds=1.0))
                except Exception as exc:
                    self.get_logger().warn(
                        f'TF lookup failed ({self.world_frame} <- {self.camera_frame}): {exc}')
                    # Fall through to fallback below
                else:
                    x_w, y_w, z_w = transform_point(tf_msg, tuple(best.position))
                    R_world = transform_rotation(tf_msg, best.rotation_matrix)
                    qw, qx, qy, qz = rot_matrix_to_quat(R_world)
                    self.get_logger().info(
                        f'Best grasp in {self.world_frame}: '
                        f'pos=({x_w:.3f}, {y_w:.3f}, {z_w:.3f}) '
                        f'quat=({qw:.3f}, {qx:.3f}, {qy:.3f}, {qz:.3f})')
                    return x_w, y_w, z_w, qw, qx, qy, qz
            else:
                self.get_logger().warn(
                    'GraspAnything returned no proposals — falling back to fingers-down.')

        # ---- Fallback: use YOLO pose + fingers-down orientation ----------
        if self.object_pose is None:
            self.get_logger().error(
                'No object pose available from perception. '
                'Run: ros2 run tidybot_bringup detect_object_real.py')
            return None

        p = self.object_pose.pose.position
        qw, qx, qy, qz = self.ORIENT_FINGERS_DOWN
        self.get_logger().info(
            f'Fallback: using YOLO pose ({p.x:.3f}, {p.y:.3f}, {p.z:.3f}) '
            'with fingers-down orientation.')
        return p.x, p.y, p.z, qw, qx, qy, qz

    # ------------------------------------------------------------------
    # Motion helpers (same as pickup.py)
    # ------------------------------------------------------------------

    def create_pose(self, x, y, z, qw, qx, qy, qz) -> Pose:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.w, pose.orientation.x = qw, qx
        pose.orientation.y, pose.orientation.z = qy, qz
        return pose

    def call_service_sync(self, request: PlanToTarget.Request, timeout_sec: float = 20.0):
        future = self.plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            self.get_logger().error('Service call timed out.')
            return None
        if future.exception() is not None:
            self.get_logger().error(f'Service call exception: {future.exception()}')
            return None
        return future.result()

    def plan_and_execute(self, pose: Pose, duration: float = 3.0) -> bool:
        request = PlanToTarget.Request()
        request.arm_name     = self.arm_name
        request.target_pose  = pose
        request.use_orientation = True
        request.execute      = True
        request.duration     = duration
        # max_condition_number uses service default (500.0)

        pos_str = f'({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})'
        self.get_logger().info(f'Planning+executing {self.arm_name} to {pos_str}')

        result = self.call_service_sync(request)
        if result is None:
            return False
        if result.success:
            self.get_logger().info(f'  SUCCESS: {result.message}')
            self.get_logger().info(
                f'  Errors: pos={result.position_error:.4f}m, '
                f'ori={result.orientation_error:.4f}rad ({np.degrees(result.orientation_error):.1f}°)')
            return True
        self.get_logger().warn(f'  FAILED: {result.message}')
        return False

    def command_gripper_pwm(self, pwm: float, duration: float = 1.0):
        msg = JointSingleCommand()
        msg.name = f'{self.arm_name}_gripper'
        msg.cmd  = float(pwm)
        start = time.time()
        while time.time() - start < duration:
            self.gripper_pub.publish(msg)
            time.sleep(0.1)

    def open_gripper(self):
        self.get_logger().info('Opening gripper...')
        self.command_gripper_pwm(450.0, duration=3.0)

    def close_gripper(self):
        self.get_logger().info('Closing gripper...')
        self.command_gripper_pwm(-350.0, duration=3.0)

    def go_to_sleep(self, max_joint_speed: float = 0.5):
        self.get_logger().info(f'Moving {self.arm_name} arm to sleep pose...')
        rclpy.spin_once(self, timeout_sec=0.1)
        joint_names = [
            f'{self.arm_name}_waist', f'{self.arm_name}_shoulder', f'{self.arm_name}_elbow',
            f'{self.arm_name}_forearm_roll', f'{self.arm_name}_wrist_angle', f'{self.arm_name}_wrist_rotate',
        ]
        current = np.array([self._latest_positions.get(j, 0.0) for j in joint_names])
        target  = np.array(self.SLEEP_POSE)
        duration = max(np.max(np.abs(target - current)) / max_joint_speed, 1.0)
        rate_hz, dt = 50.0, 1.0 / 50.0
        num_steps = max(int(duration * rate_hz), 1)
        for i in range(num_steps + 1):
            alpha = 0.5 * (1 - np.cos(np.pi * i / num_steps))
            q = current + alpha * (target - current)
            cmd = JointGroupCommand()
            cmd.name = f'{self.arm_name}_arm'
            cmd.cmd  = q.tolist()
            self.arm_group_pub.publish(cmd)
            if i < num_steps:
                time.sleep(dt)
        self.get_logger().info(f'{self.arm_name} arm in sleep pose.')

    def send_pan_tilt(self, pan: float, tilt: float, duration: float = 1.0):
        msg = Float64MultiArray()
        msg.data = [pan, tilt]
        for _ in range(int(duration * 20)):
            self.pan_tilt_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def _check_detection(self, min_confidence: float = 0.4) -> bool:
        rclpy.spin_once(self, timeout_sec=0.05)
        return (self.object_found
                and self.object_pose is not None
                and self.object_confidence >= min_confidence)

    def search_for_object(self, timeout_per_position: float = 5.0, min_confidence: float = 0.4) -> bool:
        search_positions = [
            (0.0,  0.0,  'center'),
            (0.0,  0.3,  'tilt down'),
            (-0.5, 0.0,  'pan left'),
            (0.5,  0.0,  'pan right'),
            (0.0,  0.0,  'center'),
            (-0.5, 0.0,  'pan left'),
            (0.5,  0.0,  'pan right'),
        ]
        self.get_logger().info('No object found — starting camera search sweep...')
        for pan, tilt, desc in search_positions:
            self.get_logger().info(f'  Search: {desc} (pan={pan:.2f}, tilt={tilt:.2f})')
            msg = Float64MultiArray()
            msg.data = [pan, tilt]
            for _ in range(20):
                self.pan_tilt_pub.publish(msg)
                if self._check_detection(min_confidence):
                    self.get_logger().info(f'  Object found during move to {desc}.')
                    return True
                time.sleep(0.05)
            start = time.time()
            while (time.time() - start) < timeout_per_position:
                self.pan_tilt_pub.publish(msg)
                if self._check_detection(min_confidence):
                    self.get_logger().info(f'  Object found at {desc}.')
                    return True
                time.sleep(0.05)
        self.send_pan_tilt(0.0, 0.0, duration=1.0)
        self.get_logger().warn('Search sweep complete — no object detected.')
        return False

    def wait_for_object_detection(self, timeout: float = 30.0, min_confidence: float = 0.5) -> bool:
        self.get_logger().info(f'Waiting for object detection (min confidence: {min_confidence:.2f})...')
        start = time.time()
        while (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.object_found and self.object_pose is not None and self.object_confidence >= min_confidence:
                self.get_logger().info(
                    f'Detected: "{self.object_label_det}" conf={self.object_confidence:.2f} '
                    f'pos=({self.object_pose.pose.position.x:.3f}, '
                    f'{self.object_pose.pose.position.y:.3f}, '
                    f'{self.object_pose.pose.position.z:.3f})')
                return True
            time.sleep(0.1)
        return False

    def wait_for_joint_states(self, timeout: float = 10.0) -> bool:
        start = time.time()
        while (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.joint_states_received:
                return True
        return False

    def transition_to(self, state: PickupState):
        self.get_logger().info(f'State: {self.state.name} -> {state.name}')
        self.state = state

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------

    def run_state_machine(self, grasp_x, grasp_y, grasp_z, qw, qx, qy, qz) -> bool:
        """
        Execute approach → descend → grasp → lift → sleep using the proposed grasp pose.
        Position offsets are applied along Z (vertical) in base_link frame.
        Y_OFFSET shifts the grasp point forward from the robot.
        """
        y = grasp_y + self.Y_OFFSET

        approach_pose = self.create_pose(grasp_x, y, grasp_z + self.APPROACH_HEIGHT, qw, qx, qy, qz)
        grasp_pose    = self.create_pose(grasp_x, y, grasp_z + self.GRASP_HEIGHT,    qw, qx, qy, qz)
        lift_pose     = self.create_pose(grasp_x, y, grasp_z + self.LIFT_HEIGHT,     qw, qx, qy, qz)

        self.get_logger().info('')
        self.get_logger().info('Pickup waypoints:')
        self.get_logger().info(f'  approach: ({grasp_x:.3f}, {y:.3f}, {grasp_z + self.APPROACH_HEIGHT:.3f})')
        self.get_logger().info(f'  descend:  ({grasp_x:.3f}, {y:.3f}, {grasp_z + self.GRASP_HEIGHT:.3f})')
        self.get_logger().info(f'  lift:     ({grasp_x:.3f}, {y:.3f}, {grasp_z + self.LIFT_HEIGHT:.3f})')
        self.get_logger().warn('Ensure workspace is clear before continuing.')
        input('Press Enter to start pickup sequence (Ctrl+C to abort)... ')

        while self.state != PickupState.DONE:
            if self.state == PickupState.APPROACH:
                self.open_gripper()
                if not self.plan_and_execute(approach_pose, duration=10.0):
                    return False
                self.transition_to(PickupState.DESCEND)

            elif self.state == PickupState.DESCEND:
                if not self.plan_and_execute(grasp_pose, duration=10.0):
                    return False
                self.transition_to(PickupState.GRASP)

            elif self.state == PickupState.GRASP:
                self.close_gripper()
                self.transition_to(PickupState.LIFT)

            elif self.state == PickupState.LIFT:
                if not self.plan_and_execute(lift_pose, duration=10.0):
                    return False
                self.transition_to(PickupState.SLEEP)

            elif self.state == PickupState.SLEEP:
                self.open_gripper()
                self.go_to_sleep()
                self.transition_to(PickupState.DONE)

            time.sleep(0.3)

        self.get_logger().info('Pickup complete.')
        return True


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = PickupAnythingNode()

    try:
        node.get_logger().info('Waiting for joint states...')
        if not node.wait_for_joint_states(timeout=10.0):
            node.get_logger().error('No joint states received! Check real.launch.py is running.')
            return 1

        if not node.plan_client.wait_for_service(timeout_sec=2.0):
            node.get_logger().error('IK service not available!')
            return 1

        # Wait for object detection (from detect_object_real.py) if required
        if node.require_perception:
            if not node.wait_for_object_detection(timeout=10.0, min_confidence=0.4):
                node.get_logger().warn(
                    'No detection in initial window — starting camera search sweep.')
                if not node.search_for_object(timeout_per_position=5.0, min_confidence=0.4):
                    node.get_logger().error(
                        'No object detected after full sweep. '
                        'Run: ros2 run tidybot_bringup detect_object_real.py '
                        f'--ros-args -p target_label:={node.target_label}')
                    return 1
            node.get_logger().info('Object detection ready.')

        # Get grasp pose from GraspAnything-6D (or fallback)
        result = node.get_grasp_pose_in_world(node.target_label)
        if result is None:
            node.get_logger().error('Could not obtain a grasp pose. Aborting.')
            return 1

        grasp_x, grasp_y, grasp_z, qw, qx, qy, qz = result

        ok = node.run_state_machine(grasp_x, grasp_y, grasp_z, qw, qx, qy, qz)
        return 0 if ok else 1

    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user.')
        return 130
    except Exception as exc:
        node.get_logger().error(f'Unhandled exception: {exc}')
        traceback.print_exc()
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
