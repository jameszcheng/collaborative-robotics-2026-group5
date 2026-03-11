#!/usr/bin/env python3
"""
navigate_to_object.py — Based on movement_4.py, but navigates to a detected
object pose from /perception/object_pose instead of hardcoded coordinates.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
TERMINAL 1 — Launch the robot
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  ros2 launch tidybot_bringup real.launch.py

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
TERMINAL 2 — Run object detection
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  ros2 run tidybot_bringup detect_object_real.py --ros-args -p target_label:=banana

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
TERMINAL 3 — Navigate to detected object
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  # Reset origin (recommended before every test)
  ros2 run tidybot_bringup navigate_to_object.py --ros-args -p mode:=reset_origin -p robot:=real

  # Navigate to object (real robot, default 0.4m standoff)
  ros2 run tidybot_bringup navigate_to_object.py --ros-args -p robot:=real

  # Custom standoff distance
  ros2 run tidybot_bringup navigate_to_object.py --ros-args -p robot:=real -p standoff_dist:=0.3

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
TERMINAL 3 (alt) — Test with hardcoded position (no object detection needed)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  # Go to a single position (metres relative to origin)
  ros2 run tidybot_bringup navigate_to_object.py --ros-args -p mode:=goto -p goal_x:=0.5 -p goal_y:=0.5 -p robot:=real

  # Multiple waypoints
  ros2 run tidybot_bringup navigate_to_object.py --ros-args -p mode:=goto -p waypoints:="[0.5,0.5, 1.0,0.0]" -p robot:=real

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
OPTIONAL PARAMETERS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  -p robot:=real                # real robot (default: sim)
  -p standoff_dist:=0.4        # stop this far from object (default: 0.4 m)
  -p lateral_offset:=0.15      # shift left to line up right arm (default: 0.15 m)
  -p pose_timeout:=30.0        # wait this long for first object pose (default: 30 s)
  -p kp:=1.5                   # proportional gain (default: 1.0)
  -p max_v:=0.2                # max linear speed m/s (default: 0.2)
  -p max_omega:=2.0            # max rotation speed rad/s (default: 2.0)
  -p goal_tolerance:=0.05      # goal tolerance metres (default: 0.05)

Topics:
  Sub: /perception/object_pose  (geometry_msgs/PoseStamped, from detect_object_real)
  Sub: /odom                    (nav_msgs/Odometry)
  Pub: /cmd_vel                 (geometry_msgs/Twist)
  Pub: /base/target_pose        (geometry_msgs/Pose2D, sim only)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D, PoseStamped, PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Empty
import tf2_ros
import tf2_geometry_msgs
import numpy as np
import csv
import os
import time
import math
from datetime import datetime

VERSION = "1.0.0-object"


# Hardcoded command-frame rotation (applies to BOTH sim and real).
# Positive = CCW rotation of commanded (x,y).
# Example: +90° means goal_x=1,goal_y=0 behaves like (0,1) in the original frame.
COMMAND_FRAME_OFFSET_DEG = -90.0


def wrap_angle(a: float) -> float:
    return float(np.arctan2(np.sin(a), np.cos(a)))


def rotate_xy(x: float, y: float, angle_rad: float):
    """Rotate (x,y) by angle_rad (CCW)."""
    c, s = float(np.cos(angle_rad)), float(np.sin(angle_rad))
    return float(c * x - s * y), float(s * x + c * y)


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Standard yaw-from-quaternion (robust even with small roll/pitch)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return float(np.arctan2(siny_cosp, cosy_cosp))


def circular_mean(angles):
    s = np.mean(np.sin(angles))
    c = np.mean(np.cos(angles))
    return float(np.arctan2(s, c))


def circular_range(angles):
    m = circular_mean(angles)
    diffs = [wrap_angle(a - m) for a in angles]
    return float(max(diffs) - min(diffs))


class NavigateToObject(Node):
    def __init__(self):
        super().__init__('navigate_to_object')

        # --- Parameters (same as movement_4.py) ---
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('save_data', True)
        self.declare_parameter('duration', 20.0)
        self.declare_parameter('mode', 'object')     # 'object','goto','circle','reset_origin','test'
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_tolerance', 0.05)
        self.declare_parameter('waypoints', [0.0])
        self.declare_parameter('max_v', 0.2)
        self.declare_parameter('max_omega', 2.0)
        self.declare_parameter('robot', 'sim')
        self.declare_parameter('test', 'forward')
        self.declare_parameter('test_distance', 0.5)
        self.declare_parameter('test_angle', 90.0)

        # Robustness knobs (from movement_4.py)
        self.declare_parameter('yaw_offset', float('nan'))
        self.declare_parameter('origin_namespace', '')
        self.declare_parameter('sim_pose_theta_degrees', False)

        # Final yaw at end of goto (from movement_4.py)
        self.declare_parameter('goal_theta_deg', float('nan'))
        self.declare_parameter('yaw_tolerance_deg', 5.0)
        self.declare_parameter('yaw_kp', 2.0)

        # Origin locking stability knobs
        self.declare_parameter('stable_xy_threshold', 0.005)
        self.declare_parameter('stable_theta_threshold', 0.02)
        self.declare_parameter('stable_required', 25)

        # --- NEW: object navigation parameters ---
        self.declare_parameter('standoff_dist', 0.4)   # metres — stop this far from the object
        self.declare_parameter('lateral_offset', 0.15)  # metres — positive = shift left (to line up right arm)
        self.declare_parameter('pose_timeout', 30.0)    # seconds to wait for first object pose
        self.declare_parameter('face_object', True)     # align to face the object after reaching standoff
        self.declare_parameter('pose_samples', 5)       # average this many pose readings before locking goal

        # Read params
        self.kp            = float(self.get_parameter('kp').value)
        self.save_data     = bool(self.get_parameter('save_data').value)
        self.duration      = float(self.get_parameter('duration').value)
        self.mode          = str(self.get_parameter('mode').value)
        self.goal_x        = float(self.get_parameter('goal_x').value)
        self.goal_y        = float(self.get_parameter('goal_y').value)
        self.goal_tol      = float(self.get_parameter('goal_tolerance').value)
        self.max_v         = float(self.get_parameter('max_v').value)
        self.max_omega     = float(self.get_parameter('max_omega').value)
        self.robot         = str(self.get_parameter('robot').value)
        self.test          = str(self.get_parameter('test').value)
        self.test_distance = float(self.get_parameter('test_distance').value)
        self.test_angle    = float(np.radians(self.get_parameter('test_angle').value))

        self.sim_pose_theta_degrees = bool(self.get_parameter('sim_pose_theta_degrees').value)

        # Final yaw config
        self.goal_theta_deg = float(self.get_parameter('goal_theta_deg').value)
        self.yaw_tolerance = float(np.radians(self.get_parameter('yaw_tolerance_deg').value))
        self.yaw_kp = float(self.get_parameter('yaw_kp').value)
        self._final_yaw_done = False

        # Command-frame rotation
        self.command_frame_offset = float(np.radians(COMMAND_FRAME_OFFSET_DEG))

        # Stability config
        self._stable_threshold_xy = float(self.get_parameter('stable_xy_threshold').value)
        self._stable_threshold_th = float(self.get_parameter('stable_theta_threshold').value)
        self._stable_required     = int(self.get_parameter('stable_required').value)

        # yaw offset
        yaw_offset_param = float(self.get_parameter('yaw_offset').value)
        if np.isnan(yaw_offset_param):
            self.yaw_offset = -np.pi / 2.0 if self.robot == 'sim' else 0.0
        else:
            self.yaw_offset = yaw_offset_param

        # Origin file namespace
        ns = str(self.get_parameter('origin_namespace').value).strip()
        if ns == '':
            ns = self.robot
        self.origin_file = os.path.expanduser(f'~/.tidybot_origin_{ns}.txt')

        # Object navigation params
        self.standoff_dist  = float(self.get_parameter('standoff_dist').value)
        self.lateral_offset = float(self.get_parameter('lateral_offset').value)
        self.pose_timeout   = float(self.get_parameter('pose_timeout').value)
        self.face_object   = bool(self.get_parameter('face_object').value)
        self.pose_samples  = int(self.get_parameter('pose_samples').value)

        # Build waypoint list (for goto mode)
        raw_wp = self.get_parameter('waypoints').value
        if len(raw_wp) >= 2 and not (len(raw_wp) == 1 and raw_wp[0] == 0.0):
            if len(raw_wp) % 2 != 0:
                self.get_logger().error(
                    f'Parameter "waypoints" must have an even number of values. Got {len(raw_wp)}.'
                )
                self.waypoints = [(self.goal_x, self.goal_y)]
            else:
                self.waypoints = [(raw_wp[i], raw_wp[i + 1]) for i in range(0, len(raw_wp), 2)]
        else:
            self.waypoints = [(self.goal_x, self.goal_y)]
        self.waypoint_index = 0

        # Rotate commanded waypoints (for goto mode)
        if self.mode == 'goto' and abs(self.command_frame_offset) > 1e-9:
            self.waypoints = [rotate_xy(float(x), float(y), self.command_frame_offset) for (x, y) in self.waypoints]

        # Circular trajectory parameters
        self.radius = 0.5
        self.period = 10.0

        # Robot state
        self.current_x     = 0.0
        self.current_y     = 0.0
        self.current_theta = 0.0
        self.odom_received = False

        # Origin
        self.origin_x     = None
        self.origin_y     = None
        self.origin_theta = None
        self.origin_set   = False
        self._stable_readings = []

        # Simulator goal tracking
        self.sim_goal_reached = False
        self._sim_active_goal = None
        self._sim_goal_sent_time = None

        # --- Object pose state ---
        self._pose_buffer = []                         # list of (x, y) from received poses
        self._object_pose_ready = False                # True once we have enough samples
        self._object_goal_set = False                  # True once we've computed standoff goal
        self._object_goal_x = 0.0
        self._object_goal_y = 0.0
        self._object_face_yaw = 0.0            # yaw to face the object
        self._object_aligning = False           # True after reaching standoff, aligning yaw
        self._waiting_start_time = None         # when we started waiting for pose

        # Data storage
        self.data = {
            'time': [], 'ref_x': [], 'ref_y': [],
            'actual_x': [], 'actual_y': [],
            'error_x': [], 'error_y': []
        }

        self.start_time = None
        self.running    = True

        # --- Print startup info ---
        self.get_logger().info(f'=== navigate_to_object.py  v{VERSION} ===')
        self.get_logger().info(f'Robot: {self.robot.upper()} | Mode: {self.mode}')
        self.get_logger().info(f'Origin file: {self.origin_file}')
        self.get_logger().info(f'yaw_offset = {self.yaw_offset:.4f} rad ({np.degrees(self.yaw_offset):.2f}°)')
        if self.mode == 'object':
            self.get_logger().info(
                f'Object mode: standoff={self.standoff_dist} m, '
                f'pose_timeout={self.pose_timeout} s, face_object={self.face_object}'
            )

        # --- Handle reset_origin immediately ---
        if self.mode == 'reset_origin':
            self._reset_saved_origin()
            return

        # Load saved origin (skip for test mode)
        if self.mode != 'test':
            self._load_origin()

        # --- Publishers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_complete_pub = self.create_publisher(Bool, '/coordinator/nav_complete', 10)

        if self.robot == 'sim':
            self.target_pub = self.create_publisher(Pose2D, '/base/target_pose', 10)
            self.create_subscription(Bool, '/base/goal_reached', self._sim_goal_callback, 10)

        # --- TF2 for base_link -> odom transform ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- Subscribers ---
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Subscribe to object pose from detect_object_real.py (in base_link frame)
        self.create_subscription(
            PoseStamped, '/perception/object_pose', self._object_pose_callback, 10
        )

        # Coordinator nav_goal: when received, reset state and start navigating to new object pose
        self.create_subscription(
            PoseStamped, '/coordinator/nav_goal', self._nav_goal_callback, 10
        )

        # --- Control timer (50 Hz) ---
        self.create_timer(0.02, self.control_loop)

        if self.mode == 'goto':
            self.get_logger().info(f'Waypoints: {self.waypoints} | tolerance={self.goal_tol:.3f} m')
        if self.mode == 'object':
            self.get_logger().info('Waiting for object pose on /perception/object_pose ...')

    # ------------------------------------------------------------------
    # Coordinator nav_goal callback
    # ------------------------------------------------------------------
    def _nav_goal_callback(self, msg: PoseStamped):
        """Coordinator sends a new navigation goal — reset state and start navigating."""
        self.get_logger().info(
            f'Received nav_goal from coordinator: ({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f})'
        )
        # Reset object navigation state
        self._pose_buffer = []
        self._object_pose_ready = False
        self._object_goal_set = False
        self._object_aligning = False
        self._waiting_start_time = None
        self._final_yaw_done = False
        self.start_time = None
        # Re-enable navigation
        self.mode = 'object'
        self.running = True

    # ------------------------------------------------------------------
    # Object pose callback
    # ------------------------------------------------------------------
    #
    # Object navigation coordinate transform sequence:
    #
    #   Step 1 (here, _object_pose_callback):
    #     - Receive object pose from detect_object_real.py in base_link frame
    #     - Rotate 180° to convert base_link → odom frame
    #       (base_link: +x=left, +y=back; odom: +x=right, +y=forward)
    #     - Accumulate N samples (default 5) and average to reduce noise
    #
    #   Step 2 (_compute_object_goal):
    #     - Transform averaged odom pose → origin frame (subtract saved origin)
    #     - Compute standoff position (back off standoff_dist from object)
    #     - Apply lateral offset (shift left to line up right arm)
    #     - Rotate by COMMAND_FRAME_OFFSET_DEG (-90°) to match movement_4.py
    #       command frame convention
    #     - Compute face_yaw from standoff position toward object
    #
    #   Step 3 (control_loop → go_to_position):
    #     - Navigate to the standoff goal using proportional controller
    #     - Align yaw to face the object
    #
    def _object_pose_callback(self, msg: PoseStamped):
        """Step 1: Accumulate object pose samples, converting base_link → odom."""
        if self._object_goal_set:
            return  # already locked goal, ignore further poses

        # base_link has +x=left, +y=back; odom has +x=right, +y=forward
        # so we apply a 180° rotation: (x, y) -> (-x, -y)
        odom_x = -msg.pose.position.x
        odom_y = -msg.pose.position.y
        self._pose_buffer.append((odom_x, odom_y))

        self.get_logger().info(
            f'Object pose sample {len(self._pose_buffer)}/{self.pose_samples}: '
            f'base_link=({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}) -> '
            f'odom=({odom_x:.3f}, {odom_y:.3f})'
        )

        if len(self._pose_buffer) >= self.pose_samples:
            self._object_pose_ready = True

    # ------------------------------------------------------------------
    # Simulator goal callback
    # ------------------------------------------------------------------
    def _sim_goal_callback(self, msg: Bool):
        if msg.data:
            self.sim_goal_reached = True
            self.get_logger().info('Simulator: goal reached!')

    # ------------------------------------------------------------------
    # Odometry callback (same as movement_4.py)
    # ------------------------------------------------------------------
    def odom_callback(self, msg: Odometry):
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        odom_theta = quat_to_yaw(q.x, q.y, q.z, q.w)
        raw_theta = wrap_angle(odom_theta + self.yaw_offset)

        # Test mode: raw position
        if self.mode == 'test':
            self.current_x     = raw_x
            self.current_y     = raw_y
            self.current_theta = raw_theta
            self.odom_received = True
            return

        # Lock origin once robot is confirmed stationary
        if not self.origin_set:
            self._stable_readings.append((raw_x, raw_y, raw_theta))
            if len(self._stable_readings) >= self._stable_required:
                xs  = [r[0] for r in self._stable_readings]
                ys  = [r[1] for r in self._stable_readings]
                ths = [r[2] for r in self._stable_readings]

                xy_ok = (max(xs) - min(xs) < self._stable_threshold_xy and
                         max(ys) - min(ys) < self._stable_threshold_xy)
                th_ok = (circular_range(ths) < self._stable_threshold_th)

                if xy_ok and th_ok:
                    avg_x = float(np.mean(xs))
                    avg_y = float(np.mean(ys))
                    avg_theta = circular_mean(ths)
                    self._set_origin(avg_x, avg_y, avg_theta)
                else:
                    self._stable_readings.pop(0)
            return

        # Express current pose relative to fixed origin frame
        dx = raw_x - self.origin_x
        dy = raw_y - self.origin_y
        dtheta = wrap_angle(raw_theta - self.origin_theta)

        c, s = np.cos(-self.origin_theta), np.sin(-self.origin_theta)
        self.current_x = c * dx + s * dy
        self.current_y = -s * dx + c * dy
        self.current_theta = dtheta

        self.odom_received = True

    # ------------------------------------------------------------------
    # Origin helpers (same as movement_4.py)
    # ------------------------------------------------------------------
    def _set_origin(self, raw_x, raw_y, raw_theta):
        self.origin_x = float(raw_x)
        self.origin_y = float(raw_y)
        self.origin_theta = float(raw_theta)
        self.origin_set = True
        self._save_origin()
        self.get_logger().info(
            f'Origin locked (robot stable): ({raw_x:.3f}, {raw_y:.3f}, {np.degrees(raw_theta):.1f}°) '
            f'→ saved as (0, 0, 0°)'
        )

    def _save_origin(self):
        try:
            with open(self.origin_file, 'w') as f:
                f.write(f'{self.origin_x},{self.origin_y},{self.origin_theta}\n')
            self.get_logger().info(f'Origin saved to {self.origin_file}')
        except Exception as e:
            self.get_logger().warning(f'Could not save origin: {e}')

    def _load_origin(self):
        if not os.path.exists(self.origin_file):
            self.get_logger().info('No saved origin — will lock on first stable odom.')
            return
        try:
            with open(self.origin_file, 'r') as f:
                parts = f.read().strip().split(',')
            self.origin_x = float(parts[0])
            self.origin_y = float(parts[1])
            self.origin_theta = float(parts[2])
            self.origin_set = True
            self.get_logger().info(
                f'Origin reloaded: ({self.origin_x:.3f}, {self.origin_y:.3f}, {np.degrees(self.origin_theta):.1f}°)'
            )
        except Exception as e:
            self.get_logger().warning(f'Could not load origin: {e}')
            self.origin_set = False

    def _reset_saved_origin(self):
        if os.path.exists(self.origin_file):
            os.remove(self.origin_file)
            self.get_logger().info(
                f'Origin deleted ({self.origin_file}). Next movement will define a new (0, 0).'
            )
        else:
            self.get_logger().warning('No saved origin file found — nothing to delete.')
        self.origin_set = False

    # ------------------------------------------------------------------
    # Compute standoff goal from object pose
    # ------------------------------------------------------------------
    def _compute_object_goal(self):
        """Step 2: Average pose samples, transform to origin frame, compute standoff + lateral offset."""
        xs = [p[0] for p in self._pose_buffer]
        ys = [p[1] for p in self._pose_buffer]
        obj_x_odom = float(np.mean(xs))
        obj_y_odom = float(np.mean(ys))

        self.get_logger().info(
            f'Averaged {len(self._pose_buffer)} pose samples -> '
            f'odom ({obj_x_odom:.3f}, {obj_y_odom:.3f})'
        )

        # Transform object pose from odom frame to origin frame
        # (same transform as we apply to robot pose in odom_callback)
        dx = obj_x_odom - self.origin_x
        dy = obj_y_odom - self.origin_y
        c, s = np.cos(-self.origin_theta), np.sin(-self.origin_theta)
        obj_x = float(c * dx + s * dy)
        obj_y = float(-s * dx + c * dy)

        # Compute standoff: position standoff_dist away from object, toward robot
        dx = obj_x - self.current_x
        dy = obj_y - self.current_y
        dist = math.hypot(dx, dy)

        if dist <= self.standoff_dist:
            # Already inside standoff radius — just stay and align
            self._object_goal_x = self.current_x
            self._object_goal_y = self.current_y
        else:
            # Unit vector from robot toward object
            ux = dx / dist
            uy = dy / dist
            # Standoff: back off along that vector
            self._object_goal_x = obj_x - ux * self.standoff_dist
            self._object_goal_y = obj_y - uy * self.standoff_dist
            # Lateral offset: perpendicular to robot→object direction
            # Positive = left when facing the object (to line up right arm)
            if abs(self.lateral_offset) > 1e-6:
                # Left perpendicular: rotate (ux, uy) by +90°
                perp_x = -uy
                perp_y = ux
                self._object_goal_x += perp_x * self.lateral_offset
                self._object_goal_y += perp_y * self.lateral_offset

        # Apply command-frame rotation (same as goto mode applies to waypoints)
        if abs(self.command_frame_offset) > 1e-9:
            self._object_goal_x, self._object_goal_y = rotate_xy(
                self._object_goal_x, self._object_goal_y, self.command_frame_offset
            )
            obj_x, obj_y = rotate_xy(obj_x, obj_y, self.command_frame_offset)

        # Yaw to face the object from the standoff position
        self._object_face_yaw = math.atan2(
            obj_y - self._object_goal_y,
            obj_x - self._object_goal_x,
        )

        self._object_goal_set = True
        self.get_logger().info(
            f'Object in odom: ({obj_x_odom:.3f}, {obj_y_odom:.3f}) | '
            f'origin frame: ({obj_x:.3f}, {obj_y:.3f})'
        )
        # Convert standoff goal back to odom for logging
        c, s = np.cos(self.origin_theta), np.sin(self.origin_theta)
        standoff_odom_x = float(c * self._object_goal_x - s * self._object_goal_y) + self.origin_x
        standoff_odom_y = float(s * self._object_goal_x + c * self._object_goal_y) + self.origin_y
        self.get_logger().info(
            f'Going to odom=({standoff_odom_x:.3f}, {standoff_odom_y:.3f}) '
            f'({self.standoff_dist:.2f}m standoff from object)'
        )
        self.get_logger().info(
            f'Standoff in origin frame: ({self._object_goal_x:.3f}, {self._object_goal_y:.3f}), '
            f'face_yaw={np.degrees(self._object_face_yaw):.1f}°'
        )

    # ------------------------------------------------------------------
    # Reference trajectory (circular) — from movement_4.py
    # ------------------------------------------------------------------
    def get_reference_trajectory(self, t):
        omega = 2.0 * np.pi / self.period
        ref_x = self.radius * np.cos(omega * t)
        ref_y = self.radius * np.sin(omega * t)
        ref_vx = -self.radius * omega * np.sin(omega * t)
        ref_vy = self.radius * omega * np.cos(omega * t)
        if abs(self.command_frame_offset) > 1e-9:
            ref_x, ref_y = rotate_xy(ref_x, ref_y, self.command_frame_offset)
            ref_vx, ref_vy = rotate_xy(ref_vx, ref_vy, self.command_frame_offset)
        return ref_x, ref_y, ref_vx, ref_vy

    # ------------------------------------------------------------------
    # Test mode — from movement_4.py
    # ------------------------------------------------------------------
    def run_test(self):
        if self.robot == 'sim':
            self._run_test_sim(self.test)
        else:
            self._run_test_real(self.test)
        self.stop_robot()
        self.get_logger().info(f'TEST {self.test.upper()} complete.')
        self.running = False

    def _run_test_sim(self, test):
        pose = Pose2D()
        if test == 'forward':
            pose.x, pose.y, pose.theta = self.test_distance, 0.0, 0.0
        elif test == 'backward':
            pose.x, pose.y, pose.theta = -self.test_distance, 0.0, 0.0
        elif test == 'left':
            th = np.degrees(self.test_angle) if self.sim_pose_theta_degrees else self.test_angle
            pose.x, pose.y, pose.theta = 0.0, 0.0, float(th)
        elif test == 'right':
            th = -np.degrees(self.test_angle) if self.sim_pose_theta_degrees else -self.test_angle
            pose.x, pose.y, pose.theta = 0.0, 0.0, float(th)
        else:
            self.get_logger().error(f'Unknown test "{test}".')
            return

        if test in ('forward', 'backward') and abs(self.command_frame_offset) > 1e-9:
            rx, ry = rotate_xy(float(pose.x), float(pose.y), self.command_frame_offset)
            pose.x, pose.y = float(rx), float(ry)

        self.sim_goal_reached = False
        self.target_pub.publish(pose)
        timeout = time.time() + 30.0
        while not self.sim_goal_reached and time.time() < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.sim_goal_reached:
            self.get_logger().warning('TEST timed out waiting for /base/goal_reached.')

    def _run_test_real(self, test):
        self.get_logger().info('Waiting for odometry...')
        timeout = time.time() + 5.0
        while not self.odom_received and time.time() < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.odom_received:
            self.get_logger().error('No odometry received!')
            return

        cmd = Twist()
        if test in ('forward', 'backward'):
            start_x, start_y = self.current_x, self.current_y
            sign = 1.0 if test == 'forward' else -1.0
            cmd.linear.x = sign * self.max_v
            while True:
                travelled = np.hypot(self.current_x - start_x, self.current_y - start_y)
                if travelled >= self.test_distance:
                    break
                self.cmd_vel_pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.05)
        elif test in ('left', 'right'):
            sign = 1.0 if test == 'left' else -1.0
            cmd.angular.z = sign * self.max_omega
            rotated = 0.0
            prev_theta = self.current_theta
            while True:
                d_theta = wrap_angle(self.current_theta - prev_theta)
                rotated += abs(d_theta)
                prev_theta = self.current_theta
                if rotated >= self.test_angle:
                    break
                self.cmd_vel_pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.05)

    # ------------------------------------------------------------------
    # Go-to-position (same as movement_4.py)
    # ------------------------------------------------------------------
    def go_to_position(self, goal_x: float, goal_y: float, tolerance: float = None) -> bool:
        if tolerance is None:
            tolerance = self.goal_tol

        # --- SIM path ---
        if self.robot == 'sim':
            if self._sim_active_goal != (float(goal_x), float(goal_y)):
                pose = Pose2D()
                pose.x = float(goal_x)
                pose.y = float(goal_y)
                pose.theta = 0.0

                self.sim_goal_reached = False
                self._sim_active_goal = (float(goal_x), float(goal_y))
                self._sim_goal_sent_time = time.time()
                self.target_pub.publish(pose)
                self.get_logger().info(f'SIM goto: published target_pose ({pose.x:.3f}, {pose.y:.3f})')

            if self.sim_goal_reached:
                self.get_logger().info(f'Goal ({goal_x:.3f}, {goal_y:.3f}) reached (sim).')
                self._sim_active_goal = None
                self._sim_goal_sent_time = None
                return True

            if self._sim_goal_sent_time is not None and (time.time() - self._sim_goal_sent_time) > 60.0:
                self.get_logger().warning('SIM goto: timed out. Republishing target.')
                self._sim_active_goal = None
            return False

        # --- REAL path ---
        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        distance = np.hypot(dx, dy)

        if distance < tolerance:
            self.stop_robot()
            self.get_logger().info(f'Goal ({goal_x:.3f}, {goal_y:.3f}) reached! Error={distance:.4f} m')
            return True

        desired_heading = np.arctan2(dy, dx)
        heading_error = wrap_angle(desired_heading - self.current_theta)

        v = np.clip(self.kp * distance, -self.max_v, self.max_v)
        omega = np.clip(2.0 * self.kp * heading_error, -self.max_omega, self.max_omega)

        if abs(heading_error) > np.pi / 4:
            v *= 0.3

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)
        self.cmd_vel_pub.publish(cmd)
        return False

    # ------------------------------------------------------------------
    # Align yaw to face the object
    # ------------------------------------------------------------------
    def _align_to_object(self) -> bool:
        """Rotate in place to face the object. Returns True when aligned."""
        yaw_error = wrap_angle(self._object_face_yaw - self.current_theta)

        if abs(yaw_error) <= self.yaw_tolerance:
            self.stop_robot()
            self.get_logger().info(
                f'Aligned to face object! yaw_error={np.degrees(yaw_error):.2f}°'
            )
            return True

        omega = float(np.clip(self.yaw_kp * yaw_error, -self.max_omega, self.max_omega))

        if self.robot == 'sim':
            pose = Pose2D()
            pose.x = float(self.current_x)
            pose.y = float(self.current_y)
            pose.theta = float(self._object_face_yaw)
            self.target_pub.publish(pose)
        else:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = omega
            self.cmd_vel_pub.publish(cmd)
        return False

    # ------------------------------------------------------------------
    # Final yaw alignment for goto mode (from movement_4.py)
    # ------------------------------------------------------------------
    def align_final_yaw(self) -> bool:
        if np.isnan(self.goal_theta_deg):
            self._final_yaw_done = True
            return True

        target = wrap_angle(float(np.radians(self.goal_theta_deg) + self.command_frame_offset))
        err = wrap_angle(target - float(self.current_theta))

        if abs(err) <= self.yaw_tolerance:
            self.stop_robot()
            if not self._final_yaw_done:
                self.get_logger().info(f'Final yaw aligned: target={np.degrees(target):.1f}°')
            self._final_yaw_done = True
            return True

        omega = float(np.clip(self.yaw_kp * err, -self.max_omega, self.max_omega))
        if self.robot == 'sim':
            pose = Pose2D()
            pose.x = float(self.current_x)
            pose.y = float(self.current_y)
            theta_to_send = target
            if self.sim_pose_theta_degrees:
                theta_to_send = float(np.degrees(theta_to_send))
            pose.theta = float(theta_to_send)
            self.target_pub.publish(pose)
        else:
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = omega
            self.cmd_vel_pub.publish(cmd)
        return False

    # ------------------------------------------------------------------
    # Main control loop (50 Hz)
    # ------------------------------------------------------------------
    def control_loop(self):
        if self.mode == 'reset_origin' or not self.odom_received or not self.running:
            return

        if self.start_time is None:
            self.start_time = self.get_clock().now()

        t = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        # ---- Mode: test -----------------------------------------------
        if self.mode == 'test':
            self.run_test()
            return

        # ---- Mode: object (NEW) ---------------------------------------
        if self.mode == 'object':
            # Start pose timeout on first control loop tick (after odom is ready)
            if self._waiting_start_time is None:
                self._waiting_start_time = time.time()

            # Phase 1: Wait for object pose
            if not self._object_goal_set:
                if self._object_pose_ready:
                    self._compute_object_goal()
                elif time.time() - self._waiting_start_time > self.pose_timeout:
                    self.get_logger().error(
                        f'No object pose received within {self.pose_timeout} s. Stopping.'
                    )
                    self.stop_robot()
                    self.running = False
                return

            # Phase 2: Navigate to standoff position
            if not self._object_aligning:
                if self.go_to_position(self._object_goal_x, self._object_goal_y):
                    if self.face_object:
                        self._object_aligning = True
                        self.get_logger().info('Standoff reached. Aligning to face object...')
                    else:
                        self.get_logger().info('Navigation to object complete!')
                        self.nav_complete_pub.publish(Bool(data=True))
                        self.running = False
                        self.save_results()
                return

            # Phase 3: Align yaw to face the object
            if self._align_to_object():
                self.get_logger().info('Navigation to object complete!')
                self.nav_complete_pub.publish(Bool(data=True))
                self.running = False
                self.save_results()
            return

        # ---- Mode: goto -----------------------------------------------
        if self.mode == 'goto':
            if self.waypoint_index >= len(self.waypoints):
                if self.running:
                    self.running = False
                    self.get_logger().info('All waypoints completed.')
                    self.save_results()
                return

            goal_x, goal_y = self.waypoints[self.waypoint_index]

            if (self.waypoint_index == len(self.waypoints) - 1) and (not self._final_yaw_done):
                if self.go_to_position(goal_x, goal_y):
                    if self.align_final_yaw():
                        self.waypoint_index += 1
                return

            if self.go_to_position(goal_x, goal_y):
                self.waypoint_index += 1
                if self.waypoint_index < len(self.waypoints):
                    nx, ny = self.waypoints[self.waypoint_index]
                    self.get_logger().info(
                        f'Moving to waypoint {self.waypoint_index + 1}/{len(self.waypoints)}: ({nx:.3f}, {ny:.3f})'
                    )
                else:
                    self._final_yaw_done = True
            return

        # ---- Mode: circular trajectory --------------------------------
        if t > self.duration:
            if self.running:
                self.running = False
                self.stop_robot()
                self.get_logger().info('Trajectory complete.')
                self.save_results()
            return

        ref_x, ref_y, ref_vx, ref_vy = self.get_reference_trajectory(t)
        error_x = ref_x - self.current_x
        error_y = ref_y - self.current_y

        vx_des = self.kp * error_x + ref_vx
        vy_des = self.kp * error_y + ref_vy

        theta = self.current_theta
        v = vx_des * np.cos(theta) + vy_des * np.sin(theta)

        desired_heading = np.arctan2(vy_des, vx_des)
        heading_error = wrap_angle(desired_heading - theta)
        omega = 2.0 * self.kp * heading_error

        v = np.clip(v, -self.max_v, self.max_v)
        omega = np.clip(omega, -self.max_omega, self.max_omega)

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)
        self.cmd_vel_pub.publish(cmd)

        self.data['time'].append(t)
        self.data['ref_x'].append(ref_x)
        self.data['ref_y'].append(ref_y)
        self.data['actual_x'].append(self.current_x)
        self.data['actual_y'].append(self.current_y)
        self.data['error_x'].append(error_x)
        self.data['error_y'].append(error_y)

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def stop_robot(self):
        try:
            self.cmd_vel_pub.publish(Twist())
        except Exception:
            pass

    def save_results(self):
        if not self.save_data or len(self.data['time']) == 0:
            return
        data_dir = os.path.expanduser('~/tidybot_trajectory_data')
        os.makedirs(data_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'trajectory_kp{self.kp}_{timestamp}.csv'
        filepath = os.path.join(data_dir, filename)
        with open(filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'ref_x', 'ref_y', 'actual_x', 'actual_y', 'error_x', 'error_y'])
            for i in range(len(self.data['time'])):
                writer.writerow([
                    self.data['time'][i], self.data['ref_x'][i], self.data['ref_y'][i],
                    self.data['actual_x'][i], self.data['actual_y'][i],
                    self.data['error_x'][i], self.data['error_y'][i]
                ])
        self.get_logger().info(f'Data saved to: {filepath}')


def main(args=None):
    rclpy.init(args=args)
    node = NavigateToObject()

    if getattr(node, 'mode', None) == 'reset_origin':
        try:
            node.destroy_node()
        finally:
            rclpy.shutdown()
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
