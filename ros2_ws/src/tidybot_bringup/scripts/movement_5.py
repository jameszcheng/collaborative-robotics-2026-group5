#!/usr/bin/env python3
"""
movement_5.py — TidyBot2 Circular Trajectory Tracking + Go-To-Position
v1.0.9

Fixes over movement_4.py:
  1. Test rotation: proportional slowdown in last ~20° prevents angle overshoot
  2. Test forward/backward: proportional slowdown before target distance
  3. go_to_position: removed dead sim branch inside real path (ref_x/ref_y bug)
  4. go_to_position: heading threshold tightened pi/4→pi/6 (45°→30°) for earlier speed reduction

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
TERMINAL 1 — Start the robot
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  # Simulator
  ros2 launch tidybot_bringup sim.launch.py

  # Real robot
  ros2 launch tidybot_bringup real.launch.py

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
TERMINAL 2 — BASIC TESTS (run these first)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  SIM (uses /base/target_pose + /base/goal_reached):
  python3 movement_4.py --ros-args -p mode:=test -p test:=forward
  python3 movement_4.py --ros-args -p mode:=test -p test:=backward
  python3 movement_4.py --ros-args -p mode:=test -p test:=left
  python3 movement_4.py --ros-args -p mode:=test -p test:=right

  REAL (uses /cmd_vel + /odom):
  python3 movement_4.py --ros-args -p mode:=test -p test:=forward -p robot:=real
  python3 movement_4.py --ros-args -p mode:=test -p test:=backward -p robot:=real
  python3 movement_4.py --ros-args -p mode:=test -p test:=left -p robot:=real
  python3 movement_4.py --ros-args -p mode:=test -p test:=right -p robot:=real

  Custom distance (m) or angle (deg):
  python3 movement_4.py --ros-args -p mode:=test -p test:=forward -p test_distance:=1.0 -p robot:=real
  python3 movement_4.py --ros-args -p mode:=test -p test:=left -p test_angle:=180.0 -p robot:=real

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
TERMINAL 2 — MOVEMENT COMMANDS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  1) Reset origin  (recommended before REAL runs; robot must be stationary)
     python3 movement_4.py --ros-args -p mode:=reset_origin
     python3 movement_4.py --ros-args -p mode:=reset_origin -p robot:=real

  2) Go to position (metres relative to origin)
     python3 movement_4.py --ros-args -p mode:=goto -p goal_x:=1.0 -p goal_y:=0.0
     python3 movement_4.py --ros-args -p mode:=goto -p goal_x:=1.0 -p goal_y:=0.0 -p robot:=real

  3) Multiple waypoints — flat list [x1,y1, x2,y2, ...]
     python3 movement_4.py --ros-args -p mode:=goto -p waypoints:="[1.0,0.0, 1.0,1.0, 0.0,0.0]"
     python3 movement_4.py --ros-args -p mode:=goto -p waypoints:="[1.0,0.0, 1.0,1.0, 0.0,0.0]" -p robot:=real

  4) Circular trajectory
     python3 movement_4.py --ros-args -p mode:=circle
     python3 movement_4.py --ros-args -p mode:=circle -p robot:=real

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
OPTIONAL PARAMETERS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  -p robot:=real                # real robot (default: sim)
  -p kp:=1.5                    # proportional gain (default: 1.0)
  -p max_v:=0.2                 # max linear speed m/s (default: 0.2)
  -p max_omega:=2.0             # max rotation speed rad/s (default: 2.0)
  -p goal_tolerance:=0.05       # goal tolerance metres (default: 0.05)
  -p duration:=20.0             # circle duration (s)

  # NEW (sim↔real robustness)
  -p yaw_offset:=0.0            # radians added to yaw from odom (default: sim=-pi/2, real=0)
  -p origin_namespace:=real     # origin file suffix (default: robot name; separates sim vs real)
  -p sim_pose_theta_degrees:=false  # if your sim base expects Pose2D.theta in degrees (default false=radians)

  # NEW (final yaw at end of goto)
  -p goal_theta_deg:=nan          # desired final yaw in *command frame* degrees (default: NaN disables)
  -p yaw_tolerance_deg:=5.0       # final yaw tolerance (deg)
  -p yaw_kp:=2.0                  # P gain for final yaw rotation

  # Origin locking stability tuning
  -p stable_required:=25            # samples to confirm stable (default 25 @50Hz ≈0.5s)
  -p stable_xy_threshold:=0.005     # metres (default 0.005)
  -p stable_theta_threshold:=0.02   # radians (default 0.02 ≈1.15°)

Notes:
- This version uses robust quaternion→yaw (x,y,z,w) and separate origin files for sim/real to
  avoid “disoriented” behavior caused by mixed coordinate conventions.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import numpy as np
import csv
import os
import time
from datetime import datetime

VERSION = "1.1.0"



# Hardcoded command-frame rotation (applies to BOTH sim and real).
# Positive = CCW rotation of commanded (x,y).
# Example: +90° means goal_x=1,goal_y=0 behaves like (0,1) in the original frame.
COMMAND_FRAME_OFFSET_DEG = 90.0
def wrap_angle(a: float) -> float:
    return float(np.arctan2(np.sin(a), np.cos(a)))



def rotate_xy(x: float, y: float, angle_rad: float):
    """Rotate (x,y) by angle_rad (CCW)."""
    c, s = float(np.cos(angle_rad)), float(np.sin(angle_rad))
    return float(c * x - s * y), float(s * x + c * y)


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """
    Standard yaw-from-quaternion (robust even with small roll/pitch).
    yaw = atan2(2(wz + xy), 1 - 2(y^2 + z^2))
    """
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


class TrajectoryTracker(Node):
    def __init__(self):
        super().__init__('trajectory_tracker')

        # --- Parameters ---
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('save_data', True)
        self.declare_parameter('duration', 20.0)
        self.declare_parameter('mode', 'circle')        # 'circle','goto','reset_origin','test'
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_tolerance', 0.05)
        self.declare_parameter('waypoints', [0.0])
        self.declare_parameter('max_v', 0.2)
        self.declare_parameter('max_omega', 2.0)
        self.declare_parameter('robot', 'sim')          # 'sim' or 'real'
        self.declare_parameter('test', 'forward')       # 'forward','backward','left','right'
        self.declare_parameter('test_distance', 0.5)    # metres
        self.declare_parameter('test_angle', 90.0)      # degrees

        # NEW: robustness knobs
        self.declare_parameter('yaw_offset', float('nan'))       # radians; NaN => auto (sim:-pi/2, real:0)
        self.declare_parameter('origin_namespace', '')           # '' => auto from robot ('sim'/'real')
        self.declare_parameter('sim_pose_theta_degrees', False)  # Pose2D.theta in degrees? (default radians)

        # NEW: enforce final yaw at end of goto (degrees in command frame)
        self.declare_parameter('goal_theta_deg', float('nan'))   # NaN => disabled
        self.declare_parameter('yaw_tolerance_deg', 5.0)
        self.declare_parameter('yaw_kp', 2.0)

        # Origin locking stability knobs
        self.declare_parameter('stable_xy_threshold', 0.005)     # metres
        self.declare_parameter('stable_theta_threshold', 0.02)   # radians
        self.declare_parameter('stable_required', 25)            # samples (~0.5s at 50Hz)

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

        # Hardcoded command-frame rotation for all goals/waypoints/trajectory
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

        # Build waypoint list
        raw_wp = self.get_parameter('waypoints').value
        if len(raw_wp) >= 2 and not (len(raw_wp) == 1 and raw_wp[0] == 0.0):
            if len(raw_wp) % 2 != 0:
                self.get_logger().error(
                    f'Parameter "waypoints" must have an even number of values [x1,y1,x2,y2,...]. Got {len(raw_wp)} values: {list(raw_wp)}'
                )
                self.waypoints = [(self.goal_x, self.goal_y)]
            else:
                self.waypoints = [(raw_wp[i], raw_wp[i + 1]) for i in range(0, len(raw_wp), 2)]
        else:
            self.waypoints = [(self.goal_x, self.goal_y)]
        self.waypoint_index = 0

        # Rotate commanded waypoints in the XY plane so that user commands are interpreted in a rotated frame.
        if abs(self.command_frame_offset) > 1e-9:
            self.waypoints = [rotate_xy(float(x), float(y), self.command_frame_offset) for (x, y) in self.waypoints]
            self.get_logger().info(
                f'Waypoints rotated by {np.degrees(self.command_frame_offset):.1f}° (hardcoded COMMAND_FRAME_OFFSET_DEG).'
            )

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
        self._stable_readings = []   # list of (x,y,theta)

        # Simulator goal tracking
        self.sim_goal_reached = False
        self._sim_active_goal = None  # (x,y)
        self._sim_goal_sent_time = None

        # Data storage
        self.data = {
            'time': [], 'ref_x': [], 'ref_y': [],
            'actual_x': [], 'actual_y': [],
            'error_x': [], 'error_y': []
        }

        self.start_time = None
        self.running    = True

        # --- Print startup info ---
        self.get_logger().info(f'=== movement_5.py  v{VERSION} ===')
        self.get_logger().info(f'Robot: {self.robot.upper()} | Mode: {self.mode}')
        self.get_logger().info(f'Origin file: {self.origin_file}')
        self.get_logger().info(f'yaw_offset = {self.yaw_offset:.4f} rad ({np.degrees(self.yaw_offset):.2f}°)')

        # --- Handle reset_origin immediately ---
        if self.mode == 'reset_origin':
            self._reset_saved_origin()
            return

        # Load saved origin (skip for test mode — uses raw odom directly)
        if self.mode != 'test':
            self._load_origin()

        # --- Publishers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        if self.robot == 'sim':
            self.target_pub = self.create_publisher(Pose2D, '/base/target_pose', 10)
            self.create_subscription(Bool, '/base/goal_reached', self._sim_goal_callback, 10)

        # --- Subscribers & control timer ---
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_timer(0.02, self.control_loop)  # 50 Hz

        if self.mode == 'goto':
            self.get_logger().info(f'Waypoints: {self.waypoints} | tolerance={self.goal_tol:.3f} m')
            if not np.isnan(self.goal_theta_deg):
                self.get_logger().info(
                    f'Final yaw enabled: goal_theta_deg={self.goal_theta_deg:.1f}° (command frame), '
                    f'tol={np.degrees(self.yaw_tolerance):.1f}°, yaw_kp={self.yaw_kp:.2f}'
                )
        if self.mode == 'test':
            self.get_logger().info(
                f'Test: {self.test.upper()} | '
                f'distance={self.test_distance:.2f} m | '
                f'angle={np.degrees(self.test_angle):.1f}°'
            )

    # ------------------------------------------------------------------
    # Simulator goal callback
    # ------------------------------------------------------------------
    def _sim_goal_callback(self, msg: Bool):
        if msg.data:
            self.sim_goal_reached = True
            self.get_logger().info('Simulator: goal reached!')

    # ------------------------------------------------------------------
    # Odometry callback
    # ------------------------------------------------------------------
    def odom_callback(self, msg: Odometry):
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation
        odom_theta = quat_to_yaw(q.x, q.y, q.z, q.w)
        raw_theta = wrap_angle(odom_theta + self.yaw_offset)

        # Test mode: just track raw position, no origin transform needed
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
                    # sliding window
                    self._stable_readings.pop(0)
            return

        # Express current pose relative to fixed origin frame
        dx = raw_x - self.origin_x
        dy = raw_y - self.origin_y
        dtheta = wrap_angle(raw_theta - self.origin_theta)

        # Rotate world->origin frame
        c, s = np.cos(-self.origin_theta), np.sin(-self.origin_theta)
        self.current_x = c * dx + s * dy
        self.current_y = -s * dx + c * dy
        self.current_theta = dtheta

        self.odom_received = True

    # ------------------------------------------------------------------
    # Origin helpers
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
    # Reference trajectory (circular)
    # ------------------------------------------------------------------
    def get_reference_trajectory(self, t):
        omega = 2.0 * np.pi / self.period
        ref_x = self.radius * np.cos(omega * t)
        ref_y = self.radius * np.sin(omega * t)
        ref_vx = -self.radius * omega * np.sin(omega * t)
        ref_vy = self.radius * omega * np.cos(omega * t)
        # Apply hardcoded command-frame rotation to the entire reference trajectory (position + velocity)
        if abs(self.command_frame_offset) > 1e-9:
            ref_x, ref_y = rotate_xy(ref_x, ref_y, self.command_frame_offset)
            ref_vx, ref_vy = rotate_xy(ref_vx, ref_vy, self.command_frame_offset)
        return ref_x, ref_y, ref_vx, ref_vy

    # ------------------------------------------------------------------
    # Test mode
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
        """
        Simulator test — publishes Pose2D target and waits for /base/goal_reached.
        Pose2D.theta is radians by default. If your sim expects degrees, set:
          -p sim_pose_theta_degrees:=true
        """
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
            self.get_logger().error(f'Unknown test "{test}". Use: forward, backward, left, right')
            return

        # Apply hardcoded command-frame rotation to translational SIM tests (forward/backward)
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
            self.get_logger().error(
                'No odometry received! Is the base node running?\n'
                'Launch with: ros2 launch tidybot_bringup real.launch.py'
            )
            return

        cmd = Twist()

        if test in ('forward', 'backward'):
            start_x = self.current_x
            start_y = self.current_y
            sign = 1.0 if test == 'forward' else -1.0
            slowdown_zone = 0.15   # m — start slowing in last 15 cm
            loop_timeout = time.time() + 30.0  # safety: stop if odom dies

            while time.time() < loop_timeout:
                travelled = np.hypot(self.current_x - start_x, self.current_y - start_y)
                if travelled >= self.test_distance:
                    self.cmd_vel_pub.publish(Twist())  # kill motors immediately
                    break
                remaining = self.test_distance - travelled
                if remaining < slowdown_zone:
                    speed = max(0.05, self.max_v * (remaining / slowdown_zone))
                else:
                    speed = self.max_v
                cmd = Twist()
                cmd.linear.x = sign * speed
                self.cmd_vel_pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.05)
            else:
                self.get_logger().error('TEST timed out (forward/backward) — stopping.')
                self.cmd_vel_pub.publish(Twist())

        elif test in ('left', 'right'):
            sign = 1.0 if test == 'left' else -1.0
            rotated = 0.0
            prev_theta = self.current_theta
            slowdown_zone = 0.35   # rad (~20°) — start slowing before target
            loop_timeout = time.time() + 30.0  # safety: stop if odom dies

            while time.time() < loop_timeout:
                d_theta = wrap_angle(self.current_theta - prev_theta)
                rotated += abs(d_theta)
                prev_theta = self.current_theta
                if rotated >= self.test_angle:
                    self.cmd_vel_pub.publish(Twist())  # kill motors immediately
                    break
                remaining = self.test_angle - rotated
                if remaining < slowdown_zone:
                    speed = max(0.1, self.max_omega * (remaining / slowdown_zone))
                else:
                    speed = self.max_omega
                cmd = Twist()
                cmd.angular.z = sign * speed
                self.cmd_vel_pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.05)
            else:
                self.get_logger().error('TEST timed out (rotation) — stopping.')
                self.cmd_vel_pub.publish(Twist())

        else:
            self.get_logger().error(f'Unknown test "{test}". Use: forward, backward, left, right')

    # ------------------------------------------------------------------
    # Go-to-position
    # ------------------------------------------------------------------
    def go_to_position(self, goal_x: float, goal_y: float, tolerance: float = None) -> bool:
        """Drive to (goal_x, goal_y) in the *origin* frame.

        REAL: velocity control via /cmd_vel.
        SIM : position goal via /base/target_pose + /base/goal_reached.
        """
        if tolerance is None:
            tolerance = self.goal_tol

        # --- SIM path: publish a Pose2D goal and wait for /base/goal_reached ---
        if self.robot == 'sim':
            # publish once per new goal
            if self._sim_active_goal != (float(goal_x), float(goal_y)):
                pose = Pose2D()
                pose.x = float(goal_x)
                pose.y = float(goal_y)
                pose.theta = 0.0  # not used for goto in this script

                self.sim_goal_reached = False
                self._sim_active_goal = (float(goal_x), float(goal_y))
                self._sim_goal_sent_time = time.time()
                self.target_pub.publish(pose)
                self.get_logger().info(f'SIM goto: published target_pose ({pose.x:.3f}, {pose.y:.3f})')

            # if sim reports reached, we are done
            if self.sim_goal_reached:
                self.get_logger().info(
                    f'Waypoint {self.waypoint_index + 1}/{len(self.waypoints)} '
                    f'({goal_x:.3f}, {goal_y:.3f}) reached (sim).'
                )
                self._sim_active_goal = None
                self._sim_goal_sent_time = None
                return True

            # optional timeout safety
            if self._sim_goal_sent_time is not None and (time.time() - self._sim_goal_sent_time) > 60.0:
                self.get_logger().warning('SIM goto: timed out waiting for /base/goal_reached. Republishing target.')
                self._sim_active_goal = None  # force republish next tick

            return False

        # --- REAL path: proportional velocity controller ---
        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        distance = np.hypot(dx, dy)

        if distance < tolerance:
            self.stop_robot()
            self.get_logger().info(
                f'Waypoint {self.waypoint_index + 1}/{len(self.waypoints)} '
                f'({goal_x:.3f}, {goal_y:.3f}) reached! Error={distance:.4f} m'
            )
            return True

        desired_heading = np.arctan2(dy, dx)
        heading_error = wrap_angle(desired_heading - self.current_theta)

        v = np.clip(self.kp * distance, -self.max_v, self.max_v)  # distance>=0 => v>=0 in practice
        omega = np.clip(2.0 * self.kp * heading_error, -self.max_omega, self.max_omega)

        if abs(heading_error) > np.pi / 6:   # 30° — tighter than movement_4's 45°
            v *= 0.3

        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)
        self.cmd_vel_pub.publish(cmd)
        return False

    # ------------------------------------------------------------------
    # Final yaw alignment (only for goto mode, after last waypoint)
    # ------------------------------------------------------------------
    def align_final_yaw(self) -> bool:
        # Disabled?
        if np.isnan(self.goal_theta_deg):
            self._final_yaw_done = True
            return True

        # goal_theta is specified in the same command frame as x/y.
        target = wrap_angle(float(np.radians(self.goal_theta_deg) + self.command_frame_offset))
        err = wrap_angle(target - float(self.current_theta))

        if abs(err) <= self.yaw_tolerance:
            self.stop_robot()
            if not self._final_yaw_done:
                self.get_logger().info(
                    f'Final yaw aligned: target={np.degrees(target):.1f}°, current={np.degrees(self.current_theta):.1f}° '
                    f'(tol={np.degrees(self.yaw_tolerance):.1f}°)'
                )
            self._final_yaw_done = True
            return True

        # Keep rotating in place toward the target
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

        # ---- Mode: goto -----------------------------------------------
        if self.mode == 'goto':
            if self.waypoint_index >= len(self.waypoints):
                if self.running:
                    self.running = False
                    self.get_logger().info('All waypoints completed.')
                    self.save_results()
                return

            goal_x, goal_y = self.waypoints[self.waypoint_index]

            # If we already reached the last waypoint and need final yaw alignment, do it here.
            if (self.waypoint_index == len(self.waypoints) - 1) and (not self._final_yaw_done):
                # We only start yaw alignment after position is reached.
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
                    # finished last waypoint (no final yaw requested)
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
    node = TrajectoryTracker()

    # If we only needed to reset/delete the origin file, exit cleanly.
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
