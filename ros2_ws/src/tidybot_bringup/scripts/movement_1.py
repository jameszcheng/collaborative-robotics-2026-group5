#!/usr/bin/env python3
"""
TidyBot2 Circular Trajectory Tracking + Go-To-Position
v1.0.5

Topics used:
- SIMULATOR: /base/target_pose (Pose2D), /base/goal_reached (Bool)
- REAL:      /cmd_vel (Twist), /odom (Odometry)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
TERMINAL 1 — Start the robot
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
   # Simulator
   ros2 launch tidybot_bringup sim.launch.py

   # Real robot
   ros2 launch tidybot_bringup real.launch.py

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
TERMINAL 2 — BASIC TESTS (run these first to verify robot works)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

   SIMULATOR tests (uses /base/target_pose — same as test_base_sim.py):
   python3 movement_1.py --ros-args -p mode:=test -p test:=forward
   python3 movement_1.py --ros-args -p mode:=test -p test:=backward
   python3 movement_1.py --ros-args -p mode:=test -p test:=left
   python3 movement_1.py --ros-args -p mode:=test -p test:=right

   REAL ROBOT tests (uses /cmd_vel + /odom — same as test_base_real.py):
   python3 movement_1.py --ros-args -p mode:=test -p test:=forward -p robot:=real
   python3 movement_1.py --ros-args -p mode:=test -p test:=backward -p robot:=real
   python3 movement_1.py --ros-args -p mode:=test -p test:=left -p robot:=real
   python3 movement_1.py --ros-args -p mode:=test -p test:=right -p robot:=real

   Custom distance (metres) or angle (degrees):
   python3 movement_1.py --ros-args -p mode:=test -p test:=forward -p test_distance:=1.0 -p robot:=real
   python3 movement_1.py --ros-args -p mode:=test -p test:=left -p test_angle:=180.0 -p robot:=real

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
TERMINAL 2 — MOVEMENT COMMANDS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

   1) Reset origin  — first time only, robot must be stationary
      python3 movement_1.py --ros-args -p mode:=reset_origin
      python3 movement_1.py --ros-args -p mode:=reset_origin -p robot:=real

   2) Go to position (metres relative to origin)
      python3 movement_1.py --ros-args -p mode:=goto -p goal_x:=1.0 -p goal_y:=0.0
      python3 movement_1.py --ros-args -p mode:=goto -p goal_x:=1.0 -p goal_y:=0.0 -p robot:=real

   3) Multiple waypoints — flat list [x1,y1, x2,y2, ...]
      python3 movement_1.py --ros-args -p mode:=goto -p waypoints:="[1.0,0.0, 1.0,1.0, 0.0,0.0]"
      python3 movement_1.py --ros-args -p mode:=goto -p waypoints:="[1.0,0.0, 1.0,1.0, 0.0,0.0]" -p robot:=real

   4) Circular trajectory
      python3 movement_1.py --ros-args -p mode:=circle
      python3 movement_1.py --ros-args -p mode:=circle -p robot:=real

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
OPTIONAL PARAMETERS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
   -p robot:=real           # real robot (default: sim)
   -p kp:=1.5               # proportional gain (default: 1.0)
   -p max_v:=0.2            # max linear speed m/s (default: 0.2)
   -p max_omega:=2.0        # max rotation speed rad/s (default: 2.0)
   -p goal_tolerance:=0.05  # goal tolerance metres (default: 0.05)
   -p test_distance:=0.5    # distance for forward/backward test (default: 0.5 m)
   -p test_angle:=90.0      # angle for left/right test (default: 90 degrees)
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


VERSION = "1.0.5"


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

        self.kp            = self.get_parameter('kp').value
        self.save_data     = self.get_parameter('save_data').value
        self.duration      = self.get_parameter('duration').value
        self.mode          = self.get_parameter('mode').value
        self.goal_x        = self.get_parameter('goal_x').value
        self.goal_y        = self.get_parameter('goal_y').value
        self.goal_tol      = self.get_parameter('goal_tolerance').value
        self.max_v         = self.get_parameter('max_v').value
        self.max_omega     = self.get_parameter('max_omega').value
        self.robot         = self.get_parameter('robot').value
        self.test          = self.get_parameter('test').value
        self.test_distance = self.get_parameter('test_distance').value
        self.test_angle    = np.radians(self.get_parameter('test_angle').value)

        # Build waypoint list
        raw_wp = self.get_parameter('waypoints').value
        if len(raw_wp) >= 2 and not (len(raw_wp) == 1 and raw_wp[0] == 0.0):
            self.waypoints = [(raw_wp[i], raw_wp[i+1]) for i in range(0, len(raw_wp)-1, 2)]
        else:
            self.waypoints = [(self.goal_x, self.goal_y)]
        self.waypoint_index = 0

        # Circular trajectory parameters
        self.radius = 0.5
        self.period = 10.0

        # Robot state
        self.current_x     = 0.0
        self.current_y     = 0.0
        self.current_theta = 0.0
        self.odom_received = False

        # Origin (persistent across runs via file)
        self.origin_x     = None
        self.origin_y     = None
        self.origin_theta = None
        self.origin_set   = False
        self.origin_file  = os.path.expanduser('~/.tidybot_origin.txt')

        # Stabilisation before locking origin
        self._stable_readings  = []
        self._stable_required  = 25      # ~0.5s at 50Hz
        self._stable_threshold = 0.005   # metres

        # Simulator goal tracking
        self.sim_goal_reached = False

        # Data storage
        self.data = {
            'time': [], 'ref_x': [], 'ref_y': [],
            'actual_x': [], 'actual_y': [],
            'error_x': [], 'error_y': []
        }

        self.start_time = None
        self.running    = True

        # --- Print startup info ---
        self.get_logger().info(f'=== movement_1.py  v{VERSION} ===')
        self.get_logger().info(f'Running from: {os.path.abspath(__file__)}')
        self.get_logger().info(f'Robot: {self.robot.upper()} | Mode: {self.mode}')

        # --- Handle reset_origin immediately, no ROS interfaces needed ---
        if self.mode == 'reset_origin':
            self._reset_saved_origin()
            return

        # Load saved origin (skip for test mode — uses raw odom directly)
        if self.mode != 'test':
            self._load_origin()

        # --- Publishers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Simulator additionally uses the high-level target_pose interface
        # (matches test_base_sim.py which uses /base/target_pose + /base/goal_reached)
        if self.robot == 'sim':
            self.target_pub = self.create_publisher(Pose2D, '/base/target_pose', 10)
            self.create_subscription(Bool, '/base/goal_reached',
                                     self._sim_goal_callback, 10)

        # --- Subscribers & control timer ---
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_timer(0.02, self.control_loop)  # 50 Hz

        if self.mode == 'goto':
            self.get_logger().info(
                f'Waypoints: {self.waypoints} | tolerance={self.goal_tol:.3f} m'
            )
        if self.mode == 'test':
            self.get_logger().info(
                f'Test: {self.test.upper()} | '
                f'distance={self.test_distance:.2f} m | '
                f'angle={np.degrees(self.test_angle):.1f}°'
            )

    # ------------------------------------------------------------------
    # Simulator goal callback (matches test_base_sim.py)
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

        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        odom_theta = 2.0 * np.arctan2(qz, qw)

        # Real robot: use quaternion yaw directly (matches test_base_real.py)
        # Simulator (MuJoCo): subtract π/2 to correct coordinate frame
        raw_theta = odom_theta - (np.pi / 2.0 if self.robot == 'sim' else 0.0)

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
                xs = [r[0] for r in self._stable_readings]
                ys = [r[1] for r in self._stable_readings]
                if (max(xs) - min(xs) < self._stable_threshold and
                        max(ys) - min(ys) < self._stable_threshold):
                    avg_x     = sum(xs) / len(xs)
                    avg_y     = sum(ys) / len(ys)
                    avg_theta = self._stable_readings[-1][2]
                    self._set_origin(avg_x, avg_y, avg_theta)
                else:
                    self._stable_readings.pop(0)
            return

        # Express current pose relative to fixed origin frame
        dx     = raw_x - self.origin_x
        dy     = raw_y - self.origin_y
        dtheta = raw_theta - self.origin_theta

        c, s = np.cos(-self.origin_theta), np.sin(-self.origin_theta)
        self.current_x     =  c * dx + s * dy
        self.current_y     = -s * dx + c * dy
        self.current_theta = np.arctan2(np.sin(dtheta), np.cos(dtheta))

        self.odom_received = True

    # ------------------------------------------------------------------
    # Origin helpers
    # ------------------------------------------------------------------
    def _set_origin(self, raw_x, raw_y, raw_theta):
        self.origin_x     = raw_x
        self.origin_y     = raw_y
        self.origin_theta = raw_theta
        self.origin_set   = True
        self._save_origin()
        self.get_logger().info(
            f'Origin locked (robot stable): ({raw_x:.3f}, {raw_y:.3f}, '
            f'{np.degrees(raw_theta):.1f}°) → saved as (0, 0, 0°)'
        )

    def _save_origin(self):
        try:
            with open(self.origin_file, 'w') as f:
                f.write(f'{self.origin_x},{self.origin_y},{self.origin_theta}\n')
            self.get_logger().info(f'Origin saved to {self.origin_file}')
        except Exception as e:
            self.get_logger().warn(f'Could not save origin: {e}')

    def _load_origin(self):
        if not os.path.exists(self.origin_file):
            self.get_logger().info('No saved origin — will lock on first stable odom.')
            return
        try:
            with open(self.origin_file, 'r') as f:
                parts = f.read().strip().split(',')
            self.origin_x     = float(parts[0])
            self.origin_y     = float(parts[1])
            self.origin_theta = float(parts[2])
            self.origin_set   = True
            self.get_logger().info(
                f'Origin reloaded: ({self.origin_x:.3f}, {self.origin_y:.3f}, '
                f'{np.degrees(self.origin_theta):.1f}°)'
            )
        except Exception as e:
            self.get_logger().warn(f'Could not load origin: {e}')
            self.origin_set = False

    def _reset_saved_origin(self):
        if os.path.exists(self.origin_file):
            os.remove(self.origin_file)
            self.get_logger().info(
                f'Origin deleted ({self.origin_file}). '
                'Next movement will define a new (0, 0).'
            )
        else:
            self.get_logger().warn('No saved origin file found — nothing to delete.')
        self.origin_set = False

    # ------------------------------------------------------------------
    # Reference trajectory (circular)
    # ------------------------------------------------------------------
    def get_reference_trajectory(self, t):
        omega  =  2.0 * np.pi / self.period
        ref_x  =  self.radius * np.cos(omega * t)
        ref_y  =  self.radius * np.sin(omega * t)
        ref_vx = -self.radius * omega * np.sin(omega * t)
        ref_vy =  self.radius * omega * np.cos(omega * t)
        return ref_x, ref_y, ref_vx, ref_vy

    # ------------------------------------------------------------------
    # Test mode
    # ------------------------------------------------------------------
    def run_test(self):
        """Dispatcher — routes to sim or real test implementation."""
        if self.robot == 'sim':
            self._run_test_sim(self.test)
        else:
            self._run_test_real(self.test)
        self.stop_robot()
        self.get_logger().info(f'TEST {self.test.upper()} complete.')
        self.running = False

    def _run_test_sim(self, test):
        """
        Simulator test — publishes a Pose2D target and waits for /base/goal_reached.
        Mirrors test_base_sim.py: uses /base/target_pose and /base/goal_reached.
        """
        pose = Pose2D()
        if test == 'forward':
            pose.x, pose.y, pose.theta = self.test_distance, 0.0, 0.0
            self.get_logger().info(f'SIM TEST: Moving forward {self.test_distance:.2f} m...')
        elif test == 'backward':
            pose.x, pose.y, pose.theta = -self.test_distance, 0.0, 0.0
            self.get_logger().info(f'SIM TEST: Moving backward {self.test_distance:.2f} m...')
        elif test == 'left':
            pose.x, pose.y, pose.theta = 0.0, 0.0, np.degrees(self.test_angle)
            self.get_logger().info(f'SIM TEST: Rotating left {np.degrees(self.test_angle):.1f}°...')
        elif test == 'right':
            pose.x, pose.y, pose.theta = 0.0, 0.0, -np.degrees(self.test_angle)
            self.get_logger().info(f'SIM TEST: Rotating right {np.degrees(self.test_angle):.1f}°...')
        else:
            self.get_logger().error(f'Unknown test "{test}". Use: forward, backward, left, right')
            return

        # Publish target, wait for goal_reached (mirrors test_base_sim.py)
        self.sim_goal_reached = False
        self.target_pub.publish(pose)
        timeout = time.time() + 30.0
        while not self.sim_goal_reached and time.time() < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.sim_goal_reached:
            self.get_logger().warn('TEST timed out waiting for /base/goal_reached.')

    def _run_test_real(self, test):
        """
        Real robot test — sends /cmd_vel and tracks progress via /odom.
        Mirrors test_base_real.py: time/distance based velocity commands.
        """
        # Wait for odometry (mirrors test_base_real.py)
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

        self.get_logger().info('=' * 50)
        cmd = Twist()

        if test in ('forward', 'backward'):
            start_x = self.current_x
            start_y = self.current_y
            sign    = 1.0 if test == 'forward' else -1.0
            cmd.linear.x = sign * self.max_v
            self.get_logger().info(
                f'REAL TEST: {"Forward" if sign > 0 else "Backward"} '
                f'at {self.max_v:.2f} m/s for {self.test_distance:.2f} m...'
            )
            while True:
                travelled = np.hypot(self.current_x - start_x,
                                     self.current_y - start_y)
                if travelled >= self.test_distance:
                    break
                self.cmd_vel_pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.05)
            self.get_logger().info(
                f'Distance travelled: {np.hypot(self.current_x - start_x, self.current_y - start_y):.3f} m\n'
                f'End position: ({self.current_x:.3f}, {self.current_y:.3f})'
            )

        elif test in ('left', 'right'):
            sign       = 1.0 if test == 'left' else -1.0
            cmd.angular.z = sign * self.max_omega
            rotated    = 0.0
            prev_theta = self.current_theta
            self.get_logger().info(
                f'REAL TEST: Rotating {"left" if sign > 0 else "right"} '
                f'{np.degrees(self.test_angle):.1f}° at {self.max_omega:.2f} rad/s...'
            )
            while True:
                d_theta = self.current_theta - prev_theta
                d_theta = np.arctan2(np.sin(d_theta), np.cos(d_theta))
                rotated    += abs(d_theta)
                prev_theta  = self.current_theta
                if rotated >= self.test_angle:
                    break
                self.cmd_vel_pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.05)
            self.get_logger().info(f'Rotated: {np.degrees(rotated):.1f}°')

        else:
            self.get_logger().error(f'Unknown test "{test}". Use: forward, backward, left, right')

        self.get_logger().info('=' * 50)

    # ------------------------------------------------------------------
    # Go-to-position
    # ------------------------------------------------------------------
    def go_to_position(self, goal_x: float, goal_y: float,
                       tolerance: float = None) -> bool:
        if tolerance is None:
            tolerance = self.goal_tol

        dx       = goal_x - self.current_x
        dy       = goal_y - self.current_y
        distance = np.hypot(dx, dy)

        if distance < tolerance:
            self.stop_robot()
            self.get_logger().info(
                f'Waypoint {self.waypoint_index + 1}/{len(self.waypoints)} '
                f'({goal_x:.3f}, {goal_y:.3f}) reached! Error={distance:.4f} m'
            )
            return True

        desired_heading = np.arctan2(dy, dx)
        heading_error   = desired_heading - self.current_theta
        heading_error   = np.arctan2(np.sin(heading_error), np.cos(heading_error))

        v     = np.clip(self.kp * distance,            -self.max_v,     self.max_v)
        omega = np.clip(2.0 * self.kp * heading_error, -self.max_omega, self.max_omega)

        if abs(heading_error) > np.pi / 4:
            v *= 0.3

        cmd = Twist()
        cmd.linear.x  = v
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
            if self.mode != 'test' and self.origin_x is not None:
                self.get_logger().info(
                    f'--- Control loop started ---\n'
                    f'  Origin (raw):    x={self.origin_x:.4f}  y={self.origin_y:.4f}  '
                    f'theta={np.degrees(self.origin_theta):.2f}°\n'
                    f'  Current (frame): x={self.current_x:.4f}  y={self.current_y:.4f}  '
                    f'theta={np.degrees(self.current_theta):.2f}°'
                )
            if self.mode == 'goto':
                gx, gy = self.waypoints[self.waypoint_index]
                self.get_logger().info(
                    f'  Goal: x={gx:.4f}  y={gy:.4f}  '
                    f'distance={np.hypot(gx - self.current_x, gy - self.current_y):.4f} m'
                )

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
            if self.go_to_position(goal_x, goal_y):
                self.waypoint_index += 1
                if self.waypoint_index < len(self.waypoints):
                    nx, ny = self.waypoints[self.waypoint_index]
                    self.get_logger().info(
                        f'Moving to waypoint {self.waypoint_index + 1}/'
                        f'{len(self.waypoints)}: ({nx:.3f}, {ny:.3f})'
                    )
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
        v     = vx_des * np.cos(theta) + vy_des * np.sin(theta)

        desired_heading = np.arctan2(vy_des, vx_des)
        heading_error   = desired_heading - theta
        heading_error   = np.arctan2(np.sin(heading_error), np.cos(heading_error))
        omega = 2.0 * self.kp * heading_error

        v     = np.clip(v,     -self.max_v,     self.max_v)
        omega = np.clip(omega, -self.max_omega, self.max_omega)

        cmd = Twist()
        cmd.linear.x  = v
        cmd.angular.z = omega
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
            self.get_logger().info('Robot stopped.')
        except Exception:
            pass

    def save_results(self):
        if not self.save_data or len(self.data['time']) == 0:
            return
        data_dir = os.path.expanduser('~/tidybot_trajectory_data')
        os.makedirs(data_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename  = f'trajectory_kp{self.kp}_{timestamp}.csv'
        filepath  = os.path.join(data_dir, filename)
        with open(filepath, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'ref_x', 'ref_y', 'actual_x', 'actual_y',
                             'error_x', 'error_y'])
            for i in range(len(self.data['time'])):
                writer.writerow([
                    self.data['time'][i],     self.data['ref_x'][i],
                    self.data['ref_y'][i],    self.data['actual_x'][i],
                    self.data['actual_y'][i], self.data['error_x'][i],
                    self.data['error_y'][i]
                ])
        self.get_logger().info(f'Data saved to: {filepath}')


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryTracker()
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