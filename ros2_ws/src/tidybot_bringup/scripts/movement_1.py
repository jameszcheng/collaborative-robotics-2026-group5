#!/usr/bin/env python3
"""
TidyBot2 Circular Trajectory Tracking + Go-To-Position
v1.0.3

Topics used:
- /cmd_vel (geometry_msgs/Twist) - velocity commands [v, omega]
- /odom (nav_msgs/Odometry) - robot pose feedback

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
COMMANDS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

1) RESET ORIGIN  — run this first time, or when you want a new (0,0)
   python3 movement_1.py --ros-args -p mode:=reset_origin

2) GO TO POSITION  — move to a single (x, y) in metres
   python3 movement_1.py --ros-args -p mode:=goto -p goal_x:=1.0 -p goal_y:=0.0

3) GO TO MULTIPLE WAYPOINTS  — flat list [x1,y1, x2,y2, ...]
   python3 movement_1.py --ros-args -p mode:=goto \
       -p waypoints:="[1.0, 0.0,  1.0, 1.0,  0.0, 0.0]"

4) CIRCULAR TRAJECTORY
   python3 movement_1.py --ros-args -p mode:=circle

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
OPTIONAL PARAMETERS (append to any command above)
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
   -p kp:=1.5               # proportional gain (default 1.0)
   -p max_v:=0.2            # max linear speed in m/s (default 0.2)
   -p max_omega:=2.0        # max rotation speed in rad/s (default 2.0)
   -p goal_tolerance:=0.05  # goal tolerance in metres (default 0.05)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
TYPICAL WORKFLOW
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
   # First time only — place robot at desired origin then:
   python3 movement_1.py --ros-args -p mode:=reset_origin

   # Then move freely — all positions are relative to that origin:
   python3 movement_1.py --ros-args -p mode:=goto -p goal_x:=1.0 -p goal_y:=0.0
   python3 movement_1.py --ros-args -p mode:=goto -p goal_x:=0.0 -p goal_y:=1.0
   python3 movement_1.py --ros-args -p mode:=goto -p goal_x:=0.0 -p goal_y:=0.0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import csv
import os
from datetime import datetime


VERSION = "1.0.3"


class TrajectoryTracker(Node):

    def __init__(self):
        super().__init__('trajectory_tracker')

        # --- Parameters ---
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('save_data', True)
        self.declare_parameter('duration', 20.0)
        self.declare_parameter('mode', 'circle')        # 'circle', 'goto', or 'reset_origin'
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_tolerance', 0.05)  # metres
        self.declare_parameter('waypoints', [0.0])      # flat list [x1,y1, x2,y2, ...]
        self.declare_parameter('max_v', 0.2)
        self.declare_parameter('max_omega', 2.0)

        self.kp        = self.get_parameter('kp').value
        self.save_data = self.get_parameter('save_data').value
        self.duration  = self.get_parameter('duration').value
        self.mode      = self.get_parameter('mode').value
        self.goal_x    = self.get_parameter('goal_x').value
        self.goal_y    = self.get_parameter('goal_y').value
        self.goal_tol  = self.get_parameter('goal_tolerance').value
        self.max_v     = self.get_parameter('max_v').value
        self.max_omega = self.get_parameter('max_omega').value

        # Build waypoint list
        # If waypoints param was provided as a flat list [x1,y1,x2,y2,...], use it.
        # Otherwise fall back to the single goal_x / goal_y.
        raw_wp = self.get_parameter('waypoints').value
        if len(raw_wp) >= 2 and not (len(raw_wp) == 1 and raw_wp[0] == 0.0):
            # Parse flat list into [(x1,y1), (x2,y2), ...]
            self.waypoints = [
                (raw_wp[i], raw_wp[i + 1]) for i in range(0, len(raw_wp) - 1, 2)
            ]
        else:
            # Single goal from goal_x / goal_y
            self.waypoints = [(self.goal_x, self.goal_y)]

        self.waypoint_index = 0  # which waypoint we are currently heading to

        # Circular trajectory parameters
        self.radius = 0.5
        self.period = 10.0

        # Robot state
        self.current_x     = 0.0
        self.current_y     = 0.0
        self.current_theta = 0.0
        self.odom_received = False

        # Origin offset
        self.origin_x     = None
        self.origin_y     = None
        self.origin_theta = None
        self.origin_set   = False
        self.origin_file  = os.path.expanduser('~/.tidybot_origin.txt')

        # Stabilisation: collect odom readings and only lock origin once
        # the robot has been still for 0.5s (25 readings at 50Hz)
        self._stable_readings   = []
        self._stable_required   = 25   # number of consistent readings needed
        self._stable_threshold  = 0.005  # max position change between readings (m)

        # Try to reload a previously saved origin
        self._load_origin()

        # Data storage
        self.data = {
            'time': [], 'ref_x': [], 'ref_y': [],
            'actual_x': [], 'actual_y': [],
            'error_x': [], 'error_y': []
        }

        self.start_time = None
        self.running    = True

        # --- ROS interfaces ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # In reset_origin mode we don't need odom or the control loop at all
        if self.mode != 'reset_origin':
            self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
            self.create_timer(0.02, self.control_loop)   # 50 Hz

        self.get_logger().info(f'=== movement_1.py  v{VERSION} ===')
        self.get_logger().info(f'Running from: {os.path.abspath(__file__)}')
        self.get_logger().info(
            f'TrajectoryTracker started | mode={self.mode} | Kp={self.kp}'
        )

        # Handle reset_origin mode immediately at startup, before doing anything else
        if self.mode == 'reset_origin':
            self._reset_saved_origin()
            return

        if self.mode == 'goto':
            self.get_logger().info(
                f'Waypoints: {self.waypoints} | tolerance={self.goal_tol:.3f} m'
            )

    # ------------------------------------------------------------------
    # Odometry callback
    # ------------------------------------------------------------------
    def odom_callback(self, msg: Odometry):
        raw_x = msg.pose.pose.position.x
        raw_y = msg.pose.pose.position.y

        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        odom_theta = 2.0 * np.arctan2(qz, qw)
        raw_theta  = odom_theta - np.pi / 2.0  # MuJoCo offset

        # If origin not yet set, wait for robot to be stable before locking it
        if not self.origin_set:
            self._stable_readings.append((raw_x, raw_y, raw_theta))

            if len(self._stable_readings) >= self._stable_required:
                # Check if position has been stable across all collected readings
                xs = [r[0] for r in self._stable_readings]
                ys = [r[1] for r in self._stable_readings]
                if max(xs) - min(xs) < self._stable_threshold and \
                   max(ys) - min(ys) < self._stable_threshold:
                    # Robot is stationary — lock origin from average reading
                    avg_x     = sum(xs) / len(xs)
                    avg_y     = sum(ys) / len(ys)
                    avg_theta = self._stable_readings[-1][2]
                    self._set_origin(avg_x, avg_y, avg_theta)
                else:
                    # Still moving — discard oldest reading and keep waiting
                    self._stable_readings.pop(0)
            else:
                return  # Not enough readings yet

        if not self.origin_set:
            return

        # Express current pose relative to the fixed origin frame
        dx     = raw_x - self.origin_x
        dy     = raw_y - self.origin_y
        dtheta = raw_theta - self.origin_theta

        c, s = np.cos(-self.origin_theta), np.sin(-self.origin_theta)
        self.current_x     =  c * dx + s * dy
        self.current_y     = -s * dx + c * dy
        self.current_theta = np.arctan2(np.sin(dtheta), np.cos(dtheta))

        self.odom_received = True

    def _set_origin(self, raw_x, raw_y, raw_theta):
        """
        Record the robot's pose as the fixed world origin (0, 0, 0°).
        Only called once the robot has been confirmed stationary.
        Saves to disk so the same origin is reused on every future run.
        """
        self.origin_x     = raw_x
        self.origin_y     = raw_y
        self.origin_theta = raw_theta
        self.origin_set   = True
        self._save_origin()
        self.get_logger().info(
            f'Origin locked (robot stable): raw pose ({raw_x:.3f}, {raw_y:.3f}, '
            f'{np.degrees(raw_theta):.1f}°) → saved as (0, 0, 0°)'
        )

    def _save_origin(self):
        """Write origin to ~/.tidybot_origin.txt so it persists across runs."""
        try:
            with open(self.origin_file, 'w') as f:
                f.write(f'{self.origin_x},{self.origin_y},{self.origin_theta}\n')
            self.get_logger().info(f'Origin saved to {self.origin_file}')
        except Exception as e:
            self.get_logger().warn(f'Could not save origin: {e}')

    def _load_origin(self):
        """
        Load origin from disk if it exists.
        If the file is found the robot will reuse the same (0,0) reference
        from its very first run — no matter how many times the script is restarted.
        """
        if not os.path.exists(self.origin_file):
            self.get_logger().info(
                'No saved origin found — will set it on first odom message.'
            )
            return

        try:
            with open(self.origin_file, 'r') as f:
                parts = f.read().strip().split(',')
            self.origin_x     = float(parts[0])
            self.origin_y     = float(parts[1])
            self.origin_theta = float(parts[2])
            self.origin_set   = True
            self.get_logger().info(
                f'Origin reloaded from file: ({self.origin_x:.3f}, '
                f'{self.origin_y:.3f}, {np.degrees(self.origin_theta):.1f}°) — '
                f'using same reference as first run.'
            )
        except Exception as e:
            self.get_logger().warn(f'Could not load origin: {e} — will reset on first odom.')
            self.origin_set = False

    def _reset_saved_origin(self):
        """Delete the saved origin file so the next run defines a fresh (0, 0)."""
        if os.path.exists(self.origin_file):
            os.remove(self.origin_file)
            self.get_logger().info(
                f'Origin file deleted ({self.origin_file}). '
                'The next movement will define a new (0, 0).'
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
    # Go-to-position
    # ------------------------------------------------------------------
    def go_to_position(self, goal_x: float, goal_y: float,
                       tolerance: float = None) -> bool:
        """
        Drive toward (goal_x, goal_y) in the fixed origin frame.
        Returns True when the goal is reached.
        """
        if tolerance is None:
            tolerance = self.goal_tol

        dx       = goal_x - self.current_x
        dy       = goal_y - self.current_y
        distance = np.hypot(dx, dy)

        if distance < tolerance:
            self.stop_robot()
            self.get_logger().info(
                f'Waypoint {self.waypoint_index + 1}/{len(self.waypoints)} '
                f'({goal_x:.3f}, {goal_y:.3f}) reached! '
                f'Error = {distance:.4f} m'
            )
            return True

        desired_heading = np.arctan2(dy, dx)
        heading_error   = desired_heading - self.current_theta
        heading_error   = np.arctan2(np.sin(heading_error), np.cos(heading_error))

        v     = np.clip(self.kp * distance,            -self.max_v,     self.max_v)
        omega = np.clip(2.0 * self.kp * heading_error, -self.max_omega, self.max_omega)

        # Reduce forward speed while turning sharply (> 45°)
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
            # Print full state on first control loop iteration so we can verify
            self.get_logger().info(
                f'--- Control loop started ---\n'
                f'  Origin (raw):   x={self.origin_x:.4f}  y={self.origin_y:.4f}  '
                f'theta={np.degrees(self.origin_theta):.2f}°\n'
                f'  Current (frame): x={self.current_x:.4f}  y={self.current_y:.4f}  '
                f'theta={np.degrees(self.current_theta):.2f}°'
            )
            if self.mode == 'goto':
                gx, gy = self.waypoints[self.waypoint_index]
                self.get_logger().info(
                    f'  Goal:            x={gx:.4f}  y={gy:.4f}  '
                    f'distance={np.hypot(gx - self.current_x, gy - self.current_y):.4f} m'
                )

        t = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        # ---- Mode: waypoint sequence --------------------------------
        if self.mode == 'goto':
            # All waypoints done?
            if self.waypoint_index >= len(self.waypoints):
                if self.running:
                    self.running = False
                    self.get_logger().info('All waypoints completed.')
                    self.save_results()
                return

            goal_x, goal_y = self.waypoints[self.waypoint_index]
            reached = self.go_to_position(goal_x, goal_y)

            if reached:
                # Advance to next waypoint — origin frame stays the same
                self.waypoint_index += 1
                if self.waypoint_index < len(self.waypoints):
                    nx, ny = self.waypoints[self.waypoint_index]
                    self.get_logger().info(
                        f'Moving to waypoint {self.waypoint_index + 1}/'
                        f'{len(self.waypoints)}: ({nx:.3f}, {ny:.3f})'
                    )
            return

        # ---- Mode: circular trajectory ------------------------------
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
            pass  # Node already shutting down, safe to ignore

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