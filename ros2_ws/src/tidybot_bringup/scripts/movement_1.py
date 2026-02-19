#!/usr/bin/env python3
"""
TidyBot2 Circular Trajectory Tracking + Go-To-Position

Topics used:
- /cmd_vel (geometry_msgs/Twist) - velocity commands [v, omega]
- /odom (nav_msgs/Odometry) - robot pose feedback

Usage:
    # Terminal 1: Start simulation
    ros2 launch tidybot_bringup sim.launch.py

    # Terminal 2: Run circular trajectory tracking
    ros2 run tidybot_bringup trajectory_tracking.py

    # Terminal 2: Go to a specific position (e.g. x=1.0, y=2.0)
    ros2 run tidybot_bringup trajectory_tracking.py --ros-args \
        -p mode:=goto -p goal_x:=1.0 -p goal_y:=2.0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import csv
import os
from datetime import datetime


class TrajectoryTracker(Node):

    def __init__(self):
        super().__init__('trajectory_tracker')

        # --- Parameters ---
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('save_data', True)
        self.declare_parameter('duration', 20.0)
        self.declare_parameter('mode', 'circle')   # 'circle' or 'goto'
        self.declare_parameter('goal_x', 0.0)
        self.declare_parameter('goal_y', 0.0)
        self.declare_parameter('goal_tolerance', 0.05)  # metres

        self.kp            = self.get_parameter('kp').value
        self.save_data     = self.get_parameter('save_data').value
        self.duration      = self.get_parameter('duration').value
        self.mode          = self.get_parameter('mode').value
        self.goal_x        = self.get_parameter('goal_x').value
        self.goal_y        = self.get_parameter('goal_y').value
        self.goal_tol      = self.get_parameter('goal_tolerance').value

        # Circular trajectory parameters
        self.radius = 0.5
        self.period = 10.0

        # Robot state
        self.current_x     = 0.0
        self.current_y     = 0.0
        self.current_theta = 0.0
        self.odom_received = False

        # Data storage
        self.data = {
            'time': [], 'ref_x': [], 'ref_y': [],
            'actual_x': [], 'actual_y': [],
            'error_x': [], 'error_y': []
        }

        self.start_time = None
        self.running    = True

        # Velocity limits
        self.max_v     = 1.0   # m/s
        self.max_omega = 2.0   # rad/s

        # --- ROS interfaces ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_timer(0.02, self.control_loop)   # 50 Hz

        self.get_logger().info(
            f'TrajectoryTracker started | mode={self.mode} | Kp={self.kp}'
        )
        if self.mode == 'goto':
            self.get_logger().info(
                f'Goal position: ({self.goal_x:.3f}, {self.goal_y:.3f}), '
                f'tolerance={self.goal_tol:.3f} m'
            )

    # ------------------------------------------------------------------
    # Odometry callback
    # ------------------------------------------------------------------
    def odom_callback(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        odom_theta = 2.0 * np.arctan2(qz, qw)

        # MuJoCo coordinate offset: actual heading is π/2 behind odom theta
        self.current_theta = odom_theta - np.pi / 2.0

        self.odom_received = True

    # ------------------------------------------------------------------
    # Reference trajectory (circular)
    # ------------------------------------------------------------------
    def get_reference_trajectory(self, t):
        """Return (ref_x, ref_y, ref_vx, ref_vy) for circular path at time t."""
        omega = 2.0 * np.pi / self.period
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
        Compute velocity commands to drive toward (goal_x, goal_y).

        Uses a pure proportional controller:
          - Linear velocity  v     = Kp * distance  (clamped to max_v)
          - Angular velocity omega = 2*Kp * heading_error  (clamped to max_omega)

        Returns True if the robot is within *tolerance* of the goal
        (i.e. the goal has been reached this timestep).
        Publishes the Twist command as a side effect.
        """
        if tolerance is None:
            tolerance = self.goal_tol

        dx = goal_x - self.current_x
        dy = goal_y - self.current_y
        distance = np.hypot(dx, dy)

        if distance < tolerance:
            self.stop_robot()
            self.get_logger().info(
                f'Goal ({goal_x:.3f}, {goal_y:.3f}) reached! '
                f'Final error = {distance:.4f} m'
            )
            return True

        # Desired heading toward goal
        desired_heading = np.arctan2(dy, dx)

        # Heading error normalised to [-π, π]
        heading_error = desired_heading - self.current_theta
        heading_error = np.arctan2(np.sin(heading_error), np.cos(heading_error))

        # Proportional commands
        v     = np.clip(self.kp * distance,       -self.max_v,     self.max_v)
        omega = np.clip(2.0 * self.kp * heading_error, -self.max_omega, self.max_omega)

        # Slow linear speed while turning sharply (|heading_error| > 45°)
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
        if not self.odom_received or not self.running:
            return

        if self.start_time is None:
            self.start_time = self.get_clock().now()

        t = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9

        # ---- Mode: go to a fixed position ----
        if self.mode == 'goto':
            reached = self.go_to_position(self.goal_x, self.goal_y)
            if reached:
                self.running = False
                self.save_results()
            return

        # ---- Mode: circular trajectory ----
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

        # Proportional + feedforward world-frame velocities
        vx_des = self.kp * error_x + ref_vx
        vy_des = self.kp * error_y + ref_vy

        # Project onto robot body frame
        theta = self.current_theta
        v = vx_des * np.cos(theta) + vy_des * np.sin(theta)

        desired_heading = np.arctan2(vy_des, vx_des)
        heading_error   = desired_heading - theta
        heading_error   = np.arctan2(np.sin(heading_error), np.cos(heading_error))
        omega = 2.0 * self.kp * heading_error

        # Apply limits
        v     = np.clip(v,     -self.max_v,     self.max_v)
        omega = np.clip(omega, -self.max_omega, self.max_omega)

        cmd = Twist()
        cmd.linear.x  = v
        cmd.angular.z = omega
        self.cmd_vel_pub.publish(cmd)

        # Log data
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
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info('Robot stopped.')

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
                    self.data['time'][i],   self.data['ref_x'][i],
                    self.data['ref_y'][i],  self.data['actual_x'][i],
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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
