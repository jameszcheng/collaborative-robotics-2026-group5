#!/usr/bin/env python3
"""
navigate_to_object.py

Bridges the audio/NLP pipeline with the movement system.
Listens for NLP commands, waits for an object pose, then navigates
the robot to a standoff position in front of the detected object.

State machine:
  IDLE -> WAITING_FOR_POSE -> NAVIGATING -> ALIGNING -> DONE -> IDLE
                           -> FAILED -> IDLE

Topics:
  Sub: /nlp/response          (std_msgs/String, JSON)
  Sub: /perception/object_pose (geometry_msgs/PoseStamped)
  Sub: /odom                  (nav_msgs/Odometry)
  Pub: /cmd_vel               (geometry_msgs/Twist)
  Pub: /navigation/status     (std_msgs/String)
  Pub: /navigation/goal_pose  (geometry_msgs/PoseStamped)

Testing:
  # Terminal 1 - Launch robot hardware (includes this node automatically)
  ros2 launch tidybot_bringup real.launch.py

  # Terminal 2 - Start object detector
  ros2 run tidybot_bringup detect_object_real.py --ros-args -p target_label:=banana

  # Terminal 3 - Option A: Use NLP interface (voice/text, publishes to /nlp/response)
  ros2 run tidybot_control nlp_interface_node
  # Then say or type: "go to the banana" / "pick up the banana"
  # The NLP node publishes a JSON command to /nlp/response, which triggers navigation.
  # It also publishes the object name to /perception/target_label for the detector.

  # Terminal 3 - Option B: Trigger navigation manually (no NLP)
  ros2 topic pub --once /nlp/response std_msgs/msg/String \
      '{data: "{\"type\": \"command\", \"action\": \"navigate\"}"}'

  # Monitor status
  ros2 topic echo /navigation/status
  ros2 topic echo /navigation/goal_pose
"""

import json
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String


# ---------------------------------------------------------------------------
# Angle helpers (copied from movement_4.py lines 104-122)
# ---------------------------------------------------------------------------

def wrap_angle(a: float) -> float:
    return float(np.arctan2(np.sin(a), np.cos(a)))


def quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
    """Standard yaw-from-quaternion (robust even with small roll/pitch)."""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return float(np.arctan2(siny_cosp, cosy_cosp))


# ---------------------------------------------------------------------------
# State constants
# ---------------------------------------------------------------------------

class State:
    IDLE             = "IDLE"
    WAITING_FOR_POSE = "WAITING_FOR_POSE"
    NAVIGATING       = "NAVIGATING"
    ALIGNING         = "ALIGNING"
    DONE             = "DONE"
    FAILED           = "FAILED"


# ---------------------------------------------------------------------------
# Main node
# ---------------------------------------------------------------------------

class NavigateToObject(Node):

    def __init__(self):
        super().__init__('navigate_to_object')

        # --- Parameters ---
        self.declare_parameter('robot',         'real')
        self.declare_parameter('standoff_dist', 0.40)
        self.declare_parameter('goal_tolerance', 0.08)
        self.declare_parameter('yaw_tolerance',  8.0)   # degrees
        self.declare_parameter('kp',             1.0)
        self.declare_parameter('max_v',          0.2)
        self.declare_parameter('max_omega',      2.0)
        self.declare_parameter('pose_timeout',   15.0)
        self.declare_parameter('nav_timeout',    60.0)

        self.robot         = self.get_parameter('robot').value
        self.standoff_dist = self.get_parameter('standoff_dist').value
        self.goal_tol      = self.get_parameter('goal_tolerance').value
        self.yaw_tol       = math.radians(self.get_parameter('yaw_tolerance').value)
        self.kp            = self.get_parameter('kp').value
        self.max_v         = self.get_parameter('max_v').value
        self.max_omega     = self.get_parameter('max_omega').value
        self.pose_timeout  = self.get_parameter('pose_timeout').value
        self.nav_timeout   = self.get_parameter('nav_timeout').value

        # yaw_offset: sim adds -pi/2 to match odom quaternion convention
        self.yaw_offset = -math.pi / 2 if self.robot == 'sim' else 0.0

        # --- State ---
        self.state = State.IDLE
        self._state_enter_time = None

        # Odom
        self.current_x     = 0.0
        self.current_y     = 0.0
        self.current_theta = 0.0
        self.odom_received = False

        # Locked goal (set once when navigation starts)
        self.goal_x   = None
        self.goal_y   = None
        self.goal_yaw = None

        # Latest object pose (updated by subscription; snapshot taken at trigger)
        self._latest_pose: PoseStamped | None = None
        self._pose_stamp_time: float | None = None

        # --- Subscriptions ---
        self.create_subscription(String,       '/nlp/response',           self._nlp_cb,   10)
        self.create_subscription(PoseStamped,  '/perception/object_pose', self._pose_cb,  10)
        self.create_subscription(Odometry,     '/odom',                   self._odom_cb,  10)

        # --- Publications ---
        self.cmd_vel_pub   = self.create_publisher(Twist,       '/cmd_vel',              10)
        self.status_pub    = self.create_publisher(String,      '/navigation/status',    10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/navigation/goal_pose', 10)

        # --- 50 Hz control loop ---
        self.create_timer(0.02, self._control_loop)

        self.get_logger().info(
            f'navigate_to_object started (robot={self.robot}, '
            f'standoff={self.standoff_dist} m, '
            f'goal_tol={self.goal_tol} m, '
            f'yaw_tol={math.degrees(self.yaw_tol):.1f} deg)'
        )

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _nlp_cb(self, msg: String):
        if self.state != State.IDLE:
            self.get_logger().info('NLP trigger received but not IDLE — ignoring.')
            return

        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warning(f'NLP response is not valid JSON: {msg.data!r}')
            return

        if data.get('type') != 'command':
            return  # Only act on command-type messages

        self.get_logger().info(f'NLP command received: {data}')
        self._transition(State.WAITING_FOR_POSE)

    def _pose_cb(self, msg: PoseStamped):
        self._latest_pose = msg
        self._pose_stamp_time = time.time()

    def _odom_cb(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        raw_yaw = quat_to_yaw(q.x, q.y, q.z, q.w)
        self.current_theta = wrap_angle(raw_yaw + self.yaw_offset)
        self.odom_received = True

    # ------------------------------------------------------------------
    # State transitions
    # ------------------------------------------------------------------

    def _transition(self, new_state: str):
        self.get_logger().info(f'State: {self.state} -> {new_state}')
        self.state = new_state
        self._state_enter_time = time.time()
        self._publish_status(new_state)

    def _publish_status(self, status: str):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

    # ------------------------------------------------------------------
    # Standoff goal computation
    # ------------------------------------------------------------------

    def _compute_standoff_goal(self, obj_x: float, obj_y: float):
        """
        Compute a goal position standoff_dist meters in front of the object,
        facing toward it.
        """
        dx = obj_x - self.current_x
        dy = obj_y - self.current_y
        dist = math.hypot(dx, dy)

        if dist <= self.standoff_dist:
            # Already inside standoff radius — stay in place, just align
            goal_x = self.current_x
            goal_y = self.current_y
        else:
            goal_x = obj_x - (dx / dist) * self.standoff_dist
            goal_y = obj_y - (dy / dist) * self.standoff_dist

        goal_yaw = math.atan2(dy, dx)  # face the object
        return goal_x, goal_y, goal_yaw

    def _publish_goal_pose(self, x: float, y: float, yaw: float):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = 0.0
        # Encode yaw as quaternion (flat)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pose_pub.publish(msg)

    # ------------------------------------------------------------------
    # Velocity commands
    # ------------------------------------------------------------------

    def _send_velocity(self, v: float, omega: float):
        cmd = Twist()
        cmd.linear.x = float(v)
        cmd.angular.z = float(omega)
        self.cmd_vel_pub.publish(cmd)

    def _stop(self):
        self._send_velocity(0.0, 0.0)

    # ------------------------------------------------------------------
    # Control loop (50 Hz)
    # ------------------------------------------------------------------

    def _control_loop(self):
        self._publish_status(self.state)

        if self.state == State.IDLE:
            return

        if not self.odom_received:
            return

        now = time.time()
        elapsed = now - self._state_enter_time

        # ------ WAITING_FOR_POSE ------
        if self.state == State.WAITING_FOR_POSE:
            if self._latest_pose is not None:
                # Snapshot pose and compute standoff goal
                obj_x = self._latest_pose.pose.position.x
                obj_y = self._latest_pose.pose.position.y
                self.get_logger().info(
                    f'Object pose: ({obj_x:.3f}, {obj_y:.3f}) in {self._latest_pose.header.frame_id}')
                self.goal_x, self.goal_y, self.goal_yaw = \
                    self._compute_standoff_goal(obj_x, obj_y)

                self.get_logger().info(
                    f'Goal set: ({self.goal_x:.3f}, {self.goal_y:.3f}), '
                    f'yaw={math.degrees(self.goal_yaw):.1f} deg'
                )
                self._publish_goal_pose(self.goal_x, self.goal_y, self.goal_yaw)
                self._transition(State.NAVIGATING)
            elif elapsed > self.pose_timeout:
                self.get_logger().warning(
                    f'No pose received within {self.pose_timeout} s — FAILED'
                )
                self._stop()
                self._transition(State.FAILED)
            return

        # ------ NAVIGATING ------
        if self.state == State.NAVIGATING:
            if elapsed > self.nav_timeout:
                self.get_logger().warning(
                    f'Navigation timed out after {self.nav_timeout} s — FAILED'
                )
                self._stop()
                self._transition(State.FAILED)
                return

            dx = self.goal_x - self.current_x
            dy = self.goal_y - self.current_y
            distance = math.hypot(dx, dy)

            if distance < self.goal_tol:
                self._stop()
                self.get_logger().info(
                    f'Position reached! error={distance:.4f} m'
                )
                self._transition(State.ALIGNING)
                return

            desired_heading = math.atan2(dy, dx)
            heading_error = wrap_angle(desired_heading - self.current_theta)

            v = float(np.clip(self.kp * distance, -self.max_v, self.max_v))
            omega = float(np.clip(2.0 * self.kp * heading_error, -self.max_omega, self.max_omega))

            # Slow down when heading error is large
            if abs(heading_error) > math.pi / 4:
                v *= 0.3

            self._send_velocity(v, omega)
            return

        # ------ ALIGNING ------
        if self.state == State.ALIGNING:
            yaw_error = wrap_angle(self.goal_yaw - self.current_theta)

            if abs(yaw_error) <= self.yaw_tol:
                self._stop()
                self.get_logger().info(
                    f'Aligned! yaw_error={math.degrees(yaw_error):.2f} deg'
                )
                self._transition(State.DONE)
                return

            omega = float(np.clip(self.kp * yaw_error, -self.max_omega, self.max_omega))
            self._send_velocity(0.0, omega)
            return

        # ------ DONE / FAILED ------
        if self.state in (State.DONE, State.FAILED):
            if elapsed > 1.0:
                self._transition(State.IDLE)
            return


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = NavigateToObject()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
