#!/usr/bin/env python3
"""
task2_coordinator.py — Orchestrates the full pick-and-place pipeline.

Sequences: NLP/manual trigger -> perception -> navigation -> pickup
           (pickup node handles its own camera sweep + re-detection)

State machine:
    IDLE -> SEARCHING -> NAVIGATING -> PICKING_UP -> DONE -> IDLE
    Any state -> FAILED -> IDLE (on timeout/error)

Usage:
    # Launch all pipeline nodes together:
    ros2 launch tidybot_bringup task2.launch.py

    # Manual trigger (no NLP needed):
    ros2 topic pub /coordinator/start std_msgs/String "data: banana" --once

    # Monitor status:
    ros2 topic echo /coordinator/status
"""

import json
import time
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool, Empty, Float32, Float64MultiArray, String


class State(Enum):
    IDLE = auto()
    SEARCHING = auto()
    NAVIGATING = auto()
    PICKING_UP = auto()
    DONE = auto()
    FAILED = auto()


class CoordinatorNode(Node):
    def __init__(self):
        super().__init__('coordinator')

        # Parameters
        self.declare_parameter('detect_timeout', 30.0)
        self.declare_parameter('nav_timeout', 90.0)
        self.declare_parameter('pickup_timeout', 120.0)
        self.declare_parameter('min_confidence', 0.4)
        self.declare_parameter('search_samples', 3)

        self.detect_timeout = float(self.get_parameter('detect_timeout').value)
        self.nav_timeout = float(self.get_parameter('nav_timeout').value)
        self.pickup_timeout = float(self.get_parameter('pickup_timeout').value)
        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.search_samples = int(self.get_parameter('search_samples').value)

        # State
        self.state = State.IDLE
        self.target_label = ''
        self.state_start_time = 0.0

        # Perception data
        self.object_pose = None
        self.object_confidence = 0.0
        self._search_poses = []

        # Completion signals
        self._nav_complete = False
        self._pickup_complete = None  # None = pending, True/False = result

        # --- Subscribers ---
        self.create_subscription(String, '/nlp/response', self._nlp_cb, 10)
        self.create_subscription(String, '/coordinator/start', self._start_cb, 10)
        self.create_subscription(PoseStamped, '/perception/object_pose', self._obj_pose_cb, 10)
        self.create_subscription(Float32, '/perception/object_confidence', self._obj_conf_cb, 10)
        self.create_subscription(Bool, '/coordinator/nav_complete', self._nav_complete_cb, 10)
        self.create_subscription(Bool, '/coordinator/pickup_complete', self._pickup_complete_cb, 10)

        # --- Publishers ---
        self.target_label_pub = self.create_publisher(String, '/perception/target_label', 10)
        self.status_pub = self.create_publisher(String, '/coordinator/status', 10)
        self.nav_goal_pub = self.create_publisher(PoseStamped, '/coordinator/nav_goal', 10)
        self.pickup_trigger_pub = self.create_publisher(Empty, '/coordinator/pickup_trigger', 10)
        self.pan_tilt_pub = self.create_publisher(Float64MultiArray, '/camera/pan_tilt_cmd', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # 10 Hz timer
        self.create_timer(0.1, self._tick)

        self.get_logger().info('=' * 60)
        self.get_logger().info('Coordinator node started (Task 2: find banana and place in bowl)')
        self.get_logger().info('Waiting for voice command or manual trigger...')
        self.get_logger().info('  Voice: "find the banana and place it in the bowl"')
        self.get_logger().info('  Manual: ros2 topic pub /coordinator/start std_msgs/String "data: banana" --once')
        self.get_logger().info('=' * 60)
        self._publish_status()

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _nlp_cb(self, msg: String):
        """Handle NLP response — start pipeline if it's a command with an object."""
        if self.state != State.IDLE:
            return
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        if data.get('type') != 'command':
            return
        obj = str(data.get('object', '')).strip().lower()
        if not obj or obj == 'unknown':
            self.get_logger().warn(f'NLP command has no object: {data}')
            return
        self.get_logger().info(f'Received command: "{data.get("intent", "pick up")} {obj}"')
        self._start_pipeline(obj)

    def _start_cb(self, msg: String):
        """Manual trigger — just an object name."""
        if self.state != State.IDLE:
            self.get_logger().warn(f'Ignoring start trigger — currently in {self.state.name}')
            return
        obj = msg.data.strip().lower()
        if not obj:
            self.get_logger().warn('Empty start trigger ignored')
            return
        self.get_logger().info(f'Received manual trigger: "{obj}"')
        self._start_pipeline(obj)

    def _obj_pose_cb(self, msg: PoseStamped):
        self.object_pose = msg
        if self.state == State.SEARCHING:
            if self.object_confidence >= self.min_confidence:
                self._search_poses.append(msg)

    def _obj_conf_cb(self, msg: Float32):
        self.object_confidence = msg.data

    def _nav_complete_cb(self, msg: Bool):
        self._nav_complete = True

    def _pickup_complete_cb(self, msg: Bool):
        self._pickup_complete = msg.data

    # ------------------------------------------------------------------
    # State transitions
    # ------------------------------------------------------------------
    def _set_state(self, new_state: State):
        old = self.state
        self.state = new_state
        self.state_start_time = time.time()
        self.get_logger().info(f'State: {old.name} -> {new_state.name}')
        self._publish_status()

    def _publish_status(self):
        msg = String()
        msg.data = f'{self.state.name}:{self.target_label}'
        self.status_pub.publish(msg)

    def _start_pipeline(self, label: str):
        self.target_label = label
        self.object_pose = None
        self.object_confidence = 0.0
        self._nav_complete = False
        self._pickup_complete = None
        self._search_poses = []

        self._set_state(State.SEARCHING)
        self.get_logger().info(f"Setting target label to '{label}'")
        target_msg = String()
        target_msg.data = label
        self.target_label_pub.publish(target_msg)
        self.get_logger().info(f"Searching for '{label}'... (timeout: {self.detect_timeout}s)")

    def _send_pan_tilt(self, pan: float, tilt: float):
        """Publish a pan-tilt camera command."""
        msg = Float64MultiArray()
        msg.data = [pan, tilt]
        self.pan_tilt_pub.publish(msg)

    def _fail(self, reason: str):
        self.get_logger().error(reason)
        self.get_logger().info('State: * -> FAILED -> returning to IDLE')
        # Emergency stop
        self.cmd_vel_pub.publish(Twist())
        self._set_state(State.IDLE)
        self._publish_status()

    # ------------------------------------------------------------------
    # Main tick (10 Hz)
    # ------------------------------------------------------------------
    def _tick(self):
        elapsed = time.time() - self.state_start_time

        if self.state == State.IDLE:
            return

        elif self.state == State.SEARCHING:
            if len(self._search_poses) >= self.search_samples:
                samples = self._search_poses[-self.search_samples:]
                xs = [p.pose.position.x for p in samples]
                ys = [p.pose.position.y for p in samples]
                avg_x = sum(xs) / len(xs)
                avg_y = sum(ys) / len(ys)
                for i, p in enumerate(samples):
                    self.get_logger().info(
                        f'Search sample {i+1}/{self.search_samples}: '
                        f'({p.pose.position.x:.3f}, {p.pose.position.y:.3f})'
                    )
                self.get_logger().info(
                    f"Object '{self.target_label}' detected! "
                    f"averaged pose=({avg_x:.3f}, {avg_y:.3f}) in base_link frame"
                )
                nav_goal = PoseStamped()
                nav_goal.header.stamp = self.get_clock().now().to_msg()
                nav_goal.header.frame_id = 'base_link'
                nav_goal.pose.position.x = avg_x
                nav_goal.pose.position.y = avg_y
                nav_goal.pose.position.z = samples[-1].pose.position.z
                nav_goal.pose.orientation.w = 1.0
                # Center camera to look straight ahead during navigation
                self._send_pan_tilt(0.0, 0.0)
                self._set_state(State.NAVIGATING)
                self._nav_complete = False
                self.nav_goal_pub.publish(nav_goal)
                self.get_logger().info('Sent averaged nav_goal to navigate_to_object node')
            elif elapsed > self.detect_timeout:
                self._fail(
                    f"No '{self.target_label}' detected after {self.detect_timeout:.0f}s. "
                    f"Is detect_object_real.py running?"
                )

        elif self.state == State.NAVIGATING:
            if self._nav_complete:
                self.get_logger().info('Navigation complete! Reached standoff position.')
                # Tilt camera down to look at the object at close range
                self._send_pan_tilt(0.0, 0.6)
                # Go directly to pickup — the pickup node handles its own
                # camera sweep and re-detection at close range.
                self._set_state(State.PICKING_UP)
                self._pickup_complete = None
                self.get_logger().info('Triggering pickup sequence...')
                self.pickup_trigger_pub.publish(Empty())
            elif elapsed > self.nav_timeout:
                self._fail(
                    f"Navigation timed out after {self.nav_timeout:.0f}s. "
                    f"Is navigate_to_object.py running?"
                )

        elif self.state == State.PICKING_UP:
            if self._pickup_complete is True:
                self.get_logger().info('Pickup complete! Object grasped successfully.')
                self._set_state(State.DONE)
            elif self._pickup_complete is False:
                self._fail('Pickup failed. Check arm workspace and object pose.')
            elif elapsed > self.pickup_timeout:
                self._fail(
                    f"Pickup timed out after {self.pickup_timeout:.0f}s. "
                    f"Is task2_pickup.py running?"
                )

        elif self.state == State.DONE:
            if elapsed > 2.0:
                self.get_logger().info('Task complete! Returning to IDLE.')
                self._set_state(State.IDLE)


def main(args=None):
    rclpy.init(args=args)
    node = CoordinatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Coordinator interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
