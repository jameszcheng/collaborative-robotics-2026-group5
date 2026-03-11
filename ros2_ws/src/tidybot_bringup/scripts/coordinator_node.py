#!/usr/bin/env python3
"""
coordinator_node.py — Orchestrates the full pick-and-place pipeline.

Sequences: NLP/manual trigger -> perception -> navigation -> re-detection -> pickup

State machine:
    IDLE -> SEARCHING -> NAVIGATING -> REDETECTING -> PICKING_UP -> DONE -> IDLE
    Any state -> FAILED -> IDLE (on timeout/error)

Usage:
    # Launch all pipeline nodes together:
    ros2 launch tidybot_bringup coordinator.launch.py

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
    REDETECTING = auto()
    PICKING_UP = auto()
    DONE = auto()
    FAILED = auto()


class CoordinatorNode(Node):
    def __init__(self):
        super().__init__('coordinator')

        # Parameters
        self.declare_parameter('detect_timeout', 30.0)
        self.declare_parameter('redetect_timeout', 30.0)
        self.declare_parameter('nav_timeout', 90.0)
        self.declare_parameter('pickup_timeout', 60.0)
        self.declare_parameter('min_confidence', 0.4)
        self.declare_parameter('redetect_samples', 3)
        self.declare_parameter('search_samples', 3)

        self.detect_timeout = float(self.get_parameter('detect_timeout').value)
        self.redetect_timeout = float(self.get_parameter('redetect_timeout').value)
        self.nav_timeout = float(self.get_parameter('nav_timeout').value)
        self.pickup_timeout = float(self.get_parameter('pickup_timeout').value)
        self.min_confidence = float(self.get_parameter('min_confidence').value)
        self.redetect_samples = int(self.get_parameter('redetect_samples').value)
        self.search_samples = int(self.get_parameter('search_samples').value)

        # State
        self.state = State.IDLE
        self.target_label = ''
        self.state_start_time = 0.0

        # Perception data
        self.object_found = False
        self.object_pose = None
        self.object_confidence = 0.0
        self._object_pose_stamp_sec = 0.0
        self._redetect_poses = []
        self._search_poses = []

        # Completion signals
        self._nav_complete = False
        self._pickup_complete = None  # None = pending, True/False = result

        # --- Subscribers ---
        self.create_subscription(String, '/nlp/response', self._nlp_cb, 10)
        self.create_subscription(String, '/coordinator/start', self._start_cb, 10)
        self.create_subscription(Bool, '/perception/object_found', self._obj_found_cb, 10)
        self.create_subscription(PoseStamped, '/perception/object_pose', self._obj_pose_cb, 10)
        self.create_subscription(Float32, '/perception/object_confidence', self._obj_conf_cb, 10)
        self.create_subscription(Bool, '/coordinator/nav_complete', self._nav_complete_cb, 10)
        self.create_subscription(Bool, '/coordinator/pickup_complete', self._pickup_complete_cb, 10)

        # Camera sweep state for REDETECTING
        self._redetect_sweep_positions = [
            (0.0,  0.5,  'center tilt down 0.5'),
            (0.0,  0.3,  'center tilt down 0.3'),
            (-0.3, 0.5,  'pan left tilt 0.5'),
            (0.3,  0.5,  'pan right tilt 0.5'),
            (-0.3, 0.3,  'pan left tilt 0.3'),
            (0.3,  0.3,  'pan right tilt 0.3'),
        ]
        self._redetect_sweep_index = 0
        self._redetect_sweep_start = 0.0
        self._redetect_sweep_settle_time = 5.0  # seconds to wait at each position
        self._redetect_sweep_active = False
        self._redetect_hold_start = 0.0
        self._redetect_min_pose_stamp_sec = 0.0

        # --- Publishers ---
        self.target_label_pub = self.create_publisher(String, '/perception/target_label', 10)
        self.status_pub = self.create_publisher(String, '/coordinator/status', 10)
        self.nav_goal_pub = self.create_publisher(PoseStamped, '/coordinator/nav_goal', 10)
        self.pickup_trigger_pub = self.create_publisher(Empty, '/coordinator/pickup_trigger', 10)
        self.refined_pose_pub = self.create_publisher(PoseStamped, '/coordinator/object_pose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pan_tilt_pub = self.create_publisher(Float64MultiArray, '/camera/pan_tilt_cmd', 10)

        # 10 Hz timer
        self.create_timer(0.1, self._tick)

        self.get_logger().info('=' * 60)
        self.get_logger().info('Coordinator node started')
        self.get_logger().info('Waiting for voice command or manual trigger...')
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

    def _obj_found_cb(self, msg: Bool):
        self.object_found = msg.data

    def _obj_pose_cb(self, msg: PoseStamped):
        pose_stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if self.state == State.REDETECTING and pose_stamp_sec < self._redetect_min_pose_stamp_sec:
            return

        self.object_pose = msg
        self._object_pose_stamp_sec = pose_stamp_sec
        if self.state == State.SEARCHING:
            self._search_poses.append(msg)
        elif self.state == State.REDETECTING:
            self._redetect_poses.append(msg)

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

    def _has_valid_detection(self) -> bool:
        return (
            self.object_pose is not None
            and self.object_confidence >= self.min_confidence
            and (
                self.state != State.REDETECTING
                or self._object_pose_stamp_sec >= self._redetect_min_pose_stamp_sec
            )
        )

    def _start_pipeline(self, label: str):
        self.target_label = label
        self.object_found = False
        self.object_pose = None
        self.object_confidence = 0.0
        self._object_pose_stamp_sec = 0.0
        self._nav_complete = False
        self._pickup_complete = None
        self._redetect_poses = []
        self._search_poses = []
        self._redetect_sweep_active = False
        self._redetect_hold_start = 0.0
        self._redetect_min_pose_stamp_sec = 0.0

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
            n = len(self._search_poses)
            if n >= self.search_samples:
                valid = [p for p in self._search_poses[-self.search_samples:]
                         if self.object_confidence >= self.min_confidence]
                if len(valid) >= self.search_samples:
                    xs = [p.pose.position.x for p in valid]
                    ys = [p.pose.position.y for p in valid]
                    avg_x = sum(xs) / len(xs)
                    avg_y = sum(ys) / len(ys)
                    for i, p in enumerate(valid):
                        self.get_logger().info(
                            f'Search sample {i+1}/{self.search_samples}: '
                            f'({p.pose.position.x:.3f}, {p.pose.position.y:.3f})'
                        )
                    self.get_logger().info(
                        f"Object '{self.target_label}' detected! "
                        f"confidence={self.object_confidence:.2f}, "
                        f"averaged pose=({avg_x:.3f}, {avg_y:.3f}) in base_link frame"
                    )
                    nav_goal = PoseStamped()
                    nav_goal.header.stamp = self.get_clock().now().to_msg()
                    nav_goal.header.frame_id = 'base_link'
                    nav_goal.pose.position.x = avg_x
                    nav_goal.pose.position.y = avg_y
                    nav_goal.pose.position.z = valid[-1].pose.position.z
                    nav_goal.pose.orientation.w = 1.0
                    self._set_state(State.NAVIGATING)
                    self._nav_complete = False
                    self.nav_goal_pub.publish(nav_goal)
                    self.get_logger().info('Sent averaged nav_goal to navigate_to_object node')
                else:
                    # Some samples lacked confidence — discard and keep collecting
                    self._search_poses = []
            elif elapsed > self.detect_timeout:
                self._fail(
                    f"No '{self.target_label}' detected after {self.detect_timeout:.0f}s. "
                    f"Is detect_object_real.py running?"
                )

        elif self.state == State.NAVIGATING:
            if self._nav_complete:
                self.get_logger().info('Navigation complete! Reached standoff position.')
                self._set_state(State.REDETECTING)
                self._redetect_poses = []
                self.object_found = False
                self.object_pose = None
                self.object_confidence = 0.0
                self._object_pose_stamp_sec = 0.0
                self._redetect_sweep_active = False
                self._redetect_sweep_index = 0
                self._redetect_sweep_start = time.time()
                self._redetect_hold_start = time.time()
                self._redetect_min_pose_stamp_sec = (
                    float(self.get_clock().now().nanoseconds) * 1e-9
                )
                self.get_logger().info(
                    f'Re-detecting object at close range... holding current camera pose for '
                    f'{self._redetect_sweep_settle_time:.0f}s before any sweep '
                    f'({self.redetect_samples} fresh samples needed)'
                )
            elif elapsed > self.nav_timeout:
                self._fail(
                    f"Navigation timed out after {self.nav_timeout:.0f}s. "
                    f"Is navigate_to_object.py running?"
                )

        elif self.state == State.REDETECTING:
            n = len(self._redetect_poses)
            valid_detection = self._has_valid_detection()

            # Keep publishing pan-tilt to hold position
            if (self._redetect_sweep_active
                    and self._redetect_sweep_index < len(self._redetect_sweep_positions)):
                pan, tilt, _ = self._redetect_sweep_positions[self._redetect_sweep_index]
                self._send_pan_tilt(pan, tilt)

            if n >= self.redetect_samples:
                # Got enough samples — average and proceed
                xs = [p.pose.position.x for p in self._redetect_poses[-self.redetect_samples:]]
                ys = [p.pose.position.y for p in self._redetect_poses[-self.redetect_samples:]]
                zs = [p.pose.position.z for p in self._redetect_poses[-self.redetect_samples:]]
                avg_x = sum(xs) / len(xs)
                avg_y = sum(ys) / len(ys)
                avg_z = sum(zs) / len(zs)
                for i in range(self.redetect_samples):
                    self.get_logger().info(
                        f'Re-detection sample {i+1}/{self.redetect_samples}: '
                        f'({xs[i]:.3f}, {ys[i]:.3f}, {zs[i]:.3f})'
                    )
                self.get_logger().info(
                    f'Re-detection complete. Averaged pose: ({avg_x:.3f}, {avg_y:.3f}, {avg_z:.3f})'
                )
                # Freeze camera at the pose that produced the successful re-detection.
                self._redetect_sweep_active = False
                # Publish refined averaged pose so pickup.py uses it instead of raw perception
                refined_pose = PoseStamped()
                refined_pose.header.stamp = self.get_clock().now().to_msg()
                refined_pose.header.frame_id = 'base_link'
                refined_pose.pose.position.x = avg_x
                refined_pose.pose.position.y = avg_y
                refined_pose.pose.position.z = avg_z
                refined_pose.pose.orientation.w = 1.0
                self.refined_pose_pub.publish(refined_pose)
                self._set_state(State.PICKING_UP)
                self._pickup_complete = None
                self.get_logger().info('Triggering pickup sequence...')
                self.pickup_trigger_pub.publish(Empty())
            else:
                if valid_detection:
                    if self._redetect_sweep_active:
                        self._redetect_sweep_active = False
                        self.get_logger().info(
                            'Object re-detected. Freezing camera at current pose for final samples.'
                        )
                else:
                    if not self._redetect_sweep_active:
                        hold_elapsed = time.time() - self._redetect_hold_start
                        if hold_elapsed < self._redetect_sweep_settle_time:
                            return
                        self._redetect_sweep_active = True
                        self._redetect_sweep_index = 0
                        self._redetect_sweep_start = time.time()
                        pan, tilt, desc = self._redetect_sweep_positions[0]
                        self._send_pan_tilt(pan, tilt)
                        self.get_logger().info(
                            'Object still not detected after navigation. Starting camera sweep...'
                        )
                        self.get_logger().info(
                            f'  Camera: {desc} (pan={pan:.2f}, tilt={tilt:.2f})'
                        )
                    else:
                        # Advance camera sweep if the object has remained undetected long enough.
                        sweep_elapsed = time.time() - self._redetect_sweep_start
                        if (sweep_elapsed >= self._redetect_sweep_settle_time
                                and self._redetect_sweep_index < len(self._redetect_sweep_positions) - 1):
                            self._redetect_sweep_index += 1
                            self._redetect_sweep_start = time.time()
                            pan, tilt, desc = self._redetect_sweep_positions[self._redetect_sweep_index]
                            self._send_pan_tilt(pan, tilt)
                            self.get_logger().info(
                                f'  Camera: {desc} (pan={pan:.2f}, tilt={tilt:.2f})'
                            )

                if elapsed > self.redetect_timeout:
                    self._redetect_sweep_active = False
                    self._send_pan_tilt(0.0, 0.0)
                    self._fail(
                        f"Re-detection timed out after {self.redetect_timeout:.0f}s. "
                        f"Object may have moved. Is detect_object_real.py running?"
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
                    f"Is pickup.py running?"
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
