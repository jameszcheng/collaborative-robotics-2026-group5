#!/usr/bin/env python3
"""
task2_pickup.py — Pick up an object on real hardware using a simple state machine.

State machine:
    approach -> descend -> grasp -> lift -> drop -> done

Handles its own camera sweep and re-detection at close range before pickup.

Standalone usage:
    ros2 launch tidybot_bringup real.launch.py use_planner:=true
    ros2 run tidybot_bringup detect_object_real.py --ros-args -p target_label:=banana
    ros2 run tidybot_bringup task2_pickup.py

Coordinator usage (auto_start mode — waits for trigger):
    ros2 launch tidybot_bringup coordinator.launch.py
"""

import time
from enum import Enum, auto
import traceback

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from interbotix_xs_msgs.msg import JointGroupCommand
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float32, Float64MultiArray, String
from tidybot_msgs.srv import PlanToTarget


class PickupState(Enum):
    APPROACH = auto()
    DESCEND = auto()
    GRASP = auto()
    LIFT = auto()
    DROP = auto()
    DONE = auto()


class PickupNode(Node):
    """Plan and execute a real-world block pickup using perception or fixed defaults."""

    ORIENT_FINGERS_DOWN = (0.707107, 0.0, 0.707107, 0.0)  # (qw, qx, qy, qz) — 90° rotated for horizontal banana
    SLEEP_POSE = [0.0, -1.80, 1.55, 0.0, 0.8, 0.0]  # [waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate]

    # Waypoint offsets from grasp target (meters)
    APPROACH_HEIGHT = 0.1
    GRASP_HEIGHT = 0.02
    LIFT_HEIGHT = 0.25
    Y_OFFSET = -0.05  # forward offset, negative = further from robot (meters)

    # Fixed arm pose for this script
    ARM_NAME = 'right'

    def __init__(self):
        super().__init__('pickup')
        self.arm_name = self.ARM_NAME
        self.state = PickupState.APPROACH

        # Coordinator parameters
        self.declare_parameter('auto_start', False)
        self.auto_start = bool(self.get_parameter('auto_start').value)
        self._pickup_triggered = False

        # Coordinator publishers/subscribers
        self.pickup_complete_pub = self.create_publisher(Bool, '/coordinator/pickup_complete', 10)
        self.create_subscription(Empty, '/coordinator/pickup_trigger', self._pickup_trigger_cb, 10)

        self.plan_client = self.create_client(PlanToTarget, '/plan_to_target')

        # Joint-state health check (for startup diagnostics)
        self.joint_states_received = False
        self._latest_positions = {}
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

        # Perception subscribers (from detect_object_real.py)
        self.object_found = False
        self.object_pose = None
        self.object_confidence = 0.0
        self.object_label = ""

        self.create_subscription(Bool, '/perception/object_found', self._object_found_cb, 10)
        self.create_subscription(PoseStamped, '/perception/object_pose', self._object_pose_cb, 10)
        self.create_subscription(Float32, '/perception/object_confidence', self._object_confidence_cb, 10)
        self.create_subscription(String, '/perception/object_label', self._object_label_cb, 10)

        # Pan-tilt camera control
        self.pan_tilt_pub = self.create_publisher(
            Float64MultiArray, '/camera/pan_tilt_cmd', 10
        )

        # Direct interbotix command topics
        self.arm_group_pub = self.create_publisher(
            JointGroupCommand, f'/{self.arm_name}_arm/commands/joint_group', 10
        )
        # Gripper via wrapper node (same as test_arms_real.py)
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, f'/{self.arm_name}_gripper/cmd', 10
        )

        self.get_logger().info('=' * 60)
        self.get_logger().info('TidyBot2 Pickup Node (State Machine)')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Arm: {self.arm_name}')

        self.get_logger().info('Waiting for /plan_to_target service...')
        if not self.plan_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Service not available! Start real.launch.py with use_planner:=true')
            raise RuntimeError('Planning service not available')

        self.get_logger().info('Service connected.')

    def _joint_state_cb(self, msg: JointState):
        self.joint_states_received = True
        for name, pos in zip(msg.name, msg.position):
            self._latest_positions[name] = pos

    def _object_found_cb(self, msg: Bool):
        """Callback for /perception/object_found."""
        self.object_found = msg.data

    def _object_pose_cb(self, msg: PoseStamped):
        """Callback for /perception/object_pose."""
        self.object_pose = msg

    def _object_confidence_cb(self, msg: Float32):
        """Callback for /perception/object_confidence."""
        self.object_confidence = msg.data

    def _object_label_cb(self, msg: String):
        """Callback for /perception/object_label."""
        self.object_label = msg.data

    def _pickup_trigger_cb(self, msg: Empty):
        """Coordinator triggers pickup — clear stale perception and start fresh."""
        self.get_logger().info('Pickup triggered by coordinator — clearing stale perception data')
        self.object_found = False
        self.object_pose = None
        self.object_confidence = 0.0
        self.object_label = ""
        self._pickup_triggered = True

    def _drain_callbacks(self, count: int = 30):
        """Process multiple pending callbacks to avoid missing perception messages."""
        for _ in range(count):
            rclpy.spin_once(self, timeout_sec=0.005)

    def _check_detection(self, min_confidence: float = 0.4) -> bool:
        """Drain callbacks and return True if a valid detection is available."""
        self._drain_callbacks()
        return (self.object_found
                and self.object_pose is not None
                and self.object_confidence >= min_confidence)

    def wait_for_object_detection(self, timeout: float = 30.0, min_confidence: float = 0.5,
                                   settle_time: float = 2.0) -> bool:
        """Wait for a valid object detection with sufficient confidence.

        Args:
            settle_time: Seconds to wait (discarding detections) so the camera
                         tilt physically settles and TF propagates before we
                         accept a pose.
        """
        self.get_logger().info(f'Waiting for object detection (min confidence: {min_confidence:.2f})...')
        if settle_time > 0.0:
            self.get_logger().info(f'Letting camera settle for {settle_time:.1f}s before accepting detections...')
            settle_end = time.time() + settle_time
            while time.time() < settle_end:
                # Drain and discard detections while camera/TF settles
                self._drain_callbacks()
                self.object_found = False
                self.object_pose = None
                self.object_confidence = 0.0
                time.sleep(0.05)
            self.get_logger().info('Camera settled — now accepting detections.')
        start = time.time()
        while (time.time() - start) < timeout:
            self._drain_callbacks()
            if self.object_found and self.object_pose is not None and self.object_confidence >= min_confidence:
                self.get_logger().info(
                    f'Object detected: "{self.object_label}" with confidence {self.object_confidence:.2f}'
                )
                self.get_logger().info(
                    f'Object position: ({self.object_pose.pose.position.x:.3f}, '
                    f'{self.object_pose.pose.position.y:.3f}, '
                    f'{self.object_pose.pose.position.z:.3f})'
                )
                return True
            time.sleep(0.05)
        return False

    # Camera sweep disabled — coordinator PAUSE state now handles camera reset
    # def search_for_object(self, timeout_per_position: float = 5.0, min_confidence: float = 0.4) -> bool:
    #     """
    #     Sweep the camera through several pan/tilt positions looking for an object.
    #
    #     Checks for detections continuously — during pan-tilt movement and while
    #     waiting at each position. Returns True as soon as an object is detected.
    #     """
    #     search_positions = [
    #         (0.0,  0.5,  'center tilt down 0.5'),
    #         (-0.5, 0.5,  'pan left tilt 0.5'),
    #         (0.5,  0.5,  'pan right tilt 0.5'),
    #         (0.0,  0.7,  'center tilt down 0.7'),
    #         (-0.5, 0.7,  'pan left tilt 0.7'),
    #         (0.5,  0.7,  'pan right tilt 0.7'),
    #     ]
    #
    #     self.get_logger().info('No object found — starting camera search sweep...')
    #
    #     for pan, tilt, description in search_positions:
    #         self.get_logger().info(f'  Search position: {description} (pan={pan:.2f}, tilt={tilt:.2f})')
    #
    #         # Move camera while checking for detections
    #         msg = Float64MultiArray()
    #         msg.data = [pan, tilt]
    #         for _ in range(20):  # ~1 second of movement
    #             self.pan_tilt_pub.publish(msg)
    #             if self._check_detection(min_confidence):
    #                 self.get_logger().info(
    #                     f'  Object found during move to {description}: "{self.object_label}" '
    #                     f'(confidence {self.object_confidence:.2f})')
    #                 return True
    #             time.sleep(0.05)
    #
    #         # Wait at this position, checking continuously
    #         start = time.time()
    #         while (time.time() - start) < timeout_per_position:
    #             self.pan_tilt_pub.publish(msg)
    #             if self._check_detection(min_confidence):
    #                 self.get_logger().info(
    #                     f'  Object found at {description}: "{self.object_label}" '
    #                     f'(confidence {self.object_confidence:.2f})')
    #                 return True
    #             time.sleep(0.05)
    #
    #     # Return camera to center
    #     self.send_pan_tilt(0.0, 0.0, duration=1.0)
    #     self.get_logger().warn('Camera search sweep complete — no object detected.')
    #     return False

    def send_pan_tilt(self, pan: float, tilt: float, duration: float = 1.0):
        """Send pan-tilt command, publishing repeatedly for the given duration."""
        msg = Float64MultiArray()
        msg.data = [pan, tilt]
        for _ in range(int(duration * 20)):
            self.pan_tilt_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_joint_states(self, timeout: float = 10.0) -> bool:
        start = time.time()
        while (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.joint_states_received:
                return True
        return False

    def transition_to(self, state: PickupState):
        self.get_logger().info(f'State: {self.state.name.lower()} -> {state.name.lower()}')
        self.state = state

    def create_pose(
        self,
        x: float,
        y: float,
        z: float,
        qw: float,
        qx: float,
        qy: float,
        qz: float,
    ) -> Pose:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = qw
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        return pose

    def call_service_sync(self, request: PlanToTarget.Request, timeout_sec: float = 30.0):
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
        request.arm_name = self.arm_name
        request.target_pose = pose
        request.use_orientation = True
        request.execute = True
        request.duration = duration

        pos_str = f'({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})'
        self.get_logger().info(f'Planning+executing {self.arm_name} to {pos_str}')

        result = self.call_service_sync(request, timeout_sec=duration + 20.0)
        if result is None:
            return False

        if result.success:
            self.get_logger().info(f'  SUCCESS: {result.message}')
            self.get_logger().info(
                f'  Errors: pos={result.position_error:.4f}m, '
                f'ori={result.orientation_error:.4f}rad ({np.degrees(result.orientation_error):.1f}°)'
            )
            return True

        self.get_logger().warn(f'  FAILED: {result.message}')
        return False

    def set_gripper(self, position: float, duration: float = 2.0, hold: bool = False):
        """Set gripper via wrapper node (same as test_arms_real.py).

        Args:
            position: 0.0 (open) to 1.0 (closed)
            duration: Time to hold the command (seconds)
            hold: If True, skip stop command so gripper maintains force
        """
        msg = Float64MultiArray()
        msg.data = [float(position)]

        start = time.time()
        while (time.time() - start) < duration:
            self.gripper_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)

        if not hold:
            # Send stop command (0.5 maps to PWM=0 in wrapper)
            stop_msg = Float64MultiArray()
            stop_msg.data = [0.5]
            self.gripper_pub.publish(stop_msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def open_gripper(self):
        self.get_logger().info('Opening gripper...')
        self.set_gripper(0.0, duration=2.0)

    def close_gripper(self):
        self.get_logger().info('Closing gripper...')
        self.set_gripper(1.0, duration=2.0, hold=True)

    def go_to_sleep(self, max_joint_speed: float = 0.5):
        """Send arm to sleep pose using smooth cosine interpolation."""
        self.get_logger().info(f'Moving {self.arm_name} arm to sleep pose...')
        rclpy.spin_once(self, timeout_sec=0.1)

        # Read current positions from joint states
        joint_names = [
            f'{self.arm_name}_waist', f'{self.arm_name}_shoulder', f'{self.arm_name}_elbow',
            f'{self.arm_name}_forearm_roll', f'{self.arm_name}_wrist_angle', f'{self.arm_name}_wrist_rotate',
        ]
        current = np.array([self._latest_positions.get(j, 0.0) for j in joint_names])
        target = np.array(self.SLEEP_POSE)

        max_diff = np.max(np.abs(target - current))
        duration = max(max_diff / max_joint_speed, 1.0)

        rate_hz = 50.0
        dt = 1.0 / rate_hz
        num_steps = max(int(duration * rate_hz), 1)

        for i in range(num_steps + 1):
            t = i / num_steps
            alpha = 0.5 * (1 - np.cos(np.pi * t))
            q = current + alpha * (target - current)

            cmd = JointGroupCommand()
            cmd.name = f'{self.arm_name}_arm'
            cmd.cmd = q.tolist()
            self.arm_group_pub.publish(cmd)

            if i < num_steps:
                time.sleep(dt)

        self.get_logger().info(f'{self.arm_name} arm is now in sleep pose.')

    def run_state_machine(self) -> bool:
        x = self.object_pose.pose.position.x
        y = self.object_pose.pose.position.y + self.Y_OFFSET
        z = self.object_pose.pose.position.z
        self.get_logger().info(f'Using detected object position from perception (y_offset={self.Y_OFFSET})')

        qw, qx, qy, qz = self.ORIENT_FINGERS_DOWN

        approach_pose = self.create_pose(x, y, z + self.APPROACH_HEIGHT, qw, qx, qy, qz)
        grasp_pose = self.create_pose(x, y, z + self.GRASP_HEIGHT, qw, qx, qy, qz)
        lift_pose = self.create_pose(x, y, z + self.LIFT_HEIGHT, qw, qx, qy, qz)
        # Move right 25cm (negative x in base_link) then drop
        drop_pose = self.create_pose(x - 0.25, y, z + self.LIFT_HEIGHT, qw, qx, qy, qz)

        self.get_logger().info('')
        self.get_logger().info('Pickup waypoints:')
        self.get_logger().info(f'  approach: ({x:.3f}, {y:.3f}, {z + self.APPROACH_HEIGHT:.3f})')
        self.get_logger().info(f'  descend:  ({x:.3f}, {y:.3f}, {z + self.GRASP_HEIGHT:.3f})')
        self.get_logger().info(f'  lift:     ({x:.3f}, {y:.3f}, {z + self.LIFT_HEIGHT:.3f})')
        self.get_logger().info(f'  drop:     ({x - 0.25:.3f}, {y:.3f}, {z + self.LIFT_HEIGHT:.3f})')
        if self.auto_start or self._pickup_triggered:
            self.get_logger().info('Auto-starting pickup sequence (coordinator mode)')
        else:
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
                self.transition_to(PickupState.DROP)

            elif self.state == PickupState.DROP:
                if not self.plan_and_execute(drop_pose, duration=10.0):
                    return False
                self.open_gripper()
                self.transition_to(PickupState.DONE)

            time.sleep(0.3)

        self.get_logger().info('Pickup state machine complete.')
        self.pickup_complete_pub.publish(Bool(data=True))
        return True


def _run_pickup_once(node):
    """Run the pickup pipeline once: search for object, then execute state machine."""
    node.get_logger().info('Waiting for object detection...')
    if not node.wait_for_object_detection(timeout=10.0, min_confidence=0.4):
        # Camera sweep disabled — coordinator PAUSE state now handles camera reset
        # node.get_logger().warn('No object found in initial window — starting camera search sweep.')
        # if not node.search_for_object(timeout_per_position=5.0, min_confidence=0.4):
        node.get_logger().error('No object detected. Aborting.')
        node.get_logger().error('Make sure detect_object_real.py is running and camera can see the object.')
        node.pickup_complete_pub.publish(Bool(data=False))
        return 1
    node.get_logger().info('Object detection ready!')

    ok = node.run_state_machine()
    if not ok:
        node.get_logger().error('Pickup failed.')
        node.pickup_complete_pub.publish(Bool(data=False))
        return 1
    return 0


def main(args=None):
    rclpy.init(args=args)
    node = PickupNode()

    try:
        node.get_logger().info('Waiting for joint states...')
        if not node.wait_for_joint_states(timeout=10.0):
            node.get_logger().error('No joint states received!')
            node.get_logger().error('Make sure to launch: ros2 launch tidybot_bringup real.launch.py use_planner:=true')
            return 1

        if not node.plan_client.wait_for_service(timeout_sec=2.0):
            node.get_logger().error('IK service not available!')
            node.get_logger().error('Make sure to launch: ros2 launch tidybot_bringup real.launch.py use_planner:=true')
            return 1

        if node.auto_start:
            # Coordinator mode: stay alive and wait for pickup triggers
            node.get_logger().info('Auto-start mode: waiting for /coordinator/pickup_trigger ...')
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
                if node._pickup_triggered:
                    node._pickup_triggered = False
                    node.state = PickupState.APPROACH  # reset state machine
                    _run_pickup_once(node)
            return 0
        else:
            # Standalone mode: run once and exit
            node.get_logger().info('')
            return _run_pickup_once(node)
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
