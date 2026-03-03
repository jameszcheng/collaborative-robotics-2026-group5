#!/usr/bin/env python3
"""
Pick up a block on real hardware using a simple state machine.

State machine:
  approach -> descend -> grasp -> lift -> sleep

Usage:
    # Terminal 1
    ros2 launch tidybot_bringup real.launch.py use_planner:=true

    # Terminal 2 (optional - for object detection)
    ros2 run tidybot_bringup detect_object_real.py

    # Terminal 3
    ros2 run tidybot_bringup test_pickup.py
"""

import time
from enum import Enum, auto
import traceback

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from interbotix_xs_msgs.msg import JointGroupCommand, JointSingleCommand
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32, String
from tidybot_msgs.srv import PlanToTarget


class PickupState(Enum):
    APPROACH = auto()
    DESCEND = auto()
    GRASP = auto()
    LIFT = auto()
    SLEEP = auto()
    DONE = auto()


class TestBlockReal(Node):
    """Plan and execute a real-world block pickup using perception or fixed defaults."""

    ORIENT_FINGERS_DOWN = (0.5, 0.5, 0.5, -0.5)  # (qw, qx, qy, qz)

    # Default block target in base_link frame (meters)
    BLOCK_X = -0.10
    BLOCK_Y = -0.35
    BLOCK_Z = 0.48

    # Waypoint offsets from grasp target (meters)
    APPROACH_HEIGHT = 0.12
    GRASP_HEIGHT = 0.06
    LIFT_HEIGHT = 0.18

    # Fixed arm and sleep pose for this script
    ARM_NAME = 'right'
    SLEEP_POSE = [0.0, -1.80, 1.55, 0.0, 0.8, 0.0]

    def __init__(self):
        super().__init__('test_block_real')
        self.arm_name = self.ARM_NAME
        self.state = PickupState.APPROACH

        self.plan_client = self.create_client(PlanToTarget, '/plan_to_target')

        # Joint-state health check (for startup diagnostics)
        self.joint_states_received = False
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

        # Direct interbotix command topics (same pattern as test_real_hardware.py)
        self.arm_group_pub = self.create_publisher(
            JointGroupCommand, '/right_arm/commands/joint_group', 10
        )
        self.gripper_pub = self.create_publisher(
            JointSingleCommand, '/right_arm/commands/joint_single', 10
        )

        self.get_logger().info('=' * 60)
        self.get_logger().info('TidyBot2 Real Block Pickup Test (State Machine)')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Arm: {self.arm_name}')

        self.get_logger().info('Waiting for /plan_to_target service...')
        if not self.plan_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Service not available! Start real.launch.py with use_planner:=true')
            raise RuntimeError('Planning service not available')

        self.get_logger().info('Service connected.')

    def _joint_state_cb(self, _msg: JointState):
        self.joint_states_received = True

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

    def wait_for_object_detection(self, timeout: float = 30.0, min_confidence: float = 0.5) -> bool:
        """Wait for a valid object detection with sufficient confidence."""
        self.get_logger().info(f'Waiting for object detection (min confidence: {min_confidence:.2f})...')
        start = time.time()
        while (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
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
        request.arm_name = self.arm_name
        request.target_pose = pose
        request.use_orientation = True
        request.execute = True
        request.duration = duration
        request.max_condition_number = 100.0

        pos_str = f'({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})'
        self.get_logger().info(f'Planning+executing {self.arm_name} to {pos_str}')

        result = self.call_service_sync(request)
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

    def command_gripper_pwm(self, pwm: float, duration: float = 1.0):
        """Command right gripper via JointSingleCommand PWM (test_real_hardware.py style)."""
        msg = JointSingleCommand()
        msg.name = 'right_gripper'
        msg.cmd = float(pwm)

        start = time.time()
        while time.time() - start < duration:
            self.gripper_pub.publish(msg)
            time.sleep(0.1)

    def open_gripper(self):
        self.get_logger().info('Opening gripper...')
        self.command_gripper_pwm(350.0, duration=1.5)

    def close_gripper(self):
        self.get_logger().info('Closing gripper...')
        self.command_gripper_pwm(-350.0, duration=1.5)

    def send_sleep_pose(self, duration: float = 2.0):
        self.get_logger().info('Moving arm to sleep pose...')
        msg = JointGroupCommand()
        msg.name = 'right_arm'
        msg.cmd = self.SLEEP_POSE

        start = time.time()
        while time.time() - start < duration:
            self.arm_group_pub.publish(msg)
            time.sleep(0.1)

    def run_state_machine(self) -> bool:
        # Use detected object position if available, otherwise use defaults
        if self.object_pose is not None and self.object_found:
            x = self.object_pose.pose.position.x
            y = self.object_pose.pose.position.y
            z = self.object_pose.pose.position.z
            self.get_logger().info('Using detected object position from perception')
        else:
            x = self.BLOCK_X
            y = self.BLOCK_Y
            z = self.BLOCK_Z
            self.get_logger().warn('No object detected, using hardcoded defaults')

        qw, qx, qy, qz = self.ORIENT_FINGERS_DOWN

        approach_pose = self.create_pose(x, y, z + self.APPROACH_HEIGHT, qw, qx, qy, qz)
        grasp_pose = self.create_pose(x, y, z + self.GRASP_HEIGHT, qw, qx, qy, qz)
        lift_pose = self.create_pose(x, y, z + self.LIFT_HEIGHT, qw, qx, qy, qz)

        self.get_logger().info('')
        self.get_logger().info('Pickup waypoints:')
        self.get_logger().info(f'  approach: ({x:.3f}, {y:.3f}, {z + self.APPROACH_HEIGHT:.3f})')
        self.get_logger().info(f'  descend:  ({x:.3f}, {y:.3f}, {z + self.GRASP_HEIGHT:.3f})')
        self.get_logger().info(f'  lift:     ({x:.3f}, {y:.3f}, {z + self.LIFT_HEIGHT:.3f})')
        self.get_logger().warn('Ensure workspace is clear before continuing.')
        input('Press Enter to start pickup sequence (Ctrl+C to abort)... ')

        while self.state != PickupState.DONE:
            if self.state == PickupState.APPROACH:
                self.open_gripper()
                if not self.plan_and_execute(approach_pose, duration=3.0):
                    return False
                self.transition_to(PickupState.DESCEND)

            elif self.state == PickupState.DESCEND:
                if not self.plan_and_execute(grasp_pose, duration=2.0):
                    return False
                self.transition_to(PickupState.GRASP)

            elif self.state == PickupState.GRASP:
                self.close_gripper()
                self.transition_to(PickupState.LIFT)

            elif self.state == PickupState.LIFT:
                if not self.plan_and_execute(lift_pose, duration=2.0):
                    return False
                self.transition_to(PickupState.SLEEP)

            elif self.state == PickupState.SLEEP:
                self.send_sleep_pose(duration=2.0)
                self.open_gripper()
                self.transition_to(PickupState.DONE)

            time.sleep(0.3)

        self.get_logger().info('Pickup state machine complete.')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = TestBlockReal()

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

        # Try to wait for object detection (optional)
        node.get_logger().info('')
        node.get_logger().info('Checking for object detection...')
        if node.wait_for_object_detection(timeout=10.0, min_confidence=0.4):
            node.get_logger().info('Object detection ready!')
        else:
            node.get_logger().warn('No object detected within timeout.')
            node.get_logger().warn('Will use hardcoded block position instead.')
            node.get_logger().warn('To use perception, run: ros2 run tidybot_bringup detect_object_real.py')

        ok = node.run_state_machine()
        if not ok:
            node.get_logger().error('Pickup failed.')
            return 1
        return 0
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
