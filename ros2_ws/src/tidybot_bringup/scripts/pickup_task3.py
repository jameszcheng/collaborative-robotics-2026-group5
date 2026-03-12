#!/usr/bin/env python3
"""
pickup_task3.py — Bimanual pillow pickup for Task 3.

State machine:
    R_APPROACH -> R_DESCEND -> R_GRASP -> L_APPROACH -> L_DESCEND -> L_GRASP -> LIFT -> DONE

The detected pillow centre pose (x, y, z in base_link) is split into two grasp points:
  right grasp: (x,  y - PILLOW_HALF_WIDTH,  z + GRASP_Z_ABOVE)   <- right arm
  left  grasp: (x,  y + PILLOW_HALF_WIDTH,  z + GRASP_Z_ABOVE)   <- left  arm

Right arm grasps first, then left arm, then both arms lift together.

Standalone usage:
    ros2 launch tidybot_bringup real.launch.py use_planner:=true
    ros2 run tidybot_bringup detect_object_real_task3.py
    ros2 run tidybot_bringup pickup_task3.py

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
    R_APPROACH = auto()   # right arm approaches above right grasp point
    R_DESCEND  = auto()   # right arm descends to right grasp point
    R_GRASP    = auto()   # right gripper closes
    L_APPROACH = auto()   # left arm approaches above left grasp point
    L_DESCEND  = auto()   # left arm descends to left grasp point
    L_GRASP    = auto()   # left gripper closes
    LIFT       = auto()   # both arms lift together (right first, then left)
    DONE       = auto()


class PickupNode(Node):
    """Bimanual pillow pickup — right arm grabs right end, left arm grabs left end, both lift."""

    ORIENT_FINGERS_DOWN = (0.707, 0.0, 0.707, 0.0)  # (qw, qx, qy, qz) — fingers down, rotated 90°

    # Sleep poses per arm
    SLEEP_POSE = [0.0, -1.80, 1.55, 0.0, 0.8, 0.0]

    # Grasp geometry (metres, relative to detected pillow centre in base_link)
    PILLOW_HALF_WIDTH = 0.07   # ±7 cm in y (left/right) from pillow centre
    GRASP_Z_ABOVE     = 0.10   # grasp z = detected_z + 10 cm (grip top surface of pillow)
    APPROACH_HEIGHT   = 0.17   # approach z = grasp_z + 17 cm (clear approach)
    LIFT_HEIGHT       = 0.27   # lift z = detected_z + 20 cm after grasping

    # Hardcoded grasp positions in base_link frame.
    # Used instead of the detected pose — more reliable once the robot has
    # navigated to its known standoff position in front of the pillow.
    #   x = ±7 cm lateral spread (right arm +x, left arm -x)
    #   y = 0.40 m forward from robot base
    #   z = 0.15 m — final grasp height (top surface of pillow)
    HARDCODED_GRASP_X_RIGHT =  -0.1
    HARDCODED_GRASP_X_LEFT  = 0.1
    HARDCODED_GRASP_Y       =  -0.40
    HARDCODED_GRASP_Z       =  0.18

    def __init__(self):
        super().__init__('pickup_task3')
        self.state = PickupState.R_APPROACH

        # Coordinator parameters
        self.declare_parameter('auto_start', False)
        self.auto_start = bool(self.get_parameter('auto_start').value)
        self._pickup_triggered = False

        # Coordinator publishers/subscribers
        self.pickup_complete_pub = self.create_publisher(Bool, '/coordinator/pickup_complete', 10)
        self.create_subscription(Empty, '/coordinator/pickup_trigger', self._pickup_trigger_cb, 10)
        self.create_subscription(PoseStamped, '/coordinator/object_pose', self._coordinator_pose_cb, 10)

        # IK planning service
        self.plan_client = self.create_client(PlanToTarget, '/plan_to_target')

        # Joint-state health check
        self.joint_states_received = False
        self._latest_positions = {}
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

        # Perception subscribers
        self.object_found = False
        self.object_pose = None
        self.object_confidence = 0.0
        self.object_label = ""

        self.create_subscription(Bool,        '/perception/object_found',      self._object_found_cb,      10)
        self.create_subscription(PoseStamped, '/perception/object_pose',       self._object_pose_cb,       10)
        self.create_subscription(Float32,     '/perception/object_confidence',  self._object_confidence_cb, 10)
        self.create_subscription(String,      '/perception/object_label',       self._object_label_cb,      10)

        # Right arm publishers
        self.right_arm_pub = self.create_publisher(
            JointGroupCommand, '/right_arm/commands/joint_group', 10)
        self.right_gripper_pub = self.create_publisher(
            Float64MultiArray, '/right_gripper/cmd', 10)

        # Left arm publishers
        self.left_arm_pub = self.create_publisher(
            JointGroupCommand, '/left_arm/commands/joint_group', 10)
        self.left_gripper_pub = self.create_publisher(
            Float64MultiArray, '/left_gripper/cmd', 10)

        self.get_logger().info('=' * 60)
        self.get_logger().info('TidyBot2 Bimanual Pickup Node (Task 3)')
        self.get_logger().info('  right arm → right end of pillow  (y - 7 cm)')
        self.get_logger().info('  left  arm → left  end of pillow  (y + 7 cm)')
        self.get_logger().info('=' * 60)

        self.get_logger().info('Waiting for /plan_to_target service...')
        if not self.plan_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Service not available! Start real.launch.py with use_planner:=true')
            raise RuntimeError('Planning service not available')
        self.get_logger().info('Service connected.')

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _joint_state_cb(self, msg: JointState):
        self.joint_states_received = True
        for name, pos in zip(msg.name, msg.position):
            self._latest_positions[name] = pos

    def _object_found_cb(self, msg: Bool):
        self.object_found = msg.data

    def _object_pose_cb(self, msg: PoseStamped):
        self.object_pose = msg

    def _object_confidence_cb(self, msg: Float32):
        self.object_confidence = msg.data

    def _object_label_cb(self, msg: String):
        self.object_label = msg.data

    def _pickup_trigger_cb(self, msg: Empty):
        self.get_logger().info('Pickup triggered by coordinator')
        self._pickup_triggered = True

    def _coordinator_pose_cb(self, msg: PoseStamped):
        """Refined averaged pose from coordinator — overrides raw perception in auto_start mode."""
        if self.auto_start:
            self.object_pose = msg
            self.object_found = True
            self.get_logger().info(
                f'Using coordinator refined pose: '
                f'({msg.pose.position.x:.3f}, {msg.pose.position.y:.3f}, {msg.pose.position.z:.3f})'
            )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _drain_callbacks(self, count: int = 30):
        for _ in range(count):
            rclpy.spin_once(self, timeout_sec=0.005)

    def wait_for_object_detection(self, timeout: float = 30.0, min_confidence: float = 0.5) -> bool:
        """Wait until the pillow is visible in the camera frame.

        Pose is intentionally NOT checked here — grasp positions are hardcoded.
        We only confirm the pillow is in frame before starting the arm sequence.
        """
        self.get_logger().info(f'Waiting for pillow to be visible (min confidence: {min_confidence:.2f})...')
        start = time.time()
        while (time.time() - start) < timeout:
            self._drain_callbacks()
            if self.object_found and self.object_confidence >= min_confidence:
                self.get_logger().info(
                    f'Pillow visible: conf={self.object_confidence:.2f} '
                    f'— using hardcoded grasp positions '
                    f'(x=±{self.HARDCODED_GRASP_X_RIGHT:.2f}, '
                    f'y={self.HARDCODED_GRASP_Y:.2f}, '
                    f'z={self.HARDCODED_GRASP_Z:.2f})'
                )
                return True
            time.sleep(0.05)
        return False

    def wait_for_joint_states(self, timeout: float = 10.0) -> bool:
        start = time.time()
        while (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.joint_states_received:
                return True
        return False

    def transition_to(self, state: PickupState):
        self.get_logger().info(f'State: {self.state.name} -> {state.name}')
        self.state = state

    def create_pose(self, x, y, z, qw, qx, qy, qz) -> Pose:
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.position.z = float(z)
        pose.orientation.w = float(qw)
        pose.orientation.x = float(qx)
        pose.orientation.y = float(qy)
        pose.orientation.z = float(qz)
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

    def _make_plan_request(self, arm_name: str, pose: Pose, duration: float = 3.0) -> PlanToTarget.Request:
        """Build a PlanToTarget request without sending it."""
        request = PlanToTarget.Request()
        request.arm_name = arm_name
        request.target_pose = pose
        request.use_orientation = True
        request.execute = True
        request.duration = duration
        return request

    def _check_result(self, arm_name: str, result) -> bool:
        """Log and return success/failure for one IK result."""
        if result is None:
            return False
        if result.success:
            self.get_logger().info(
                f'  {arm_name} SUCCESS: {result.message} | '
                f'pos_err={result.position_error:.4f}m  '
                f'ori_err={np.degrees(result.orientation_error):.1f}°'
            )
            return True
        self.get_logger().warn(f'  {arm_name} FAILED: {result.message}')
        return False

    def plan_and_execute(self, arm_name: str, pose: Pose, duration: float = 3.0) -> bool:
        """Send an IK plan+execute request for the given arm."""
        pos_str = f'({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})'
        self.get_logger().info(f'Planning+executing {arm_name} arm to {pos_str}')

        request = self._make_plan_request(arm_name, pose, duration)
        result = self.call_service_sync(request)
        return self._check_result(arm_name, result)

    def plan_and_execute_both(self, r_pose: Pose, l_pose: Pose, duration: float = 5.0) -> bool:
        """Send IK requests for both arms simultaneously and wait for both to finish."""
        self.get_logger().info(
            f'Planning+executing BOTH arms simultaneously: '
            f'right=({r_pose.position.x:.3f}, {r_pose.position.y:.3f}, {r_pose.position.z:.3f}), '
            f'left=({l_pose.position.x:.3f}, {l_pose.position.y:.3f}, {l_pose.position.z:.3f})'
        )

        r_req = self._make_plan_request('right', r_pose, duration)
        l_req = self._make_plan_request('left',  l_pose, duration)

        r_future = self.plan_client.call_async(r_req)
        l_future = self.plan_client.call_async(l_req)

        # Spin until both futures complete
        timeout = time.time() + duration + 15.0
        while not (r_future.done() and l_future.done()):
            rclpy.spin_once(self, timeout_sec=0.05)
            if time.time() > timeout:
                self.get_logger().error('Simultaneous lift timed out.')
                return False

        r_result = r_future.result() if r_future.done() and r_future.exception() is None else None
        l_result = l_future.result() if l_future.done() and l_future.exception() is None else None

        r_ok = self._check_result('right', r_result)
        l_ok = self._check_result('left',  l_result)
        return r_ok and l_ok

    def set_gripper(self, arm_name: str, position: float, duration: float = 2.0, hold: bool = False):
        """Set gripper position via wrapper node.

        Args:
            arm_name: 'right' or 'left'
            position: 0.0 (open) to 1.0 (closed)
            duration: seconds to hold the command
            hold: if True, skip stop command so gripper maintains force
        """
        pub = self.right_gripper_pub if arm_name == 'right' else self.left_gripper_pub
        msg = Float64MultiArray()
        msg.data = [float(position)]

        start = time.time()
        while (time.time() - start) < duration:
            pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)

        if not hold:
            stop_msg = Float64MultiArray()
            stop_msg.data = [0.5]
            pub.publish(stop_msg)
            rclpy.spin_once(self, timeout_sec=0.05)

    def open_gripper(self, arm_name: str):
        self.get_logger().info(f'Opening {arm_name} gripper...')
        self.set_gripper(arm_name, 0.0, duration=2.0)

    def close_gripper(self, arm_name: str):
        self.get_logger().info(f'Closing {arm_name} gripper...')
        self.set_gripper(arm_name, 1.0, duration=2.0, hold=True)

    def go_to_sleep(self, arm_name: str, max_joint_speed: float = 0.5):
        """Send one arm to sleep pose using cosine interpolation."""
        self.get_logger().info(f'Moving {arm_name} arm to sleep pose...')
        rclpy.spin_once(self, timeout_sec=0.1)

        joint_names = [
            f'{arm_name}_waist', f'{arm_name}_shoulder', f'{arm_name}_elbow',
            f'{arm_name}_forearm_roll', f'{arm_name}_wrist_angle', f'{arm_name}_wrist_rotate',
        ]
        current = np.array([self._latest_positions.get(j, 0.0) for j in joint_names])
        target = np.array(self.SLEEP_POSE)

        max_diff = np.max(np.abs(target - current))
        duration = max(max_diff / max_joint_speed, 1.0)
        pub = self.right_arm_pub if arm_name == 'right' else self.left_arm_pub

        rate_hz = 50.0
        dt = 1.0 / rate_hz
        num_steps = max(int(duration * rate_hz), 1)

        for i in range(num_steps + 1):
            t = i / num_steps
            alpha = 0.5 * (1 - np.cos(np.pi * t))
            q = current + alpha * (target - current)
            cmd = JointGroupCommand()
            cmd.name = f'{arm_name}_arm'
            cmd.cmd = q.tolist()
            pub.publish(cmd)
            if i < num_steps:
                time.sleep(dt)

        self.get_logger().info(f'{arm_name} arm is now in sleep pose.')

    # ------------------------------------------------------------------
    # Bimanual state machine
    # ------------------------------------------------------------------

    def run_state_machine(self) -> bool:
        """Execute bimanual pillow pickup sequence using hardcoded grasp positions."""
        # Hardcoded grasp positions — lateral spread in x, forward in y, height in z
        rx = self.HARDCODED_GRASP_X_RIGHT   # +0.07
        lx = self.HARDCODED_GRASP_X_LEFT    # -0.07
        gy = self.HARDCODED_GRASP_Y         # 0.40 m forward
        gz = self.HARDCODED_GRASP_Z         # 0.15 m height (grasp z)
        az = gz + self.APPROACH_HEIGHT      # 0.32 m (clear approach from above)
        lz = gz + (self.LIFT_HEIGHT - self.GRASP_Z_ABOVE)  # 0.30 m (lift after grasp)

        qw, qx, qy, qz = self.ORIENT_FINGERS_DOWN

        r_approach = self.create_pose(rx, gy, az, qw, qx, qy, qz)
        r_grasp    = self.create_pose(rx, gy, gz, qw, qx, qy, qz)
        l_approach = self.create_pose(lx, gy, az, qw, qx, qy, qz)
        l_grasp    = self.create_pose(lx, gy, gz, qw, qx, qy, qz)
        r_lift     = self.create_pose(rx, gy, lz, qw, qx, qy, qz)
        l_lift     = self.create_pose(lx, gy, lz, qw, qx, qy, qz)

        self.get_logger().info('')
        self.get_logger().info('Bimanual pickup waypoints — HARDCODED (base_link frame):')
        self.get_logger().info(f'  right approach:    ({rx:.3f}, {gy:.3f}, {az:.3f})')
        self.get_logger().info(f'  right grasp:       ({rx:.3f}, {gy:.3f}, {gz:.3f})')
        self.get_logger().info(f'  left  approach:    ({lx:.3f}, {gy:.3f}, {az:.3f})')
        self.get_logger().info(f'  left  grasp:       ({lx:.3f}, {gy:.3f}, {gz:.3f})')
        self.get_logger().info(f'  lift  (both):      (*, {gy:.3f}, {lz:.3f})')

        if self.auto_start or self._pickup_triggered:
            self.get_logger().info('Auto-starting pickup sequence (coordinator mode)')
        else:
            self.get_logger().warn('Ensure workspace is clear before continuing.')
            input('Press Enter to start bimanual pickup (Ctrl+C to abort)... ')

        while self.state != PickupState.DONE:
            if self.state == PickupState.R_APPROACH:
                self.open_gripper('right')
                if not self.plan_and_execute('right', r_approach, duration=10.0):
                    return False
                self.transition_to(PickupState.R_DESCEND)

            elif self.state == PickupState.R_DESCEND:
                if not self.plan_and_execute('right', r_grasp, duration=10.0):
                    return False
                self.transition_to(PickupState.R_GRASP)

            elif self.state == PickupState.R_GRASP:
                self.close_gripper('right')
                self.transition_to(PickupState.L_APPROACH)

            elif self.state == PickupState.L_APPROACH:
                self.open_gripper('left')
                if not self.plan_and_execute('left', l_approach, duration=10.0):
                    return False
                self.transition_to(PickupState.L_DESCEND)

            elif self.state == PickupState.L_DESCEND:
                if not self.plan_and_execute('left', l_grasp, duration=10.0):
                    return False
                self.transition_to(PickupState.L_GRASP)

            elif self.state == PickupState.L_GRASP:
                self.close_gripper('left')
                self.transition_to(PickupState.LIFT)

            elif self.state == PickupState.LIFT:
                self.get_logger().info('Lifting both arms simultaneously...')
                if not self.plan_and_execute_both(r_lift, l_lift, duration=5.0):
                    return False
                self.transition_to(PickupState.DONE)

            time.sleep(0.3)

        self.get_logger().info('Bimanual pickup complete.')
        self.pickup_complete_pub.publish(Bool(data=True))
        return True


# ------------------------------------------------------------------
# Run helpers
# ------------------------------------------------------------------

def _run_pickup_once(node: PickupNode):
    node.get_logger().info('Skipping object detection — using hardcoded grasp positions.')
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
            return 1

        if node.auto_start:
            node.get_logger().info('Auto-start mode: waiting for /coordinator/pickup_trigger ...')
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.1)
                if node._pickup_triggered:
                    node._pickup_triggered = False
                    node.state = PickupState.R_APPROACH  # reset state machine
                    rc = _run_pickup_once(node)
                    if rc != 0:
                        node.get_logger().error(f'Pickup failed with code {rc}')
            return 0
        else:
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
