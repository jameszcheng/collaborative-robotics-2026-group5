#!/usr/bin/env python3
"""
Test bimanual arm control with various gripper orientations.

Demonstrates reaching different orientations with both arms using the
IK planning service. Each test logs the orientation name and quaternion
so you can see which orientations are reachable.

Usage:
    # Terminal 1: Start hardware (or sim) with motion planner
    ros2 launch tidybot_bringup real.launch.py use_planner:=true

    # Terminal 2: Run this test
    python3 test_bimanual.py
"""

import time

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from interbotix_xs_msgs.msg import JointGroupCommand
from rclpy.node import Node
from sensor_msgs.msg import JointState
from tidybot_msgs.srv import PlanToTarget


# ---------------------------------------------------------------------------
# Named orientations (quaternion wxyz) in base_link frame
#
# The WX250s ee x-axis runs along the gripper fingers.
# ---------------------------------------------------------------------------
ORIENTATIONS = {
    'fingers_down': {
        'quat': (0.5, 0.5, 0.5, -0.5),
        'desc': 'Fingers point straight down (-Z). Standard top-down grasp.',
    },
    'fingers_down_rot90': {
        'quat': (0.707107, 0.0, 0.707107, 0.0),
        'desc': 'Fingers down, wrist rotated 90 deg. Rotated top-down grasp.',
    },
    'fingers_forward': {
        'quat': (0.0, 0.707107, 0.0, -0.707107),
        'desc': 'Fingers point forward (-Y). Horizontal reach.',
    },
    'fingers_inward_right': {
        'quat': (0.5, 0.5, -0.5, -0.5),
        'desc': 'Fingers point left (+X). Right arm reaching inward.',
    },
    'fingers_inward_left': {
        'quat': (0.5, -0.5, 0.5, -0.5),
        'desc': 'Fingers point right (-X). Left arm reaching inward.',
    },
    'fingers_outward_right': {
        'quat': (0.5, -0.5, 0.5, -0.5),
        'desc': 'Fingers point right (-X). Right arm reaching outward.',
    },
    'fingers_outward_left': {
        'quat': (0.5, 0.5, -0.5, -0.5),
        'desc': 'Fingers point left (+X). Left arm reaching outward.',
    },
    'tilted_45_down_forward': {
        'quat': (0.271, 0.653, 0.271, -0.653),
        'desc': 'Fingers tilted 45 deg between down and forward. Angled grasp.',
    },
}

# Test positions in base_link frame (x, y, z)
# -Y is forward, +X is left, +Z is up
# Right arm shoulder ~(-0.15, -0.12, 0.45), left ~(0.15, -0.12, 0.45)
# Spread arms wider (0.20m from center each) and keep above shoulder height
# to avoid camera mast and inter-arm collisions
RIGHT_TEST_POS = (-0.20, -0.30, 0.55)
LEFT_TEST_POS = (0.20, -0.30, 0.55)

SLEEP_POSE = [0.0, -1.80, 1.55, 0.0, 0.8, 0.0]


class TestBimanual(Node):
    def __init__(self):
        super().__init__('test_bimanual')

        self.plan_client = self.create_client(PlanToTarget, '/plan_to_target')

        self.arm_cmd_pubs = {
            'right': self.create_publisher(JointGroupCommand, '/right_arm/commands/joint_group', 10),
            'left': self.create_publisher(JointGroupCommand, '/left_arm/commands/joint_group', 10),
        }

        self.joint_states_received = False
        self.current_joint_positions = {}
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        self.get_logger().info('Waiting for /plan_to_target service...')
        if not self.plan_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(
                'Planning service not available! '
                'Launch with: ros2 launch tidybot_bringup real.launch.py use_planner:=true'
            )
        self.get_logger().info('Service connected.')

        # Wait for joint states
        for _ in range(50):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.joint_states_received:
                break

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _js_cb(self, msg):
        self.joint_states_received = True
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]

    def _make_pose(self, x, y, z, qw, qx, qy, qz):
        p = Pose()
        p.position.x = float(x)
        p.position.y = float(y)
        p.position.z = float(z)
        p.orientation.w = float(qw)
        p.orientation.x = float(qx)
        p.orientation.y = float(qy)
        p.orientation.z = float(qz)
        return p

    def _call_plan(self, request, timeout=15.0):
        future = self.plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.exception() is not None:
            return None
        return future.result()

    def plan_and_execute(self, arm_name, pose, use_orientation=True, duration=3.0):
        req = PlanToTarget.Request()
        req.arm_name = arm_name
        req.target_pose = pose
        req.use_orientation = use_orientation
        req.execute = True
        req.duration = duration
        req.max_condition_number = 500.0
        return self._call_plan(req)

    def plan_only(self, arm_name, pose, use_orientation=True):
        req = PlanToTarget.Request()
        req.arm_name = arm_name
        req.target_pose = pose
        req.use_orientation = use_orientation
        req.execute = False
        req.duration = 2.0
        req.max_condition_number = 500.0
        return self._call_plan(req)

    def go_to_sleep(self, arm_name, max_speed=0.5):
        rclpy.spin_once(self, timeout_sec=0.1)
        joint_names = [f'{arm_name}_{j}' for j in
                       ['waist', 'shoulder', 'elbow', 'forearm_roll', 'wrist_angle', 'wrist_rotate']]
        current = np.array([self.current_joint_positions.get(n, 0.0) for n in joint_names])
        target = np.array(SLEEP_POSE)
        duration = max(np.max(np.abs(target - current)) / max_speed, 1.0)

        rate_hz = 50.0
        num_steps = max(int(duration * rate_hz), 1)
        for i in range(num_steps + 1):
            alpha = 0.5 * (1 - np.cos(np.pi * i / num_steps))
            q = current + alpha * (target - current)
            cmd = JointGroupCommand()
            cmd.name = f'{arm_name}_arm'
            cmd.cmd = q.tolist()
            self.arm_cmd_pubs[arm_name].publish(cmd)
            if i < num_steps:
                time.sleep(1.0 / rate_hz)

    # ------------------------------------------------------------------
    # Test routines
    # ------------------------------------------------------------------

    def test_orientation(self, arm_name, orient_name, orient_info, pos, execute=False):
        """Test a single orientation. Returns True if IK succeeded."""
        qw, qx, qy, qz = orient_info['quat']
        x, y, z = pos
        pose = self._make_pose(x, y, z, qw, qx, qy, qz)

        self.get_logger().info(
            f'  [{arm_name:>5}] {orient_name:<30} '
            f'quat=({qw:.3f}, {qx:.3f}, {qy:.3f}, {qz:.3f})'
        )
        self.get_logger().info(f'         {orient_info["desc"]}')

        if execute:
            result = self.plan_and_execute(arm_name, pose, use_orientation=True, duration=3.0)
        else:
            result = self.plan_only(arm_name, pose, use_orientation=True)

        if result is None:
            self.get_logger().error(f'         -> SERVICE ERROR')
            return False

        if result.success:
            self.get_logger().info(
                f'         -> OK  pos_err={result.position_error:.4f}m  '
                f'ori_err={np.degrees(result.orientation_error):.1f}deg  '
                f'cond={result.condition_number:.0f}'
            )
            return True
        else:
            self.get_logger().warn(f'         -> FAILED: {result.message}')
            return False

    def run_tests(self):
        # ── Part A: Plan-only survey of all orientations ──────────────
        self.get_logger().info('')
        self.get_logger().info('=' * 65)
        self.get_logger().info('PART A: Orientation reachability survey (plan only, no motion)')
        self.get_logger().info('=' * 65)

        results = {}

        for arm_name, pos in [('right', RIGHT_TEST_POS), ('left', LEFT_TEST_POS)]:
            self.get_logger().info('')
            self.get_logger().info(f'--- {arm_name.upper()} ARM at pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) ---')
            for orient_name, orient_info in ORIENTATIONS.items():
                ok = self.test_orientation(arm_name, orient_name, orient_info, pos, execute=False)
                results[(arm_name, orient_name)] = ok
                time.sleep(0.5)

        # Print summary table
        self.get_logger().info('')
        self.get_logger().info('=' * 65)
        self.get_logger().info('REACHABILITY SUMMARY')
        self.get_logger().info('=' * 65)
        self.get_logger().info(f'  {"Orientation":<30} {"Right":>8} {"Left":>8}')
        self.get_logger().info(f'  {"-"*30} {"-"*8} {"-"*8}')
        for orient_name in ORIENTATIONS:
            r = 'OK' if results.get(('right', orient_name)) else 'FAIL'
            l = 'OK' if results.get(('left', orient_name)) else 'FAIL'
            self.get_logger().info(f'  {orient_name:<30} {r:>8} {l:>8}')

        # ── Part B: Execute reachable orientations ────────────────────
        reachable = [
            (arm, name) for (arm, name), ok in results.items() if ok
        ]

        if not reachable:
            self.get_logger().warn('No reachable orientations found — skipping execution.')
            return

        self.get_logger().info('')
        self.get_logger().info('=' * 65)
        self.get_logger().info('PART B: Execute reachable orientations (robot WILL move)')
        self.get_logger().info(f'  {len(reachable)} reachable orientation(s) to demonstrate.')
        self.get_logger().info('=' * 65)
        input('\n  Press Enter to begin execution (Ctrl+C to abort)...\n')

        for arm_name, orient_name in reachable:
            pos = RIGHT_TEST_POS if arm_name == 'right' else LEFT_TEST_POS
            orient_info = ORIENTATIONS[orient_name]

            self.get_logger().info('')
            self.get_logger().info(f'--- Executing: {arm_name} arm, {orient_name} ---')
            self.test_orientation(arm_name, orient_name, orient_info, pos, execute=True)
            time.sleep(3.0)

            # Return to sleep between tests
            self.get_logger().info(f'  Returning {arm_name} arm to sleep...')
            self.go_to_sleep(arm_name)
            time.sleep(1.0)

        # ── Part C: Simultaneous bimanual demo ────────────────────────
        # Pick an orientation reachable by both arms
        both_ok = [
            name for name in ORIENTATIONS
            if results.get(('right', name)) and results.get(('left', name))
        ]

        if both_ok:
            orient_name = both_ok[0]
            orient_info = ORIENTATIONS[orient_name]
            qw, qx, qy, qz = orient_info['quat']

            self.get_logger().info('')
            self.get_logger().info('=' * 65)
            self.get_logger().info(f'PART C: Simultaneous bimanual move — {orient_name}')
            self.get_logger().info('=' * 65)
            input('\n  Press Enter to move both arms together (Ctrl+C to abort)...\n')

            # Plan both arms first (don't execute yet)
            right_pose = self._make_pose(*RIGHT_TEST_POS, qw, qx, qy, qz)
            left_pose = self._make_pose(*LEFT_TEST_POS, qw, qx, qy, qz)

            self.get_logger().info('Planning right arm...')
            r_req = PlanToTarget.Request()
            r_req.arm_name = 'right'
            r_req.target_pose = right_pose
            r_req.use_orientation = True
            r_req.execute = False
            r_req.duration = 3.0
            r_req.max_condition_number = 500.0
            r_result = self._call_plan(r_req)

            self.get_logger().info('Planning left arm...')
            l_req = PlanToTarget.Request()
            l_req.arm_name = 'left'
            l_req.target_pose = left_pose
            l_req.use_orientation = True
            l_req.execute = False
            l_req.duration = 3.0
            l_req.max_condition_number = 500.0
            l_result = self._call_plan(l_req)

            if r_result and r_result.success and l_result and l_result.success:
                self.get_logger().info('Both plans succeeded — executing simultaneously...')

                # Execute both by publishing joint commands at the same time
                r_req.execute = True
                l_req.execute = True
                # Send right arm
                self.plan_and_execute('right', right_pose, use_orientation=True, duration=3.0)
                # Send left arm immediately after
                self.plan_and_execute('left', left_pose, use_orientation=True, duration=3.0)

                time.sleep(4.0)
            else:
                self.get_logger().warn('One or both plans failed — skipping bimanual demo.')

        # ── Return to sleep ───────────────────────────────────────────
        self.get_logger().info('')
        self.get_logger().info('Returning both arms to sleep...')
        self.go_to_sleep('right')
        self.go_to_sleep('left')

        self.get_logger().info('')
        self.get_logger().info('=' * 65)
        self.get_logger().info('Test complete!')
        self.get_logger().info('=' * 65)


def main(args=None):
    rclpy.init(args=args)
    node = TestBimanual()

    try:
        node.run_tests()
    except KeyboardInterrupt:
        print('\nInterrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
