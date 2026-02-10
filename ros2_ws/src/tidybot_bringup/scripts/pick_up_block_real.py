#!/usr/bin/env python3
"""
TidyBot2 Block Pickup Demo - Real Hardware with IK Service

Uses the Pinocchio-based IK service (/plan_to_target) to compute joint positions
for picking up a block from a specified position.

Controls the arm using the direct Interbotix SDK interface (JointGroupCommand)
and grippers via JointSingleCommand (PWM mode), matching the patterns in
test_real_hardware.py and test_planner_real.py.

Services used:
  - /plan_to_target (tidybot_msgs/srv/PlanToTarget) : IK solver
  - /right_arm/torque_enable (interbotix_xs_msgs/srv/TorqueEnable) : Enable motors

Topics:
  - /right_arm/commands/joint_group (JointGroupCommand) : arm joint commands
  - /right_arm/commands/joint_single (JointSingleCommand) : gripper PWM commands
  - /joint_states (sensor_msgs/JointState) : merged joint states

Usage:
    # Make sure the IK service is running:
    ros2 launch tidybot_bringup real.launch.py use_planner:=true

    # Then run this script:
    python3 pick_up_block_real.py
"""

import sys
import time
import numpy as np

import rclpy
from rclpy.node import Node
from interbotix_xs_msgs.srv import TorqueEnable
from interbotix_xs_msgs.msg import JointGroupCommand, JointSingleCommand
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, Quaternion, Point
from tidybot_msgs.srv import PlanToTarget


# ----------------------------
# Block position in base_link frame (meters)
# TODO: Replace with actual block detection (e.g., from camera)
# ----------------------------
BLOCK_POS = np.array([0.45, -0.10, 0.02])  # [x, y, z] in base_link frame

# Grasp approach offsets (relative to block position)
APPROACH_HEIGHT = 0.15  # 15cm above block
GRASP_HEIGHT = 0.06     # 6cm above block (adjust based on gripper + block size)
LIFT_HEIGHT = 0.20      # 20cm lift after grasping

# Top-down grasp orientation (gripper pointing down)
# Quaternion for 180 deg rotation around Y-axis
TOP_DOWN_QUAT = [0.0, 1.0, 0.0, 0.0]  # [x, y, z, w]

# Sleep pose (safe resting position)
SLEEP_POSE = [0.0, -1.80, 1.55, 0.0, 0.8, 0.0]

# Gripper PWM values (matching test_real_hardware.py)
GRIPPER_OPEN_PWM = -350.0
GRIPPER_CLOSE_PWM = 350.0

# Right arm joint names (for reading from merged /joint_states)
RIGHT_ARM_JOINT_NAMES = [
    'right_waist', 'right_shoulder', 'right_elbow',
    'right_forearm_roll', 'right_wrist_angle', 'right_wrist_rotate',
]


class PickupReal(Node):
    def __init__(self):
        super().__init__("pickup_real")

        # Arm publisher - direct Interbotix SDK topic (JointGroupCommand)
        self.right_arm_pub = self.create_publisher(
            JointGroupCommand, '/right_arm/commands/joint_group', 10
        )

        # Gripper publisher - direct Interbotix SDK topic (JointSingleCommand, PWM)
        self.right_gripper_pub = self.create_publisher(
            JointSingleCommand, '/right_arm/commands/joint_single', 10
        )

        # Service clients
        self.right_torque_client = self.create_client(TorqueEnable, "/right_arm/torque_enable")
        self.ik_client = self.create_client(PlanToTarget, "/plan_to_target")

        # Subscribe to merged /joint_states (matching test_planner_real.py)
        self.joint_states_received = False
        self.current_joint_positions = {}
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

        # Wait for services
        self.get_logger().info("Waiting for services...")

        torque_ready = self.right_torque_client.wait_for_service(timeout_sec=5.0)
        if torque_ready:
            self.get_logger().info("Connected to right_arm torque service")
        else:
            self.get_logger().warn("No right_arm torque service found!")

        ik_ready = self.ik_client.wait_for_service(timeout_sec=5.0)
        if ik_ready:
            self.get_logger().info("Connected to IK service")
        else:
            self.get_logger().error("IK service not available!")
            self.get_logger().error("Launch with: ros2 launch tidybot_bringup real.launch.py use_planner:=true")

    def _joint_state_cb(self, msg):
        self.joint_states_received = True
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]

    def wait_for_joint_states(self, timeout=10.0):
        """Wait for joint states to be received."""
        start = time.time()
        while (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.joint_states_received:
                return True
        return False

    def get_right_arm_positions(self) -> np.ndarray:
        """Get current right arm joint positions from merged /joint_states."""
        return np.array([
            self.current_joint_positions.get(name, 0.0)
            for name in RIGHT_ARM_JOINT_NAMES
        ])

    def set_torque(self, enable: bool):
        """Enable/disable torque on the right arm."""
        req = TorqueEnable.Request()
        req.cmd_type = "group"
        req.name = "right_arm"
        req.enable = enable

        future = self.right_torque_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result()

    def solve_ik_for_pose(self, position, orientation=None, use_orientation=False):
        """
        Solve IK for a target pose using the /plan_to_target service.

        Args:
            position: [x, y, z] target position in base_link frame
            orientation: [x, y, z, w] quaternion (optional)
            use_orientation: whether to match orientation or position only

        Returns:
            (success, joint_positions, error_message)
        """
        req = PlanToTarget.Request()
        req.arm_name = "right"
        req.execute = False  # We just want the solution, not execution
        req.use_orientation = use_orientation
        req.duration = 2.0
        req.max_condition_number = 100.0

        # Set target pose
        req.target_pose = Pose()
        req.target_pose.position = Point(x=position[0], y=position[1], z=position[2])

        if orientation is not None:
            req.target_pose.orientation = Quaternion(
                x=orientation[0], y=orientation[1],
                z=orientation[2], w=orientation[3]
            )
        else:
            req.target_pose.orientation = Quaternion(
                x=TOP_DOWN_QUAT[0], y=TOP_DOWN_QUAT[1],
                z=TOP_DOWN_QUAT[2], w=TOP_DOWN_QUAT[3]
            )

        # Call IK service
        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f"IK solved: pos_err={response.position_error:.4f}m, "
                    f"cond={response.condition_number:.1f}"
                )
                return True, list(response.joint_positions), ""
            else:
                return False, None, response.message
        else:
            return False, None, "IK service call failed"

    def move_right_arm(self, target_positions, max_joint_speed=0.5):
        """Move right arm using cosine-interpolated trajectory (from test_planner_real.py).

        Args:
            target_positions: list of 6 joint positions in radians
            max_joint_speed: maximum joint velocity in rad/s (default 0.5 ~ 30 deg/s)
        """
        # Spin to get latest joint states
        rclpy.spin_once(self, timeout_sec=0.1)

        current = self.get_right_arm_positions()
        target = np.array(target_positions)

        # Calculate duration based on max joint difference
        max_diff = np.max(np.abs(target - current))
        duration = max(max_diff / max_joint_speed, 1.0)  # At least 1 second

        self.get_logger().info(
            f"Moving right arm over {duration:.1f}s "
            f"(max joint diff: {max_diff:.2f} rad)"
        )

        # Cosine-interpolated trajectory at 50Hz
        rate_hz = 50.0
        dt = 1.0 / rate_hz
        num_steps = max(int(duration * rate_hz), 1)

        for i in range(num_steps + 1):
            t = i / num_steps
            # Cosine interpolation for smooth acceleration/deceleration
            alpha = 0.5 * (1 - np.cos(np.pi * t))

            q = current + alpha * (target - current)

            cmd = JointGroupCommand()
            cmd.name = 'right_arm'
            cmd.cmd = q.tolist()
            self.right_arm_pub.publish(cmd)

            if i < num_steps:
                time.sleep(dt)

    def set_right_gripper(self, open_gripper: bool, duration=1.0):
        """
        Control right gripper using direct PWM via JointSingleCommand.

        Args:
            open_gripper: True to open, False to close
            duration: how long to send the PWM command (seconds)
        """
        msg = JointSingleCommand()
        msg.name = 'right_gripper'
        msg.cmd = GRIPPER_OPEN_PWM if open_gripper else GRIPPER_CLOSE_PWM

        state_desc = "OPEN" if open_gripper else "CLOSED"
        self.get_logger().info(f"Right gripper -> {state_desc} (PWM={msg.cmd:.0f})")

        start = time.time()
        while (time.time() - start) < duration:
            self.right_gripper_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)  # 10Hz

    def run_pickup(self):
        """Execute the pickup sequence using IK to compute waypoints."""
        print()
        print("=" * 60)
        print("  TidyBot2 Block Pickup - Real Hardware")
        print("=" * 60)
        print(f"Block position: {BLOCK_POS.tolist()} (in base_link frame)")
        print("Sequence: approach -> descend -> grasp -> lift -> sleep")
        print()

        # Enable torque
        self.get_logger().info("Enabling torque on right_arm...")
        self.set_torque(True)
        time.sleep(0.5)

        # Open gripper before starting
        self.get_logger().info("Opening gripper...")
        self.set_right_gripper(open_gripper=True, duration=1.0)

        # ====================================================================
        # Phase 1: APPROACH - Move to position above the block
        # ====================================================================
        self.get_logger().info("\n--- Phase 1: APPROACH ---")
        approach_pos = BLOCK_POS + np.array([0, 0, APPROACH_HEIGHT])
        self.get_logger().info(f"Target: {approach_pos.tolist()}")

        success, approach_joints, error_msg = self.solve_ik_for_pose(approach_pos, use_orientation=True)
        if not success:
            self.get_logger().error(f"Failed to solve IK for approach: {error_msg}")
            return False

        self.move_right_arm(approach_joints)

        # ====================================================================
        # Phase 2: DESCEND - Lower to grasp height
        # ====================================================================
        self.get_logger().info("\n--- Phase 2: DESCEND ---")
        grasp_pos = BLOCK_POS + np.array([0, 0, GRASP_HEIGHT])
        self.get_logger().info(f"Target: {grasp_pos.tolist()}")

        success, grasp_joints, error_msg = self.solve_ik_for_pose(grasp_pos, use_orientation=True)
        if not success:
            self.get_logger().error(f"Failed to solve IK for grasp: {error_msg}")
            return False

        self.move_right_arm(grasp_joints)

        # ====================================================================
        # Phase 3: GRASP - Close gripper
        # ====================================================================
        self.get_logger().info("\n--- Phase 3: GRASP ---")
        self.set_right_gripper(open_gripper=False, duration=1.5)

        # ====================================================================
        # Phase 4: LIFT - Raise the grasped block
        # ====================================================================
        self.get_logger().info("\n--- Phase 4: LIFT ---")
        lift_pos = BLOCK_POS + np.array([0, 0, LIFT_HEIGHT])
        self.get_logger().info(f"Target: {lift_pos.tolist()}")

        success, lift_joints, error_msg = self.solve_ik_for_pose(lift_pos, use_orientation=True)
        if not success:
            self.get_logger().error(f"Failed to solve IK for lift: {error_msg}")
            return False

        self.move_right_arm(lift_joints)

        # ====================================================================
        # Phase 5: SLEEP - Return to safe position
        # ====================================================================
        self.get_logger().info("\n--- Phase 5: SLEEP ---")
        self.move_right_arm(SLEEP_POSE)

        print()
        print("=" * 60)
        print("  Pickup complete!")
        print("=" * 60)
        print()

        return True


def main():
    rclpy.init()
    node = PickupReal()

    try:
        print("\nWaiting for joint states...")
        if not node.wait_for_joint_states(timeout=10.0):
            print("ERROR: No joint states received!")
            print("\nMake sure to launch:")
            print("  ros2 launch tidybot_bringup real.launch.py use_planner:=true")
            return 1

        # Check if IK service is available
        if not node.ik_client.wait_for_service(timeout_sec=2.0):
            print("ERROR: IK service not available!")
            print("\nMake sure to launch with planner:")
            print("  ros2 launch tidybot_bringup real.launch.py use_planner:=true")
            return 1

        # Run the pickup sequence
        success = node.run_pickup()

        if success:
            print("\nPickup sequence completed successfully!")
            return 0
        else:
            print("\nPickup sequence failed")
            return 1

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
        return 130
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
