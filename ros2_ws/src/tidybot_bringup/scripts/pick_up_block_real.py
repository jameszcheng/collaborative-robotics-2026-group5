#!/usr/bin/env python3
"""
TidyBot2 Block Pickup Demo - Real Hardware with IK Service

Uses the Pinocchio-based IK service (/plan_to_target) to compute joint positions
for picking up a block from a specified position.

Services used:
  - /plan_to_target (tidybot_msgs/srv/PlanToTarget) : IK solver
  - /right_arm/torque_enable (interbotix_xs_msgs/srv/TorqueEnable) : Enable motors

Topics:
  - /right_arm/joint_cmd (Float64MultiArray) : 6 joint positions
  - /right_gripper/cmd (Float64MultiArray) : 0=open, 1=closed, 0.5=stop
  - /right_arm/joint_states (sensor_msgs/JointState) : Current joint states

Usage:
    # Make sure the IK service is running:
    ros2 run tidybot_ik motion_planner_real_node

    # Then run this script:
    python3 pick_up_block_real.py
"""

import sys
import time
import numpy as np

import rclpy
from rclpy.node import Node
from interbotix_xs_msgs.srv import TorqueEnable
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
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
# Quaternion for 180° rotation around X-axis
TOP_DOWN_QUAT = [0.0, 1.0, 0.0, 0.0]  # [x, y, z, w]

# Sleep pose (safe resting position)
SLEEP_POSE = [0.0, -1.80, 1.55, 0.0, 0.8, 0.0]  # [waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate]


class PickupReal(Node):
    def __init__(self):
        super().__init__("pickup_real")

        # Publishers for arm and gripper commands
        self.right_arm_pub = self.create_publisher(Float64MultiArray, "/right_arm/joint_cmd", 10)
        self.right_gripper_pub = self.create_publisher(Float64MultiArray, "/right_gripper/cmd", 10)

        # Service clients
        self.right_torque_client = self.create_client(TorqueEnable, "/right_arm/torque_enable")
        self.ik_client = self.create_client(PlanToTarget, "/plan_to_target")

        # Subscriber for joint states
        self.right_joint_states = None
        self.right_js_sub = self.create_subscription(
            JointState, "/right_arm/joint_states", self._right_joint_state_cb, 10
        )

        # Wait for services
        self.get_logger().info("Waiting for services...")

        torque_ready = self.right_torque_client.wait_for_service(timeout_sec=5.0)
        if torque_ready:
            self.get_logger().info("✓ Connected to right_arm torque service")
        else:
            self.get_logger().warn("✗ No right_arm torque service found!")

        ik_ready = self.ik_client.wait_for_service(timeout_sec=5.0)
        if ik_ready:
            self.get_logger().info("✓ Connected to IK service")
        else:
            self.get_logger().error("✗ IK service not available!")
            self.get_logger().error("Please run: ros2 run tidybot_ik motion_planner_real_node")

    def _right_joint_state_cb(self, msg):
        self.right_joint_states = msg

    def wait_for_joint_states(self, timeout=10.0):
        """Wait for joint states to be received."""
        start = time.time()
        while (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.right_joint_states is not None:
                return True
        return False

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

        # Set target pose
        req.target_pose = Pose()
        req.target_pose.position = Point(x=position[0], y=position[1], z=position[2])

        if orientation is not None:
            req.target_pose.orientation = Quaternion(
                x=orientation[0], y=orientation[1],
                z=orientation[2], w=orientation[3]
            )
        else:
            # Use top-down grasp orientation by default
            req.target_pose.orientation = Quaternion(
                x=TOP_DOWN_QUAT[0], y=TOP_DOWN_QUAT[1],
                z=TOP_DOWN_QUAT[2], w=TOP_DOWN_QUAT[3]
            )

        # Call IK service
        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(
                    f"IK solved: pos_err={response.position_error:.4f}m, "
                    f"cond={response.condition_number:.1f}"
                )
                return True, response.joint_positions, ""
            else:
                return False, None, response.message
        else:
            return False, None, "IK service call failed"

    # --- Motion helpers: follow your tester style closely ---
    def move_right_arm(self, positions, move_time=2.0):
        msg = Float64MultiArray()
        msg.data = [float(x) for x in positions]

        self.get_logger().info(f"Moving right arm to {[f'{p:.2f}' for p in positions]}")
        self.right_arm_pub.publish(msg)

        time.sleep(move_time)
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.05)

    def set_right_gripper(self, position, duration=1.0):
        """
        Wrapper gripper behavior copied from your tester:
          - publish [position] for duration at ~10Hz
          - then publish [0.5] to stop PWM
        """
        msg = Float64MultiArray()
        msg.data = [float(position)]

        state_desc = "OPEN" if position < 0.5 else "CLOSED"
        self.get_logger().info(f"Right gripper -> {state_desc} ({position:.1f})")

        start = time.time()
        while (time.time() - start) < duration:
            self.right_gripper_pub.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)  # 10Hz

        stop_msg = Float64MultiArray()
        stop_msg.data = [0.5]
        self.right_gripper_pub.publish(stop_msg)
        rclpy.spin_once(self, timeout_sec=0.05)

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
        self.set_right_gripper(0.0, duration=1.0)

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

        self.move_right_arm(approach_joints, move_time=3.0)

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

        # Move with intermediate waypoint for smooth descent
        mid_joints = [(a + g) / 2 for a, g in zip(approach_joints, grasp_joints)]
        self.move_right_arm(mid_joints, move_time=1.0)
        self.move_right_arm(grasp_joints, move_time=1.0)

        # ====================================================================
        # Phase 3: GRASP - Close gripper
        # ====================================================================
        self.get_logger().info("\n--- Phase 3: GRASP ---")
        self.set_right_gripper(1.0, duration=1.5)

        # ====================================================================
        # Phase 4: LIFT - Raise the grasped block
        # ====================================================================
        self.get_logger().info("\n--- Phase 4: LIFT ---")
        lift_pos = BLOCK_POS + np.array([0, 0, LIFT_HEIGHT])
        self.get_logger().info(f"Target: {lift_pos.tolist()}")

        success, lift_joints, error_msg = self.solve_ik_for_pose(lift_pos, use_orientation=True)
        if not success:
            self.get_logger().error(f"Failed to solve IK for lift: {error_msg}")
            # Still try to lift with interpolation from current position
            lift_joints = [(g + a) / 2 for g, a in zip(grasp_joints, approach_joints)]

        # Move with intermediate waypoint for smooth lift
        mid_joints = [(g + l) / 2 for g, l in zip(grasp_joints, lift_joints)]
        self.move_right_arm(mid_joints, move_time=1.0)
        self.move_right_arm(lift_joints, move_time=1.0)

        # ====================================================================
        # Phase 5: SLEEP - Return to safe position
        # ====================================================================
        self.get_logger().info("\n--- Phase 5: SLEEP ---")
        self.move_right_arm(SLEEP_POSE, move_time=3.0)

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
        print("\nWaiting for right arm joint states...")
        if not node.wait_for_joint_states(timeout=10.0):
            print("ERROR: No joint states received!")
            print("\nMake sure to launch:")
            print("  Terminal 1: ros2 launch tidybot_bringup real.launch.py")
            print("  Terminal 2: ros2 run tidybot_ik motion_planner_real_node")
            return 1

        # Check if IK service is available
        if not node.ik_client.wait_for_service(timeout_sec=2.0):
            print("ERROR: IK service not available!")
            print("\nMake sure to run:")
            print("  ros2 run tidybot_ik motion_planner_real_node")
            return 1

        # Run the pickup sequence
        success = node.run_pickup()

        if success:
            print("\n✓ Pickup sequence completed successfully!")
            return 0
        else:
            print("\n✗ Pickup sequence failed")
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