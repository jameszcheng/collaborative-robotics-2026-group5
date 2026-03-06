#!/usr/bin/env python3
"""
Test if a block position is reachable by the right arm.

This script tests the IK solver for a given block position without
actually moving the robot. Useful for validating block positions
before attempting pickup.

Usage:
    python3 test_block_reachability.py [x] [y] [z]

Example:
    python3 test_block_reachability.py 0.45 -0.10 0.02
"""

import sys
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Quaternion, Point
from tidybot_msgs.srv import PlanToTarget


class ReachabilityTester(Node):
    def __init__(self):
        super().__init__('reachability_tester')

        # Create IK service client
        self.ik_client = self.create_client(PlanToTarget, '/plan_to_target')

        self.get_logger().info('Waiting for IK service...')
        if self.ik_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('✓ Connected to IK service')
        else:
            self.get_logger().error('✗ IK service not available!')
            self.get_logger().error('Please run: ros2 run tidybot_ik motion_planner_real_node')

    def test_reachability(self, block_pos, approach_height=0.15, grasp_height=0.06, lift_height=0.20):
        """Test if all pickup waypoints are reachable."""

        # Top-down grasp orientation
        top_down_quat = [0.0, 1.0, 0.0, 0.0]  # [x, y, z, w]

        print()
        print("=" * 70)
        print("  Block Reachability Test")
        print("=" * 70)
        print(f"Block position: [{block_pos[0]:.3f}, {block_pos[1]:.3f}, {block_pos[2]:.3f}]")
        print()

        # Test approach position
        print("Testing waypoints...")
        print("-" * 70)

        waypoints = {
            'Approach': block_pos + np.array([0, 0, approach_height]),
            'Grasp': block_pos + np.array([0, 0, grasp_height]),
            'Lift': block_pos + np.array([0, 0, lift_height]),
        }

        all_reachable = True

        for name, target_pos in waypoints.items():
            success, joints, error_msg = self.solve_ik_for_pose(target_pos, top_down_quat)

            if success:
                print(f"✓ {name:10s} [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]")
                print(f"              Joints: {[f'{j:.2f}' for j in joints]}")
            else:
                print(f"✗ {name:10s} [{target_pos[0]:.3f}, {target_pos[1]:.3f}, {target_pos[2]:.3f}]")
                print(f"              Error: {error_msg}")
                all_reachable = False

            print()

        print("-" * 70)

        if all_reachable:
            print("✓ All waypoints are reachable!")
            print()
            print("You can now run the pickup with this block position:")
            print(f"  BLOCK_POS = np.array([{block_pos[0]:.3f}, {block_pos[1]:.3f}, {block_pos[2]:.3f}])")
            print()
        else:
            print("✗ Some waypoints are NOT reachable")
            print()
            print("Suggestions:")
            print("  1. Move the block closer to the robot (decrease X)")
            print("  2. Adjust the Y position (left/right)")
            print("  3. Adjust the Z position (height)")
            print("  4. Check if the block is within the arm's workspace")
            print()
            print("Right arm workspace (approximate):")
            print("  X: 0.20 to 0.60 meters (forward from base)")
            print("  Y: -0.40 to 0.20 meters (right to left)")
            print("  Z: -0.10 to 0.40 meters (ground to maximum height)")
            print()

        print("=" * 70)
        print()

        return all_reachable

    def solve_ik_for_pose(self, position, orientation):
        """Solve IK for a target pose."""
        req = PlanToTarget.Request()
        req.arm_name = "right"
        req.execute = False
        req.use_orientation = True

        req.target_pose = Pose()
        req.target_pose.position = Point(x=position[0], y=position[1], z=position[2])
        req.target_pose.orientation = Quaternion(
            x=orientation[0], y=orientation[1],
            z=orientation[2], w=orientation[3]
        )

        future = self.ik_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            response = future.result()
            if response.success:
                return True, response.joint_positions, ""
            else:
                return False, None, response.message
        else:
            return False, None, "IK service call failed"


def main():
    rclpy.init()
    node = ReachabilityTester()

    # Parse command-line arguments
    if len(sys.argv) == 4:
        try:
            x = float(sys.argv[1])
            y = float(sys.argv[2])
            z = float(sys.argv[3])
            block_pos = np.array([x, y, z])
        except ValueError:
            print("Error: Invalid coordinates")
            print("Usage: python3 test_block_reachability.py [x] [y] [z]")
            print("Example: python3 test_block_reachability.py 0.45 -0.10 0.02")
            return 1
    else:
        # Default test position
        print("No coordinates provided, using default position")
        block_pos = np.array([0.45, -0.10, 0.02])

    try:
        # Check if IK service is available
        if not node.ik_client.wait_for_service(timeout_sec=2.0):
            print("\nERROR: IK service not available!")
            print("Please run:")
            print("  ros2 run tidybot_ik motion_planner_real_node")
            return 1

        # Test reachability
        success = node.test_reachability(block_pos)

        return 0 if success else 1

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


if __name__ == '__main__':
    sys.exit(main())
