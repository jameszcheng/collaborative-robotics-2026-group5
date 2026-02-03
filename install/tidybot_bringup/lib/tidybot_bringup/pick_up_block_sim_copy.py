#!/usr/bin/env python3
"""
Pickup sequence on REAL robot using simulation-compatible wrapper topics.
NO mujoco, NO mink, NO IK.

Topics (same as your tester):
  - /right_arm/joint_cmd (Float64MultiArray) : 6 joint positions
  - /right_gripper/cmd (Float64MultiArray)  : 0=open, 1=closed, 0.5=stop

Services:
  - /right_arm/torque_enable (interbotix_xs_msgs/srv/TorqueEnable)

Subscribes:
  - /right_arm/joint_states (sensor_msgs/JointState)
"""

import sys
import time
import numpy as np

import rclpy
from rclpy.node import Node
from interbotix_xs_msgs.srv import TorqueEnable
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


# ----------------------------
# Hardcoded "same as sim" block position (meters) for reference/logging only
# (No IK is used here.)
# ----------------------------
BLOCK_POS_SIM = np.array([0.45, -0.10, 0.02])

# ----------------------------
# YOU MUST FILL THESE 3 POSES IN (6 joints):
# [waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate]
#
# Tip: Use ros2 topic echo /right_arm/joint_states while you manually place the arm
# in the desired approach/grasp/lift configurations, then paste the 6 values here.
# ----------------------------
APPROACH_POSE = [0.0, -1.0, 1.2, 0.0, 0.40, 0.0]  # above block
GRASP_POSE    = [0.0, -1.25, 1.2, 0.0, 0.55, 0.0]  # at block
LIFT_POSE     = [0.0, -0.55, 1.2, 0.0, 0.35, 0.0]  # lift up
SLEEP_POSE = [0.0, -1.80, 1.55, 0.0, 0.8, 0.0]


class PickupReal(Node):
    def __init__(self):
        super().__init__("pickup_real")

        # Match your tester: simulation-compatible arm topic
        self.right_arm_pub = self.create_publisher(Float64MultiArray, "/right_arm/joint_cmd", 10)

        # Match your tester: simulation-compatible gripper topic
        self.right_gripper_pub = self.create_publisher(Float64MultiArray, "/right_gripper/cmd", 10)

        # Match your tester: torque service
        self.right_torque_client = self.create_client(TorqueEnable, "/right_arm/torque_enable")

        # Match your tester: right arm joint states
        self.right_joint_states = None
        self.right_js_sub = self.create_subscription(
            JointState, "/right_arm/joint_states", self._right_joint_state_cb, 10
        )

        self.get_logger().info("Waiting for xs_sdk services...")
        ready = self.right_torque_client.wait_for_service(timeout_sec=5.0)
        if ready:
            self.get_logger().info("Connected to right_arm xs_sdk!")
        else:
            self.get_logger().warn("No right_arm xs_sdk service found!")

    def _right_joint_state_cb(self, msg):
        self.right_joint_states = msg

    def wait_for_joint_states(self, timeout=10.0):
        start = time.time()
        while (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.right_joint_states is not None:
                return True
        return False

    def set_torque(self, enable: bool):
        req = TorqueEnable.Request()
        req.cmd_type = "group"
        req.name = "right_arm"
        req.enable = enable

        future = self.right_torque_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        return future.result()

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
        print()
        print("=== REAL PICKUP (no IK) ===")
        print(f"Hardcoded block pos (sim reference only): {BLOCK_POS_SIM.tolist()}")
        print("Sequence: approach -> descend -> grasp -> lift")
        print()

        # Enable torque
        self.get_logger().info("Enabling torque on right_arm...")
        self.set_torque(True)
        time.sleep(0.5)

        # Open gripper before starting
        self.get_logger().info("Opening gripper...")
        self.set_right_gripper(0.0, duration=1.0)

        # Phase timings match your sim script
        # approach ~3s
        self.get_logger().info("Phase: approach")
        self.move_right_arm(APPROACH_POSE, move_time=25.0)

        # descend ~2s (approach -> grasp)
        self.get_logger().info("Phase: descend")
        # To match your sim interpolation feel but keep tester style:
        # publish one mid waypoint then the final grasp pose
        mid = (np.array(APPROACH_POSE) + np.array(GRASP_POSE)) * 0.5
        self.move_right_arm(mid.tolist(), move_time=25.0)
        self.move_right_arm(GRASP_POSE, move_time=10.0)

        # grasp ~1s
        self.get_logger().info("Phase: grasp (closing gripper)")
        self.set_right_gripper(1.0, duration=3.0)

        # lift ~2s
        self.get_logger().info("Phase: lift")
        mid2 = (np.array(GRASP_POSE) + np.array(LIFT_POSE)) * 0.5
        self.move_right_arm(mid2.tolist(), move_time=25.0)
        self.move_right_arm(LIFT_POSE, move_time=5.0)

        # go to sleep pose
        self.move_right_arm(SLEEP_POSE, move_time=5.0)


        self.get_logger().info("Phase: done")
        print("Pickup complete.\n")


def main():
    rclpy.init()
    node = PickupReal()

    try:
        print("Waiting for right arm joint states...")
        if not node.wait_for_joint_states(timeout=10.0):
            print("ERROR: No joint states received!")
            print("Make sure to launch:")
            print("  ros2 launch tidybot_bringup real.launch.py")
            return 1

        node.run_pickup()

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())