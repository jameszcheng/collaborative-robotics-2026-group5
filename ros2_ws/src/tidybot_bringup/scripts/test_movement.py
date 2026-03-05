# (Same file contents as the download — save as: test_movement.py)
#!/usr/bin/env python3
"""
test_movement.py — TidyBot2 base motion troubleshooting (SIM + REAL)

This script is meant for *basic* validation/debug:
  - forward/backward
  - rotate left/right
  - prints odom deltas + yaw (with configurable yaw_offset)

It follows the same topic split used in the provided course demos:
  SIM:  /base/target_pose (Pose2D), /base/goal_reached (Bool)
  REAL: /cmd_vel (Twist), /odom (Odometry)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
TERMINAL 1 — Start the robot
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  # Simulator
  ros2 launch tidybot_bringup sim.launch.py

  # Real robot
  ros2 launch tidybot_bringup real.launch.py

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
TERMINAL 2 — Run tests
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

  # --- Simulator (high-level target_pose) ---
  python3 test_movement.py --ros-args -p robot:=sim  -p test:=forward  -p distance:=0.5
  python3 test_movement.py --ros-args -p robot:=sim  -p test:=backward -p distance:=0.5
  python3 test_movement.py --ros-args -p robot:=sim  -p test:=left     -p angle_deg:=90.0
  python3 test_movement.py --ros-args -p robot:=sim  -p test:=right    -p angle_deg:=90.0

  # If SIM looks rotated/mirrored, try adjusting yaw_offset_deg (default -90° for sim):
  python3 test_movement.py --ros-args -p robot:=sim  -p test:=left     -p angle_deg:=90.0 -p yaw_offset_deg:=-90.0
  python3 test_movement.py --ros-args -p robot:=sim  -p test:=left     -p angle_deg:=90.0 -p yaw_offset_deg:=0.0
  python3 test_movement.py --ros-args -p robot:=sim  -p test:=left     -p angle_deg:=90.0 -p yaw_offset_deg:=90.0

  # --- Real robot (low-level cmd_vel + odom feedback) ---
  python3 test_movement.py --ros-args -p robot:=real -p test:=forward  -p distance:=0.5 -p v:=0.15
  python3 test_movement.py --ros-args -p robot:=real -p test:=backward -p distance:=0.5 -p v:=0.15
  python3 test_movement.py --ros-args -p robot:=real -p test:=left     -p angle_deg:=90.0 -p omega:=1.0
  python3 test_movement.py --ros-args -p robot:=real -p test:=right    -p angle_deg:=90.0 -p omega:=1.0

  # Optional: just print odom/yaw for ~10s (no movement)
  python3 test_movement.py --ros-args -p robot:=real -p test:=report -p report_sec:=10.0

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Notes
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
- yaw_offset_deg is applied as:
      theta_used = wrap(theta_from_odom + yaw_offset_deg)
  This is a practical shim if SIM vs REAL define "0 yaw" differently.

- In SIM, Pose2D.theta for /base/target_pose is treated as DEGREES in the
  course-provided movement script. So this file defaults to sending degrees.
"""

import time
import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


def wrap_angle(rad: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(rad), math.cos(rad))


def yaw_from_quat_z_w(qz: float, qw: float) -> float:
    """
    Minimal yaw extraction used in the provided base scripts:
      yaw = 2 * atan2(qz, qw)
    Assumes planar motion / small roll/pitch.
    """
    return 2.0 * math.atan2(qz, qw)


class TestMovement(Node):
    def __init__(self):
        super().__init__("test_movement")

        # ---------------- Parameters ----------------
        self.declare_parameter("robot", "sim")  # sim | real
        self.declare_parameter("test", "forward")  # forward|backward|left|right|report
        self.declare_parameter("distance", 0.5)  # meters (forward/back)
        self.declare_parameter("angle_deg", 90.0)  # degrees (left/right)
        self.declare_parameter("v", 0.15)  # m/s (real only)
        self.declare_parameter("omega", 1.0)  # rad/s (real only)
        self.declare_parameter("timeout_sec", 30.0)

        # SIM specifics
        self.declare_parameter("sim_send_theta_in_degrees", True)

        # Yaw alignment
        self.declare_parameter("yaw_offset_deg", -90.0)  # good default for many MuJoCo setups
        self.declare_parameter("report_sec", 10.0)

        self.robot = str(self.get_parameter("robot").value).strip().lower()
        self.test = str(self.get_parameter("test").value).strip().lower()
        self.distance = float(self.get_parameter("distance").value)
        self.angle_deg = float(self.get_parameter("angle_deg").value)
        self.v = float(self.get_parameter("v").value)
        self.omega = float(self.get_parameter("omega").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.sim_send_theta_in_degrees = bool(self.get_parameter("sim_send_theta_in_degrees").value)
        self.yaw_offset = math.radians(float(self.get_parameter("yaw_offset_deg").value))
        self.report_sec = float(self.get_parameter("report_sec").value)

        if self.robot not in ("sim", "real"):
            raise ValueError("robot must be 'sim' or 'real'")

        # For real robot, default yaw_offset should typically be 0
        if self.robot == "real" and abs(self.get_parameter("yaw_offset_deg").value + 90.0) < 1e-6:
            # Only override if user didn't explicitly change it from the sim-like default.
            self.yaw_offset = 0.0

        # ---------------- ROS I/O ----------------
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.target_pub = None
        self.goal_reached = False
        self.goal_sub = None
        if self.robot == "sim":
            self.target_pub = self.create_publisher(Pose2D, "/base/target_pose", 10)
            self.goal_sub = self.create_subscription(Bool, "/base/goal_reached", self._goal_cb, 10)

        self.odom_received = False
        self.raw_x = 0.0
        self.raw_y = 0.0
        self.raw_yaw = 0.0  # yaw from odom (before yaw_offset)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0  # yaw used by script (after yaw_offset)

        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)

        self.get_logger().info("=" * 60)
        self.get_logger().info("TidyBot2 Base Test — test_movement.py")
        self.get_logger().info("=" * 60)
        self.get_logger().info(
            f"robot={self.robot.upper()} | test={self.test.upper()} | "
            f"distance={self.distance:.2f}m | angle={self.angle_deg:.1f}deg | "
            f"yaw_offset={math.degrees(self.yaw_offset):.1f}deg"
        )

    def _goal_cb(self, msg: Bool):
        if msg.data:
            self.goal_reached = True

    def _odom_cb(self, msg: Odometry):
        self.odom_received = True
        self.raw_x = msg.pose.pose.position.x
        self.raw_y = msg.pose.pose.position.y

        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.raw_yaw = yaw_from_quat_z_w(qz, qw)

        self.x = self.raw_x
        self.y = self.raw_y
        self.yaw = wrap_angle(self.raw_yaw + self.yaw_offset)

    def _wait_for_odom(self, timeout: float = 5.0) -> bool:
        self.get_logger().info("Waiting for /odom ...")
        t_end = time.time() + timeout
        while rclpy.ok() and (not self.odom_received) and time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.odom_received:
            self.get_logger().error("No /odom received. Is the base node running?")
            return False
        self.get_logger().info(
            f"Connected. Start raw pose: x={self.raw_x:.3f} y={self.raw_y:.3f} "
            f"raw_yaw={math.degrees(self.raw_yaw):.1f}deg | "
            f"yaw_used={math.degrees(self.yaw):.1f}deg"
        )
        return True

    def stop(self):
        self.cmd_vel_pub.publish(Twist())

    # ---------------- SIM tests ----------------
    def _sim_send_target(self, x: float, y: float, theta_deg: float):
        assert self.target_pub is not None
        pose = Pose2D()
        pose.x = float(x)
        pose.y = float(y)
        pose.theta = float(theta_deg if self.sim_send_theta_in_degrees else math.radians(theta_deg))
        self.target_pub.publish(pose)

    def run_sim(self) -> int:
        # Try to get odom for logging (not strictly required)
        self._wait_for_odom(timeout=2.0)

        # Drain any latched/old goal_reached
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.05)

        self.goal_reached = False
        t0 = time.time()

        if self.test == "forward":
            self.get_logger().info(f"SIM: target forward {self.distance:.2f} m")
            self._sim_send_target(self.distance, 0.0, 0.0)
        elif self.test == "backward":
            self.get_logger().info(f"SIM: target backward {self.distance:.2f} m")
            self._sim_send_target(-self.distance, 0.0, 0.0)
        elif self.test == "left":
            self.get_logger().info(f"SIM: target rotate left {self.angle_deg:.1f} deg")
            self._sim_send_target(0.0, 0.0, self.angle_deg)
        elif self.test == "right":
            self.get_logger().info(f"SIM: target rotate right {self.angle_deg:.1f} deg")
            self._sim_send_target(0.0, 0.0, -self.angle_deg)
        elif self.test == "report":
            self.get_logger().info(f"SIM: report-only for {self.report_sec:.1f}s (no command)")
        else:
            self.get_logger().error("Unknown test. Use: forward, backward, left, right, report")
            return 2

        # Wait loop
        timeout = t0 + self.timeout_sec
        last_print = 0.0
        while rclpy.ok() and time.time() < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.test == "report":
                if time.time() - t0 >= self.report_sec:
                    break
            else:
                if self.goal_reached:
                    break

            # periodic logging
            if time.time() - last_print > 0.5 and self.odom_received:
                last_print = time.time()
                self.get_logger().info(
                    f"  odom: x={self.raw_x:.3f} y={self.raw_y:.3f} "
                    f"raw_yaw={math.degrees(self.raw_yaw):.1f}deg | "
                    f"yaw_used={math.degrees(self.yaw):.1f}deg"
                )

        if self.test != "report" and not self.goal_reached:
            self.get_logger().warn("SIM: timed out waiting for /base/goal_reached")
            return 1

        self.get_logger().info("SIM: done.")
        return 0

    # ---------------- REAL tests ----------------
    def run_real(self) -> int:
        if not self._wait_for_odom(timeout=5.0):
            return 2

        x0, y0, yaw0 = self.x, self.y, self.yaw
        raw_x0, raw_y0, raw_yaw0 = self.raw_x, self.raw_y, self.raw_yaw

        cmd = Twist()
        t0 = time.time()
        timeout = t0 + self.timeout_sec

        if self.test == "forward" or self.test == "backward":
            sign = 1.0 if self.test == "forward" else -1.0
            cmd.linear.x = sign * abs(self.v)
            self.get_logger().info(
                f"REAL: {self.test} at v={cmd.linear.x:.2f} m/s until distance={self.distance:.2f} m "
                f"(timeout {self.timeout_sec:.1f}s)"
            )

            while rclpy.ok() and time.time() < timeout:
                travelled = math.hypot(self.x - x0, self.y - y0)
                if travelled >= self.distance:
                    break
                self.cmd_vel_pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.05)

            self.stop()
            rclpy.spin_once(self, timeout_sec=0.2)

            travelled = math.hypot(self.x - x0, self.y - y0)
            self.get_logger().info(
                f"REAL: travelled={travelled:.3f} m | "
                f"start(x,y)=({x0:.3f},{y0:.3f}) end(x,y)=({self.x:.3f},{self.y:.3f})"
            )

        elif self.test == "left" or self.test == "right":
            sign = 1.0 if self.test == "left" else -1.0
            cmd.angular.z = sign * abs(self.omega)
            target = math.radians(abs(self.angle_deg))

            self.get_logger().info(
                f"REAL: rotate {self.test} at omega={cmd.angular.z:.2f} rad/s until "
                f"angle={self.angle_deg:.1f} deg (timeout {self.timeout_sec:.1f}s)"
            )

            rotated = 0.0
            prev = self.yaw
            while rclpy.ok() and time.time() < timeout:
                d = wrap_angle(self.yaw - prev)
                rotated += abs(d)
                prev = self.yaw

                if rotated >= target:
                    break

                self.cmd_vel_pub.publish(cmd)
                rclpy.spin_once(self, timeout_sec=0.05)

            self.stop()
            rclpy.spin_once(self, timeout_sec=0.2)

            self.get_logger().info(
                f"REAL: rotated≈{math.degrees(rotated):.1f} deg | "
                f"start_yaw={math.degrees(yaw0):.1f}deg end_yaw={math.degrees(self.yaw):.1f}deg"
            )

        elif self.test == "report":
            self.get_logger().info(f"REAL: report-only for {self.report_sec:.1f}s (no command)")
            while rclpy.ok() and time.time() - t0 < self.report_sec:
                rclpy.spin_once(self, timeout_sec=0.1)
                self.get_logger().info(
                    f"  odom: x={self.raw_x:.3f} y={self.raw_y:.3f} "
                    f"raw_yaw={math.degrees(self.raw_yaw):.1f}deg | "
                    f"yaw_used={math.degrees(self.yaw):.1f}deg"
                )
            self.stop()

        else:
            self.get_logger().error("Unknown test. Use: forward, backward, left, right, report")
            return 2

        # Extra summary (raw)
        dx_raw = self.raw_x - raw_x0
        dy_raw = self.raw_y - raw_y0
        dyaw_raw = wrap_angle(self.raw_yaw - raw_yaw0)
        self.get_logger().info(
            f"RAW SUMMARY: Δx={dx_raw:.3f} Δy={dy_raw:.3f} Δyaw={math.degrees(dyaw_raw):.1f}deg "
            f"(this is before yaw_offset)"
        )

        if time.time() >= timeout and self.test != "report":
            self.get_logger().warn("REAL: timed out before reaching target.")
            return 1

        self.get_logger().info("REAL: done.")
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = TestMovement()
    rc = 0
    try:
        if node.robot == "sim":
            rc = node.run_sim()
        else:
            rc = node.run_real()
    except KeyboardInterrupt:
        node.get_logger().warn("Interrupted by user.")
        rc = 130
    finally:
        try:
            node.stop()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(rc)


if __name__ == "__main__":
    main()