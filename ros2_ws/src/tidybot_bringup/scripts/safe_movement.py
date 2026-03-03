#!/usr/bin/env python3
"""
safe_movement.py — SUPER SAFE base motion test (REAL-first, SIM-compatible) + FILE LOGGING

This version adds robust logging to disk so you can share the logs for troubleshooting.

Core behavior (same as before):
- REAL-first: uses /cmd_vel with timed, ramped commands + /odom for connectivity & reporting.
- SIM-compatible:
    * default: /base/target_pose + /base/goal_reached
    * optional: -p sim_interface:=cmd_vel to use /cmd_vel in sim too

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
TERMINAL 1 — Start the robot
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  # Simulator
  ros2 launch tidybot_bringup sim.launch.py

  # Real robot
  ros2 launch tidybot_bringup real.launch.py

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
TERMINAL 2 — SAFE BASIC TESTS
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
  # REAL
  python3 safe_movement.py --ros-args -p robot:=real -p test:=report -p report_sec:=5.0
  python3 safe_movement.py --ros-args -p robot:=real -p test:=forward  -p distance:=0.20
  python3 safe_movement.py --ros-args -p robot:=real -p test:=backward -p distance:=0.20
  python3 safe_movement.py --ros-args -p robot:=real -p test:=left     -p angle_deg:=30.0
  python3 safe_movement.py --ros-args -p robot:=real -p test:=right    -p angle_deg:=30.0

  # SIM (default: target_pose)
  python3 safe_movement.py --ros-args -p robot:=sim -p test:=forward -p distance:=0.50
  python3 safe_movement.py --ros-args -p robot:=sim -p test:=left    -p angle_deg:=90.0

  # SIM using cmd_vel (match REAL)
  python3 safe_movement.py --ros-args -p robot:=sim -p sim_interface:=cmd_vel -p test:=forward -p distance:=0.20

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
LOGGING
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
By default logs are saved to:
  ~/tidybot_logs/

You will get TWO files per run:
  1) a human-readable .log
  2) a .csv with time series samples (odom + commanded v/omega)

Useful params:
  -p save_log:=true|false
  -p log_dir:="~/tidybot_logs"
  -p log_prefix:="safe_movement"
  -p log_sample_hz:=20.0      # how often to write CSV during waits

Yaw reporting helper (NOT used for control):
  -p yaw_offset_deg:=0.0
"""

import time
import math
import os
from dataclasses import dataclass
from datetime import datetime

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


def wrap_angle(rad: float) -> float:
    return math.atan2(math.sin(rad), math.cos(rad))


def yaw_from_quat_z_w(qz: float, qw: float) -> float:
    # Matches the simple planar extraction used in the provided base scripts
    return 2.0 * math.atan2(qz, qw)


def expand_user(path: str) -> str:
    return os.path.expanduser(path)


@dataclass
class OdomState:
    x: float = 0.0
    y: float = 0.0
    yaw_raw: float = 0.0  # rad
    yaw: float = 0.0      # rad (after offset)


class SafeMovement(Node):
    def __init__(self):
        super().__init__("safe_movement")

        # ---------------- Parameters ----------------
        self.declare_parameter("robot", "real")  # real|sim (default real for safety)
        self.declare_parameter("test", "report")  # forward|backward|left|right|stop|report

        self.declare_parameter("distance", 0.20)    # m
        self.declare_parameter("angle_deg", 30.0)   # deg
        self.declare_parameter("v", 0.08)           # m/s
        self.declare_parameter("omega", 0.50)       # rad/s

        self.declare_parameter("ramp_sec", 0.25)
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("timeout_sec", 10.0)

        self.declare_parameter("auto_confirm", False)
        self.declare_parameter("report_sec", 5.0)

        # SIM interface
        self.declare_parameter("sim_interface", "target_pose")  # target_pose|cmd_vel
        self.declare_parameter("sim_send_theta_in_degrees", True)

        # Yaw reporting only
        self.declare_parameter("yaw_offset_deg", 0.0)

        # Logging
        self.declare_parameter("save_log", True)
        self.declare_parameter("log_dir", "~/tidybot_logs")
        self.declare_parameter("log_prefix", "safe_movement")
        self.declare_parameter("log_sample_hz", 20.0)

        self.robot = str(self.get_parameter("robot").value).strip().lower()
        self.test = str(self.get_parameter("test").value).strip().lower()

        self.distance = float(self.get_parameter("distance").value)
        self.angle_deg = float(self.get_parameter("angle_deg").value)
        self.v = abs(float(self.get_parameter("v").value))
        self.omega = abs(float(self.get_parameter("omega").value))

        self.ramp_sec = max(0.0, float(self.get_parameter("ramp_sec").value))
        self.publish_hz = max(5.0, float(self.get_parameter("publish_hz").value))
        self.timeout_sec = max(1.0, float(self.get_parameter("timeout_sec").value))

        self.auto_confirm = bool(self.get_parameter("auto_confirm").value)
        self.report_sec = max(0.5, float(self.get_parameter("report_sec").value))

        self.sim_interface = str(self.get_parameter("sim_interface").value).strip().lower()
        self.sim_send_theta_in_degrees = bool(self.get_parameter("sim_send_theta_in_degrees").value)

        self.yaw_offset = math.radians(float(self.get_parameter("yaw_offset_deg").value))

        self.save_log = bool(self.get_parameter("save_log").value)
        self.log_dir = expand_user(str(self.get_parameter("log_dir").value))
        self.log_prefix = str(self.get_parameter("log_prefix").value).strip()
        self.log_sample_hz = max(1.0, float(self.get_parameter("log_sample_hz").value))

        if self.robot not in ("real", "sim"):
            raise ValueError("robot must be 'real' or 'sim'")
        if self.sim_interface not in ("target_pose", "cmd_vel"):
            raise ValueError("sim_interface must be 'target_pose' or 'cmd_vel'")
        if self.test not in ("forward", "backward", "left", "right", "stop", "report"):
            raise ValueError("test must be forward/backward/left/right/stop/report")

        # ---------------- ROS I/O ----------------
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.target_pub = None
        self.goal_reached = False
        if self.robot == "sim" and self.sim_interface == "target_pose":
            self.target_pub = self.create_publisher(Pose2D, "/base/target_pose", 10)
            self.create_subscription(Bool, "/base/goal_reached", self._goal_cb, 10)

        # Odom for connectivity + logging
        self.odom_received = False
        self.odom = OdomState()
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)

        # ---------------- Logging setup ----------------
        self.session_start_wall = time.time()
        self.session_id = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = None
        self.csv_path = None
        self._log_f = None
        self._csv_f = None

        if self.save_log:
            os.makedirs(self.log_dir, exist_ok=True)
            base = f"{self.log_prefix}_{self.robot}_{self.test}_{self.session_id}"
            self.log_path = os.path.join(self.log_dir, base + ".log")
            self.csv_path = os.path.join(self.log_dir, base + ".csv")
            self._log_f = open(self.log_path, "w", encoding="utf-8")
            self._csv_f = open(self.csv_path, "w", encoding="utf-8")
            self._csv_f.write(
                "wall_time, t, phase, cmd_v, cmd_omega, x, y, yaw_raw_deg, yaw_deg, goal_reached\n"
            )
            self._log_line(f"Log file: {self.log_path}")
            self._log_line(f"CSV file: {self.csv_path}")

        # ---------------- Intro ----------------
        self._log_line("=" * 70)
        self._log_line("SAFE BASE MOVEMENT TEST — safe_movement.py (with file logging)")
        self._log_line("=" * 70)
        self._log_line(
            f"robot={self.robot.upper()} | test={self.test.upper()} | "
            f"distance={self.distance:.2f}m | angle={self.angle_deg:.1f}deg | "
            f"v={self.v:.2f}m/s | omega={self.omega:.2f}rad/s"
        )
        if self.robot == "sim":
            self._log_line(f"SIM interface: {self.sim_interface} | theta_degrees={self.sim_send_theta_in_degrees}")
        self._log_line(f"yaw_offset_deg={math.degrees(self.yaw_offset):.1f}")
        self._log_line("Safety: ramped timed motion + always STOP at end.\n")

    # ---------------- Logging helpers ----------------
    def _wall_ts(self) -> str:
        return datetime.now().isoformat(timespec="milliseconds")

    def _t(self) -> float:
        return time.time() - self.session_start_wall

    def _log_line(self, msg: str):
        # Console
        try:
            self.get_logger().info(msg)
        except Exception:
            pass
        # File
        if self._log_f is not None:
            self._log_f.write(f"[{self._wall_ts()}] {msg}\n")
            self._log_f.flush()

    def _csv_sample(self, phase: str, cmd_v: float, cmd_omega: float):
        if self._csv_f is None:
            return
        self._csv_f.write(
            f"{self._wall_ts()},{self._t():.4f},{phase},{cmd_v:.4f},{cmd_omega:.4f},"
            f"{self.odom.x:.6f},{self.odom.y:.6f},"
            f"{math.degrees(self.odom.yaw_raw):.3f},{math.degrees(self.odom.yaw):.3f},"
            f"{int(self.goal_reached)}\n"
        )
        self._csv_f.flush()

    def _close_logs(self):
        try:
            if self._log_f is not None:
                self._log_f.flush()
                self._log_f.close()
        finally:
            self._log_f = None
        try:
            if self._csv_f is not None:
                self._csv_f.flush()
                self._csv_f.close()
        finally:
            self._csv_f = None

    # ---------------- Callbacks ----------------
    def _goal_cb(self, msg: Bool):
        if msg.data:
            self.goal_reached = True

    def _odom_cb(self, msg: Odometry):
        self.odom_received = True
        self.odom.x = msg.pose.pose.position.x
        self.odom.y = msg.pose.pose.position.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        self.odom.yaw_raw = yaw_from_quat_z_w(qz, qw)
        self.odom.yaw = wrap_angle(self.odom.yaw_raw + self.yaw_offset)

    # ---------------- Utilities ----------------
    def _spin_for(self, sec: float, phase: str = "spin", cmd_v: float = 0.0, cmd_omega: float = 0.0):
        dt = 1.0 / self.log_sample_hz
        end = time.time() + sec
        next_sample = time.time()
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.time()
            if now >= next_sample:
                self._csv_sample(phase=phase, cmd_v=cmd_v, cmd_omega=cmd_omega)
                next_sample = now + dt

    def _wait_for_odom(self, timeout: float = 5.0) -> bool:
        self._log_line("Waiting for /odom ...")
        end = time.time() + timeout
        while rclpy.ok() and (not self.odom_received) and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.odom_received:
            self._log_line("ERROR: No /odom received. Is the base node running?")
            return False
        self._log_line(
            f"Connected. Start: x={self.odom.x:.3f} y={self.odom.y:.3f} "
            f"yaw_raw={math.degrees(self.odom.yaw_raw):.1f}deg yaw={math.degrees(self.odom.yaw):.1f}deg"
        )
        self._csv_sample(phase="odom_ready", cmd_v=0.0, cmd_omega=0.0)
        return True

    def stop(self):
        # Send a few zeros to be extra sure
        z = Twist()
        for _ in range(5):
            self.cmd_pub.publish(z)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(0.02)
        self._csv_sample(phase="stop", cmd_v=0.0, cmd_omega=0.0)

    def _confirm_if_needed(self):
        if self.robot == "real" and (not self.auto_confirm) and self.test not in ("stop", "report"):
            self._log_line("The robot WILL MOVE. Make sure the area is clear.")
            input("Press Enter to continue (Ctrl+C to abort)... ")

    # ---------------- SIM: target_pose path ----------------
    def _sim_target_pose(self) -> int:
        assert self.target_pub is not None
        self.goal_reached = False

        pose = Pose2D()
        if self.test == "forward":
            pose.x, pose.y, theta_deg = self.distance, 0.0, 0.0
        elif self.test == "backward":
            pose.x, pose.y, theta_deg = -self.distance, 0.0, 0.0
        elif self.test == "left":
            pose.x, pose.y, theta_deg = 0.0, 0.0, abs(self.angle_deg)
        elif self.test == "right":
            pose.x, pose.y, theta_deg = 0.0, 0.0, -abs(self.angle_deg)
        else:
            return 0

        pose.theta = float(theta_deg if self.sim_send_theta_in_degrees else math.radians(theta_deg))

        self._log_line(
            f"SIM(target_pose): publish Pose2D(x={pose.x:.2f}, y={pose.y:.2f}, theta={pose.theta:.2f})"
        )
        self.target_pub.publish(pose)
        self._csv_sample(phase="sim_publish_target", cmd_v=0.0, cmd_omega=0.0)

        timeout = time.time() + self.timeout_sec
        dt = 1.0 / self.log_sample_hz
        next_sample = time.time()
        while rclpy.ok() and time.time() < timeout and not self.goal_reached:
            rclpy.spin_once(self, timeout_sec=0.1)
            now = time.time()
            if now >= next_sample:
                self._csv_sample(phase="sim_wait_goal", cmd_v=0.0, cmd_omega=0.0)
                next_sample = now + dt

        if not self.goal_reached:
            self._log_line("WARN: SIM(target_pose) timed out waiting for /base/goal_reached")
            return 1

        self._log_line("SIM(target_pose): goal reached.")
        self._csv_sample(phase="sim_goal_reached", cmd_v=0.0, cmd_omega=0.0)
        return 0

    # ---------------- cmd_vel timed motion (REAL + optional SIM) ----------------
    def _publish_ramped(self, lin_x: float, ang_z: float, duration: float) -> None:
        dt = 1.0 / self.publish_hz
        n = max(1, int(duration / dt))

        ramp = min(self.ramp_sec, 0.5 * duration)
        ramp_steps = max(1, int(ramp / dt)) if ramp > 0 else 0

        self._log_line(f"cmd_vel: publish ramped (v={lin_x:.3f}, omega={ang_z:.3f}) for {duration:.2f}s")
        for i in range(n):
            if ramp_steps > 0 and i < ramp_steps:
                a = (i + 1) / ramp_steps
            elif ramp_steps > 0 and i > n - ramp_steps:
                a = (n - i) / ramp_steps
            else:
                a = 1.0

            msg = Twist()
            msg.linear.x = float(a * lin_x)
            msg.angular.z = float(a * ang_z)
            self.cmd_pub.publish(msg)
            self._csv_sample(phase="cmd_vel", cmd_v=msg.linear.x, cmd_omega=msg.angular.z)

            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(dt)

        self.stop()

    def _cmd_vel_motion(self) -> int:
        if self.test in ("forward", "backward"):
            if self.v < 1e-3:
                self._log_line("ERROR: v too small.")
                return 2
            t = abs(self.distance) / self.v
            t = min(t, self.timeout_sec)
            sign = 1.0 if self.test == "forward" else -1.0
            self._log_line(f"cmd_vel: {self.test} for ~{t:.2f}s at v={sign*self.v:.3f} m/s")
            self._publish_ramped(lin_x=sign * self.v, ang_z=0.0, duration=t)
            return 0

        if self.test in ("left", "right"):
            if self.omega < 1e-3:
                self._log_line("ERROR: omega too small.")
                return 2
            angle = math.radians(abs(self.angle_deg))
            t = angle / self.omega
            t = min(t, self.timeout_sec)
            sign = 1.0 if self.test == "left" else -1.0
            self._log_line(f"cmd_vel: rotate {self.test} for ~{t:.2f}s at omega={sign*self.omega:.3f} rad/s")
            self._publish_ramped(lin_x=0.0, ang_z=sign * self.omega, duration=t)
            return 0

        return 0

    # ---------------- Main run ----------------
    def run(self) -> int:
        self.stop()

        if self.robot == "real":
            if not self._wait_for_odom(timeout=5.0):
                return 2
        else:
            # still useful for logging; don't fail sim if odom isn't immediate
            self._wait_for_odom(timeout=2.0)

        x0, y0, yaw0 = self.odom.x, self.odom.y, self.odom.yaw
        yaw_raw0 = self.odom.yaw_raw

        if self.test == "report":
            self._log_line(f"REPORT: logging odom for {self.report_sec:.1f}s (no motion)")
            self._spin_for(self.report_sec, phase="report", cmd_v=0.0, cmd_omega=0.0)
            self.stop()
            return 0

        if self.test == "stop":
            self._log_line("STOP: publishing zero /cmd_vel")
            self.stop()
            return 0

        self._confirm_if_needed()

        if self.robot == "sim" and self.sim_interface == "target_pose" and self.target_pub is not None:
            rc = self._sim_target_pose()
        else:
            rc = self._cmd_vel_motion()

        self.stop()
        self._spin_for(0.2, phase="post_stop", cmd_v=0.0, cmd_omega=0.0)

        dx = self.odom.x - x0
        dy = self.odom.y - y0
        dist = math.hypot(dx, dy)
        dyaw_raw = wrap_angle(self.odom.yaw_raw - yaw_raw0)
        dyaw = wrap_angle(self.odom.yaw - yaw0)

        self._log_line("-" * 70)
        self._log_line(
            f"SUMMARY: Δx={dx:.3f} Δy={dy:.3f} dist≈{dist:.3f} m | "
            f"Δyaw_raw={math.degrees(dyaw_raw):.1f}deg Δyaw={math.degrees(dyaw):.1f}deg"
        )
        self._csv_sample(phase="summary", cmd_v=0.0, cmd_omega=0.0)

        if self.save_log:
            self._log_line(f"Saved logs:\n  {self.log_path}\n  {self.csv_path}")

        return rc


def main(args=None):
    rclpy.init(args=args)
    node = SafeMovement()
    rc = 0
    try:
        rc = node.run()
    except KeyboardInterrupt:
        node._log_line("Interrupted. Stopping robot.")
        rc = 130
    finally:
        try:
            node.stop()
        except Exception:
            pass
        try:
            node._close_logs()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()

    raise SystemExit(rc)


if __name__ == "__main__":
    main()
