#!/usr/bin/env python3
"""
TidyBot2 Pick/Place Demo (ROS2 Simulation)

Assumes the simulation is already running:

    ros2 launch tidybot_bringup sim.launch.py

This node does NOT open a MuJoCo viewer. It publishes arm + gripper commands over ROS
topics (like the other `*_sim.py` scripts). To get joint targets from a Cartesian pick/place
pose, it runs a small local IK loop using MuJoCo kinematics *offline* (no physics, no viewer).
"""

from __future__ import annotations

import os
import time
from enum import Enum, auto

import mujoco
import numpy as np
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from tidybot_msgs.msg import ArmCommand


# =============================== USER CONFIG ================================
# Which arm to use: "right" or "left"
ARM_SIDE = "left"

# Pick/place positions (meters). Defaults match `test_planner_sim.py` demo poses.
PICK_POS = np.array([0.35, -0.15, 0.0], dtype=float)
PLACE_POS = np.array([0.35, -0.05, 0.0], dtype=float)

# Fixed Z offset applied to end-effector targets (meters).
GRIPPER_Z_OFFSET = 0.03

# Heights relative to the target (meters)
APPROACH_Z = 0.10
LIFT_Z = 0.10

# Timing (seconds)
ARM_MOVE_DURATION = 2.0
SETTLE_TIME = 0.25
GRIPPER_TIME = 1.0

# IK settings
# If True, also match the gripper orientation (top-down).
USE_ORIENTATION = False
IK_MAX_ITERS = 200
IK_POS_TOL = 0.01
# Orientation tolerance (radians). Larger = accept more orientation error.
IK_ORI_TOL = 0.60
IK_DAMPING = 1e-3
IK_STEP_SCALE = 0.6

# Top-down quaternion used by `pick_box_mujoco.py`.
# IMPORTANT: that script uses Mink/MuJoCo convention (wxyz), not geometry_msgs (xyzw).
TOP_DOWN_QUAT_WXYZ = (0.7071, 0.0, 0.7071, 0.0)


class State(Enum):
    OPEN_GRIPPER = auto()
    MOVE_APPROACH_PICK = auto()
    MOVE_PICK = auto()
    CLOSE_GRIPPER = auto()
    LIFT = auto()
    MOVE_APPROACH_PLACE = auto()
    MOVE_PLACE = auto()
    RELEASE = auto()
    RETURN_HOME = auto()
    DONE = auto()


def _find_ik_model_path() -> str:
    repo_root = os.environ.get("TIDYBOT_REPO_ROOT", "")
    if repo_root:
        candidate = os.path.join(repo_root, "simulation", "assets", "mujoco", "tidybot_wx250s_bimanual.xml")
        if os.path.isfile(candidate):
            return candidate

    cur = os.path.dirname(os.path.abspath(__file__))
    for _ in range(10):
        candidate = os.path.join(cur, "simulation", "assets", "mujoco", "tidybot_wx250s_bimanual.xml")
        if os.path.isfile(candidate):
            return candidate
        cur = os.path.dirname(cur)

    raise FileNotFoundError("Could not find `simulation/assets/mujoco/tidybot_wx250s_bimanual.xml` (set $TIDYBOT_REPO_ROOT).")


def _quat_wxyz_to_rotmat(q_wxyz: tuple[float, float, float, float]) -> np.ndarray:
    w, x, y, z = q_wxyz
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ],
        dtype=float,
    )


def _rotmat_to_omega(R_err: np.ndarray) -> np.ndarray:
    return 0.5 * np.array([R_err[2, 1] - R_err[1, 2], R_err[0, 2] - R_err[2, 0], R_err[1, 0] - R_err[0, 1]])


class PickBoxSim(Node):
    def __init__(self) -> None:
        super().__init__("pick_box_sim")

        self.arm = ARM_SIDE.strip().lower()
        if self.arm not in {"left", "right"}:
            raise ValueError(f"ARM_SIDE must be 'left' or 'right', got {ARM_SIDE!r}")

        self.get_logger().info("=" * 60)
        self.get_logger().info("Pick/Place demo (ROS2 sim; topic-based)")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Arm: {self.arm}")

        # Publishers (match other sim scripts)
        self.arm_pub = self.create_publisher(ArmCommand, f"/{self.arm}_arm/cmd", 10)
        self.gripper_pub = self.create_publisher(Float64MultiArray, f"/{self.arm}_gripper/cmd", 10)

        # Joint state subscription (seed IK from current robot state)
        self._arm_joint_names = [
            f"{self.arm}_waist",
            f"{self.arm}_shoulder",
            f"{self.arm}_elbow",
            f"{self.arm}_forearm_roll",
            f"{self.arm}_wrist_angle",
            f"{self.arm}_wrist_rotate",
        ]
        self._have_arm_joint_states = False
        self._current_arm_joints: np.ndarray | None = None
        self._home_arm_joints: np.ndarray | None = None
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)

        # Offline IK model (no viewer)
        ik_model_path = _find_ik_model_path()
        self.get_logger().info(f"Loading IK model: {ik_model_path}")
        self._ik_model = mujoco.MjModel.from_xml_path(ik_model_path)
        self._ik_data = mujoco.MjData(self._ik_model)

        try:
            home_key_id = mujoco.mj_name2id(self._ik_model, mujoco.mjtObj.mjOBJ_KEY, "home")
            mujoco.mj_resetDataKeyframe(self._ik_model, self._ik_data, home_key_id)
            self.get_logger().info("IK model: loaded 'home' keyframe.")
        except Exception:
            mujoco.mj_resetData(self._ik_model, self._ik_data)
            self.get_logger().warn("IK model: no 'home' keyframe; using default state.")

        self._arm_joint_ids = [mujoco.mj_name2id(self._ik_model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in self._arm_joint_names]
        if any(jid < 0 for jid in self._arm_joint_ids):
            missing = [n for n, jid in zip(self._arm_joint_names, self._arm_joint_ids) if jid < 0]
            raise ValueError(f"IK model missing joints: {missing}")

        self._arm_qpos_adrs = [int(self._ik_model.jnt_qposadr[jid]) for jid in self._arm_joint_ids]
        self._arm_dof_adrs = [int(self._ik_model.jnt_dofadr[jid]) for jid in self._arm_joint_ids]

        self._site_name = f"{self.arm}_pinch_site"
        self._site_id = mujoco.mj_name2id(self._ik_model, mujoco.mjtObj.mjOBJ_SITE, self._site_name)
        if self._site_id < 0:
            raise ValueError(f"IK model missing site: {self._site_name}")

        # NOTE: `*_pinch_site` has no explicit orientation in the XML, so its frame
        # orientation is inherited from the parent gripper body. We match the *site*
        # orientation here.
        self._target_R = _quat_wxyz_to_rotmat(TOP_DOWN_QUAT_WXYZ)

        # Targets
        z_off = np.array([0.0, 0.0, GRIPPER_Z_OFFSET], dtype=float)
        pick = PICK_POS + z_off
        place = PLACE_POS + z_off
        self._targets = {
            "approach_pick": pick + np.array([0.0, 0.0, APPROACH_Z]),
            "pick": pick,
            "lift": pick + np.array([0.0, 0.0, LIFT_Z]),
            "approach_place": place + np.array([0.0, 0.0, APPROACH_Z]),
            "place": place,
        }

        self.get_logger().info(f"PICK_POS  (user): {PICK_POS}")
        self.get_logger().info(f"PLACE_POS (user): {PLACE_POS}")
        self.get_logger().info(f"GRIPPER_Z_OFFSET: {GRIPPER_Z_OFFSET}")
        self.get_logger().info(f"USE_ORIENTATION: {USE_ORIENTATION}")

        # State machine
        self.state = State.OPEN_GRIPPER
        self.state_start_time = time.time()
        self.wait_until_time: float | None = None
        self._after_wait_state: State | None = None
        self._state_action_sent = False

        self.timer = self.create_timer(0.05, self._tick)  # 20 Hz

    def _publish_gripper(self, closed: bool) -> None:
        msg = Float64MultiArray()
        msg.data = [1.0 if closed else 0.0]
        self.gripper_pub.publish(msg)

    def _publish_arm(self, joint_positions: np.ndarray, duration: float) -> None:
        cmd = ArmCommand()
        cmd.joint_positions = [float(x) for x in joint_positions.tolist()]
        cmd.duration = float(duration)
        self.arm_pub.publish(cmd)

    def _joint_state_cb(self, msg: JointState) -> None:
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        if all(n in name_to_idx for n in self._arm_joint_names):
            self._current_arm_joints = np.array([msg.position[name_to_idx[n]] for n in self._arm_joint_names], dtype=float)
            if not self._have_arm_joint_states:
                self._have_arm_joint_states = True
                self.get_logger().info("Received /joint_states (arm joints present). Starting demo.")
            # Capture the starting pose once, so we can return to it at the end.
            if self._home_arm_joints is None:
                self._home_arm_joints = self._current_arm_joints.copy()

    def _transition(self, new_state: State) -> None:
        self.state = new_state
        self.state_start_time = time.time()
        self._state_action_sent = False
        self.get_logger().info(f"State -> {new_state.name}")

    def _sleep_then(self, seconds: float, next_state: State) -> None:
        self.wait_until_time = time.time() + float(seconds)
        self._after_wait_state = next_state

    def _solve_ik(self, target_xyz: np.ndarray) -> tuple[bool, np.ndarray, float, float]:
        if self._current_arm_joints is None:
            return False, np.zeros(6), float("inf"), float("inf")

        # Seed state
        for adr, val in zip(self._arm_qpos_adrs, self._current_arm_joints.tolist()):
            self._ik_data.qpos[adr] = val
        mujoco.mj_forward(self._ik_model, self._ik_data)

        jacp = np.zeros((3, self._ik_model.nv), dtype=float)
        jacr = np.zeros((3, self._ik_model.nv), dtype=float)

        q = self._current_arm_joints.copy()
        for _ in range(IK_MAX_ITERS):
            mujoco.mj_forward(self._ik_model, self._ik_data)
            cur_p = self._ik_data.site_xpos[self._site_id].copy()
            pos_err_vec = target_xyz - cur_p
            pos_err = float(np.linalg.norm(pos_err_vec))

            ori_err = 0.0
            omega = None
            if USE_ORIENTATION:
                cur_R = self._ik_data.site_xmat[self._site_id].reshape(3, 3).copy()
                R_err = cur_R.T @ self._target_R
                omega = _rotmat_to_omega(R_err)
                ori_err = float(np.linalg.norm(omega))

            if pos_err < IK_POS_TOL and (not USE_ORIENTATION or ori_err < IK_ORI_TOL):
                return True, q, pos_err, ori_err

            mujoco.mj_jacSite(self._ik_model, self._ik_data, jacp, jacr, self._site_id)
            cols = self._arm_dof_adrs
            Jp = jacp[:, cols]  # 3x6

            if USE_ORIENTATION and omega is not None:
                Jr = jacr[:, cols]
                e = np.concatenate([pos_err_vec, omega], axis=0)  # 6
                J = np.vstack([Jp, Jr])  # 6x6
            else:
                e = pos_err_vec
                J = Jp  # 3x6

            JJt = J @ J.T
            dq = J.T @ np.linalg.solve(JJt + IK_DAMPING * np.eye(JJt.shape[0]), e)
            dq = IK_STEP_SCALE * dq

            for i, (jid, adr) in enumerate(zip(self._arm_joint_ids, self._arm_qpos_adrs)):
                low, high = self._ik_model.jnt_range[jid]
                q[i] = float(np.clip(q[i] + dq[i], low, high))
                self._ik_data.qpos[adr] = q[i]

        mujoco.mj_forward(self._ik_model, self._ik_data)
        cur_p = self._ik_data.site_xpos[self._site_id].copy()
        pos_err = float(np.linalg.norm(target_xyz - cur_p))
        return False, q, pos_err, float("inf")

    def _tick(self) -> None:
        now = time.time()

        # Keep gripper commanded continuously (helps robustness)
        if self.state in {State.CLOSE_GRIPPER, State.LIFT, State.MOVE_APPROACH_PLACE, State.MOVE_PLACE}:
            self._publish_gripper(closed=True)
        else:
            self._publish_gripper(closed=False)

        if not self._have_arm_joint_states:
            if int(now) % 2 == 0:
                self.get_logger().warn("Waiting for /joint_states (arm joints) before starting...")
            return

        if self.wait_until_time is not None:
            if now < self.wait_until_time:
                return
            self.wait_until_time = None
            if self._after_wait_state is not None:
                nxt = self._after_wait_state
                self._after_wait_state = None
                self._transition(nxt)
            return

        if self.state == State.OPEN_GRIPPER:
            if not self._state_action_sent:
                self._state_action_sent = True
                self._sleep_then(GRIPPER_TIME, State.MOVE_APPROACH_PICK)

        elif self.state == State.MOVE_APPROACH_PICK:
            if not self._state_action_sent:
                self._state_action_sent = True
                ok, q, pos_err, _ = self._solve_ik(self._targets["approach_pick"])
                if not ok:
                    self.get_logger().error(f"IK failed (approach_pick): pos_err={pos_err:.3f}m")
                    self._transition(State.DONE)
                    return
                self._publish_arm(q, ARM_MOVE_DURATION)
                self._sleep_then(ARM_MOVE_DURATION + SETTLE_TIME, State.MOVE_PICK)

        elif self.state == State.MOVE_PICK:
            if not self._state_action_sent:
                self._state_action_sent = True
                ok, q, pos_err, _ = self._solve_ik(self._targets["pick"])
                if not ok:
                    self.get_logger().error(f"IK failed (pick): pos_err={pos_err:.3f}m")
                    self._transition(State.DONE)
                    return
                self._publish_arm(q, ARM_MOVE_DURATION)
                self._sleep_then(ARM_MOVE_DURATION + SETTLE_TIME, State.CLOSE_GRIPPER)

        elif self.state == State.CLOSE_GRIPPER:
            if not self._state_action_sent:
                self._state_action_sent = True
                self._sleep_then(GRIPPER_TIME, State.LIFT)

        elif self.state == State.LIFT:
            if not self._state_action_sent:
                self._state_action_sent = True
                ok, q, pos_err, _ = self._solve_ik(self._targets["lift"])
                if not ok:
                    self.get_logger().error(f"IK failed (lift): pos_err={pos_err:.3f}m")
                    self._transition(State.DONE)
                    return
                self._publish_arm(q, ARM_MOVE_DURATION)
                self._sleep_then(ARM_MOVE_DURATION + SETTLE_TIME, State.MOVE_APPROACH_PLACE)

        elif self.state == State.MOVE_APPROACH_PLACE:
            if not self._state_action_sent:
                self._state_action_sent = True
                ok, q, pos_err, _ = self._solve_ik(self._targets["approach_place"])
                if not ok:
                    self.get_logger().error(f"IK failed (approach_place): pos_err={pos_err:.3f}m")
                    self._transition(State.DONE)
                    return
                self._publish_arm(q, ARM_MOVE_DURATION)
                self._sleep_then(ARM_MOVE_DURATION + SETTLE_TIME, State.MOVE_PLACE)

        elif self.state == State.MOVE_PLACE:
            if not self._state_action_sent:
                self._state_action_sent = True
                ok, q, pos_err, _ = self._solve_ik(self._targets["place"])
                if not ok:
                    self.get_logger().error(f"IK failed (place): pos_err={pos_err:.3f}m")
                    self._transition(State.DONE)
                    return
                self._publish_arm(q, ARM_MOVE_DURATION)
                self._sleep_then(ARM_MOVE_DURATION + SETTLE_TIME, State.RELEASE)

        elif self.state == State.RELEASE:
            if not self._state_action_sent:
                self._state_action_sent = True
                self._sleep_then(GRIPPER_TIME, State.RETURN_HOME)

        elif self.state == State.RETURN_HOME:
            if not self._state_action_sent:
                self._state_action_sent = True
                if self._home_arm_joints is None:
                    self.get_logger().warn("No home joint snapshot available; skipping RETURN_HOME.")
                    self._transition(State.DONE)
                    return
                self._publish_arm(self._home_arm_joints, ARM_MOVE_DURATION)
                self._sleep_then(ARM_MOVE_DURATION + SETTLE_TIME, State.DONE)

        elif self.state == State.DONE:
            return


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PickBoxSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()

