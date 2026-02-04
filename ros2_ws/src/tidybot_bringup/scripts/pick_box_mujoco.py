#!/usr/bin/env python3
"""
TidyBot2 Block Pickup Demo - Using Mink for Inverse Kinematics

Minimal pick-and-place-style demo using a single arm with top-down grasping.

This script intentionally keeps things simple:
- Loads the standard bimanual TidyBot MuJoCo scene (robot + floor; no objects required)
- Drives the selected arm end-effector through hard-coded waypoints (pick pose -> place pose)
- Opens/closes the gripper to mimic pick/place timing
"""

from __future__ import annotations

import os
import time
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np

import mink


# ============================== USER CONFIG =================================
# World frame (meters).
PICK_POS = np.array([0.0, -0.4, 0.0], dtype=float)
PLACE_POS = np.array([0.2, -0.4, 0.0], dtype=float)

# "right" or "left"
ARM_SIDE = "left"

# If True, force both finger joints to track the same commanded value. This makes the
# fingers move together even if physics/contact would otherwise cause asymmetry.
COUPLE_GRIPPER_FINGERS = True

# Fixed offset applied to the end-effector target (meters). Positive Z keeps the
# gripper slightly above the nominal pick/place point for top-down picking.
GRIPPER_Z_OFFSET = 0.06

# Heights relative to the pick/place targets (meters).
APPROACH_HEIGHT = 0.15
LIFT_HEIGHT = 0.20

# Top-down gripper orientation quaternion in Mink/MuJoCo convention (wxyz).
TOP_DOWN_QUAT_WXYZ = np.array([0.7071, 0.0, 0.7071, 0.0], dtype=float)


def _find_mujoco_model_path() -> Path:
    """
    Find a MuJoCo model XML to load.

    MuJoCo assets live under `simulation/assets/mujoco/` at the repo root.
    """
    script_dir = Path(__file__).resolve().parent

    # Preferred location (repo root known via env var)
    repo_root = os.environ.get("TIDYBOT_REPO_ROOT")
    if repo_root:
        candidate = Path(repo_root) / "simulation/assets/mujoco/scene_wx250s_bimanual.xml"
        if candidate.is_file():
            return candidate.resolve()

    # Search upward for the `simulation/assets/mujoco` directory.
    search_path = script_dir
    for _ in range(12):
        candidate = search_path / "simulation/assets/mujoco/scene_wx250s_bimanual.xml"
        if candidate.is_file():
            return candidate.resolve()
        search_path = search_path.parent

    raise FileNotFoundError(
        "Could not find MuJoCo model `scene_wx250s_bimanual.xml`.\n"
        "Expected it at `simulation/assets/mujoco/scene_wx250s_bimanual.xml` (using $TIDYBOT_REPO_ROOT or upward search)."
    )


def main():
    # ==========================================================================
    # Load Model
    # ==========================================================================
    model_path = _find_mujoco_model_path()
    model = mujoco.MjModel.from_xml_path(str(model_path))
    data = mujoco.MjData(model)

    # ==========================================================================
    # Get IDs / Names
    # ==========================================================================
    arm_side = ARM_SIDE.strip().lower()
    if arm_side not in {"left", "right"}:
        raise ValueError(f"ARM_SIDE must be 'left' or 'right', got: {ARM_SIDE!r}")

    # Arm joints (for IK)
    arm_joints = [
        f"{arm_side}_waist",
        f"{arm_side}_shoulder",
        f"{arm_side}_elbow",
        f"{arm_side}_forearm_roll",
        f"{arm_side}_wrist_angle",
        f"{arm_side}_wrist_rotate",
    ]

    # Get joint qpos addresses
    joint_qpos_addrs = {}
    for jname in arm_joints:
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, jname)
        if jid < 0:
            raise ValueError(f"Joint not found: {jname}")
        joint_qpos_addrs[jname] = model.jnt_qposadr[jid]

    # Arm actuators
    arm_actuators = {}
    for jname in arm_joints:
        aname = jname  # actuator name matches joint name
        aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, aname)
        if aid < 0:
            raise ValueError(f"Actuator not found: {aname}")
        arm_actuators[jname] = aid

    # Gripper finger actuators (prismatic fingers)
    left_finger_name = f"{arm_side}_left_finger"
    right_finger_name = f"{arm_side}_right_finger"
    left_finger_ctrl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, left_finger_name)
    right_finger_ctrl = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, right_finger_name)
    if left_finger_ctrl < 0 or right_finger_ctrl < 0:
        raise ValueError(f"Gripper actuators not found: {left_finger_name}, {right_finger_name}")

    # Gripper command ranges (meters) from the model.
    # Use these to set a valid initial pose (otherwise the fingers can start interpenetrating geometry).
    ctrlrange = model.actuator_ctrlrange
    gripper_closed = float(min(ctrlrange[left_finger_ctrl, 0], ctrlrange[left_finger_ctrl, 1]))
    gripper_open = float(max(ctrlrange[left_finger_ctrl, 0], ctrlrange[left_finger_ctrl, 1]))

    # Finger joint state addresses (used for optional hard-coupling)
    left_finger_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, left_finger_name)
    right_finger_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, right_finger_name)
    if left_finger_joint_id < 0 or right_finger_joint_id < 0:
        raise ValueError(f"Gripper joints not found: {left_finger_name}, {right_finger_name}")
    left_finger_qpos_adr = int(model.jnt_qposadr[left_finger_joint_id])
    right_finger_qpos_adr = int(model.jnt_qposadr[right_finger_joint_id])
    left_finger_qvel_adr = int(model.jnt_dofadr[left_finger_joint_id])
    right_finger_qvel_adr = int(model.jnt_dofadr[right_finger_joint_id])

    # End-effector site (for position error)
    pinch_site_name = f"{arm_side}_pinch_site"
    pinch_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, pinch_site_name)
    if pinch_site_id < 0:
        raise ValueError(f"Site not found: {pinch_site_name}")

    # ==========================================================================
    # Initialize
    # ==========================================================================
    # Prefer the `home` keyframe for a stable initial pose (includes gripper opening).
    # This avoids starting the finger joints at 0, which can self-collide and appear "stuck".
    try:
        home_key_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_KEY, "home")
        mujoco.mj_resetDataKeyframe(model, data, home_key_id)
    except Exception:
        mujoco.mj_resetData(model, data)
        # Force grippers to a sane open pose.
        data.qpos[left_finger_qpos_adr] = gripper_open
        data.qpos[right_finger_qpos_adr] = gripper_open

    # Also command grippers open initially (position actuators)
    data.ctrl[left_finger_ctrl] = gripper_open
    data.ctrl[right_finger_ctrl] = gripper_open
    mujoco.mj_forward(model, data)

    # ==========================================================================
    # Setup Mink IK
    # ==========================================================================
    configuration = mink.Configuration(model)
    configuration.update(data.qpos)

    # End-effector task for right arm
    ee_task = mink.FrameTask(
        frame_name=pinch_site_name,
        frame_type="site",
        position_cost=1.0,
        orientation_cost=1.0,
    )

    # ==========================================================================
    # Define simple waypoints (world frame)
    # ==========================================================================
    pick_pos = PICK_POS.copy()
    place_pos = PLACE_POS.copy()

    gripper_offset = np.array([0.0, 0.0, GRIPPER_Z_OFFSET], dtype=float)
    pick_target = pick_pos + gripper_offset
    place_target = place_pos + gripper_offset

    approach_pos = pick_target + np.array([0.0, 0.0, APPROACH_HEIGHT])
    grasp_pos = pick_target
    lift_pos = pick_target + np.array([0.0, 0.0, LIFT_HEIGHT])

    place_approach = place_target + np.array([0.0, 0.0, APPROACH_HEIGHT])
    place_release = place_target
    retreat_pos = place_target + np.array([0.0, 0.0, LIFT_HEIGHT])

    # ==========================================================================
    # Run Simulation
    # ==========================================================================
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Camera setup
        viewer.cam.distance = 1.2
        viewer.cam.elevation = -30
        viewer.cam.azimuth = 150
        viewer.cam.lookat[:] = [0.4, -0.1, 0.2]

        # Keep the viewer uncluttered for demos.
        viewer.opt.frame = mujoco.mjtFrame.mjFRAME_NONE

        phase = "approach"
        phase_start_time = time.time()

        def set_phase(new_phase: str) -> None:
            nonlocal phase, phase_start_time
            phase = new_phase
            phase_start_time = time.time()

        # IK parameters
        dt = model.opt.timestep
        ik_steps_per_sim_step = 1

        while viewer.is_running():
            step_start = time.time()
            phase_elapsed = time.time() - phase_start_time
            ee_pos = data.site_xpos[pinch_site_id].copy()

            # ==================================================================
            # State Machine
            # ==================================================================
            # Default gripper command (can be overridden per phase)
            desired_gripper = gripper_open

            if phase == "approach":
                target_pos = approach_pos
                if np.linalg.norm(ee_pos - approach_pos) < 0.02 or phase_elapsed > 3.0:
                    set_phase("descend")

            elif phase == "descend":
                # Interpolate from approach to grasp
                t = min(phase_elapsed / 2.0, 1.0)
                target_pos = approach_pos + t * (grasp_pos - approach_pos)
                if np.linalg.norm(ee_pos - grasp_pos) < 0.005 or phase_elapsed > 5.0:
                    set_phase("grasp")

            elif phase == "grasp":
                target_pos = grasp_pos
                # Close gripper gradually
                gripper_progress = min(phase_elapsed / 1.0, 1.0)
                desired_gripper = gripper_open + (gripper_closed - gripper_open) * gripper_progress
                if phase_elapsed > 1.2:
                    set_phase("lift")

            elif phase == "lift":
                # Interpolate from grasp to lift
                t = min(phase_elapsed / 2.0, 1.0)
                target_pos = grasp_pos + t * (lift_pos - grasp_pos)
                desired_gripper = gripper_closed
                if np.linalg.norm(ee_pos - lift_pos) < 0.02 or phase_elapsed > 2.5:
                    set_phase("move_to_place")

            elif phase == "move_to_place":
                # Move above the place location
                t = min(phase_elapsed / 2.0, 1.0)
                target_pos = lift_pos + t * (place_approach - lift_pos)
                desired_gripper = gripper_closed
                if np.linalg.norm(ee_pos - place_approach) < 0.03 or phase_elapsed > 3.0:
                    set_phase("lower_to_place")

            elif phase == "lower_to_place":
                # Lower down to release height
                t = min(phase_elapsed / 2.0, 1.0)
                target_pos = place_approach + t * (place_release - place_approach)
                desired_gripper = gripper_closed
                if np.linalg.norm(ee_pos - place_release) < 0.01 or phase_elapsed > 3.0:
                    set_phase("release")

            elif phase == "release":
                target_pos = place_release
                # Open gripper gradually
                gripper_progress = min(phase_elapsed / 0.8, 1.0)
                desired_gripper = gripper_closed + (gripper_open - gripper_closed) * gripper_progress
                if phase_elapsed > 1.0:
                    set_phase("retreat")

            elif phase == "retreat":
                t = min(phase_elapsed / 1.5, 1.0)
                target_pos = place_release + t * (retreat_pos - place_release)
                desired_gripper = gripper_open
                if np.linalg.norm(ee_pos - retreat_pos) < 0.03 or phase_elapsed > 2.0:
                    set_phase("done")

            else:  # done
                target_pos = retreat_pos
                desired_gripper = gripper_open

            # ==================================================================
            # Solve IK
            # ==================================================================
            # Update configuration with current qpos
            configuration.update(data.qpos)

            # Set target pose (position + top-down orientation)
            target_pose = mink.SE3.from_rotation_and_translation(
                rotation=mink.SO3(TOP_DOWN_QUAT_WXYZ),
                translation=target_pos,
            )
            ee_task.set_target(target_pose)

            # Solve IK
            for _ in range(ik_steps_per_sim_step):
                vel = mink.solve_ik(
                    configuration,
                    [ee_task],
                    dt=dt,
                    solver="quadprog",
                    damping=1e-3,
                )
                configuration.integrate_inplace(vel, dt)

            # Apply solved joint positions to actuators
            for jname in arm_joints:
                qpos_addr = joint_qpos_addrs[jname]
                act_id = arm_actuators[jname]
                data.ctrl[act_id] = configuration.q[qpos_addr]

            # Apply gripper command (always) AFTER arm commands, and clip to ctrlrange.
            desired_gripper = float(desired_gripper)
            desired_gripper = float(np.clip(desired_gripper, gripper_closed, gripper_open))
            data.ctrl[left_finger_ctrl] = desired_gripper
            data.ctrl[right_finger_ctrl] = desired_gripper

            # ==================================================================
            # Step Simulation
            # ==================================================================
            mujoco.mj_step(model, data)

            # Optional: hard-couple the two fingers so they move together.
            if COUPLE_GRIPPER_FINGERS:
                coupled_qpos = float(np.clip(desired_gripper, gripper_closed, gripper_open))
                data.qpos[left_finger_qpos_adr] = coupled_qpos
                data.qpos[right_finger_qpos_adr] = coupled_qpos
                data.qvel[left_finger_qvel_adr] = 0.0
                data.qvel[right_finger_qvel_adr] = 0.0
                mujoco.mj_forward(model, data)

            viewer.sync()

            # Real-time sync
            elapsed_step = time.time() - step_start
            sleep_time = dt - elapsed_step
            if sleep_time > 0:
                time.sleep(sleep_time)


if __name__ == "__main__":
    main()