# navigate_to_object — Testing Guide

Bridges the audio/NLP pipeline with base navigation. The `navigate_to_object` node launches automatically with `real.launch.py`.

## How It Works

```
Voice/text → nlp_interface_node
    → /nlp/response            → navigate_to_object  (triggers WAITING_FOR_POSE)
    → /perception/target_label → detect_object_real  (tells YOLO what to look for)

detect_object_real (YOLO + depth camera + TF)
    → /perception/object_pose  → navigate_to_object  (locks goal, starts driving)

navigate_to_object → /cmd_vel → robot moves to standoff position
```

## Full Pipeline with NLP + Real Object (5 terminals)

**Before starting:** Point the camera at the object (e.g. banana) and leave it in view.

**Terminal 1 — Launch hardware:**
```bash
cd ros2_ws && source setup_env.bash
ros2 launch tidybot_bringup real.launch.py
```

**Terminal 2 — Reset odometry origin to current robot position:**
```bash
cd ros2_ws && source setup_env.bash
python3 src/tidybot_bringup/scripts/movement_4.py --ros-args -p mode:=reset_origin -p robot:=real
```
> Do this once after hardware is up, before starting navigation. Sets the robot's
> current position as the odom origin so goal coordinates are relative to here.

**Terminal 3 — Perception (YOLO object detection):**
```bash
cd ros2_ws && source setup_env.bash
ros2 run tidybot_bringup detect_object_real.py --ros-args -p target_label:=banana
```
> The NLP node will update the target label at runtime via `/perception/target_label`,
> so the initial `target_label` param only matters if you're not using NLP.

**Terminal 4 — NLP (interactive terminal, type or use voice):**
```bash
cd ros2_ws && source setup_env.bash
ros2 run tidybot_control nlp_interface_node
```
Then say or type something like: **"go pick up the banana"**
The NLP parses this as `type: command, object: banana`, triggering navigation.

**Terminal 5 — Monitor navigation:**
```bash
cd ros2_ws && source setup_env.bash
ros2 topic echo /navigation/status
```

> **Tip:** After launching the perception node, wait until you see the log line:
> `Camera intrinsics: fx=... fy=...` — this confirms the depth-to-pose pipeline is
> ready. Then give YOLO a few seconds to detect the object before issuing the NLP
> command. The default `pose_timeout` is only 3 seconds.

---

## Manual Trigger (no NLP/perception needed)

Useful for testing navigation in isolation:

```bash
# Fake NLP command
ros2 topic pub --once /nlp/response std_msgs/String \
  '{"data": "{\"type\": \"command\", \"object\": \"cup\"}"}'

# Fake object pose (in odom frame)
ros2 topic pub --once /perception/object_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: "odom"}, pose: {position: {x: 1.5, y: 0.3, z: 0.0}, orientation: {w: 1.0}}}'
```

---

## Prerequisites

- **Odom**: `/odom` must be publishing (phoenix6_base_node or static transform)
- **Camera**: RGB + aligned depth streams from RealSense
- **TF chain**: `odom → base_link → ... → camera_link → camera_color_optical_frame`
  - `odom → base_link`: published by phoenix6_base_node
  - `base_link → camera_link`: published by robot_state_publisher (needs pan-tilt joint states)
  - `camera_link → camera_color_optical_frame`: published by RealSense driver (`publish_tf: true`)
- **YOLO**: `ultralytics` package installed (`uv add ultralytics`)
- **Gemini API key**: set `GEMINI_API_KEY` env var for NLP

---

## Parameter Tuning

| Parameter | Default | Notes |
|-----------|---------|-------|
| `standoff_dist` | 0.5 | Distance (m) from object to stop |
| `goal_tolerance` | 0.08 | Position error (m) to declare "arrived" |
| `yaw_tolerance` | 8.0 | Heading error (deg) to declare "aligned" |
| `kp` | 1.0 | Proportional gain |
| `max_v` | 0.2 | Max linear speed (m/s). Keep <= 0.3 for safety |
| `max_omega` | 2.0 | Max rotation speed (rad/s) |
| `pose_timeout` | 15.0 | Seconds to wait for object pose before FAILED |
| `nav_timeout` | 60.0 | Seconds before giving up on navigation |

---

## State Machine

```
IDLE
  → (NLP type=="command") → WAITING_FOR_POSE
      → (pose received)   → NAVIGATING
      → (pose_timeout)    → FAILED → IDLE
          → (within goal_tolerance) → ALIGNING
          → (nav_timeout)           → FAILED → IDLE
              → (within yaw_tolerance) → DONE → IDLE
```

## Topics

| Topic | Direction | Type | Description |
|-------|-----------|------|-------------|
| `/nlp/response` | Sub | std_msgs/String (JSON) | Trigger on `type=="command"` |
| `/perception/object_pose` | Sub | geometry_msgs/PoseStamped | Object position in odom frame |
| `/odom` | Sub | nav_msgs/Odometry | Robot pose |
| `/cmd_vel` | Pub | geometry_msgs/Twist | Velocity commands |
| `/navigation/status` | Pub | std_msgs/String | Current state |
| `/navigation/goal_pose` | Pub | geometry_msgs/PoseStamped | Standoff goal (RViz debug) |

## Troubleshooting

**No object pose published (navigate times out):**
- Check detector is running: `ros2 topic echo /perception/object_found`
- Check TF chain: `ros2 run tf2_tools view_frames` then open `frames.pdf`
- Check depth stream: `ros2 topic hz /camera/aligned_depth_to_color/image_raw`

**Robot doesn't move:**
- Check odom: `ros2 topic echo /odom`
- Check cmd_vel is being sent: `ros2 topic echo /cmd_vel`
- Check navigation status: `ros2 topic echo /navigation/status`
