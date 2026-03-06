# navigate_to_object — Testing Guide

Bridges the audio/NLP pipeline with base navigation.

## Full Pipeline (4 terminals)

**Terminal 1 — Launch hardware:**
```bash
cd ros2_ws && source setup_env.bash
ros2 launch tidybot_bringup real.launch.py use_navigate_to_object:=true
```

**Terminal 2 — NLP + perception nodes (already working pipeline):**
```bash
cd ros2_ws && source setup_env.bash
# Launch your NLP and perception/detect_object nodes as usual
```

**Terminal 3 — Monitor navigation:**
```bash
source ros2_ws/install/setup.bash
ros2 topic echo /navigation/status
# In a second pane:
ros2 topic echo /navigation/goal_pose
```

**Terminal 4 — Send test commands:**
```bash
source ros2_ws/install/setup.bash
# See manual trigger section below
```

---

## Manual Trigger (no NLP node needed)

Publish a fake NLP command directly:
```bash
ros2 topic pub --once /nlp/response std_msgs/String \
  '{"data": "{\"type\": \"command\", \"object\": \"cup\"}"}'
```

Publish a fake object pose (in odom frame):
```bash
ros2 topic pub --once /perception/object_pose geometry_msgs/PoseStamped \
  '{header: {frame_id: "odom"}, pose: {position: {x: 1.5, y: 0.3, z: 0.0}, orientation: {w: 1.0}}}'
```

---

## Standalone (without launch file)

```bash
cd ros2_ws && source setup_env.bash
ros2 run tidybot_bringup navigate_to_object.py \
  --ros-args -p robot:=real -p standoff_dist:=0.5 -p kp:=1.2
```

---

## Colcon Build

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tidybot_bringup
source install/setup.bash
```

---

## Parameter Tuning

| Parameter | Default | Notes |
|-----------|---------|-------|
| `standoff_dist` | 0.5 | Distance (m) from object to stop. Increase for more clearance. |
| `goal_tolerance` | 0.08 | Position error (m) to declare "arrived". |
| `yaw_tolerance` | 8.0 | Heading error (deg) to declare "aligned". |
| `kp` | 1.0 | Proportional gain. Increase for faster response, decrease to reduce overshoot. |
| `max_v` | 0.2 | Max linear speed (m/s). Keep ≤ 0.3 for safety. |
| `max_omega` | 2.0 | Max rotation speed (rad/s). |
| `pose_timeout` | 3.0 | Seconds to wait for first pose before FAILED. |
| `nav_timeout` | 60.0 | Seconds before giving up on navigation. |

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
