# Task 2 — Pick and Place Pipeline

## Overview

The robot interprets a verbal command (e.g. "pick up the banana and place it in the bin"), navigates to the object, picks it up with the right arm, moves the arm 25 cm to the right, and drops the object there (into a bowl or bin positioned beside the robot).

The coordinator (`task2_coordinator.py`) orchestrates the full pipeline. It does not duplicate any logic from the individual nodes — it sends signals between them at the right time.

---

## Architecture

```
                    ┌──────────────────────┐
                    │    NLP Interface      │  (separate terminal)
                    │  nlp_interface_node   │
                    │  /nlp/response (JSON) │
                    └──────────┬───────────┘
                               │
                               v
┌──────────┐  /coordinator/start   ┌──────────────────────┐
│  Manual  │ ────────────────────> │   task2_coordinator   │
│  Trigger │                       │     (state machine)   │
└──────────┘                       └──┬────┬────┬────┬────┘
                                      │    │    │    │
              /perception/            │    │    │    │  /coordinator/pickup_trigger
              target_label            │    │    │    │  /coordinator/object_pose
                 ┌────────────────────┘    │    │    └──────────────────────┐
                 v                         │    │                           v
        ┌─────────────────┐                │    │                 ┌─────────────────┐
        │ detect_object   │                │    │                 │  task2_pickup   │
        │   _real.py      │                │    │                 │  (auto_start)   │
        └────────┬────────┘                │    │                 └────────┬────────┘
                 │ /perception/            │    │                          │
                 │ object_pose             │    │  /coordinator/           │ /coordinator/
                 │ object_confidence       │    │  nav_goal                │ pickup_complete
                 └─────────────────────────┘   │                          │
                                               v                          │
                                      ┌─────────────────┐                 │
                                      │ navigate_to     │ <───────────────┘
                                      │  _object.py     │
                                      └────────┬────────┘
                                               │ /coordinator/nav_complete
                                               └──────────────────────────>
```

---

## State Machine

```
IDLE → SEARCHING → NAVIGATING → REDETECTING → PICKING_UP → DONE → IDLE
                                                    ↓
Any state ──────────────────────────── FAILED ────> IDLE
```

| State | What happens | Timeout |
|-------|-------------|---------|
| **IDLE** | Waits for a voice command or manual trigger on `/coordinator/start` | — |
| **SEARCHING** | Publishes the target label to perception; collects 3 confident detections, averages their x/y, and sends the averaged pose as a nav goal | 30 s |
| **NAVIGATING** | Waits for `nav_complete` from `navigate_to_object` (robot drives to standoff position in front of the object) | 90 s |
| **REDETECTING** | Collects 3 close-range detections at the same camera position and averages x/y/z for a precise grasp pose; if the object is not immediately visible, runs a 6-position camera pan/tilt sweep | 30 s |
| **PICKING_UP** | Publishes the refined averaged pose and triggers `task2_pickup`; waits for pickup complete signal | 60 s |
| **DONE** | Logs success, waits 2 s, returns to IDLE | — |
| **FAILED** | Emergency stops the base, logs the error, returns to IDLE | — |

---

## How Each State Works In Detail

### SEARCHING
- Publishes the object name to `/perception/target_label` (tells detection node what to look for)
- Listens to `/perception/object_pose` and `/perception/object_confidence`
- Accumulates poses with confidence ≥ 0.4 until it has 3 samples
- Averages the 3 x/y positions to reduce noise
- Publishes the averaged pose as a `PoseStamped` on `/coordinator/nav_goal`

### NAVIGATING
- `navigate_to_object.py` receives the nav goal and drives the robot to a standoff position ~0.4 m in front of the object
- The robot also aligns its heading to face the object
- When it arrives, it publishes `True` on `/coordinator/nav_complete`

### REDETECTING
- Clears previous detection data; waits for fresh detections from the new (closer) vantage point
- Timestamps are checked to ensure only post-arrival detections are used
- At each camera position: waits up to 5 s with no detection before sweeping to the next angle
- Once 3 samples are collected at a single camera position, averages x/y/z
- Publishes the refined pose on `/coordinator/object_pose` (used by pickup node)

Camera sweep positions (if object not found immediately):
1. Center, tilt 0.5 rad
2. Center, tilt 0.3 rad
3. Pan left −0.3, tilt 0.5
4. Pan right +0.3, tilt 0.5
5. Pan left −0.3, tilt 0.3
6. Pan right +0.3, tilt 0.3

### PICKING_UP
- Publishes an `Empty` message on `/coordinator/pickup_trigger`
- `task2_pickup.py` receives the trigger and the refined pose, then runs:
  1. **APPROACH** — open gripper, plan arm to approach height (17 cm above object)
  2. **DESCEND** — plan arm down to grasp height (6 cm above object)
  3. **GRASP** — close gripper with sustained force
  4. **LIFT** — plan arm up to lift height (25 cm above object)
  5. **DROP** — move arm 25 cm to the right (−x in base_link), then open gripper to release object
- Publishes `True` on `/coordinator/pickup_complete` on success, `False` on failure

> **Bowl/bin placement:** The drop position is 25 cm to the robot's right at lift height. Position the target container to the right side of the robot before triggering the task.

---

## Running Task 2

### Terminal 1 — Robot hardware + motion planner
```bash
ros2 launch tidybot_bringup real.launch.py use_planner:=true
```

### Terminal 2 — Full Task 2 pipeline
```bash
ros2 launch tidybot_bringup task2.launch.py
```

### Terminal 3 — Voice input (optional)
```bash
ros2 run tidybot_control nlp_interface_node
```

### Manual trigger (no voice needed)
```bash
ros2 topic pub /coordinator/start std_msgs/String "data: banana" --once
```

---

## Voice Command Flow

The NLP node uses a **two-step** confirmation flow before executing:

```
You:   "Pick up the banana and place it in the bin"
Robot: "I'll pick up the banana and place it in the bin. Should I go ahead?"
You:   "Yes"
       → Pipeline starts (IDLE → SEARCHING)
```

Valid objects: **`apple`**, **`banana`**
Valid targets: **`bin`**, **`bowl`**, **`table`**

Natural phrasing works — the NLP node (Gemini-backed) handles variations like:
- "grab the banana and put it in the bowl"
- "move the apple to the bin"
- "pick up the banana"

---

## Monitoring

```bash
# Watch state transitions in real time
ros2 topic echo /coordinator/status

# Individual signals
ros2 topic echo /coordinator/nav_complete
ros2 topic echo /coordinator/pickup_complete

# Object detections
ros2 topic echo /perception/object_pose
ros2 topic echo /perception/object_confidence
```

The status topic publishes strings of the form `STATE_NAME:object_label`, e.g. `NAVIGATING:banana`.

---

## Launch Parameters

```bash
ros2 launch tidybot_bringup task2.launch.py \
    target_label:=banana \      # default object for detection node
    standoff_dist:=0.35 \       # metres to stop from object (default 0.4)
    detect_timeout:=30.0 \      # seconds to find object (default 30)
    nav_timeout:=90.0 \         # seconds to navigate to object (default 90)
    pickup_timeout:=60.0 \      # seconds to complete pickup+drop (default 60)
    min_confidence:=0.4         # minimum YOLO detection confidence (default 0.4)
```

---

## Difference from Task 1

| | Task 1 | Task 2 |
|---|---|---|
| After pickup | Returns robot to start position | Drops object 25 cm to the right |
| Arm state after | Holding object, arm lifted | Gripper open, object released |
| Extra state | `RETURNING` | — |
| Launch file | `task1.launch.py` | `task2.launch.py` |
| Coordinator | `task1_coordinator.py` | `task2_coordinator.py` |
| Pickup node | `task1_pickup.py` | `task2_pickup.py` |

---

## Topics Reference

### Coordinator Subscribes To
| Topic | Type | Source | Used In State |
|-------|------|--------|---------------|
| `/nlp/response` | String (JSON) | NLP node | IDLE |
| `/coordinator/start` | String | Manual trigger | IDLE |
| `/perception/object_pose` | PoseStamped | Detection node | SEARCHING, REDETECTING |
| `/perception/object_confidence` | Float32 | Detection node | SEARCHING, REDETECTING |
| `/coordinator/nav_complete` | Bool | Navigation node | NAVIGATING |
| `/coordinator/pickup_complete` | Bool | Pickup node | PICKING_UP |

### Coordinator Publishes
| Topic | Type | Target | Sent In State |
|-------|------|--------|---------------|
| `/perception/target_label` | String | Detection node | SEARCHING (start) |
| `/coordinator/status` | String | Monitoring | All states |
| `/coordinator/nav_goal` | PoseStamped | Navigation node | SEARCHING (end) |
| `/coordinator/object_pose` | PoseStamped | Pickup node | REDETECTING (end) |
| `/coordinator/pickup_trigger` | Empty | Pickup node | REDETECTING (end) |
| `/camera/pan_tilt_cmd` | Float64MultiArray | Camera | REDETECTING (sweep) |
| `/cmd_vel` | Twist (zero) | Base | FAILED (e-stop) |

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| "No object detected after 30s" | Detection node not running or camera can't see object | Check `detect_object_real.py` is running; move object into camera FOV |
| "Navigation timed out after 90s" | Nav node stuck or odometry drift | Check `navigate_to_object.py` output; verify `/odom` is publishing |
| "Re-detection timed out after 30s" | Object moved or not visible from standoff | Reduce `standoff_dist`; check camera tilt angle |
| "Pickup failed" | Object out of arm reach or IK failure | Verify object is within arm workspace; check object pose values |
| Object dropped in wrong place | Drop offset doesn't align with bowl | Reposition bowl to robot's right side, ~25 cm away |
| Coordinator ignores voice trigger | Already in a non-IDLE state | Wait for current task to complete or restart Terminal 2 |
