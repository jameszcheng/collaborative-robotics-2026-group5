# Task 1 — Object Retrieval Pipeline

## Overview

The robot interprets a verbal command (e.g. "locate the apple and retrieve it"), searches for the object, navigates to it, picks it up with the right arm, and returns to the starting position.

The coordinator (`task1_coordinator.py`) orchestrates the full pipeline. It does not duplicate any logic from the individual nodes — it sends signals between them at the right time.

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
│  Manual  │ ────────────────────> │   task1_coordinator   │
│  Trigger │                       │     (state machine)   │
└──────────┘                       └──┬────┬────┬────┬────┘
                                      │    │    │    │
              /perception/            │    │    │    │  /coordinator/pickup_trigger
              target_label            │    │    │    │  /coordinator/object_pose
                 ┌────────────────────┘    │    │    └──────────────────────┐
                 v                         │    │                           v
        ┌─────────────────┐                │    │                 ┌─────────────────┐
        │ detect_object   │                │    │                 │  task1_pickup   │
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
                                               │ /coordinator/return_to_origin (subscribe)
                                               └──────────────────────────────>
```

---

## State Machine

```
IDLE → SEARCHING → NAVIGATING → REDETECTING → PICKING_UP → RETURNING → DONE → IDLE
                                                                ↓
Any state ──────────────────────────────────── FAILED ────────> IDLE
```

| State | What happens | Timeout |
|-------|-------------|---------|
| **IDLE** | Waits for a voice command or manual trigger on `/coordinator/start` | — |
| **SEARCHING** | Publishes the target label to perception; collects 3 confident detections, averages their x/y, and sends the averaged pose as a nav goal | 30 s |
| **NAVIGATING** | Waits for `nav_complete` from `navigate_to_object` (robot drives to standoff position in front of the object) | 90 s |
| **REDETECTING** | Collects 3 close-range detections at the same camera position and averages x/y/z for a precise grasp pose; if the object is not immediately visible, runs a 6-position camera pan/tilt sweep | 30 s |
| **PICKING_UP** | Publishes the refined averaged pose and triggers `task1_pickup`; waits for pickup complete signal | 60 s |
| **RETURNING** | Sends a return-to-origin command to `navigate_to_object`; robot drives back to the saved start position (0, 0) | 120 s |
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
- Stamps are checked to ensure only post-arrival detections are used
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
- `task1_pickup.py` receives the trigger and the refined pose, then runs:
  1. **APPROACH** — open gripper, plan arm to approach height (17 cm above object)
  2. **DESCEND** — plan arm down to grasp height (6 cm above object)
  3. **GRASP** — close gripper with sustained force
  4. **LIFT** — plan arm up to lift height (25 cm above object)
- Publishes `True` on `/coordinator/pickup_complete` on success, `False` on failure

### RETURNING
- Resets the `nav_complete` flag
- Publishes an `Empty` message on `/coordinator/return_to_origin`
- `navigate_to_object.py` receives this and switches to `goto` mode targeting (0, 0) — the exact position where the robot started (saved when `reset_origin_on_start=True` at launch)
- When the robot reaches the origin, `navigate_to_object` publishes `nav_complete`
- Coordinator transitions to DONE

---

## Running Task 1

### Terminal 1 — Robot hardware + motion planner
```bash
ros2 launch tidybot_bringup real.launch.py use_planner:=true
```

### Terminal 2 — Full Task 1 pipeline
```bash
ros2 launch tidybot_bringup task1.launch.py
```

### Terminal 3 — Voice input (optional)
```bash
ros2 run tidybot_control nlp_interface_node
```

### Manual trigger (no voice needed)
```bash
ros2 topic pub /coordinator/start std_msgs/String "data: apple" --once
```

---

## Voice Command Flow

The NLP node uses a **two-step** confirmation flow before executing:

```
You:   "Pick up the apple"  (or "locate the apple and retrieve it")
Robot: "I'll pick up the apple. Should I go ahead?"
You:   "Yes"
       → Pipeline starts (IDLE → SEARCHING)
```

Valid objects: **`apple`**, **`banana`**

Natural phrasing works — the NLP node (Gemini-backed) handles variations like:
- "find the banana and bring it back"
- "retrieve the apple"
- "grab the banana"

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

The status topic publishes strings of the form `STATE_NAME:object_label`, e.g. `NAVIGATING:apple`.

---

## Launch Parameters

```bash
ros2 launch tidybot_bringup task1.launch.py \
    target_label:=apple \       # default object for detection node
    standoff_dist:=0.35 \       # metres to stop from object (default 0.4)
    detect_timeout:=30.0 \      # seconds to find object (default 30)
    nav_timeout:=90.0 \         # seconds to navigate to object (default 90)
    pickup_timeout:=60.0 \      # seconds to complete pickup (default 60)
    return_timeout:=120.0 \     # seconds to return to origin (default 120)
    min_confidence:=0.4         # minimum YOLO detection confidence (default 0.4)
```

---

## Origin / Return-to-Start

The origin is saved automatically when `navigate_to_object.py` starts — it locks the robot's current odometry position as (0, 0) once the robot is confirmed stationary (`reset_origin_on_start: True` in the launch file).

**Important:** The robot must be stationary when Terminal 2 is launched. The origin is stored in `~/.tidybot_origin_real.txt` and reused across restarts until reset.

To manually reset the origin (e.g. if the robot is repositioned):
```bash
ros2 run tidybot_bringup navigate_to_object.py --ros-args -p mode:=reset_origin -p robot:=real
```

---

## Topics Reference

### Coordinator Subscribes To
| Topic | Type | Source | Used In State |
|-------|------|--------|---------------|
| `/nlp/response` | String (JSON) | NLP node | IDLE |
| `/coordinator/start` | String | Manual trigger | IDLE |
| `/perception/object_pose` | PoseStamped | Detection node | SEARCHING, REDETECTING |
| `/perception/object_confidence` | Float32 | Detection node | SEARCHING, REDETECTING |
| `/coordinator/nav_complete` | Bool | Navigation node | NAVIGATING, RETURNING |
| `/coordinator/pickup_complete` | Bool | Pickup node | PICKING_UP |

### Coordinator Publishes
| Topic | Type | Target | Sent In State |
|-------|------|--------|---------------|
| `/perception/target_label` | String | Detection node | SEARCHING (start) |
| `/coordinator/status` | String | Monitoring | All states |
| `/coordinator/nav_goal` | PoseStamped | Navigation node | SEARCHING (end) |
| `/coordinator/object_pose` | PoseStamped | Pickup node | REDETECTING (end) |
| `/coordinator/pickup_trigger` | Empty | Pickup node | REDETECTING (end) |
| `/coordinator/return_to_origin` | Empty | Navigation node | PICKING_UP (end) |
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
| "Return to origin timed out" | Nav node not responding or large position drift | Check `navigate_to_object.py` is still running; restart pipeline |
| Coordinator ignores voice trigger | Already in a non-IDLE state | Wait for current task to complete or restart Terminal 2 |
| Robot doesn't return to exact start | Odometry drift during long navigation | Reset origin and run again from a closer starting position |
