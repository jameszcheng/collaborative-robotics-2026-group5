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
              target_label            │    │    │    │
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
IDLE → SEARCHING → NAVIGATING → PAUSE → PICKING_UP → RETURNING → DONE → IDLE
                                                           ↓
Any state ─────────────────────────────── FAILED ────────> IDLE
```

| State | What happens | Timeout |
|-------|-------------|---------|
| **IDLE** | Waits for a voice command or manual trigger on `/coordinator/start` | — |
| **SEARCHING** | Publishes the target label to perception; collects 3 confident detections, averages their x/y, and sends the averaged pose as a nav goal | 30 s |
| **NAVIGATING** | Waits for `nav_complete` from `navigate_to_object` (robot drives to standoff position in front of the object); on arrival tilts camera down to 0.5 rad | 90 s |
| **PAUSE** | Short settling delay (default 3 s) after navigation; stale detection data clears during this window before pickup is triggered | — |
| **PICKING_UP** | Publishes an `Empty` trigger to `task1_pickup` and waits for the pickup complete signal | 120 s |
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
- When it arrives, the coordinator tilts the camera down (0.0 pan, 0.5 rad tilt) and transitions to PAUSE
- The navigation node publishes `True` on `/coordinator/nav_complete`

### PAUSE
- A short configurable delay (default 3 s) that lets the robot settle and stale perception data age out
- No signals are sent during this state; the coordinator simply waits before triggering pickup

### PICKING_UP
- Publishes an `Empty` message on `/coordinator/pickup_trigger`
- `task1_pickup.py` receives the trigger and runs its own camera sweep + re-detection internally, then:
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
    pickup_timeout:=120.0 \     # seconds to complete pickup (default 120)
    return_timeout:=120.0 \     # seconds to return to origin (default 120)
    min_confidence:=0.4 \       # minimum YOLO detection confidence (default 0.4)
    search_samples:=3 \         # confident detections to average before navigating (default 3)
    pause_duration:=3.0         # settling delay after navigation before pickup (default 3)
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
| `/perception/object_pose` | PoseStamped | Detection node | SEARCHING |
| `/perception/object_confidence` | Float32 | Detection node | SEARCHING |
| `/coordinator/nav_complete` | Bool | Navigation node | NAVIGATING, RETURNING |
| `/coordinator/pickup_complete` | Bool | Pickup node | PICKING_UP |

### Coordinator Publishes
| Topic | Type | Target | Sent In State |
|-------|------|--------|---------------|
| `/perception/target_label` | String | Detection node | SEARCHING (start) |
| `/coordinator/status` | String | Monitoring | All states |
| `/coordinator/nav_goal` | PoseStamped | Navigation node | SEARCHING (end) |
| `/coordinator/pickup_trigger` | Empty | Pickup node | PAUSE (end) |
| `/coordinator/return_to_origin` | Empty | Navigation node | PICKING_UP (end) |
| `/camera/pan_tilt_cmd` | Float64MultiArray | Camera | NAVIGATING (end) |
| `/cmd_vel` | Twist (zero) | Base | FAILED (e-stop) |

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| "No object detected after 30s" | Detection node not running or camera can't see object | Check `detect_object_real.py` is running; move object into camera FOV |
| "Navigation timed out after 90s" | Nav node stuck or odometry drift | Check `navigate_to_object.py` output; verify `/odom` is publishing |
| "Pickup failed" | Object out of arm reach or IK failure | Verify object is within arm workspace; check object pose values |
| "Return to origin timed out" | Nav node not responding or large position drift | Check `navigate_to_object.py` is still running; restart pipeline |
| Coordinator ignores voice trigger | Already in a non-IDLE state | Wait for current task to complete or restart Terminal 2 |
| Robot doesn't return to exact start | Odometry drift during long navigation | Reset origin and run again from a closer starting position |
