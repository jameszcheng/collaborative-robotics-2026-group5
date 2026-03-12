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
IDLE → SEARCHING → NAVIGATING → PAUSE → PICKING_UP → DONE → IDLE
                                                ↓
Any state ──────────────────────── FAILED ────> IDLE
```

| State | What happens | Timeout |
|-------|-------------|---------|
| **IDLE** | Waits for a voice command or manual trigger on `/coordinator/start` | — |
| **SEARCHING** | Publishes the target label to perception; collects 3 confident detections, averages their x/y, and sends the averaged pose as a nav goal | 30 s |
| **NAVIGATING** | Waits for `nav_complete` from `navigate_to_object` (robot drives to standoff position in front of the object); on arrival tilts camera down to 0.5 rad | 90 s |
| **PAUSE** | Short settling delay (default 3 s) after navigation; stale detection data clears during this window before pickup is triggered | — |
| **PICKING_UP** | Publishes an `Empty` trigger to `task2_pickup` and waits for the pickup complete signal | 120 s |
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
- `task2_pickup.py` receives the trigger and runs its own camera sweep + re-detection internally, then:
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
    pickup_timeout:=120.0 \     # seconds to complete pickup+drop (default 120)
    min_confidence:=0.4 \       # minimum YOLO detection confidence (default 0.4)
    search_samples:=3 \         # confident detections to average before navigating (default 3)
    pause_duration:=3.0         # settling delay after navigation before pickup (default 3)
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
| `/perception/object_pose` | PoseStamped | Detection node | SEARCHING |
| `/perception/object_confidence` | Float32 | Detection node | SEARCHING |
| `/coordinator/nav_complete` | Bool | Navigation node | NAVIGATING |
| `/coordinator/pickup_complete` | Bool | Pickup node | PICKING_UP |

### Coordinator Publishes
| Topic | Type | Target | Sent In State |
|-------|------|--------|---------------|
| `/perception/target_label` | String | Detection node | SEARCHING (start) |
| `/coordinator/status` | String | Monitoring | All states |
| `/coordinator/nav_goal` | PoseStamped | Navigation node | SEARCHING (end) |
| `/coordinator/pickup_trigger` | Empty | Pickup node | PAUSE (end) |
| `/camera/pan_tilt_cmd` | Float64MultiArray | Camera | NAVIGATING (end) |
| `/cmd_vel` | Twist (zero) | Base | FAILED (e-stop) |

---

## Troubleshooting

| Symptom | Likely Cause | Fix |
|---------|-------------|-----|
| "No object detected after 30s" | Detection node not running or camera can't see object | Check `detect_object_real.py` is running; move object into camera FOV |
| "Navigation timed out after 90s" | Nav node stuck or odometry drift | Check `navigate_to_object.py` output; verify `/odom` is publishing |
| "Pickup failed" | Object out of arm reach or IK failure | Verify object is within arm workspace; check object pose values |
| Object dropped in wrong place | Drop offset doesn't align with bowl | Reposition bowl to robot's right side, ~25 cm away |
| Coordinator ignores voice trigger | Already in a non-IDLE state | Wait for current task to complete or restart Terminal 2 |
