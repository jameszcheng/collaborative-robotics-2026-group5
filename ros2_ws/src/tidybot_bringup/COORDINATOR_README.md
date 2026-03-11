# Coordinator Pipeline — End-to-End Pick and Place

## Overview

The coordinator node orchestrates the full pipeline: **voice/manual trigger -> detect object -> navigate to it -> re-detect at close range -> pick it up**.

It acts as a "traffic cop" — it doesn't duplicate any logic from the individual nodes, it just sends signals between them at the right time.

## Architecture

```
                    ┌──────────────────┐
                    │   NLP Interface   │  (separate terminal - voice input)
                    │  /nlp/response    │
                    └────────┬─────────┘
                             │ JSON command
                             v
┌──────────┐  /coordinator/start   ┌──────────────────┐
│  Manual  │ ────────────────────> │   COORDINATOR    │
│  Trigger │                       │  (state machine)  │
└──────────┘                       └──┬───┬───┬───┬───┘
                                      │   │   │   │
                 /perception/         │   │   │   │  /coordinator/
                 target_label         │   │   │   │  pickup_trigger
                    ┌─────────────────┘   │   │   └──────────────────┐
                    v                     │   │                      v
           ┌────────────────┐             │   │             ┌────────────────┐
           │  detect_object │             │   │             │  test_pickup   │
           │   _real.py     │             │   │             │   (auto_start) │
           └───────┬────────┘             │   │             └───────┬────────┘
                   │ /perception/         │   │                     │
                   │ object_pose          │   │  /coordinator/      │ /coordinator/
                   │ object_found         │   │  nav_goal           │ pickup_complete
                   └──────────────────────┘   │                     │
                                              v                     │
                                     ┌────────────────┐             │
                                     │ navigate_to    │             │
                                     │ _object.py     │             │
                                     └───────┬────────┘             │
                                             │ /coordinator/        │
                                             │ nav_complete         │
                                             └──────────────────────┘
```

## State Machine

```
IDLE → SEARCHING → NAVIGATING → REDETECTING → PICKING_UP → DONE → IDLE
                                                              ↓
Any state ──────────────────────────────────── FAILED ──────→ IDLE
```

| State | What happens | Timeout |
|-------|-------------|---------|
| IDLE | Waits for NLP command or manual trigger | - |
| SEARCHING | Publishes target label, waits for detection with sufficient confidence | 30s |
| NAVIGATING | Sends pose to nav node, waits for nav_complete signal | 90s |
| REDETECTING | Collects 3 fresh pose samples at close range, averages them | 10s |
| PICKING_UP | Triggers pickup node, waits for pickup_complete signal | 60s |
| DONE | Logs success, returns to IDLE after 2s | - |

## Quick Start

### Terminal 1 — Robot hardware + planner
```bash
ros2 launch tidybot_bringup real.launch.py use_planner:=true
```

### Terminal 2 — Coordinator pipeline (detection + pickup + coordinator)
```bash
ros2 launch tidybot_bringup coordinator.launch.py
```
Note: `navigate_to_object.py` is already launched by `real.launch.py` in Terminal 1.

### Terminal 3 — Voice input (optional)
```bash
ros2 run tidybot_control nlp_interface_node
```
Then say: "pick up the banana"

### Without voice — manual trigger
```bash
ros2 topic pub /coordinator/start std_msgs/String "data: banana" --once
```

## Monitoring

```bash
# Watch coordinator state transitions
ros2 topic echo /coordinator/status

# Watch all coordinator topics
ros2 topic echo /coordinator/nav_complete
ros2 topic echo /coordinator/pickup_complete
```

## Launch Parameters

```bash
ros2 launch tidybot_bringup coordinator.launch.py \
    target_label:=apple \
    standoff_dist:=0.3 \
    detect_timeout:=20.0 \
    nav_timeout:=60.0 \
    pickup_timeout:=45.0 \
    min_confidence:=0.5
```

## "Place in Bin" Strategy

The pickup node's existing DROP state moves the object 25cm to the right and opens the gripper. Position the bin to the robot's right side.

## Troubleshooting

| Problem | Likely Cause | Fix |
|---------|-------------|-----|
| "No banana detected after 30s" | Detection node not running or object not visible | Check `detect_object_real.py` is launched, object is in camera view |
| "Navigation timed out after 90s" | Nav node stuck or odometry issues | Check `navigate_to_object.py` output, verify `/odom` topic |
| "Pickup failed" | Object out of arm reach or IK failure | Check arm workspace, verify object pose is reasonable |
| "Re-detection timed out" | Object moved or camera can't see it from standoff | Reduce standoff distance, check camera angle |
| Coordinator ignores trigger | Already in a non-IDLE state | Wait for current task to complete or restart |

## Topics Reference

### Coordinator Subscribes To
| Topic | Type | Source |
|-------|------|--------|
| `/nlp/response` | String (JSON) | NLP node |
| `/coordinator/start` | String | Manual trigger |
| `/perception/object_found` | Bool | Detection node |
| `/perception/object_pose` | PoseStamped | Detection node |
| `/perception/object_confidence` | Float32 | Detection node |
| `/coordinator/nav_complete` | Bool | Navigation node |
| `/coordinator/pickup_complete` | Bool | Pickup node |

### Coordinator Publishes
| Topic | Type | Target |
|-------|------|--------|
| `/perception/target_label` | String | Detection node |
| `/coordinator/status` | String | Monitoring |
| `/coordinator/nav_goal` | PoseStamped | Navigation node |
| `/coordinator/pickup_trigger` | Empty | Pickup node |
| `/cmd_vel` | Twist | Emergency stop |
