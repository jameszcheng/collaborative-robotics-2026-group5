# Pick-and-Place Pipeline — Full Technical Explanation

This document explains every node in the coordinator pick-and-place pipeline in detail:
how each one works internally, what it publishes/subscribes to, and how they interact.

---

## Pipeline Overview

```
nlp_interface_node.py     — voice/text interface, command parsing, spoken feedback
detect_object_real.py     — vision: YOLO + depth → 3D pose in base_link
navigate_to_object.py     — base navigation to standoff position
pickup.py                 — arm IK planning and grasp execution
coordinator_node.py       — state machine orchestrating the above three
```

The coordinator is the only node that "knows" the full physical sequence — the other
nodes are reactive and respond to the signals the coordinator sends them. The NLP node
acts as the operator-facing layer: it turns speech/text into structured commands, keeps
the target label aligned with perception, and speaks progress updates back to the user.

---

## 0. nlp_interface_node.py

**Role:** Provide the conversational front end for the pipeline. It accepts typed or
spoken user input, parses it into a structured command, publishes that command to the
robot pipeline, and relays perception/coordinator progress back as spoken feedback.

### How it works

**Command parsing:**
1. Accepts text from the interactive terminal or the `/nlp/parse` service
2. Uses the NLP parser to classify the input as `chat`, `confirm`, `command`, or `exit`
3. Publishes the raw JSON response on `/nlp/response`
4. If the final result is a real `command`, also publishes the requested object label to
   `/perception/target_label`

**Why `/nlp/response` matters:**
The coordinator subscribes to `/nlp/response` and only starts the pipeline when it sees
a JSON message with `type == "command"` and a valid object. This keeps normal
conversation separate from robot execution.

**Perception-aware conversation:**
- The NLP layer keeps a lightweight snapshot of current perception state
- If an object is currently detected, the NLP prompt can answer questions like
  "what do you see?" in a grounded way
- If perception is not currently publishing, the assistant still responds
  conversationally and can guide the user toward a command

**Progress narration:**
The NLP node subscribes to coordinator/perception status topics and turns them into
short spoken updates such as:
- "Searching for the banana."
- "I found the banana. Target locked."
- "Going for the pickup now."
- "Task complete. The banana is in the bin."

**Manual speech fallback:**
The interactive terminal also supports keyboard-triggered speech lines. This gives the
operator a way to keep the interaction flowing even if the physical pipeline pauses or
needs recovery.

### Topics / Services

| Direction | Topic/Service | Type |
|-----------|--------------|------|
| Pub | `/nlp/response` | String (JSON) |
| Pub | `/perception/target_label` | String |
| Sub | `/coordinator/status` | String |
| Sub | `/perception/object_found` | Bool |
| Sub | `/perception/object_confidence` | Float32 |
| Sub | `/perception/object_label` | String |
| Sub | `/coordinator/pickup_complete` | Bool |
| Srv | `/nlp/parse` | NlpCommand |

---

## 1. detect_object_real.py

**Role:** Continuously run YOLO on the RGB camera stream. When the target object is
detected, compute its 3D position in the robot's `base_link` frame using the depth
camera and TF transforms, and publish it.

### How it works

**Input streams:**
- `/camera/color/image_raw` — raw RGB frames from the RealSense D435
- `/camera/aligned_depth_to_color/image_raw` — depth frames aligned to RGB pixels
  (falls back to `/camera/depth/image_raw` if the aligned topic doesn't appear within 3s)
- `/camera/color/camera_info` — camera intrinsics (focal length, principal point).
  Subscribed once; unsubscribed after first message.
- `/perception/target_label` — runtime override of what object to look for

**Every RGB frame (`rgb_cb`):**
1. Convert ROS image to OpenCV BGR
2. Run `YOLO.predict()` at `conf_threshold` (default 0.35)
3. Find the highest-confidence detection matching `target_label`
4. Publish: `object_found` (Bool), `object_label`, `object_confidence`, `object_bbox`
5. Compute 3D pose (`_publish_pose`):
   - Sample a 21×21 pixel patch around the bounding box center in the depth image
   - Take the **median** depth of valid (non-zero, finite) pixels — robust to noise
   - Convert pixel + depth → camera optical frame XYZ using pinhole back-projection:
     `x_cam = (u - cx) * z / fx`, `y_cam = (v - cy) * z / fy`
   - Look up TF transform from `camera_color_optical_frame` → `base_link` at
     `rclpy.time.Time()` (latest available)
   - Apply the rotation + translation to get world-frame XYZ
   - Publish `PoseStamped` to `/perception/object_pose` with `frame_id = base_link`

**Key design detail — coordinate frame:**
The pose is published in `base_link`. The camera optical frame has Z pointing forward,
X pointing right, Y pointing down. After the TF transform to `base_link` (X=left,
Y=back, Z=up), the sign conventions are flipped. This is why `navigate_to_object.py`
applies a 180° rotation (`odom_x = -pose.x`, `odom_y = -pose.y`) to convert to its
navigation frame.

**Target label override:**
When the coordinator publishes to `/perception/target_label`, `target_label_cb` updates
the detection target at runtime. This is how the coordinator tells the detector what to
look for when a new task begins.

### Topics

| Direction | Topic | Type |
|-----------|-------|------|
| Sub | `/camera/color/image_raw` | Image |
| Sub | `/camera/aligned_depth_to_color/image_raw` | Image |
| Sub | `/camera/color/camera_info` | CameraInfo |
| Sub | `/perception/target_label` | String |
| Pub | `/perception/object_found` | Bool |
| Pub | `/perception/object_pose` | PoseStamped (base_link frame) |
| Pub | `/perception/object_confidence` | Float32 |
| Pub | `/perception/object_label` | String |
| Pub | `/perception/object_bbox` | Int32MultiArray [x, y, w, h] |
| Pub | `/perception/object_debug_image` | Image (annotated) |

---

## 2. navigate_to_object.py

**Role:** Navigate the robot base to a standoff position in front of the detected object,
then align the robot to face it. Publish `nav_complete` when done.

### How it works

**Two operating modes relevant to the pipeline:**
- `object` mode (default when launched by coordinator): waits for a nav_goal, then drives
  to the computed standoff position
- Under the coordinator, the node never does its own pose averaging — it receives a
  pre-averaged goal directly via `/coordinator/nav_goal`

**Startup and origin:**
- On startup with `reset_origin_on_start: True`, deletes the saved origin file
- Waits for odometry to stabilize (25 consecutive readings within 0.005m and 0.02rad
  threshold) before locking the origin
- All navigation is done in an **origin-relative frame**: the robot's position when it
  first stabilized is (0, 0, 0°)

**`_nav_goal_callback` — receiving the coordinator's pre-averaged pose:**
When `/coordinator/nav_goal` arrives:
1. Resets all internal nav state (goal, alignment, timing)
2. Sets `_nav_goal_received = True` — this gates `_object_pose_callback` so the node
   won't accumulate raw poses until coordinator says go
3. Converts the base_link pose to the navigation odom frame:
   `odom_x = -pose.x`, `odom_y = -pose.y` (180° flip due to frame convention)
4. Injects it directly into `_pose_buffer` as a single entry and sets
   `_object_pose_ready = True` — bypassing the local multi-sample accumulation entirely

**`_object_pose_callback` — raw pose accumulation (bypassed in coordinator mode):**
When `_nav_goal_received` is False, this callback returns immediately. When True but
already locked (`_object_goal_set`), also returns. Otherwise: applies the 180° frame
flip and accumulates into `_pose_buffer`. Once `pose_samples` (default 5, unused in
coordinator mode since we inject a single entry) samples are accumulated, marks
`_object_pose_ready = True`.

**`_compute_object_goal` — standoff computation:**
1. Averages the pose buffer (single entry in coordinator mode)
2. Transforms from odom frame → origin frame (subtracts saved origin, rotates by
   `-origin_theta`)
3. Computes standoff: steps back `standoff_dist` (default 0.4m) along the
   robot→object direction
4. Applies `lateral_offset` (default 0.15m left, to center the right arm on the object):
   perpendicular to the robot→object direction
5. Applies the command frame rotation (-90°) to match movement_4.py convention
6. Computes `face_yaw`: the heading from standoff position toward the object

**Control loop (50 Hz) — three phases:**
1. **Wait for goal**: if `_object_pose_ready` but `_object_goal_set` not yet, call
   `_compute_object_goal()`. If timeout (default 30s) with no pose, stop and fail.
2. **Navigate to standoff**: proportional controller on position error:
   - `v = kp * distance` (clamped to `max_v = 0.2 m/s`)
   - `omega = 2 * kp * heading_error` (clamped to `max_omega = 2.0 rad/s`)
   - Slows linear speed to 30% when heading error > 45°
   - When within `goal_tolerance` (0.08m), transitions to phase 3
3. **Align yaw**: rotates in place until facing the object within `yaw_tolerance` (5°).
   When done, publishes `Bool(data=True)` to `/coordinator/nav_complete` and stops.

### Topics

| Direction | Topic | Type |
|-----------|-------|------|
| Sub | `/perception/object_pose` | PoseStamped (gated by `_nav_goal_received`) |
| Sub | `/coordinator/nav_goal` | PoseStamped (pre-averaged goal from coordinator) |
| Sub | `/odom` | Odometry |
| Pub | `/cmd_vel` | Twist |
| Pub | `/coordinator/nav_complete` | Bool |

---

## 3. pickup.py

**Role:** Execute the grasp sequence on the right arm using the IK planning service.
In `auto_start` mode (used by coordinator), wait idle until triggered, then use the
coordinator's refined pose for arm planning.

### Two modes

**Standalone mode** (`auto_start: False`):
- Waits for object detection on `/perception/object_pose` directly
- Prompts user to press Enter before executing
- Runs once and exits

**Coordinator mode** (`auto_start: True`, set by launch file):
- Stays alive, spinning, waiting for `/coordinator/pickup_trigger`
- When triggered, immediately runs `_run_pickup_once()` then resets and waits again
- Overwrites `self.object_pose` with the coordinator's refined averaged pose from
  `/coordinator/object_pose` in `_coordinator_pose_cb` — never uses raw perception
  for arm planning

### `_coordinator_pose_cb` — why it exists

The coordinator computes a 3-sample average from close range during REDETECTING and
publishes it to `/coordinator/object_pose` **just before** sending the pickup trigger.
When this message arrives, `_coordinator_pose_cb` overwrites `self.object_pose` with
the refined value. Then when the trigger arrives a moment later and
`wait_for_object_detection` runs, it finds the coordinator pose already set. This
ensures the arm plans to the carefully averaged close-range pose, not a noisy
single-frame raw detection.

### Arm state machine (`run_state_machine`)

All waypoints are computed from `self.object_pose` (the coordinator-refined position):

```
APPROACH  → open gripper, move arm to object_x/y at approach_height (+0.17m above object)
DESCEND   → move arm down to grasp_height (+0.06m above object)
GRASP     → close gripper (hold PWM continuously)
LIFT      → move arm up to lift_height (+0.25m above object)
DROP      → move arm 0.25m to the right (x-0.25), open gripper
DONE      → publish pickup_complete=True
```

Each arm movement uses the `PlanToTarget` ROS2 service — it runs inverse kinematics,
plans a trajectory, and executes it. If any `plan_and_execute` call fails (IK failure,
workspace limit), the state machine returns False and the coordinator is notified with
`pickup_complete=False`.

**Orientation:** All waypoints use `ORIENT_FINGERS_DOWN = (qw=0.5, qx=0.5, qy=0.5, qz=-0.5)`,
which orients the gripper fingers pointing straight down for a top-down grasp.

**Y offset:** A `Y_OFFSET = -0.07m` is applied to all waypoints — shifts the grasp
target 7cm further from the robot in the base_link Y direction, compensating for
the systematic error between the camera-reported position and the actual grasping
contact point.

**Gripper control:**
The gripper is controlled by publishing to `/{arm}_gripper/cmd` (Float64MultiArray).
- Open: publish `[0.0]` for 2 seconds
- Close: publish `[1.0]` for 2 seconds with `hold=True` (skips the stop command,
  maintaining grip force)
- Stop: publish `[0.5]` (maps to PWM=0 in the wrapper node)

### Topics / Services

| Direction | Topic/Service | Type |
|-----------|--------------|------|
| Sub | `/coordinator/pickup_trigger` | Empty |
| Sub | `/coordinator/object_pose` | PoseStamped (refined pose) |
| Sub | `/perception/object_pose` | PoseStamped (standalone mode fallback) |
| Sub | `/perception/object_found` | Bool |
| Sub | `/perception/object_confidence` | Float32 |
| Sub | `/joint_states` | JointState (startup health check) |
| Pub | `/coordinator/pickup_complete` | Bool |
| Pub | `/right_arm/commands/joint_group` | JointGroupCommand (sleep pose) |
| Pub | `/right_gripper/cmd` | Float64MultiArray |
| Client | `/plan_to_target` | PlanToTarget.srv |

---

## 4. coordinator_node.py

**Role:** State machine that sequences the entire pipeline. Does not do perception,
navigation, or manipulation itself — it reads data, makes decisions, and sends signals.

### State machine

```
IDLE → SEARCHING → NAVIGATING → REDETECTING → PICKING_UP → DONE → IDLE
  ↑                                                                  |
  └──────────────────── FAILED (any state) ──────────────────────────┘
```

The coordinator runs a 10 Hz timer (`_tick`). All decision logic lives in `_tick`;
callbacks only update data fields.

### IDLE

Waits for a trigger on either:
- `/nlp/response` — JSON string from the NLP voice interface, e.g.
  `{"type": "command", "intent": "pick up", "object": "banana"}`
- `/coordinator/start` — plain string, e.g. `"banana"` (manual trigger)

Both call `_start_pipeline(label)` which resets all state and transitions to SEARCHING.
Triggers are ignored if already in a non-IDLE state.

### SEARCHING

Goal: get a confident, noise-reduced estimate of the object position to send to the
navigator.

1. Publishes target label to `/perception/target_label` so `detect_object_real.py`
   knows what to look for
2. Each pose callback (`_obj_pose_cb`) checks `object_confidence >= min_confidence`
   **at collection time** and appends to `_search_poses` if confident
3. In `_tick`: once `search_samples` (default 3) confident poses are collected:
   - Averages their x/y (z from last sample — not needed for 2D navigation)
   - Packs into a `PoseStamped` in `base_link` frame
   - Publishes to `/coordinator/nav_goal`
   - Transitions to NAVIGATING
4. Timeout → FAILED if no detection after `detect_timeout` (default 30s)

**Why 3-sample averaging here:** A single frame from YOLO can be noisy. Three confident
detections average out single-frame depth noise before we commit to a navigation goal.

### NAVIGATING

Waits for `/coordinator/nav_complete` (published by `navigate_to_object.py` when it
reaches the standoff and aligns). No logic besides waiting and timeout checking.

When `nav_complete` arrives:
- Transitions to REDETECTING
- Resets all perception state (clears buffered poses, object_pose, confidence)
- Records `_redetect_min_pose_stamp_sec` = current ROS time — any pose message
  stamped before this time is rejected in `_obj_pose_cb`. This prevents stale poses
  captured during navigation (while robot was moving, degrading TF accuracy) from
  contaminating the close-range estimate.

Timeout → FAILED after `nav_timeout` (default 90s).

### REDETECTING

Goal: get a high-quality close-range estimate of the object position for the arm.
All `redetect_samples` must come from the **same camera position** to prevent
averaging poses from different angles.

**Three exclusive branches based on `n = len(_redetect_poses)`:**

**`n >= redetect_samples` (default 3):**
- All samples guaranteed from same camera position — average x/y/z
- Publish averaged `PoseStamped` to `/coordinator/object_pose`
- Immediately publish `Empty` to `/coordinator/pickup_trigger`
- Transition to PICKING_UP

**`n > 0` (first sample arrived, waiting for rest):**
- On first entry: freeze camera (`_redetect_sweep_active = False`), record
  `_redetect_detected_hold_start`
- Each tick: check how long we've been waiting at this position
- If `position_elapsed > _redetect_position_sample_timeout` (10s): discard all
  samples, advance camera to next sweep position, reset `_redetect_detected_hold_start`

**`n == 0` (no samples yet at current position):**
- If sweeping: advance to next position after `_redetect_sweep_settle_time` (5s)
- If in initial hold (just arrived from navigation): wait 5s then start sweep if
  still no detection

**Camera sweep (`_advance_redetect_sweep`):**
If the object isn't visible at the initial camera position, the coordinator sweeps
through 6 pan-tilt positions via `/camera/pan_tilt_cmd`:
```
(pan=0.0,  tilt=0.5)   center, looking down steeply
(pan=0.0,  tilt=0.3)   center, shallower
(pan=-0.3, tilt=0.5)   left
(pan=+0.3, tilt=0.5)   right
(pan=-0.3, tilt=0.3)   left, shallower
(pan=+0.3, tilt=0.3)   right, shallower
```
Each position is held for 5s. The moment a confident sample arrives at any position,
the sweep freezes and waits for the remaining samples at that exact position.
`_advance_redetect_sweep()` always resets `_redetect_poses = []` before moving,
guaranteeing no samples from different angles are mixed.

**Timestamp filtering in `_obj_pose_cb`:**
```python
if self.state == State.REDETECTING and pose_stamp_sec < self._redetect_min_pose_stamp_sec:
    return  # reject stale pose from during navigation
```
Only poses timestamped after the redetection started are accepted.

Timeout → FAILED after `redetect_timeout` (default 30s). Camera pans back to center
and robot stops.

### PICKING_UP

Waits for `/coordinator/pickup_complete` (Bool from `pickup.py`):
- `True` → DONE
- `False` → FAILED with log "Pickup failed"
- Timeout → FAILED after `pickup_timeout` (default 60s)

### DONE

Waits 2 seconds (lets logs flush, arm settle), then returns to IDLE. Ready for next command.

### FAILED

Not a persistent state — immediately returns to IDLE after publishing status and sending
an emergency stop on `/cmd_vel` (zero Twist). The operator can trigger a new task
immediately without restarting.

### Topics

| Direction | Topic | Type |
|-----------|-------|------|
| Sub | `/nlp/response` | String (JSON) |
| Sub | `/coordinator/start` | String |
| Sub | `/perception/object_pose` | PoseStamped |
| Sub | `/perception/object_confidence` | Float32 |
| Sub | `/coordinator/nav_complete` | Bool |
| Sub | `/coordinator/pickup_complete` | Bool |
| Pub | `/perception/target_label` | String |
| Pub | `/coordinator/status` | String (`STATE:label`) |
| Pub | `/coordinator/nav_goal` | PoseStamped (SEARCHING average) |
| Pub | `/coordinator/object_pose` | PoseStamped (REDETECTING average) |
| Pub | `/coordinator/pickup_trigger` | Empty |
| Pub | `/camera/pan_tilt_cmd` | Float64MultiArray [pan, tilt] |
| Pub | `/cmd_vel` | Twist (emergency stop) |

---

## End-to-End Data Flow

Here is what happens to the command, perception state, and object pose as they flow
through the pipeline:

```
User / nlp_interface_node.py
  User says "pick up the banana"
  NLP parses → {"type": "command", "object": "banana", ...}
  Published to: /nlp/response
  Also sets: /perception/target_label = "banana"
        │
        ▼
coordinator_node.py (IDLE -> SEARCHING)
  Accepts the command, sets target label, begins the state machine
        │
        ▼
detect_object_real.py
  Per-frame YOLO + median depth → single noisy pose in base_link
  Published to: /perception/object_pose, /perception/object_confidence, /perception/object_label
        │
        ▼
coordinator_node.py (SEARCHING)
  Collects 3 confident frames → averages x/y
  Published to: /coordinator/nav_goal  (base_link, averaged)
        │
        ▼
navigate_to_object.py
  Receives pre-averaged goal, applies 180° frame flip
  Computes standoff (0.4m back) + lateral offset (0.15m left)
  Drives base to that position + aligns yaw to face object
  Published to: /coordinator/nav_complete
        │
        ▼
coordinator_node.py (REDETECTING)
  Discards all pre-navigation poses (timestamp filter)
  Collects 3 confident close-range frames from SAME camera position → averages x/y/z
  Published to: /coordinator/object_pose  (base_link, refined)
  Then immediately: /coordinator/pickup_trigger
        │
        ▼
pickup.py
  Receives refined pose → overwrites self.object_pose
  Receives trigger → runs arm state machine
  Plans IK to: approach → descend → grasp → lift → drop
  Published to: /coordinator/pickup_complete
        │
        ▼
coordinator_node.py (PICKING_UP → DONE → IDLE)
        │
        ▼
nlp_interface_node.py
  Subscribes to coordinator/perception topics
  Speaks status back to the operator as the task progresses
```

---

## Why There Are Two Rounds of Averaging

| Round | Where | Purpose |
|-------|-------|---------|
| SEARCHING (3 samples) | From robot's starting position | Stable nav goal — reduces single-frame depth noise before committing to a navigation direction |
| REDETECTING (3 samples) | From standoff position (~0.4m from object) | Precise arm target — close range gives better depth accuracy; timestamp filter excludes TF-noisy poses from while robot was moving |

A single round of averaging at the starting position would not work for arm planning
because: (1) depth accuracy degrades with distance, (2) TF accuracy is lower while
the robot is moving, and (3) the robot needs to be at standoff before the arm can
reach the object anyway.

---

## Confidence Filtering

Confidence is checked **at collection time** in `_obj_pose_cb`, not at tick evaluation
time. This prevents a common bug where a confident sample is queued, confidence drops
before the tick runs, and the buffered sample gets rejected even though it was captured
at high confidence. Both `_search_poses` and `_redetect_poses` only ever contain frames
where `object_confidence >= min_confidence` (default 0.4) was true **when the frame
arrived**.
