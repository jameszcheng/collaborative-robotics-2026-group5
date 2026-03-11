# Pickup Issue — 2026-03-11 Morning

## Symptom

When running the full coordinator pipeline, the arm was moving to the wrong
position during pickup. The robot would navigate to the object correctly, but
the arm trajectory was targeting an inaccurate or stale position.

## Root Cause

The coordinator and pickup node were using **different object poses**.

The pipeline has three separate averaging steps:

1. **`navigate_to_object.py`** — independently collects 5 raw samples from
   `/perception/object_pose` and averages them before locking the navigation
   goal. This is what the user observed as "waiting for 5 poses before moving."

2. **`coordinator_node.py` REDETECTING state** — after navigation completes,
   resets all perception state, sets a minimum timestamp to discard stale
   detections, then collects `redetect_samples` (default 3) fresh close-range
   poses and averages them. This refined averaged pose was stored only in the
   coordinator's local `self.object_pose` and **never published anywhere**.

3. **`pickup.py`** — was subscribing directly to `/perception/object_pose`
   (raw live detection) and used whatever the latest single frame happened to
   be at the time of the pickup trigger. It completely bypassed the
   coordinator's careful averaging and timestamp filtering.

This meant the arm could receive a noisy single-frame detection, or in the
worst case a pose from during navigation (when the robot was still moving and
TF accuracy was lower), rather than the refined close-range average the
coordinator had computed.

## Fix

**`coordinator_node.py`**: After computing the averaged redetection pose,
publish it to a new `/coordinator/object_pose` topic (as a `PoseStamped` in
`base_link` frame) immediately before sending the pickup trigger.

**`pickup.py`**: Added a subscriber to `/coordinator/object_pose`. When a
message arrives and the node is in `auto_start` mode, it overwrites
`self.object_pose` with the coordinator's refined value. Since the refined
pose is published just before the trigger, `wait_for_object_detection` will
find the coordinator pose already set when it runs.

Standalone mode (running `pickup.py` without the coordinator) is unaffected —
it still reads from `/perception/object_pose` as before.

## Additional Issue — Mixed Samples from Different Camera Positions

### Symptom

A secondary contributor to the wrong arm position: during the REDETECTING camera sweep,
samples collected at **different camera angles** were mixed together in the same average.

### Root Cause

The `_redetect_poses` buffer was never reset when the camera sweep moved to a new position.
If a sample was captured at sweep position A (e.g., pan left), the camera advanced to
position B (e.g., pan right), and another sample arrived there, both samples would be
averaged together — yielding a pose that corresponded to neither camera angle and was
geometrically inconsistent.

Additionally, the freeze-on-detection logic used the live `valid_detection` flag (which can
flicker) rather than `n > 0` (whether a committed, confidence-filtered sample was actually
collected). This meant the camera could unfreeze on a momentary detection loss and resume
advancing, collecting subsequent samples at a different angle before re-freezing.

### Fix

**`coordinator_node.py`** REDETECTING state rewritten around three exclusive branches:

1. **`n >= redetect_samples`**: all samples are from the same position — proceed to pickup.
2. **`n > 0`**: first sample arrived at this position. Camera is immediately frozen
   (`_redetect_sweep_active = False`). A `_redetect_detected_hold_start` timer starts.
   If the remaining samples aren't collected within `_redetect_position_sample_timeout`
   (10s), **all samples are discarded** and the sweep advances to the next position.
   This guarantees every sample in the average comes from the same camera angle.
3. **`n == 0`**: no samples yet. If sweeping, advance after `_redetect_sweep_settle_time`
   (5s) with no detection; if in initial hold, wait the same settle time then start sweep.

A new `_advance_redetect_sweep()` helper handles all sweep advances — it always resets
`_redetect_poses` and `_redetect_detected_hold_start` before moving the camera, ensuring
a clean slate at each position.

## Files Changed

- `ros2_ws/src/tidybot_bringup/scripts/coordinator_node.py`
- `ros2_ws/src/tidybot_bringup/scripts/pickup.py`
