# Pan-Tilt Camera Tilt Sign Issue

## Problem

The `camera_tilt` motor (Dynamixel ID 22) reports **positive** values when the camera is tilted **down**, but the URDF's `camera_tilt` joint expects **negative** values for downward tilt (joint axis is Y, so negative rotation = forward/down tilt).

This causes the TF tree (`camera_color_optical_frame` → `base_link`) to compute incorrect 3D positions. Specifically, the x-coordinate is flipped (objects to the right appear to the left, and vice versa).

### Evidence

- Camera physically tilted down at ~45°
- `camera_tilt` joint state reports: **+0.589 rad (~34°)** — should be **negative**
- `camera_pan` reports -0.0015 (near zero, correct)
- Object (banana) placed to the right of robot center reports `x=+0.437` in `base_link` frame — should be **negative** (x+ is left in base_link)

### Root Cause

The `camera_tilt` motor's `Drive_Mode` is set to `0` (normal direction) in `ros2_ws/src/tidybot_bringup/config/right_arm_pantilt.yaml`. The motor's positive rotation direction doesn't match the URDF joint convention.

### Impact

All 3D object positions computed by `detect_object_real.py` (and any other node using the camera TF) are incorrect. The pickup pipeline (`test_pickup.py`) receives wrong target positions, causing IK to fail or the arm to reach the wrong location.

## Potential Fixes

### Fix 1: Change Drive_Mode in YAML config (Simplest)

Change `Drive_Mode` from `0` to `1` for `camera_tilt` in `right_arm_pantilt.yaml`:

```yaml
camera_tilt:
  ID: 22
  Baud_Rate: 3
  Return_Delay_Time: 0
  Drive_Mode: 1  # Reverse direction to match URDF convention
```

**Pros:** Single line change, fixes it at the source for all consumers.
**Cons:** Requires `load_configs: true` on next launch (which is the default). May affect pan-tilt commands too (commands would also need to flip sign, unless the SDK handles it automatically).

### Fix 2: Joint State Corrector Node

Create a small ROS2 node that:
1. Subscribes to `/camera/pan_tilt_state_raw` (remap xs_sdk output in launch file)
2. Negates the tilt position/velocity
3. Republishes to `/camera/pan_tilt_state`

Changes needed:
- New node: `tidybot_control/pan_tilt_corrector_node.py`
- Launch file: remap xs_sdk pan-tilt output to `_raw`, add corrector node
- Register in `tidybot_control/setup.py`

**Pros:** No motor config changes, doesn't affect command direction.
**Cons:** Extra node, slight added complexity.

### Fix 3: Correct in detect_object_real.py Only

After looking up the TF, manually apply a tilt sign correction. This is a localized hack.

**Pros:** Minimal changes, only affects the detector.
**Cons:** Hacky. Other nodes using the camera TF would still get wrong transforms. Doesn't fix the underlying issue.

### Fix 4: Change URDF Joint Axis Direction

Flip the `camera_tilt` joint axis in the URDF from `<axis xyz="0 1 0"/>` to `<axis xyz="0 -1 0"/>`.

**Pros:** No motor config changes.
**Cons:** Could break simulation if the sim uses the same URDF. Joint limits would also need to be swapped.

## Recommendation

**Fix 1** (Drive_Mode) is the cleanest if it doesn't cause issues with pan-tilt commands. Test by changing the YAML value, relaunching, and verifying that:
1. `camera_tilt` reports negative when tilted down
2. Pan-tilt commands still move the camera in the correct direction

If Fix 1 causes command direction issues, use **Fix 2** (corrector node) as it only affects the state reporting, not commands.
