# Camera Mount URDF Issue — Wrong base_to_camera_mount Yaw

## Summary

Commit `3cee5a4` (Matt, Mar 8 2026) changed the `base_to_camera_mount` joint yaw from
`-1.5708` (−90°) to `0` in `tidybot_wx250s.urdf.xacro`. This broke all 3D object pose
estimates from `detect_object_real.py` by rotating the entire camera TF chain 90° from
physical reality.

**This was reverted.** The correct value is `rpy="0 0 -1.5708"`.

---

## The Affected Joint

```xml
<!-- tidybot_wx250s.urdf.xacro -->
<joint name="base_to_camera_mount" type="fixed">
  <parent link="base_link"/>
  <child link="camera_mount"/>
  <origin xyz="0 -0.1125 0.279" rpy="0 0 -1.5708"/>  <!-- -90° yaw: correct -->
</joint>
```

This joint anchors the pan-tilt camera mount to the robot base. Every frame downstream
inherits its orientation:

```
base_link
  └── camera_mount          ← this joint
        └── pan_link        (rotates L/R)
              └── tilt_link (rotates U/D)
                    └── camera_link
                          └── camera_color_optical_frame  ← used by detect_object_real.py
```

---

## Why −90° Is Correct

With `rpy="0 0 -1.5708"`, the rotation matrix for `base_to_camera_mount` is:

```
Rz(-90°) = [[ 0,  1,  0],
             [-1,  0,  0],
             [ 0,  0,  1]]
```

This means `camera_mount +X` points in `base_link -Y` direction (to the right of the robot).
The pan joint then rotates around Z. At `pan = 0`, the camera therefore points in
`camera_mount +X` → which after the tilt chain resolves to `base_link +X` (forward). This
matches physical reality: the camera faces forward when pan = 0.

With `rpy="0 0 0"` (the broken value), `camera_mount +X` aligns with `base_link +X`
(forward) immediately. At `pan = 0` the camera then appears to point **left** in base_link
coordinates — 90° off from physical reality.

---

## Impact of the Wrong Value

All 3D object positions from `detect_object_real.py` (and any node using the camera TF)
were rotated 90° around the robot's Z axis:

- Object straight ahead → reported as being to the left
- Object to the right → reported as being in front
- The x-coordinate sign/magnitude was consistently wrong

`test_pickup.py` and `pickup_anything.py` received these wrong positions, causing the arm
to reach to the wrong location.

---

## Related Issue: Tilt Sign Bug

A **separate** pre-existing issue also exists: the `camera_tilt` motor (ID 22) reports
**positive** values when the camera is physically tilted **down**, but the URDF joint axis
(`<axis xyz="0 1 0"/>`) expects **negative** for downward tilt. This causes a more subtle
depth-dependent error in 3D poses. See [pan_tilt_sign_issue.md](pan_tilt_sign_issue.md)
for details and fix options.

---

## How to Verify the URDF Is Correct

With the robot running and the camera at `pan=0, tilt=0`:

```bash
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

The translation should put the camera roughly in front of and above the robot base
(positive x, small y, positive z). If x is near zero and y is large, the mount yaw
is still wrong.

Also check object detection:

```bash
ros2 run tidybot_bringup detect_object_real.py --ros-args -p target_label:=banana
```

Place a banana directly in front of the robot (~0.5m). The logged pose x should be
~0.5, y near 0.
