# 3D Object Pose Pipeline README

This document defines the pipeline to extend 2D detection into a 3D pose output for navigation/planning.

## Goal

Given a detected object in RGB, publish its 3D pose in a robot/navigation frame.

## Core Steps

1. Detect object in RGB and get pixel `(u, v)` (usually bbox center).
2. Get depth at `(u, v)` from a depth image aligned to the RGB frame.
3. Back-project `(u, v, depth)` to 3D in camera frame.
4. Transform 3D point to `base_link` or `map` with TF2.
5. Publish pose for planner/navigation.

## Required Inputs

- RGB image: `/camera/color/image_raw`
- Depth image: aligned-to-color depth topic (or align first)
- Camera intrinsics:
  - `/camera/color/camera_info`
  - `/camera/depth/camera_info`
- TF tree including camera optical frame to `base_link`/`map`
- 2D detector output (bbox or center pixel)

## Alignment Requirement

Depth and RGB are not inherently pixel-aligned on RealSense (different lenses).
You must align depth to color before reading depth for RGB pixel `(u, v)`.

Preferred:
- Enable RealSense alignment (`align_depth.enable=true`).

Fallback:
- Manual alignment pipeline/script (as discussed by staff).

## Back-Projection (Camera Frame)

Given aligned depth `z` (meters) and color intrinsics `(fx, fy, cx, cy)`:

- `x = (u - cx) * z / fx`
- `y = (v - cy) * z / fy`
- `z = depth(u, v)`

Result is a 3D point in camera optical frame.

## Transform to Robot/Nav Frame

Use TF2 to transform camera-frame point to:
- `base_link` for robot-relative behavior, or
- `map` for global navigation goals.

## Recommended Output Contract

Primary topic:
- `/perception/object_pose`
- `geometry_msgs/PoseStamped`
- `header.frame_id`: `base_link` or `map`
- `pose.position`: `x, y, z` (meters)
- `pose.orientation`: identity quaternion if orientation is not estimated

Optional metadata topic:
- label/class name
- detection confidence
- depth validity flag
- timestamp age

## Validation Checklist

- RGB and aligned depth share same resolution and indexing.
- Depth near object center is valid (nonzero, finite).
- Published pose is stable when object is static.
- Pose moves consistently when object moves.
- RViz shows pose in the expected physical location.

## Notes for Navigation Team

- If pose is in `base_link`, nav should TF-transform to `map` before global planning.
- For base-only navigation, typically use `x, y` and apply a standoff offset.
- Reject stale/low-confidence/invalid poses.
