# `detect_object_real.py` Test Guide

This guide shows how to test `detect_object_real.py` on:
- Real TidyBot hardware
- MuJoCo simulation

The node currently does **2D object detection only** (no 3D pose yet).

## What The Node Does

Input:
- `/camera/color/image_raw` (`sensor_msgs/Image`)

Outputs:
- `/perception/object_found` (`std_msgs/Bool`)
- `/perception/object_label` (`std_msgs/String`)
- `/perception/object_bbox` (`std_msgs/Int32MultiArray`) as `[x, y, w, h]`
- `/perception/object_debug_image` (`sensor_msgs/Image`)

## Parameters

- `rgb_topic` (default: `/camera/color/image_raw`)
- `target_label` (default: `apple`)
- `model_path` (default: `yolov8n.pt`)
- `conf_threshold` (default: `0.35`)
- `imgsz` (default: `640`)
- `publish_debug_image` (default: `true`)

## Prerequisites

From repo root:

```bash
uv sync
uv pip install ultralytics
```

Then build ROS package:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tidybot_bringup
source install/setup.bash
```

## Real Robot Test

Terminal 1: bring up robot stack (camera only is enough for perception test)

```bash
cd ros2_ws
source setup_env.bash
ros2 launch tidybot_bringup real.launch.py use_base:=false use_arms:=false
```

Terminal 2: sanity-check camera feed

```bash
cd ros2_ws
source setup_env.bash
ros2 topic hz /camera/color/image_raw
```

Terminal 3: run detector

```bash
cd ros2_ws
source setup_env.bash
ros2 run tidybot_bringup detect_object_real.py --ros-args -p target_label:=apple
```

Terminal 4: inspect outputs

```bash
cd ros2_ws
source setup_env.bash
ros2 topic echo /perception/object_found
ros2 topic echo /perception/object_label
ros2 topic echo /perception/object_bbox
```

Optional debug view:

```bash
ros2 run rqt_image_view rqt_image_view
```

Choose `/perception/object_debug_image`.

## Simulation Test

Terminal 1:

```bash
cd ros2_ws
source setup_env.bash
ros2 launch tidybot_bringup sim.launch.py
```

Terminal 2:

```bash
cd ros2_ws
source setup_env.bash
ros2 run tidybot_bringup detect_object_real.py --ros-args -p target_label:=apple
```

Terminal 3:

```bash
cd ros2_ws
source setup_env.bash
ros2 topic echo /perception/object_found
ros2 topic echo /perception/object_bbox
```

Optional:

```bash
ros2 run rqt_image_view rqt_image_view
```

## Quick Pass Criteria

- With target in view:
  - `/perception/object_found` becomes `true`
  - `/perception/object_label` matches `target_label`
  - `/perception/object_bbox` updates as object moves
- With target out of view:
  - `/perception/object_found` becomes `false`

## Troubleshooting

1. `ImportError: ultralytics`
- Install dependency:
```bash
uv pip install ultralytics
```

2. No detections at all
- Verify object class exists in model classes.
- Lower threshold:
```bash
ros2 run tidybot_bringup detect_object_real.py --ros-args -p target_label:=apple -p conf_threshold:=0.2
```
- Increase image size:
```bash
ros2 run tidybot_bringup detect_object_real.py --ros-args -p target_label:=apple -p imgsz:=960
```

3. Wrong object label names
- COCO examples:
  - `apple`
  - `banana`
  - `cup` (not `mug`)

4. Camera topic missing
- Verify:
```bash
ros2 topic list | rg /camera/color/image_raw
```

5. Slow inference
- Use smaller model and image size:
  - `model_path:=yolov8n.pt`
  - `imgsz:=640` or `imgsz:=480`

## Notes

- This detector is currently 2D only.
- Next step is depth fusion + TF transform for 3D target position in robot frame.
