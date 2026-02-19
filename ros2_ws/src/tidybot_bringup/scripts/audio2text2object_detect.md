# Audio2Text -> Object Detection (Real Robot) Quickstart

This guide is only for testing the loop:
1. Voice or typed command -> NLP
2. NLP publishes `/perception/target_label`
3. Detector switches target and runs YOLO

## One-Time Setup

```bash
cd /home/ubuntu/Desktop/collaborative
uv sync
uv pip install ultralytics

cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select tidybot_control tidybot_bringup
```

## Run (2 Terminals)

Terminal 1: launch camera + microphone + NLP
```bash
cd /home/ubuntu/Desktop/collaborative/ros2_ws
source setup_env.bash
export GEMINI_API_KEY="YOUR_KEY"
ros2 launch tidybot_bringup real.launch.py \
  use_base:=false use_arms:=false use_camera:=true \
  use_microphone:=true use_nlp:=true use_rviz:=false
```

Terminal 2: run detector
```bash
cd /home/ubuntu/Desktop/collaborative/ros2_ws
source setup_env.bash
ros2 run tidybot_bringup detect_object_real.py --ros-args -p target_label:=apple
```

`target_label:=apple` is just a startup default. NLP overrides it at runtime via `/perception/target_label`.

## Trigger Test

In Terminal 1 (NLP interactive prompt):
- Say or type: `pick up the banana and place it in the bin`
- Then: `yes`

Expected:
- NLP publishes `banana` on `/perception/target_label`
- Detector logs `Updated target label: banana`
- If banana is visible:
  - `/perception/object_found` -> `true`
  - `/perception/object_label` -> `banana`
  - `/perception/object_bbox` updates

## Optional Monitoring (3rd Terminal)

```bash
cd /home/ubuntu/Desktop/collaborative/ros2_ws
source setup_env.bash
ros2 topic echo /perception/target_label
ros2 topic echo /perception/object_found
```

## Quick Troubleshooting

- No voice input: confirm `microphone_node` is running and mic is on the robot machine.
- No NLP response: verify `GEMINI_API_KEY` is set in Terminal 1.
- No detector switch: check `/perception/target_label` topic and detector logs.
- No detections: verify `/camera/color/image_raw` exists and banana is visible.
