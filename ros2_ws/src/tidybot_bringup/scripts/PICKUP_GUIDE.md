# TidyBot2 Block Pickup Guide

## Overview

The `pick_up_block_real.py` script enables your robot to pick up blocks from the ground using:
- **Pinocchio-based IK solver** (via `/plan_to_target` service)
- **Top-down grasping** approach
- **Automatic waypoint computation** based on block position

## Quick Start

### 1. Launch the Robot

**Terminal 1 - Start robot hardware:**
```bash
cd ~/collaborative-robotics-2026-group5/ros2_ws
source setup_env.bash
ros2 launch tidybot_bringup real.launch.py
```

**Terminal 2 - Start IK service:**
```bash
cd ~/collaborative-robotics-2026-group5/ros2_ws
source setup_env.bash
ros2 run tidybot_ik motion_planner_real_node
```

### 2. Run the Pickup Script

**Terminal 3 - Execute pickup:**
```bash
cd ~/collaborative-robotics-2026-group5/ros2_ws/src/tidybot_bringup/scripts
source ../../setup_env.bash
python3 pick_up_block_real.py
```

## How It Works

### Pickup Sequence

1. **APPROACH** - Move to 15cm above the block
2. **DESCEND** - Lower to 6cm above the block (grasp height)
3. **GRASP** - Close gripper to grip the block
4. **LIFT** - Raise block to 20cm above original position
5. **SLEEP** - Return to safe resting position

### IK-Based Motion Planning

Unlike the old hardcoded version, this script:
- ✅ Computes joint positions automatically using IK
- ✅ Handles different block positions dynamically
- ✅ Checks for singularities and collisions
- ✅ Uses top-down grasping orientation (gripper pointing down)

## Configuration

### Block Position

Edit the block position in `pick_up_block_real.py`:

```python
# Block position in base_link frame (meters)
BLOCK_POS = np.array([0.45, -0.10, 0.02])  # [x, y, z]
```

**Coordinate Frame:**
- **X** - Forward (positive) / Backward (negative)
- **Y** - Left (positive) / Right (negative)
- **Z** - Up (positive) / Down (negative)
- **Origin** - Robot's base_link (center of mobile base)

### Grasp Heights

Adjust these constants based on your block size and gripper:

```python
APPROACH_HEIGHT = 0.15  # 15cm above block
GRASP_HEIGHT = 0.06     # 6cm above block (adjust for block + gripper)
LIFT_HEIGHT = 0.20      # 20cm lift after grasping
```

### Grasp Orientation

The script uses a top-down grasp (gripper pointing straight down):

```python
# Quaternion for 180° rotation around X-axis
TOP_DOWN_QUAT = [0.0, 1.0, 0.0, 0.0]  # [x, y, z, w]
```

For different grasp orientations, modify this quaternion.

## Adding Block Detection

Currently, the block position is hardcoded. Here's how to add vision-based detection:

### Option 1: Using RealSense Camera

```python
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2

class PickupReal(Node):
    def __init__(self):
        super().__init__("pickup_real")

        # Add camera subscribers
        self.bridge = CvBridge()
        self.rgb_sub = self.create_subscription(
            Image, '/camera/rgb/image_raw', self.rgb_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10
        )

        self.latest_rgb = None
        self.latest_depth = None

    def rgb_callback(self, msg):
        self.latest_rgb = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def depth_callback(self, msg):
        self.latest_depth = self.bridge.imgmsg_to_cv2(msg, "passthrough")

    def detect_block(self):
        """Detect block and return its 3D position in base_link frame."""
        if self.latest_rgb is None or self.latest_depth is None:
            return None

        # 1. Color-based detection (example for red blocks)
        hsv = cv2.cvtColor(self.latest_rgb, cv2.COLOR_BGR2HSV)

        # Red color mask (adjust ranges for your block)
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if not contours:
            return None

        # Get largest contour (assumed to be the block)
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)

        if M["m00"] == 0:
            return None

        # Centroid in image coordinates
        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])

        # 2. Get 3D position using depth
        depth_value = self.latest_depth[cy, cx]  # in mm or m (check your camera)

        # 3. Transform from camera frame to base_link frame
        # You'll need camera intrinsics and tf transformation
        # This is simplified - use proper camera calibration
        fx = 616.0  # focal length X (from camera info)
        fy = 616.0  # focal length Y
        ppx = 320.0  # principal point X
        ppy = 240.0  # principal point Y

        # Convert to 3D point in camera frame
        z_cam = depth_value / 1000.0  # convert mm to meters
        x_cam = (cx - ppx) * z_cam / fx
        y_cam = (cy - ppy) * z_cam / fy

        # Transform to base_link (you need TF lookup for proper transformation)
        # This is a simplified example assuming camera is at fixed position
        x_base = x_cam + 0.2  # camera offset from base
        y_base = y_cam
        z_base = z_cam - 0.3  # camera height

        return np.array([x_base, y_base, z_base])
```

Then in `run_pickup()`, replace:
```python
# Before:
approach_pos = BLOCK_POS + np.array([0, 0, APPROACH_HEIGHT])

# After:
detected_block_pos = self.detect_block()
if detected_block_pos is None:
    self.get_logger().error("No block detected!")
    return False

self.get_logger().info(f"Block detected at: {detected_block_pos}")
approach_pos = detected_block_pos + np.array([0, 0, APPROACH_HEIGHT])
```

### Option 2: Using AprilTags

If you attach AprilTags to blocks:

```bash
# Install AprilTag ROS package
sudo apt install ros-humble-apriltag-ros

# Launch AprilTag detection
ros2 launch apriltag_ros continuous_detection.launch.py
```

Subscribe to detections:
```python
from apriltag_msgs.msg import AprilTagDetectionArray

def apriltag_callback(self, msg):
    for detection in msg.detections:
        if detection.id[0] == YOUR_TAG_ID:
            # Get pose from detection
            pos = detection.pose.pose.pose.position
            block_pos = np.array([pos.x, pos.y, pos.z])
            return block_pos
```

## Troubleshooting

### IK Fails to Solve

**Problem:** `Failed to solve IK for approach/grasp/lift`

**Solutions:**
1. Check if block position is reachable:
   - Right arm workspace: approximately 0.2m to 0.6m from base
2. Try adjusting the block position closer to the robot
3. Reduce `use_orientation=True` to `use_orientation=False` for position-only IK
4. Check joint limits in the IK service

### Gripper Doesn't Close

**Problem:** Gripper opens but doesn't close on block

**Solutions:**
1. Adjust `GRASP_HEIGHT` - may be too high
2. Increase gripper closing duration: `self.set_right_gripper(1.0, duration=2.0)`
3. Check gripper motor torque settings

### Arm Moves Too Fast/Slow

Adjust `move_time` parameter:
```python
self.move_right_arm(approach_joints, move_time=5.0)  # slower (5 seconds)
self.move_right_arm(approach_joints, move_time=1.0)  # faster (1 second)
```

### Block Position Inaccurate

1. **Calibrate camera**: Ensure camera intrinsics are correct
2. **Check TF tree**: Verify camera-to-base transformation
3. **Measure manually**: Use a ruler to verify detected vs. actual position
4. **Add offset correction**: Empirically tune detection

## Testing Tips

### 1. Test IK Service Independently

```bash
# Test if IK service is working
ros2 service call /plan_to_target tidybot_msgs/srv/PlanToTarget \
  "{arm_name: 'right',
    target_pose: {position: {x: 0.4, y: -0.1, z: 0.2},
                  orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}},
    use_orientation: true,
    execute: false}"
```

### 2. Visualize in RViz

Add a marker to visualize the target block position:

```python
from visualization_msgs.msg import Marker

def publish_block_marker(self, position):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.type = Marker.CUBE
    marker.action = Marker.ADD
    marker.pose.position.x = position[0]
    marker.pose.position.y = position[1]
    marker.pose.position.z = position[2]
    marker.scale.x = marker.scale.y = marker.scale.z = 0.05
    marker.color.a = 1.0
    marker.color.r = 1.0
    self.marker_pub.publish(marker)
```

### 3. Dry Run (No Gripper)

Test motion without closing gripper:
```python
# Comment out the gripper closing line:
# self.set_right_gripper(1.0, duration=1.5)
```

## Next Steps

1. **Add block detection** using camera or AprilTags
2. **Integrate obstacle avoidance** for safer motion
3. **Add error recovery** - retry if grasp fails
4. **Support multiple blocks** - detect and pick up multiple objects
5. **Add place operation** - complete pick-and-place task

## Further Reading

- [Pinocchio IK Documentation](https://stack-of-tasks.github.io/pinocchio/)
- [RealSense Camera Setup](https://github.com/IntelRealSense/realsense-ros)
- [OpenCV Object Detection Tutorial](https://docs.opencv.org/4.x/d7/d8b/tutorial_py_lucas_kanade.html)
