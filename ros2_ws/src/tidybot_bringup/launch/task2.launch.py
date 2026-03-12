"""
Task 2 Launch File — Pick and Place pipeline.

Launches:
  1. detect_object_real.py  — perception (object detection)
  2. navigate_to_object.py  — navigation to standoff position
  3. task2_pickup.py        — arm pickup + drop (auto_start mode, waits for trigger)
  4. task2_coordinator.py   — orchestrator (state machine)

NLP node is NOT included — run it in a separate terminal for voice input:
    ros2 run tidybot_control nlp_interface_node

Usage:
    # Terminal 1: Launch robot hardware
    ros2 launch tidybot_bringup real.launch.py use_planner:=true

    # Terminal 2: Launch Task 2 pipeline
    ros2 launch tidybot_bringup task2.launch.py

    # Terminal 3 (optional): Voice input
    ros2 run tidybot_control nlp_interface_node

    # Or trigger manually:
    ros2 topic pub /coordinator/start std_msgs/String "data: banana" --once
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'target_label', default_value='banana',
            description='Default object to detect'
        ),
        DeclareLaunchArgument(
            'standoff_dist', default_value='0.4',
            description='Navigation standoff distance from object (meters)'
        ),
        DeclareLaunchArgument(
            'detect_timeout', default_value='30.0',
            description='Timeout for initial object detection (seconds)'
        ),
        DeclareLaunchArgument(
            'nav_timeout', default_value='90.0',
            description='Timeout for navigation (seconds)'
        ),
        DeclareLaunchArgument(
            'pickup_timeout', default_value='60.0',
            description='Timeout for pickup (seconds)'
        ),
        DeclareLaunchArgument(
            'min_confidence', default_value='0.4',
            description='Minimum detection confidence'
        ),
        LogInfo(msg='=== Task 2 Pipeline: Launching detection, navigation, pickup, coordinator ==='),

        # 1. Object detection
        Node(
            package='tidybot_bringup',
            executable='detect_object_real.py',
            name='detect_object_real',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'target_label': LaunchConfiguration('target_label'),
                'publish_debug_image': True,
                'save_debug_image': True,
            }],
        ),

        # 2. Navigation (reset_origin_on_start locks current position as (0,0))
        Node(
            package='tidybot_bringup',
            executable='navigate_to_object.py',
            name='navigate_to_object',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'robot': 'real',
                'mode': 'object',
                'reset_origin_on_start': True,
                'standoff_dist': LaunchConfiguration('standoff_dist'),
                'lateral_offset': 0.15,
                'goal_tolerance': 0.08,
                'kp': 1.0,
                'max_v': 0.2,
                'max_omega': 2.0,
                'pose_timeout': 30.0,
            }],
        ),

        # 3. Pickup (auto_start — waits for coordinator trigger)
        Node(
            package='tidybot_bringup',
            executable='task2_pickup.py',
            name='pickup',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'auto_start': True,
            }],
        ),

        # 4. Coordinator (orchestrator)
        Node(
            package='tidybot_bringup',
            executable='task2_coordinator.py',
            name='coordinator',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'detect_timeout': LaunchConfiguration('detect_timeout'),
                'nav_timeout': LaunchConfiguration('nav_timeout'),
                'pickup_timeout': LaunchConfiguration('pickup_timeout'),
                'min_confidence': LaunchConfiguration('min_confidence'),
            }],
        ),
    ])
