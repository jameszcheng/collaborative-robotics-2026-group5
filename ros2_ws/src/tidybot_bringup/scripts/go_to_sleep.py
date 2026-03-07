#!/usr/bin/env python3
"""
Send one or both arms to the sleep position.

Usage:
    # Terminal 1: launch hardware
    ros2 launch tidybot_bringup real.launch.py

    # Terminal 2: send both arms to sleep
    ros2 run tidybot_bringup go_to_sleep.py

    # Or specify which arm:
    ros2 run tidybot_bringup go_to_sleep.py --ros-args -p arm:=right
    ros2 run tidybot_bringup go_to_sleep.py --ros-args -p arm:=left
"""

import time

import numpy as np
import rclpy
from interbotix_xs_msgs.msg import JointGroupCommand
from rclpy.node import Node
from sensor_msgs.msg import JointState

# [waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate]
SLEEP_POSE = [0.0, -1.80, 1.55, 0.0, 0.8, 0.0]


class GoToSleep(Node):

    def __init__(self):
        super().__init__('go_to_sleep')

        self.declare_parameter('arm', 'both')  # 'right', 'left', or 'both'
        self.declare_parameter('max_joint_speed', 0.5)  # rad/s

        self.arm = self.get_parameter('arm').value
        self.max_joint_speed = self.get_parameter('max_joint_speed').value

        self.current_joint_positions = {}
        self.joint_states_received = False
        self.create_subscription(JointState, '/joint_states', self._js_cb, 10)

        self.arm_pubs = {
            'right': self.create_publisher(JointGroupCommand, '/right_arm/commands/joint_group', 10),
            'left': self.create_publisher(JointGroupCommand, '/left_arm/commands/joint_group', 10),
        }

    def _js_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.current_joint_positions[name] = pos
        self.joint_states_received = True

    def wait_for_joint_states(self, timeout: float = 10.0) -> bool:
        start = time.time()
        while (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.joint_states_received:
                return True
        return False

    def get_arm_positions(self, arm_name: str) -> np.ndarray:
        joint_names = [
            f'{arm_name}_waist', f'{arm_name}_shoulder', f'{arm_name}_elbow',
            f'{arm_name}_forearm_roll', f'{arm_name}_wrist_angle', f'{arm_name}_wrist_rotate',
        ]
        return np.array([self.current_joint_positions.get(j, 0.0) for j in joint_names])

    def go_to_sleep(self, arm_name: str):
        rclpy.spin_once(self, timeout_sec=0.1)

        current = self.get_arm_positions(arm_name)
        target = np.array(SLEEP_POSE)

        max_diff = np.max(np.abs(target - current))
        duration = max(max_diff / self.max_joint_speed, 1.0)

        self.get_logger().info(
            f'Moving {arm_name} arm to sleep pose over {duration:.1f}s '
            f'(max joint diff: {max_diff:.2f} rad)'
        )

        rate_hz = 50.0
        dt = 1.0 / rate_hz
        num_steps = max(int(duration * rate_hz), 1)

        for i in range(num_steps + 1):
            t = i / num_steps
            alpha = 0.5 * (1 - np.cos(np.pi * t))
            q = current + alpha * (target - current)

            cmd = JointGroupCommand()
            cmd.name = f'{arm_name}_arm'
            cmd.cmd = q.tolist()
            self.arm_pubs[arm_name].publish(cmd)

            if i < num_steps:
                time.sleep(dt)

        self.get_logger().info(f'{arm_name} arm is now in sleep pose.')


def main(args=None):
    rclpy.init(args=args)
    node = GoToSleep()

    try:
        node.get_logger().info('Waiting for joint states...')
        if not node.wait_for_joint_states():
            node.get_logger().error('No joint states received! Is real.launch.py running?')
            return

        arms = ['right', 'left'] if node.arm == 'both' else [node.arm]
        for arm in arms:
            node.go_to_sleep(arm)

        node.get_logger().info('Done.')
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
