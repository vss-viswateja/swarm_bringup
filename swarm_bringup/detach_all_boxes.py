#!/usr/bin/env python3
"""
Detach All Aruco Boxes at Startup

This script sends detach commands to all 48 aruco boxes for a specified robot.
Run this after the robot arm has spawned to release all initially-attached boxes.

Parameters:
    robot_model (str): Robot model name (e.g., 'ur5_assembly', 'mobile_manipulator')
    world_name (str): Gazebo world name

Usage:
    ros2 run swarm_bringup detach_all_boxes --ros-args -p robot_model:=ur5_assembly -p world_name:=my_world
    
Or via launch file:
    ros2 launch swarm_bringup detach_all_boxes.launch.py robot_model:=ur5_assembly world_name:=my_world
"""

import subprocess
import sys
import time

import rclpy
from rclpy.node import Node

# List of aruco box IDs to detach
# Only boxes 26-33 exist in the current world
ARUCO_BOX_IDS = [26, 27, 28, 29, 30, 31, 32, 33]


class DetachAllBoxesNode(Node):
    """
    ROS 2 node that detaches all aruco boxes from a robot.
    """

    def __init__(self):
        super().__init__('detach_all_boxes')

        # Declare parameters
        self.declare_parameter('robot_model', '')
        self.declare_parameter('world_name', '')
        self.declare_parameter('delay', 0.05)

        self.robot_model = self.get_parameter('robot_model').get_parameter_value().string_value
        self.world_name = self.get_parameter('world_name').get_parameter_value().string_value
        self.delay = self.get_parameter('delay').get_parameter_value().double_value

        if not self.robot_model:
            self.get_logger().warn('No robot_model specified!')
        if not self.world_name:
            self.get_logger().warn('No world_name specified!')

        self.get_logger().info(f'Robot Model: {self.robot_model}')
        self.get_logger().info(f'World Name: {self.world_name}')

    def detach_all_boxes(self):
        """
        Send detach commands for all aruco boxes.
        """
        self.get_logger().info(f"Detaching all {len(ARUCO_BOX_IDS)} boxes from {self.robot_model}...")

        success_count = 0
        fail_count = 0

        for box_id in ARUCO_BOX_IDS:
            topic = f"/{self.robot_model}/detach_box_{box_id}"

            try:
                result = subprocess.run(
                    ['ign', 'topic', '-t', topic, '-m', 'ignition.msgs.Empty', '-p', ''],
                    capture_output=True,
                    text=True,
                    timeout=2
                )

                if result.returncode == 0:
                    success_count += 1
                else:
                    fail_count += 1
                    self.get_logger().warn(f"Failed to detach box_{box_id}")

            except subprocess.TimeoutExpired:
                fail_count += 1
                self.get_logger().warn(f"Timeout for box_{box_id}")
            except Exception as e:
                fail_count += 1
                self.get_logger().error(f"Error for box_{box_id}: {e}")

            # Small delay to avoid overwhelming the system
            time.sleep(self.delay)

        self.get_logger().info(f"Done! Detached: {success_count}, Failed: {fail_count}")
        return success_count, fail_count


def main(args=None):
    rclpy.init(args=args)

    node = DetachAllBoxesNode()

    # Wait for simulation to stabilize
    node.get_logger().info("Waiting 2 seconds for simulation to stabilize...")
    time.sleep(2)

    # Detach all boxes
    node.detach_all_boxes()

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
