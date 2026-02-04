"""
Launch file for the Object State Manager node.

This node provides services to switch aruco boxes between static and dynamic
states in Gazebo simulation for optimizing physics performance.

Usage:
    ros2 launch swarm_bringup object_state_manager.launch.py

    # With custom world name:
    ros2 launch swarm_bringup object_state_manager.launch.py world_name:=my_world
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='construction_world',
        description='Name of the Gazebo world (must match the world name in SDF)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    # Object State Manager Node
    object_state_manager_node = Node(
        package='swarm_bringup',
        executable='object_state_manager',
        name='object_state_manager',
        output='screen',
        parameters=[{
            'world_name': LaunchConfiguration('world_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        world_name_arg,
        use_sim_time_arg,
        object_state_manager_node,
    ])
