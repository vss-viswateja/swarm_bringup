"""
Launch file for the Simple Gripper Service node.

This node provides services to attach/detach objects to the robot gripper
using dynamic joint spawning in Gazebo.

Usage:
    ros2 launch swarm_bringup simple_gripper.launch.py
    ros2 launch swarm_bringup simple_gripper.launch.py world_name:=my_world
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='construction_world_v3',
        description='Name of the Gazebo world (must match the world name in SDF)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    # Simple Gripper Service Node
    simple_gripper_node = Node(
        package='swarm_bringup',
        executable='simple_gripper_service',
        name='simple_gripper_service',
        output='screen',
        parameters=[{
            'world_name': LaunchConfiguration('world_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        world_name_arg,
        use_sim_time_arg,
        simple_gripper_node,
    ])
