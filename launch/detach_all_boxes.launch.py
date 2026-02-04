"""
Launch file for the Detach All Boxes node.

This node sends detach commands to all aruco boxes for a specified robot.
Uses the DetachableJoint plugin via ign topic commands.

Usage:
    ros2 launch swarm_bringup detach_all_boxes.launch.py
    ros2 launch swarm_bringup detach_all_boxes.launch.py robot_model:=agent1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='mobman',
        description='Robot model name for DetachableJoint topics. Must match the '
                    'robot_namespace used when spawning (e.g., "agent1", "mobman")'
    )

    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='construction_world',
        description='Name of the Gazebo world'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    # Detach All Boxes Node
    detach_all_boxes_node = Node(
        package='swarm_bringup',
        executable='detach_all_boxes',
        name='detach_all_boxes',
        output='screen',
        parameters=[{
            'robot_model': LaunchConfiguration('robot_model'),
            'world_name': LaunchConfiguration('world_name'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        robot_model_arg,
        world_name_arg,
        use_sim_time_arg,
        detach_all_boxes_node,
    ])
