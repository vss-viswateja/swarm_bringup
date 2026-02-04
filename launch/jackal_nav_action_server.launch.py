#!/usr/bin/env python3
"""
Launch file for Jackal Navigation Action Server.

This launch file starts the jackal_nav_action_server node which bridges
RobotTask action requests to Nav2 NavigateToPose.

Usage:
    ros2 launch swarm_bringup jackal_nav_action_server.launch.py
    
    # With custom namespace
    ros2 launch swarm_bringup jackal_nav_action_server.launch.py robot_namespace:=robot1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Jackal nav action server."""
    
    # Declare launch arguments
    declare_robot_namespace = DeclareLaunchArgument(
        'robot_namespace',
        default_value='jackal',
        description='Namespace for the robot'
    )
    
    declare_feedback_rate = DeclareLaunchArgument(
        'feedback_rate',
        default_value='1.0',
        description='Feedback publishing rate in Hz'
    )
    
    # Jackal Navigation Action Server node
    jackal_nav_action_server = Node(
        package='swarm_bringup',
        executable='jackal_nav_action_server',
        name='jackal_nav_action_server',
        output='screen',
        parameters=[{
            'robot_namespace': LaunchConfiguration('robot_namespace'),
            'feedback_rate': LaunchConfiguration('feedback_rate'),
        }],
    )
    
    return LaunchDescription([
        declare_robot_namespace,
        declare_feedback_rate,
        jackal_nav_action_server,
    ])
