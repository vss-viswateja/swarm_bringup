#!/usr/bin/env python3
"""
Launch file for Mobile Manipulator Task Action Server.

This launch file starts the mobman_task_action_server node which handles
navigation, pick, and place tasks for the mobile manipulator.

Usage:
    ros2 launch swarm_bringup mobman_task_action_server.launch.py
    
    # With custom namespace
    ros2 launch swarm_bringup mobman_task_action_server.launch.py robot_namespace:=robot1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for MobMan task action server."""
    
    # Declare launch arguments
    declare_robot_namespace = DeclareLaunchArgument(
        'robot_namespace',
        default_value='mobman',
        description='Namespace for the robot'
    )
    
    declare_planning_group = DeclareLaunchArgument(
        'planning_group',
        default_value='mobman_arm',
        description='MoveIt planning group name'
    )
    
    declare_end_effector_link = DeclareLaunchArgument(
        'end_effector_link',
        default_value='arm_link6_1',
        description='End effector link name (without namespace prefix)'
    )
    
    declare_feedback_rate = DeclareLaunchArgument(
        'feedback_rate',
        default_value='2.0',
        description='Feedback publishing rate in Hz (2.0 = 500ms)'
    )

    robot_namespace = LaunchConfiguration('robot_namespace')

    declare_base_frame = DeclareLaunchArgument(
        'base_frame',
        default_value=[robot_namespace, '/base_link'],
        description='Base frame of the robot'
    )

    declare_odom_frame = DeclareLaunchArgument(
        'odom_frame',
        default_value=[robot_namespace, '/odom'],
        description='Odom frame of the robot'
    )

    declare_planning_frame = DeclareLaunchArgument(
        'planning_frame',
        default_value=[robot_namespace, '/world'],
        description='Planning frame for MoveIt'
    )
    
    declare_transform_frame = DeclareLaunchArgument(
        'transform_frame',
        default_value=[robot_namespace, '/chassis_link'],
        description='Transform frame for MoveIt'
    )
    
    # Mobile Manipulator Task Action Server node
    mobman_task_action_server = Node(
        package='swarm_bringup',
        executable='mobman_task_action_server',
        name='mobman_task_action_server',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_namespace': LaunchConfiguration('robot_namespace'),
            'planning_group': LaunchConfiguration('planning_group'),
            'end_effector_link': LaunchConfiguration('end_effector_link'),
            'feedback_rate': LaunchConfiguration('feedback_rate'),
            'base_frame': LaunchConfiguration('base_frame'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'planning_frame': LaunchConfiguration('planning_frame'),
            'transform_frame': LaunchConfiguration('transform_frame'),
        }],
    )
    
    return LaunchDescription([
        declare_robot_namespace,
        declare_planning_group,
        declare_end_effector_link,
        declare_feedback_rate,
        declare_base_frame,
        declare_odom_frame,
        declare_planning_frame,
        declare_transform_frame,
        mobman_task_action_server,
    ])
