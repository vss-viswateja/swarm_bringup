#!/usr/bin/env python3
"""
Launch file for UR5 Task Action Server with Gripper Service.

This launch file starts:
1. ur5_task_action_server - handles pick/place tasks via MoveIt2
2. simple_gripper_service - handles attach/detach operations

Usage:
    ros2 launch swarm_bringup ur5_task_action_server.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for UR5 task action server + gripper."""
    
    # Declare launch arguments
    declare_robot_namespace = DeclareLaunchArgument(
        'robot_namespace',
        default_value='ur5',
        description='Namespace for the robot'
    )
    
    declare_planning_group = DeclareLaunchArgument(
        'planning_group',
        default_value='arm',
        description='MoveIt planning group name'
    )
    
    declare_end_effector_link = DeclareLaunchArgument(
        'end_effector_link',
        default_value='ur5/link6_1',
        description='End effector link name'
    )
    
    declare_planning_frame = DeclareLaunchArgument(
        'planning_frame',
        default_value='ur5/world',
        description='MoveIt planning frame'
    )
    
    declare_feedback_rate = DeclareLaunchArgument(
        'feedback_rate',
        default_value='2.0',
        description='Feedback publishing rate in Hz (2.0 = 500ms)'
    )
    
    declare_planning_time = DeclareLaunchArgument(
        'planning_time',
        default_value='10.0',
        description='MoveIt planning timeout in seconds'
    )
    
    # UR5 Task Action Server node
    ur5_task_action_server = Node(
        package='swarm_bringup',
        executable='ur5_task_action_server',
        name='ur5_task_action_server',
        output='screen',
        parameters=[{
            'robot_namespace': LaunchConfiguration('robot_namespace'),
            'planning_group': LaunchConfiguration('planning_group'),
            'end_effector_link': LaunchConfiguration('end_effector_link'),
            'planning_frame': LaunchConfiguration('planning_frame'),
            'feedback_rate': LaunchConfiguration('feedback_rate'),
            'planning_time': LaunchConfiguration('planning_time'),
        }],
    )
    
    # Simple Gripper Service node
    simple_gripper_service = Node(
        package='swarm_bringup',
        executable='simple_gripper_service',
        name='simple_gripper_service',
        output='screen',
    )
    
    return LaunchDescription([
        declare_robot_namespace,
        declare_planning_group,
        declare_end_effector_link,
        declare_planning_frame,
        declare_feedback_rate,
        declare_planning_time,
        ur5_task_action_server,
        simple_gripper_service,
    ])
