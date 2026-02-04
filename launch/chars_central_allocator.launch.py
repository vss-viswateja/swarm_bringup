#!/usr/bin/env python3
"""
Launch file for CHARS Central Allocator and all robot task action servers.

This comprehensive launch file starts the complete CHARS allocation system:
1. chars_central_allocator - The master node for task allocation
2. jackal_nav_action_server - Jackal navigation action server
3. ur5_task_action_server - UR5 pick/place action server
4. mobman_task_action_server - Mobile Manipulator full-stack action server
5. simple_gripper_service - Gripper attach/detach service

Usage:
    ros2 launch swarm_bringup chars_central_allocator.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for CHARS allocation system."""
    
    # ===================== Launch Arguments =====================
    declare_alpha = DeclareLaunchArgument(
        'alpha', default_value='0.5',
        description='Time score decay factor'
    )
    
    declare_beta = DeclareLaunchArgument(
        'beta', default_value='1.0',
        description='Energy score shape factor'
    )
    
    declare_gamma = DeclareLaunchArgument(
        'gamma', default_value='0.3',
        description='Reachability decay factor'
    )
    
    declare_w_time = DeclareLaunchArgument(
        'w_time', default_value='0.4',
        description='Time weight'
    )
    
    declare_w_energy = DeclareLaunchArgument(
        'w_energy', default_value='0.2',
        description='Energy weight'
    )
    
    declare_w_reach = DeclareLaunchArgument(
        'w_reach', default_value='0.4',
        description='Reachability weight'
    )
    
    # ===================== CHARS Central Allocator =====================
    chars_allocator = Node(
        package='swarm_bringup',
        executable='chars_central_allocator',
        name='chars_central_allocator',
        output='screen',
        parameters=[{
            'alpha': LaunchConfiguration('alpha'),
            'beta': LaunchConfiguration('beta'),
            'gamma': LaunchConfiguration('gamma'),
            'w_time': LaunchConfiguration('w_time'),
            'w_energy': LaunchConfiguration('w_energy'),
            'w_reach': LaunchConfiguration('w_reach'),
        }],
    )
    
    # ===================== Jackal Nav Action Server =====================
    jackal_action_server = Node(
        package='swarm_bringup',
        executable='jackal_nav_action_server',
        name='jackal_nav_action_server',
        output='screen',
        parameters=[{
            'robot_namespace': 'jackal',
            'feedback_rate': 2.0,
        }],
    )
    
    # ===================== UR5 Task Action Server =====================
    ur5_action_server = Node(
        package='swarm_bringup',
        executable='ur5_task_action_server',
        name='ur5_task_action_server',
        output='screen',
        parameters=[{
            'robot_namespace': 'ur5',
            'planning_group': 'arm',
            'end_effector_link': 'ur5/link6_1',
            'feedback_rate': 2.0,
        }],
    )
    
    # ===================== MobMan Task Action Server =====================
    mobman_action_server = Node(
        package='swarm_bringup',
        executable='mobman_task_action_server',
        name='mobman_task_action_server',
        output='screen',
        parameters=[{
            'robot_namespace': 'mobman',
            'planning_group': 'arm',
            'end_effector_link': 'mobman/arm_link6_1',
            'feedback_rate': 2.0,
        }],
    )
    
    # ===================== Gripper Service =====================
    gripper_service = Node(
        package='swarm_bringup',
        executable='simple_gripper_service',
        name='simple_gripper_service',
        output='screen',
    )
    
    return LaunchDescription([
        # Arguments
        declare_alpha,
        declare_beta,
        declare_gamma,
        declare_w_time,
        declare_w_energy,
        declare_w_reach,
        # Nodes
        chars_allocator,
        jackal_action_server,
        ur5_action_server,
        mobman_action_server,
        gripper_service,
    ])
