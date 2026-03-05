#!/usr/bin/env python3
"""
Launch file for CHARS Central Allocator.

Starts the central allocator node with configurable agent mode.

Agent modes:
    - 'mobman' (default): Homogeneous fleet of mobile manipulators.
      Agent action servers are launched separately via agent_bringup.launch.py.
    - 'mixed': Heterogeneous fleet (Jackal + UR5 + MobMan).

Usage:
    # Homogeneous (default):
    ros2 launch swarm_bringup chars_central_allocator.launch.py

    # Heterogeneous:
    ros2 launch swarm_bringup chars_central_allocator.launch.py agent_type:=mixed
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
    
    declare_agent_type = DeclareLaunchArgument(
        'agent_type', default_value='mobman',
        description="Agent mode: 'mobman' (homogeneous) or 'mixed' (heterogeneous)"
    )
    
    declare_agent_namespaces = DeclareLaunchArgument(
        'agent_namespaces',
        default_value="['agent1', 'agent2']",
        #default_value="['agent1']",
        description="List of agent namespaces (only used when agent_type='mobman')"
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
            'agent_type': LaunchConfiguration('agent_type'),
            'agent_namespaces': LaunchConfiguration('agent_namespaces'),
        }],
    )
    
    return LaunchDescription([
        # Arguments
        declare_alpha,
        declare_beta,
        declare_gamma,
        declare_w_time,
        declare_w_energy,
        declare_w_reach,
        declare_agent_type,
        declare_agent_namespaces,
        # Nodes
        chars_allocator,
    ])
