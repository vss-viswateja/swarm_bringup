#!/usr/bin/env python3
"""
Swarm launch file for 3 mobile manipulator agents.

Launches all 3 agents with staggered timing to avoid overloading Gazebo.
Each agent gets its own namespace, spawn position, and nav2 params.

Usage:
    ros2 launch swarm_bringup swarm3_bringup.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    swarm_bringup_dir = get_package_share_directory('swarm_bringup')

    agent_launch_file = os.path.join(
        swarm_bringup_dir, 'launch', 'agent_bringup.launch.py'
    )

    # --- Agent definitions: (namespace, x, y, z) ---
    agents = [
        ('agent1', '1.0',  '-3.0', '0.22'),
        ('agent2', '0.0',  '-2.0', '0.22'),
        ('agent3', '-1.0', '-3.0', '0.22'),
    ]

    # Stagger each agent by 70s to let the previous one fully initialize
    # (each agent takes ~60s internally to bring up all subsystems)
    AGENT_STAGGER_SECONDS = 70.0

    actions = []

    for i, (ns, x, y, z) in enumerate(agents):
        agent_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(agent_launch_file),
            launch_arguments={
                'robot_namespace': ns,
                'pose_x': x,
                'pose_y': y,
                'pose_z': z,
            }.items(),
        )

        delay = i * AGENT_STAGGER_SECONDS
        if delay == 0.0:
            actions.append(agent_launch)
        else:
            actions.append(
                TimerAction(period=delay, actions=[agent_launch])
            )

    return LaunchDescription(actions)
