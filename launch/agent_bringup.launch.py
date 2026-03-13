#!/usr/bin/env python3
"""
Combined launch file for a single mobile manipulator agent.

Launches all subsystems in sequence with proper timing:
  1. Robot bringup (URDF, controllers, odom) — immediate
  2. Nav2 navigation stack — delayed 30s
  3. MoveIt motion planning — delayed 30s
  4. Task action server — delayed 45s

Usage:
    ros2 launch swarm_bringup agent_bringup.launch.py robot_namespace:=agent1 pose_x:=1.0 pose_y:=-3.0 pose_z:=0.22
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package directories
    swarm_description_dir = get_package_share_directory('swarm_description')
    swarm_bringup_dir = get_package_share_directory('swarm_bringup')
    moveit_config_dir = get_package_share_directory('mobman_moveit_config')

    # Launch configurations
    robot_namespace = LaunchConfiguration('robot_namespace')
    pose_x = LaunchConfiguration('pose_x')
    pose_y = LaunchConfiguration('pose_y')
    pose_z = LaunchConfiguration('pose_z')
    world_name = LaunchConfiguration('world_name')

    # --- Declare launch arguments ---
    declare_robot_namespace = DeclareLaunchArgument(
        'robot_namespace',
        default_value='mobman',
        description='Namespace for the robot'
    )

    declare_pose_x = DeclareLaunchArgument(
        'pose_x',
        default_value='1.0',
        description='Initial x position of the robot'
    )

    declare_pose_y = DeclareLaunchArgument(
        'pose_y',
        default_value='-3.0',
        description='Initial y position of the robot'
    )

    declare_pose_z = DeclareLaunchArgument(
        'pose_z',
        default_value='0.22',
        description='Initial z position of the robot'
    )

    declare_world_name = DeclareLaunchArgument(
        'world_name',
        default_value='construction_world',
        description='Name of the Gazebo world (must match the world name in SDF)'
    )

    declare_controller_base_delay = DeclareLaunchArgument(
        'controller_base_delay',
        default_value='15.0',
        description='Base delay before spawning controllers. Increase for multi-robot (e.g., 25.0).'
    )

    controller_base_delay = LaunchConfiguration('controller_base_delay')

    # ---------------------------------------------------------------
    # 1. Robot Bringup (t=0s)
    #    Spawns robot, publishes URDF, starts controllers
    #    Internal delays: spawn 2s, controllers at controller_base_delay
    # ---------------------------------------------------------------
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(swarm_description_dir, 'launch', 'mobile_manipulator.launch.py')
        ),
        launch_arguments={
            'robot_namespace': robot_namespace,
            'pose_x': pose_x,
            'pose_y': pose_y,
            'pose_z': pose_z,
            'use_sim_time': 'true',
            'controller_base_delay': controller_base_delay,
        }.items(),
    )

    # ---------------------------------------------------------------
    # 2. Nav2 Navigation Stack (t=40s)
    #    Needs controllers up + TF tree stable
    # ---------------------------------------------------------------
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(swarm_description_dir, 'launch', 'mobman_nav2.launch.py')
        ),
        launch_arguments={
            'robot_namespace': robot_namespace,
            'pose_x': pose_x,
            'pose_y': pose_y,
            'pose_z': pose_z,
        }.items(),
    )

    delayed_nav2 = TimerAction(
        period=40.0,
        actions=[nav2_launch]
    )

    # ---------------------------------------------------------------
    # 3. MoveIt Motion Planning (t=40s, parallel with Nav2)
    #    Needs joint_state_broadcaster up and publishing
    # ---------------------------------------------------------------
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_dir, 'launch', 'mobman_moveit.launch.py')
        ),
        launch_arguments={
            'robot_namespace': robot_namespace,
            'use_sim_time': 'true',
            'use_rviz': 'false',
        }.items(),
    )

    delayed_moveit = TimerAction(
        period=55.0,
        actions=[moveit_launch]
    )

    # ---------------------------------------------------------------
    # 4. Task Action Server (t=55s)
    #    Needs both Nav2 and MoveIt fully initialized
    # ---------------------------------------------------------------
    task_action_server_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(swarm_bringup_dir, 'launch', 'mobman_task_action_server.launch.py')
        ),
        launch_arguments={
            'robot_namespace': robot_namespace,
        }.items(),
    )

    delayed_task_action_server = TimerAction(
        period=70.0,
        actions=[task_action_server_launch]
    )

    # ---------------------------------------------------------------
    # 5. Simple Gripper Service (t=55s, parallel with action server)
    #    Provides attach/detach services for the gripper
    # ---------------------------------------------------------------
    gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(swarm_bringup_dir, 'launch', 'simple_gripper.launch.py')
        ),
        launch_arguments={
            'robot_namespace': robot_namespace,
            'robot_model': robot_namespace,
            'world_name': world_name,
            'use_sim_time': 'true',
        }.items(),
    )

    delayed_gripper = TimerAction(
        period=70.0,
        actions=[gripper_launch]
    )

    # ---------------------------------------------------------------
    # 6. Detach All Boxes (t=60s)
    #    Runs after everything is up to detach gripper joints
    # ---------------------------------------------------------------
    detach_boxes_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(swarm_bringup_dir, 'launch', 'detach_all_boxes.launch.py')
        ),
        launch_arguments={
            'robot_model': robot_namespace,
            'use_sim_time': 'true',
        }.items(),
    )

    delayed_detach_boxes = TimerAction(
        period=60.0,
        actions=[detach_boxes_launch]
    )

    # ---------------------------------------------------------------
    # 7. TF Alias: chassis_link → {namespace}/chassis_link
    #    MoveIt's SRDF uses <virtual_joint child_link="chassis_link"/> which
    #    requires a bare "chassis_link" frame in TF. But robot_state_publisher
    #    publishes "agent1/chassis_link" (with frame_prefix). This alias fixes
    #    the "Unable to sample any valid states" planning failure.
    # ---------------------------------------------------------------
    moveit_tf_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='moveit_chassis_link_alias',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', [robot_namespace, '/chassis_link'],
            '--child-frame-id', 'chassis_link',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # ---------------------------------------------------------------
    return LaunchDescription([
        declare_robot_namespace,
        declare_pose_x,
        declare_pose_y,
        declare_pose_z,
        declare_world_name,
        declare_controller_base_delay,
        # 1. Robot bringup — immediate
        robot_bringup,
        # 2. Nav2 — delayed 40s
        delayed_nav2,
        # 3. MoveIt — delayed 40s
        delayed_moveit,
        # 4. Task action server — delayed 55s
        delayed_task_action_server,
        # 5. Simple gripper — delayed 55s
        delayed_gripper,
        # 6. Detach all boxes — delayed 60s
        delayed_detach_boxes,
        # 7. TF alias for MoveIt — immediate
        moveit_tf_alias,
    ])
