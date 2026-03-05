import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Configure package paths and files
    pkg_name = 'swarm_bringup'
    desc_pkg_name = 'swarm_description'
    pkg_share = get_package_share_directory(pkg_name)
    desc_share = get_package_share_directory(desc_pkg_name)
    xacro_path = os.path.join(desc_share, 'urdf', 'camera.urdf.xacro')

  
    

    world_file = LaunchConfiguration('world_file')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_world_file_cmd = DeclareLaunchArgument(
        'world_file',
        default_value=os.path.join(pkg_share, 'worlds', 'test_world_v2.sdf'),
        description='Path to the world file to load'
    )

    robot_desc_pkg_prefix = get_package_prefix('swarm_description')
    resource_path = os.path.join(robot_desc_pkg_prefix, 'share') + ':' + '/home/viswa/Desktop/Gazebo_models'
    
    ign_resource_path = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=resource_path
    )
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_path
    )

    
    # Start Gazebo (Ignition) Server (headless)
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={
            'gz_args': [world_file, ' -r -s'],
        }.items(),
    )
    
    # Start Gazebo (Ignition) GUI
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={
            'gz_args': '-g',
        }.items(),
    )

    robot_description_1 = ParameterValue(
        Command(['xacro ', xacro_path, ' robot_namespace:=', 'cam1']), 
        value_type=str
    )
    
    # Create frame_prefix using PythonExpression for conditional logic
    frame_prefix_1 = PythonExpression(["'", 'cam1', "/' if '", 'cam1', "' else ''"])

    # Create robot_state_publisher node
    robot_state_publisher_node_1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='cam1',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            # --- FIX: Pass the string content directly ---
            'robot_description': robot_description_1,
            'use_sim_time': True,
            'frame_prefix': frame_prefix_1
        }],
    )

    spawn_cam_1 = Node(
        package='ros_gz_sim',
        executable='create',
        namespace='cam1',
        arguments=[
            '-name', 'cam1',
            '-topic', 'robot_description',  # Use relative topic name
            '-x', '0.36',
            '-y', '2.7',
            '-z', '0.05',
            '-R', '0',
            '-P', '0',
            '-Y', '0',
        ],
        output='screen',
    )

    # Static transform from map to camera_base
    static_tf_map_to_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_camera_base',
        output='screen',
        arguments=[
            '--x', '0.36',
            '--y', '2.7',
            '--z', '0.1',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'gz_world',
            '--child-frame-id', 'cam1/camera_base'
        ],
        parameters=[{'use_sim_time': True}]
    )

    static_tf_map_to_gz_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_to_gz_world',
        output='screen',
        arguments=[
            '--x', '0.0',
            '--y', '-3.0',
            '--z', '0.213',
            '--roll', '0',
            '--pitch', '0',
            '--yaw', '0',
            '--frame-id', 'gz_world',
            '--child-frame-id', 'map'
        ],
        parameters=[{'use_sim_time': True}]
    )

    robot_description_2 = ParameterValue(
        Command(['xacro ', xacro_path, ' robot_namespace:=', 'cam2']), 
        value_type=str
    )
    
    # Create frame_prefix using PythonExpression for conditional logic
    frame_prefix_2 = PythonExpression(["'", 'cam2', "/' if '", 'cam2', "' else ''"])

    # Create robot_state_publisher node
    robot_state_publisher_node_2 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='cam2',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            # --- FIX: Pass the string content directly ---
            'robot_description': robot_description_2,
            'use_sim_time': True,
            'frame_prefix': frame_prefix_2
        }],
    )

    spawn_cam_2 = Node(
        package='ros_gz_sim',
        executable='create',
        namespace='cam2',
        arguments=[
            '-name', 'cam2',
            '-topic', 'robot_description',  # Use relative topic name
            '-x', '-7.01',
            '-y', '3.0',
            '-z', '0.29',
            '-R', '0',
            '-P', '0',
            '-Y', '1.57',
        ],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'config', 'swarm.rviz')],
        parameters=[{
            'use_sim_time': True,
        }],
    )
    

    #gz_bridge 
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(pkg_share, 'config', 'world_v2_mobman_swarm3.yaml')
        }]
    )

    # Object State Manager launch (for switching boxes between static/dynamic)
    object_state_manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            pkg_share, 'launch', 'object_state_manager.launch.py')]),
        launch_arguments={
            'world_name': 'construction_world',
            'use_sim_time': 'true',
        }.items(),
    )


    # Create and return launch description
    ld = LaunchDescription([
        declare_use_sim_time_cmd,
        declare_world_file_cmd,
        ign_resource_path,
        gz_resource_path,
        gazebo_server,
        gazebo_gui,
        bridge,
        # Delay cam1 spawn to ensure Gazebo is ready
        TimerAction(
            period=3.0,
            actions=[
                robot_state_publisher_node_1,
                spawn_cam_1,
                static_tf_map_to_camera,
                static_tf_map_to_gz_world,
            ]
        ),
        # Delay cam2 spawn to avoid race condition with cam1
        #TimerAction(
        #    period=6.0,
        #    actions=[
        #        robot_state_publisher_node_2,
        #        spawn_cam_2,
        #    ]
        #),
        # 
        rviz,
        # Delay object_state_manager to ensure Gazebo is ready
        TimerAction(
            period=5.0,
            actions=[object_state_manager_launch]
        ),
    ])

    return ld
