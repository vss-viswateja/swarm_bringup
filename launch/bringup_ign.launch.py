import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, SetEnvironmentVariable, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Configure package paths and files
    pkg_name = 'swarm_bringup'
    desc_pkg_name = 'swarm_description'
    robot_1_desc_pkg_name = 'jackal_description'
    robot_2_desc_pkg_name = 'ur5_description'
    pkg_share = get_package_share_directory(pkg_name)
    desc_share = get_package_share_directory(desc_pkg_name)
  
    

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

    spawn_agent1_with_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('swarm_description'), 'launch', 'jackal_control.launch.py')]),
            launch_arguments={
                'robot_namespace': 'robot1',
            }.items(),
    )

    spawn_agent2_with_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('swarm_description'), 'launch', 'jackal_control.launch.py')]),
            launch_arguments={
                'robot_namespace': 'robot2',
                'pose_y': '-2.0',
            }.items(),  
    )

    spawn_agent1_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('swarm_description'), 'launch', 'jackal_nav2.launch.py')]),
        launch_arguments={
            'robot_namespace': 'robot1',
            'pose_x': '0.0',
            'pose_y': '0.0',
            'pose_z': '0.0',
        }.items(),
    )

    spawn_agent2_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('swarm_description'), 'launch', 'jackal_nav2.launch.py')]),
        launch_arguments={
            'robot_namespace': 'robot2',
            'pose_x': '0.0',
            'pose_y': '1.0',
            'pose_z': '0.0',
        }.items(),
    )

    # Staggered robot spawning to avoid race conditions
    # Robot 1 spawns 15s after Gazebo starts (enough time for world to load)
    delayed_agent1_spawner = TimerAction(
        period=15.0,
        actions=[spawn_agent1_with_control]
    )
    
    # Robot 2 spawns 10s after Robot 1 (25s total) to ensure no conflicts
    delayed_agent2_spawner = TimerAction(
        period=25.0,
        actions=[spawn_agent2_with_control]
    )

    # Navigation starts after Control/EKF is ready (Control starts at 15s + ~12s init = ~27s)
    delayed_agent1_nav = TimerAction(
        period=30.0,
        actions=[spawn_agent1_nav]
    )

    # Navigation starts after Control/EKF is ready (Control starts at 25s + ~12s init = ~37s)
    delayed_agent2_nav = TimerAction(
        period=40.0,
        actions=[spawn_agent2_nav]
    )

    '''
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'config', 'default_view.rviz')],
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }],
    )
    '''

    #gz_bridge 
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(pkg_share, 'config', 'mobile_swarm_bridge.yaml')
        }]
    )
    


    # Create and return launch description
    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_world_file_cmd,
        ign_resource_path,
        gz_resource_path,
        gazebo_server,
        gazebo_gui,
        bridge,
        # Staggered robot spawning - Robot1 at 15s, Robot2 at 25s
        delayed_agent1_spawner,
        delayed_agent2_spawner,
        delayed_agent1_nav,
        delayed_agent2_nav,
    ])
