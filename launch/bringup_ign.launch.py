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

    # Define robot configurations
    robots = [
        {'name': 'robot1', 'x': '0.0', 'y': '-3.0', 'z': '0.22', 'nav_x': '0.0', 'nav_y': '0.0', 'nav_z': '0.0', 'spawn_delay': 15.0, 'nav_delay': 30.0},
        {'name': 'robot2', 'x': '0.0', 'y': '-2.0', 'z': '0.22', 'nav_x': '0.0', 'nav_y': '1.0', 'nav_z': '0.0', 'spawn_delay': 25.0, 'nav_delay': 40.0},
        {'name': 'robot3', 'x': '0.0', 'y': '-1.0', 'z': '0.22', 'nav_x': '0.0', 'nav_y': '2.0', 'nav_z': '0.0','spawn_delay': 35.0, 'nav_delay': 50.0},
        {'name': 'robot4', 'x': '-1.0', 'y': '-3.0', 'z': '0.22',  'nav_x': '-1.0', 'nav_y': '0.0', 'nav_z': '0.0', 'spawn_delay': 45.0, 'nav_delay': 60.0},
        {'name': 'robot5', 'x': '-1.0', 'y': '-2.0', 'z': '0.22', 'nav_x': '-1.0', 'nav_y': '1.0', 'nav_z': '0.0', 'spawn_delay': 55.0, 'nav_delay': 70.0},
    ]

    
    robot_instances = []
    
    for robot in robots:
        # Spawn robot with control
        spawn_control = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('swarm_description'), 'launch', 'jackal_control.launch.py')]),
            launch_arguments={
                'robot_namespace': robot['name'],
                'pose_x': robot['x'],
                'pose_y': robot['y'],
                'pose_z': robot['z'],
            }.items(),
        )

        # Spawn navigation
        spawn_nav = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
                get_package_share_directory('swarm_description'), 'launch', 'jackal_nav2.launch.py')]),
            launch_arguments={
                'robot_namespace': robot['name'],
                'pose_x': robot['nav_x'],
                'pose_y': robot['nav_y'],
                'pose_z': robot['nav_z'],
            }.items(),
        )

        # Delayed actions
        delayed_spawn = TimerAction(
            period=robot['spawn_delay'],
            actions=[spawn_control]
        )

        delayed_nav = TimerAction(
            period=robot['nav_delay'],
            actions=[spawn_nav]
        )

        robot_instances.append(delayed_spawn)
        robot_instances.append(delayed_nav)

    
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share, 'config', 'mobile_swarm.rviz')],
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
            'config_file': os.path.join(pkg_share, 'config', 'mobile_swarm_bridge.yaml')
        }]
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
        rviz,
    ])
    
    #for action in robot_instances:
    #    ld.add_action(action)

    return ld
