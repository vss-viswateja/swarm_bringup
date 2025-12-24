import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
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
        default_value=os.path.join(pkg_share, 'worlds', 'test_world_v1.sdf'),
        description='Path to the world file to load'
    )

    robot_desc_pkg_prefix = get_package_prefix('swarm_description')
    resource_path = os.path.join(robot_desc_pkg_prefix, 'share')
    
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
            'gz_args': ['empty.sdf', ' -r -s'],
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

    spawn_jackal_with_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('swarm_description'), 'launch', 'jackal_control.launch.py')])
    )

    spawn_ur5_with_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('swarm_description'), 'launch', 'ur5_control.launch.py')])
    )

    delayed_jackal_spawner = TimerAction(
        period=10.0,
        actions=[spawn_jackal_with_control]
    )
    
    delayed_ur5_spawner = TimerAction(
        period=10.0,
        actions=[spawn_ur5_with_control]
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
            'config_file': os.path.join(pkg_share, 'config', 'swarm_bridge.yaml')
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
        #delayed_jackal_spawner,
        #delayed_ur5_spawner,
        #rviz,
        bridge,

        
    ])
