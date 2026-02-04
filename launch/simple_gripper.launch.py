"""
Launch file for the Simple Gripper Service node.

This node provides services to attach/detach objects to the robot gripper
using the DetachableJoint plugin in Gazebo.

Usage:
    # Single robot (default)
    ros2 launch swarm_bringup simple_gripper.launch.py

    # Multi-robot with namespace
    ros2 launch swarm_bringup simple_gripper.launch.py robot_namespace:=agent1 robot_model:=agent1
    ros2 launch swarm_bringup simple_gripper.launch.py robot_namespace:=agent2 robot_model:=agent2

Note:
    - robot_namespace: ROS 2 namespace for the service (e.g., /agent1/attach_object)
    - robot_model: Must match the robot_namespace used when spawning the robot,
                   as this determines the DetachableJoint ign topic names
                   (e.g., /agent1/attach_box_30)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='mobman',
        description='Namespace for the gripper service (e.g., "agent1"). '
                    'Services will be at /<namespace>/attach_object'
    )

    world_name_arg = DeclareLaunchArgument(
        'world_name',
        default_value='construction_world',
        description='Name of the Gazebo world (must match the world name in SDF)'
    )

    robot_model_arg = DeclareLaunchArgument(
        'robot_model',
        default_value='mobman',
        description='Robot model name for DetachableJoint topics. Must match the '
                    'robot_namespace used when spawning (e.g., "agent1", "mobman")'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    # Simple Gripper Service Node
    simple_gripper_node = Node(
        package='swarm_bringup',
        executable='simple_gripper_service',
        name='simple_gripper_service',
        namespace=LaunchConfiguration('robot_namespace'),
        output='screen',
        parameters=[{
            'world_name': LaunchConfiguration('world_name'),
            'robot_model': LaunchConfiguration('robot_model'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        robot_namespace_arg,
        world_name_arg,
        robot_model_arg,
        use_sim_time_arg,
        simple_gripper_node,
    ])
