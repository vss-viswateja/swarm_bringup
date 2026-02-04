#!/usr/bin/env python3
"""
Simple Gripper Service Node

This node provides ROS 2 services to simulate a vacuum gripper by using
the DetachableJoint plugin in Gazebo Fortress. It sends attach/detach
commands via Ignition Transport topics.

Services:
    /attach_object (AttachObject): Attach an object to the robot gripper
    /detach_object (DetachObject): Detach an object from the gripper

Parameters:
    world_name (str): Gazebo world name
    robot_model (str): Robot model name used in URDF detachable joint topics
                       (e.g., 'ur5_assembly', 'mobile_manipulator')

Note:
    The robot URDF must include DetachableJoint plugins from detachable_joints.xacro
    for this service to work. The attach/detach topics are:
        /<robot_model>/attach_box_<id>
        /<robot_model>/detach_box_<id>
"""

import subprocess
import time
import re

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from swarm_interfaces.srv import AttachObject, DetachObject


class SimpleGripperService(Node):
    """
    A ROS 2 node that simulates a vacuum gripper using DetachableJoint plugin.
    
    Uses Ignition Transport topics to send attach/detach commands to the
    DetachableJoint plugins defined in the robot's URDF.
    """

    def __init__(self):
        super().__init__('simple_gripper_service')

        # Declare parameters
        self.declare_parameter('world_name', 'construction_world')
        self.declare_parameter('robot_model', 'mobile_manipulator')
        
        self.world_name = self.get_parameter('world_name').get_parameter_value().string_value
        self.robot_model = self.get_parameter('robot_model').get_parameter_value().string_value
        
        if not self.world_name:
            self.get_logger().warn('No world_name specified!')
        if not self.robot_model:
            self.get_logger().warn('No robot_model specified! Using "mobile_manipulator" as default.')
            self.robot_model = 'mobile_manipulator'

        # Use reentrant callback group for service calls
        self.callback_group = ReentrantCallbackGroup()

        # Track attached objects for reference
        # Format: {'aruco_box_30': {'attached_at': timestamp, 'robot_model': 'mobile_manipulator'}}
        self.attached_objects = {}

        # Create services
        self.attach_srv = self.create_service(
            AttachObject,
            'attach_object',
            self.attach_callback,
            callback_group=self.callback_group
        )
        self.detach_srv = self.create_service(
            DetachObject,
            'detach_object',
            self.detach_callback,
            callback_group=self.callback_group
        )

        # Get namespace for accurate logging
        ns = self.get_namespace()
        ns_prefix = ns if ns != '/' else ''
        
        self.get_logger().info('Simple Gripper Service ready!')
        self.get_logger().info(f'  World: {self.world_name}')
        self.get_logger().info(f'  Robot Model (for ign topics): {self.robot_model}')
        self.get_logger().info(f'  Services: {ns_prefix}/attach_object, {ns_prefix}/detach_object')
        self.get_logger().info(f'  Ign attach topic pattern: /{self.robot_model}/attach_box_<id>')
        self.get_logger().info(f'  Ign detach topic pattern: /{self.robot_model}/detach_box_<id>')

    def attach_callback(self, request: AttachObject.Request, response: AttachObject.Response):
        """
        Service callback to attach an object to the robot gripper.
        
        Uses the DetachableJoint plugin by publishing to the attach topic.
        The request parameters parent_model and parent_link are used for logging
        but the actual robot_model comes from the node parameter since the
        DetachableJoint topics are defined in the URDF.
        """
        child_model = request.child_model  # e.g., 'aruco_box_30'
        
        # Extract box ID from the model name (e.g., 'aruco_box_30' -> '30')
        box_id = self._extract_box_id(child_model)
        if box_id is None:
            response.success = False
            response.message = f"Could not extract box ID from '{child_model}'. Expected format: 'aruco_box_<id>'"
            response.joint_name = ""
            self.get_logger().error(response.message)
            return response

        self.get_logger().info(
            f"Attaching: {child_model} to {request.parent_model}::{request.parent_link} "
            f"(using topic: /{self.robot_model}/attach_box_{box_id})"
        )

        # Send attach command via ign topic
        success, message = self._send_attach_command(box_id)

        if success:
            # Track the attachment
            joint_name = f"detachable_joint_{child_model}"
            self.attached_objects[child_model] = {
                'attached_at': time.time(),
                'robot_model': self.robot_model,
                'box_id': box_id
            }
            response.success = True
            response.message = f"Successfully attached {child_model} to {request.parent_model}"
            response.joint_name = joint_name
            self.get_logger().info(f"Attached successfully: {child_model}")
        else:
            response.success = False
            response.message = message
            response.joint_name = ""
            self.get_logger().error(f"Attach failed: {message}")

        return response

    def detach_callback(self, request: DetachObject.Request, response: DetachObject.Response):
        """
        Service callback to detach an object from the robot gripper.
        
        The joint_name should be in format 'detachable_joint_aruco_box_<id>'
        or just 'aruco_box_<id>'.
        """
        joint_name = request.joint_name

        self.get_logger().info(f"Detaching: {joint_name}")

        # Extract box ID from joint name
        # Handle both 'detachable_joint_aruco_box_30' and 'aruco_box_30' formats
        box_id = self._extract_box_id(joint_name)
        if box_id is None:
            response.success = False
            response.message = f"Could not extract box ID from '{joint_name}'. Expected format containing 'aruco_box_<id>'"
            self.get_logger().error(response.message)
            return response

        # Determine the object name
        object_name = f"aruco_box_{box_id}"

        # Send detach command via ign topic
        success, message = self._send_detach_command(box_id)

        if success:
            # Remove from tracking
            if object_name in self.attached_objects:
                del self.attached_objects[object_name]
            response.success = True
            response.message = f"Successfully detached {object_name}"
            self.get_logger().info(f"Detached successfully: {object_name}")
        else:
            response.success = False
            response.message = message
            self.get_logger().error(f"Detach failed: {message}")

        return response

    def _extract_box_id(self, name: str) -> str:
        """
        Extract the box ID number from a name string.
        
        Handles formats like:
          - 'aruco_box_30' -> '30'
          - 'detachable_joint_aruco_box_30' -> '30'
          - 'gripper_joint_aruco_box_30_123456' -> '30'
        
        Returns None if no ID could be extracted.
        """
        # Look for 'aruco_box_<number>' pattern
        match = re.search(r'aruco_box_(\d+)', name)
        if match:
            return match.group(1)
        return None

    def _send_attach_command(self, box_id: str) -> tuple:
        """
        Send attach command to DetachableJoint plugin via ign topic.
        
        Args:
            box_id: The aruco box ID number (e.g., '30')
            
        Returns:
            Tuple of (success, message)
        """
        topic = f"/{self.robot_model}/attach_box_{box_id}"
        return self._send_ign_topic_command(topic, "attach")

    def _send_detach_command(self, box_id: str) -> tuple:
        """
        Send detach command to DetachableJoint plugin via ign topic.
        
        Args:
            box_id: The aruco box ID number (e.g., '30')
            
        Returns:
            Tuple of (success, message)
        """
        topic = f"/{self.robot_model}/detach_box_{box_id}"
        return self._send_ign_topic_command(topic, "detach")

    def _send_ign_topic_command(self, topic: str, action: str) -> tuple:
        """
        Send an Empty message to an Ignition topic.
        
        Args:
            topic: The Ignition topic to publish to
            action: Description for logging ('attach' or 'detach')
            
        Returns:
            Tuple of (success, message)
        """
        try:
            self.get_logger().debug(f"Sending {action} command to topic: {topic}")
            
            result = subprocess.run(
                ['ign', 'topic', '-t', topic, '-m', 'ignition.msgs.Empty', '-p', ''],
                capture_output=True, text=True, timeout=5
            )
            
            # ign topic doesn't return meaningful output on success
            # Check for errors in stderr
            if result.returncode != 0:
                error_msg = result.stderr if result.stderr else f"Command returned code {result.returncode}"
                self.get_logger().error(f"ign topic command failed: {error_msg}")
                return False, f"ign topic command failed: {error_msg}"
            
            self.get_logger().debug(f"Successfully sent {action} command to {topic}")
            return True, f"Successfully sent {action} command"
            
        except subprocess.TimeoutExpired:
            return False, f"Timeout sending {action} command to {topic}"
        except Exception as e:
            return False, f"Exception sending {action} command: {str(e)}"


def main(args=None):
    rclpy.init(args=args)

    node = SimpleGripperService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
