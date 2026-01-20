#!/usr/bin/env python3
"""
Simple Gripper Service Node

This node provides ROS 2 services to simulate a vacuum gripper by dynamically
spawning and removing fixed joints between the robot end-effector and objects
in Gazebo Fortress.

Services:
    /attach_object (AttachObject): Attach an object to the robot gripper
    /detach_object (DetachObject): Detach an object from the gripper

Parameters:
    world_name (str): Gazebo world name (default: "construction_world_v3")
"""

import os
import subprocess
import tempfile
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from swarm_interfaces.srv import AttachObject, DetachObject


class SimpleGripperService(Node):
    """
    A ROS 2 node that simulates a vacuum gripper using dynamic joint spawning.
    
    The "attach" operation spawns a fixed joint between the robot's end-effector
    and the target object. The "detach" operation removes this joint.
    """

    # SDF template for the gripper attachment
    # We create a minimal model with a virtual link and a fixed joint connecting
    # the parent (robot link) to the child (object link)
    # The virtual link has negligible mass and acts as the joint anchor
    JOINT_SDF_TEMPLATE = '''<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{joint_name}">
    <static>false</static>
    <pose>{child_x} {child_y} {child_z} 0 0 0</pose>
    
    <!-- Virtual link that anchors the joint -->
    <link name="virtual_link">
      <pose>0 0 0 0 0 0</pose>
      <inertial>
        <mass>0.001</mass>
        <inertia>
          <ixx>0.000001</ixx>
          <iyy>0.000001</iyy>
          <izz>0.000001</izz>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyz>0</iyz>
        </inertia>
      </inertial>
    </link>
    
    <!-- Joint connecting gripper model to parent (robot end-effector) -->
    <joint name="attach_to_gripper" type="fixed">
      <parent>{parent_model}::{parent_link}</parent>
      <child>virtual_link</child>
    </joint>
    
    <!-- Joint connecting virtual link to child (object) -->
    <joint name="attach_to_object" type="fixed">
      <parent>virtual_link</parent>
      <child>{child_model}::{child_link}</child>
    </joint>
  </model>
</sdf>'''

    def __init__(self):
        super().__init__('simple_gripper_service')

        # Declare parameters
        self.declare_parameter('world_name', 'construction_world_v3')
        self.world_name = self.get_parameter('world_name').get_parameter_value().string_value

        # Use reentrant callback group for service calls
        self.callback_group = ReentrantCallbackGroup()

        # Track active joints for cleanup
        # Format: {'joint_name': {'parent': 'robot::link', 'child': 'object::link', 'created_at': timestamp}}
        self.active_joints = {}

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

        self.get_logger().info('Simple Gripper Service ready!')
        self.get_logger().info(f'  World: {self.world_name}')
        self.get_logger().info('  Services: /attach_object, /detach_object')

    def attach_callback(self, request: AttachObject.Request, response: AttachObject.Response):
        """
        Service callback to attach an object to the robot gripper.
        
        Creates a fixed joint between the parent (robot link) and child (object link).
        """
        parent_model = request.parent_model
        parent_link = request.parent_link
        child_model = request.child_model
        child_link = request.child_link if request.child_link else 'link'

        self.get_logger().info(
            f"Attaching: {parent_model}::{parent_link} -> {child_model}::{child_link}"
        )

        # Get child object's current position for the model pose
        child_pos = self._get_model_position(child_model)
        if child_pos is None:
            child_pos = (0.0, 0.0, 0.0)
            self.get_logger().warn(f"Could not get position for {child_model}, using origin")

        # Generate unique joint name
        timestamp = int(time.time() * 1000)
        joint_name = f"gripper_joint_{child_model}_{timestamp}"

        # Generate SDF for the joint
        sdf_content = self.JOINT_SDF_TEMPLATE.format(
            joint_name=joint_name,
            parent_model=parent_model,
            parent_link=parent_link,
            child_model=child_model,
            child_link=child_link,
            child_x=child_pos[0],
            child_y=child_pos[1],
            child_z=child_pos[2]
        )

        self.get_logger().debug(f"Generated SDF:\n{sdf_content}")

        # Spawn the joint in Gazebo
        success, message = self._spawn_joint(joint_name, sdf_content)

        if success:
            # Track the joint for later cleanup
            self.active_joints[joint_name] = {
                'parent': f"{parent_model}::{parent_link}",
                'child': f"{child_model}::{child_link}",
                'created_at': time.time()
            }
            response.success = True
            response.message = f"Successfully attached {child_model} to {parent_model}"
            response.joint_name = joint_name
            self.get_logger().info(f"Attached successfully. Joint name: {joint_name}")
        else:
            response.success = False
            response.message = message
            response.joint_name = ""
            self.get_logger().error(f"Attach failed: {message}")

        return response

    def detach_callback(self, request: DetachObject.Request, response: DetachObject.Response):
        """
        Service callback to detach an object from the robot gripper.
        
        Removes the joint entity from Gazebo.
        """
        joint_name = request.joint_name

        self.get_logger().info(f"Detaching joint: {joint_name}")

        # Check if joint exists in our tracking
        if joint_name not in self.active_joints:
            self.get_logger().warn(f"Joint {joint_name} not in active joints, but will try to remove anyway")

        # Remove the joint from Gazebo
        success, message = self._remove_joint(joint_name)

        if success:
            # Remove from tracking
            if joint_name in self.active_joints:
                del self.active_joints[joint_name]
            response.success = True
            response.message = f"Successfully detached joint {joint_name}"
            self.get_logger().info(f"Detached successfully: {joint_name}")
        else:
            response.success = False
            response.message = message
            self.get_logger().error(f"Detach failed: {message}")

        return response

    def _get_model_position(self, model_name: str) -> tuple:
        """
        Get the current position of a model from Gazebo.
        
        Args:
            model_name: Name of the model
            
        Returns:
            Tuple of (x, y, z) or None if failed
        """
        try:
            import re
            
            result = subprocess.run(
                ['ign', 'model', '-m', model_name, '-p'],
                capture_output=True, text=True, timeout=5
            )
            
            output = result.stdout
            
            # Parse position: look for [x y z] pattern
            # Format: [-6.849470 2.652150 0.859400]
            bracket_pattern = r'\[([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\]'
            matches = re.findall(bracket_pattern, output)
            
            if matches:
                x = float(matches[0][0])
                y = float(matches[0][1])
                z = float(matches[0][2])
                self.get_logger().debug(f"Got position for {model_name}: ({x}, {y}, {z})")
                return (x, y, z)
            else:
                self.get_logger().warn(f"Could not parse position for {model_name}")
                return None
                
        except Exception as e:
            self.get_logger().warn(f"Exception getting position for {model_name}: {e}")
            return None

    def _spawn_joint(self, joint_name: str, sdf_content: str) -> tuple[bool, str]:
        """
        Spawn a joint entity in Gazebo using the create service.
        
        Args:
            joint_name: Name of the joint to create
            sdf_content: SDF XML content for the joint
            
        Returns:
            Tuple of (success, message)
        """
        sdf_file = None
        try:
            # Write SDF to a temporary file
            sdf_file = f'/tmp/gripper_joint_{int(time.time() * 1000)}.sdf'
            with open(sdf_file, 'w') as f:
                f.write(sdf_content)

            self.get_logger().debug(f"Created SDF file: {sdf_file}")

            # Call Gazebo create service
            service_name = f'/world/{self.world_name}/create'
            req_type = 'ignition.msgs.EntityFactory'
            rep_type = 'ignition.msgs.Boolean'

            request_msg = f'sdf_filename: "{sdf_file}", name: "{joint_name}"'

            self.get_logger().debug(f"Calling service: {service_name}")

            result = subprocess.run(
                ['ign', 'service', '-s', service_name,
                 '--reqtype', req_type, '--reptype', rep_type,
                 '--timeout', '5000', '--req', request_msg],
                capture_output=True, text=True, timeout=10
            )

            self.get_logger().debug(f"Service result: {result.stdout}")

            # Wait for Gazebo to read the file
            time.sleep(0.3)

            # Cleanup temp file
            if sdf_file and os.path.exists(sdf_file):
                os.unlink(sdf_file)

            if 'data: true' in result.stdout.lower():
                return True, "Joint created successfully"
            else:
                error_msg = result.stderr if result.stderr else result.stdout
                return False, f"Gazebo service returned false: {error_msg}"

        except subprocess.TimeoutExpired:
            return False, "Timeout calling Gazebo create service"
        except Exception as e:
            return False, f"Exception spawning joint: {str(e)}"
        finally:
            # Ensure cleanup
            if sdf_file and os.path.exists(sdf_file):
                try:
                    os.unlink(sdf_file)
                except:
                    pass

    def _remove_joint(self, joint_name: str) -> tuple[bool, str]:
        """
        Remove a joint entity from Gazebo using the remove service.
        
        Args:
            joint_name: Name of the joint to remove
            
        Returns:
            Tuple of (success, message)
        """
        try:
            service_name = f'/world/{self.world_name}/remove'
            req_type = 'ignition.msgs.Entity'
            rep_type = 'ignition.msgs.Boolean'

            # Entity type 2 = MODEL (we create a model that contains the joint)
            request_msg = f'name: "{joint_name}", type: 2'

            self.get_logger().debug(f"Calling remove service for joint: {joint_name}")

            result = subprocess.run(
                ['ign', 'service', '-s', service_name,
                 '--reqtype', req_type, '--reptype', rep_type,
                 '--timeout', '5000', '--req', request_msg],
                capture_output=True, text=True, timeout=10
            )

            self.get_logger().debug(f"Remove result: {result.stdout}")

            if 'data: true' in result.stdout.lower():
                return True, "Joint removed successfully"
            else:
                error_msg = result.stderr if result.stderr else result.stdout
                return False, f"Gazebo service returned false: {error_msg}"

        except subprocess.TimeoutExpired:
            return False, "Timeout calling Gazebo remove service"
        except Exception as e:
            return False, f"Exception removing joint: {str(e)}"


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
