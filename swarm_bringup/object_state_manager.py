#!/usr/bin/env python3
"""
Object State Manager Node

This node manages the dynamic/static state of aruco boxes in the Gazebo simulation.
It allows switching boxes between static (frozen) and dynamic (manipulable) states
to optimize physics performance while still allowing robot manipulation.

Services:
    /set_box_state (SetBoxState): Switch a box between static and dynamic states
    /get_box_pose (GetBoxPose): Get the current pose of a box

Architecture:
    - Uses Ignition Transport services directly via subprocess (ign service)
    - Subscribes to pose topics to track model positions
    - Maintains internal state tracking for all managed boxes
"""

import os
import subprocess
import tempfile
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Pose
from swarm_interfaces.srv import SetBoxState, GetBoxPose

# For subscribing to Ignition pose messages via ros_gz_bridge
from tf2_msgs.msg import TFMessage


class ObjectStateManager(Node):
    """
    Manages the static/dynamic state of objects in Gazebo simulation.
    Uses direct Ignition Transport service calls for reliability.
    """

    # Path pattern to find aruco box models in Ignition Fuel cache
    FUEL_CACHE_PATH = os.path.expanduser(
        "~/.ignition/fuel/fuel.gazebosim.org/viswatejabottu/models"
    )

    def __init__(self):
        super().__init__('object_state_manager')

        # Declare parameters
        self.declare_parameter('world_name', 'construction_world_v3')
        self.world_name = self.get_parameter('world_name').get_parameter_value().string_value

        # Use reentrant callback group for nested service calls
        self.callback_group = ReentrantCallbackGroup()

        # Internal state tracking
        # Format: {'aruco_box_26': {'is_dynamic': True, 'pose': Pose(), 'aruco_id': 26}}
        self.box_states = {}

        # Current poses of all models (updated from TF or pose topic)
        self.model_poses = {}

        # Initial poses from world file (for static boxes that don't publish poses)
        self.initial_poses = {}
        self._load_initial_poses()

        # Create our custom services
        self.set_state_srv = self.create_service(
            SetBoxState,
            'set_box_state',
            self.set_box_state_callback,
            callback_group=self.callback_group
        )
        self.get_pose_srv = self.create_service(
            GetBoxPose,
            'get_box_pose',
            self.get_box_pose_callback,
            callback_group=self.callback_group
        )

        # Subscribe to dynamic pose info from Ignition (bridged via ros_gz_bridge)
        self.pose_sub = self.create_subscription(
            TFMessage,
            '/world/{}/dynamic_pose/info'.format(self.world_name),
            self.pose_callback,
            10
        )

        # Verify Ignition is running by checking if the world exists
        if self._check_gazebo_running():
            self.get_logger().info('Object State Manager ready!')
            self.get_logger().info(f'  World: {self.world_name}')
            self.get_logger().info('  Services: /set_box_state, /get_box_pose')
            self.get_logger().info(f'  Loaded {len(self.initial_poses)} initial box poses')
        else:
            self.get_logger().warn('Gazebo may not be running. Services will still work once Gazebo starts.')

    def _load_initial_poses(self):
        """
        Load initial poses for aruco boxes from the world file or hardcoded values.
        This is used for static boxes that don't publish their poses.
        """
        # These are the poses from test_world_v3.sdf for quick lookup
        # Format: {box_name: (x, y, z, roll, pitch, yaw)}
        aruco_poses = {
            'aruco_box_26': (-7.79826, 3.13806, 0.8594, 1.63281, -1.57079, 3.14159),
            'aruco_box_27': (-7.81769, 2.699, 0.8594, 0.742797, -1.57079, 0.84311),
            'aruco_box_28': (-7.8, 3.38272, 0.8594, -0.000363, -1.57079, 0.000343),
            'aruco_box_29': (-7.82861, 2.9131, 0.8594, -2.92038, -1.57079, 2.9113),
            'aruco_box_30': (-7.60994, 2.69118, 0.8594, 1.07023, -1.57079, -1.07013),
            'aruco_box_31': (-7.61375, 2.90945, 0.8594, 3.13559, -1.57079, -3.12431),
            'aruco_box_32': (-7.3712, 3.12884, 0.8594, 3.14159, -1.57079, 3.13857),
            'aruco_box_33': (-7.3728, 2.70012, 0.8594, -2.21648, -1.57079, 2.21649),
            'aruco_box_34': (-7.37193, 2.91897, 0.8594, 1.91075, -1.57079, -1.91969),
            'aruco_box_35': (-6.84947, 2.65215, 0.8594, 0.000103, -1.57079, -0.000103),
            'aruco_box_36': (-7.37346, 3.37794, 0.8594, -3.11502, -1.57079, 3.11502),
            'aruco_box_37': (-6.84392, 2.88839, 0.8594, 3.14159, -1.57079, 3.14159),
            'aruco_box_38': (-7.58616, 3.37978, 0.8594, -3.13202, -1.57079, -3.14139),
            'aruco_box_39': (-7.5721, 3.13635, 0.8594, -0.010379, -1.57079, 0.016719),
            'aruco_box_40': (-6.85005, 3.11825, 0.8594, 1.14897, -1.57079, -1.14897),
            'aruco_box_41': (-6.85125, 3.3649, 0.8594, 0.000233, -1.57079, -0.000243),
            'aruco_box_42': (-6.63113, 3.37288, 0.8594, -2.14183, -1.57079, 2.13473),
            'aruco_box_43': (-6.38479, 3.37332, 0.8594, 0.213262, -1.57079, -0.166272),
            'aruco_box_44': (-6.36807, 2.69752, 0.8594, -3e-06, -1.57079, -3e-06),
            'aruco_box_45': (-6.62108, 2.69961, 0.8594, -1.67279, -1.57079, -1.486),
            'aruco_box_46': (-6.62411, 3.14677, 0.8594, -2.63886, -1.57079, -0.50273),
            'aruco_box_47': (-6.37477, 3.14377, 0.8594, -3.14153, -1.57079, 3.14154),
            'aruco_box_48': (-6.37853, 2.91386, 0.8594, -1.636, -1.57079, 1.64431),
            'aruco_box_49': (-6.6123, 2.92226, 0.8594, -2.80506, -1.57079, 2.80486),
            'aruco_box_0': (-7.79826, -3.05534, 0.8594, 1.63281, -1.57079, 3.14159),
            'aruco_box_1': (-7.81769, -2.61628, 0.8594, 0.742797, -1.57079, 0.84311),
            'aruco_box_2': (-7.8, -3.3, 0.8594, -0.000363, -1.57079, 0.000343),
            'aruco_box_3': (-7.82861, -2.83038, 0.8594, -2.92038, -1.57079, 2.9113),
            'aruco_box_4': (-7.60994, -2.60846, 0.8594, 1.07023, -1.57079, -1.07013),
            'aruco_box_5': (-7.61375, -2.82673, 0.8594, 3.13559, -1.57079, -3.12431),
            'aruco_box_6': (-7.3712, -3.04612, 0.8594, 3.14159, -1.57079, 3.13857),
            'aruco_box_7': (-7.3728, -2.6174, 0.8594, -2.21648, -1.57079, 2.21649),
            'aruco_box_8': (-7.37193, -2.83625, 0.8594, 1.91075, -1.57079, -1.91969),
            'aruco_box_9': (-6.84947, -2.61352, 0.8594, 0.000107, -1.57079, -0.000107),
            'aruco_box_10': (-7.37346, -3.29522, 0.8594, -3.11502, -1.57079, 3.11502),
            'aruco_box_11': (-6.84392, -2.83912, 0.8594, 3.14159, -1.57079, 3.14159),
            'aruco_box_12': (-7.58616, -3.29706, 0.8594, -3.13202, -1.57079, -3.14139),
            'aruco_box_13': (-7.5721, -3.05363, 0.8594, -0.010379, -1.57079, 0.016719),
            'aruco_box_14': (-6.85005, -3.069, 0.8594, 1.16019, -1.57079, -1.16019),
            'aruco_box_15': (-6.85125, -3.28218, 0.8594, 0.000233, -1.57079, -0.000243),
            'aruco_box_16': (-6.63113, -3.29016, 0.8594, -2.14183, -1.57079, 2.13473),
            'aruco_box_17': (-6.38479, -3.2906, 0.8594, 0.213262, -1.57079, -0.166272),
            'aruco_box_18': (-6.36807, -2.6148, 0.8594, -3e-06, -1.57079, -3e-06),
            'aruco_box_19': (-6.62108, -2.61689, 0.8594, -1.67279, -1.57079, -1.486),
            'aruco_box_20': (-6.62411, -3.06405, 0.8594, -2.63886, -1.57079, -0.50273),
            'aruco_box_21': (-6.37477, -3.06105, 0.8594, -3.14153, -1.57079, 3.14154),
            'aruco_box_22': (-6.37853, -2.83114, 0.8594, -1.636, -1.57079, 1.64431),
            'aruco_box_23': (-6.6123, -2.83954, 0.8594, -2.80506, -1.57079, 2.80486),
        }
        
        for name, (x, y, z, roll, pitch, yaw) in aruco_poses.items():
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z
            # Convert euler to quaternion
            quat = self._euler_to_quaternion(roll, pitch, yaw)
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]
            self.initial_poses[name] = pose

    def _euler_to_quaternion(self, roll, pitch, yaw):
        """Convert euler angles to quaternion."""
        import math
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy
        
        return (x, y, z, w)

    def _check_gazebo_running(self) -> bool:
        """Check if Gazebo is running by querying the worlds list."""
        try:
            result = subprocess.run(
                ['ign', 'service', '-s', '/gazebo/worlds', '--reqtype', 'ignition.msgs.Empty',
                 '--reptype', 'ignition.msgs.StringMsg_V', '--timeout', '2000', '--req', ''],
                capture_output=True, text=True, timeout=5
            )
            return self.world_name in result.stdout
        except Exception as e:
            self.get_logger().debug(f'Gazebo check failed: {e}')
            return False

    def pose_callback(self, msg: TFMessage):
        """
        Callback for receiving pose updates from Ignition.
        Updates internal pose tracking for all models.
        """
        for transform in msg.transforms:
            model_name = transform.child_frame_id
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation.x = transform.transform.rotation.x
            pose.orientation.y = transform.transform.rotation.y
            pose.orientation.z = transform.transform.rotation.z
            pose.orientation.w = transform.transform.rotation.w
            self.model_poses[model_name] = pose

    def get_box_pose_callback(self, request: GetBoxPose.Request, response: GetBoxPose.Response):
        """
        Service callback to get the current pose of a box.
        """
        box_name = request.box_name

        # Priority: dynamic poses > stored state > initial poses
        if box_name in self.model_poses:
            response.success = True
            response.message = f"Pose retrieved for {box_name} (dynamic)"
            response.pose = self.model_poses[box_name]
        elif box_name in self.box_states and 'pose' in self.box_states[box_name]:
            response.success = True
            response.message = f"Pose retrieved for {box_name} (cached)"
            response.pose = self.box_states[box_name]['pose']
        elif box_name in self.initial_poses:
            response.success = True
            response.message = f"Pose retrieved for {box_name} (initial)"
            response.pose = self.initial_poses[box_name]
        else:
            response.success = False
            response.message = f"Box '{box_name}' not found"
            response.pose = Pose()

        return response

    def set_box_state_callback(self, request: SetBoxState.Request, response: SetBoxState.Response):
        """
        Service callback to switch a box between static and dynamic states.
        
        Workflow:
        1. Get current pose of the box
        2. Delete the current box model
        3. Spawn a new box with the desired static/dynamic setting at the same pose
        """
        box_name = request.box_name
        make_dynamic = request.make_dynamic

        self.get_logger().info(f"Setting {box_name} to {'dynamic' if make_dynamic else 'static'}")

        # Extract aruco ID from name (e.g., "aruco_box_26" -> 26)
        try:
            aruco_id = self._extract_aruco_id(box_name)
        except ValueError as e:
            response.success = False
            response.message = str(e)
            return response

        # Step 1: Get current pose
        current_pose = self._get_current_pose(box_name)
        if current_pose is None:
            response.success = False
            response.message = f"Could not find pose for {box_name}. Check box name."
            return response

        self.get_logger().info(
            f"Current pose: x={current_pose.position.x:.3f}, "
            f"y={current_pose.position.y:.3f}, z={current_pose.position.z:.3f}"
        )

        # Step 2: Delete the current model
        delete_success = self._delete_model(box_name)
        if not delete_success:
            response.success = False
            response.message = f"Failed to delete {box_name}"
            return response

        # Step 3: Spawn new model with desired static setting
        spawn_success = self._spawn_box(box_name, aruco_id, current_pose, static=(not make_dynamic))
        if not spawn_success:
            response.success = False
            response.message = f"Failed to spawn {box_name} as {'dynamic' if make_dynamic else 'static'}"
            return response

        # Update internal state
        self.box_states[box_name] = {
            'is_dynamic': make_dynamic,
            'pose': current_pose,
            'aruco_id': aruco_id
        }

        response.success = True
        response.message = f"Successfully set {box_name} to {'dynamic' if make_dynamic else 'static'}"
        self.get_logger().info(response.message)

        return response

    def _extract_aruco_id(self, box_name: str) -> int:
        """
        Extract the aruco ID number from the box name.
        e.g., "aruco_box_26" -> 26
        """
        parts = box_name.split('_')
        if len(parts) >= 3 and parts[0] == 'aruco' and parts[1] == 'box':
            try:
                return int(parts[2])
            except ValueError:
                pass
        raise ValueError(f"Invalid box name format: {box_name}. Expected 'aruco_box_<number>'")

    def _get_current_pose(self, model_name: str) -> Pose:
        """
        Get the current pose of a model.
        Priority: Live Gazebo pose > tracked poses > cached state > initial poses
        """
        # First, try to get live pose from Gazebo (most accurate)
        live_pose = self._get_pose_from_gazebo(model_name)
        if live_pose is not None:
            self.get_logger().info(f"Got live pose from Gazebo for {model_name}")
            return live_pose
        
        # Fall back to tracked poses from subscription
        if model_name in self.model_poses:
            self.get_logger().info(f"Using tracked pose for {model_name}")
            return self.model_poses[model_name]
        
        # Fall back to cached state
        if model_name in self.box_states and 'pose' in self.box_states[model_name]:
            self.get_logger().info(f"Using cached state pose for {model_name}")
            return self.box_states[model_name]['pose']
        
        # Fall back to initial poses
        if model_name in self.initial_poses:
            self.get_logger().info(f"Using initial pose for {model_name}")
            return self.initial_poses[model_name]
        
        return None

    def _get_pose_from_gazebo(self, model_name: str) -> Pose:
        """
        Fetch the current pose of a model directly from Gazebo using ign model command.
        This gets the live pose, not cached data.
        
        Output format from 'ign model -m <name> -p':
            Model: [514]
              - Name: aruco_box_35
              - Pose [ XYZ (m) ] [ RPY (rad) ]:
                [-6.849470 2.652150 0.859400]
                [0.000103 -1.570790 -0.000103]
        """
        try:
            import re
            import math
            
            # Use 'ign model -p' command to get pose
            result = subprocess.run(
                ['ign', 'model', '-m', model_name, '-p'],
                capture_output=True, text=True, timeout=5
            )
            
            output = result.stdout
            self.get_logger().info(f"ign model output for {model_name}: {output[:200]}...")
            
            if not output or 'error' in output.lower():
                self.get_logger().warn(f"ign model command failed: {result.stderr}")
                return None
            
            pose = Pose()
            
            # Parse position: look for [x y z] pattern after "Pose"
            # Format: [-6.849470 2.652150 0.859400]
            lines = output.split('\n')
            xyz_line = None
            rpy_line = None
            
            for i, line in enumerate(lines):
                # Find the line with XYZ values (first bracketed numbers after Pose header)
                if 'Pose' in line and 'XYZ' in line:
                    # Next line should be XYZ, line after that should be RPY
                    if i + 1 < len(lines):
                        xyz_line = lines[i + 1]
                    if i + 2 < len(lines):
                        rpy_line = lines[i + 2]
                    break
            
            if not xyz_line:
                # Try alternative pattern: look for any line with bracketed numbers
                bracket_pattern = r'\[([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\]'
                matches = re.findall(bracket_pattern, output)
                if len(matches) >= 1:
                    pose.position.x = float(matches[0][0])
                    pose.position.y = float(matches[0][1])
                    pose.position.z = float(matches[0][2])
                    if len(matches) >= 2:
                        roll = float(matches[1][0])
                        pitch = float(matches[1][1])
                        yaw = float(matches[1][2])
                        quat = self._euler_to_quaternion(roll, pitch, yaw)
                        pose.orientation.x = quat[0]
                        pose.orientation.y = quat[1]
                        pose.orientation.z = quat[2]
                        pose.orientation.w = quat[3]
                    else:
                        pose.orientation.w = 1.0
                    
                    self.get_logger().info(
                        f"Parsed pose for {model_name}: pos=({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f})"
                    )
                    return pose
                else:
                    self.get_logger().warn(f"Could not parse position from: {output}")
                    return None
            
            # Parse XYZ line: "    [-6.849470 2.652150 0.859400]"
            xyz_pattern = r'\[([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\]'
            xyz_match = re.search(xyz_pattern, xyz_line)
            
            if xyz_match:
                pose.position.x = float(xyz_match.group(1))
                pose.position.y = float(xyz_match.group(2))
                pose.position.z = float(xyz_match.group(3))
            else:
                self.get_logger().warn(f"Could not parse XYZ from line: {xyz_line}")
                return None
            
            # Parse RPY line: "    [0.000103 -1.570790 -0.000103]"
            if rpy_line:
                rpy_match = re.search(xyz_pattern, rpy_line)  # Same pattern works
                if rpy_match:
                    roll = float(rpy_match.group(1))
                    pitch = float(rpy_match.group(2))
                    yaw = float(rpy_match.group(3))
                    # Convert RPY to quaternion
                    quat = self._euler_to_quaternion(roll, pitch, yaw)
                    pose.orientation.x = quat[0]
                    pose.orientation.y = quat[1]
                    pose.orientation.z = quat[2]
                    pose.orientation.w = quat[3]
                else:
                    pose.orientation.w = 1.0
            else:
                pose.orientation.w = 1.0
            
            self.get_logger().info(
                f"Parsed pose for {model_name}: pos=({pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}), "
                f"orient=({pose.orientation.x:.3f}, {pose.orientation.y:.3f}, {pose.orientation.z:.3f}, {pose.orientation.w:.3f})"
            )
            
            return pose
            
        except subprocess.TimeoutExpired:
            self.get_logger().warn(f"Timeout getting pose from Gazebo for {model_name}")
            return None
        except Exception as e:
            self.get_logger().error(f"Exception getting pose from Gazebo for {model_name}: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return None

    def _delete_model(self, model_name: str) -> bool:
        """
        Delete a model from the simulation using Ignition service call.
        """
        try:
            # Use ign service to call the remove service
            service_name = f'/world/{self.world_name}/remove'
            req_type = 'ignition.msgs.Entity'
            rep_type = 'ignition.msgs.Boolean'
            
            # Entity type 2 = MODEL
            request_msg = f'name: "{model_name}", type: 2'
            
            result = subprocess.run(
                ['ign', 'service', '-s', service_name,
                 '--reqtype', req_type, '--reptype', rep_type,
                 '--timeout', '5000', '--req', request_msg],
                capture_output=True, text=True, timeout=10
            )
            
            if 'data: true' in result.stdout.lower():
                self.get_logger().info(f"Deleted model: {model_name}")
                # Remove from pose tracking
                if model_name in self.model_poses:
                    del self.model_poses[model_name]
                return True
            else:
                self.get_logger().error(f"Delete failed: {result.stdout} {result.stderr}")
                return False
                
        except subprocess.TimeoutExpired:
            self.get_logger().error(f"Delete timed out for {model_name}")
            return False
        except Exception as e:
            self.get_logger().error(f"Delete exception: {e}")
            return False

    def _spawn_box(self, box_name: str, aruco_id: int, pose: Pose, static: bool) -> bool:
        """
        Spawn an aruco box at the specified pose with the given static setting.
        Uses Ignition service call with a persistent temp file.
        """
        sdf_file = None
        try:
            # Generate the SDF for the box
            sdf_string = self._generate_box_sdf(aruco_id, static)
            
            # Write SDF to a temporary file - don't auto-delete, we'll clean up later
            # Use a fixed location to avoid race conditions
            import time
            sdf_file = f'/tmp/aruco_box_{aruco_id}_{int(time.time())}.sdf'
            with open(sdf_file, 'w') as f:
                f.write(sdf_string)
            
            self.get_logger().info(f"Created SDF file: {sdf_file}")
            
            service_name = f'/world/{self.world_name}/create'
            req_type = 'ignition.msgs.EntityFactory'
            rep_type = 'ignition.msgs.Boolean'
            
            # Build the request message with SDF file path
            request_msg = (
                f'sdf_filename: "{sdf_file}", '
                f'name: "{box_name}", '
                f'pose: {{'
                f'position: {{x: {pose.position.x}, y: {pose.position.y}, z: {pose.position.z}}}, '
                f'orientation: {{x: {pose.orientation.x}, y: {pose.orientation.y}, '
                f'z: {pose.orientation.z}, w: {pose.orientation.w}}}'
                f'}}'
            )
            
            self.get_logger().info(f"Spawning with request: {request_msg}")
            
            result = subprocess.run(
                ['ign', 'service', '-s', service_name,
                 '--reqtype', req_type, '--reptype', rep_type,
                 '--timeout', '5000', '--req', request_msg],
                capture_output=True, text=True, timeout=10
            )
            
            self.get_logger().info(f"Spawn result stdout: {result.stdout}")
            if result.stderr:
                self.get_logger().warn(f"Spawn result stderr: {result.stderr}")
            
            # Wait a moment for Gazebo to read the file before cleaning up
            import time
            time.sleep(0.5)
            
            # Clean up temp file
            if sdf_file and os.path.exists(sdf_file):
                os.unlink(sdf_file)
            
            if 'data: true' in result.stdout.lower():
                self.get_logger().info(f"Spawned model: {box_name} (static={static})")
                return True
            else:
                self.get_logger().error(f"Spawn failed: {result.stdout} {result.stderr}")
                return False
                
        except subprocess.TimeoutExpired:
            self.get_logger().error(f"Spawn timed out for {box_name}")
            return False
        except Exception as e:
            self.get_logger().error(f"Spawn exception: {e}")
            return False
        finally:
            # Ensure cleanup even on error
            if sdf_file and os.path.exists(sdf_file):
                try:
                    os.unlink(sdf_file)
                except:
                    pass

    def _generate_box_sdf(self, aruco_id: int, static: bool) -> str:
        """
        Generate an SDF string for an aruco box with the specified ID and static setting.
        """
        static_str = "true" if static else "false"
        
        # Try to find the texture file path
        model_path = os.path.join(
            self.FUEL_CACHE_PATH,
            f"aruco box {aruco_id}",
            "1"
        )
        texture_path = f"aruco_{aruco_id}.png"
        
        # Check if model exists locally
        if os.path.exists(model_path):
            texture_uri = os.path.join(model_path, texture_path)
        else:
            texture_uri = texture_path

        sdf = f'''<?xml version="1.0"?>
<sdf version="1.9">
    <model name="box_{aruco_id}">
        <pose>0 0 0.1 0 0 0</pose>
        <static>{static_str}</static>
        <link name="link">
            <inertial>
                <mass>0.5</mass>
                <inertia>
                    <ixx>0.006667</ixx>
                    <iyy>0.006667</iyy>
                    <izz>0.006667</izz>
                    <ixy>0</ixy>
                    <ixz>0</ixz>
                    <iyz>0</iyz>
                </inertia>
            </inertial>
      
            <!-- Collision geometry -->
            <collision name="collision">
                <geometry>
                    <box>
                        <size>0.2 0.2 0.2</size>
                    </box>
                </geometry>
                <surface>
                    <friction>
                        <ode>
                            <mu>0.8</mu>
                            <mu2>0.8</mu2>
                        </ode>
                    </friction>
                    <contact>
                        <ode>
                            <kp>1e6</kp>
                            <kd>100</kd>
                        </ode>
                    </contact>
                </surface>
            </collision>
      
            <!-- Visual geometry - Front face with ArUco tag -->
            <visual name="visual_front">
                <pose>0.1 0 0 0 0 0</pose>
                <geometry>
                    <box>
                        <size>0.001 0.2 0.2</size>
                    </box>
                </geometry>
                <material>
                    <ambient>1 1 1 1</ambient>
                    <diffuse>1 1 1 1</diffuse>
                    <specular>0.1 0.1 0.1 1</specular>
                    <pbr>
                        <metal>
                            <albedo_map>{texture_uri}</albedo_map>
                        </metal>
                    </pbr>
                </material>
            </visual>
      
            <!-- Visual geometry - Back face (brick color) -->
            <visual name="visual_back">
                <pose>-0.1 0 0 0 0 0</pose>
                <geometry>
                    <box>
                        <size>0.001 0.2 0.2</size>
                    </box>
                </geometry>
                <material>
                    <ambient>0.6 0.3 0.2 1</ambient>
                    <diffuse>0.7 0.35 0.25 1</diffuse>
                    <specular>0.1 0.05 0.05 1</specular>
                </material>
            </visual>
      
            <!-- Visual geometry - Left face (brick color) -->
            <visual name="visual_left">
                <pose>0 0.1 0 0 0 0</pose>
                <geometry>
                    <box>
                        <size>0.2 0.001 0.2</size>
                    </box>
                </geometry>
                <material>
                    <ambient>0.6 0.3 0.2 1</ambient>
                    <diffuse>0.7 0.35 0.25 1</diffuse>
                    <specular>0.1 0.05 0.05 1</specular>
                </material>
            </visual>
      
            <!-- Visual geometry - Right face (brick color) -->
            <visual name="visual_right">
                <pose>0 -0.1 0 0 0 0</pose>
                <geometry>
                    <box>
                        <size>0.2 0.001 0.2</size>
                    </box>
                </geometry>
                <material>
                    <ambient>0.6 0.3 0.2 1</ambient>
                    <diffuse>0.7 0.35 0.25 1</diffuse>
                    <specular>0.1 0.05 0.05 1</specular>
                </material>
            </visual>
      
            <!-- Visual geometry - Top face (brick color) -->
            <visual name="visual_top">
                <pose>0 0 0.1 0 0 0</pose>
                <geometry>
                    <box>
                        <size>0.2 0.2 0.001</size>
                    </box>
                </geometry>
                <material>
                    <ambient>0.6 0.3 0.2 1</ambient>
                    <diffuse>0.7 0.35 0.25 1</diffuse>
                    <specular>0.1 0.05 0.05 1</specular>
                </material>
            </visual>
      
            <!-- Visual geometry - Bottom face (brick color) -->
            <visual name="visual_bottom">
                <pose>0 0 -0.1 0 0 0</pose>
                <geometry>
                    <box>
                        <size>0.2 0.2 0.001</size>
                    </box>
                </geometry>
                <material>
                    <ambient>0.6 0.3 0.2 1</ambient>
                    <diffuse>0.7 0.35 0.25 1</diffuse>
                    <specular>0.1 0.05 0.05 1</specular>
                </material>
            </visual>
        </link>
    </model>
</sdf>'''
        return sdf


def main(args=None):
    rclpy.init(args=args)
    
    node = ObjectStateManager()
    
    # Use multi-threaded executor for service callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
