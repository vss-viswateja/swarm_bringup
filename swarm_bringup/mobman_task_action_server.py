#!/usr/bin/env python3
"""
Mobile Manipulator Task Action Server

This node implements an Action Server for the /mobman/task_control action (RobotTask interface).
It coordinates navigation (Nav2) and manipulation (MoveIt2) tasks for the mobile manipulator.

Supported task types:
- 'navigate': Forwards goal to Nav2 NavigateToPose
- 'pick': Moves arm to target pose, then calls attach_object service
- 'place': Moves arm to target pose, then calls detach_object service

Features:
- Action Server: /mobman/task_control (RobotTask)
- Action Client: /mobman/navigate_to_pose (NavigateToPose)
- MoveIt2 integration for arm control (mobman_arm group)
- Attach/Detach service integration
- Preemption handling for both Nav2 and MoveIt2
- Feedback every 500ms with current pose and status

Usage:
    ros2 run swarm_bringup mobman_task_action_server
    ros2 launch swarm_bringup mobman_task_action_server.launch.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Pose
from sensor_msgs.msg import JointState

from swarm_interfaces.action import RobotTask
from swarm_interfaces.srv import AttachObject, DetachObject, SetBoxState
from nav2_msgs.action import NavigateToPose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    JointConstraint,
)
from shape_msgs.msg import SolidPrimitive

from tf2_ros import Buffer, TransformListener, LookupException

import threading
import time
from action_msgs.msg import GoalStatus


class MobManTaskActionServer(Node):
    """
    Action Server that handles navigation and manipulation tasks for Mobile Manipulator.
    
    Supports 'navigate', 'pick', and 'place' task types with full preemption support.
    """

    def __init__(self):
        super().__init__('mobman_task_action_server')
        
        # Callback groups
        self.action_cb_group = ReentrantCallbackGroup()
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        
        # Parameters
        self.declare_parameter('robot_namespace', 'mobman')
        self.declare_parameter('planning_group', 'mobman_arm')
        self.declare_parameter('end_effector_link', 'arm_link6_1')
        self.declare_parameter('base_frame', 'mobman/base_link')
        self.declare_parameter('planning_frame', 'mobman/world')
        self.declare_parameter('feedback_rate', 2.0)  # 500ms = 2Hz
        self.declare_parameter('planning_time', 10.0)
        
        self.robot_namespace = self.get_parameter('robot_namespace').value
        self.planning_group = self.get_parameter('planning_group').value
        self.ee_link = self.get_parameter('end_effector_link').value
        self.base_frame = self.get_parameter('base_frame').value
        self.planning_frame = self.get_parameter('planning_frame').value
        self.feedback_rate = self.get_parameter('feedback_rate').value
        self.planning_time = self.get_parameter('planning_time').value
        
        # State tracking
        self._current_base_pose = None
        self._current_ee_pose = None
        self._pose_lock = threading.Lock()
        
        # Nav2 goal tracking
        self._nav2_goal_handle = None
        self._nav2_lock = threading.Lock()
        
        # MoveIt goal tracking
        self._moveit_goal_handle = None
        self._moveit_lock = threading.Lock()
        
        # Current task state
        self._current_status = 'Idle'
        self._is_executing = False
        self._cancel_requested = False
        
        # Attached object tracking (for place task)
        self._attached_object_joint = None
        self._attached_object_name = None  # Track box name for making it static on place
        
        # TF2 buffer for pose lookups
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Subscribe to AMCL pose for base feedback
        pose_topic = f'/{self.robot_namespace}/amcl_pose'
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            pose_topic,
            self._pose_callback,
            10,
            callback_group=self.action_cb_group
        )
        
        # Action Client for Nav2 NavigateToPose
        nav2_action_name = f'/{self.robot_namespace}/navigate_to_pose'
        self._nav2_client = ActionClient(
            self,
            NavigateToPose,
            nav2_action_name,
            callback_group=self.action_cb_group
        )
        
        # Action Client for MoveIt MoveGroup
        moveit_action_name = f'/{self.robot_namespace}/move_action'
        self._moveit_client = ActionClient(
            self,
            MoveGroup,
            moveit_action_name,
            callback_group=self.action_cb_group
        )
        
        # Service clients for attach/detach (namespaced to match simple_gripper_service)
        attach_service = f'/{self.robot_namespace}/attach_object'
        detach_service = f'/{self.robot_namespace}/detach_object'
        
        self._attach_client = self.create_client(
            AttachObject,
            attach_service,
            callback_group=self.service_cb_group
        )
        self._detach_client = self.create_client(
            DetachObject,
            detach_service,
            callback_group=self.service_cb_group
        )
        self._set_box_state_client = self.create_client(
            SetBoxState,
            '/set_box_state',
            callback_group=self.service_cb_group
        )
        
        # Action Server for RobotTask
        task_action_name = f'/{self.robot_namespace}/task_control'
        self._action_server = ActionServer(
            self,
            RobotTask,
            task_action_name,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.action_cb_group
        )
        
        self.get_logger().info('=' * 70)
        self.get_logger().info('Mobile Manipulator Task Action Server Started')
        self.get_logger().info(f'  Robot Namespace: {self.robot_namespace}')
        self.get_logger().info(f'  Task Action: {task_action_name}')
        self.get_logger().info(f'  Nav2 Action: {nav2_action_name}')
        self.get_logger().info(f'  MoveIt Action: {moveit_action_name}')
        self.get_logger().info(f'  Attach Service: {attach_service}')
        self.get_logger().info(f'  Detach Service: {detach_service}')
        self.get_logger().info(f'  Planning Group: {self.planning_group}')
        self.get_logger().info(f'  End Effector: {self.ee_link}')
        self.get_logger().info(f'  Feedback Rate: {self.feedback_rate} Hz')
        self.get_logger().info('=' * 70)

    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        """Update current base pose from AMCL."""
        with self._pose_lock:
            self._current_base_pose = msg.pose.pose

    def _get_ee_pose(self) -> Pose:
        """Get current end-effector pose from TF."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.planning_frame,
                self.ee_link,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
            return pose
        except Exception as e:
            self.get_logger().debug(f'Failed to get EE pose: {e}')
            return None

    def _goal_callback(self, goal_request):
        """Handle incoming goal requests."""
        task_type = goal_request.task_type.lower()
        
        self.get_logger().info(f'Received goal request: task_type="{task_type}"')
        
        # Validate task type
        if task_type not in ['navigate', 'pick', 'place']:
            self.get_logger().warn(f'Unknown task type: {task_type}')
            return GoalResponse.REJECT
        
        # Check availability of required action servers
        if task_type == 'navigate':
            if not self._nav2_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error('Nav2 navigate_to_pose action server not available!')
                return GoalResponse.REJECT
        else:  # pick or place
            if not self._moveit_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error('MoveIt move_action server not available!')
                return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """Handle cancel requests."""
        self.get_logger().info('Received cancel request - stopping all operations')
        self._cancel_requested = True
        
        # Cancel Nav2 goal if active
        self._cancel_nav2_goal()
        
        # Cancel MoveIt goal if active
        self._cancel_moveit_goal()
        
        return CancelResponse.ACCEPT

    def _cancel_nav2_goal(self):
        """Cancel active Nav2 goal."""
        with self._nav2_lock:
            if self._nav2_goal_handle is not None:
                self.get_logger().info('Canceling Nav2 goal...')
                self._nav2_goal_handle.cancel_goal_async()
                self._nav2_goal_handle = None

    def _cancel_moveit_goal(self):
        """Cancel active MoveIt goal."""
        with self._moveit_lock:
            if self._moveit_goal_handle is not None:
                self.get_logger().info('Canceling MoveIt goal...')
                self._moveit_goal_handle.cancel_goal_async()
                self._moveit_goal_handle = None

    async def _execute_callback(self, goal_handle):
        """Execute the RobotTask goal."""
        self.get_logger().info('Executing goal...')
        self._cancel_requested = False
        self._is_executing = True
        
        request = goal_handle.request
        target_pose = request.target_pose
        task_type = request.task_type.lower()
        
        self.get_logger().info(
            f'Task: {task_type}, Target: '
            f'({target_pose.pose.position.x:.2f}, '
            f'{target_pose.pose.position.y:.2f}, '
            f'{target_pose.pose.position.z:.2f})'
        )
        
        result = RobotTask.Result()
        
        try:
            if task_type == 'navigate':
                result = await self._execute_navigate(goal_handle, target_pose)
            elif task_type == 'pick':
                result = await self._execute_pick(goal_handle, target_pose)
            elif task_type == 'place':
                result = await self._execute_place(goal_handle, target_pose)
        except Exception as e:
            self.get_logger().error(f'Execution error: {e}')
            result.success = False
            result.message = f'Execution error: {str(e)}'
            goal_handle.abort()
        finally:
            self._is_executing = False
            self._current_status = 'Idle'
        
        return result

    async def _execute_navigate(self, goal_handle, target_pose: PoseStamped) -> RobotTask.Result:
        """Execute navigation task using Nav2."""
        self._current_status = 'Moving Base'
        result = RobotTask.Result()
        
        # Create Nav2 goal
        nav2_goal = NavigateToPose.Goal()
        nav2_goal.pose = target_pose
        
        self.get_logger().info('Sending navigation goal to Nav2...')
        send_goal_future = self._nav2_client.send_goal_async(nav2_goal)
        nav2_goal_handle = await send_goal_future
        
        if not nav2_goal_handle.accepted:
            self.get_logger().error('Nav2 goal was rejected!')
            result.success = False
            result.message = 'Nav2 rejected the navigation goal'
            goal_handle.abort()
            return result
        
        self.get_logger().info('Nav2 goal accepted')
        
        with self._nav2_lock:
            self._nav2_goal_handle = nav2_goal_handle
        
        # Start feedback loop
        result_future = nav2_goal_handle.get_result_async()
        feedback_period = 1.0 / self.feedback_rate  # 500ms
        
        while not result_future.done():
            if goal_handle.is_cancel_requested or self._cancel_requested:
                self.get_logger().info('Navigation canceled')
                self._cancel_nav2_goal()
                goal_handle.canceled()
                result.success = False
                result.message = 'Navigation canceled'
                return result
            
            # Publish feedback
            feedback = RobotTask.Feedback()
            feedback.current_status = 'Moving Base'
            with self._pose_lock:
                if self._current_base_pose is not None:
                    feedback.current_pose = self._current_base_pose
            goal_handle.publish_feedback(feedback)
            
            time.sleep(feedback_period)
        
        # Process result
        nav2_result = result_future.result()
        
        with self._nav2_lock:
            self._nav2_goal_handle = None
        
        if nav2_result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded!')
            result.success = True
            result.message = 'Navigation completed successfully'
            goal_handle.succeed()
        else:
            self.get_logger().error(f'Navigation failed: status={nav2_result.status}')
            result.success = False
            result.message = f'Navigation failed with status: {nav2_result.status}'
            goal_handle.abort()
        
        return result

    def _get_pose_from_tf(self, target_frame: str, z_offset: float = 0.0) -> PoseStamped:
        """
        Get pose of a TF frame relative to the planning frame.
        
        Args:
            target_frame: The TF frame to look up (e.g., 'aruco_box_0')
            z_offset: Optional offset to add to Z position (positive = above the frame)
        
        Returns:
            PoseStamped with the frame's pose relative to planning_frame, or None if lookup fails
        """
        try:
            # Look up the transform from planning frame to target frame
            # This ensures the returned pose is in the frame MoveIt expects
            transform = self.tf_buffer.lookup_transform(
                self.planning_frame,  # Use planning frame (mobman/world) for MoveIt compatibility
                target_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            
            # Create PoseStamped from transform
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.planning_frame
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose.position.x = transform.transform.translation.x
            pose_stamped.pose.position.y = transform.transform.translation.y
            pose_stamped.pose.position.z = transform.transform.translation.z + z_offset
            pose_stamped.pose.orientation = transform.transform.rotation
            
            self.get_logger().info(
                f'Got pose for {target_frame} in {self.planning_frame}: '
                f'({pose_stamped.pose.position.x:.3f}, '
                f'{pose_stamped.pose.position.y:.3f}, '
                f'{pose_stamped.pose.position.z:.3f})'
            )
            
            return pose_stamped
            
        except Exception as e:
            self.get_logger().error(f'Failed to get TF for {target_frame} from {self.planning_frame}: {e}')
            return None

    async def _execute_pick(self, goal_handle, target_pose: PoseStamped) -> RobotTask.Result:
        """
        Execute pick task with the following phases:
        1. Move arm to ready config
        2. Plan and execute arm motion to pick position
        3. Make box dynamic
        4. Attach box
        5. Return to ready config
        6. Return to home config
        
        The target_pose.header.frame_id should contain the TF frame name of the object to pick (e.g., 'aruco_box_0').
        The actual pose values in target_pose are ignored - the pose is looked up from TF.
        """
        result = RobotTask.Result()
        
        # Extract box name from target pose frame_id (e.g., 'aruco_box_0')
        box_name = target_pose.header.frame_id if target_pose.header.frame_id else None
        
        if not box_name:
            result.success = False
            result.message = 'Pick failed: No TF frame name provided in target_pose.header.frame_id'
            goal_handle.abort()
            return result
        
        # Phase 1: Move arm to ready position first
        self._current_status = 'Moving to Ready Position'
        self.get_logger().info('Moving arm to ready position before pick...')
        
        ready_success = await self._move_arm_to_named_position(goal_handle, 'ready')
        
        if not ready_success:
            result.success = False
            result.message = 'Failed to move arm to ready position'
            goal_handle.abort()
            return result
        
        if self._cancel_requested:
            result.success = False
            result.message = 'Pick task canceled during ready move'
            goal_handle.canceled()
            return result
        
        # Phase 2: Look up the object pose from TF tree
        self._current_status = 'Looking up Object Pose'
        self.get_logger().info(f'Looking up TF frame: {box_name}')
        
        # Add a small Z offset (e.g., 12cm above the box) for approach
        pick_z_offset = 0.11
        tf_pose = self._get_pose_from_tf(box_name, z_offset=pick_z_offset)
        
        if tf_pose is None:
            result.success = False
            result.message = f'Pick failed: Could not find TF frame "{box_name}"'
            goal_handle.abort()
            return result
        
        # Override orientation with a fixed "gripper pointing down" orientation
        # Quaternion for 90° rotation around Y axis
        tf_pose.pose.orientation.x = 0.0
        tf_pose.pose.orientation.y = 0.707
        tf_pose.pose.orientation.z = 0.0
        tf_pose.pose.orientation.w = 0.707
        
        self.get_logger().info(
            f'Pick target for {box_name}: pos=({tf_pose.pose.position.x:.3f}, '
            f'{tf_pose.pose.position.y:.3f}, {tf_pose.pose.position.z:.3f}), '
            f'orientation=gripper-down'
        )
        
        # Phase 3: Plan and execute arm motion to pick position
        self._current_status = 'Planning Arm Path'
        self.get_logger().info(f'Planning arm motion to pick {box_name}...')
        
        arm_success = await self._move_arm_to_pose(goal_handle, tf_pose)
        
        if not arm_success:
            result.success = False
            result.message = 'Failed to move arm to pick position'
            goal_handle.abort()
            return result
        
        if self._cancel_requested:
            result.success = False
            result.message = 'Pick task canceled'
            goal_handle.canceled()
            return result
        
        # Phase 4: Make box dynamic (so it can be manipulated)
        if box_name:
            self._current_status = 'Preparing Object'
            self.get_logger().info(f'Making {box_name} dynamic for manipulation...')
            
            dynamic_success, dynamic_msg = await self._call_set_box_state_service(box_name, make_dynamic=True)
            
            if not dynamic_success:
                self.get_logger().warn(f'Failed to make box dynamic: {dynamic_msg}. Attempting attach anyway...')
            else:
                # Wait for physics simulation to update after making box dynamic
                self.get_logger().info('Waiting for physics to stabilize...')
                time.sleep(0.5)
        
        if self._cancel_requested:
            result.success = False
            result.message = 'Pick task canceled before attach'
            goal_handle.canceled()
            return result
        
        # Phase 5: Attach object
        self._current_status = 'Grasping Object'
        self.get_logger().info('Attaching object...')
        
        attach_success, attach_msg, joint_name = await self._call_attach_service(box_name)
        
        if not attach_success:
            result.success = False
            result.message = f'Attach failed: {attach_msg}'
            goal_handle.abort()
            return result
        
        self._attached_object_joint = joint_name
        self._attached_object_name = box_name  # Remember box name for static toggle on place
        
        if self._cancel_requested:
            result.success = False
            result.message = 'Pick task canceled after attach'
            goal_handle.canceled()
            return result
        
        # Phase 6: Return arm to ready position
        self._current_status = 'Returning to Ready'
        self.get_logger().info('Returning arm to ready position...')
        
        ready_return_success = await self._move_arm_to_named_position(goal_handle, 'ready')
        
        if not ready_return_success:
            self.get_logger().warn('Failed to return arm to ready position, attempting home directly...')
        
        if self._cancel_requested:
            result.success = False
            result.message = 'Pick task canceled during return'
            goal_handle.canceled()
            return result
        
        # Phase 7: Return arm to home position
        self._current_status = 'Returning to Home'
        self.get_logger().info('Returning arm to home position...')
        
        home_success = await self._move_arm_to_named_position(goal_handle, 'home')
        
        if home_success:
            result.success = True
            result.message = f'Pick completed and arm returned to home: {attach_msg}'
            goal_handle.succeed()
        else:
            # Pick was successful but return to home failed - still report success for the pick
            self.get_logger().warn('Failed to return arm to home position, but pick was successful')
            result.success = True
            result.message = f'Pick completed but failed to return to home: {attach_msg}'
            goal_handle.succeed()
        
        return result

    async def _execute_place(self, goal_handle, target_pose: PoseStamped) -> RobotTask.Result:
        """Execute place task: move arm to target, detach object, make box static, then return to zero."""
        result = RobotTask.Result()
        
        # Get the attached box name for making it static after detach
        attached_box_name = self._attached_object_name
        
        # Phase 1: Plan and execute arm motion to place position
        self._current_status = 'Planning Arm Path'
        self.get_logger().info('Planning arm motion for place...')
        
        arm_success = await self._move_arm_to_pose(goal_handle, target_pose)
        
        if not arm_success:
            result.success = False
            result.message = 'Failed to move arm to place position'
            goal_handle.abort()
            return result
        
        if self._cancel_requested:
            result.success = False
            result.message = 'Place task canceled'
            goal_handle.canceled()
            return result
        
        # Phase 2: Detach object
        self._current_status = 'Releasing Object'
        self.get_logger().info('Detaching object...')
        
        if self._attached_object_joint:
            detach_success, detach_msg = await self._call_detach_service(self._attached_object_joint)
            self._attached_object_joint = None
        else:
            detach_success = True
            detach_msg = 'No object was attached'
        
        if not detach_success:
            result.success = False
            result.message = f'Detach failed: {detach_msg}'
            goal_handle.abort()
            return result
        
        # Phase 3: Make box static (freeze in place)
        if attached_box_name:
            self._current_status = 'Freezing Object'
            self.get_logger().info(f'Making {attached_box_name} static (freezing in place)...')
            
            static_success, static_msg = await self._call_set_box_state_service(attached_box_name, make_dynamic=False)
            
            if not static_success:
                self.get_logger().warn(f'Failed to make box static: {static_msg}. Continuing anyway...')
            
            self._attached_object_name = None  # Clear the tracked name
        
        if self._cancel_requested:
            result.success = False
            result.message = 'Place task canceled after detach'
            goal_handle.canceled()
            return result
        
        # Phase 4: Return arm to zero position (same as home - all joints at 0)
        self._current_status = 'Returning to Zero'
        self.get_logger().info('Returning arm to zero position...')
        
        zero_success = await self._move_arm_to_named_position(goal_handle, 'home')  # 'home' in SRDF is all zeros
        
        if zero_success:
            result.success = True
            result.message = f'Place completed and arm returned to zero: {detach_msg}'
            goal_handle.succeed()
        else:
            # Place was successful but return to zero failed - still report success for the place
            self.get_logger().warn('Failed to return arm to zero position, but place was successful')
            result.success = True
            result.message = f'Place completed but failed to return to zero: {detach_msg}'
            goal_handle.succeed()
        
        return result

    async def _move_arm_to_pose(self, goal_handle, target_pose: PoseStamped) -> bool:
        """Move arm to target pose using MoveIt."""
        # Create MoveGroup goal
        moveit_goal = MoveGroup.Goal()
        moveit_goal.request.group_name = self.planning_group
        moveit_goal.request.num_planning_attempts = 10
        moveit_goal.request.allowed_planning_time = self.planning_time
        moveit_goal.request.max_velocity_scaling_factor = 0.3
        moveit_goal.request.max_acceleration_scaling_factor = 0.3
        
        # Workspace bounds
        moveit_goal.request.workspace_parameters.header.frame_id = self.planning_frame
        moveit_goal.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        moveit_goal.request.workspace_parameters.min_corner.x = -3.0
        moveit_goal.request.workspace_parameters.min_corner.y = -3.0
        moveit_goal.request.workspace_parameters.min_corner.z = -1.0
        moveit_goal.request.workspace_parameters.max_corner.x = 3.0
        moveit_goal.request.workspace_parameters.max_corner.y = 3.0
        moveit_goal.request.workspace_parameters.max_corner.z = 3.0
        
        # Create pose constraint
        constraints = Constraints()
        constraints.name = 'arm_pose_goal'
        
        # Position constraint - use planning frame for consistency
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.planning_frame
        position_constraint.header.stamp = self.get_clock().now().to_msg()
        position_constraint.link_name = self.ee_link
        
        bounding_volume = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.01]  # 1cm tolerance
        bounding_volume.primitives.append(primitive)
        bounding_volume.primitive_poses.append(target_pose.pose)
        
        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0
        constraints.position_constraints.append(position_constraint)
        
        # Orientation constraint - use planning frame for consistency
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.planning_frame
        orientation_constraint.header.stamp = self.get_clock().now().to_msg()
        orientation_constraint.link_name = self.ee_link
        orientation_constraint.orientation = target_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0
        constraints.orientation_constraints.append(orientation_constraint)
        
        moveit_goal.request.goal_constraints.append(constraints)
        moveit_goal.planning_options.plan_only = False
        moveit_goal.planning_options.replan = True
        moveit_goal.planning_options.replan_attempts = 3
        
        # Send goal
        self._current_status = 'Moving Arm'
        self.get_logger().info('Sending arm motion goal to MoveIt...')
        
        send_goal_future = self._moveit_client.send_goal_async(moveit_goal)
        moveit_goal_handle = await send_goal_future
        
        if not moveit_goal_handle.accepted:
            self.get_logger().error('MoveIt goal was rejected!')
            return False
        
        self.get_logger().info('MoveIt goal accepted')
        
        with self._moveit_lock:
            self._moveit_goal_handle = moveit_goal_handle
        
        # Wait for result with feedback
        result_future = moveit_goal_handle.get_result_async()
        feedback_period = 1.0 / self.feedback_rate
        
        while not result_future.done():
            if self._cancel_requested:
                self._cancel_moveit_goal()
                return False
            
            # Publish feedback with EE pose
            feedback = RobotTask.Feedback()
            feedback.current_status = self._current_status
            ee_pose = self._get_ee_pose()
            if ee_pose:
                feedback.current_pose = ee_pose
            goal_handle.publish_feedback(feedback)
            
            time.sleep(feedback_period)
        
        # Check result
        moveit_result = result_future.result()
        
        with self._moveit_lock:
            self._moveit_goal_handle = None
        
        if moveit_result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Arm motion succeeded!')
            return True
        else:
            self.get_logger().error(f'Arm motion failed: status={moveit_result.status}')
            return False

    async def _move_arm_to_named_position(self, goal_handle, position_name: str) -> bool:
        """Move arm to a named position (e.g., 'home', 'vertical') using joint constraints."""
        # Define named positions (must match SRDF group_state definitions)
        named_positions = {
            'home': {
                'arm_base_link1_joint': 0.0,
                'arm_link1_link2p1_joint': 0.0,
                'arm_link2p3_link3_joint': 0.0,
                'arm_link3_link4_joint': 0.0,
                'arm_link4_link5_joint': 0.0,
                'arm_link5_link6_joint': 0.0,
            },
            'ready': {
                'arm_base_link1_joint': 1.57,
                'arm_link1_link2p1_joint': 0.0,
                'arm_link2p3_link3_joint': 0.0,
                'arm_link3_link4_joint': 0.0,
                'arm_link4_link5_joint': 0.0,
                'arm_link5_link6_joint': 0.0,
            },
            'vertical': {
                'arm_base_link1_joint': 0.0,
                'arm_link1_link2p1_joint': 0.0,
                'arm_link2p3_link3_joint': 1.57,
                'arm_link3_link4_joint': 1.57,
                'arm_link4_link5_joint': 0.0,
                'arm_link5_link6_joint': 0.0,
            },
        }
        
        if position_name not in named_positions:
            self.get_logger().error(f'Unknown named position: {position_name}')
            return False
        
        joint_values = named_positions[position_name]
        
        # Create MoveGroup goal with joint constraints
        moveit_goal = MoveGroup.Goal()
        moveit_goal.request.group_name = self.planning_group
        moveit_goal.request.num_planning_attempts = 10
        moveit_goal.request.allowed_planning_time = self.planning_time
        moveit_goal.request.max_velocity_scaling_factor = 0.3
        moveit_goal.request.max_acceleration_scaling_factor = 0.3
        
        # Workspace bounds
        moveit_goal.request.workspace_parameters.header.frame_id = self.planning_frame
        moveit_goal.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        moveit_goal.request.workspace_parameters.min_corner.x = -3.0
        moveit_goal.request.workspace_parameters.min_corner.y = -3.0
        moveit_goal.request.workspace_parameters.min_corner.z = -1.0
        moveit_goal.request.workspace_parameters.max_corner.x = 3.0
        moveit_goal.request.workspace_parameters.max_corner.y = 3.0
        moveit_goal.request.workspace_parameters.max_corner.z = 3.0
        
        # Create joint constraints for each joint
        constraints = Constraints()
        constraints.name = f'arm_{position_name}_goal'
        
        for joint_name, joint_value in joint_values.items():
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = joint_name
            joint_constraint.position = joint_value
            joint_constraint.tolerance_above = 0.01
            joint_constraint.tolerance_below = 0.01
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)
        
        moveit_goal.request.goal_constraints.append(constraints)
        moveit_goal.planning_options.plan_only = False
        moveit_goal.planning_options.replan = True
        moveit_goal.planning_options.replan_attempts = 3
        
        # Send goal
        self._current_status = f'Moving to {position_name.capitalize()}'
        self.get_logger().info(f'Sending arm motion goal to MoveIt for {position_name} position...')
        
        send_goal_future = self._moveit_client.send_goal_async(moveit_goal)
        moveit_goal_handle = await send_goal_future
        
        if not moveit_goal_handle.accepted:
            self.get_logger().error(f'MoveIt goal for {position_name} was rejected!')
            return False
        
        self.get_logger().info(f'MoveIt goal for {position_name} accepted')
        
        with self._moveit_lock:
            self._moveit_goal_handle = moveit_goal_handle
        
        # Wait for result with feedback
        result_future = moveit_goal_handle.get_result_async()
        feedback_period = 1.0 / self.feedback_rate
        
        while not result_future.done():
            if self._cancel_requested:
                self._cancel_moveit_goal()
                return False
            
            # Publish feedback with EE pose
            feedback = RobotTask.Feedback()
            feedback.current_status = self._current_status
            ee_pose = self._get_ee_pose()
            if ee_pose:
                feedback.current_pose = ee_pose
            goal_handle.publish_feedback(feedback)
            
            time.sleep(feedback_period)
        
        # Check result
        moveit_result = result_future.result()
        
        with self._moveit_lock:
            self._moveit_goal_handle = None
        
        if moveit_result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Arm motion to {position_name} succeeded!')
            return True
        else:
            self.get_logger().error(f'Arm motion to {position_name} failed: status={moveit_result.status}')
            return False

    async def _call_attach_service(self, box_name: str):
        """Call attach_object service.
        
        Args:
            box_name: Name of the box to attach (e.g., 'aruco_box_0')
        """
        if not self._attach_client.wait_for_service(timeout_sec=2.0):
            return False, 'Attach service not available', None
        
        request = AttachObject.Request()
        request.parent_model = 'mobman'
        request.parent_link = 'arm_link6_1'
        request.child_model = box_name
        request.child_link = 'link'
        
        future = self._attach_client.call_async(request)
        
        try:
            response = await future
            return response.success, response.message, response.joint_name
        except Exception as e:
            return False, str(e), None

    async def _call_detach_service(self, joint_name: str):
        """Call detach_object service."""
        if not self._detach_client.wait_for_service(timeout_sec=2.0):
            return False, 'Detach service not available'
        
        request = DetachObject.Request()
        request.joint_name = joint_name
        
        future = self._detach_client.call_async(request)
        
        try:
            response = await future
            return response.success, response.message
        except Exception as e:
            return False, str(e)

    async def _call_set_box_state_service(self, box_name: str, make_dynamic: bool):
        """
        Call set_box_state service to toggle box between static and dynamic states.
        
        Args:
            box_name: Name of the box (e.g., 'aruco_box_0')
            make_dynamic: True to make box dynamic (for manipulation), False to make static (freeze)
        
        Returns:
            Tuple of (success, message)
        """
        if not self._set_box_state_client.wait_for_service(timeout_sec=2.0):
            return False, 'SetBoxState service not available'
        
        state_str = 'dynamic' if make_dynamic else 'static'
        self.get_logger().info(f'Setting {box_name} to {state_str}...')
        
        request = SetBoxState.Request()
        request.box_name = box_name
        request.make_dynamic = make_dynamic
        
        future = self._set_box_state_client.call_async(request)
        
        try:
            response = await future
            if response.success:
                self.get_logger().info(f'Successfully set {box_name} to {state_str}')
            else:
                self.get_logger().error(f'Failed to set {box_name} to {state_str}: {response.message}')
            return response.success, response.message
        except Exception as e:
            self.get_logger().error(f'Exception calling set_box_state: {e}')
            return False, str(e)


def main(args=None):
    rclpy.init(args=args)
    
    node = MobManTaskActionServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
