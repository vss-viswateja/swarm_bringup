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

from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import JointState

from swarm_interfaces.action import RobotTask
from swarm_interfaces.srv import AttachObject, DetachObject, SetBoxState
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import GetCostmap
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
import tf2_geometry_msgs

import math
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
        self.declare_parameter('transform_frame', 'mobman/chassis_link')
        self.declare_parameter('feedback_rate', 2.0)  # 500ms = 2Hz
        self.declare_parameter('planning_time', 10.0)
        self.declare_parameter('nav_stuck_threshold', 0.05)  # meters
        self.declare_parameter('nav_stuck_timeout', 60.0)    # seconds
        self.declare_parameter('nav_max_retries', 3)
        # Place / Pick standoff search parameters
        self.declare_parameter('pick_radius_min', 0.5)          # inner ring radius (m)
        self.declare_parameter('place_radius_min', 0.5)          # inner ring radius (m)
        self.declare_parameter('pick_radius_max', 0.75)          # outer ring radius (m)
        self.declare_parameter('place_radius_max', 0.75)          # outer ring radius (m)
        self.declare_parameter('place_angle_step', 30.0)         # angular step in degrees
        self.declare_parameter('place_max_retries', 5)           # max IK-invalid candidates before abort
        self.declare_parameter('pick_max_retries', 5)            # max IK-invalid candidates before abort (pick)
        self.declare_parameter('costmap_lethal_threshold', 125)  # cost value above which cell is "lethal"
        
        self.robot_namespace = self.get_parameter('robot_namespace').value
        self.planning_group = self.get_parameter('planning_group').value
        self.ee_link = self.get_parameter('end_effector_link').value
        self.base_frame = self.get_parameter('base_frame').value
        self.planning_frame = self.get_parameter('planning_frame').value
        self.transform_frame = self.get_parameter('transform_frame').value
        self.feedback_rate = self.get_parameter('feedback_rate').value
        self.planning_time = self.get_parameter('planning_time').value
        self.nav_stuck_threshold = self.get_parameter('nav_stuck_threshold').value
        self.nav_stuck_timeout = self.get_parameter('nav_stuck_timeout').value
        self.nav_max_retries = self.get_parameter('nav_max_retries').value
        self.place_radius_min = self.get_parameter('place_radius_min').value
        self.place_radius_max = self.get_parameter('place_radius_max').value
        self.place_angle_step = self.get_parameter('place_angle_step').value
        self.place_max_retries = self.get_parameter('place_max_retries').value
        self.pick_max_retries = self.get_parameter('pick_max_retries').value
        self.pick_radius_min = self.get_parameter('pick_radius_min').value
        self.pick_radius_max = self.get_parameter('pick_radius_max').value
        self.costmap_lethal_threshold = self.get_parameter('costmap_lethal_threshold').value
        
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
        
        # Poll TF for base pose at 10 Hz (replaces AMCL subscriber —
        # AMCL has tf_broadcast: false so the topic is unreliable;
        # the map→odom→base_link chain is maintained by the external TF tree)
        self._pose_timer = self.create_timer(
            0.1,  # 10 Hz
            self._update_base_pose_from_tf,
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
        
        # Service client for querying global costmap (used in standoff pose search)
        get_costmap_service = f'/{self.robot_namespace}/global_costmap/get_costmap'
        self._get_costmap_client = self.create_client(
            GetCostmap,
            get_costmap_service,
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

    def _update_base_pose_from_tf(self):
        """Poll the TF tree at 10 Hz to keep self._current_base_pose up-to-date.

        Looks up the transform from 'map' to self.base_frame
        (e.g. 'mobman/base_link') and stores it as a geometry_msgs/Pose.
        This replaces the AMCL topic subscription because AMCL is configured
        with tf_broadcast: false.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                self.base_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.05)
            )
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
            with self._pose_lock:
                self._current_base_pose = pose
        except Exception:
            # TF not yet available — silently skip; pose stays as last known
            pass

    def _get_ee_pose(self) -> Pose:
        """Get current end-effector pose from TF."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.transform_frame,
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

    async def _execute_navigate(
        self,
        goal_handle,
        target_pose: PoseStamped,
        is_subtask: bool = False,
    ) -> RobotTask.Result:
        """Execute navigation task using Nav2 with stuck detection.

        Args:
            goal_handle:  The RobotTask action goal handle.
            target_pose:  Desired Nav2 pose.
            is_subtask:   When True, this call is a SUB-STEP of a larger task
                          (e.g. standoff navigation inside _execute_place).
                          In that mode the function does NOT call any terminal
                          goal_handle method (succeed/abort/canceled) — the
                          caller owns the goal handle lifecycle.
                          When False (default), navigate is the top-level task
                          and it sets the goal handle terminal state itself.

        If the robot's x,y position doesn't change by more than nav_stuck_threshold
        for nav_stuck_timeout seconds, the Nav2 goal is canceled and re-sent.
        After nav_max_retries re-sends, navigation is aborted.
        """
        self._current_status = 'Moving Base'
        result = RobotTask.Result()
        retry_count = 0

        while retry_count <= self.nav_max_retries:
            # Create and send Nav2 goal
            nav2_goal = NavigateToPose.Goal()
            nav2_goal.pose = target_pose

            attempt_str = f' (retry {retry_count}/{self.nav_max_retries})' if retry_count > 0 else ''
            self.get_logger().info(f'Sending navigation goal to Nav2...{attempt_str}')
            send_goal_future = self._nav2_client.send_goal_async(nav2_goal)
            nav2_goal_handle = await send_goal_future

            if not nav2_goal_handle.accepted:
                self.get_logger().error('Nav2 goal was rejected!')
                result.success = False
                result.message = 'Nav2 rejected the navigation goal'
                if not is_subtask:
                    goal_handle.abort()
                return result

            self.get_logger().info('Nav2 goal accepted')

            with self._nav2_lock:
                self._nav2_goal_handle = nav2_goal_handle

            # Initialize stuck detection
            last_moved_x = None
            last_moved_y = None
            last_moved_time = time.time()
            stuck_detected = False

            # Start feedback loop
            result_future = nav2_goal_handle.get_result_async()
            feedback_period = 1.0 / self.feedback_rate  # 500ms

            while not result_future.done():
                if goal_handle.is_cancel_requested or self._cancel_requested:
                    self.get_logger().info('Navigation canceled')
                    self._cancel_nav2_goal()
                    if not is_subtask:
                        goal_handle.canceled()
                    result.success = False
                    result.message = 'Navigation canceled'
                    self._cancel_requested = True  # propagate to caller
                    return result

                # Get current pose for stuck detection
                current_x = None
                current_y = None
                with self._pose_lock:
                    if self._current_base_pose is not None:
                        current_x = self._current_base_pose.position.x
                        current_y = self._current_base_pose.position.y

                # Stuck detection logic
                if current_x is not None and current_y is not None:
                    if last_moved_x is None:
                        # First reading — initialize
                        last_moved_x = current_x
                        last_moved_y = current_y
                        last_moved_time = time.time()
                    else:
                        dist = math.sqrt(
                            (current_x - last_moved_x) ** 2 +
                            (current_y - last_moved_y) ** 2
                        )
                        if dist > self.nav_stuck_threshold:
                            # Robot is moving — update reference
                            last_moved_x = current_x
                            last_moved_y = current_y
                            last_moved_time = time.time()
                        else:
                            # Check if stuck timeout exceeded
                            elapsed = time.time() - last_moved_time
                            if elapsed > self.nav_stuck_timeout:
                                self.get_logger().warn(
                                    f'Robot stuck! Position ({current_x:.3f}, {current_y:.3f}) '
                                    f'has not changed by > {self.nav_stuck_threshold}m '
                                    f'for {elapsed:.1f}s. Re-sending goal...'
                                )
                                stuck_detected = True
                                break

                # Publish feedback
                feedback = RobotTask.Feedback()
                feedback.current_status = 'Moving Base'
                with self._pose_lock:
                    if self._current_base_pose is not None:
                        feedback.current_pose = self._current_base_pose
                goal_handle.publish_feedback(feedback)

                time.sleep(feedback_period)

            if stuck_detected:
                # Cancel the current Nav2 goal and retry
                self.get_logger().info('Canceling stuck Nav2 goal before retry...')
                self._cancel_nav2_goal()

                # Wait for cancel to take effect
                time.sleep(1.0)

                retry_count += 1
                if retry_count > self.nav_max_retries:
                    self.get_logger().error(
                        f'Navigation aborted: robot stuck after {self.nav_max_retries} retries'
                    )
                    result.success = False
                    result.message = (
                        f'Navigation aborted: robot stuck after '
                        f'{self.nav_max_retries} retries'
                    )
                    if not is_subtask:
                        goal_handle.abort()
                    return result

                continue  # Retry with a fresh Nav2 goal

            # Normal completion — process result
            nav2_result = result_future.result()

            with self._nav2_lock:
                self._nav2_goal_handle = None

            if nav2_result.status == GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().info('Navigation succeeded!')
                result.success = True
                result.message = 'Navigation completed successfully'
                if not is_subtask:
                    goal_handle.succeed()
            else:
                self.get_logger().error(f'Navigation failed: status={nav2_result.status}')
                result.success = False
                result.message = f'Navigation failed with status: {nav2_result.status}'
                if not is_subtask:
                    goal_handle.abort()

            return result

        # Should not reach here, but safety fallback
        result.success = False
        result.message = 'Navigation failed unexpectedly'
        if not is_subtask:
            goal_handle.abort()
        return result

    def _get_pose_from_tf(self, target_frame: str, z_offset: float = 0.0) -> PoseStamped:
        """
        Get pose of a TF frame relative to the planning frame.
        
        Args:
            target_frame: The TF frame to look up (e.g., 'aruco_box_0')
            z_offset: Optional offset to add to Z position (positive = above the frame)
        
        Returns:
            PoseStamped with the frame's pose relative to transform_frame, or None if lookup fails
        """
        try:
            # Look up the transform from planning frame to target frame.
            # IMPORTANT: use the current clock time (not time=0 / "latest") so that
            # TF2 resolves ALL segments of the chain at the same moment.
            # Using rclpy.time.Time() (time=0) allows TF2 to mix a fresh robot pose
            # with a stale box detection, producing a significant positional error.
            transform = self.tf_buffer.lookup_transform(
                self.transform_frame,
                target_frame,
                self.get_clock().now(),
                timeout=rclpy.duration.Duration(seconds=5.0)
            )
            
            # Create PoseStamped from transform
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = self.transform_frame
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.pose.position.x = transform.transform.translation.x
            pose_stamped.pose.position.y = transform.transform.translation.y
            pose_stamped.pose.position.z = transform.transform.translation.z + z_offset
            pose_stamped.pose.orientation = transform.transform.rotation
            
            self.get_logger().info(
                f'Got pose for {target_frame} in {self.transform_frame}: '
                f'({pose_stamped.pose.position.x:.3f}, '
                f'{pose_stamped.pose.position.y:.3f}, '
                f'{pose_stamped.pose.position.z:.3f})'
            )
            
            return pose_stamped
            
        except Exception as e:
            self.get_logger().error(f'Failed to get TF for {target_frame} from {self.transform_frame}: {e}')
            return None

    def _get_object_pose_in_map(self, target_frame: str):
        """
        Look up the TF frame position of an object in the map frame.

        Used to find the x, y coordinates in the global map so that
        standoff candidate base positions can be generated around the object.

        Args:
            target_frame: TF frame of the object (e.g., 'aruco_box_0')

        Returns:
            (x, y) tuple in map frame, or (None, None) on failure.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                target_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            self.get_logger().info(
                f'[PICK] Object "{target_frame}" in map: ({x:.3f}, {y:.3f})'
            )
            return x, y
        except Exception as e:
            self.get_logger().error(
                f'[PICK] Failed to get map-frame pose for "{target_frame}": {e}'
            )
            return None, None

    def _transform_pose_to_planning_frame(self, pose_stamped: PoseStamped) -> PoseStamped:
        """
        Transform a PoseStamped from its current frame (e.g., 'map') to the
        planning frame (mobman/chassis_link) using TF2.
        
        Args:
            pose_stamped: Input pose in any frame (expected: 'map')
        
        Returns:
            PoseStamped in the planning frame, or None if transform fails
        """
        source_frame = pose_stamped.header.frame_id
        target_frame = self.transform_frame  # mobman/chassis_link
        
        if source_frame == target_frame:
            self.get_logger().info('Pose is already in the planning frame, no transform needed')
            return pose_stamped
        
        try:
            # Look up the transform from source frame to planning frame
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            
            # Apply the transform to the pose
            transformed_pose = tf2_geometry_msgs.do_transform_pose_stamped(pose_stamped, transform)
            
            self.get_logger().info(
                f'Transformed pose from {source_frame} to {target_frame}: '
                f'({pose_stamped.pose.position.x:.3f}, '
                f'{pose_stamped.pose.position.y:.3f}, '
                f'{pose_stamped.pose.position.z:.3f}) -> '
                f'({transformed_pose.pose.position.x:.3f}, '
                f'{transformed_pose.pose.position.y:.3f}, '
                f'{transformed_pose.pose.position.z:.3f})'
            )
            
            return transformed_pose
            
        except Exception as e:
            self.get_logger().error(
                f'Failed to transform pose from {source_frame} to {target_frame}: {e}'
            )
            return None

    async def _execute_pick(self, goal_handle, target_pose: PoseStamped) -> RobotTask.Result:
        """
        Atomic Pick Action with Dynamic Reachability Search.

        Algorithm:
          Phase 0: Resolve object TF frame name from target_pose.header.frame_id.
                   Look up object x, y in map frame via TF to generate standoff poses.
          Phase 1: Generate candidate standoff base poses on rings around the object.
          Phase 2: Filter candidates by global costmap (reject lethal cells).
          Phase 3: Rank by distance to current robot position.
          Phase 4: Iterate candidates:
            a. Navigate to candidate base pose (Nav2, is_subtask=True).
            b. Look up fresh TF pose of object in planning frame (with Z offset).
            c. Probe MoveIt IK (plan only) — if fail, try next candidate.
            d. Execute the validated MoveIt plan.
          Phase 5: Make box dynamic, attach object.
          Phase 6: Return arm to ready, then home.

        The target_pose.header.frame_id must contain the TF frame name of the
        object to pick (e.g., 'aruco_box_0'). Pose values in target_pose are
        only used for the arm goal Z height; x/y and standoff positions are
        derived from TF.

        After pick_max_retries IK failures, abort with "Target Unreachable".
        """
        result = RobotTask.Result()

        # ----------------------------------------------------------------
        # Phase 0: Validate box frame and resolve map-frame x, y from TF
        # ----------------------------------------------------------------
        box_name = target_pose.header.frame_id if target_pose.header.frame_id else None

        if not box_name:
            result.success = False
            result.message = 'Pick failed: No TF frame name provided in target_pose.header.frame_id'
            goal_handle.abort()
            return result

        self._current_status = 'Resolving Object Position'
        self.get_logger().info(f'[PICK] Resolving map-frame position of "{box_name}"...')

        obj_map_x, obj_map_y = self._get_object_pose_in_map(box_name)

        if obj_map_x is None:
            result.success = False
            result.message = f'Pick failed: Cannot find TF frame "{box_name}" in map'
            goal_handle.abort()
            return result

        # ----------------------------------------------------------------
        # Phase 1: Orient robot toward the object (in-place rotation)
        # ----------------------------------------------------------------
        self._current_status = 'Orienting Toward Object'
        with self._pose_lock:
            current_base_pose = self._current_base_pose

        if current_base_pose is not None:
            robot_x = current_base_pose.position.x
            robot_y = current_base_pose.position.y
            orient_yaw = math.atan2(obj_map_y - robot_y, obj_map_x - robot_x)
            orient_qz = math.sin(orient_yaw / 2.0)
            orient_qw = math.cos(orient_yaw / 2.0)

            orient_goal = PoseStamped()
            orient_goal.header.frame_id = 'map'
            orient_goal.header.stamp = self.get_clock().now().to_msg()
            orient_goal.pose.position.x = robot_x
            orient_goal.pose.position.y = robot_y
            orient_goal.pose.position.z = 0.0
            orient_goal.pose.orientation.x = 0.0
            orient_goal.pose.orientation.y = 0.0
            orient_goal.pose.orientation.z = orient_qz
            orient_goal.pose.orientation.w = orient_qw

            self.get_logger().info(
                f'[PICK] Orienting toward object at ({obj_map_x:.2f}, {obj_map_y:.2f}), '
                f'yaw={math.degrees(orient_yaw):.1f}°'
            )
            orient_result = await self._execute_navigate(goal_handle, orient_goal, is_subtask=True)
            if self._cancel_requested:
                result.success = False
                result.message = 'Pick task canceled during orientation'
                goal_handle.canceled()
                return result
            if not orient_result.success:
                self.get_logger().warn(
                    '[PICK] Orientation navigation failed or was aborted — continuing anyway'
                )
        else:
            self.get_logger().warn('[PICK] No base pose available, skipping orientation step')
        
        #wait for one second
        time.sleep(1.0)

        # ----------------------------------------------------------------
        # Phase 1b: Move arm to nav config before standoff search
        # ----------------------------------------------------------------
        self._current_status = 'Moving Arm to Nav Config'
        self.get_logger().info('[PICK] Moving arm to nav config before standoff search...')
        nav_success = await self._move_arm_to_named_position(goal_handle, 'nav')
        if not nav_success:
            self.get_logger().warn('[PICK] Failed to move arm to nav config, continuing anyway...')

        # ----------------------------------------------------------------
        # Phase 2: Generate standoff candidates
        # ----------------------------------------------------------------
        self._current_status = 'Searching Standoff Poses'
        self.get_logger().info(
            f'[PICK] Generating standoff candidates around '
            f'({obj_map_x:.2f}, {obj_map_y:.2f})'
        )

        candidates = self._generate_standoff_candidates(obj_map_x, obj_map_y)
        self.get_logger().info(f'[PICK] Generated {len(candidates)} raw candidates')

        # ----------------------------------------------------------------
        # Phase 3: Filter by costmap
        # ----------------------------------------------------------------
        costmap_msg = await self._get_costmap_snapshot()

        valid_candidates = []
        for (cx, cy, cyaw) in candidates:
            cost = self._costmap_cost_at(costmap_msg, cx, cy)
            if cost < self.costmap_lethal_threshold:
                valid_candidates.append((cx, cy, cyaw))
            else:
                self.get_logger().debug(
                    f'[PICK] Candidate ({cx:.2f}, {cy:.2f}) rejected: cost={cost}'
                )

        self.get_logger().info(
            f'[PICK] {len(valid_candidates)} candidates after costmap filter '
            f'(lethal threshold={self.costmap_lethal_threshold})'
        )

        if not valid_candidates:
            result.success = False
            result.message = 'Pick failed: No obstacle-free standoff positions found around object'
            goal_handle.abort()
            return result

        # ----------------------------------------------------------------
        # Phase 4: Rank by distance to current robot position
        # ----------------------------------------------------------------
        with self._pose_lock:
            base_pose = self._current_base_pose

        if base_pose is not None:
            robot_x = base_pose.position.x
            robot_y = base_pose.position.y
            valid_candidates.sort(
                key=lambda c: (c[0] - robot_x) ** 2 + (c[1] - robot_y) ** 2
            )
            self.get_logger().info(
                f'[PICK] Candidates ranked by distance from robot '
                f'({robot_x:.2f}, {robot_y:.2f})'
            )

        # ----------------------------------------------------------------
        # Phase 5: Iterate candidates — navigate → TF re-lookup → IK probe → execute
        # ----------------------------------------------------------------
        ik_fail_count = 0
        arm_success = False
        pick_z_offset = 0.125  # 12.5 cm above box top for approach

        for attempt_idx, (cx, cy, cyaw) in enumerate(valid_candidates):
            if self._cancel_requested:
                result.success = False
                result.message = 'Pick task canceled during standoff search'
                goal_handle.canceled()
                return result

            if ik_fail_count >= self.pick_max_retries:
                self.get_logger().error(
                    f'[PICK] Aborted: IK failed at {ik_fail_count} candidates '
                    f'(max_retries={self.pick_max_retries})'
                )
                result.success = False
                result.message = (
                    f'Pick aborted: Target unreachable after '
                    f'{ik_fail_count} IK-probe failures'
                )
                goal_handle.abort()
                return result

            # Build yaw → quaternion for the base goal pose
            qz = math.sin(cyaw / 2.0)
            qw = math.cos(cyaw / 2.0)

            standoff_pose = PoseStamped()
            standoff_pose.header.frame_id = 'map'
            standoff_pose.header.stamp = self.get_clock().now().to_msg()
            standoff_pose.pose.position.x = cx
            standoff_pose.pose.position.y = cy
            standoff_pose.pose.position.z = 0.0
            standoff_pose.pose.orientation.x = 0.0
            standoff_pose.pose.orientation.y = 0.0
            standoff_pose.pose.orientation.z = qz
            standoff_pose.pose.orientation.w = qw

            self.get_logger().info(
                f'[PICK] Candidate {attempt_idx + 1}/{len(valid_candidates)}: '
                f'navigating to ({cx:.2f}, {cy:.2f}, yaw={math.degrees(cyaw):.0f}°)'
            )

            # --- Phase 4a: Navigate to standoff pose ---
            self._current_status = 'Navigating to Standoff'
            nav_result = await self._execute_navigate(
                goal_handle, standoff_pose, is_subtask=True
            )

            if not nav_result.success:
                self.get_logger().warn(
                    f'[PICK] Navigation to candidate {attempt_idx + 1} failed: '
                    f'{nav_result.message}. Trying next...'
                )
                if self._cancel_requested:
                    result.success = False
                    result.message = 'Pick task canceled during navigation to standoff'
                    return result
                ik_fail_count += 1
                continue

            if self._cancel_requested:
                result.success = False
                result.message = 'Pick task canceled after navigation to standoff'
                goal_handle.canceled()
                return result

            #sleep for one second after navigation
            time.sleep(1.0)
            # --- Phase 4b: Fresh TF lookup in planning frame after navigation ---
            self._current_status = 'Looking up Object Pose'
            self.get_logger().info(
                f'[PICK] Re-looking up TF for "{box_name}" from candidate '
                f'{attempt_idx + 1} position...'
            )

            tf_pose = self._get_pose_from_tf(box_name, z_offset=pick_z_offset)

            if tf_pose is None:
                self.get_logger().warn(
                    f'[PICK] TF lookup failed at candidate {attempt_idx + 1}. '
                    f'Trying next...'
                )
                ik_fail_count += 1
                continue

            # Override orientation: gripper pointing down
            tf_pose.pose.orientation.x = 0.0
            tf_pose.pose.orientation.y = 0.707
            tf_pose.pose.orientation.z = 0.0
            tf_pose.pose.orientation.w = 0.707

            self.get_logger().info(
                f'[PICK] Object in {self.planning_frame}: '
                f'({tf_pose.pose.position.x:.3f}, '
                f'{tf_pose.pose.position.y:.3f}, '
                f'{tf_pose.pose.position.z:.3f}) orientation=gripper-down'
            )

            # --- Phase 4c: MoveIt IK probe (plan only, no execution) ---
            self._current_status = 'IK Probe'
            self.get_logger().info(
                f'[PICK] Running MoveIt IK probe for candidate {attempt_idx + 1}...'
            )

            ik_ok = await self._probe_arm_ik(goal_handle, tf_pose)

            if not ik_ok:
                self.get_logger().warn(
                    f'[PICK] IK probe failed at candidate {attempt_idx + 1}. '
                    f'Trying next...'
                )
                ik_fail_count += 1
                continue

            # --- Phase 4d: Execute the validated plan ---
            self.get_logger().info(
                f'[PICK] IK probe succeeded at candidate {attempt_idx + 1}. '
                f'Executing pick motion...'
            )
            self._current_status = 'Executing Pick Motion'
            arm_success = await self._move_arm_to_pose(goal_handle, tf_pose)
            break  # Done — exit search loop

        # ----------------------------------------------------------------
        # Check if we exhausted all candidates without success
        # ----------------------------------------------------------------
        if not arm_success:
            if ik_fail_count >= self.pick_max_retries:
                pass  # Already aborted inside the loop
            else:
                result.success = False
                result.message = (
                    'Pick failed: Exhausted all standoff candidates without '
                    'successful arm motion'
                )
                goal_handle.abort()
            return result

        if self._cancel_requested:
            result.success = False
            result.message = 'Pick task canceled after arm motion'
            goal_handle.canceled()
            return result

        # sleep for one second after arm motion
        time.sleep(1.0)
        # ----------------------------------------------------------------
        # Phase 5: Make box dynamic, then attach
        # ----------------------------------------------------------------
        self._current_status = 'Preparing Object'
        self.get_logger().info(f'[PICK] Making "{box_name}" dynamic for manipulation...')

        dynamic_success, dynamic_msg = await self._call_set_box_state_service(
            box_name, make_dynamic=True
        )

        #sleep for one second after making box dynamic
        time.sleep(1.0)

        if not dynamic_success:
            self.get_logger().warn(
                f'[PICK] Failed to make box dynamic: {dynamic_msg}. '
                f'Attempting attach anyway...'
            )
        else:
            self.get_logger().info('[PICK] Waiting for physics to stabilize...')
            time.sleep(3.0)

        if self._cancel_requested:
            result.success = False
            result.message = 'Pick task canceled before attach'
            goal_handle.canceled()
            return result

        self._current_status = 'Grasping Object'
        self.get_logger().info('[PICK] Attaching object...')

        attach_success, attach_msg, joint_name = await self._call_attach_service(box_name)

        if not attach_success:
            result.success = False
            result.message = f'Pick attach failed: {attach_msg}'
            goal_handle.abort()
            return result

        self._attached_object_joint = joint_name
        self._attached_object_name = box_name  # Remember for static toggle on place

        if self._cancel_requested:
            result.success = False
            result.message = 'Pick task canceled after attach'
            goal_handle.canceled()
            return result

        # ----------------------------------------------------------------
        # Phase 6: Return arm to home
        # ----------------------------------------------------------------
        
        self._current_status = 'Returning to Home'
        self.get_logger().info('[PICK] Returning arm to home position...')

        home_success = await self._move_arm_to_named_position(goal_handle, 'home')

        result.success = True
        if home_success:
            result.message = f'Pick completed and arm returned to home: {attach_msg}'
        else:
            self.get_logger().warn('[PICK] Failed to return arm to home, but pick succeeded')
            result.message = f'Pick completed but failed to return to home: {attach_msg}'

        goal_handle.succeed()
        return result

    # ------------------------------------------------------------------
    # Place standoff search helpers
    # ------------------------------------------------------------------

    async def _get_costmap_snapshot(self):
        """
        Fetch the current global costmap as a nav2_msgs/Costmap snapshot.
        Returns the Costmap message, or None on failure.
        """
        if not self._get_costmap_client.service_is_ready():
            self.get_logger().warn('[PLACE] GetCostmap service not ready, skipping costmap filter')
            return None
        
        request = GetCostmap.Request()
        future = self._get_costmap_client.call_async(request)
        try:
            response = await future
            return response.map
        except Exception as e:
            self.get_logger().warn(f'[PLACE] GetCostmap call failed: {e}')
            return None

    def _costmap_cost_at(self, costmap_msg, map_x: float, map_y: float, robot_radius: float = 0.35) -> int:
        """
        Look up the WORST (highest) costmap cost within a circular patch of
        radius `robot_radius` around (map_x, map_y).

        Checking a single cell is insufficient because a candidate's center cell
        may sit in a low-cost zone on the outer edge of an inflation gradient,
        while cells just a few centimetres away (where the robot body would be)
        are lethal.  Taking the maximum cost over the full footprint patch gives
        a conservative, correct answer.

        Returns cost as integer 0-255, or 255 (lethal) if out of bounds.
        """
        if costmap_msg is None:
            return 0  # Assume free if we have no map

        meta = costmap_msg.metadata
        res = meta.resolution
        ox = meta.origin.position.x
        oy = meta.origin.position.y
        width = meta.size_x
        height = meta.size_y

        # How many grid cells does the robot radius span?
        r_cells = int(math.ceil(robot_radius / res))

        center_cx = int((map_x - ox) / res)
        center_cy = int((map_y - oy) / res)

        max_cost = 0
        for dy in range(-r_cells, r_cells + 1):
            for dx in range(-r_cells, r_cells + 1):
                # Stay within circular footprint
                if dx * dx + dy * dy > r_cells * r_cells:
                    continue
                gx = center_cx + dx
                gy = center_cy + dy
                if gx < 0 or gy < 0 or gx >= width or gy >= height:
                    return 255  # Any out-of-bounds cell → lethal
                idx = gy * width + gx
                cost = costmap_msg.data[idx]
                if cost > max_cost:
                    max_cost = cost

        return max_cost

    def _generate_standoff_candidates(self, target_x: float, target_y: float) -> list:
        """
        Generate candidate base positions on a ring around (target_x, target_y).
        Returns a list of (x, y, yaw) tuples where yaw faces the target.
        Samples every place_angle_step degrees for each radius step.
        """
        candidates = []
        # Sample two radius rings: inner and outer
        radius_steps = [self.pick_radius_min, self.pick_radius_max]
        angle_step_rad = math.radians(self.place_angle_step)
        num_steps = int(round(360.0 / self.place_angle_step))
        
        for r in radius_steps:
            for i in range(num_steps):
                theta = i * angle_step_rad
                cx = target_x + r * math.cos(theta)
                cy = target_y + r * math.sin(theta)
                # Robot should face the target
                yaw = math.atan2(target_y - cy, target_x - cx)
                candidates.append((cx, cy, yaw))
        
        return candidates

    async def _execute_place(self, goal_handle, target_pose: PoseStamped) -> RobotTask.Result:
        """
        Atomic Place Action with Dynamic Reachability Search.

        Algorithm:
          Phase 0: Generate candidate standoff base poses around the target.
          Phase 1: Filter candidates by global costmap (reject lethal cells).
          Phase 2: Rank by distance to current robot position.
          Phase 3: Iterate through candidates:
            a. Navigate to candidate base pose (Nav2).
            b. Transform target to planning frame.
            c. Probe MoveIt (plan only) — if fail, try next candidate.
            d. Execute the validated plan.
          Phase 4: Detach object, make box static, return arm to home.

        After place_max_retries IK failures, abort with "Target Unreachable".
        """
        result = RobotTask.Result()
        attached_box_name = self._attached_object_name
        
        # ----------------------------------------------------------------
        # Phase 0: Generate standoff candidates
        # ----------------------------------------------------------------
        target_x = target_pose.pose.position.x
        target_y = target_pose.pose.position.y
        target_z = target_pose.pose.position.z  # preserve Z for arm goal
        
        self._current_status = 'Searching Standoff Poses'
        self.get_logger().info(
            f'[PLACE] Generating standoff candidates around '
            f'({target_x:.2f}, {target_y:.2f})'
        )
        
        candidates = self._generate_standoff_candidates(target_x, target_y)
        self.get_logger().info(f'[PLACE] Generated {len(candidates)} raw candidates')
        
        # ----------------------------------------------------------------
        # Phase 1: Filter by costmap
        # ----------------------------------------------------------------
        costmap_msg = await self._get_costmap_snapshot()
        
        valid_candidates = []
        for (cx, cy, cyaw) in candidates:
            cost = self._costmap_cost_at(costmap_msg, cx, cy)
            if cost < self.costmap_lethal_threshold:
                valid_candidates.append((cx, cy, cyaw))
            else:
                self.get_logger().debug(
                    f'[PLACE] Candidate ({cx:.2f}, {cy:.2f}) rejected: cost={cost}'
                )
        
        self.get_logger().info(
            f'[PLACE] {len(valid_candidates)} candidates after costmap filter '
            f'(lethal threshold={self.costmap_lethal_threshold})'
        )
        
        if not valid_candidates:
            result.success = False
            result.message = 'Place failed: No obstacle-free standoff positions found around target'
            goal_handle.abort()
            return result
        
        # ----------------------------------------------------------------
        # Phase 2: Rank by distance to current robot position
        # ----------------------------------------------------------------
        with self._pose_lock:
            base_pose = self._current_base_pose
        
        if base_pose is not None:
            robot_x = base_pose.position.x
            robot_y = base_pose.position.y
            valid_candidates.sort(
                key=lambda c: (c[0] - robot_x) ** 2 + (c[1] - robot_y) ** 2
            )
            self.get_logger().info(
                f'[PLACE] Candidates ranked by distance from robot '
                f'({robot_x:.2f}, {robot_y:.2f})'
            )
        
        # ----------------------------------------------------------------
        # Phase 3: Iterate candidates — navigate → IK probe → execute
        # ----------------------------------------------------------------
        ik_fail_count = 0
        arm_success = False
        validated_transformed_pose = None
        
        for attempt_idx, (cx, cy, cyaw) in enumerate(valid_candidates):
            if self._cancel_requested:
                result.success = False
                result.message = 'Place task canceled during standoff search'
                goal_handle.canceled()
                return result
            
            if ik_fail_count >= self.place_max_retries:
                self.get_logger().error(
                    f'[PLACE] Aborted: IK failed at {ik_fail_count} candidates '
                    f'(max_retries={self.place_max_retries})'
                )
                result.success = False
                result.message = (
                    f'Place aborted: Target unreachable after '
                    f'{ik_fail_count} IK-probe failures'
                )
                goal_handle.abort()
                return result
            
            # Build a yaw → quaternion for the base goal pose
            # q = [0, 0, sin(yaw/2), cos(yaw/2)]
            qz = math.sin(cyaw / 2.0)
            qw = math.cos(cyaw / 2.0)
            
            standoff_pose = PoseStamped()
            standoff_pose.header.frame_id = 'map'
            standoff_pose.header.stamp = self.get_clock().now().to_msg()
            standoff_pose.pose.position.x = cx
            standoff_pose.pose.position.y = cy
            standoff_pose.pose.position.z = 0.0
            standoff_pose.pose.orientation.x = 0.0
            standoff_pose.pose.orientation.y = 0.0
            standoff_pose.pose.orientation.z = qz
            standoff_pose.pose.orientation.w = qw
            
            self.get_logger().info(
                f'[PLACE] Candidate {attempt_idx + 1}/{len(valid_candidates)}: '
                f'navigating to ({cx:.2f}, {cy:.2f}, yaw={math.degrees(cyaw):.0f}°)'
            )
            
            # --- Phase 3a: Navigate to standoff pose (sub-step, don't terminate goal handle) ---
            self._current_status = 'Navigating to Standoff'
            nav_result = await self._execute_navigate(goal_handle, standoff_pose, is_subtask=True)
            
            if not nav_result.success:
                # Navigation failure (e.g. blocked path) — try next candidate
                self.get_logger().warn(
                    f'[PLACE] Navigation to candidate {attempt_idx + 1} failed: '
                    f'{nav_result.message}. Trying next...'
                )
                # Reset goal handle state for continued iteration
                # (navigate already called abort/cancel; we need a fresh goal handle check)
                if self._cancel_requested:
                    result.success = False
                    result.message = 'Place task canceled during navigation to standoff'
                    return result
                ik_fail_count += 1
                continue
            
            if self._cancel_requested:
                result.success = False
                result.message = 'Place task canceled after navigation to standoff'
                goal_handle.canceled()
                return result
            
            # --- Phase 3b: Transform target to planning frame ---
            self._current_status = 'Transforming Pose'
            
            # Rebuild target pose with correct Z preserved
            place_target_map = PoseStamped()
            place_target_map.header.frame_id = target_pose.header.frame_id or 'map'
            place_target_map.header.stamp = self.get_clock().now().to_msg()
            place_target_map.pose.position.x = target_x
            place_target_map.pose.position.y = target_y
            place_target_map.pose.position.z = target_z
            place_target_map.pose.orientation.w = 1.0
            
            transformed_pose = self._transform_pose_to_planning_frame(place_target_map)
            
            if transformed_pose is None:
                self.get_logger().warn(
                    f'[PLACE] Cannot transform target to planning frame at candidate '
                    f'{attempt_idx + 1}. Trying next...'
                )
                ik_fail_count += 1
                continue
            
            # Fix orientation: gripper pointing down
            transformed_pose.pose.orientation.x = 0.0
            transformed_pose.pose.orientation.y = 0.707
            transformed_pose.pose.orientation.z = 0.0
            transformed_pose.pose.orientation.w = 0.707
            
            self.get_logger().info(
                f'[PLACE] Target in {self.planning_frame}: '
                f'({transformed_pose.pose.position.x:.3f}, '
                f'{transformed_pose.pose.position.y:.3f}, '
                f'{transformed_pose.pose.position.z:.3f}) orientation=gripper-down'
            )
            
            # --- Phase 3c: MoveIt IK probe (plan only) ---
            self._current_status = 'IK Probe'
            self.get_logger().info(
                f'[PLACE] Running MoveIt IK probe for candidate {attempt_idx + 1}...'
            )
            
            ik_ok = await self._probe_arm_ik(goal_handle, transformed_pose)
            
            if not ik_ok:
                self.get_logger().warn(
                    f'[PLACE] IK probe failed at candidate {attempt_idx + 1}. '
                    f'Marking invalid and trying next...'
                )
                ik_fail_count += 1
                continue
            
            # --- Phase 3d: Execute the validated plan ---
            self.get_logger().info(
                f'[PLACE] IK probe succeeded at candidate {attempt_idx + 1}. '
                f'Executing place motion...'
            )
            self._current_status = 'Executing Place Motion'
            arm_success = await self._move_arm_to_pose(goal_handle, transformed_pose)
            validated_transformed_pose = transformed_pose
            break  # Done — exit the search loop
        
        # ----------------------------------------------------------------
        # Check if we exhausted all candidates without success
        # ----------------------------------------------------------------
        if not arm_success:
            if ik_fail_count >= self.place_max_retries:
                # Already aborted inside the loop
                pass
            else:
                result.success = False
                result.message = (
                    'Place failed: Exhausted all standoff candidates without '
                    'successful arm motion'
                )
                goal_handle.abort()
            return result
        
        if self._cancel_requested:
            result.success = False
            result.message = 'Place task canceled after arm motion'
            goal_handle.canceled()
            return result
        
        # ----------------------------------------------------------------
        # Phase 4: Detach object
        # ----------------------------------------------------------------
        self._current_status = 'Releasing Object'
        self.get_logger().info('[PLACE] Detaching object...')
        
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
        
        # Phase 5: Make box static (freeze in place)
        if attached_box_name:
            self._current_status = 'Freezing Object'
            self.get_logger().info(f'[PLACE] Making {attached_box_name} static...')
            
            static_success, static_msg = await self._call_set_box_state_service(
                attached_box_name, make_dynamic=False
            )
            
            if not static_success:
                self.get_logger().warn(
                    f'[PLACE] Failed to make box static: {static_msg}. Continuing...'
                )
            
            self._attached_object_name = None
        
        if self._cancel_requested:
            result.success = False
            result.message = 'Place task canceled after detach'
            goal_handle.canceled()
            return result
        
        # Phase 6: Return arm to home
        self._current_status = 'Returning to Home'
        self.get_logger().info('[PLACE] Returning arm to home position...')
        
        zero_success = await self._move_arm_to_named_position(goal_handle, 'home')
        
        if not zero_success:
            self.get_logger().warn('[PLACE] Failed to return arm to home, but place was successful')
        
        # Phase 7: Move arm to nav config
        self._current_status = 'Moving Arm to Nav Config'
        self.get_logger().info('[PLACE] Moving arm to nav config...')
        nav_success = await self._move_arm_to_named_position(goal_handle, 'nav')
        if not nav_success:
            self.get_logger().warn('[PLACE] Failed to move arm to nav config, continuing anyway...')
        
        if zero_success:
            result.success = True
            result.message = f'Place completed, arm returned to home and nav config: {detach_msg}'
        else:
            result.success = True
            result.message = f'Place completed but failed to return to home: {detach_msg}'
        goal_handle.succeed()
        return result

    async def _probe_arm_ik(self, goal_handle, target_pose: PoseStamped) -> bool:
        """
        Send a plan-only MoveIt goal to validate IK reachability at the current
        robot base position WITHOUT executing any trajectory.

        Returns True if MoveIt found a valid plan, False otherwise.
        """
        moveit_goal = MoveGroup.Goal()
        moveit_goal.request.group_name = self.planning_group
        moveit_goal.request.num_planning_attempts = 5
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

        # Pose constraint
        constraints = Constraints()
        constraints.name = 'ik_probe_goal'

        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.planning_frame
        position_constraint.header.stamp = self.get_clock().now().to_msg()
        position_constraint.link_name = self.ee_link
        bounding_volume = BoundingVolume()
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.01]
        bounding_volume.primitives.append(primitive)
        bounding_volume.primitive_poses.append(target_pose.pose)
        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0
        constraints.position_constraints.append(position_constraint)

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
        # KEY: plan only — do NOT execute the trajectory
        moveit_goal.planning_options.plan_only = True
        moveit_goal.planning_options.replan = False
        moveit_goal.planning_options.replan_attempts = 0

        send_goal_future = self._moveit_client.send_goal_async(moveit_goal)
        probe_handle = await send_goal_future

        if not probe_handle.accepted:
            self.get_logger().warn('[PLACE] IK probe goal rejected by MoveIt')
            return False

        result_future = probe_handle.get_result_async()
        feedback_period = 1.0 / self.feedback_rate

        while not result_future.done():
            if self._cancel_requested:
                probe_handle.cancel_goal_async()
                return False
            # Publish lightweight feedback so the action client stays warm
            feedback = RobotTask.Feedback()
            feedback.current_status = 'IK Probe'
            goal_handle.publish_feedback(feedback)
            time.sleep(feedback_period)

        probe_result = result_future.result()
        return probe_result.status == GoalStatus.STATUS_SUCCEEDED

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
            'nav': {
                'arm_base_link1_joint': 0.0,
                'arm_link1_link2p1_joint': 0.0,
                'arm_link2p3_link3_joint': -1.2,
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
        if not self._attach_client.wait_for_service(timeout_sec=5.0):
            return False, 'Attach service not available', None
        
        request = AttachObject.Request()
        request.parent_model = self.robot_namespace
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
        if not self._detach_client.wait_for_service(timeout_sec=5.0):
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
        if not self._set_box_state_client.wait_for_service(timeout_sec=5.0):
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
