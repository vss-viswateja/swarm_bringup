#!/usr/bin/env python3
"""
UR5 Task Action Server

This node implements an Action Server for the /ur5/task_control action (RobotTask interface).
It handles pick and place tasks using MoveIt2 for arm control and gripper services.

Supported task types:
- 'pick': Moves arm to target pose, then calls attach_object service
- 'place': Moves arm to target pose, then calls detach_object service

Features:
- Action Server: /ur5/task_control (RobotTask)
- MoveIt2 integration for arm control (arm planning group)
- Attach/Detach service integration
- Preemption handling with immediate MoveIt2 cancellation
- Feedback every 500ms with EE pose and status

Usage:
    ros2 run swarm_bringup ur5_task_action_server
    ros2 launch swarm_bringup ur5_task_action_server.launch.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Pose

from swarm_interfaces.action import RobotTask
from swarm_interfaces.srv import AttachObject, DetachObject
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    MoveItErrorCodes,
)
from shape_msgs.msg import SolidPrimitive

from tf2_ros import Buffer, TransformListener

import threading
import asyncio
from action_msgs.msg import GoalStatus


class UR5TaskActionServer(Node):
    """
    Action Server that handles pick and place tasks for UR5 robotic arm.
    
    Uses MoveIt2 for motion planning and gripper services for object manipulation.
    """

    def __init__(self):
        super().__init__('ur5_task_action_server')
        
        # Callback groups
        self.action_cb_group = ReentrantCallbackGroup()
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        
        # Parameters
        self.declare_parameter('robot_namespace', 'ur5')
        self.declare_parameter('planning_group', 'arm')
        self.declare_parameter('end_effector_link', 'ur5/link6_1')
        self.declare_parameter('planning_frame', 'ur5/world')
        self.declare_parameter('feedback_rate', 2.0)  # 500ms = 2Hz
        self.declare_parameter('planning_time', 10.0)
        self.declare_parameter('max_velocity_scaling', 0.3)
        self.declare_parameter('max_acceleration_scaling', 0.3)
        
        self.robot_namespace = self.get_parameter('robot_namespace').value
        self.planning_group = self.get_parameter('planning_group').value
        self.ee_link = self.get_parameter('end_effector_link').value
        self.planning_frame = self.get_parameter('planning_frame').value
        self.feedback_rate = self.get_parameter('feedback_rate').value
        self.planning_time = self.get_parameter('planning_time').value
        self.max_vel_scaling = self.get_parameter('max_velocity_scaling').value
        self.max_acc_scaling = self.get_parameter('max_acceleration_scaling').value
        
        # State tracking
        self._current_status = 'Idle'
        self._is_executing = False
        self._cancel_requested = False
        
        # MoveIt goal tracking
        self._moveit_goal_handle = None
        self._moveit_lock = threading.Lock()
        
        # Attached object tracking (for place task)
        self._attached_object_joint = None
        
        # TF2 buffer for EE pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Action Client for MoveIt MoveGroup
        moveit_action_name = f'/{self.robot_namespace}/move_action'
        self._moveit_client = ActionClient(
            self,
            MoveGroup,
            moveit_action_name,
            callback_group=self.action_cb_group
        )
        
        # Service clients for attach/detach
        self._attach_client = self.create_client(
            AttachObject,
            '/attach_object',
            callback_group=self.service_cb_group
        )
        self._detach_client = self.create_client(
            DetachObject,
            '/detach_object',
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
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('UR5 Task Action Server Started')
        self.get_logger().info(f'  Robot Namespace: {self.robot_namespace}')
        self.get_logger().info(f'  Task Action: {task_action_name}')
        self.get_logger().info(f'  MoveIt Action: {moveit_action_name}')
        self.get_logger().info(f'  Planning Group: {self.planning_group}')
        self.get_logger().info(f'  End Effector: {self.ee_link}')
        self.get_logger().info(f'  Planning Frame: {self.planning_frame}')
        self.get_logger().info(f'  Feedback Rate: {self.feedback_rate} Hz')
        self.get_logger().info('=' * 60)

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
            return Pose()

    def _goal_callback(self, goal_request):
        """Handle incoming goal requests."""
        task_type = goal_request.task_type.lower()
        
        self.get_logger().info(f'Received goal request: task_type="{task_type}"')
        
        # Validate task type (UR5 only supports pick and place)
        if task_type not in ['pick', 'place']:
            self.get_logger().warn(f'Unsupported task type: {task_type} (UR5 supports: pick, place)')
            return GoalResponse.REJECT
        
        # Check MoveIt availability
        if not self._moveit_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('MoveIt move_action server not available!')
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """Handle cancel requests - immediately stop MoveIt execution."""
        self.get_logger().info('Received cancel request - stopping arm motion')
        self._cancel_requested = True
        self._cancel_moveit_goal()
        return CancelResponse.ACCEPT

    def _cancel_moveit_goal(self):
        """Cancel active MoveIt goal immediately."""
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
            f'({target_pose.pose.position.x:.3f}, '
            f'{target_pose.pose.position.y:.3f}, '
            f'{target_pose.pose.position.z:.3f})'
        )
        
        result = RobotTask.Result()
        
        try:
            if task_type == 'pick':
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

    async def _execute_pick(self, goal_handle, target_pose: PoseStamped) -> RobotTask.Result:
        """Execute pick task: move arm to target, then attach object."""
        result = RobotTask.Result()
        
        # Phase 1: Plan and execute arm motion
        self._current_status = 'Planning Path'
        self.get_logger().info('Planning arm motion for pick...')
        
        arm_success, arm_error = await self._move_arm_to_pose(goal_handle, target_pose)
        
        if not arm_success:
            result.success = False
            result.message = f'MoveIt2 planning/execution failed: {arm_error}'
            goal_handle.abort()
            return result
        
        if self._cancel_requested:
            result.success = False
            result.message = 'Pick task canceled'
            goal_handle.canceled()
            return result
        
        # Phase 2: Attach object
        self._current_status = 'Actuating Gripper'
        self.get_logger().info('Attaching object...')
        
        # Publish one more feedback
        feedback = RobotTask.Feedback()
        feedback.current_status = 'Actuating Gripper'
        feedback.current_pose = self._get_ee_pose()
        goal_handle.publish_feedback(feedback)
        
        attach_success, attach_msg, joint_name = await self._call_attach_service(target_pose)
        
        if attach_success:
            self._attached_object_joint = joint_name
            result.success = True
            result.message = f'Pick completed successfully: {attach_msg}'
            goal_handle.succeed()
        else:
            result.success = False
            result.message = f'Gripper attach failed: {attach_msg}'
            goal_handle.abort()
        
        return result

    async def _execute_place(self, goal_handle, target_pose: PoseStamped) -> RobotTask.Result:
        """Execute place task: move arm to target, then detach object."""
        result = RobotTask.Result()
        
        # Phase 1: Plan and execute arm motion
        self._current_status = 'Planning Path'
        self.get_logger().info('Planning arm motion for place...')
        
        arm_success, arm_error = await self._move_arm_to_pose(goal_handle, target_pose)
        
        if not arm_success:
            result.success = False
            result.message = f'MoveIt2 planning/execution failed: {arm_error}'
            goal_handle.abort()
            return result
        
        if self._cancel_requested:
            result.success = False
            result.message = 'Place task canceled'
            goal_handle.canceled()
            return result
        
        # Phase 2: Detach object
        self._current_status = 'Actuating Gripper'
        self.get_logger().info('Detaching object...')
        
        # Publish one more feedback
        feedback = RobotTask.Feedback()
        feedback.current_status = 'Actuating Gripper'
        feedback.current_pose = self._get_ee_pose()
        goal_handle.publish_feedback(feedback)
        
        if self._attached_object_joint:
            detach_success, detach_msg = await self._call_detach_service(self._attached_object_joint)
            self._attached_object_joint = None
        else:
            detach_success = True
            detach_msg = 'No object was attached'
        
        if detach_success:
            result.success = True
            result.message = f'Place completed successfully: {detach_msg}'
            goal_handle.succeed()
        else:
            result.success = False
            result.message = f'Gripper detach failed: {detach_msg}'
            goal_handle.abort()
        
        return result

    async def _move_arm_to_pose(self, goal_handle, target_pose: PoseStamped) -> tuple:
        """
        Move arm to target pose using MoveIt.
        Returns (success: bool, error_message: str)
        """
        # Create MoveGroup goal
        moveit_goal = MoveGroup.Goal()
        moveit_goal.request.group_name = self.planning_group
        moveit_goal.request.num_planning_attempts = 10
        moveit_goal.request.allowed_planning_time = self.planning_time
        moveit_goal.request.max_velocity_scaling_factor = self.max_vel_scaling
        moveit_goal.request.max_acceleration_scaling_factor = self.max_acc_scaling
        
        # Workspace bounds
        moveit_goal.request.workspace_parameters.header.frame_id = self.planning_frame
        moveit_goal.request.workspace_parameters.header.stamp = self.get_clock().now().to_msg()
        moveit_goal.request.workspace_parameters.min_corner.x = -2.0
        moveit_goal.request.workspace_parameters.min_corner.y = -2.0
        moveit_goal.request.workspace_parameters.min_corner.z = -1.0
        moveit_goal.request.workspace_parameters.max_corner.x = 2.0
        moveit_goal.request.workspace_parameters.max_corner.y = 2.0
        moveit_goal.request.workspace_parameters.max_corner.z = 2.0
        
        # Create pose constraint
        constraints = Constraints()
        constraints.name = 'arm_pose_goal'
        
        # Position constraint
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = target_pose.header.frame_id or self.planning_frame
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
        
        # Orientation constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = target_pose.header.frame_id or self.planning_frame
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
        self._current_status = 'Executing Motion'
        self.get_logger().info('Sending arm motion goal to MoveIt...')
        
        send_goal_future = self._moveit_client.send_goal_async(moveit_goal)
        moveit_goal_handle = await send_goal_future
        
        if not moveit_goal_handle.accepted:
            self.get_logger().error('MoveIt goal was rejected!')
            return False, 'Goal rejected by MoveIt'
        
        self.get_logger().info('MoveIt goal accepted, executing motion...')
        
        with self._moveit_lock:
            self._moveit_goal_handle = moveit_goal_handle
        
        # Wait for result with feedback loop
        result_future = moveit_goal_handle.get_result_async()
        feedback_period = 1.0 / self.feedback_rate
        
        while not result_future.done():
            if self._cancel_requested:
                self._cancel_moveit_goal()
                return False, 'Motion canceled by user'
            
            # Publish feedback with EE pose
            feedback = RobotTask.Feedback()
            feedback.current_status = self._current_status
            feedback.current_pose = self._get_ee_pose()
            goal_handle.publish_feedback(feedback)
            
            await asyncio.sleep(feedback_period)
        
        # Process result
        moveit_result = result_future.result()
        
        with self._moveit_lock:
            self._moveit_goal_handle = None
        
        if moveit_result.status == GoalStatus.STATUS_SUCCEEDED:
            # Check MoveIt error code
            error_code = moveit_result.result.error_code.val
            if error_code == MoveItErrorCodes.SUCCESS:
                self.get_logger().info('Arm motion completed successfully!')
                return True, 'Success'
            else:
                error_msg = self._moveit_error_to_string(error_code)
                self.get_logger().error(f'MoveIt error: {error_msg}')
                return False, error_msg
        elif moveit_result.status == GoalStatus.STATUS_CANCELED:
            return False, 'Motion canceled'
        else:
            return False, f'Motion failed with status: {moveit_result.status}'

    def _moveit_error_to_string(self, error_code: int) -> str:
        """Convert MoveIt error code to string."""
        error_map = {
            MoveItErrorCodes.SUCCESS: 'Success',
            MoveItErrorCodes.FAILURE: 'General failure',
            MoveItErrorCodes.PLANNING_FAILED: 'Planning failed - no valid path found',
            MoveItErrorCodes.INVALID_MOTION_PLAN: 'Invalid motion plan',
            MoveItErrorCodes.MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE: 'Plan invalidated by environment',
            MoveItErrorCodes.CONTROL_FAILED: 'Controller execution failed',
            MoveItErrorCodes.UNABLE_TO_AQUIRE_SENSOR_DATA: 'Unable to acquire sensor data',
            MoveItErrorCodes.TIMED_OUT: 'Planning timed out',
            MoveItErrorCodes.PREEMPTED: 'Preempted',
            MoveItErrorCodes.START_STATE_IN_COLLISION: 'Start state in collision',
            MoveItErrorCodes.START_STATE_VIOLATES_PATH_CONSTRAINTS: 'Start state violates constraints',
            MoveItErrorCodes.GOAL_IN_COLLISION: 'Goal state in collision',
            MoveItErrorCodes.GOAL_VIOLATES_PATH_CONSTRAINTS: 'Goal violates constraints',
            MoveItErrorCodes.GOAL_CONSTRAINTS_VIOLATED: 'Goal constraints violated',
            MoveItErrorCodes.INVALID_GROUP_NAME: 'Invalid group name',
            MoveItErrorCodes.INVALID_GOAL_CONSTRAINTS: 'Invalid goal constraints',
            MoveItErrorCodes.INVALID_ROBOT_STATE: 'Invalid robot state',
            MoveItErrorCodes.INVALID_LINK_NAME: 'Invalid link name',
            MoveItErrorCodes.INVALID_OBJECT_NAME: 'Invalid object name',
            MoveItErrorCodes.FRAME_TRANSFORM_FAILURE: 'Frame transform failure',
            MoveItErrorCodes.COLLISION_CHECKING_UNAVAILABLE: 'Collision checking unavailable',
            MoveItErrorCodes.ROBOT_STATE_STALE: 'Robot state stale',
            MoveItErrorCodes.SENSOR_INFO_STALE: 'Sensor info stale',
            MoveItErrorCodes.NO_IK_SOLUTION: 'No IK solution found',
        }
        return error_map.get(error_code, f'Unknown error code: {error_code}')

    async def _call_attach_service(self, target_pose: PoseStamped):
        """Call attach_object service."""
        if not self._attach_client.wait_for_service(timeout_sec=2.0):
            return False, 'Attach service not available', None
        
        request = AttachObject.Request()
        request.parent_model = 'ur5'
        request.parent_link = 'link6_1'
        # Extract object name from frame_id (e.g., 'aruco_box_0')
        request.child_model = target_pose.header.frame_id if target_pose.header.frame_id else 'object'
        request.child_link = 'link'
        
        self.get_logger().info(f'Calling attach: parent=ur5/link6_1, child={request.child_model}')
        
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
        
        self.get_logger().info(f'Calling detach: joint_name={joint_name}')
        
        future = self._detach_client.call_async(request)
        
        try:
            response = await future
            return response.success, response.message
        except Exception as e:
            return False, str(e)


def main(args=None):
    rclpy.init(args=args)
    
    node = UR5TaskActionServer()
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
