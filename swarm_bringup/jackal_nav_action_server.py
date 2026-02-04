#!/usr/bin/env python3
"""
Jackal Navigation Action Server

This node implements an Action Server for the /jackal/task_control action (RobotTask interface).
When a 'navigate' goal is received, it forwards the goal to the Jackal's Nav2 stack.

Features:
- Action Server: /jackal/task_control (RobotTask)
- Action Client: /jackal/navigate_to_pose (NavigateToPose)
- Handles preemption and cancellation
- Publishes feedback with current pose every 1 second

Usage:
    ros2 run swarm_bringup jackal_nav_action_server
    
    # Or via launch file:
    ros2 launch swarm_bringup jackal_nav_action_server.launch.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from swarm_interfaces.action import RobotTask
from nav2_msgs.action import NavigateToPose

import threading
import time


class JackalNavActionServer(Node):
    """
    Action Server that bridges RobotTask requests to Nav2 NavigateToPose.
    
    Handles 'navigate' task type by forwarding goals to Nav2 and publishing
    feedback with current pose.
    """

    def __init__(self):
        super().__init__('jackal_nav_action_server')
        
        # Callback group for concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()
        
        # Parameters
        self.declare_parameter('robot_namespace', 'jackal')
        self.declare_parameter('feedback_rate', 1.0)  # Hz
        
        self.robot_namespace = self.get_parameter('robot_namespace').value
        self.feedback_rate = self.get_parameter('feedback_rate').value
        
        # Current pose tracking
        self._current_pose = None
        self._current_pose_lock = threading.Lock()
        
        # Nav2 goal tracking
        self._nav2_goal_handle = None
        self._nav2_goal_lock = threading.Lock()
        self._is_navigating = False
        
        # Subscribe to AMCL pose for feedback
        pose_topic = f'/{self.robot_namespace}/amcl_pose'
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            pose_topic,
            self._pose_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Action Client for Nav2 NavigateToPose
        nav2_action_name = f'/{self.robot_namespace}/navigate_to_pose'
        self._nav2_client = ActionClient(
            self,
            NavigateToPose,
            nav2_action_name,
            callback_group=self.callback_group
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
            callback_group=self.callback_group
        )
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Jackal Navigation Action Server Started')
        self.get_logger().info(f'  Robot Namespace: {self.robot_namespace}')
        self.get_logger().info(f'  Task Action: {task_action_name}')
        self.get_logger().info(f'  Nav2 Action: {nav2_action_name}')
        self.get_logger().info(f'  Pose Topic: {pose_topic}')
        self.get_logger().info(f'  Feedback Rate: {self.feedback_rate} Hz')
        self.get_logger().info('=' * 60)

    def _pose_callback(self, msg: PoseWithCovarianceStamped):
        """Update current pose from AMCL."""
        with self._current_pose_lock:
            self._current_pose = msg.pose.pose

    def _goal_callback(self, goal_request):
        """Handle incoming goal requests."""
        task_type = goal_request.task_type.lower()
        
        self.get_logger().info(f'Received goal request: task_type="{task_type}"')
        
        # Validate task type
        if task_type not in ['navigate', 'pick', 'place']:
            self.get_logger().warn(f'Unknown task type: {task_type}')
            return GoalResponse.REJECT
        
        # Currently only 'navigate' is implemented
        if task_type != 'navigate':
            self.get_logger().warn(f'Task type "{task_type}" not implemented yet')
            return GoalResponse.REJECT
        
        # Check if Nav2 action server is available
        if not self._nav2_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('Nav2 navigate_to_pose action server not available!')
            return GoalResponse.REJECT
        
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """Handle cancel requests by canceling underlying Nav2 goal."""
        self.get_logger().info('Received cancel request')
        
        # Cancel the Nav2 goal if active
        self._cancel_nav2_goal()
        
        return CancelResponse.ACCEPT

    def _cancel_nav2_goal(self):
        """Cancel the current Nav2 goal if one exists."""
        with self._nav2_goal_lock:
            if self._nav2_goal_handle is not None:
                self.get_logger().info('Canceling Nav2 goal...')
                cancel_future = self._nav2_goal_handle.cancel_goal_async()
                self._nav2_goal_handle = None
                self._is_navigating = False
                return cancel_future
        return None

    async def _execute_callback(self, goal_handle):
        """Execute the RobotTask goal by forwarding to Nav2."""
        self.get_logger().info('Executing goal...')
        
        request = goal_handle.request
        target_pose = request.target_pose
        task_type = request.task_type.lower()
        
        self.get_logger().info(
            f'Task: {task_type}, Target: '
            f'({target_pose.pose.position.x:.2f}, '
            f'{target_pose.pose.position.y:.2f}, '
            f'{target_pose.pose.position.z:.2f})'
        )
        
        # Create Nav2 goal
        nav2_goal = NavigateToPose.Goal()
        nav2_goal.pose = target_pose
        
        # Send goal to Nav2
        self.get_logger().info('Sending goal to Nav2...')
        send_goal_future = self._nav2_client.send_goal_async(
            nav2_goal,
            feedback_callback=self._nav2_feedback_callback
        )
        
        # Wait for goal acceptance
        nav2_goal_handle = await send_goal_future
        
        if not nav2_goal_handle.accepted:
            self.get_logger().error('Nav2 goal was rejected!')
            goal_handle.abort()
            result = RobotTask.Result()
            result.success = False
            result.message = 'Nav2 rejected the navigation goal'
            return result
        
        self.get_logger().info('Nav2 goal accepted')
        
        # Store the goal handle for potential cancellation
        with self._nav2_goal_lock:
            self._nav2_goal_handle = nav2_goal_handle
            self._is_navigating = True
        
        # Start feedback publishing
        feedback = RobotTask.Feedback()
        feedback_period = 1.0 / self.feedback_rate
        
        # Wait for Nav2 result while publishing feedback
        result_future = nav2_goal_handle.get_result_async()
        
        while not result_future.done():
            # Check if this goal was canceled
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Goal canceled by client')
                self._cancel_nav2_goal()
                goal_handle.canceled()
                result = RobotTask.Result()
                result.success = False
                result.message = 'Navigation canceled'
                return result
            
            # Publish feedback
            feedback.current_status = 'Navigating'
            with self._current_pose_lock:
                if self._current_pose is not None:
                    feedback.current_pose = self._current_pose
            
            goal_handle.publish_feedback(feedback)
            
            # Wait before next feedback
            await self._sleep(feedback_period)
        
        # Get Nav2 result
        nav2_result = result_future.result()
        
        with self._nav2_goal_lock:
            self._nav2_goal_handle = None
            self._is_navigating = False
        
        # Create result based on Nav2 outcome
        result = RobotTask.Result()
        
        if nav2_result.status == 4:  # SUCCEEDED
            self.get_logger().info('Navigation succeeded!')
            result.success = True
            result.message = 'Navigation completed successfully'
            goal_handle.succeed()
        elif nav2_result.status == 5:  # CANCELED
            self.get_logger().info('Navigation was canceled')
            result.success = False
            result.message = 'Navigation was canceled'
            goal_handle.canceled()
        else:
            self.get_logger().error(f'Navigation failed with status: {nav2_result.status}')
            result.success = False
            result.message = f'Navigation failed with status code: {nav2_result.status}'
            goal_handle.abort()
        
        return result

    def _nav2_feedback_callback(self, feedback_msg):
        """Handle feedback from Nav2 (optional logging)."""
        # Nav2 provides distance remaining and other info
        distance = feedback_msg.feedback.distance_remaining
        self.get_logger().debug(f'Nav2 feedback: distance_remaining={distance:.2f}m')

    async def _sleep(self, duration: float):
        """Async sleep helper."""
        import asyncio
        await asyncio.sleep(duration)


def main(args=None):
    rclpy.init(args=args)
    
    node = JackalNavActionServer()
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
