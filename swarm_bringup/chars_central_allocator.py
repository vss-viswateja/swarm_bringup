#!/usr/bin/env python3
"""
CHARS Central Allocator Node

Cognitive-Hierarchical Architecture for Robust Swarm Intelligence (CHARS) 2.0
Centralized Task Allocator - The "Master" node that transforms PDDL-derived tasks
into real-time robot assignments using a vectorized Utility Matrix.

Mathematical Framework:
    H_{i,j} = Φ_{i,j} × [w_t × S_time + w_e × S_energy + w_r × S_reach]

Where:
    - Φ: Compatibility Matrix (hard filter based on task_type)
    - S_time: Time score = e^(-α × D)
    - S_energy: Energy score = (1 - e^(-β / M_f)) × (B_curr / 100)
    - S_reach: Reachability score = e^(-γ × δ)

Agent Modes:
    - 'mobman' (default): Homogeneous fleet of N mobile manipulators.
      Agent namespaces are set via the 'agent_namespaces' parameter.
    - 'mixed': Heterogeneous fleet (Jackal + UR5 + MobMan) with
      hardcoded capabilities and special-case scoring.

Task Input:
    - Single tasks via /chars/task_request (PoseStamped with 'type:frame' encoding)
    - Full task DAG via /chars/task_dag (JSON String with dependencies)

Features:
    - DAG-aware scheduling: dispatches all dependency-met tasks in parallel
    - Multi-threaded executor for concurrent robot feedback
    - Configurable homogeneous or heterogeneous agent fleet
    - Utility Matrix calculation with hard filters
    - Availability tracking with A vector
    - Failure recovery with global recalculation
    - Live DAG status published on /chars/dag_status

Usage:
    ros2 run swarm_bringup chars_central_allocator
    ros2 launch swarm_bringup chars_central_allocator.launch.py
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String

from swarm_interfaces.action import RobotTask
from nav2_msgs.action import ComputePathToPose

from tf2_ros import Buffer, TransformListener

import numpy as np
import math
import json
import threading
from enum import Enum
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Callable
from action_msgs.msg import GoalStatus


class AgentStatus(Enum):
    """Agent availability status."""
    IDLE = 0
    BUSY = 1
    FAILED = 2


class TaskStatus(Enum):
    """Task execution status."""
    PENDING = 0
    ASSIGNED = 1
    IN_PROGRESS = 2
    COMPLETED = 3
    FAILED = 4


@dataclass
class Agent:
    """Agent (robot) representation."""
    name: str
    namespace: str
    status: AgentStatus = AgentStatus.IDLE
    current_task: Optional[str] = None
    position: Optional[Pose] = None
    battery_percent: float = 100.0
    mass_factor: float = 1.0
    capabilities: List[str] = field(default_factory=list)
    goal_handle: Optional[object] = None


@dataclass
class Task:
    """Task representation from PDDL plan."""
    task_id: str
    task_type: str  # 'navigate', 'pick', 'place'
    target_pose: PoseStamped
    status: TaskStatus = TaskStatus.PENDING
    assigned_agent: Optional[str] = None
    priority: int = 0
    dependencies: List[str] = field(default_factory=list)


class CHARSCentralAllocator(Node):
    """
    CHARS Centralized Allocator - The Master node for multi-robot task allocation.
    
    Implements the Utility Matrix framework for optimal task-agent assignment.
    """

    def __init__(self):
        super().__init__('chars_central_allocator')
        
        # Callback group for concurrent operations
        self.callback_group = ReentrantCallbackGroup()
        
        # ===================== Parameters =====================
        self.declare_parameter('alpha', 0.5)      # Time score decay factor
        self.declare_parameter('beta', 1.0)       # Energy score shape factor
        self.declare_parameter('gamma', 0.3)      # Reachability decay factor
        self.declare_parameter('w_time', 0.4)     # Time weight
        self.declare_parameter('w_energy', 0.2)   # Energy weight
        self.declare_parameter('w_reach', 0.4)    # Reachability weight
        self.declare_parameter('ur5_nav_distance', 1000.0)  # High D for UR5 nav tasks
        self.declare_parameter('allocation_rate', 1.0)  # Hz
        self.declare_parameter('agent_type', 'mobman')  # 'mobman' or 'mixed'
        self.declare_parameter('agent_namespaces', ['agent1', 'agent2', 'agent3'])
        
        self.alpha = self.get_parameter('alpha').value
        self.beta = self.get_parameter('beta').value
        self.gamma = self.get_parameter('gamma').value
        self.w_time = self.get_parameter('w_time').value
        self.w_energy = self.get_parameter('w_energy').value
        self.w_reach = self.get_parameter('w_reach').value
        self.ur5_nav_distance = self.get_parameter('ur5_nav_distance').value
        self.allocation_rate = self.get_parameter('allocation_rate').value
        self.agent_type = self.get_parameter('agent_type').value
        self.agent_namespaces = self.get_parameter('agent_namespaces').value
        
        # ===================== Agent Registry =====================
        self.agents: Dict[str, Agent] = self._build_agent_registry()
        
        # Compatibility Matrix Φ (task_type -> agent -> compatible)
        self.compatibility_matrix = self._build_compatibility_matrix()
        
        # ===================== Task Queue =====================
        self.task_queue: List[Task] = []
        self.task_lock = threading.Lock()
        
        # ===================== TF2 for Position Tracking =====================
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # ===================== Action Clients =====================
        self.action_clients: Dict[str, ActionClient] = {}
        for agent_name, agent in self.agents.items():
            action_name = f'/{agent.namespace}/task_control'
            self.action_clients[agent_name] = ActionClient(
                self,
                RobotTask,
                action_name,
                callback_group=self.callback_group
            )
        
        # ===================== Path Computation Action Clients =====================
        self.path_clients: Dict[str, ActionClient] = {}
        for agent_name, agent in self.agents.items():
            if 'navigate' in agent.capabilities:
                action_name = f'/{agent.namespace}/compute_path_to_pose'
                self.path_clients[agent_name] = ActionClient(
                    self,
                    ComputePathToPose,
                    action_name,
                    callback_group=self.callback_group
                )
        
        # ===================== Publishers/Subscribers =====================
        self.status_pub = self.create_publisher(
            String, '/chars/allocator_status', 10
        )
        
        self.dag_status_pub = self.create_publisher(
            String, '/chars/dag_status', 10
        )
        
        # Task input subscriber (from PDDL planner or manual input)
        self.task_sub = self.create_subscription(
            PoseStamped, '/chars/task_request',
            self._task_request_callback, 10,
            callback_group=self.callback_group
        )
        
        # DAG input subscriber (JSON task graph)
        self.dag_sub = self.create_subscription(
            String, '/chars/task_dag',
            self._dag_callback, 10,
            callback_group=self.callback_group
        )
        
        # ===================== Allocation Timer =====================
        self.allocation_timer = self.create_timer(
            1.0 / self.allocation_rate,
            self._allocation_loop,
            callback_group=self.callback_group
        )
        
        # ===================== Startup =====================
        self._publish_status('CHARS Central Allocator Started')
        self.get_logger().info('=' * 70)
        self.get_logger().info('CHARS Central Allocator v2.0 Started')
        self.get_logger().info(f'  Agent Type: {self.agent_type}')
        self.get_logger().info('  Registered Agents: ' + ', '.join(self.agents.keys()))
        self.get_logger().info(f'  Weights: w_time={self.w_time}, w_energy={self.w_energy}, w_reach={self.w_reach}')
        self.get_logger().info(f'  Decay factors: α={self.alpha}, β={self.beta}, γ={self.gamma}')
        self.get_logger().info('=' * 70)
        
        # Check action server availability
        self._check_agent_connections()

    def _build_agent_registry(self) -> Dict[str, Agent]:
        """Build agent registry based on agent_type parameter."""
        if self.agent_type == 'mixed':
            # Heterogeneous: original Jackal + UR5 + MobMan
            return {
                'jackal': Agent(
                    name='jackal',
                    namespace='jackal',
                    mass_factor=1.0,
                    capabilities=['navigate']
                ),
                'ur5': Agent(
                    name='ur5',
                    namespace='ur5',
                    mass_factor=1.0,
                    capabilities=['pick', 'place']
                ),
                'mobman': Agent(
                    name='mobman',
                    namespace='mobman',
                    mass_factor=1.8,
                    capabilities=['navigate', 'pick', 'place']
                ),
            }
        else:
            # Homogeneous: N mobile manipulators
            agents = {}
            for ns in self.agent_namespaces:
                agents[ns] = Agent(
                    name=ns,
                    namespace=ns,
                    mass_factor=1.8,
                    capabilities=['navigate', 'pick', 'place']
                )
            return agents

    def _build_compatibility_matrix(self) -> Dict[str, Dict[str, int]]:
        """Build compatibility matrix Φ based on agent capabilities."""
        task_types = ['navigate', 'pick', 'place']
        matrix = {}
        for task_type in task_types:
            matrix[task_type] = {}
            for agent_name, agent in self.agents.items():
                matrix[task_type][agent_name] = 1 if task_type in agent.capabilities else 0
        return matrix

    def _check_agent_connections(self):
        """Check if all agent action servers are available."""
        for agent_name, client in self.action_clients.items():
            available = client.wait_for_server(timeout_sec=2.0)
            status = "CONNECTED" if available else "OFFLINE"
            self.get_logger().info(f'  Agent {agent_name}: {status}')

    def _publish_status(self, message: str):
        """Publish allocator status."""
        msg = String()
        msg.data = message
        self.status_pub.publish(msg)

    def _task_request_callback(self, msg: PoseStamped):
        """Handle incoming single task requests (legacy interface)."""
        # Parse task type from frame_id (e.g., "navigate:map" or "pick:aruco_box_0")
        frame_parts = msg.header.frame_id.split(':')
        task_type = frame_parts[0] if frame_parts else 'navigate'
        frame_id = frame_parts[1] if len(frame_parts) > 1 else 'map'
        
        msg.header.frame_id = frame_id
        
        task = Task(
            task_id=f'task_{len(self.task_queue)}',
            task_type=task_type,
            target_pose=msg
        )
        
        with self.task_lock:
            self.task_queue.append(task)
        
        self.get_logger().info(f'Queued task: {task.task_id} ({task.task_type})')

    def _dag_callback(self, msg: String):
        """
        Handle incoming task DAG (Directed Acyclic Graph).
        
        Expected JSON format:
        {
          "tasks": [
            {
              "task_id": "nav_to_table",
              "task_type": "navigate",
              "frame_id": "map",
              "pose": {"x": 1.5, "y": 2.0, "z": 0.0,
                       "qx": 0.0, "qy": 0.0, "qz": 0.0, "qw": 1.0},
              "dependencies": []
            },
            {
              "task_id": "pick_box",
              "task_type": "pick",
              "frame_id": "aruco_box_0",
              "pose": {},
              "dependencies": ["nav_to_table"]
            }
          ]
        }
        """
        try:
            dag_data = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse DAG JSON: {e}')
            return
        
        tasks_data = dag_data.get('tasks', [])
        if not tasks_data:
            self.get_logger().warn('Received empty DAG')
            return
        
        # Validate: check for duplicate task IDs
        task_ids = [t.get('task_id', '') for t in tasks_data]
        if len(task_ids) != len(set(task_ids)):
            self.get_logger().error('DAG contains duplicate task IDs')
            return
        
        # Validate: check that all dependency references exist
        task_id_set = set(task_ids)
        for t_data in tasks_data:
            for dep in t_data.get('dependencies', []):
                if dep not in task_id_set:
                    self.get_logger().error(
                        f'DAG error: task "{t_data.get("task_id")}" depends on '
                        f'unknown task "{dep}"'
                    )
                    return
        
        # Clear existing task queue and load the DAG
        with self.task_lock:
            self.task_queue.clear()
            
            for t_data in tasks_data:
                pose_data = t_data.get('pose', {})
                
                target_pose = PoseStamped()
                target_pose.header.frame_id = t_data.get('frame_id', 'map')
                target_pose.pose.position.x = float(pose_data.get('x', 0.0))
                target_pose.pose.position.y = float(pose_data.get('y', 0.0))
                target_pose.pose.position.z = float(pose_data.get('z', 0.0))
                target_pose.pose.orientation.x = float(pose_data.get('qx', 0.0))
                target_pose.pose.orientation.y = float(pose_data.get('qy', 0.0))
                target_pose.pose.orientation.z = float(pose_data.get('qz', 0.0))
                target_pose.pose.orientation.w = float(pose_data.get('qw', 1.0))
                
                task = Task(
                    task_id=t_data['task_id'],
                    task_type=t_data.get('task_type', 'navigate'),
                    target_pose=target_pose,
                    dependencies=t_data.get('dependencies', [])
                )
                
                self.task_queue.append(task)
        
        self.get_logger().info(f'Loaded DAG with {len(self.task_queue)} tasks')
        for task in self.task_queue:
            deps_str = ', '.join(task.dependencies) if task.dependencies else 'none'
            self.get_logger().info(
                f'  {task.task_id} ({task.task_type}) deps=[{deps_str}]'
            )
        
        self._publish_dag_status()

    def add_task(self, task_type: str, target_pose: PoseStamped, 
                 priority: int = 0, dependencies: List[str] = None):
        """Programmatically add a task to the queue."""
        task = Task(
            task_id=f'task_{len(self.task_queue)}',
            task_type=task_type,
            target_pose=target_pose,
            priority=priority,
            dependencies=dependencies or []
        )
        
        with self.task_lock:
            self.task_queue.append(task)
        
        self.get_logger().info(f'Added task: {task.task_id} ({task.task_type})')
        return task.task_id

    def _allocation_loop(self):
        """
        Main allocation loop - runs at allocation_rate Hz.
        
        Iterates all pending tasks, dispatches every task whose
        dependencies are met to the best available agent.
        Multiple independent tasks can be dispatched in a single cycle.
        """
        with self.task_lock:
            pending_tasks = [t for t in self.task_queue if t.status == TaskStatus.PENDING]
        
        if not pending_tasks:
            return
        
        # Get availability vector (updated within the loop as agents get assigned)
        availability = self._get_availability_vector()
        
        dispatched_any = False
        
        for task in pending_tasks:
            # Check dependencies
            if not self._dependencies_satisfied(task):
                continue
            
            # Check if any agent is still available
            if not any(availability.values()):
                self.get_logger().debug('No more agents available this cycle')
                break
            
            # Calculate Utility Matrix for this task
            utility_scores = self._calculate_utility_matrix(task, availability)
            
            if not utility_scores:
                continue
            
            # Select best agent (argmax)
            best_agent = max(utility_scores, key=utility_scores.get)
            best_score = utility_scores[best_agent]
            
            if best_score <= 0:
                continue
            
            self.get_logger().info(
                f'Allocating {task.task_id} ({task.task_type}) to {best_agent} '
                f'(score={best_score:.3f})'
            )
            
            # Dispatch task
            self._dispatch_task(task, best_agent)
            
            # Mark agent as unavailable for the rest of this cycle
            availability[best_agent] = False
            dispatched_any = True
        
        if dispatched_any:
            self._publish_dag_status()

    def _dependencies_satisfied(self, task: Task) -> bool:
        """Check if all task dependencies are completed."""
        for dep_id in task.dependencies:
            dep_task = next((t for t in self.task_queue if t.task_id == dep_id), None)
            if dep_task and dep_task.status != TaskStatus.COMPLETED:
                return False
        return True

    def _get_availability_vector(self) -> Dict[str, bool]:
        """Get availability vector A where a_j=1 if agent is idle."""
        return {
            name: agent.status == AgentStatus.IDLE
            for name, agent in self.agents.items()
        }

    def _calculate_utility_matrix(self, task: Task, 
                                  availability: Dict[str, bool]) -> Dict[str, float]:
        """
        Calculate utility scores for all agents for a given task.
        
        H_{i,j} = Φ_{i,j} × [w_t × S_time + w_e × S_energy + w_r × S_reach]
        """
        utility_scores = {}
        
        for agent_name, agent in self.agents.items():
            # Check availability (A vector masking)
            if not availability.get(agent_name, False):
                continue
            
            # Hard filter: Compatibility Matrix Φ
            phi = self._get_compatibility(task.task_type, agent_name)
            if phi == 0:
                continue
            
            # Calculate heuristic components
            s_time = self._calculate_time_score(agent, task)
            s_energy = self._calculate_energy_score(agent)
            s_reach = self._calculate_reachability_score(agent, task)
            
            # Combined utility
            utility = phi * (
                self.w_time * s_time +
                self.w_energy * s_energy +
                self.w_reach * s_reach
            )
            
            utility_scores[agent_name] = utility
            
            self.get_logger().debug(
                f'  {agent_name}: Φ={phi}, S_time={s_time:.3f}, '
                f'S_energy={s_energy:.3f}, S_reach={s_reach:.3f}, H={utility:.3f}'
            )
        
        return utility_scores

    def _get_compatibility(self, task_type: str, agent_name: str) -> int:
        """Get compatibility value Φ for task-agent pair."""
        return self.compatibility_matrix.get(task_type, {}).get(agent_name, 0)

    def _calculate_time_score(self, agent: Agent, task: Task) -> float:
        """
        Calculate time score: S_time = e^(-α × D)
        D = path distance from Nav2 ComputePathToPose
        """
        # UR5 cannot navigate - set D to high value
        if agent.name == 'ur5' and task.task_type == 'navigate':
            return math.exp(-self.alpha * self.ur5_nav_distance)
        
        # For manipulation tasks, use euclidean distance as proxy
        if task.task_type in ['pick', 'place']:
            distance = self._get_euclidean_distance(agent, task)
        else:
            # For navigation, try to get path distance (or fall back to euclidean)
            distance = self._get_path_distance(agent, task)
        
        return math.exp(-self.alpha * distance)

    def _calculate_energy_score(self, agent: Agent) -> float:
        """
        Calculate energy score: S_energy = (1 - e^(-β / M_f)) × (B_curr / 100)
        """
        if agent.name == 'ur5':
            # UR5 is plugged in, always full energy
            return 1.0
        
        mass_factor = agent.mass_factor
        battery = agent.battery_percent / 100.0
        
        return (1.0 - math.exp(-self.beta / mass_factor)) * battery

    def _calculate_reachability_score(self, agent: Agent, task: Task) -> float:
        """
        Calculate reachability score: S_reach = e^(-γ × δ)
        δ = Euclidean distance between robot base and target_pose
        """
        distance = self._get_euclidean_distance(agent, task)
        return math.exp(-self.gamma * distance)

    def _get_euclidean_distance(self, agent: Agent, task: Task) -> float:
        """Calculate Euclidean distance from agent to task target."""
        agent_pose = self._get_agent_pose(agent)
        if agent_pose is None:
            return 10.0  # Default high distance if pose unknown
        
        target = self._get_target_position(task)
        dx = target.x - agent_pose.position.x
        dy = target.y - agent_pose.position.y
        dz = target.z - agent_pose.position.z
        
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def _get_target_position(self, task: Task):
        """
        Get the actual target position for a task.
        
        For 'pick' tasks, the target_pose carries the object's TF frame name
        in header.frame_id (e.g. 'aruco_box_0') with dummy position values.
        This method looks up the real position from TF.
        
        For other tasks ('navigate', 'place'), the position in target_pose
        is already meaningful and is returned directly.
        """
        if task.task_type == 'pick':
            object_frame = task.target_pose.header.frame_id
            try:
                transform = self.tf_buffer.lookup_transform(
                    'map', object_frame,
                    rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5)
                )
                # Return the real object position from TF
                pose = Pose()
                pose.position.x = transform.transform.translation.x
                pose.position.y = transform.transform.translation.y
                pose.position.z = transform.transform.translation.z
                return pose.position
            except Exception as e:
                self.get_logger().warn(
                    f'Failed to look up TF for pick target "{object_frame}": {e}. '
                    f'Falling back to target_pose values.'
                )
        
        return task.target_pose.pose.position

    def _get_path_distance(self, agent: Agent, task: Task) -> float:
        """Get path distance from Nav2 ComputePathToPose service."""
        # For now, use euclidean as fallback
        # TODO: Implement async service call to ComputePathToPose
        return self._get_euclidean_distance(agent, task)

    def _get_agent_pose(self, agent: Agent) -> Optional[Pose]:
        """Get agent's current pose from TF."""
        try:
            frame = f'{agent.namespace}/base_link'
            if agent.name == 'ur5':
                frame = 'ur5/world'  # UR5 is fixed
            
            transform = self.tf_buffer.lookup_transform(
                'map', frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            pose = Pose()
            pose.position.x = transform.transform.translation.x
            pose.position.y = transform.transform.translation.y
            pose.position.z = transform.transform.translation.z
            pose.orientation = transform.transform.rotation
            
            agent.position = pose
            return pose
            
        except Exception as e:
            self.get_logger().debug(f'Failed to get pose for {agent.name}: {e}')
            return agent.position

    def _dispatch_task(self, task: Task, agent_name: str):
        """Dispatch task to selected agent."""
        agent = self.agents[agent_name]
        client = self.action_clients[agent_name]
        
        # Update states
        task.status = TaskStatus.ASSIGNED
        task.assigned_agent = agent_name
        agent.status = AgentStatus.BUSY
        agent.current_task = task.task_id
        
        # Create goal
        goal = RobotTask.Goal()
        goal.target_pose = task.target_pose
        goal.task_type = task.task_type
        
        self._publish_status(f'Dispatching {task.task_id} to {agent_name}')
        
        # Send goal asynchronously
        send_goal_future = client.send_goal_async(
            goal,
            feedback_callback=lambda fb: self._feedback_callback(agent_name, task, fb)
        )
        send_goal_future.add_done_callback(
            lambda future: self._goal_response_callback(future, agent_name, task)
        )

    def _goal_response_callback(self, future, agent_name: str, task: Task):
        """Handle goal acceptance/rejection."""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error(f'Goal rejected by {agent_name}')
            self._handle_failure(agent_name, task, 'Goal rejected')
            return
        
        self.get_logger().info(f'Goal accepted by {agent_name}')
        task.status = TaskStatus.IN_PROGRESS
        
        # Store goal handle for potential cancellation
        self.agents[agent_name].goal_handle = goal_handle
        
        # Wait for result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda future: self._result_callback(future, agent_name, task)
        )

    def _feedback_callback(self, agent_name: str, task: Task, feedback_msg):
        """Handle feedback from agent."""
        feedback = feedback_msg.feedback
        status = feedback.current_status
        
        self.get_logger().debug(f'{agent_name}: {status}')
        self._publish_status(f'{agent_name}: {status}')

    def _result_callback(self, future, agent_name: str, task: Task):
        """Handle task completion or failure."""
        result = future.result()
        agent = self.agents[agent_name]
        
        # Clear goal handle
        agent.goal_handle = None
        agent.current_task = None
        
        if result.status == GoalStatus.STATUS_SUCCEEDED and result.result.success:
            # Success!
            task.status = TaskStatus.COMPLETED
            agent.status = AgentStatus.IDLE
            
            self.get_logger().info(
                f'Task {task.task_id} COMPLETED by {agent_name}: '
                f'{result.result.message}'
            )
            self._publish_status(f'COMPLETED: {task.task_id} by {agent_name}')
            self._publish_dag_status()
            
        elif result.status == GoalStatus.STATUS_CANCELED:
            # Canceled
            self._handle_failure(agent_name, task, 'Task canceled')
            
        else:
            # Failed
            error_msg = result.result.message if result.result else 'Unknown error'
            self._handle_failure(agent_name, task, error_msg)

    def _handle_failure(self, agent_name: str, task: Task, error_msg: str):
        """Handle task failure with global recalculation."""
        self.get_logger().error(f'FAILURE: {task.task_id} on {agent_name}: {error_msg}')
        self._publish_status(f'FAILURE: {task.task_id} - {error_msg}')
        
        agent = self.agents[agent_name]
        
        # Reset task to pending for re-allocation
        task.status = TaskStatus.PENDING
        task.assigned_agent = None
        
        # Check if hardware is still functional
        # For now, assume it is and reset to IDLE
        agent.status = AgentStatus.IDLE
        agent.current_task = None
        
        self.get_logger().info(f'Triggering global recalculation for {task.task_id}')
        self._publish_dag_status()
        # The allocation loop will pick up the pending task

    def _publish_dag_status(self):
        """Publish the current DAG state as JSON on /chars/dag_status."""
        status_data = {
            'tasks': [
                {
                    'task_id': task.task_id,
                    'task_type': task.task_type,
                    'status': task.status.name,
                    'assigned_agent': task.assigned_agent,
                    'dependencies': task.dependencies,
                }
                for task in self.task_queue
            ]
        }
        msg = String()
        msg.data = json.dumps(status_data)
        self.dag_status_pub.publish(msg)

    def cancel_task(self, task_id: str):
        """Cancel a task and its assigned agent's goal."""
        task = next((t for t in self.task_queue if t.task_id == task_id), None)
        if not task:
            return False
        
        if task.assigned_agent:
            agent = self.agents[task.assigned_agent]
            if agent.goal_handle:
                agent.goal_handle.cancel_goal_async()
        
        task.status = TaskStatus.FAILED
        return True


def main(args=None):
    rclpy.init(args=args)
    
    node = CHARSCentralAllocator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down CHARS Allocator')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
