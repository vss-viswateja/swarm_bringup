# Object State Manager

A ROS 2 node for dynamically switching aruco boxes between static and dynamic states in Gazebo simulation to optimize physics performance.

## Problem

Simulating 48 dynamic aruco boxes causes significant physics engine load, dropping the Real Time Factor (RTF) to ~3-30%. Most boxes don't need physics calculations until a robot actually manipulates them.

## Solution

This package provides:
1. A service to switch individual boxes between static (frozen) and dynamic (movable) states
2. A helper script to make all boxes static in the world file
3. Automatic pose tracking to preserve box locations during state changes

## Quick Start

### Step 1: Make Boxes Static in World File

```bash
cd ~/swarm_ws/src/swarm_bringup/scripts
python3 make_boxes_static.py ../worlds/test_world_v3.sdf ../worlds/test_world_v3_static.sdf
```

### Step 2: Launch Simulation with Static World

```bash
ros2 launch swarm_bringup bringup_ign.launch.py world_file:=/path/to/test_world_v3_static.sdf
```

### Step 3: Launch Object State Manager

```bash
ros2 launch swarm_bringup object_state_manager.launch.py world_name:=construction_world_v3
```

### Step 4: Switch Boxes as Needed

```bash
# Make a box dynamic (before robot picks it up)
ros2 service call /set_box_state swarm_interfaces/srv/SetBoxState "{box_name: 'aruco_box_26', make_dynamic: true}"

# Freeze a box (after robot places it)
ros2 service call /set_box_state swarm_interfaces/srv/SetBoxState "{box_name: 'aruco_box_26', make_dynamic: false}"

# Get current pose of a box
ros2 service call /get_box_pose swarm_interfaces/srv/GetBoxPose "{box_name: 'aruco_box_26'}"
```

## Integration with MoveIt/Manipulation Pipeline

In your manipulation node, call the service before and after grasping:

```python
from swarm_interfaces.srv import SetBoxState
from rclpy.node import Node

class ManipulationNode(Node):
    def __init__(self):
        super().__init__('manipulation_node')
        self.set_box_state_client = self.create_client(
            SetBoxState, 
            '/set_box_state'
        )
    
    def pick_box(self, box_name: str):
        # 1. Make box dynamic before picking
        request = SetBoxState.Request()
        request.box_name = box_name
        request.make_dynamic = True
        self.set_box_state_client.call_async(request)
        
        # 2. Execute pick motion
        self.execute_pick(box_name)
    
    def place_box(self, box_name: str, place_pose):
        # 1. Execute place motion
        self.execute_place(place_pose)
        
        # 2. Freeze box after placing
        request = SetBoxState.Request()
        request.box_name = box_name
        request.make_dynamic = False
        self.set_box_state_client.call_async(request)
```

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `world_name` | `construction_world_v3` | Name of the Gazebo world (must match SDF) |
| `use_sim_time` | `true` | Use simulation clock |

## Services

| Service | Type | Description |
|---------|------|-------------|
| `/set_box_state` | `swarm_interfaces/srv/SetBoxState` | Switch box static/dynamic |
| `/get_box_pose` | `swarm_interfaces/srv/GetBoxPose` | Get current box pose |

## Performance Impact

| Configuration | RTF (6 robots) |
|---------------|----------------|
| 48 Dynamic Boxes | ~3-10% |
| 48 Static Boxes + 1 Dynamic | ~80-95% |
| 48 Static Boxes + 4 Dynamic | ~60-80% |
