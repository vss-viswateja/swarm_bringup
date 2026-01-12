# swarm_bringup

ROS 2 Humble launch orchestration package for multi-robot swarm systems and mobile manipulator simulation using Gazebo (Ignition). This package provides comprehensive launch infrastructure for deploying both Jackal UGV swarms with Nav2 navigation and UR5 arm-equipped mobile manipulators.

> **Note**: This package is part of the work-in-progress CHARS (Collaborative Heterogeneous Autonomous Robot Swarm) architecture.

## Features

- **Multi-Robot Swarm Simulation**: Sequential spawning of multiple Jackal UGVs with Nav2 navigation stack
- **Mobile Manipulator Support**: Single UR5 robotic arm simulation with MoveIt2 integration
- **Gazebo Integration**: Full Ignition Gazebo (Fortress/Garden) support with ros_gz_bridge
- **Namespace Isolation**: TF frame prefixing for multi-robot compatibility
- **Staggered Deployment**: TimerAction-based sequential robot spawning to prevent collisions
- **Pre-configured Worlds**: Test environments and maps for simulation
- **RViz Visualization**: Custom RViz configurations for swarm and mobile manipulator views

## Package Contents

```
swarm_bringup/
├── launch/
│   ├── bringup_ign.launch.py          # Multi-robot Jackal swarm launcher
│   └── single_arm_moveit.launch.py    # UR5 + MoveIt2 simulation launcher
├── config/
│   ├── swarm_bridge.yaml              # Gazebo-ROS bridge for Jackal swarm
│   ├── mobile_swarm_bridge.yaml       # Bridge config for mobile manipulators
│   ├── swarm.rviz                     # RViz config for swarm visualization
│   └── mobile_swarm.rviz              # RViz config for mobile manipulator
├── worlds/
│   ├── empty_world.sdf                # Minimal test environment
│   ├── test_world_v1.sdf              # Basic test world
│   └── test_world_v2.sdf              # Advanced test environment
└── maps/
    ├── construction_site_v1.pgm       # Nav2 map for construction site
    └── construction_site_v1.yaml      # Map metadata
```

## Dependencies

### ROS 2 Packages
```bash
# Core simulation and control
ros-humble-ros-gz-sim
ros-humble-ros-gz-bridge
ros-humble-robot-state-publisher
ros-humble-controller-manager
ros-humble-xacro

# Navigation
ros-humble-nav2-bringup
ros-humble-navigation2

# MoveIt2 (for mobile manipulator)
ros-humble-moveit
ros-humble-moveit-ros-move-group
ros-humble-moveit-configs-utils

# Visualization
ros-humble-rviz2
```

### Workspace Packages
- `swarm_description` - URDF/Xacro models for Jackal, UR5, and mobile manipulator
- `ur5_moveit_config` - MoveIt2 configuration for UR5 arm

### Installation

1. **Install ROS 2 Humble** (if not already installed):
```bash
sudo apt update
sudo apt install ros-humble-desktop
```

2. **Install dependencies**:
```bash
# Navigation stack
sudo apt install ros-humble-nav2-bringup ros-humble-navigation2

# Gazebo simulation
sudo apt install ros-humble-ros-gz ros-humble-ros-gz-sim ros-humble-ros-gz-bridge

# Robot control
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers
sudo apt install ros-humble-controller-manager
sudo apt install ros-humble-diff-drive-controller
sudo apt install ros-humble-robot-localization

# MoveIt2
sudo apt install ros-humble-moveit ros-humble-moveit-configs-utils

# Utilities
sudo apt install ros-humble-xacro
```

3. **Clone and build workspace**:
```bash
cd ~/swarm_ws/src
# Clone your repository here

cd ~/swarm_ws
colcon build --packages-select swarm_bringup swarm_description
source install/setup.bash
```

## Usage

### Multi-Robot Jackal Swarm

Launch multiple Jackal UGVs with Nav2 navigation:

```bash
# Default: 5 robots with test_world_v2
ros2 launch swarm_bringup bringup_ign.launch.py

# Custom world file
ros2 launch swarm_bringup bringup_ign.launch.py \
    world_file:=/path/to/custom_world.sdf

# Disable simulation time
ros2 launch swarm_bringup bringup_ign.launch.py use_sim_time:=false
```

**Robot Configuration** (currently configured in `bringup_ign.launch.py`):
- robot1: spawns at (0.0, -3.0, 0.22) with 15s delay
- robot2: spawns at (0.0, -2.0, 0.22) with 25s delay
- robot3: spawns at (0.0, -1.0, 0.22) with 35s delay
- robot4: spawns at (-1.0, -3.0, 0.22) with 45s delay
- robot5: spawns at (-1.0, -2.0, 0.22) with 55s delay

> **Note**: Multi-robot spawning is currently disabled by default (commented out). Uncomment the robot spawn loop in the launch file to enable.

### Single UR5 Arm with MoveIt2

Launch a single UR5 robotic arm with motion planning:

```bash
# Default launch with RViz
ros2 launch swarm_bringup single_arm_moveit.launch.py

# Without RViz
ros2 launch swarm_bringup single_arm_moveit.launch.py use_rviz:=false

# Custom namespace and position
ros2 launch swarm_bringup single_arm_moveit.launch.py \
    robot_namespace:=robot1 \
    spawn_x:=1.0 \
    spawn_y:=0.0 \
    spawn_z:=0.125

# Custom world
ros2 launch swarm_bringup single_arm_moveit.launch.py \
    world_file:=/path/to/world.sdf
```

**Launch Sequence Timing**:
- 0s: Gazebo server/GUI, robot state publisher, robot spawn
- 15s: Joint state broadcaster controller
- 20s: Arm position controller
- 25s: MoveIt2 move_group node
- 27s: RViz (if enabled)

### Testing Commands

**Check robot namespaces**:
```bash
ros2 topic list | grep robot1
```

**Send navigation goal** (for Jackal swarm):
```bash
ros2 topic pub /robot1/goal_pose geometry_msgs/msg/PoseStamped "{...}"
```

**Control UR5 via MoveIt** (after launch):
- Use RViz's MoveIt Motion Planning plugin
- Or programmatically via MoveIt2 Python/C++ API

**Monitor TF tree**:
```bash
ros2 run tf2_tools view_frames
evince frames.pdf
```

## Architecture Notes

### Namespace Strategy
All robots use the `robot_namespace` parameter with TF frame prefixing:
- Links/joints: `${robot_namespace}_base_link`, `${robot_namespace}_link_1`, etc.
- Topics: `/robot1/cmd_vel`, `/robot2/scan`, etc.
- Controllers: Scoped under `/${robot_namespace}/controller_manager`

### Gazebo Bridge Configuration
The `ros_gz_bridge` translates Gazebo topics to ROS 2:
- Clock synchronization (`/clock`)
- IMU data (`/jackal/imu`)
- LiDAR scans (`/jackal/scan`, `/jackal/scan/points`)
- Camera streams (`/camera/camera_info`, `/camera/image`)

Custom bridge configurations in [config/swarm_bridge.yaml](config/swarm_bridge.yaml).

### Staggered Robot Spawning
`TimerAction` delays prevent Gazebo physics collisions:
```python
delayed_spawn = TimerAction(
    period=robot['spawn_delay'],  # e.g., 15.0s
    actions=[spawn_control]
)
```

## Configuration Files

### RViz Configs
- **swarm.rviz**: Multi-robot visualization with TF trees and sensor overlays
- **mobile_swarm.rviz**: Mobile manipulator view with MoveIt planning scene

### World Files
- **empty_world.sdf**: Minimal ground plane for basic testing
- **test_world_v1.sdf**: Simple obstacles and structures
- **test_world_v2.sdf**: Complex environment with walls and objects

### Maps
Navigation maps in PGM format with YAML metadata for Nav2 AMCL/Costmap layers.

## Troubleshooting

**Gazebo models not loading**:
```bash
# Check resource paths are set correctly
echo $IGN_GAZEBO_RESOURCE_PATH
echo $GZ_SIM_RESOURCE_PATH

# Ensure swarm_description meshes are accessible
ls $(ros2 pkg prefix swarm_description)/share/swarm_description/meshes
```

**Controllers not loading**:
```bash
# Check controller manager
ros2 control list_controllers --controller-manager /robot1/controller_manager

# Manually load if needed
ros2 control load_controller --controller-manager /robot1/controller_manager diff_drive_controller
ros2 control set_controller_state --controller-manager /robot1/controller_manager diff_drive_controller active
```

**MoveIt planning failures**:
- Ensure joint limits are within URDF specifications
- Check collision scene for obstacles blocking motion
- Verify controller is publishing joint states: `ros2 topic echo /ur5/joint_states`

**Navigation not working**:
```bash
# Check map server
ros2 topic echo /robot1/map --once

# Verify localization
ros2 topic echo /robot1/amcl_pose

# Inspect costmaps
ros2 run nav2_costmap_2d nav2_costmap_2d_markers voxel_grid:=/robot1/global_costmap/voxel_grid visualization_marker:=/robot1/global_costmap_marker
```

## Development Roadmap (CHARS Architecture)

- [ ] Enable multi-robot spawning by default
- [ ] Add dynamic robot configuration from YAML
- [ ] Implement centralized swarm coordinator node
- [ ] Integrate mobile manipulator (Jackal + UR5) composite model
- [ ] Add multi-robot task allocation system
- [ ] Implement formation control behaviors
- [ ] Create custom Nav2 plugins for swarm navigation

## License

Apache License 2.0

## Maintainer

**Viswa Teja Bottu**  
Email: vss.viswatejabottu@gmail.com

## Contributing

This package is part of ongoing research in heterogeneous multi-robot systems. For questions or collaboration opportunities, please contact the maintainer.

## See Also

- [swarm_description](../swarm_description/) - Robot URDF models and descriptions
- [plansys2_turtlesim_example](../plansys2_turtlesim_example/) - VLM-powered PDDL planning integration
- [Nav2 Documentation](https://navigation.ros.org/)
- [MoveIt2 Documentation](https://moveit.picknik.ai/humble/)
- [Gazebo Documentation](https://gazebosim.org/)
