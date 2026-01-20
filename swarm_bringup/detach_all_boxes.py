#!/usr/bin/env python3
"""
Detach All Aruco Boxes at Startup

This script sends detach commands to all 48 aruco boxes for a specified robot.
Run this after the robot arm has spawned to release all initially-attached boxes.

Usage:
    ros2 run swarm_bringup detach_all_boxes --ros-args -p robot_ns:=ur5_assembly
    
Or via command line:
    python3 detach_all_boxes.py ur5_assembly
"""

import subprocess
import sys
import time

# List of all aruco box IDs that exist in the simulation
# Note: Boxes 24 and 25 are excluded (they don't exist in the world file)
ARUCO_BOX_IDS = [
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
    10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
    20, 21, 22, 23,
    # 24, 25 are excluded - don't exist
    26, 27, 28, 29,
    30, 31, 32, 33, 34, 35, 36, 37, 38, 39,
    40, 41, 42, 43, 44, 45, 46, 47, 48, 49
]


def detach_all_boxes(robot_ns: str, delay: float = 0.05):
    """
    Send detach commands for all aruco boxes.
    
    Args:
        robot_ns: Robot namespace (e.g., 'ur5_assembly', 'mobile_manipulator')
        delay: Delay between detach commands (seconds)
    """
    print(f"Detaching all {len(ARUCO_BOX_IDS)} boxes from {robot_ns}...")
    
    success_count = 0
    fail_count = 0
    
    for box_id in ARUCO_BOX_IDS:
        topic = f"/{robot_ns}/detach_box_{box_id}"
        
        try:
            result = subprocess.run(
                ['ign', 'topic', '-t', topic, '-m', 'ignition.msgs.Empty', '-p', ''],
                capture_output=True,
                text=True,
                timeout=2
            )
            
            if result.returncode == 0:
                success_count += 1
            else:
                fail_count += 1
                print(f"  Warning: Failed to detach box_{box_id}")
                
        except subprocess.TimeoutExpired:
            fail_count += 1
            print(f"  Warning: Timeout for box_{box_id}")
        except Exception as e:
            fail_count += 1
            print(f"  Error for box_{box_id}: {e}")
        
        # Small delay to avoid overwhelming the system
        time.sleep(delay)
    
    print(f"Done! Detached: {success_count}, Failed: {fail_count}")
    return success_count, fail_count


def main():
    # Default robot namespace
    robot_ns = "ur5_assembly"
    
    # Check for command line argument
    if len(sys.argv) > 1:
        robot_ns = sys.argv[1]
    
    # Optional: Check for ROS parameter (if run as ROS node)
    try:
        import rclpy
        from rclpy.node import Node
        
        rclpy.init()
        node = Node('detach_all_boxes')
        node.declare_parameter('robot_ns', 'ur5_assembly')
        robot_ns = node.get_parameter('robot_ns').get_parameter_value().string_value
        node.destroy_node()
        rclpy.shutdown()
    except:
        pass  # Not running as ROS node, use command line arg
    
    print(f"Robot namespace: {robot_ns}")
    print("Waiting 2 seconds for simulation to stabilize...")
    time.sleep(2)
    
    detach_all_boxes(robot_ns)


if __name__ == '__main__':
    main()
