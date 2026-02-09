#!/usr/bin/env python3
"""
Trajectory Recorder and Plotter for Odometry Drift Analysis

This script records the robot's trajectory from two sources:
1. TF: The predicted pose from gz_world -> mobman/base_link
2. Gazebo: The actual pose from Gazebo simulation (via ign model CLI)

After recording, it plots both trajectories and the error magnitude over time.

Usage:
    ros2 run swarm_bringup trajectory_recorder --ros-args -p model_name:=mobman -p duration:=60.0
    
    Press Ctrl+C to stop recording and generate plots.
"""

import os
import re
import math
import csv
import subprocess
from datetime import datetime

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import matplotlib.pyplot as plt
import numpy as np


class TrajectoryRecorder(Node):
    """
    Records robot trajectory from TF and Gazebo ground truth,
    then plots the comparison and error analysis.
    """

    def __init__(self):
        super().__init__('trajectory_recorder')

        # Declare parameters
        self.declare_parameter('model_name', 'mobman')
        self.declare_parameter('sample_rate', 20.0)  # Hz (increased for better resolution)
        self.declare_parameter('duration', 120.0)  # seconds (0 = until Ctrl+C)
        self.declare_parameter('output_dir', '/tmp/trajectory_data')
        self.declare_parameter('parent_frame', 'gz_world')

        self.model_name = self.get_parameter('model_name').get_parameter_value().string_value
        self.sample_rate = self.get_parameter('sample_rate').get_parameter_value().double_value
        self.duration = self.get_parameter('duration').get_parameter_value().double_value
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value

        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)

        # Data storage
        self.tf_trajectory = []  # [(timestamp, x, y, z, yaw), ...]
        self.gz_trajectory = []  # [(timestamp, x, y, z, yaw), ...]
        self.start_time = None

        # TF2 setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Frame names
        self.child_frame = f'{self.model_name}/base_link'

        # Timer for sampling
        self.sample_timer = self.create_timer(
            1.0 / self.sample_rate,
            self.sample_callback
        )

        # Duration timer (if set)
        if self.duration > 0:
            self.duration_timer = self.create_timer(
                self.duration,
                self.stop_recording
            )

        self.get_logger().info(f'Trajectory Recorder started!')
        self.get_logger().info(f'  Model: {self.model_name}')
        self.get_logger().info(f'  Sample rate: {self.sample_rate} Hz')
        self.get_logger().info(f'  Duration: {self.duration}s (0 = until Ctrl+C)')
        self.get_logger().info(f'  Output dir: {self.output_dir}')
        self.get_logger().info(f'  TF frames: {self.parent_frame} -> {self.child_frame}')
        self.get_logger().info(f'  Gazebo pose: via "ign model -m {self.model_name} -p"')
        self.get_logger().info('Press Ctrl+C to stop and generate plots.')

    def sample_callback(self):
        """Sample both TF and Gazebo poses at each timer tick."""
        if self.start_time is None:
            self.start_time = self.get_clock().now()

        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9

        # Sample TF pose
        tf_pose = self.get_tf_pose()
        if tf_pose is not None:
            x, y, z, yaw = tf_pose
            self.tf_trajectory.append((elapsed, x, y, z, yaw))

        # Sample Gazebo pose via CLI
        gz_pose = self.get_gazebo_pose()
        if gz_pose is not None:
            x, y, z, yaw = gz_pose
            self.gz_trajectory.append((elapsed, x, y, z, yaw))

        # Log progress every 10 seconds
        if len(self.tf_trajectory) > 0 and len(self.tf_trajectory) % (int(self.sample_rate) * 10) == 0:
            self.get_logger().info(
                f'Recorded {len(self.tf_trajectory)} TF samples, '
                f'{len(self.gz_trajectory)} Gazebo samples'
            )

    def get_tf_pose(self):
        """Get current pose from TF2."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.parent_frame,
                self.child_frame,
                rclpy.time.Time()
            )
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            z = transform.transform.translation.z
            yaw = self.quaternion_to_yaw(transform.transform.rotation)
            return (x, y, z, yaw)
        except TransformException as e:
            if len(self.tf_trajectory) == 0:
                self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=5.0)
            return None

    def get_gazebo_pose(self):
        """
        Get current pose from Gazebo using ign model CLI command.
        
        Output format from 'ign model -m <name> -p':
            Model: [514]
              - Name: mobman
              - Pose [ XYZ (m) ] [ RPY (rad) ]:
                [1.000000 -3.000000 0.213400]
                [0.000000 0.000000 0.000000]
        """
        try:
            result = subprocess.run(
                ['ign', 'model', '-m', self.model_name, '-p'],
                capture_output=True, text=True, timeout=2
            )
            
            output = result.stdout
            if not output or 'error' in output.lower():
                if len(self.gz_trajectory) == 0:
                    self.get_logger().warn(f'ign model failed: {result.stderr}', throttle_duration_sec=5.0)
                return None
            
            # Parse position: look for [x y z] pattern
            bracket_pattern = r'\[([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\]'
            matches = re.findall(bracket_pattern, output)
            
            if len(matches) >= 1:
                x = float(matches[0][0])
                y = float(matches[0][1])
                z = float(matches[0][2])
                
                yaw = 0.0
                if len(matches) >= 2:
                    # Second match is RPY
                    yaw = float(matches[1][2])  # Yaw is the third element
                
                return (x, y, z, yaw)
            else:
                return None
                
        except subprocess.TimeoutExpired:
            if len(self.gz_trajectory) == 0:
                self.get_logger().warn('ign model command timed out', throttle_duration_sec=5.0)
            return None
        except Exception as e:
            if len(self.gz_trajectory) == 0:
                self.get_logger().warn(f'Gazebo pose exception: {e}', throttle_duration_sec=5.0)
            return None

    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle."""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def stop_recording(self):
        """Stop recording and generate plots."""
        self.get_logger().info('Duration reached. Stopping recording...')
        self.sample_timer.cancel()
        if hasattr(self, 'duration_timer'):
            self.duration_timer.cancel()
        self.generate_output()
        rclpy.shutdown()

    def generate_output(self):
        """Generate CSV files and plots."""
        self.get_logger().info(f'Final counts: {len(self.tf_trajectory)} TF, {len(self.gz_trajectory)} Gazebo')
        
        if len(self.tf_trajectory) == 0:
            self.get_logger().error('No TF data collected!')
            return
            
        if len(self.gz_trajectory) == 0:
            self.get_logger().error('No Gazebo data collected!')
            return

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')

        # Save to CSV
        self.save_csv(timestamp)

        # Generate plots
        self.generate_plots(timestamp)

        self.get_logger().info(f'Output saved to {self.output_dir}')

    def save_csv(self, timestamp):
        """Save trajectory data to CSV files."""
        # TF trajectory
        tf_file = os.path.join(self.output_dir, f'tf_trajectory_{timestamp}.csv')
        with open(tf_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'x', 'y', 'z', 'yaw'])
            writer.writerows(self.tf_trajectory)
        self.get_logger().info(f'Saved TF trajectory to {tf_file}')

        # Gazebo trajectory
        gz_file = os.path.join(self.output_dir, f'gz_trajectory_{timestamp}.csv')
        with open(gz_file, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time', 'x', 'y', 'z', 'yaw'])
            writer.writerows(self.gz_trajectory)
        self.get_logger().info(f'Saved Gazebo trajectory to {gz_file}')

    def generate_plots(self, timestamp):
        """Generate trajectory comparison and error plots as separate images."""
        # Convert to numpy arrays for easier manipulation
        tf_data = np.array(self.tf_trajectory)
        gz_data = np.array(self.gz_trajectory)

        # Ensure same length (truncate to shorter)
        min_len = min(len(tf_data), len(gz_data))
        if min_len == 0:
            self.get_logger().error('No overlapping data!')
            return
            
        tf_data = tf_data[:min_len]
        gz_data = gz_data[:min_len]

        # Extract columns
        time = tf_data[:, 0]
        tf_x, tf_y = tf_data[:, 1], tf_data[:, 2]
        gz_x, gz_y = gz_data[:, 1], gz_data[:, 2]

        # Calculate errors
        error_x = gz_x - tf_x
        error_y = gz_y - tf_y
        error_magnitude = np.sqrt(error_x**2 + error_y**2)

        # ===== Plot 1: Trajectory comparison (X-Y plane) =====
        fig1, ax1 = plt.subplots(figsize=(10, 8))
        ax1.plot(tf_x, tf_y, 'b-', label='TF (Predicted)', linewidth=1.5)
        ax1.plot(gz_x, gz_y, 'r-', label='Gazebo (Ground Truth)', linewidth=1.5)
        ax1.plot(tf_x[0], tf_y[0], 'go', markersize=10, label='Start')
        ax1.plot(tf_x[-1], tf_y[-1], 'bx', markersize=10, label='TF End')
        ax1.plot(gz_x[-1], gz_y[-1], 'rx', markersize=10, label='Gazebo End')
        ax1.set_xlabel('X (m)', fontsize=12)
        ax1.set_ylabel('Y (m)', fontsize=12)
        ax1.set_title('Trajectory Comparison (X-Y Plane)', fontsize=14)
        ax1.legend(fontsize=10)
        ax1.grid(True, which='both', linestyle='-', alpha=0.7)
        ax1.axis('equal')
        # Set 10cm (0.1m) grid ticks
        x_min, x_max = min(tf_x.min(), gz_x.min()), max(tf_x.max(), gz_x.max())
        y_min, y_max = min(tf_y.min(), gz_y.min()), max(tf_y.max(), gz_y.max())
        # Add some padding
        x_pad = (x_max - x_min) * 0.1 + 0.1
        y_pad = (y_max - y_min) * 0.1 + 0.1
        ax1.set_xlim(x_min - x_pad, x_max + x_pad)
        ax1.set_ylim(y_min - y_pad, y_max + y_pad)
        # 10cm ticks
        ax1.xaxis.set_major_locator(plt.MultipleLocator(0.1))
        ax1.yaxis.set_major_locator(plt.MultipleLocator(0.1))
        ax1.xaxis.set_minor_locator(plt.MultipleLocator(0.05))
        ax1.yaxis.set_minor_locator(plt.MultipleLocator(0.05))
        ax1.grid(True, which='minor', linestyle=':', alpha=0.4)
        plt.tight_layout()
        plot1_file = os.path.join(self.output_dir, f'trajectory_xy_{timestamp}.png')
        plt.savefig(plot1_file, dpi=150)
        self.get_logger().info(f'Saved trajectory plot to {plot1_file}')
        plt.close(fig1)

        # ===== Plot 2: Error magnitude over time =====
        fig2, ax2 = plt.subplots(figsize=(10, 6))
        ax2.plot(time, error_magnitude * 100, 'g-', linewidth=1.5)  # Convert to cm
        ax2.set_xlabel('Time (s)', fontsize=12)
        ax2.set_ylabel('Error Magnitude (cm)', fontsize=12)
        ax2.set_title('Position Error Over Time', fontsize=14)
        ax2.grid(True, which='both', linestyle='-', alpha=0.7)
        # 10cm (10 on Y-axis since it's in cm) major ticks
        ax2.yaxis.set_major_locator(plt.MultipleLocator(10))
        ax2.yaxis.set_minor_locator(plt.MultipleLocator(5))
        ax2.grid(True, which='minor', linestyle=':', alpha=0.4)
        plt.tight_layout()
        plot2_file = os.path.join(self.output_dir, f'error_magnitude_{timestamp}.png')
        plt.savefig(plot2_file, dpi=150)
        self.get_logger().info(f'Saved error magnitude plot to {plot2_file}')
        plt.close(fig2)

        # ===== Plot 3: X and Y errors over time =====
        fig3, ax3 = plt.subplots(figsize=(10, 6))
        ax3.plot(time, error_x * 100, 'b-', label='X Error', linewidth=1.5)
        ax3.plot(time, error_y * 100, 'r-', label='Y Error', linewidth=1.5)
        ax3.set_xlabel('Time (s)', fontsize=12)
        ax3.set_ylabel('Error (cm)', fontsize=12)
        ax3.set_title('X and Y Errors Over Time', fontsize=14)
        ax3.legend(fontsize=10)
        ax3.grid(True, which='both', linestyle='-', alpha=0.7)
        # 10cm major ticks
        ax3.yaxis.set_major_locator(plt.MultipleLocator(10))
        ax3.yaxis.set_minor_locator(plt.MultipleLocator(5))
        ax3.grid(True, which='minor', linestyle=':', alpha=0.4)
        plt.tight_layout()
        plot3_file = os.path.join(self.output_dir, f'error_xy_{timestamp}.png')
        plt.savefig(plot3_file, dpi=150)
        self.get_logger().info(f'Saved X/Y error plot to {plot3_file}')
        plt.close(fig3)

        # Print statistics to terminal
        self.get_logger().info('\n' + '='*50)
        self.get_logger().info('TRAJECTORY STATISTICS')
        self.get_logger().info('='*50)
        self.get_logger().info(f'Duration: {time[-1]:.1f} seconds')
        self.get_logger().info(f'Samples: {len(time)}')
        self.get_logger().info(f'\nFinal Position Error:')
        self.get_logger().info(f'  X: {error_x[-1]*100:.1f} cm')
        self.get_logger().info(f'  Y: {error_y[-1]*100:.1f} cm')
        self.get_logger().info(f'  Total: {error_magnitude[-1]*100:.1f} cm')
        self.get_logger().info(f'\nMaximum Errors:')
        self.get_logger().info(f'  X: {np.max(np.abs(error_x))*100:.1f} cm')
        self.get_logger().info(f'  Y: {np.max(np.abs(error_y))*100:.1f} cm')
        self.get_logger().info(f'  Total: {np.max(error_magnitude)*100:.1f} cm')
        self.get_logger().info(f'\nAverage Error: {np.mean(error_magnitude)*100:.1f} cm')
        self.get_logger().info(f'RMS Error: {np.sqrt(np.mean(error_magnitude**2))*100:.1f} cm')
        self.get_logger().info('='*50)


def main(args=None):
    rclpy.init(args=args)

    node = TrajectoryRecorder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received. Generating output...')
        node.generate_output()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
