#!/usr/bin/env python3
"""
Trajectory Data Collection Script

This script collects synchronized pose data from:
1. Raw Cartographer localization pose (/cartographer/loc_pose)
2. Filtered pose from robot_localization (/odometry/filtered)

Collection is triggered by loc_pose messages. When loc_pose is received,
the latest filtered odometry is paired with it.

The data is saved to a timestamped CSV file for later evaluation.
Collection continues until:
- User presses Ctrl+C
- Timeout is reached (default: 300s)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
import csv
import math
import signal
import sys
from pathlib import Path
from datetime import datetime


class TrajectoryDataCollector(Node):
    def __init__(self):
        super().__init__('trajectory_data_collector')
        
        # Create output directory
        self.output_dir = Path.home() / '.dexe' / 'localization' / 'localization_assessment' / 'trajectory_data'
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f'Output directory: {self.output_dir}')
        
        # Use fixed filename (no timestamp)
        self.csv_filename = self.output_dir / 'trajectory_data.csv'
        
        # Declare parameters
        self.declare_parameter('loc_pose_topic', '/loc_pose')
        self.declare_parameter('filtered_odom_topic', '/odometry/filtered')
        self.declare_parameter('timeout_sec', 300.0)  # Default: 5 minutes
        
        # Get parameters
        self.loc_pose_topic = self.get_parameter('loc_pose_topic').value
        self.filtered_odom_topic = self.get_parameter('filtered_odom_topic').value
        self.timeout_sec = self.get_parameter('timeout_sec').value
        
        # Data storage
        self.data_count = 0
        self.csv_file = None
        self.csv_writer = None
        
        # Track collection start for timeout only
        self.collection_start_time = None
        
        # Cache for latest filtered odometry
        self.latest_filtered_odom = None
        
        # Open CSV file for writing
        self.initialize_csv()
        
        # Subscribe to filtered odometry (to cache latest message)
        self.filtered_odom_sub = self.create_subscription(
            Odometry,
            self.filtered_odom_topic,
            self.filtered_odom_callback,
            10
        )
        
        # Subscribe to loc_pose (triggers data recording)
        self.loc_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.loc_pose_topic,
            self.loc_pose_callback,
            10
        )
        
        # Create timer to check timeout
        self.timeout_timer = self.create_timer(1.0, self.check_timeout)
        
        # Setup signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        
        self.get_logger().info(f'Subscribing to:')
        self.get_logger().info(f'  - Raw pose: {self.loc_pose_topic} (trigger)')
        self.get_logger().info(f'  - Filtered pose: {self.filtered_odom_topic} (cache)')
        self.get_logger().info(f'Data collection rate matches loc_pose frequency')
        self.get_logger().info(f'Timeout: {self.timeout_sec}s')
        self.get_logger().info(f'Press Ctrl+C to stop collection and save data')
        self.get_logger().info(f'Saving to: {self.csv_filename}')
        
    def initialize_csv(self):
        """Initialize CSV file with headers"""
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp', 
            'loc_x', 'loc_y', 'loc_z',
            'loc_qx', 'loc_qy', 'loc_qz', 'loc_qw',
            'loc_yaw',
            'filtered_x', 'filtered_y', 'filtered_z',
            'filtered_qx', 'filtered_qy', 'filtered_qz', 'filtered_qw',
            'filtered_yaw'
        ])
        self.get_logger().info('CSV file initialized')
        
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def filtered_odom_callback(self, msg):
        """Cache the latest filtered odometry message"""
        self.latest_filtered_odom = msg
    
    def loc_pose_callback(self, loc_pose_msg):
        """Record data when loc_pose is received"""
        # Only record if we have filtered odometry data
        if self.latest_filtered_odom is None:
            self.get_logger().warn('No filtered odometry received yet, skipping...', 
                                  throttle_duration_sec=5.0)
            return
        
        # Track collection start time for timeout
        if self.collection_start_time is None:
            self.collection_start_time = self.get_clock().now()
            
        # Extract raw cartographer pose
        loc_pose = loc_pose_msg.pose.pose
        loc_yaw = self.quaternion_to_yaw(loc_pose.orientation)
        
        # Extract filtered pose
        filtered_pose = self.latest_filtered_odom.pose.pose
        filtered_yaw = self.quaternion_to_yaw(filtered_pose.orientation)
        
        # Get timestamp
        timestamp = loc_pose_msg.header.stamp.sec + loc_pose_msg.header.stamp.nanosec * 1e-9
        
        # Write to CSV
        self.csv_writer.writerow([
            timestamp,
            loc_pose.position.x, loc_pose.position.y, loc_pose.position.z,
            loc_pose.orientation.x, loc_pose.orientation.y, 
            loc_pose.orientation.z, loc_pose.orientation.w,
            loc_yaw,
            filtered_pose.position.x, filtered_pose.position.y, filtered_pose.position.z,
            filtered_pose.orientation.x, filtered_pose.orientation.y,
            filtered_pose.orientation.z, filtered_pose.orientation.w,
            filtered_yaw
        ])
        
        # Increment count after successful recording
        self.data_count += 1
        
        # Log progress every 10 samples
        if self.data_count % 10 == 0:
            self.get_logger().info(f'Collected {self.data_count} samples')
    
    def check_timeout(self):
        """Check if timeout has been reached"""
        if self.collection_start_time is not None:
            elapsed = (self.get_clock().now() - self.collection_start_time).nanoseconds / 1e9
            if elapsed >= self.timeout_sec:
                self.get_logger().warn(f'Timeout reached ({self.timeout_sec}s). Stopping collection...')
                self.shutdown()
    
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C gracefully"""
        self.get_logger().info('\nCtrl+C detected. Saving data and shutting down...')
        self.shutdown()
        
    def shutdown(self):
        """Graceful shutdown"""
        if self.csv_file:
            self.csv_file.close()
            self.get_logger().info(f'Data saved to: {self.csv_filename}')
            self.get_logger().info(f'Total samples collected: {self.data_count}')
        
        rclpy.shutdown()
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    
    collector = TrajectoryDataCollector()
    
    try:
        rclpy.spin(collector)
    except KeyboardInterrupt:
        collector.get_logger().info('Interrupted by user')
    finally:
        if not collector.csv_file.closed:
            collector.shutdown()


if __name__ == '__main__':
    main()
