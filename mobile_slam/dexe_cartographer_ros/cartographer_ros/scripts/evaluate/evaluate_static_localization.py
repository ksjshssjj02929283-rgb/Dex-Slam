#!/usr/bin/env python3
"""
Static Localization Evaluation Script (Unified: Collection + Evaluation)

This script:
1. Collects static pose data from:
   - Raw Cartographer localization pose (/cartographer/loc_pose)
   - Filtered pose from robot_localization (/odometry/filtered)
2. Automatically evaluates the collected data
3. Generates visualizations and reports

Simply run this script to collect data and get evaluation results.
Data and results are saved without timestamps (fixed filenames).
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
from matplotlib.patches import Ellipse
import pandas as pd
import csv
import math
import signal
import sys
from pathlib import Path
from datetime import datetime


class StaticLocalizationEvaluator(Node):
    def __init__(self):
        super().__init__('static_localization_evaluator')
        
        # Create output directory
        self.output_dir = Path.home() / '.dexe' / 'localization' / 'localization_assessment' / 'static_data'
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.get_logger().info(f'Output directory: {self.output_dir}')
        
        # Use fixed filename (no timestamp)
        self.csv_filename = self.output_dir / 'static_data.csv'
        
        # Declare parameters
        self.declare_parameter('loc_pose_topic', '/loc_pose')
        self.declare_parameter('filtered_odom_topic', '/robot_localization/odometry/filtered')
        self.declare_parameter('buffer_size', 100)
        
        # Get parameters
        self.loc_pose_topic = self.get_parameter('loc_pose_topic').value
        self.filtered_odom_topic = self.get_parameter('filtered_odom_topic').value
        self.buffer_size = self.get_parameter('buffer_size').value
        
        # Data storage
        self.data_count = 0
        self.csv_file = None
        self.csv_writer = None
        
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
        
        # Setup signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        
        self.get_logger().info(f'Subscribing to:')
        self.get_logger().info(f'  - Raw pose: {self.loc_pose_topic} (trigger)')
        self.get_logger().info(f'  - Filtered pose: {self.filtered_odom_topic} (cache)')
        self.get_logger().info(f'Collecting up to {self.buffer_size} samples')
        self.get_logger().info(f'Data recording triggered by each loc_pose message')
        self.get_logger().info(f'Collection rate = loc_pose publish rate')
        self.get_logger().info(f'Will automatically evaluate and exit after {self.buffer_size} samples')
        self.get_logger().info(f'Or press Ctrl+C to stop early and evaluate with current data')
        self.get_logger().info(f'Saving to: {self.csv_filename}')
        
    def initialize_csv(self):
        """Initialize CSV file with headers"""
        self.csv_file = open(self.csv_filename, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'timestamp', 
            'loc_x', 'loc_y', 'loc_yaw',
            'filtered_x', 'filtered_y', 'filtered_yaw'
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
            loc_pose.position.x, loc_pose.position.y, loc_yaw,
            filtered_pose.position.x, filtered_pose.position.y, filtered_yaw
        ])
        
        # Increment count after successful recording
        self.data_count += 1
        
        # Log progress every 10 samples
        if self.data_count % 10 == 0:
            self.get_logger().info(
                f'Collected {self.data_count}/{self.buffer_size} samples'
            )
        
        # Check if we've collected enough samples (after recording)
        if self.data_count >= self.buffer_size:
            self.get_logger().info(f'Collected {self.buffer_size} samples. Starting evaluation...')
            # Unsubscribe to stop receiving more messages
            self.destroy_subscription(self.loc_pose_sub)
            self.destroy_subscription(self.filtered_odom_sub)
            # Run evaluation and exit
            self.shutdown_and_evaluate()
            # Force exit
            raise SystemExit(0)
    
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C gracefully"""
        self.get_logger().info('\nCtrl+C detected. Starting evaluation...')
        self.shutdown_and_evaluate()
        
    def shutdown_and_evaluate(self):
        """Close CSV and run evaluation"""
        # Close CSV file if still open
        if self.csv_file and not self.csv_file.closed:
            self.csv_file.close()
            self.get_logger().info(f'Data saved to: {self.csv_filename}')
            self.get_logger().info(f'Total samples collected: {self.data_count}')
        
        # Run evaluation
        self.run_evaluation()
        
        # Shutdown ROS and exit
        try:
            self.destroy_node()
        except:
            pass
        
        try:
            rclpy.shutdown()
        except:
            pass
    
    def run_evaluation(self):
        """Evaluate the collected static data"""
        if self.data_count == 0:
            print("No data collected. Exiting without evaluation.")
            sys.exit(0)
        
        print("\n" + "="*70)
        print("STATIC LOCALIZATION EVALUATION")
        print("="*70)
        
        # Load data
        print(f"\nLoading data from: {self.csv_filename}")
        data = pd.read_csv(self.csv_filename)
        print(f"Loaded {len(data)} static pose samples")
        
        # Extract poses
        loc_poses = data[['loc_x', 'loc_y', 'loc_yaw']].values
        filtered_poses = data[['filtered_x', 'filtered_y', 'filtered_yaw']].values
        
        # Compute metrics
        loc_metrics = self.compute_metrics(loc_poses, "Raw Cartographer Localization")
        filtered_metrics = self.compute_metrics(filtered_poses, "Robot Localization Filtered")
        
        # Print metrics
        self.print_metrics(loc_metrics)
        self.print_metrics(filtered_metrics)
        
        # Save report
        report_path = self.output_dir / 'static_evaluation_report.txt'
        self.save_report(report_path, loc_metrics, filtered_metrics, len(data))
        
        # Generate visualization
        print("\nGenerating visualization...")
        plot_path = self.output_dir / 'static_localization.png'
        self.create_visualization(loc_poses, filtered_poses, loc_metrics, filtered_metrics, plot_path)
        
        print("\nEvaluation complete!")
        print(f"Results saved to: {self.output_dir}")
        print(f"  - Data: static_data.csv")
        print(f"  - Report: static_evaluation_report.txt")
        print(f"  - Visualization: static_localization.png")
        print("\nExiting automatically...")
        
        # Force exit after evaluation
        sys.exit(0)
    
    def compute_metrics(self, poses, name):
        """Compute evaluation metrics for a set of poses"""
        poses_array = np.array(poses)
        x = poses_array[:, 0]
        y = poses_array[:, 1]
        theta = poses_array[:, 2]
        
        # Unwrap theta to handle angle wrapping
        theta_unwrapped = np.unwrap(theta)
        
        # Mean position
        mean_x = np.mean(x)
        mean_y = np.mean(y)
        mean_theta = np.mean(theta_unwrapped)
        
        # Standard deviation
        std_x = np.std(x)
        std_y = np.std(y)
        std_theta = np.std(theta_unwrapped)
        
        # RMSE (relative to mean)
        rmse_x = np.sqrt(np.mean((x - mean_x)**2))
        rmse_y = np.sqrt(np.mean((y - mean_y)**2))
        rmse_theta = np.sqrt(np.mean((theta_unwrapped - mean_theta)**2))
        
        # Maximum deviation
        max_dev_x = np.max(np.abs(x - mean_x))
        max_dev_y = np.max(np.abs(y - mean_y))
        max_dev_theta = np.max(np.abs(theta_unwrapped - mean_theta))
        
        # Jitter analysis - count changes exceeding threshold
        position_threshold = 0.01  # meters
        orientation_threshold = 0.05  # radians
        
        if len(poses) > 1:
            dx = np.diff(x)
            dy = np.diff(y)
            dtheta = np.diff(theta_unwrapped)
            
            position_jitters = np.sum(np.sqrt(dx**2 + dy**2) > position_threshold)
            orientation_jitters = np.sum(np.abs(dtheta) > orientation_threshold)
            
            jitter_rate_position = position_jitters / (len(poses) - 1)
            jitter_rate_orientation = orientation_jitters / (len(poses) - 1)
        else:
            jitter_rate_position = 0
            jitter_rate_orientation = 0
        
        # 3-sigma outlier detection
        outliers_x = np.sum(np.abs(x - mean_x) > 3 * std_x)
        outliers_y = np.sum(np.abs(y - mean_y) > 3 * std_y)
        outliers_theta = np.sum(np.abs(theta_unwrapped - mean_theta) > 3 * std_theta)
        
        outlier_ratio_x = outliers_x / len(poses)
        outlier_ratio_y = outliers_y / len(poses)
        outlier_ratio_theta = outliers_theta / len(poses)
        
        metrics = {
            'name': name,
            'mean': [mean_x, mean_y, mean_theta],
            'std': [std_x, std_y, std_theta],
            'rmse': [rmse_x, rmse_y, rmse_theta],
            'max_dev': [max_dev_x, max_dev_y, max_dev_theta],
            'jitter_rate': [jitter_rate_position, jitter_rate_orientation],
            'outlier_ratio': [outlier_ratio_x, outlier_ratio_y, outlier_ratio_theta]
        }
        
        return metrics
    
    def print_metrics(self, metrics):
        """Print evaluation metrics"""
        print(f"\n{'='*60}")
        print(f"Evaluation Results for {metrics['name']}")
        print(f"{'='*60}")
        
        print(f"\n1. Position Fluctuation Indicators:")
        print(f"   Standard Deviation (STD):")
        print(f"     σ_x     = {metrics['std'][0]:.6f} m")
        print(f"     σ_y     = {metrics['std'][1]:.6f} m")
        print(f"     σ_θ     = {metrics['std'][2]:.6f} rad ({np.rad2deg(metrics['std'][2]):.4f}°)")
        
        print(f"\n   Root Mean Square Error (RMSE):")
        print(f"     RMSE_x  = {metrics['rmse'][0]:.6f} m")
        print(f"     RMSE_y  = {metrics['rmse'][1]:.6f} m")
        print(f"     RMSE_θ  = {metrics['rmse'][2]:.6f} rad ({np.rad2deg(metrics['rmse'][2]):.4f}°)")
        
        print(f"\n   Maximum Deviation:")
        print(f"     Max_x   = {metrics['max_dev'][0]:.6f} m")
        print(f"     Max_y   = {metrics['max_dev'][1]:.6f} m")
        print(f"     Max_θ   = {metrics['max_dev'][2]:.6f} rad ({np.rad2deg(metrics['max_dev'][2]):.4f}°)")
        
        print(f"\n2. Stability Scoring:")
        print(f"   Jitter Frequency (change rate exceeding threshold):")
        print(f"     Position:    {metrics['jitter_rate'][0]:.2%}")
        print(f"     Orientation: {metrics['jitter_rate'][1]:.2%}")
        
        print(f"\n   3-Sigma Outliers:")
        print(f"     X:     {metrics['outlier_ratio'][0]:.2%}")
        print(f"     Y:     {metrics['outlier_ratio'][1]:.2%}")
        print(f"     Theta: {metrics['outlier_ratio'][2]:.2%}")
        
        print(f"{'='*60}\n")
    
    def visualize_poses(self, poses, metrics, ax, title, color):
        """Visualize poses with very small uniform arrows and metrics"""
        poses_array = np.array(poses)
        x = poses_array[:, 0]
        y = poses_array[:, 1]
        theta = poses_array[:, 2]
        
        mean_x, mean_y, mean_theta = metrics['mean']
        std_x, std_y, std_theta = metrics['std']
        
        # Plot poses as scatter points
        ax.scatter(x, y, c=color, alpha=0.6, s=30, label='Poses')
        
        # Plot mean position
        ax.scatter(mean_x, mean_y, c='red', marker='x', s=200, linewidths=3, 
                   label=f'Mean ({mean_x:.3f}, {mean_y:.3f})', zorder=5)
        
        # Draw orientation arrows - very small and uniform
        arrow_step = max(1, len(poses) // 20)  # Show ~20 arrows
        
        # Use fixed small arrow length (3mm = 0.003m) for uniform appearance
        arrow_length = 0.002  # Fixed 2mm length
        arrow_width = 0.00005  # Fixed 0.2mm width
        
        # for i in range(0, len(poses), arrow_step):
        #     dx = arrow_length * np.cos(theta[i])
        #     dy = arrow_length * np.sin(theta[i])
        #     # Very small uniform head for needle-like appearance
        #     ax.arrow(x[i], y[i], dx, dy, 
        #             head_width=arrow_width, 
        #             head_length=arrow_width,
        #             fc='black', ec='black', alpha=0.7, linewidth=0.01)
        
        # Plot 1-sigma and 3-sigma confidence ellipses
        for n_std, alpha, label_text in [(1, 0.3, '1σ'), (3, 0.15, '3σ')]:
            ellipse = Ellipse((mean_x, mean_y), 
                             width=n_std * std_x * 2, 
                             height=n_std * std_y * 2,
                             alpha=alpha, color=color, 
                             label=f'{label_text} ellipse')
            ax.add_patch(ellipse)
        
        # Add text annotations with metrics
        textstr = '\n'.join([
            f'STD: x={std_x:.4f}m, y={std_y:.4f}m',
            f'     θ={np.rad2deg(std_theta):.3f}°',
            f'RMSE: x={metrics["rmse"][0]:.4f}m, y={metrics["rmse"][1]:.4f}m',
            f'Max Dev: {metrics["max_dev"][0]:.4f}m, {metrics["max_dev"][1]:.4f}m',
            f'Jitter: pos={metrics["jitter_rate"][0]:.1%}, θ={metrics["jitter_rate"][1]:.1%}',
            f'3σ Outliers: {max(metrics["outlier_ratio"]):.1%}'
        ])
        
        props = dict(boxstyle='round', facecolor='wheat', alpha=0.8)
        ax.text(0.05, 0.95, textstr, transform=ax.transAxes, fontsize=9,
                verticalalignment='top', bbox=props, family='monospace')
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_title(title)
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        ax.legend(loc='upper right', fontsize=8)
    
    def create_visualization(self, loc_poses, filtered_poses, loc_metrics, filtered_metrics, save_path):
        """Create and save visualization without displaying"""
        fig, axes = plt.subplots(1, 2, figsize=(16, 7))
        
        self.visualize_poses(loc_poses, loc_metrics, axes[0], 
                           'Raw Cartographer Localization', 'blue')
        self.visualize_poses(filtered_poses, filtered_metrics, axes[1], 
                           'Robot Localization Filtered', 'green')
        
        # Make both plots use the same axis limits for better comparison
        # Get the combined data range from both datasets
        all_x = np.concatenate([loc_poses[:, 0], filtered_poses[:, 0]])
        all_y = np.concatenate([loc_poses[:, 1], filtered_poses[:, 1]])
        
        x_min, x_max = np.min(all_x), np.max(all_x)
        y_min, y_max = np.min(all_y), np.max(all_y)
        
        # Add some padding (10% of range)
        x_range = x_max - x_min
        y_range = y_max - y_min
        padding = 0.1
        
        x_min -= x_range * padding
        x_max += x_range * padding
        y_min -= y_range * padding
        y_max += y_range * padding
        
        # Apply same limits to both axes
        axes[0].set_xlim(x_min, x_max)
        axes[0].set_ylim(y_min, y_max)
        axes[1].set_xlim(x_min, x_max)
        axes[1].set_ylim(y_min, y_max)
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=150)
        print(f"Saved visualization to: {save_path}")
        plt.close()
    
    def save_report(self, report_path, loc_metrics, filtered_metrics, num_samples):
        """Save detailed evaluation report to file"""
        with open(report_path, 'w') as f:
            f.write("="*70 + "\n")
            f.write("STATIC LOCALIZATION EVALUATION REPORT\n")
            f.write(f"Generated at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Number of samples: {num_samples}\n")
            f.write("="*70 + "\n\n")
            
            for metrics in [loc_metrics, filtered_metrics]:
                f.write("="*60 + "\n")
                f.write(f"Evaluation Results for {metrics['name']}\n")
                f.write("="*60 + "\n\n")
                
                f.write("1. Position Fluctuation Indicators:\n")
                f.write("   Standard Deviation (STD):\n")
                f.write(f"     σ_x     = {metrics['std'][0]:.6f} m\n")
                f.write(f"     σ_y     = {metrics['std'][1]:.6f} m\n")
                f.write(f"     σ_θ     = {metrics['std'][2]:.6f} rad ({np.rad2deg(metrics['std'][2]):.4f}°)\n\n")
                
                f.write("   Root Mean Square Error (RMSE):\n")
                f.write(f"     RMSE_x  = {metrics['rmse'][0]:.6f} m\n")
                f.write(f"     RMSE_y  = {metrics['rmse'][1]:.6f} m\n")
                f.write(f"     RMSE_θ  = {metrics['rmse'][2]:.6f} rad ({np.rad2deg(metrics['rmse'][2]):.4f}°)\n\n")
                
                f.write("   Maximum Deviation:\n")
                f.write(f"     Max_x   = {metrics['max_dev'][0]:.6f} m\n")
                f.write(f"     Max_y   = {metrics['max_dev'][1]:.6f} m\n")
                f.write(f"     Max_θ   = {metrics['max_dev'][2]:.6f} rad ({np.rad2deg(metrics['max_dev'][2]):.4f}°)\n\n")
                
                f.write("2. Stability Scoring:\n")
                f.write("   Jitter Frequency (change rate exceeding threshold):\n")
                f.write(f"     Position:    {metrics['jitter_rate'][0]:.2%}\n")
                f.write(f"     Orientation: {metrics['jitter_rate'][1]:.2%}\n\n")
                
                f.write("   3-Sigma Outliers:\n")
                f.write(f"     X:     {metrics['outlier_ratio'][0]:.2%}\n")
                f.write(f"     Y:     {metrics['outlier_ratio'][1]:.2%}\n")
                f.write(f"     Theta: {metrics['outlier_ratio'][2]:.2%}\n\n")
        
        print(f"Saved evaluation report to: {report_path}")


def main(args=None):
    rclpy.init(args=args)
    
    evaluator = StaticLocalizationEvaluator()
    
    try:
        rclpy.spin(evaluator)
    except KeyboardInterrupt:
        evaluator.get_logger().info('Interrupted by user')
        evaluator.shutdown_and_evaluate()
        sys.exit(0)
    except SystemExit as e:
        # Normal exit after evaluation - propagate the exit code
        sys.exit(e.code if hasattr(e, 'code') else 0)
    finally:
        # Only evaluate if we haven't already
        if evaluator.csv_file and not evaluator.csv_file.closed:
            evaluator.shutdown_and_evaluate()
        
        # Ensure clean shutdown
        if rclpy.ok():
            try:
                evaluator.destroy_node()
            except:
                pass
            try:
                rclpy.shutdown()
            except:
                pass


if __name__ == '__main__':
    main()
