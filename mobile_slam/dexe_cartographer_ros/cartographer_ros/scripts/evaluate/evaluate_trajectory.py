#!/usr/bin/env python3
"""
Trajectory Evaluation Script

This script evaluates trajectory data collected by collect_trajectory_data.py
It compares raw Cartographer localization with filtered robot_localization output.
Filtered trajectory is used as the reference.

Evaluation includes:
1. Trajectory visualization (both on same plot)
2. Position error analysis with heatmap (on filtered reference)
3. Orientation error analysis with heatmap (on filtered reference)
4. Error distribution analysis
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
from matplotlib import cm
from matplotlib.colors import Normalize
import pandas as pd
import argparse
from pathlib import Path
from datetime import datetime
import math


class TrajectoryEvaluator:
    def __init__(self, csv_file, output_dir=None):
        """
        Initialize evaluator with trajectory data
        
        Args:
            csv_file: Path to CSV file with trajectory data
            output_dir: Directory to save results (default: same as csv_file)
        """
        self.csv_file = Path(csv_file)
        if not self.csv_file.exists():
            raise FileNotFoundError(f"CSV file not found: {csv_file}")
        
        # Set output directory
        if output_dir is None:
            self.output_dir = self.csv_file.parent
        else:
            self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Load data
        print(f"Loading trajectory data from: {csv_file}")
        self.data = pd.read_csv(csv_file)
        print(f"Loaded {len(self.data)} trajectory points")
        
        # Extract poses
        self.extract_poses()
        
    def extract_poses(self):
        """Extract pose data from CSV"""
        # Raw Cartographer poses
        self.loc_x = self.data['loc_x'].values
        self.loc_y = self.data['loc_y'].values
        self.loc_yaw = self.data['loc_yaw'].values
        
        # Filtered poses
        self.filtered_x = self.data['filtered_x'].values
        self.filtered_y = self.data['filtered_y'].values
        self.filtered_yaw = self.data['filtered_yaw'].values
        
        # Timestamps
        self.timestamps = self.data['timestamp'].values
        
    def compute_position_errors(self):
        """Compute position errors between loc and filtered poses"""
        dx = self.loc_x - self.filtered_x
        dy = self.loc_y - self.filtered_y
        
        # Euclidean distance error
        position_errors = np.sqrt(dx**2 + dy**2)
        
        return position_errors, dx, dy
    
    def compute_orientation_errors(self):
        """Compute orientation errors (shortest angular distance)"""
        # Compute angular difference with wrapping
        diff = self.loc_yaw - self.filtered_yaw
        
        # Normalize to [-pi, pi]
        orientation_errors = np.arctan2(np.sin(diff), np.cos(diff))
        
        return np.abs(orientation_errors)
    
    def compute_ate(self, traj1_x, traj1_y, traj2_x, traj2_y):
        """
        Compute Absolute Trajectory Error (ATE)
        
        ATE measures the global consistency of the trajectory
        """
        errors = np.sqrt((traj1_x - traj2_x)**2 + (traj1_y - traj2_y)**2)
        
        ate_stats = {
            'mean': np.mean(errors),
            'median': np.median(errors),
            'std': np.std(errors),
            'min': np.min(errors),
            'max': np.max(errors),
            'rmse': np.sqrt(np.mean(errors**2))
        }
        
        return ate_stats, errors
    
    def compute_rpe(self, x, y, yaw, delta=1):
        """
        Compute Relative Pose Error (RPE)
        
        RPE measures the local accuracy of the trajectory
        
        Args:
            x, y, yaw: Trajectory coordinates
            delta: Step size for computing relative poses
        """
        n = len(x)
        if n < delta + 1:
            return None, None
        
        # Compute relative transformations
        trans_errors = []
        rot_errors = []
        
        for i in range(n - delta):
            # Translation error
            dx = x[i + delta] - x[i]
            dy = y[i + delta] - y[i]
            trans_error = np.sqrt(dx**2 + dy**2)
            trans_errors.append(trans_error)
            
            # Rotation error
            dyaw = yaw[i + delta] - yaw[i]
            # Normalize to [-pi, pi]
            dyaw = np.arctan2(np.sin(dyaw), np.cos(dyaw))
            rot_errors.append(np.abs(dyaw))
        
        trans_errors = np.array(trans_errors)
        rot_errors = np.array(rot_errors)
        
        rpe_stats = {
            'trans_mean': np.mean(trans_errors),
            'trans_median': np.median(trans_errors),
            'trans_std': np.std(trans_errors),
            'trans_rmse': np.sqrt(np.mean(trans_errors**2)),
            'rot_mean': np.mean(rot_errors),
            'rot_median': np.median(rot_errors),
            'rot_std': np.std(rot_errors),
            'rot_rmse': np.sqrt(np.mean(rot_errors**2))
        }
        
        return rpe_stats, (trans_errors, rot_errors)
    
    def plot_trajectories(self, save_path):
        """Plot both trajectories with orientation arrows on single plot"""
        fig, ax = plt.subplots(1, 1, figsize=(12, 10))
        
        # Plot filtered trajectory (reference) - solid line
        ax.plot(self.filtered_x, self.filtered_y, 'g-', alpha=0.6, linewidth=2.5, 
               label='Filtered (Reference)', zorder=2)
        
        # Plot raw Cartographer trajectory - dashed line
        ax.plot(self.loc_x, self.loc_y, 'b--', alpha=0.7, linewidth=2, 
               label='Raw Cartographer', zorder=1)
        
        # Plot start and end points
        ax.scatter(self.filtered_x[0], self.filtered_y[0], c='green', marker='o', s=200, 
                  edgecolors='black', linewidths=2, label='Start', zorder=5)
        ax.scatter(self.filtered_x[-1], self.filtered_y[-1], c='red', marker='s', s=200,
                  edgecolors='black', linewidths=2, label='End', zorder=5)
        
        # Draw orientation arrows for filtered trajectory (every nth point)
        n_points = len(self.filtered_x)
        arrow_step = max(1, n_points // 20)  # Show ~20 arrows
        
        # Calculate arrow length based on trajectory scale
        traj_range = max(np.ptp(self.filtered_x), np.ptp(self.filtered_y))
        arrow_length = traj_range * 0.02  # 2% of trajectory range
        
        for i in range(0, n_points, arrow_step):
            dx = arrow_length * np.cos(self.filtered_yaw[i])
            dy = arrow_length * np.sin(self.filtered_yaw[i])
            ax.arrow(self.filtered_x[i], self.filtered_y[i], dx, dy, 
                    head_width=arrow_length*0.3, 
                    head_length=arrow_length*0.3,
                    fc='green', ec='green', alpha=0.5, linewidth=0.8)
        
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title('Trajectory Comparison: Raw vs Filtered', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        ax.legend(loc='best', fontsize=11)
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved trajectory plot to: {save_path}")
        plt.close()
    
    def _plot_single_trajectory(self, ax, x, y, yaw, title, color):
        """Helper function to plot a single trajectory"""
        # Plot trajectory as connected line
        ax.plot(x, y, color=color, alpha=0.6, linewidth=1.5, label='Trajectory')
        
        # Plot start and end points
        ax.scatter(x[0], y[0], c='green', marker='o', s=200, 
                  edgecolors='black', linewidths=2, label='Start', zorder=5)
        ax.scatter(x[-1], y[-1], c='red', marker='s', s=200,
                  edgecolors='black', linewidths=2, label='End', zorder=5)
        
        # Draw orientation arrows (every nth point to avoid clutter)
        n_points = len(x)
        arrow_step = max(1, n_points // 30)  # Show ~30 arrows
        
        # Calculate arrow length based on trajectory scale
        traj_range = max(np.ptp(x), np.ptp(y))
        arrow_length = traj_range * 0.02  # 2% of trajectory range
        
        for i in range(0, n_points, arrow_step):
            dx = arrow_length * np.cos(yaw[i])
            dy = arrow_length * np.sin(yaw[i])
            ax.arrow(x[i], y[i], dx, dy, 
                    head_width=arrow_length*0.4, 
                    head_length=arrow_length*0.4,
                    fc='black', ec='black', alpha=0.6, linewidth=0.5)
        
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title(title, fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        ax.legend(loc='best', fontsize=10)
    
    def plot_error_heatmap(self, save_path):
        """Plot position and orientation errors as heatmap on filtered trajectory (reference)"""
        # Compute errors (using filtered as reference)
        pos_errors, dx, dy = self.compute_position_errors()
        ori_errors = self.compute_orientation_errors()
        
        fig, axes = plt.subplots(1, 2, figsize=(18, 8))
        
        # 1. Position error heatmap on filtered trajectory (reference)
        self._plot_error_on_trajectory(
            axes[0], self.filtered_x, self.filtered_y, self.filtered_yaw, pos_errors,
            'Position Error (Raw vs Filtered Reference)',
            'Position Error (m)', cm.jet
        )
        
        # 2. Orientation error heatmap on filtered trajectory (reference)
        ori_errors_deg = np.rad2deg(ori_errors)
        self._plot_error_on_trajectory(
            axes[1], self.filtered_x, self.filtered_y, self.filtered_yaw, ori_errors_deg,
            'Orientation Error (Raw vs Filtered Reference)',
            'Orientation Error (°)', cm.hot
        )
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved error heatmap to: {save_path}")
        plt.close()
    
    def _plot_error_on_trajectory(self, ax, x, y, yaw, errors, title, cbar_label, colormap):
        """Helper function to plot error as heatmap on trajectory"""
        # Create colormap normalization
        norm = Normalize(vmin=np.min(errors), vmax=np.max(errors))
        
        # Plot trajectory segments colored by error
        for i in range(len(x) - 1):
            color = colormap(norm(errors[i]))
            ax.plot([x[i], x[i+1]], [y[i], y[i+1]], 
                   color=color, linewidth=3, alpha=0.8)
        
        # Add colorbar
        sm = cm.ScalarMappable(cmap=colormap, norm=norm)
        sm.set_array([])
        cbar = plt.colorbar(sm, ax=ax)
        cbar.set_label(cbar_label, fontsize=11)
        
        # Plot start and end points
        ax.scatter(x[0], y[0], c='green', marker='o', s=200, 
                  edgecolors='black', linewidths=2, label='Start', zorder=5)
        ax.scatter(x[-1], y[-1], c='red', marker='s', s=200,
                  edgecolors='black', linewidths=2, label='End', zorder=5)
        
        # Add statistics text
        stats_text = f'Mean: {np.mean(errors):.4f}\n'
        stats_text += f'Median: {np.median(errors):.4f}\n'
        stats_text += f'Std: {np.std(errors):.4f}\n'
        stats_text += f'Max: {np.max(errors):.4f}'
        
        props = dict(boxstyle='round', facecolor='wheat', alpha=0.8)
        ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, 
               fontsize=10, verticalalignment='top', bbox=props,
               family='monospace')
        
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title(title, fontsize=12, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        ax.legend(loc='lower right', fontsize=9)
    
    def plot_error_distribution(self, save_path):
        """Plot error distributions and time series"""
        pos_errors, dx, dy = self.compute_position_errors()
        ori_errors = self.compute_orientation_errors()
        ori_errors_deg = np.rad2deg(ori_errors)
        
        # Compute time from start
        time_from_start = self.timestamps - self.timestamps[0]
        
        fig = plt.figure(figsize=(18, 12))
        gs = fig.add_gridspec(3, 2, hspace=0.3, wspace=0.3)
        
        # 1. Position error over time
        ax1 = fig.add_subplot(gs[0, :])
        ax1.plot(time_from_start, pos_errors, 'b-', linewidth=1, alpha=0.7)
        ax1.fill_between(time_from_start, 0, pos_errors, alpha=0.3)
        ax1.axhline(np.mean(pos_errors), color='r', linestyle='--', 
                   label=f'Mean: {np.mean(pos_errors):.4f}m')
        ax1.set_xlabel('Time (s)', fontsize=12)
        ax1.set_ylabel('Position Error (m)', fontsize=12)
        ax1.set_title('Position Error Over Time', fontsize=13, fontweight='bold')
        ax1.grid(True, alpha=0.3)
        ax1.legend()
        
        # 2. Position error histogram
        ax2 = fig.add_subplot(gs[1, 0])
        ax2.hist(pos_errors, bins=50, color='blue', alpha=0.7, edgecolor='black')
        ax2.axvline(np.mean(pos_errors), color='r', linestyle='--', linewidth=2,
                   label=f'Mean: {np.mean(pos_errors):.4f}m')
        ax2.axvline(np.median(pos_errors), color='g', linestyle='--', linewidth=2,
                   label=f'Median: {np.median(pos_errors):.4f}m')
        ax2.set_xlabel('Position Error (m)', fontsize=12)
        ax2.set_ylabel('Frequency', fontsize=12)
        ax2.set_title('Position Error Distribution', fontsize=13, fontweight='bold')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # 3. Orientation error histogram
        ax3 = fig.add_subplot(gs[1, 1])
        ax3.hist(ori_errors_deg, bins=50, color='orange', alpha=0.7, edgecolor='black')
        ax3.axvline(np.mean(ori_errors_deg), color='r', linestyle='--', linewidth=2,
                   label=f'Mean: {np.mean(ori_errors_deg):.4f}°')
        ax3.axvline(np.median(ori_errors_deg), color='g', linestyle='--', linewidth=2,
                   label=f'Median: {np.median(ori_errors_deg):.4f}°')
        ax3.set_xlabel('Orientation Error (°)', fontsize=12)
        ax3.set_ylabel('Frequency', fontsize=12)
        ax3.set_title('Orientation Error Distribution', fontsize=13, fontweight='bold')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        
        # 4. XY position errors
        ax4 = fig.add_subplot(gs[2, 0])
        ax4.plot(time_from_start, dx, 'r-', linewidth=1, alpha=0.7, label='X Error')
        ax4.plot(time_from_start, dy, 'b-', linewidth=1, alpha=0.7, label='Y Error')
        ax4.axhline(0, color='k', linestyle='-', linewidth=0.5)
        ax4.set_xlabel('Time (s)', fontsize=12)
        ax4.set_ylabel('Error (m)', fontsize=12)
        ax4.set_title('X and Y Position Errors Over Time', fontsize=13, fontweight='bold')
        ax4.legend()
        ax4.grid(True, alpha=0.3)
        
        # 5. Orientation error over time
        ax5 = fig.add_subplot(gs[2, 1])
        ax5.plot(time_from_start, ori_errors_deg, 'g-', linewidth=1, alpha=0.7)
        ax5.fill_between(time_from_start, 0, ori_errors_deg, alpha=0.3, color='green')
        ax5.axhline(np.mean(ori_errors_deg), color='r', linestyle='--',
                   label=f'Mean: {np.mean(ori_errors_deg):.4f}°')
        ax5.set_xlabel('Time (s)', fontsize=12)
        ax5.set_ylabel('Orientation Error (°)', fontsize=12)
        ax5.set_title('Orientation Error Over Time', fontsize=13, fontweight='bold')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
        
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved error distribution plot to: {save_path}")
        plt.close()
    
    def evaluate(self):
        """Run complete evaluation and save all results"""
        print("\n" + "="*70)
        print("TRAJECTORY EVALUATION REPORT")
        print("Reference: Filtered trajectory from robot_localization")
        print("="*70)
        
        # Compute errors (using filtered as reference)
        print("\n--- Position and Orientation Errors (vs Filtered Reference) ---")
        pos_errors, dx, dy = self.compute_position_errors()
        ori_errors = self.compute_orientation_errors()
        ori_errors_deg = np.rad2deg(ori_errors)
        
        print(f"\nPosition Error Statistics:")
        print(f"  Mean:   {np.mean(pos_errors):.6f} m")
        print(f"  Median: {np.median(pos_errors):.6f} m")
        print(f"  Std:    {np.std(pos_errors):.6f} m")
        print(f"  RMSE:   {np.sqrt(np.mean(pos_errors**2)):.6f} m")
        print(f"  Min:    {np.min(pos_errors):.6f} m")
        print(f"  Max:    {np.max(pos_errors):.6f} m")
        
        print(f"\nOrientation Error Statistics:")
        print(f"  Mean:   {np.mean(ori_errors_deg):.6f}°")
        print(f"  Median: {np.median(ori_errors_deg):.6f}°")
        print(f"  Std:    {np.std(ori_errors_deg):.6f}°")
        print(f"  RMSE:   {np.sqrt(np.mean(ori_errors_deg**2)):.6f}°")
        print(f"  Min:    {np.min(ori_errors_deg):.6f}°")
        print(f"  Max:    {np.max(ori_errors_deg):.6f}°")
        
        print("\n" + "="*70)
        
        # Save report to file (fixed filename)
        report_path = self.output_dir / 'trajectory_evaluation_report.txt'
        self.save_report(report_path, pos_errors, ori_errors_deg)
        
        # Generate plots (without displaying, fixed filenames)
        print("\nGenerating visualizations...")
        
        # 1. Trajectory comparison (single plot)
        traj_plot_path = self.output_dir / 'trajectory_comparison.png'
        self.plot_trajectories(traj_plot_path)
        
        # 2. Error heatmaps (2 plots on filtered reference)
        heatmap_path = self.output_dir / 'error_heatmap.png'
        self.plot_error_heatmap(heatmap_path)
        
        # 3. Error distributions
        dist_path = self.output_dir / 'error_distribution.png'
        self.plot_error_distribution(dist_path)
        
        print("\nEvaluation complete!")
        print(f"Results saved to: {self.output_dir}")
    
    def save_report(self, report_path, pos_errors, ori_errors_deg):
        """Save detailed evaluation report to file"""
        with open(report_path, 'w') as f:
            f.write("="*70 + "\n")
            f.write("TRAJECTORY EVALUATION REPORT\n")
            f.write(f"Generated at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Data file: {self.csv_file.name}\n")
            f.write(f"Number of poses: {len(self.data)}\n")
            f.write("Reference: Filtered trajectory from robot_localization\n")
            f.write("="*70 + "\n\n")
            
            f.write("--- Position Error (vs Filtered Reference) ---\n")
            f.write(f"  Mean:   {np.mean(pos_errors):.6f} m\n")
            f.write(f"  Median: {np.median(pos_errors):.6f} m\n")
            f.write(f"  Std:    {np.std(pos_errors):.6f} m\n")
            f.write(f"  RMSE:   {np.sqrt(np.mean(pos_errors**2)):.6f} m\n")
            f.write(f"  Min:    {np.min(pos_errors):.6f} m\n")
            f.write(f"  Max:    {np.max(pos_errors):.6f} m\n\n")
            
            f.write("--- Orientation Error (vs Filtered Reference) ---\n")
            f.write(f"  Mean:   {np.mean(ori_errors_deg):.6f}°\n")
            f.write(f"  Median: {np.median(ori_errors_deg):.6f}°\n")
            f.write(f"  Std:    {np.std(ori_errors_deg):.6f}°\n")
            f.write(f"  RMSE:   {np.sqrt(np.mean(ori_errors_deg**2)):.6f}°\n")
            f.write(f"  Min:    {np.min(ori_errors_deg):.6f}°\n")
            f.write(f"  Max:    {np.max(ori_errors_deg):.6f}°\n")
        
        print(f"Saved evaluation report to: {report_path}")


def main():
    parser = argparse.ArgumentParser(
        description='Evaluate trajectory data comparing raw and filtered localization'
    )
    
    # Default path to trajectory data
    default_csv = Path.home() / '.dexe' / 'localization' / 'localization_assessment' / 'trajectory_data' / 'trajectory_data.csv'
    
    parser.add_argument(
        'csv_file',
        type=str,
        nargs='?',  # Make it optional
        default=str(default_csv),
        help=f'Path to CSV file with trajectory data (default: {default_csv})'
    )
    parser.add_argument(
        '--output-dir',
        type=str,
        default=None,
        help='Output directory for results (default: same as input file)'
    )
    
    args = parser.parse_args()
    
    # Run evaluation
    evaluator = TrajectoryEvaluator(args.csv_file, args.output_dir)
    evaluator.evaluate()


if __name__ == '__main__':
    main()
