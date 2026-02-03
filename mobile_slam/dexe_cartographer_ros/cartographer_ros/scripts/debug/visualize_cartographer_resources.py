#!/usr/bin/env python3
"""
Cartographer Resource Usage Visualization

This script visualizes CPU and memory usage data collected by monitor_cartographer_resources.sh
It generates time-series plots showing resource consumption over time.
"""

import numpy as np
import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend
import matplotlib.pyplot as plt
import pandas as pd
import argparse
from pathlib import Path
from datetime import datetime


class ResourceVisualizer:
    def __init__(self, csv_file, output_dir=None):
        """
        Initialize visualizer with resource usage data
        
        Args:
            csv_file: Path to CSV file with resource data
            output_dir: Directory to save plots (default: same as csv_file)
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
        print(f"Loading resource data from: {csv_file}")
        self.data = pd.read_csv(csv_file)
        print(f"Loaded {len(self.data)} data points")
        
        # Convert timestamp to relative time (seconds from start)
        self.data['time_elapsed'] = self.data['timestamp'] - self.data['timestamp'].iloc[0]
        
    def compute_statistics(self):
        """Compute statistics for resource usage"""
        cpu_stats = {
            'mean': self.data['cpu_percent'].mean(),
            'median': self.data['cpu_percent'].median(),
            'std': self.data['cpu_percent'].std(),
            'min': self.data['cpu_percent'].min(),
            'max': self.data['cpu_percent'].max()
        }
        
        mem_stats = {
            'mean': self.data['mem_percent'].mean(),
            'median': self.data['mem_percent'].median(),
            'std': self.data['mem_percent'].std(),
            'min': self.data['mem_percent'].min(),
            'max': self.data['mem_percent'].max()
        }
        
        rss_stats = {
            'mean': self.data['rss_mb'].mean(),
            'median': self.data['rss_mb'].median(),
            'std': self.data['rss_mb'].std(),
            'min': self.data['rss_mb'].min(),
            'max': self.data['rss_mb'].max()
        }
        
        return cpu_stats, mem_stats, rss_stats
    
    def plot_resource_usage(self, save_path):
        """Plot CPU and memory usage over time"""
        fig, axes = plt.subplots(3, 1, figsize=(14, 12))
        
        time_elapsed = self.data['time_elapsed'].values
        cpu_percent = self.data['cpu_percent'].values
        mem_percent = self.data['mem_percent'].values
        rss_mb = self.data['rss_mb'].values
        
        # 1. CPU Usage
        ax1 = axes[0]
        ax1.plot(time_elapsed, cpu_percent, 'b-', linewidth=1.5, alpha=0.7, label='CPU Usage')
        ax1.fill_between(time_elapsed, 0, cpu_percent, alpha=0.3, color='blue')
        ax1.axhline(np.mean(cpu_percent), color='r', linestyle='--', linewidth=2,
                   label=f'Mean: {np.mean(cpu_percent):.2f}%')
        ax1.set_xlabel('Time (s)', fontsize=12)
        ax1.set_ylabel('CPU Usage (%)', fontsize=12)
        ax1.set_title('Cartographer Node - CPU Usage Over Time', fontsize=14, fontweight='bold')
        ax1.grid(True, alpha=0.3)
        ax1.legend(loc='upper right', fontsize=10)
        
        # Add statistics box
        stats_text = f'Mean: {np.mean(cpu_percent):.2f}%\n'
        stats_text += f'Median: {np.median(cpu_percent):.2f}%\n'
        stats_text += f'Std: {np.std(cpu_percent):.2f}%\n'
        stats_text += f'Min: {np.min(cpu_percent):.2f}%\n'
        stats_text += f'Max: {np.max(cpu_percent):.2f}%'
        props = dict(boxstyle='round', facecolor='wheat', alpha=0.8)
        ax1.text(0.02, 0.98, stats_text, transform=ax1.transAxes,
                fontsize=9, verticalalignment='top', bbox=props, family='monospace')
        
        # 2. Memory Usage (Percentage)
        ax2 = axes[1]
        ax2.plot(time_elapsed, mem_percent, 'g-', linewidth=1.5, alpha=0.7, label='Memory Usage')
        ax2.fill_between(time_elapsed, 0, mem_percent, alpha=0.3, color='green')
        ax2.axhline(np.mean(mem_percent), color='r', linestyle='--', linewidth=2,
                   label=f'Mean: {np.mean(mem_percent):.2f}%')
        ax2.set_xlabel('Time (s)', fontsize=12)
        ax2.set_ylabel('Memory Usage (%)', fontsize=12)
        ax2.set_title('Cartographer Node - Memory Usage Over Time', fontsize=14, fontweight='bold')
        ax2.grid(True, alpha=0.3)
        ax2.legend(loc='upper right', fontsize=10)
        
        # Add statistics box
        stats_text = f'Mean: {np.mean(mem_percent):.2f}%\n'
        stats_text += f'Median: {np.median(mem_percent):.2f}%\n'
        stats_text += f'Std: {np.std(mem_percent):.2f}%\n'
        stats_text += f'Min: {np.min(mem_percent):.2f}%\n'
        stats_text += f'Max: {np.max(mem_percent):.2f}%'
        ax2.text(0.02, 0.98, stats_text, transform=ax2.transAxes,
                fontsize=9, verticalalignment='top', bbox=props, family='monospace')
        
        # 3. RSS Memory (MB)
        ax3 = axes[2]
        ax3.plot(time_elapsed, rss_mb, 'orange', linewidth=1.5, alpha=0.7, label='RSS Memory')
        ax3.fill_between(time_elapsed, 0, rss_mb, alpha=0.3, color='orange')
        ax3.axhline(np.mean(rss_mb), color='r', linestyle='--', linewidth=2,
                   label=f'Mean: {np.mean(rss_mb):.2f} MB')
        ax3.set_xlabel('Time (s)', fontsize=12)
        ax3.set_ylabel('RSS Memory (MB)', fontsize=12)
        ax3.set_title('Cartographer Node - RSS Memory Over Time', fontsize=14, fontweight='bold')
        ax3.grid(True, alpha=0.3)
        ax3.legend(loc='upper right', fontsize=10)
        
        # Add statistics box
        stats_text = f'Mean: {np.mean(rss_mb):.2f} MB\n'
        stats_text += f'Median: {np.median(rss_mb):.2f} MB\n'
        stats_text += f'Std: {np.std(rss_mb):.2f} MB\n'
        stats_text += f'Min: {np.min(rss_mb):.2f} MB\n'
        stats_text += f'Max: {np.max(rss_mb):.2f} MB'
        ax3.text(0.02, 0.98, stats_text, transform=ax3.transAxes,
                fontsize=9, verticalalignment='top', bbox=props, family='monospace')
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved resource usage plot to: {save_path}")
        plt.close()
    
    def plot_combined_view(self, save_path):
        """Plot CPU and memory on same chart with dual y-axis"""
        fig, ax1 = plt.subplots(figsize=(14, 8))
        
        time_elapsed = self.data['time_elapsed'].values
        cpu_percent = self.data['cpu_percent'].values
        rss_mb = self.data['rss_mb'].values
        
        # Plot CPU on left y-axis
        color = 'tab:blue'
        ax1.set_xlabel('Time (s)', fontsize=12)
        ax1.set_ylabel('CPU Usage (%)', color=color, fontsize=12)
        line1 = ax1.plot(time_elapsed, cpu_percent, color=color, linewidth=2, 
                        alpha=0.7, label='CPU Usage (%)')
        ax1.tick_params(axis='y', labelcolor=color)
        ax1.grid(True, alpha=0.3)
        
        # Create second y-axis for memory
        ax2 = ax1.twinx()
        color = 'tab:orange'
        ax2.set_ylabel('RSS Memory (MB)', color=color, fontsize=12)
        line2 = ax2.plot(time_elapsed, rss_mb, color=color, linewidth=2, 
                        alpha=0.7, label='RSS Memory (MB)')
        ax2.tick_params(axis='y', labelcolor=color)
        
        # Add title
        plt.title('Cartographer Node - CPU and Memory Usage Over Time', 
                 fontsize=14, fontweight='bold', pad=20)
        
        # Combine legends
        lines = line1 + line2
        labels = [l.get_label() for l in lines]
        ax1.legend(lines, labels, loc='upper left', fontsize=11)
        
        # Add statistics text
        stats_text = 'CPU Statistics:\n'
        stats_text += f'  Mean: {np.mean(cpu_percent):.2f}%\n'
        stats_text += f'  Max: {np.max(cpu_percent):.2f}%\n\n'
        stats_text += 'Memory Statistics:\n'
        stats_text += f'  Mean: {np.mean(rss_mb):.2f} MB\n'
        stats_text += f'  Max: {np.max(rss_mb):.2f} MB'
        
        props = dict(boxstyle='round', facecolor='wheat', alpha=0.8)
        ax1.text(0.02, 0.98, stats_text, transform=ax1.transAxes,
                fontsize=10, verticalalignment='top', bbox=props, family='monospace')
        
        plt.tight_layout()
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved combined resource plot to: {save_path}")
        plt.close()
    
    def visualize(self):
        """Generate all visualizations and save report"""
        print("\n" + "="*70)
        print("CARTOGRAPHER NODE RESOURCE USAGE REPORT")
        print("="*70)
        
        # Compute statistics
        cpu_stats, mem_stats, rss_stats = self.compute_statistics()
        
        # Print statistics
        print("\n--- CPU Usage Statistics ---")
        print(f"  Mean:   {cpu_stats['mean']:.2f}%")
        print(f"  Median: {cpu_stats['median']:.2f}%")
        print(f"  Std:    {cpu_stats['std']:.2f}%")
        print(f"  Min:    {cpu_stats['min']:.2f}%")
        print(f"  Max:    {cpu_stats['max']:.2f}%")
        
        print("\n--- Memory Usage Statistics (%) ---")
        print(f"  Mean:   {mem_stats['mean']:.2f}%")
        print(f"  Median: {mem_stats['median']:.2f}%")
        print(f"  Std:    {mem_stats['std']:.2f}%")
        print(f"  Min:    {mem_stats['min']:.2f}%")
        print(f"  Max:    {mem_stats['max']:.2f}%")
        
        print("\n--- RSS Memory Statistics (MB) ---")
        print(f"  Mean:   {rss_stats['mean']:.2f} MB")
        print(f"  Median: {rss_stats['median']:.2f} MB")
        print(f"  Std:    {rss_stats['std']:.2f} MB")
        print(f"  Min:    {rss_stats['min']:.2f} MB")
        print(f"  Max:    {rss_stats['max']:.2f} MB")
        
        print("\n" + "="*70)
        
        # Save report
        report_path = self.output_dir / 'resource_usage_report.txt'
        self.save_report(report_path, cpu_stats, mem_stats, rss_stats)
        
        # Generate plots
        print("\nGenerating visualizations...")
        
        # 1. Detailed resource usage (3 subplots)
        plot_path = self.output_dir / 'resource_usage.png'
        self.plot_resource_usage(plot_path)
        
        # 2. Combined view (dual y-axis)
        combined_path = self.output_dir / 'resource_usage_combined.png'
        self.plot_combined_view(combined_path)
        
        print("\nVisualization complete!")
        print(f"Results saved to: {self.output_dir}")
    
    def save_report(self, report_path, cpu_stats, mem_stats, rss_stats):
        """Save detailed report to file"""
        with open(report_path, 'w') as f:
            f.write("="*70 + "\n")
            f.write("CARTOGRAPHER NODE RESOURCE USAGE REPORT\n")
            f.write(f"Generated at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Data file: {self.csv_file.name}\n")
            f.write(f"Number of samples: {len(self.data)}\n")
            f.write(f"Monitoring duration: {self.data['time_elapsed'].iloc[-1]:.1f} seconds\n")
            f.write("="*70 + "\n\n")
            
            f.write("--- CPU Usage Statistics ---\n")
            f.write(f"  Mean:   {cpu_stats['mean']:.2f}%\n")
            f.write(f"  Median: {cpu_stats['median']:.2f}%\n")
            f.write(f"  Std:    {cpu_stats['std']:.2f}%\n")
            f.write(f"  Min:    {cpu_stats['min']:.2f}%\n")
            f.write(f"  Max:    {cpu_stats['max']:.2f}%\n\n")
            
            f.write("--- Memory Usage Statistics (%) ---\n")
            f.write(f"  Mean:   {mem_stats['mean']:.2f}%\n")
            f.write(f"  Median: {mem_stats['median']:.2f}%\n")
            f.write(f"  Std:    {mem_stats['std']:.2f}%\n")
            f.write(f"  Min:    {mem_stats['min']:.2f}%\n")
            f.write(f"  Max:    {mem_stats['max']:.2f}%\n\n")
            
            f.write("--- RSS Memory Statistics (MB) ---\n")
            f.write(f"  Mean:   {rss_stats['mean']:.2f} MB\n")
            f.write(f"  Median: {rss_stats['median']:.2f} MB\n")
            f.write(f"  Std:    {rss_stats['std']:.2f} MB\n")
            f.write(f"  Min:    {rss_stats['min']:.2f} MB\n")
            f.write(f"  Max:    {rss_stats['max']:.2f} MB\n")
        
        print(f"Saved report to: {report_path}")


def main():
    parser = argparse.ArgumentParser(
        description='Visualize locaization_node resource usage'
    )
    
    # Default path to resource data
    default_dir = Path.home() / '.dexe' / 'localization' / 'localization_assessment'
    default_csv = default_dir / 'localization_resources.csv'
    
    parser.add_argument(
        'csv_file',
        type=str,
        nargs='?',
        help='Path to CSV file with resource data'
    )
    parser.add_argument(
        '--output-dir',
        type=str,
        default=None,
        help='Output directory for plots (default: same as input file)'
    )
    parser.add_argument(
        '--list',
        action='store_true',
        help=f'List available resource data files in {default_dir}'
    )
    
    args = parser.parse_args()
    
    # List available files if requested
    if args.list:
        if default_dir.exists():
            csv_files = sorted(default_dir.glob('localization_resources*.csv'))
            if csv_files:
                print(f"\nAvailable resource data files in {default_dir}:")
                for i, f in enumerate(csv_files, 1):
                    print(f"  {i}. {f.name}")
            else:
                print(f"\nNo resource data files found in {default_dir}")
        else:
            print(f"\nDirectory does not exist: {default_dir}")
        return
    
    # Check if csv_file is provided
    if args.csv_file is None:
        # Try to use the default file
        if default_csv.exists():
            csv_file = default_csv
            print(f"No file specified. Using default: {csv_file}")
        else:
            print(f"Error: Default resource data file not found: {default_csv}")
            print("Please specify a CSV file or run monitor_cartographer_resources.sh first")
            return
    else:
        csv_file = args.csv_file
    
    # Run visualization
    visualizer = ResourceVisualizer(csv_file, args.output_dir)
    visualizer.visualize()


if __name__ == '__main__':
    main()
