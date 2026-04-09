#!/usr/bin/env python3
"""
Lightning 3D SLAM / Localization Resource Usage Visualization

This script visualizes CPU and memory usage data collected by monitor_lightning_resources.sh
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
            'mean':   self.data['cpu_percent'].mean(),
            'median': self.data['cpu_percent'].median(),
            'std':    self.data['cpu_percent'].std(),
            'min':    self.data['cpu_percent'].min(),
            'max':    self.data['cpu_percent'].max(),
        }
        mem_stats = {
            'mean':   self.data['mem_percent'].mean(),
            'median': self.data['mem_percent'].median(),
            'std':    self.data['mem_percent'].std(),
            'min':    self.data['mem_percent'].min(),
            'max':    self.data['mem_percent'].max(),
        }
        rss_stats = {
            'mean':   self.data['rss_mb'].mean(),
            'median': self.data['rss_mb'].median(),
            'std':    self.data['rss_mb'].std(),
            'min':    self.data['rss_mb'].min(),
            'max':    self.data['rss_mb'].max(),
        }
        return cpu_stats, mem_stats, rss_stats

    def plot_resource_usage(self, save_path):
        """Plot CPU and memory usage over time (3 subplots)"""
        fig, axes = plt.subplots(3, 1, figsize=(14, 12))

        t   = self.data['time_elapsed'].values
        cpu = self.data['cpu_percent'].values
        mem = self.data['mem_percent'].values
        rss = self.data['rss_mb'].values

        props = dict(boxstyle='round', facecolor='wheat', alpha=0.8)

        # --- 1. CPU Usage ---
        ax = axes[0]
        ax.plot(t, cpu, 'b-', linewidth=1.5, alpha=0.7, label='CPU Usage')
        ax.fill_between(t, 0, cpu, alpha=0.3, color='blue')
        ax.axhline(np.mean(cpu), color='r', linestyle='--', linewidth=2,
                   label=f'Mean: {np.mean(cpu):.2f}%')
        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('CPU Usage (%)', fontsize=12)
        ax.set_title('Lightning Node - CPU Usage Over Time', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right', fontsize=10)
        stats = (f'Mean:   {np.mean(cpu):.2f}%\n'
                 f'Median: {np.median(cpu):.2f}%\n'
                 f'Std:    {np.std(cpu):.2f}%\n'
                 f'Min:    {np.min(cpu):.2f}%\n'
                 f'Max:    {np.max(cpu):.2f}%')
        ax.text(0.02, 0.98, stats, transform=ax.transAxes,
                fontsize=9, verticalalignment='top', bbox=props, family='monospace')

        # --- 2. Memory Usage (%) ---
        ax = axes[1]
        ax.plot(t, mem, 'g-', linewidth=1.5, alpha=0.7, label='Memory Usage')
        ax.fill_between(t, 0, mem, alpha=0.3, color='green')
        ax.axhline(np.mean(mem), color='r', linestyle='--', linewidth=2,
                   label=f'Mean: {np.mean(mem):.2f}%')
        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('Memory Usage (%)', fontsize=12)
        ax.set_title('Lightning Node - Memory Usage Over Time', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right', fontsize=10)
        stats = (f'Mean:   {np.mean(mem):.2f}%\n'
                 f'Median: {np.median(mem):.2f}%\n'
                 f'Std:    {np.std(mem):.2f}%\n'
                 f'Min:    {np.min(mem):.2f}%\n'
                 f'Max:    {np.max(mem):.2f}%')
        ax.text(0.02, 0.98, stats, transform=ax.transAxes,
                fontsize=9, verticalalignment='top', bbox=props, family='monospace')

        # --- 3. RSS Memory (MB) ---
        ax = axes[2]
        ax.plot(t, rss, color='orange', linewidth=1.5, alpha=0.7, label='RSS Memory')
        ax.fill_between(t, 0, rss, alpha=0.3, color='orange')
        ax.axhline(np.mean(rss), color='r', linestyle='--', linewidth=2,
                   label=f'Mean: {np.mean(rss):.2f} MB')
        ax.set_xlabel('Time (s)', fontsize=12)
        ax.set_ylabel('RSS Memory (MB)', fontsize=12)
        ax.set_title('Lightning Node - RSS Memory Over Time', fontsize=14, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='upper right', fontsize=10)
        stats = (f'Mean:   {np.mean(rss):.2f} MB\n'
                 f'Median: {np.median(rss):.2f} MB\n'
                 f'Std:    {np.std(rss):.2f} MB\n'
                 f'Min:    {np.min(rss):.2f} MB\n'
                 f'Max:    {np.max(rss):.2f} MB')
        ax.text(0.02, 0.98, stats, transform=ax.transAxes,
                fontsize=9, verticalalignment='top', bbox=props, family='monospace')

        plt.tight_layout()
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved resource usage plot to: {save_path}")
        plt.close()

    def plot_combined_view(self, save_path):
        """Plot CPU and RSS memory on the same chart with dual y-axis"""
        fig, ax1 = plt.subplots(figsize=(14, 8))

        t   = self.data['time_elapsed'].values
        cpu = self.data['cpu_percent'].values
        rss = self.data['rss_mb'].values

        # Left y-axis: CPU
        color = 'tab:blue'
        ax1.set_xlabel('Time (s)', fontsize=12)
        ax1.set_ylabel('CPU Usage (%)', color=color, fontsize=12)
        line1 = ax1.plot(t, cpu, color=color, linewidth=2, alpha=0.7, label='CPU Usage (%)')
        ax1.tick_params(axis='y', labelcolor=color)
        ax1.grid(True, alpha=0.3)

        # Right y-axis: RSS memory
        ax2 = ax1.twinx()
        color = 'tab:orange'
        ax2.set_ylabel('RSS Memory (MB)', color=color, fontsize=12)
        line2 = ax2.plot(t, rss, color=color, linewidth=2, alpha=0.7, label='RSS Memory (MB)')
        ax2.tick_params(axis='y', labelcolor=color)

        plt.title('Lightning Node - CPU and Memory Usage Over Time',
                  fontsize=14, fontweight='bold', pad=20)

        lines  = line1 + line2
        labels = [l.get_label() for l in lines]
        ax1.legend(lines, labels, loc='upper left', fontsize=11)

        stats = ('CPU Statistics:\n'
                 f'  Mean: {np.mean(cpu):.2f}%\n'
                 f'  Max:  {np.max(cpu):.2f}%\n\n'
                 'Memory Statistics:\n'
                 f'  Mean: {np.mean(rss):.2f} MB\n'
                 f'  Max:  {np.max(rss):.2f} MB')
        props = dict(boxstyle='round', facecolor='wheat', alpha=0.8)
        ax1.text(0.02, 0.98, stats, transform=ax1.transAxes,
                 fontsize=10, verticalalignment='top', bbox=props, family='monospace')

        plt.tight_layout()
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f"Saved combined resource plot to: {save_path}")
        plt.close()

    def save_report(self, report_path, cpu_stats, mem_stats, rss_stats):
        """Save detailed text report to file"""
        with open(report_path, 'w') as f:
            f.write('=' * 70 + '\n')
            f.write('LIGHTNING NODE RESOURCE USAGE REPORT\n')
            f.write(f"Generated at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"Data file: {self.csv_file.name}\n")
            f.write(f"Number of samples: {len(self.data)}\n")
            f.write(f"Monitoring duration: {self.data['time_elapsed'].iloc[-1]:.1f} seconds\n")
            f.write('=' * 70 + '\n\n')

            f.write('--- CPU Usage Statistics ---\n')
            for k, v in cpu_stats.items():
                f.write(f'  {k.capitalize():8s}: {v:.2f}%\n')
            f.write('\n')

            f.write('--- Memory Usage Statistics (%) ---\n')
            for k, v in mem_stats.items():
                f.write(f'  {k.capitalize():8s}: {v:.2f}%\n')
            f.write('\n')

            f.write('--- RSS Memory Statistics (MB) ---\n')
            for k, v in rss_stats.items():
                f.write(f'  {k.capitalize():8s}: {v:.2f} MB\n')

        print(f"Saved report to: {report_path}")

    def visualize(self):
        """Generate all visualizations and save report"""
        print('\n' + '=' * 70)
        print('LIGHTNING NODE RESOURCE USAGE REPORT')
        print('=' * 70)

        cpu_stats, mem_stats, rss_stats = self.compute_statistics()

        print('\n--- CPU Usage Statistics ---')
        for k, v in cpu_stats.items():
            print(f'  {k.capitalize():8s}: {v:.2f}%')

        print('\n--- Memory Usage Statistics (%) ---')
        for k, v in mem_stats.items():
            print(f'  {k.capitalize():8s}: {v:.2f}%')

        print('\n--- RSS Memory Statistics (MB) ---')
        for k, v in rss_stats.items():
            print(f'  {k.capitalize():8s}: {v:.2f} MB')

        print('\n' + '=' * 70)

        # Save text report
        report_path = self.output_dir / 'lightning_resource_report.txt'
        self.save_report(report_path, cpu_stats, mem_stats, rss_stats)

        # Generate plots
        print('\nGenerating visualizations...')
        self.plot_resource_usage(self.output_dir / 'lightning_resource_usage.png')
        self.plot_combined_view(self.output_dir / 'lightning_resource_combined.png')

        print('\nVisualization complete!')
        print(f'Results saved to: {self.output_dir}')


def main():
    parser = argparse.ArgumentParser(
        description='Visualize Lightning 3D SLAM / localization node resource usage'
    )

    default_dir = Path.home() / '.dexe' / 'lightning' / 'assessment'
    default_csv = default_dir / 'lightning_resources.csv'

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
            csv_files = sorted(default_dir.glob('lightning_resources*.csv'))
            if csv_files:
                print(f'\nAvailable resource data files in {default_dir}:')
                for i, f in enumerate(csv_files, 1):
                    print(f'  {i}. {f.name}')
            else:
                print(f'\nNo resource data files found in {default_dir}')
        else:
            print(f'\nDirectory does not exist: {default_dir}')
        return

    # Resolve CSV path
    if args.csv_file is None:
        if default_csv.exists():
            csv_file = default_csv
            print(f'No file specified. Using default: {csv_file}')
        else:
            print(f'Error: Default resource data file not found: {default_csv}')
            print('Please specify a CSV file or run monitor_lightning_resources.sh first')
            return
    else:
        csv_file = args.csv_file

    visualizer = ResourceVisualizer(csv_file, args.output_dir)
    visualizer.visualize()


if __name__ == '__main__':
    main()
