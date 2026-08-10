#!/usr/bin/env python3
"""
PGO调试日志可视化工具
读取 pgo_debug_*.csv 文件并绘制各阶段位姿对比图

用法:
    python3 visualize_pgo_debug.py [csv文件路径]
    
默认读取最新的 /tmp/pgo_debug/pgo_debug_*.csv 文件
"""

import sys
import os
import glob
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import argparse

# 配置中文字体
plt.rcParams['font.sans-serif'] = [
    'WenQuanYi Zen Hei',
    'WenQuanYi Micro Hei',
    'Noto Sans CJK SC',
    'AR PL UKai CN',
    'SimHei',
    'DejaVu Sans'
]
plt.rcParams['axes.unicode_minus'] = False

def find_latest_csv():
    """查找最新的CSV文件"""
    csv_files = glob.glob('/tmp/pgo_debug/pgo_debug_*.csv')
    if not csv_files:
        print("错误: 未找到 /tmp/pgo_debug/pgo_debug_*.csv 文件")
        print("请确保PGO调试日志已启用并运行过定位程序")
        sys.exit(1)
    
    # 按修改时间排序，返回最新的
    csv_files.sort(key=os.path.getmtime, reverse=True)
    return csv_files[0]

def load_csv(csv_path):
    """加载CSV文件"""
    print(f"加载文件: {csv_path}")
    
    # 读取CSV，跳过注释行
    df = pd.read_csv(csv_path, comment='#')
    
    # 转换时间戳为可读格式
    df['time_relative'] = df['timestamp'] - df['timestamp'].iloc[0]
    
    print(f"共加载 {len(df)} 条记录")
    print(f"时间跨度: {df['time_relative'].iloc[0]:.2f}s - {df['time_relative'].iloc[-1]:.2f}s")
    
    return df

def plot_trajectory_comparison(df, output_pdf):
    """绘制轨迹对比图"""
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('PGO融合过程 - 位姿对比', fontsize=16, fontweight='bold')
    
    # 提取各阶段数据
    stages = {
        'LIDARLOC_INPUT': ('LidarLoc输入', 'red', 'o'),
        'OPTIMIZED': ('PGO优化后', 'green', 's'),
        'EXTRAPOLATED': ('外推后', 'blue', '^'),
        'SMOOTHED_OUTPUT': ('平滑输出', 'purple', 'D'),
    }
    
    # XY轨迹对比
    ax = axes[0, 0]
    for stage, (label, color, marker) in stages.items():
        stage_df = df[df['stage'] == stage]
        if len(stage_df) > 0:
            ax.plot(stage_df['tx'], stage_df['ty'], 
                   label=label, alpha=0.6, linewidth=1, marker=marker, markersize=2)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('XY平面轨迹对比')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    # Z轴对比
    ax = axes[0, 1]
    for stage, (label, color, marker) in stages.items():
        stage_df = df[df['stage'] == stage]
        if len(stage_df) > 0:
            ax.plot(stage_df['time_relative'], stage_df['tz'], 
                   label=label, alpha=0.7, linewidth=1, marker=marker, markersize=2)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('Z (m)')
    ax.set_title('Z轴高度对比')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 输入数据源对比
    ax = axes[1, 0]
    input_stages = {
        'DR_INPUT': ('DR输入', 'orange', 'o'),
        'LO_INPUT': ('LO输入', 'cyan', 's'),
        'LIDARLOC_INPUT': ('LidarLoc输入', 'red', '^'),
    }
    for stage, (label, color, marker) in input_stages.items():
        stage_df = df[df['stage'] == stage]
        if len(stage_df) > 0:
            ax.plot(stage_df['time_relative'], stage_df['tx'], 
                   label=f'{label} (X)', alpha=0.7, linewidth=1, linestyle='-', marker=marker, markersize=2)
            ax.plot(stage_df['time_relative'], stage_df['ty'], 
                   label=f'{label} (Y)', alpha=0.7, linewidth=1, linestyle='--', marker=marker, markersize=2)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('平移 (m)')
    ax.set_title('输入数据源XY对比')
    ax.legend(ncol=2, fontsize=8)
    ax.grid(True, alpha=0.3)
    
    # 平滑因子变化
    ax = axes[1, 1]
    smoothed_df = df[df['stage'] == 'SMOOTHED_OUTPUT']
    if len(smoothed_df) > 0 and not smoothed_df['smooth_factor'].isna().all():
        ax.plot(smoothed_df['time_relative'], smoothed_df['smooth_factor'], 
               'purple', linewidth=2, marker='D', markersize=3)
        ax.set_xlabel('时间 (s)')
        ax.set_ylabel('平滑因子 α')
        ax.set_title('自适应平滑因子变化')
        ax.grid(True, alpha=0.3)
        ax.set_ylim(-0.05, 1.05)
    
    plt.tight_layout()
    output_pdf.savefig(fig)
    plt.close()
    
    print("✓ 轨迹对比图已生成")

def plot_position_error(df, output_pdf):
    """绘制位置误差图"""
    fig, axes = plt.subplots(3, 1, figsize=(14, 10))
    fig.suptitle('PGO融合过程 - 位置误差分析', fontsize=16, fontweight='bold')
    
    # 以平滑输出为基准，计算其他阶段与它的误差
    smoothed_df = df[df['stage'] == 'SMOOTHED_OUTPUT'].copy()
    smoothed_df = smoothed_df.set_index('timestamp')
    
    stages_to_compare = ['LIDARLOC_INPUT', 'OPTIMIZED', 'EXTRAPOLATED']
    labels = ['LidarLoc输入', 'PGO优化后', '外推后']
    colors = ['red', 'green', 'blue']
    
    for idx, (stage, label, color) in enumerate(zip(stages_to_compare, labels, colors)):
        stage_df = df[df['stage'] == stage].copy()
        if len(stage_df) == 0:
            continue
            
        errors = []
        times = []
        
        for _, row in stage_df.iterrows():
            # 找到时间最接近的平滑输出
            ts = row['timestamp']
            if ts in smoothed_df.index:
                smoothed = smoothed_df.loc[ts]
                error = np.sqrt((row['tx'] - smoothed['tx'])**2 + 
                              (row['ty'] - smoothed['ty'])**2 + 
                              (row['tz'] - smoothed['tz'])**2)
                errors.append(error)
                times.append(row['time_relative'])
        
        if errors:
            axes[0].plot(times, errors, label=label, alpha=0.7, linewidth=1, marker='o', markersize=2)
    
    axes[0].set_xlabel('时间 (s)')
    axes[0].set_ylabel('位置误差 (m)')
    axes[0].set_title('各阶段与平滑输出的位置误差')
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)
    
    # LidarLoc置信度
    lidarloc_df = df[df['stage'] == 'LIDARLOC_INPUT']
    if len(lidarloc_df) > 0 and not lidarloc_df['confidence'].isna().all():
        axes[1].plot(lidarloc_df['time_relative'], lidarloc_df['confidence'], 
                    'red', linewidth=1, marker='o', markersize=2)
        axes[1].axhline(y=0.5, color='orange', linestyle='--', label='阈值 (0.5)')
        axes[1].axhline(y=0.6, color='green', linestyle='--', label='满权重阈值 (0.6)')
        axes[1].set_xlabel('时间 (s)')
        axes[1].set_ylabel('置信度')
        axes[1].set_title('LidarLoc定位置信度')
        axes[1].legend()
        axes[1].grid(True, alpha=0.3)
        axes[1].set_ylim(-0.05, 1.05)
    
    # 外推时间差
    extrap_df = df[df['stage'] == 'EXTRAPOLATED']
    if len(extrap_df) > 0 and not extrap_df['dt_extrap'].isna().all():
        axes[2].plot(extrap_df['time_relative'], extrap_df['dt_extrap'], 
                    'blue', linewidth=1, marker='^', markersize=2)
        axes[2].set_xlabel('时间 (s)')
        axes[2].set_ylabel('外推时间差 (s)')
        axes[2].set_title('外推时间差分布')
        axes[2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_pdf.savefig(fig)
    plt.close()
    
    print("✓ 误差分析图已生成")

def plot_statistics(df, output_pdf):
    """绘制统计信息"""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))
    fig.suptitle('PGO融合过程 - 统计信息', fontsize=16, fontweight='bold')
    
    # 各阶段数据量
    stage_counts = df['stage'].value_counts()
    ax = axes[0, 0]
    stage_counts.plot(kind='bar', ax=ax, color=['orange', 'cyan', 'red', 'green', 'blue', 'purple'])
    ax.set_xlabel('融合阶段')
    ax.set_ylabel('数据点数量')
    ax.set_title('各阶段数据量统计')
    ax.tick_params(axis='x', rotation=45)
    
    # 时间间隔分布
    ax = axes[0, 1]
    for stage in ['DR_INPUT', 'LO_INPUT', 'LIDARLOC_INPUT', 'SMOOTHED_OUTPUT']:
        stage_df = df[df['stage'] == stage].sort_values('timestamp')
        if len(stage_df) > 1:
            dt = stage_df['timestamp'].diff().dropna()
            ax.hist(dt, bins=50, alpha=0.5, label=stage, density=True)
    ax.set_xlabel('时间间隔 (s)')
    ax.set_ylabel('密度')
    ax.set_title('各阶段时间间隔分布')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # 位置范围
    ax = axes[1, 0]
    stages_pos = ['LIDARLOC_INPUT', 'OPTIMIZED', 'EXTRAPOLATED', 'SMOOTHED_OUTPUT']
    labels_pos = ['LidarLoc', '优化后', '外推后', '平滑输出']
    box_data = []
    for stage in stages_pos:
        stage_df = df[df['stage'] == stage]
        if len(stage_df) > 0:
            pos_norm = np.sqrt(stage_df['tx']**2 + stage_df['ty']**2 + stage_df['tz']**2)
            box_data.append(pos_norm)
    
    if box_data:
        ax.boxplot(box_data, labels=labels_pos)
        ax.set_ylabel('位置模长 (m)')
        ax.set_title('各阶段位置分布')
        ax.grid(True, alpha=0.3)
    
    # 置信度分布
    ax = axes[1, 1]
    lidarloc_df = df[df['stage'] == 'LIDARLOC_INPUT']
    if len(lidarloc_df) > 0 and not lidarloc_df['confidence'].isna().all():
        ax.hist(lidarloc_df['confidence'], bins=30, alpha=0.7, color='red', edgecolor='black')
        ax.axvline(x=0.5, color='orange', linestyle='--', label='阈值 (0.5)')
        ax.axvline(x=0.6, color='green', linestyle='--', label='满权重 (0.6)')
        ax.set_xlabel('置信度')
        ax.set_ylabel('频次')
        ax.set_title('LidarLoc置信度分布')
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_pdf.savefig(fig)
    plt.close()
    
    print("✓ 统计信息图已生成")

def print_summary(df):
    """打印统计摘要"""
    print("\n" + "="*60)
    print("PGO融合过程统计摘要")
    print("="*60)
    
    print(f"\n总记录数: {len(df)}")
    print(f"时间跨度: {df['time_relative'].iloc[-1] - df['time_relative'].iloc[0]:.2f} 秒")
    
    print("\n各阶段数据量:")
    for stage in df['stage'].unique():
        count = len(df[df['stage'] == stage])
        print(f"  {stage:20s}: {count:6d} 条")
    
    # LidarLoc置信度统计
    lidarloc_df = df[df['stage'] == 'LIDARLOC_INPUT']
    if len(lidarloc_df) > 0 and not lidarloc_df['confidence'].isna().all():
        print(f"\nLidarLoc置信度统计:")
        print(f"  平均值: {lidarloc_df['confidence'].mean():.4f}")
        print(f"  最小值: {lidarloc_df['confidence'].min():.4f}")
        print(f"  最大值: {lidarloc_df['confidence'].max():.4f}")
        low_conf = len(lidarloc_df[lidarloc_df['confidence'] < 0.5])
        print(f"  低分(<0.5)次数: {low_conf} ({low_conf/len(lidarloc_df)*100:.1f}%)")
    
    # 平滑因子统计
    smoothed_df = df[df['stage'] == 'SMOOTHED_OUTPUT']
    if len(smoothed_df) > 0 and not smoothed_df['smooth_factor'].isna().all():
        print(f"\n平滑因子统计:")
        print(f"  平均值: {smoothed_df['smooth_factor'].mean():.4f}")
        print(f"  最小值: {smoothed_df['smooth_factor'].min():.4f}")
        print(f"  最大值: {smoothed_df['smooth_factor'].max():.4f}")
    
    # 外推时间差统计
    extrap_df = df[df['stage'] == 'EXTRAPOLATED']
    if len(extrap_df) > 0 and not extrap_df['dt_extrap'].isna().all():
        print(f"\n外推时间差统计:")
        print(f"  平均值: {extrap_df['dt_extrap'].mean():.4f} 秒")
        print(f"  最大值: {extrap_df['dt_extrap'].max():.4f} 秒")
        print(f"  最小值: {extrap_df['dt_extrap'].min():.4f} 秒")
    
    print("="*60)

def main():
    parser = argparse.ArgumentParser(description='PGO调试日志可视化工具')
    parser.add_argument('csv_file', nargs='?', help='CSV文件路径 (可选，默认使用最新的)')
    parser.add_argument('-o', '--output', help='输出PDF文件路径 (默认: pgo_debug_analysis.pdf)')
    args = parser.parse_args()
    
    # 确定CSV文件路径
    if args.csv_file:
        csv_path = args.csv_file
    else:
        csv_path = find_latest_csv()
    
    if not os.path.exists(csv_path):
        print(f"错误: 文件不存在 {csv_path}")
        sys.exit(1)
    
    # 加载数据
    df = load_csv(csv_path)
    
    # 确定输出路径
    output_pdf_path = args.output or 'pgo_debug_analysis.pdf'
    
    # 生成PDF报告
    with PdfPages(output_pdf_path) as pdf:
        print(f"\n生成可视化报告: {output_pdf_path}")
        plot_trajectory_comparison(df, pdf)
        plot_position_error(df, pdf)
        plot_statistics(df, pdf)
    
    # 打印统计摘要
    print_summary(df)
    
    print(f"\n✓ 可视化报告已保存至: {output_pdf_path}")
    print("  使用 PDF 查看器打开报告")

if __name__ == '__main__':
    main()