#!/usr/bin/env python3
"""
PGO调试日志可视化工具 - 绝对时间版本
忽略DR_input开始的相对时间数据，只使用绝对时间戳的数据进行分析

新增功能：
1. 对比 DR,LO,LidarLoc输入、PGO优化后、外推后、平滑输出的XY轨迹和X/Y/Z随时间变化
2. 对比DR,LO,LidarLoc输入和内插后的DR/LO/LidarLoc
3. 优化可视化清晰度，避免平滑输出遮挡其他结果

用法:
    python3 visualize_pgo_debug_absolute.py [csv文件路径]
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

# 颜色配置 - 使用更区分度高的颜色
COLORS = {
    'DR_INPUT': '#FF8C00',        # 深橙色
    'LO_INPUT': '#00CED1',        # 深青色
    'LIDARLOC_INPUT': '#DC143C',  # 深红色
    'INTERPOLATED': '#9370DB',    # 紫色
    'OPTIMIZED': '#228B22',       # 深绿色
    'EXTRAPOLATED': '#4169E1',    # 宝蓝色
    'SMOOTHED_OUTPUT': '#8B008B', # 深紫色
}

LINE_STYLES = {
    'DR_INPUT': '-',
    'LO_INPUT': '--',
    'LIDARLOC_INPUT': '-.',
    'INTERPOLATED': ':',
    'OPTIMIZED': '-',
    'EXTRAPOLATED': '--',
    'SMOOTHED_OUTPUT': '-',
}

LINE_WIDTHS = {
    'DR_INPUT': 1.5,
    'LO_INPUT': 1.5,
    'LIDARLOC_INPUT': 1.5,
    'INTERPOLATED': 2.0,
    'OPTIMIZED': 2.0,
    'EXTRAPOLATED': 1.5,
    'SMOOTHED_OUTPUT': 2.5,
}

def find_latest_csv():
    """查找最新的CSV文件"""
    csv_files = glob.glob('/tmp/pgo_debug/pgo_debug_*.csv')
    if not csv_files:
        print("错误: 未找到 /tmp/pgo_debug/pgo_debug_*.csv 文件")
        sys.exit(1)
    
    csv_files.sort(key=os.path.getmtime, reverse=True)
    return csv_files[0]

def load_csv(csv_path):
    """加载CSV文件并过滤掉DR的相对时间数据"""
    print(f"加载文件: {csv_path}")
    
    df = pd.read_csv(csv_path, comment='#')
    
    # 过滤DR_INPUT数据：只保留时间戳 > 1000 的数据（绝对时间）
    dr_mask = df['stage'] != 'DR_INPUT'
    dr_absolute_mask = (df['stage'] == 'DR_INPUT') & (df['timestamp'] > 1000)
    df_filtered = df[dr_mask | dr_absolute_mask].copy()
    
    dr_removed = len(df) - len(df_filtered)
    print(f"过滤掉 {dr_removed} 条DR相对时间数据")
    print(f"剩余 {len(df_filtered)} 条记录")
    
    # 使用绝对时间戳作为时间基准
    min_timestamp = df_filtered['timestamp'].min()
    df_filtered['time_relative'] = df_filtered['timestamp'] - min_timestamp
    
    print(f"时间跨度: {df_filtered['time_relative'].iloc[0]:.2f}s - {df_filtered['time_relative'].iloc[-1]:.2f}s")
    
    return df_filtered

def plot_all_stages_trajectory(df, output_pdf):
    """绘制所有阶段的XY轨迹对比"""
    fig, ax = plt.subplots(1, 1, figsize=(14, 10))
    fig.suptitle('所有阶段XY平面轨迹对比', fontsize=16, fontweight='bold')
    
    stages_order = ['DR_INPUT', 'LO_INPUT', 'LIDARLOC_INPUT', 'OPTIMIZED', 'EXTRAPOLATED', 'SMOOTHED_OUTPUT']
    stage_labels = {
        'DR_INPUT': 'DR输入',
        'LO_INPUT': 'LO输入',
        'LIDARLOC_INPUT': 'LidarLoc输入',
        'OPTIMIZED': 'PGO优化后',
        'EXTRAPOLATED': '外推后',
        'SMOOTHED_OUTPUT': '平滑输出',
    }
    
    for stage in stages_order:
        stage_df = df[df['stage'] == stage]
        if len(stage_df) > 0:
            ax.plot(stage_df['tx'], stage_df['ty'],
                   label=stage_labels[stage],
                   color=COLORS[stage],
                   linestyle=LINE_STYLES[stage],
                   linewidth=LINE_WIDTHS[stage],
                   alpha=0.8)
    
    ax.set_xlabel('X (m)', fontsize=12)
    ax.set_ylabel('Y (m)', fontsize=12)
    ax.set_title('XY平面轨迹对比', fontsize=14)
    ax.legend(loc='best', fontsize=10, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    plt.tight_layout()
    output_pdf.savefig(fig)
    plt.close()
    print("✓ 所有阶段XY轨迹对比图已生成")

def plot_xyz_time_series(df, output_pdf):
    """绘制X/Y/Z随时间变化的对比图"""
    fig, axes = plt.subplots(3, 1, figsize=(16, 12), sharex=True)
    fig.suptitle('X/Y/Z位置随时间变化对比', fontsize=16, fontweight='bold')
    
    stages_order = ['DR_INPUT', 'LO_INPUT', 'LIDARLOC_INPUT', 'OPTIMIZED', 'EXTRAPOLATED', 'SMOOTHED_OUTPUT']
    stage_labels = {
        'DR_INPUT': 'DR输入',
        'LO_INPUT': 'LO输入',
        'LIDARLOC_INPUT': 'LidarLoc输入',
        'OPTIMIZED': 'PGO优化后',
        'EXTRAPOLATED': '外推后',
        'SMOOTHED_OUTPUT': '平滑输出',
    }
    
    for idx, (axis, label) in enumerate(zip(['tx', 'ty', 'tz'], ['X', 'Y', 'Z'])):
        ax = axes[idx]
        for stage in stages_order:
            stage_df = df[df['stage'] == stage]
            if len(stage_df) > 0:
                ax.plot(stage_df['time_relative'], stage_df[axis],
                       label=stage_labels[stage],
                       color=COLORS[stage],
                       linestyle=LINE_STYLES[stage],
                       linewidth=LINE_WIDTHS[stage],
                       alpha=0.85)
        ax.set_ylabel(f'{label} (m)', fontsize=11)
        ax.set_title(f'{label}轴位置对比', fontsize=13)
        ax.legend(loc='upper right', fontsize=8, ncol=2, framealpha=0.9)
        ax.grid(True, alpha=0.3)
    
    axes[2].set_xlabel('时间 (s)', fontsize=12)
    plt.tight_layout()
    output_pdf.savefig(fig)
    plt.close()
    print("✓ X/Y/Z时间序列对比图已生成")

def plot_interpolation_comparison(df, output_pdf):
    """对比原始输入和内插后的DR/LO/LidarLoc"""
    fig, axes = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('原始输入 vs 内插后数据对比', fontsize=16, fontweight='bold')
    
    # 提取内插数据
    interp_df = df[df['stage'] == 'INTERPOLATED']
    
    if len(interp_df) == 0:
        print("  警告: 未找到内插数据，跳过此图")
        plt.close()
        return
    
    # XY轨迹对比
    ax = axes[0, 0]
    
    # 原始输入
    for stage, label, color in [('DR_INPUT', 'DR输入', COLORS['DR_INPUT']),
                                 ('LO_INPUT', 'LO输入', COLORS['LO_INPUT']),
                                 ('LIDARLOC_INPUT', 'LidarLoc输入', COLORS['LIDARLOC_INPUT'])]:
        stage_df = df[df['stage'] == stage]
        if len(stage_df) > 0:
            ax.plot(stage_df['tx'], stage_df['ty'],
                   label=f'原始{label}',
                   color=color,
                   linestyle='-',
                   linewidth=1.5,
                   alpha=0.6)
    
    # 内插数据
    if len(interp_df) > 0 and not interp_df['dr_tx'].isna().all():
        ax.plot(interp_df['dr_tx'], interp_df['dr_ty'],
               label='内插后DR',
               color=COLORS['DR_INPUT'],
               linestyle=':',
               linewidth=2.5,
               alpha=0.9)
        ax.plot(interp_df['lo_tx'], interp_df['lo_ty'],
               label='内插后LO',
               color=COLORS['LO_INPUT'],
               linestyle=':',
               linewidth=2.5,
               alpha=0.9)
        ax.plot(interp_df['loc_tx'], interp_df['loc_ty'],
               label='内插后LidarLoc',
               color=COLORS['LIDARLOC_INPUT'],
               linestyle=':',
               linewidth=2.5,
               alpha=0.9)
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('XY轨迹对比')
    ax.legend(fontsize=8, ncol=2, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    # X轴对比
    ax = axes[0, 1]
    for stage, label, color in [('DR_INPUT', 'DR输入', COLORS['DR_INPUT']),
                                 ('LO_INPUT', 'LO输入', COLORS['LO_INPUT']),
                                 ('LIDARLOC_INPUT', 'LidarLoc输入', COLORS['LIDARLOC_INPUT'])]:
        stage_df = df[df['stage'] == stage]
        if len(stage_df) > 0:
            ax.plot(stage_df['time_relative'], stage_df['tx'],
                   label=f'原始{label}',
                   color=color,
                   linestyle='-',
                   linewidth=1.5,
                   alpha=0.6)
    
    if len(interp_df) > 0 and not interp_df['dr_tx'].isna().all():
        ax.plot(interp_df['time_relative'], interp_df['dr_tx'],
               label='内插后DR',
               color=COLORS['DR_INPUT'],
               linestyle=':',
               linewidth=2.5,
               alpha=0.9)
        ax.plot(interp_df['time_relative'], interp_df['lo_tx'],
               label='内插后LO',
               color=COLORS['LO_INPUT'],
               linestyle=':',
               linewidth=2.5,
               alpha=0.9)
        ax.plot(interp_df['time_relative'], interp_df['loc_tx'],
               label='内插后LidarLoc',
               color=COLORS['LIDARLOC_INPUT'],
               linestyle=':',
               linewidth=2.5,
               alpha=0.9)
    
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('X (m)')
    ax.set_title('X轴对比')
    ax.legend(fontsize=8, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    
    # Y轴对比
    ax = axes[1, 0]
    for stage, label, color in [('DR_INPUT', 'DR输入', COLORS['DR_INPUT']),
                                 ('LO_INPUT', 'LO输入', COLORS['LO_INPUT']),
                                 ('LIDARLOC_INPUT', 'LidarLoc输入', COLORS['LIDARLOC_INPUT'])]:
        stage_df = df[df['stage'] == stage]
        if len(stage_df) > 0:
            ax.plot(stage_df['time_relative'], stage_df['ty'],
                   label=f'原始{label}',
                   color=color,
                   linestyle='-',
                   linewidth=1.5,
                   alpha=0.6)
    
    if len(interp_df) > 0 and not interp_df['dr_ty'].isna().all():
        ax.plot(interp_df['time_relative'], interp_df['dr_ty'],
               label='内插后DR',
               color=COLORS['DR_INPUT'],
               linestyle=':',
               linewidth=2.5,
               alpha=0.9)
        ax.plot(interp_df['time_relative'], interp_df['lo_ty'],
               label='内插后LO',
               color=COLORS['LO_INPUT'],
               linestyle=':',
               linewidth=2.5,
               alpha=0.9)
        ax.plot(interp_df['time_relative'], interp_df['loc_ty'],
               label='内插后LidarLoc',
               color=COLORS['LIDARLOC_INPUT'],
               linestyle=':',
               linewidth=2.5,
               alpha=0.9)
    
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('Y (m)')
    ax.set_title('Y轴对比')
    ax.legend(fontsize=8, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    
    # Z轴对比
    ax = axes[1, 1]
    for stage, label, color in [('DR_INPUT', 'DR输入', COLORS['DR_INPUT']),
                                 ('LO_INPUT', 'LO输入', COLORS['LO_INPUT']),
                                 ('LIDARLOC_INPUT', 'LidarLoc输入', COLORS['LIDARLOC_INPUT'])]:
        stage_df = df[df['stage'] == stage]
        if len(stage_df) > 0:
            ax.plot(stage_df['time_relative'], stage_df['tz'],
                   label=f'原始{label}',
                   color=color,
                   linestyle='-',
                   linewidth=1.5,
                   alpha=0.6)
    
    if len(interp_df) > 0 and not interp_df['dr_tz'].isna().all():
        ax.plot(interp_df['time_relative'], interp_df['dr_tz'],
               label='内插后DR',
               color=COLORS['DR_INPUT'],
               linestyle=':',
               linewidth=2.5,
               alpha=0.9)
        ax.plot(interp_df['time_relative'], interp_df['lo_tz'],
               label='内插后LO',
               color=COLORS['LO_INPUT'],
               linestyle=':',
               linewidth=2.5,
               alpha=0.9)
        ax.plot(interp_df['time_relative'], interp_df['loc_tz'],
               label='内插后LidarLoc',
               color=COLORS['LIDARLOC_INPUT'],
               linestyle=':',
               linewidth=2.5,
               alpha=0.9)
    
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('Z (m)')
    ax.set_title('Z轴对比')
    ax.legend(fontsize=8, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_pdf.savefig(fig)
    plt.close()
    print("✓ 内插对比图已生成")

def plot_stage_comparison(df, output_pdf):
    """分阶段详细对比"""
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle('各阶段详细对比', fontsize=16, fontweight='bold')
    
    # 1. 输入源对比 (DR vs LO vs LidarLoc)
    ax = axes[0, 0]
    for stage, label, color in [('DR_INPUT', 'DR', COLORS['DR_INPUT']),
                                 ('LO_INPUT', 'LO', COLORS['LO_INPUT']),
                                 ('LIDARLOC_INPUT', 'LidarLoc', COLORS['LIDARLOC_INPUT'])]:
        stage_df = df[df['stage'] == stage]
        if len(stage_df) > 0:
            ax.plot(stage_df['tx'], stage_df['ty'],
                   label=label,
                   color=color,
                   linewidth=2,
                   alpha=0.8)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('输入源XY轨迹')
    ax.legend(fontsize=9, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    # 2. 优化前后对比
    ax = axes[0, 1]
    for stage, label, color in [('LIDARLOC_INPUT', 'LidarLoc输入', COLORS['LIDARLOC_INPUT']),
                                 ('OPTIMIZED', '优化后', COLORS['OPTIMIZED'])]:
        stage_df = df[df['stage'] == stage]
        if len(stage_df) > 0:
            ax.plot(stage_df['tx'], stage_df['ty'],
                   label=label,
                   color=color,
                   linewidth=2,
                   alpha=0.8)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('优化前后对比')
    ax.legend(fontsize=9, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    # 3. 外推 vs 平滑
    ax = axes[0, 2]
    for stage, label, color in [('EXTRAPOLATED', '外推', COLORS['EXTRAPOLATED']),
                                 ('SMOOTHED_OUTPUT', '平滑输出', COLORS['SMOOTHED_OUTPUT'])]:
        stage_df = df[df['stage'] == stage]
        if len(stage_df) > 0:
            ax.plot(stage_df['tx'], stage_df['ty'],
                   label=label,
                   color=color,
                   linewidth=2,
                   alpha=0.8)
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_title('外推 vs 平滑')
    ax.legend(fontsize=9, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    # 4. Z轴时间序列 - 所有阶段
    ax = axes[1, 0]
    stages_order = ['DR_INPUT', 'LO_INPUT', 'LIDARLOC_INPUT', 'OPTIMIZED', 'EXTRAPOLATED', 'SMOOTHED_OUTPUT']
    stage_labels = {
        'DR_INPUT': 'DR',
        'LO_INPUT': 'LO',
        'LIDARLOC_INPUT': 'LidarLoc',
        'OPTIMIZED': '优化后',
        'EXTRAPOLATED': '外推',
        'SMOOTHED_OUTPUT': '平滑输出',
    }
    for stage in stages_order:
        stage_df = df[df['stage'] == stage]
        if len(stage_df) > 0:
            ax.plot(stage_df['time_relative'], stage_df['tz'],
                   label=stage_labels[stage],
                   color=COLORS[stage],
                   linewidth=1.5,
                   alpha=0.8)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('Z (m)')
    ax.set_title('Z轴时间序列')
    ax.legend(fontsize=7, ncol=2, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    
    # 5. 平滑因子
    ax = axes[1, 1]
    smoothed_df = df[df['stage'] == 'SMOOTHED_OUTPUT']
    if len(smoothed_df) > 0 and not smoothed_df['smooth_factor'].isna().all():
        ax.plot(smoothed_df['time_relative'], smoothed_df['smooth_factor'],
               color=COLORS['SMOOTHED_OUTPUT'],
               linewidth=2,
               alpha=0.9)
        ax.axhline(y=0.5, color='orange', linestyle='--', alpha=0.5, label='0.5')
        ax.axhline(y=0.8, color='green', linestyle='--', alpha=0.5, label='0.8')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('平滑因子')
    ax.set_title('平滑因子变化')
    ax.legend(fontsize=9, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(-0.05, 1.05)
    
    # 6. LidarLoc置信度
    ax = axes[1, 2]
    lidarloc_df = df[df['stage'] == 'LIDARLOC_INPUT']
    if len(lidarloc_df) > 0 and not lidarloc_df['confidence'].isna().all():
        ax.plot(lidarloc_df['time_relative'], lidarloc_df['confidence'],
               color=COLORS['LIDARLOC_INPUT'],
               linewidth=1.5,
               alpha=0.8)
        ax.axhline(y=0.5, color='orange', linestyle='--', alpha=0.5, label='阈值0.5')
        ax.axhline(y=0.6, color='green', linestyle='--', alpha=0.5, label='满权重0.6')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('置信度')
    ax.set_title('LidarLoc置信度')
    ax.legend(fontsize=9, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(-0.05, 1.05)
    
    plt.tight_layout()
    output_pdf.savefig(fig)
    plt.close()
    print("✓ 分阶段对比图已生成")

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
        print(f"  低置信(<0.5)次数: {low_conf} ({low_conf/len(lidarloc_df)*100:.1f}%)")
    
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
    parser = argparse.ArgumentParser(description='PGO调试日志可视化工具 - 绝对时间版本')
    parser.add_argument('csv_file', nargs='?', help='CSV文件路径 (可选，默认使用最新的)')
    parser.add_argument('-o', '--output', help='输出PDF文件路径 (默认: pgo_debug_absolute_analysis.pdf)')
    args = parser.parse_args()
    
    if args.csv_file:
        csv_path = args.csv_file
    else:
        csv_path = find_latest_csv()
    
    if not os.path.exists(csv_path):
        print(f"错误: 文件不存在 {csv_path}")
        sys.exit(1)
    
    df = load_csv(csv_path)
    
    output_pdf_path = args.output or 'pgo_debug_absolute_analysis.pdf'
    
    with PdfPages(output_pdf_path) as pdf:
        print(f"\n生成可视化报告: {output_pdf_path}")
        plot_all_stages_trajectory(df, pdf)
        plot_xyz_time_series(df, pdf)
        plot_interpolation_comparison(df, pdf)
        plot_stage_comparison(df, pdf)
    
    print_summary(df)
    
    print(f"\n✓ 可视化报告已保存至: {output_pdf_path}")

if __name__ == '__main__':
    main()