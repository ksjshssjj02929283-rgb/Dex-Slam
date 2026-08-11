#!/usr/bin/env python3
"""
PGO调试日志可视化工具 - 相对位姿版本

将所有位姿转换为相对于其初始时刻的相对位姿，便于不同数据源之间的对比分析。

转换逻辑：
- 对于每个阶段，取其第一个有效位姿作为参考原点
- 后续所有位姿都转换为相对于该原点的相对位姿
- 相对平移：relative_t = R_initial^(-1) * (t_current - t_initial)
- 相对旋转：relative_q = q_initial^(-1) * q_current

用法:
    python3 visualize_pgo_debug_relative.py [csv文件路径]
"""

import sys
import os
import glob
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages
import argparse
from scipy.spatial.transform import Rotation as R

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

# 颜色配置
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

def quat_to_matrix(qx, qy, qz, qw):
    """四元数转旋转矩阵"""
    return R.from_quat([qx, qy, qz, qw]).as_matrix()

def compute_relative_pose(tx, ty, tz, qx, qy, qz, qw, ref_tx, ref_ty, ref_tz, ref_qx, ref_qy, ref_qz, ref_qw):
    """计算相对位姿：relative_pose = ref_pose.inverse() * current_pose"""
    # 参考位姿的旋转矩阵和逆
    R_ref = quat_to_matrix(ref_qx, ref_qy, ref_qz, ref_qw)
    R_ref_inv = R_ref.T  # 旋转矩阵的逆等于转置
    
    # 当前位姿的旋转矩阵
    R_cur = quat_to_matrix(qx, qy, qz, qw)
    
    # 相对旋转：R_rel = R_ref^T * R_cur
    R_rel = R_ref_inv @ R_cur
    
    # 相对平移：t_rel = R_ref^T * (t_cur - t_ref)
    t_cur = np.array([tx, ty, tz])
    t_ref = np.array([ref_tx, ref_ty, ref_tz])
    t_rel = R_ref_inv @ (t_cur - t_ref)
    
    # 相对四元数
    q_rel = R.from_matrix(R_rel).as_quat()
    
    return t_rel[0], t_rel[1], t_rel[2], q_rel[0], q_rel[1], q_rel[2], q_rel[3]

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

def convert_to_relative_poses(df):
    """将各阶段位姿转换为相对于初始时刻的相对位姿"""
    print("\n转换位姿为相对位姿...")
    
    stages_to_convert = ['DR_INPUT', 'LO_INPUT', 'LIDARLOC_INPUT', 'OPTIMIZED', 'EXTRAPOLATED', 'SMOOTHED_OUTPUT']
    
    for stage in stages_to_convert:
        stage_mask = df['stage'] == stage
        stage_df = df[stage_mask]
        
        if len(stage_df) == 0:
            continue
        
        # 获取第一个有效位姿作为参考
        first_row = stage_df.iloc[0]
        ref_tx = first_row['tx']
        ref_ty = first_row['ty']
        ref_tz = first_row['tz']
        ref_qx = first_row['qx']
        ref_qy = first_row['qy']
        ref_qz = first_row['qz']
        ref_qw = first_row['qw']
        
        print(f"  {stage}: 参考位姿 ({ref_tx:.3f}, {ref_ty:.3f}, {ref_tz:.3f})")
        
        # 计算所有位姿相对于参考位姿的相对位姿
        relative_poses = []
        for idx, row in stage_df.iterrows():
            rel_tx, rel_ty, rel_tz, rel_qx, rel_qy, rel_qz, rel_qw = compute_relative_pose(
                row['tx'], row['ty'], row['tz'],
                row['qx'], row['qy'], row['qz'], row['qw'],
                ref_tx, ref_ty, ref_tz,
                ref_qx, ref_qy, ref_qz, ref_qw
            )
            relative_poses.append({
                'idx': idx,
                'rel_tx': rel_tx,
                'rel_ty': rel_ty,
                'rel_tz': rel_tz,
                'rel_qx': rel_qx,
                'rel_qy': rel_qy,
                'rel_qz': rel_qz,
                'rel_qw': rel_qw
            })
        
        # 更新DataFrame
        rel_df = pd.DataFrame(relative_poses).set_index('idx')
        df.loc[rel_df.index, 'tx'] = rel_df['rel_tx']
        df.loc[rel_df.index, 'ty'] = rel_df['rel_ty']
        df.loc[rel_df.index, 'tz'] = rel_df['rel_tz']
        df.loc[rel_df.index, 'qx'] = rel_df['rel_qx']
        df.loc[rel_df.index, 'qy'] = rel_df['rel_qy']
        df.loc[rel_df.index, 'qz'] = rel_df['rel_qz']
        df.loc[rel_df.index, 'qw'] = rel_df['rel_qw']
    
    print("✓ 位姿转换完成")
    return df

def plot_all_stages_trajectory(df, output_pdf):
    """绘制所有阶段的XY轨迹对比"""
    fig, ax = plt.subplots(1, 1, figsize=(14, 10))
    fig.suptitle('所有阶段XY平面轨迹对比（相对位姿）', fontsize=16, fontweight='bold')
    
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
    
    ax.set_xlabel('X (m) - 相对初始位置', fontsize=12)
    ax.set_ylabel('Y (m) - 相对初始位置', fontsize=12)
    ax.set_title('XY平面轨迹对比（相对位姿）', fontsize=14)
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
    fig.suptitle('X/Y/Z位置随时间变化对比（相对位姿）', fontsize=16, fontweight='bold')
    
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
        ax.set_ylabel(f'{label} (m) - 相对初始位置', fontsize=11)
        ax.set_title(f'{label}轴位置对比（相对位姿）', fontsize=13)
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
    fig.suptitle('原始输入 vs 内插后数据对比（相对位姿）', fontsize=16, fontweight='bold')
    
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
    
    # 内插数据（使用主位姿列）
    if len(interp_df) > 0 and not interp_df['tx'].isna().all():
        ax.plot(interp_df['tx'], interp_df['ty'],
               label='内插主位姿',
               color='#000000',
               linestyle=':',
               linewidth=2.5,
               alpha=0.9)
    
    ax.set_xlabel('X (m) - 相对初始位置')
    ax.set_ylabel('Y (m) - 相对初始位置')
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
    
    if len(interp_df) > 0 and not interp_df['tx'].isna().all():
        ax.plot(interp_df['time_relative'], interp_df['tx'],
               label='内插主位姿',
               color='#000000',
               linestyle=':',
               linewidth=2.5,
               alpha=0.9)
    
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('X (m) - 相对初始位置')
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
    
    if len(interp_df) > 0 and not interp_df['ty'].isna().all():
        ax.plot(interp_df['time_relative'], interp_df['ty'],
               label='内插主位姿',
               color='#000000',
               linestyle=':',
               linewidth=2.5,
               alpha=0.9)
    
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('Y (m) - 相对初始位置')
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
    
    if len(interp_df) > 0 and not interp_df['tz'].isna().all():
        ax.plot(interp_df['time_relative'], interp_df['tz'],
               label='内插主位姿',
               color='#000000',
               linestyle=':',
               linewidth=2.5,
               alpha=0.9)
    
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('Z (m) - 相对初始位置')
    ax.set_title('Z轴对比')
    ax.legend(fontsize=8, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_pdf.savefig(fig)
    plt.close()
    print("✓ 内插对比图已生成")

def plot_stage_comparison(df, output_pdf):
    """分阶段详细对比 - 分为三组"""
    
    # ===== 第一组：输入源轨迹对比 (DR vs LO vs LidarLoc) =====
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle('输入源轨迹对比（相对位姿）', fontsize=16, fontweight='bold')
    
    input_stages = [
        ('DR_INPUT', 'DR输入', COLORS['DR_INPUT'], '-'),
        ('LO_INPUT', 'LO输入', COLORS['LO_INPUT'], '--'),
        ('LIDARLOC_INPUT', 'LidarLoc输入', COLORS['LIDARLOC_INPUT'], '-.'),
    ]
    
    # XY轨迹
    ax = axes[0]
    for stage, label, color, ls in input_stages:
        stage_df = df[df['stage'] == stage]
        if len(stage_df) > 0:
            ax.plot(stage_df['tx'], stage_df['ty'],
                   label=label, color=color, linestyle=ls, linewidth=2, alpha=0.8)
    ax.set_xlabel('X (m) - 相对初始位置')
    ax.set_ylabel('Y (m) - 相对初始位置')
    ax.set_title('XY轨迹')
    ax.legend(fontsize=9, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    # X轴时间序列
    ax = axes[1]
    for stage, label, color, ls in input_stages:
        stage_df = df[df['stage'] == stage]
        if len(stage_df) > 0:
            ax.plot(stage_df['time_relative'], stage_df['tx'],
                   label=label, color=color, linestyle=ls, linewidth=1.5, alpha=0.8)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('X (m) - 相对初始位置')
    ax.set_title('X轴对比')
    ax.legend(fontsize=9, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    
    # Y轴时间序列
    ax = axes[2]
    for stage, label, color, ls in input_stages:
        stage_df = df[df['stage'] == stage]
        if len(stage_df) > 0:
            ax.plot(stage_df['time_relative'], stage_df['ty'],
                   label=label, color=color, linestyle=ls, linewidth=1.5, alpha=0.8)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('Y (m) - 相对初始位置')
    ax.set_title('Y轴对比')
    ax.legend(fontsize=9, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_pdf.savefig(fig)
    plt.close()
    print("✓ 输入源轨迹对比图已生成")
    
    # ===== 第二组：优化前后对比 (LidarLoc输入 vs 优化后) =====
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle('优化前后对比（相对位姿）', fontsize=16, fontweight='bold')
    
    optim_stages = [
        ('LIDARLOC_INPUT', 'LidarLoc输入', COLORS['LIDARLOC_INPUT'], '-.'),
        ('OPTIMIZED', '优化后', COLORS['OPTIMIZED'], '-'),
    ]
    
    # XY轨迹
    ax = axes[0]
    for stage, label, color, ls in optim_stages:
        stage_df = df[df['stage'] == stage]
        if len(stage_df) > 0:
            ax.plot(stage_df['tx'], stage_df['ty'],
                   label=label, color=color, linestyle=ls, linewidth=2, alpha=0.8)
    ax.set_xlabel('X (m) - 相对初始位置')
    ax.set_ylabel('Y (m) - 相对初始位置')
    ax.set_title('XY轨迹')
    ax.legend(fontsize=9, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    # X轴时间序列
    ax = axes[1]
    for stage, label, color, ls in optim_stages:
        stage_df = df[df['stage'] == stage]
        if len(stage_df) > 0:
            ax.plot(stage_df['time_relative'], stage_df['tx'],
                   label=label, color=color, linestyle=ls, linewidth=1.5, alpha=0.8)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('X (m) - 相对初始位置')
    ax.set_title('X轴对比')
    ax.legend(fontsize=9, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    
    # Y轴时间序列
    ax = axes[2]
    for stage, label, color, ls in optim_stages:
        stage_df = df[df['stage'] == stage]
        if len(stage_df) > 0:
            ax.plot(stage_df['time_relative'], stage_df['ty'],
                   label=label, color=color, linestyle=ls, linewidth=1.5, alpha=0.8)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('Y (m) - 相对初始位置')
    ax.set_title('Y轴对比')
    ax.legend(fontsize=9, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_pdf.savefig(fig)
    plt.close()
    print("✓ 优化前后对比图已生成")
    
    # ===== 第三组：优化后+外推+平滑输出对比 =====
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle('优化后+外推+平滑输出对比（相对位姿）', fontsize=16, fontweight='bold')
    
    output_stages = [
        ('OPTIMIZED', '优化后', COLORS['OPTIMIZED'], '-'),
        ('EXTRAPOLATED', '外推', COLORS['EXTRAPOLATED'], '--'),
        ('SMOOTHED_OUTPUT', '平滑输出', COLORS['SMOOTHED_OUTPUT'], '-'),
    ]
    
    # XY轨迹
    ax = axes[0]
    for stage, label, color, ls in output_stages:
        stage_df = df[df['stage'] == stage]
        if len(stage_df) > 0:
            ax.plot(stage_df['tx'], stage_df['ty'],
                   label=label, color=color, linestyle=ls, linewidth=2, alpha=0.8)
    ax.set_xlabel('X (m) - 相对初始位置')
    ax.set_ylabel('Y (m) - 相对初始位置')
    ax.set_title('XY轨迹')
    ax.legend(fontsize=9, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    # X轴时间序列
    ax = axes[1]
    for stage, label, color, ls in output_stages:
        stage_df = df[df['stage'] == stage]
        if len(stage_df) > 0:
            ax.plot(stage_df['time_relative'], stage_df['tx'],
                   label=label, color=color, linestyle=ls, linewidth=1.5, alpha=0.8)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('X (m) - 相对初始位置')
    ax.set_title('X轴对比')
    ax.legend(fontsize=9, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    
    # Y轴时间序列
    ax = axes[2]
    for stage, label, color, ls in output_stages:
        stage_df = df[df['stage'] == stage]
        if len(stage_df) > 0:
            ax.plot(stage_df['time_relative'], stage_df['ty'],
                   label=label, color=color, linestyle=ls, linewidth=1.5, alpha=0.8)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('Y (m) - 相对初始位置')
    ax.set_title('Y轴对比')
    ax.legend(fontsize=9, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_pdf.savefig(fig)
    plt.close()
    print("✓ 优化后+外推+平滑输出对比图已生成")
    
    # ===== 第四组：优化后+外推对比 =====
    fig, axes = plt.subplots(1, 3, figsize=(18, 6))
    fig.suptitle('优化后+外推对比（相对位姿）', fontsize=16, fontweight='bold')
    
    optim_extrap_stages = [
        ('OPTIMIZED', '优化后', COLORS['OPTIMIZED'], '-'),
        ('EXTRAPOLATED', '外推', COLORS['EXTRAPOLATED'], '--'),
    ]
    
    # XY轨迹
    ax = axes[0]
    for stage, label, color, ls in optim_extrap_stages:
        stage_df = df[df['stage'] == stage]
        if len(stage_df) > 0:
            ax.plot(stage_df['tx'], stage_df['ty'],
                   label=label, color=color, linestyle=ls, linewidth=2, alpha=0.8)
    ax.set_xlabel('X (m) - 相对初始位置')
    ax.set_ylabel('Y (m) - 相对初始位置')
    ax.set_title('XY轨迹')
    ax.legend(fontsize=9, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    # X轴时间序列
    ax = axes[1]
    for stage, label, color, ls in optim_extrap_stages:
        stage_df = df[df['stage'] == stage]
        if len(stage_df) > 0:
            ax.plot(stage_df['time_relative'], stage_df['tx'],
                   label=label, color=color, linestyle=ls, linewidth=1.5, alpha=0.8)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('X (m) - 相对初始位置')
    ax.set_title('X轴对比')
    ax.legend(fontsize=9, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    
    # Y轴时间序列
    ax = axes[2]
    for stage, label, color, ls in optim_extrap_stages:
        stage_df = df[df['stage'] == stage]
        if len(stage_df) > 0:
            ax.plot(stage_df['time_relative'], stage_df['ty'],
                   label=label, color=color, linestyle=ls, linewidth=1.5, alpha=0.8)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('Y (m) - 相对初始位置')
    ax.set_title('Y轴对比')
    ax.legend(fontsize=9, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    output_pdf.savefig(fig)
    plt.close()
    print("✓ 优化后+外推对比图已生成")
    
    # ===== 附加图：Z轴对比（四组） =====
    # Z轴 - 输入源
    fig, ax = plt.subplots(1, 1, figsize=(12, 5))
    fig.suptitle('Z轴对比 - 输入源（相对位姿）', fontsize=14, fontweight='bold')
    for stage, label, color, ls in input_stages:
        stage_df = df[df['stage'] == stage]
        if len(stage_df) > 0:
            ax.plot(stage_df['time_relative'], stage_df['tz'],
                   label=label, color=color, linestyle=ls, linewidth=1.5, alpha=0.8)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('Z (m) - 相对初始位置')
    ax.legend(fontsize=9, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    output_pdf.savefig(fig)
    plt.close()
    
    # Z轴 - 优化前后
    fig, ax = plt.subplots(1, 1, figsize=(12, 5))
    fig.suptitle('Z轴对比 - 优化前后（相对位姿）', fontsize=14, fontweight='bold')
    for stage, label, color, ls in optim_stages:
        stage_df = df[df['stage'] == stage]
        if len(stage_df) > 0:
            ax.plot(stage_df['time_relative'], stage_df['tz'],
                   label=label, color=color, linestyle=ls, linewidth=1.5, alpha=0.8)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('Z (m) - 相对初始位置')
    ax.legend(fontsize=9, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    output_pdf.savefig(fig)
    plt.close()
    
    # Z轴 - 优化后+外推
    fig, ax = plt.subplots(1, 1, figsize=(12, 5))
    fig.suptitle('Z轴对比 - 优化后+外推（相对位姿）', fontsize=14, fontweight='bold')
    for stage, label, color, ls in optim_extrap_stages:
        stage_df = df[df['stage'] == stage]
        if len(stage_df) > 0:
            ax.plot(stage_df['time_relative'], stage_df['tz'],
                   label=label, color=color, linestyle=ls, linewidth=1.5, alpha=0.8)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('Z (m) - 相对初始位置')
    ax.legend(fontsize=9, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    output_pdf.savefig(fig)
    plt.close()
    
    # Z轴 - 输出阶段
    fig, ax = plt.subplots(1, 1, figsize=(12, 5))
    fig.suptitle('Z轴对比 - 优化后+外推+平滑（相对位姿）', fontsize=14, fontweight='bold')
    for stage, label, color, ls in output_stages:
        stage_df = df[df['stage'] == stage]
        if len(stage_df) > 0:
            ax.plot(stage_df['time_relative'], stage_df['tz'],
                   label=label, color=color, linestyle=ls, linewidth=1.5, alpha=0.8)
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('Z (m) - 相对初始位置')
    ax.legend(fontsize=9, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    output_pdf.savefig(fig)
    plt.close()
    print("✓ Z轴对比图已生成（四组）")
    
    # ===== 附加图：平滑因子和置信度 =====
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle('平滑因子与置信度', fontsize=14, fontweight='bold')
    
    # 平滑因子
    ax = axes[0]
    smoothed_df = df[df['stage'] == 'SMOOTHED_OUTPUT']
    if len(smoothed_df) > 0 and not smoothed_df['smooth_factor'].isna().all():
        ax.plot(smoothed_df['time_relative'], smoothed_df['smooth_factor'],
               color=COLORS['SMOOTHED_OUTPUT'], linewidth=2, alpha=0.9)
        ax.axhline(y=0.5, color='orange', linestyle='--', alpha=0.5, label='0.5')
        ax.axhline(y=0.8, color='green', linestyle='--', alpha=0.5, label='0.8')
    ax.set_xlabel('时间 (s)')
    ax.set_ylabel('平滑因子')
    ax.set_title('平滑因子变化')
    ax.legend(fontsize=9, framealpha=0.9)
    ax.grid(True, alpha=0.3)
    ax.set_ylim(-0.05, 1.05)
    
    # LidarLoc置信度
    ax = axes[1]
    lidarloc_df = df[df['stage'] == 'LIDARLOC_INPUT']
    if len(lidarloc_df) > 0 and not lidarloc_df['confidence'].isna().all():
        ax.plot(lidarloc_df['time_relative'], lidarloc_df['confidence'],
               color=COLORS['LIDARLOC_INPUT'], linewidth=1.5, alpha=0.8)
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
    print("✓ 平滑因子与置信度图已生成")

def print_summary(df):
    """打印统计摘要"""
    print("\n" + "="*60)
    print("PGO融合过程统计摘要（相对位姿版本）")
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
    parser = argparse.ArgumentParser(description='PGO调试日志可视化工具 - 相对位姿版本')
    parser.add_argument('csv_file', nargs='?', help='CSV文件路径 (可选，默认使用最新的)')
    parser.add_argument('-o', '--output', help='输出PDF文件路径 (默认: pgo_debug_relative_analysis.pdf)')
    args = parser.parse_args()
    
    if args.csv_file:
        csv_path = args.csv_file
    else:
        csv_path = find_latest_csv()
    
    if not os.path.exists(csv_path):
        print(f"错误: 文件不存在 {csv_path}")
        sys.exit(1)
    
    df = load_csv(csv_path)
    
    # 转换为相对位姿
    df = convert_to_relative_poses(df)
    
    output_pdf_path = args.output or 'pgo_debug_relative_analysis.pdf'
    
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