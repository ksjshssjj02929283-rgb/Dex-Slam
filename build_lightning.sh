#!/bin/bash
# ============================================================
# CPU 版编译 (无需 CUDA，适用于普通 PC / 笔记本)
# BBS 全局重定位使用 CPU 多线程 (cpu_bbs3d，速度较慢)
# ============================================================
set -e
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DBUILD_CUDA=OFF
