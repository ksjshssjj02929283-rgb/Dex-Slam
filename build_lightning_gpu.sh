#!/bin/bash
# ============================================================
# GPU 版编译 (需要 CUDA，适用于 Jetson 等带 GPU 的设备)
# BBS 全局重定位使用 GPU 加速 (gpu_bbs3d)
# ============================================================
set -e
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
colcon build --cmake-args -DBUILD_CUDA=ON
