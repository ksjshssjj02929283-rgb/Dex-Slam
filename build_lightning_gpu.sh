#!/bin/bash
# ============================================================
# GPU 版编译 (需要 CUDA，适用于 Jetson 等带 GPU 的设备)
# BBS 全局重定位使用 GPU 加速 (gpu_bbs3d)
# ============================================================
set -e
cd "$(dirname "$0")"
source /opt/ros/humble/setup.bash
 
# 自动探测 CUDA: 若 nvcc 不在 PATH, 尝试 /usr/local/cuda (Ubuntu 默认安装位置)
if ! command -v nvcc >/dev/null 2>&1; then
    if [ -x /usr/local/cuda/bin/nvcc ]; then
        export CUDA_HOME=/usr/local/cuda
        export PATH="${CUDA_HOME}/bin:${PATH}"
        export LD_LIBRARY_PATH="${CUDA_HOME}/lib64:${LD_LIBRARY_PATH}"
        echo "[build_lightning_gpu] 已自动导出 CUDA 路径: ${CUDA_HOME}"
    else
        echo "[build_lightning_gpu] 错误: 找不到 nvcc, 请安装 CUDA Toolkit 或手动将 nvcc 加入 PATH" >&2
        exit 1
    fi
fi

colcon build --cmake-args -DBUILD_CUDA=ON
