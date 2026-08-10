#!/bin/bash
# 安装 Pangolin 库
# 参考: https://github.com/stevenlovegrove/Pangolin

set -e

echo "========================================="
echo "开始安装 Pangolin 库"
echo "========================================="

# 安装依赖
echo "[1/5] 安装 Pangolin 依赖..."
sudo apt install -y \
    libglew-dev \
    libpython3-dev \
    libegl1-mesa-dev \
    libxcb-xinerama0 \
    libxcb-xinerama0-dev \
    libepoxy-dev \
    cmake \
    g++ \
    git

# 安装 Python wheel 模块（Pangolin CMake 需要）
echo "[2/5] 安装 Python wheel 模块..."
sudo apt install -y python3-pip python3-wheel python3-setuptools

# 克隆 Pangolin 源码
echo "[3/5] 克隆 Pangolin 源码..."
cd /tmp
if [ -d "Pangolin" ]; then
    echo "Pangolin 目录已存在，使用现有目录"
    cd Pangolin
else
    git clone --recurse-submodules https://github.com/stevenlovegrove/Pangolin.git
    cd Pangolin
fi

# 编译 Pangolin
echo "[4/5] 编译 Pangolin..."
mkdir -p build
cd build
cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DBUILD_PYTHON_INTERFACE=OFF

make -j$(nproc)

# 安装
echo "[5/5] 安装 Pangolin..."
sudo make install

# 更新动态链接库缓存
sudo ldconfig

echo "========================================="
echo "Pangolin 安装完成!"
echo "========================================="