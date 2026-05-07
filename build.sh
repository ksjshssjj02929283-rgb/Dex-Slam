#!/bin/bash

# 设置 ROS 环境变量
source /opt/ros/humble/setup.bash
WORKSPACE_DIR=$(pwd)

# 输出环境中已有的ROS包名字, 检查是否有nav_interfaces包
if ros2 pkg list | grep -q "nav_interfaces"; then
    echo "✅ 找到 nav_interfaces 包, 跳过dexe_interface包编译。"
else
    echo "⚠️ 未找到 nav_interfaces 包, 需要重新编译 dexe_interfaces 包。"
    if [ -f "${WORKSPACE_DIR}/.gitmodules" ]; then
        echo "🔄 检测到 .gitmodules，正在同步子模块..."
        git submodule sync --recursive
        git submodule update --init --recursive
        echo "✅ 子模块已更新完成。"
    else
        echo "ℹ️ 未检测到 .gitmodules，跳过子模块更新。"
    fi
fi

# 打印并执行 colcon build 命令进行构建
echo "Running colcon build..."
# 仓根 package.xml 是供 CI 读版本号用的元数据,不参与构建。
# colcon 默认会在根目录识别到 package.xml 后停止下钻,从而屏蔽全部子包,
# 因此用 --base-paths 显式列出含子包的子目录(动态扫描,含 dexe_interfaces 子模块)。
SUBPATHS=$(find . -mindepth 2 -name package.xml -printf '%h\n' | awk -F/ '{print $2}' | sort -u | tr '\n' ' ')
echo "📂 子包搜索路径: ${SUBPATHS}"
colcon build --base-paths ${SUBPATHS}
