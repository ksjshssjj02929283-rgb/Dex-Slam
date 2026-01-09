#!/bin/bash

# 设置 ROS 环境变量
source /opt/ros/humble/setup.bash

# 打印并执行 colcon build 命令进行构建
echo "Running colcon build..."
colcon build
