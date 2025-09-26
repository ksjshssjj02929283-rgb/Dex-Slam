#!/bin/bash

source ../../../../../install/setup.bash
# 地图路径需要看情况修改
map_dir="${HOME}/hz/Work/Grabotics/B1-project/B1-SLAM/src/B1-2dLidar-SLAM/SLAM_ros/B1-cartographer_ros/cartographer_ros/results/map"
map_name="rslidar-outdoor-gps"

# 检查文件夹是否存在, 如果不存在就创建文件夹
if [ ! -d "$map_dir" ];then
  echo "文件夹不存在, 正在创建文件夹"
  mkdir -p $map_dir
fi


# finish slam
ros2 service call /finish_trajectory 0

# make pbstream
ros2 service call /write_state "{filename: '$map_dir/$map_name.pbstream'}"

# pbstream to map（.pgm/.yaml）
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
-pbstream_filename=$map_dir/$map_name.pbstream \
-map_filestem=$map_dir/$map_name
