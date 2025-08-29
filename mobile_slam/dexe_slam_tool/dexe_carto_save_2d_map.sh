#!/bin/bash
script_path=$(dirname "$(realpath "$0")")
source $script_path/../../../install/setup.bash

# 地图名
current_date_time=$(date +%Y%m%d%H%M%S)
map_filename="w1_${current_date_time}"
# echo "Map name is: $map_name"
# map_name="b1_indoor_20240307"

# mobile_nav2下的地图文件夹
map_dir="$script_path/../../../dexe_mobile_application/mobile_simulation/w1_nav2/maps/$map_filename"

json_file="$script_path/../../../dexe_mobile_application/mobile_simulation/w1_nav2/config/config.json"


#grip_map_dir="$script_path/../../mobile_map/2d_lidar_grid_maps"
#nav2_map下的地图文件夹
#nav_grip_map_dir="$script_path/../../../dexe_mobile_application/mobile_simulation/buddy_nav2/maps"

# 检查文件夹是否存在, 如果不存在就创建文件夹
if [ ! -d "$map_dir" ];then
  echo "文件夹不存在, 正在创建文件夹"
  mkdir -p $map_dir
fi
echo "The Latest Map Path is: $map_dir"
# 检查文件夹是否存在, 如果不存在就创建文件夹
# if [ ! -d "$grip_map_dir" ];then
#   echo "文件夹不存在, 正在创建文件夹"
#   mkdir -p $grip_map_dir
# fi


# finish slam
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"

# make pbstream
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '$map_dir/map.pbstream', include_unfinished_submaps: 1}"

# pbstream to map
# 先把地图保存在b1_slam下
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
-pbstream_filename=$map_dir/map.pbstream \
-map_filestem=$map_dir/map


# json_file="$script_path/../../../dexe_mobile_application/mobile_simulation/w1_nav2/config/config.json"

# 检查文件夹是否存在, 如果不存在就创建文件夹
if [ ! -d "$json_file" ];then
  echo "文件不存在, 正在创建文件夹"
  mkdir -p $map_dir
fi
echo "The Latest Map Path is: $map_dir"

# 更新 JSON 里的 map_time 字段
jq --arg map_filename "$map_filename" '.map_filename = $map_filename' "$json_file" > "${json_file}.tmp" && mv "${json_file}.tmp" "$json_file"

echo "Updated JSON file: $json_file"
cat $json_file


sleep 5

# 检查文件夹是否存在, 如果不存在就创建文件夹
# if [ ! -d "$nav_grip_map_dir" ];then
#   echo "$nav_grip_map_dir 文件夹不存在, 正在创建文件夹"
#   mkdir -p $nav_grip_map_dir
# fi
# 再把地图复制到b1_map下
# cp $grip_map_dir/$map_name.pgm $grip_map_dir/$map_name.yaml $nav_grip_map_dir/

# echo "The map name is: $map_name"


