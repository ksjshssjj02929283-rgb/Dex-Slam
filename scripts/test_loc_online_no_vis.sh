#!/bin/bash

source /home/dex/projects/dexe_mobile_slam3d/install/setup.bash
ros2 launch lightning loc_online.launch.py lidar_type:=livox   map_folder:=/home/dex/.dexe/data/maps   map_name:=my_slam_bag_0
