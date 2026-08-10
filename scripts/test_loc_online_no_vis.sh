#!/bin/bash

source /home/dex/projects/dexe_mobile_slam3d/install/setup.bash
ros2 launch lightning loc_online.launch.py   lidar_type:=livox   map_folder:=/home/dexforce/.dexe/data/maps   map_name:=final
