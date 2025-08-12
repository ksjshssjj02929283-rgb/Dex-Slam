# B1-2dLidar-SLAM
B1-2dLidar-SLAM is a real-time SLAM framework for 2D Lidar-Inertial Systems in B1 project. It is developed on the basis of Cartographer and slam_toolbox respectively. 
# 1. Download
- clone repository into your ROS2 workspace  
    ```
    cd ~/ros2_ws/src  
    git clone git@github.com:zzhh00/B1-2dLidar-SLAM.git
    ```
# 2. Prerequisites
## B1-cartographer_ros
- Ubuntu 20.04(focal) + ROS2(galactic/foxy) (you need switch to corresponding branch).
- copy `B1-carto-lib-build.sh` to your path for 3rd library
- run `./B1-carto-lib-build.sh` to install dependencies
## B1-slam_toolbox_ros
- Ubuntu 20.04(focal) + ROS2(galactic/foxy) (you need switch to corresponding branch).
- run `rosdep install --from-paths src/B1-2dLidar-SLAM/SLAM_ros/B1-slam_toolbox_ros --ignore-src --rosdistro=${ROS_DISTRO} -y` to install dependencies
# 3. Build B1-SLAM
## B1-cartographer_ros
- Open the terminal and compile B1-cartographer and B1-cartographer_ros
  ```
  cd .. 
  colon build
  ```

## B1-slam_toolbox_ros
- Open the terminal and compile B1-slam_toolbox_ros seperately 
  ```
  cd .. 
  colon build --paths src/B1-2dLidar-SLAM/SLAM_ros/B1-slam_toolbox_ros --packages-select slam_toolbox
  ```
## B1-SLAM
- Open the terminal and compile B1-SLAM including cartographer and slam_toolbox
  ```
  cd .. 
  colon build
  ```
# 4. Run B1-SLAM
- set environment variables  
  ```
  source install setup.bash
  ```
- launch **B1-cartographer_ros**  
  ```
  ros2 launch cartographer_ros B1-2d_mapping_indoor.launch.py
  ``` 
  <font color=red>Note :</font> Some bug exists now when running B1-cartographer_ros in ROS2(foxy). It should be fixed in the next few days.
- launch **B1-slam_toolbox_ros** 
  ```
  ros2 launch slam_toolbox B1-2d_mapping_indoor.launch.py
  ```
# 5. Topics


# 6. Documents
The detailed description for each framework can be found under the corresponding ROS wrapper.

# Todo list
- [x] Support cartographer in ROS2(galactic)
- [x] Support cartographer in ROS2(foxy)
- [x] Support slam_toolbox in ROS2(galactic)
- [x] Support slam_toolbox in ROS2(foxy)
- [ ] Add 2d-Lidar-SLAM which is totally developed by ourself
- [ ] Fusion with GPT-4  