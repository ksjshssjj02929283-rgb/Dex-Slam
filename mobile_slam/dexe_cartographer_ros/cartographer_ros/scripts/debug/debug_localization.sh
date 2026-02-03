#!/bin/bash

# Debug script for localization node
# Usage: ./debug_localization.sh [map_name]

# Get the workspace directory dynamically
# When installed via ros2, the script is in install/cartographer_ros/lib/cartographer_ros/
# We need to go up to the workspace root (install/../..)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "$SCRIPT_DIR" == *"/install/cartographer_ros/lib/cartographer_ros"* ]]; then
    # Running from installed location (ros2 run)
    # Path: install/cartographer_ros/lib/cartographer_ros -> ../../../.. -> workspace root
    WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
else
    # Running from source
    WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../../../../.." && pwd)"
fi

# Get user home directory
HOME_DIR="${HOME}"
DEFAULT_DATA_DIR="${HOME_DIR}/.dexe/data"

# Default parameters
MAP_FOLDER="${DEFAULT_DATA_DIR}/maps"
MAP_NAME="${1:-map_2_standard}"
RESOLUTION="0.04"

# Configuration paths
CARTOGRAPHER_ROS_INSTALL="${WORKSPACE_DIR}/install/cartographer_ros"
EXECUTABLE="${CARTOGRAPHER_ROS_INSTALL}/lib/cartographer_ros/localization_node"
CONFIG_DIR="${CARTOGRAPHER_ROS_INSTALL}/share/cartographer_ros/configuration_files"
CONFIG_BASENAME="dexe_localization_2d.lua"

# Source ROS2 environment
source "${WORKSPACE_DIR}/install/setup.bash"

# Print debug information
echo "========================================"
echo "Localization Node Debug Configuration"
echo "========================================"
echo "Executable: ${EXECUTABLE}"
echo "Config Dir: ${CONFIG_DIR}"
echo "Config File: ${CONFIG_BASENAME}"
echo "Map Folder: ${MAP_FOLDER}"
echo "Map Name: ${MAP_NAME}"
echo "Resolution: ${RESOLUTION}"
echo "========================================"
echo ""
echo "Starting GDB..."
echo "GDB commands:"
echo "  r           - Run the program"
echo "  b <line>    - Set breakpoint"
echo "  c           - Continue execution"
echo "  bt          - Backtrace (stack trace)"
echo "  p <var>     - Print variable"
echo "  q           - Quit GDB"
echo "========================================"
echo ""

# Set ROS2 topic remappings as environment variables
export ROS_ARGS="--ros-args"
export ROS_REMAP_1="-r scan_1:=laser_scan/front"
export ROS_REMAP_2="-r scan_2:=laser_scan/rear"
export ROS_REMAP_3="-r imu:=imu/chassis/refined"
export ROS_REMAP_4="-r odom:=feedback/odom"
export ROS_PARAM_1="-p use_sim_time:=false"

# Run with GDB
gdb -ex run -args "${EXECUTABLE}" \
    -configuration_directory "${CONFIG_DIR}" \
    -configuration_basename "${CONFIG_BASENAME}" \
    -map_folder "${MAP_FOLDER}" \
    -map_name "${MAP_NAME}" \
    -collect_metrics false \
    ${ROS_ARGS} \
    ${ROS_REMAP_1} \
    ${ROS_REMAP_2} \
    ${ROS_REMAP_3} \
    ${ROS_REMAP_4} \
    ${ROS_PARAM_1}
