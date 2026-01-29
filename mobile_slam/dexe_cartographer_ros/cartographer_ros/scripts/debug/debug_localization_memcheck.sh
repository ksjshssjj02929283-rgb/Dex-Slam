#!/bin/bash

# Valgrind Memcheck for Memory Leak Detection in Localization Node
# Usage: ./debug_localization_memcheck.sh [map_name]

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Get the workspace directory dynamically
# When installed via ros2, the script is in install/cartographer_ros/lib/cartographer_ros/
# We need to go up to the workspace root
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "$SCRIPT_DIR" == *"/install/cartographer_ros/lib/cartographer_ros"* ]]; then
    # Running from installed location (ros2 run)
    # Path: install/cartographer_ros/lib/cartographer_ros/ -> go up 4 levels to workspace root
    WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../../.." && pwd)"
else
    # Running from source
    # Path: dexe_mobile_slam/mobile_slam/dexe_cartographer_ros/cartographer_ros/scripts/debug/
    # -> go up 6 levels to workspace root
    WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../../../.." && pwd)"
fi

# Configuration
MAP_FOLDER="$HOME/.dexe/data/maps"
MAP_NAME="${1:-map_2}"
CONFIG_DIR="${WORKSPACE_DIR}/install/cartographer_ros/share/cartographer_ros/configuration_files"
OUTPUT_DIR="${WORKSPACE_DIR}/valgrind_output"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# Create output directory
mkdir -p "${OUTPUT_DIR}"

echo -e "${GREEN}======================================${NC}"
echo -e "${GREEN}Valgrind Memcheck - Memory Leak Detection${NC}"
echo -e "${GREEN}======================================${NC}"
echo -e "Map: ${YELLOW}${MAP_NAME}${NC}"
echo -e "Output Directory: ${YELLOW}${OUTPUT_DIR}${NC}"
echo ""

# Source ROS2 and workspace
echo -e "${YELLOW}Sourcing ROS2 and workspace...${NC}"
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

# Verify map exists
if [ ! -d "${MAP_FOLDER}/${MAP_NAME}" ]; then
    echo -e "${RED}Error: Map directory not found: ${MAP_FOLDER}/${MAP_NAME}${NC}"
    exit 1
fi

# Find the localization_node executable
LOCALIZATION_EXEC="${WORKSPACE_DIR}/install/cartographer_ros/lib/cartographer_ros/localization_node"

if [ ! -f "${LOCALIZATION_EXEC}" ]; then
    echo -e "${RED}Error: localization_node executable not found: ${LOCALIZATION_EXEC}${NC}"
    exit 1
fi

echo -e "${GREEN}Found executable: ${LOCALIZATION_EXEC}${NC}"
echo ""

# Memcheck output file
MEMCHECK_LOG="${OUTPUT_DIR}/memcheck_${TIMESTAMP}.log"

echo -e "${YELLOW}Starting Valgrind Memcheck...${NC}"
echo -e "Log file: ${MEMCHECK_LOG}"
echo ""
echo -e "${RED}Note: This will run VERY slowly. Press Ctrl+C to stop after collecting enough data.${NC}"
echo ""

# Run Valgrind Memcheck with CPU feature restrictions
export VALGRIND_OPTS="--vex-guest-max-insns=50"
export VEX_IROPT_PRECISE_MEMORY_EXNS=1

# Check if suppression file exists
SUPPRESSION_FILE="/opt/ros/humble/share/ros2_tracing/valgrind/python.supp"
SUPPRESSION_ARG=""
if [ -f "${SUPPRESSION_FILE}" ]; then
    SUPPRESSION_ARG="--suppressions=${SUPPRESSION_FILE}"
    echo -e "${BLUE}Using suppression file: ${SUPPRESSION_FILE}${NC}"
else
    echo -e "${YELLOW}Suppression file not found, continuing without it${NC}"
fi
echo ""

valgrind \
    --tool=memcheck \
    --leak-check=full \
    --show-leak-kinds=all \
    --track-origins=yes \
    --verbose \
    --log-file="${MEMCHECK_LOG}" \
    ${SUPPRESSION_ARG} \
    --gen-suppressions=all \
    --num-callers=50 \
    --keep-debuginfo=yes \
    --leak-resolution=high \
    --vex-iropt-register-updates=allregs-at-mem-access \
    "${LOCALIZATION_EXEC}" \
    -configuration_directory "${CONFIG_DIR}" \
    -configuration_basename "dexe_localization_2d.lua" \
    -map_folder "${MAP_FOLDER}" \
    -map_name "${MAP_NAME}" \
    -collect_metrics false \
    --ros-args \
    -r scan_1:=laser_scan/front \
    -r scan_2:=laser_scan/rear \
    -r imu:=imu/chassis/refined \
    -r odom:=feedback/odom \
    -p use_sim_time:=false

echo ""
echo -e "${GREEN}======================================${NC}"
echo -e "${GREEN}Memcheck Complete!${NC}"
echo -e "${GREEN}======================================${NC}"
echo ""
echo -e "Memcheck log: ${YELLOW}${MEMCHECK_LOG}${NC}"
echo ""
echo -e "${YELLOW}To view the leak summary:${NC}"
echo -e "  grep -A 20 'LEAK SUMMARY' ${MEMCHECK_LOG}"
echo ""
echo -e "${YELLOW}To view detailed leak info:${NC}"
echo -e "  grep -B 5 -A 10 'definitely lost\\|indirectly lost\\|possibly lost' ${MEMCHECK_LOG}"
echo ""
echo -e "${YELLOW}To view all errors:${NC}"
echo -e "  cat ${MEMCHECK_LOG} | less"
echo ""
