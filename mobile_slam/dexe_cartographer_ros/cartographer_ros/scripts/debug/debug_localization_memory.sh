#!/bin/bash

# Valgrind Massif Memory Profiling Script for Localization Node
# Usage: ./debug_localization_memory.sh [map_name]

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Get the workspace directory dynamically
# When installed via ros2, the script is in install/cartographer_ros/lib/cartographer_ros/
# We need to go up to the workspace root (install/../..)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "$SCRIPT_DIR" == *"/install/cartographer_ros/lib/cartographer_ros"* ]]; then
    # Running from installed location (ros2 run)
    WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../../../.." && pwd)"
else
    # Running from source
    WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../../../../.." && pwd)"
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
echo -e "${GREEN}Valgrind Massif Memory Profiling${NC}"
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

if [ ! -f "${MAP_FOLDER}/${MAP_NAME}/map.pbstream" ]; then
    echo -e "${RED}Error: Map pbstream file not found: ${MAP_FOLDER}/${MAP_NAME}/map.pbstream${NC}"
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

# Massif output file
MASSIF_OUT="${OUTPUT_DIR}/massif.out.${TIMESTAMP}"
MASSIF_REPORT="${OUTPUT_DIR}/massif_report_${TIMESTAMP}.txt"

echo -e "${YELLOW}Starting Valgrind Massif profiling...${NC}"
echo -e "Output file: ${MASSIF_OUT}"
echo -e "Report file: ${MASSIF_REPORT}"
echo ""
echo -e "${RED}Note: This will run VERY slowly. Press Ctrl+C to stop after collecting enough data.${NC}"
echo ""

# Run Valgrind Massif with CPU feature restrictions
# Use --vex-guest-max-insns to avoid AVX-512 issues
export VALGRIND_OPTS="--vex-guest-max-insns=50"
export VEX_IROPT_PRECISE_MEMORY_EXNS=1

valgrind \
    --tool=massif \
    --time-unit=ms \
    --massif-out-file="${MASSIF_OUT}" \
    --heap=yes \
    --heap-admin=8 \
    --stacks=no \
    --depth=50 \
    --alloc-fn=g_malloc \
    --alloc-fn=g_realloc \
    --alloc-fn=g_try_malloc \
    --alloc-fn=g_malloc0 \
    --alloc-fn=g_mem_chunk_alloc \
    --threshold=0.1 \
    --peak-inaccuracy=0.1 \
    --detailed-freq=10 \
    --max-snapshots=1000 \
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

# Generate human-readable report
echo ""
echo -e "${GREEN}Generating Massif report...${NC}"
ms_print "${MASSIF_OUT}" > "${MASSIF_REPORT}"

echo ""
echo -e "${GREEN}======================================${NC}"
echo -e "${GREEN}Profiling Complete!${NC}"
echo -e "${GREEN}======================================${NC}"
echo ""
echo -e "Massif output: ${YELLOW}${MASSIF_OUT}${NC}"
echo -e "Readable report: ${YELLOW}${MASSIF_REPORT}${NC}"
echo ""
echo -e "${YELLOW}To view the report:${NC}"
echo -e "  cat ${MASSIF_REPORT}"
echo -e "  less ${MASSIF_REPORT}"
echo ""
echo -e "${YELLOW}To visualize with massif-visualizer (if installed):${NC}"
echo -e "  massif-visualizer ${MASSIF_OUT}"
echo ""
echo -e "${YELLOW}Key sections to check in the report:${NC}"
echo -e "  - Peak memory usage (look for 'KB' values)"
echo -e "  - Memory allocation trends over time"
echo -e "  - Top allocators (functions/call stacks using most memory)"
echo ""
