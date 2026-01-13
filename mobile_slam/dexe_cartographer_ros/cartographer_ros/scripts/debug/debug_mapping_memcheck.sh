#!/bin/bash

# Valgrind Memcheck for Memory Leak Detection in Mapping Node
# Usage: ./debug_mapping_memcheck.sh [map_name]

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
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
MAP_NAME="${1:-map_test_$(date +%Y%m%d_%H%M%S)}"
CONFIG_DIR="${WORKSPACE_DIR}/install/cartographer_ros/share/cartographer_ros/configuration_files"
OUTPUT_DIR="${WORKSPACE_DIR}/valgrind_output"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# Create output directory
mkdir -p "${OUTPUT_DIR}"
mkdir -p "${MAP_FOLDER}"

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Valgrind Memcheck - Mapping Node${NC}"
echo -e "${GREEN}========================================${NC}"
echo -e "Workspace: ${BLUE}${WORKSPACE_DIR}${NC}"
echo -e "Map Name: ${YELLOW}${MAP_NAME}${NC}"
echo -e "Map Folder: ${YELLOW}${MAP_FOLDER}${NC}"
echo -e "Config Dir: ${YELLOW}${CONFIG_DIR}${NC}"
echo -e "Output Directory: ${YELLOW}${OUTPUT_DIR}${NC}"
echo ""

# Source ROS2 and workspace
echo -e "${YELLOW}Sourcing ROS2 and workspace...${NC}"
source /opt/ros/humble/setup.bash
source "${WORKSPACE_DIR}/install/setup.bash"

# Verify config directory exists
if [ ! -d "${CONFIG_DIR}" ]; then
    echo -e "${RED}Error: Config directory not found: ${CONFIG_DIR}${NC}"
    exit 1
fi

# Find the cartographer_node executable (used for mapping)
MAPPING_EXEC="${WORKSPACE_DIR}/install/cartographer_ros/lib/cartographer_ros/cartographer_node"

if [ ! -f "${MAPPING_EXEC}" ]; then
    echo -e "${RED}Error: cartographer_node executable not found: ${MAPPING_EXEC}${NC}"
    echo -e "${YELLOW}Available executables:${NC}"
    ls -la "${WORKSPACE_DIR}/install/cartographer_ros/lib/cartographer_ros/" | grep -E "^-rwx" | awk '{print "  - " $9}'
    exit 1
fi

echo -e "${GREEN}Found executable: ${MAPPING_EXEC}${NC}"
echo ""

# Memcheck output file
MEMCHECK_LOG="${OUTPUT_DIR}/memcheck_mapping_${TIMESTAMP}.log"
SUMMARY_LOG="${OUTPUT_DIR}/memcheck_mapping_${TIMESTAMP}_summary.txt"

echo -e "${YELLOW}Starting Valgrind Memcheck...${NC}"
echo -e "Log file: ${MEMCHECK_LOG}"
echo -e "Summary will be saved to: ${SUMMARY_LOG}"
echo ""
echo -e "${RED}======================================${NC}"
echo -e "${RED}IMPORTANT NOTES:${NC}"
echo -e "${RED}======================================${NC}"
echo -e "1. This will run ${RED}VERY SLOWLY${NC} (10-100x slower than normal)"
echo -e "2. Let it run for at least ${YELLOW}5-10 minutes${NC} to collect meaningful data"
echo -e "3. Drive around or play a rosbag to generate map data"
echo -e "4. Press ${YELLOW}Ctrl+C${NC} to stop after collecting enough data"
echo -e "5. Memory leak detection focuses on:"
echo -e "   - ${BLUE}IKD-Tree allocations${NC}"
echo -e "   - ${BLUE}Submap memory management${NC}"
echo -e "   - ${BLUE}Point cloud storage${NC}"
echo -e "${RED}======================================${NC}"
echo ""

sleep 2

# Run Valgrind Memcheck with CPU feature restrictions
export VALGRIND_OPTS="--vex-guest-max-insns=50"
export VEX_IROPT_PRECISE_MEMORY_EXNS=1

echo -e "${GREEN}Starting mapping node with Valgrind...${NC}"
echo ""

# Check if suppression file exists
SUPPRESSION_FILE="/opt/ros/humble/share/ros2_tracing/valgrind/python.supp"
SUPPRESSION_ARG=""
if [ -f "${SUPPRESSION_FILE}" ]; then
    SUPPRESSION_ARG="--suppressions=${SUPPRESSION_FILE}"
    echo -e "${BLUE}Using suppression file: ${SUPPRESSION_FILE}${NC}"
else
    echo -e "${YELLOW}Suppression file not found, continuing without it${NC}"
fi

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
    --trace-children=yes \
    --child-silent-after-fork=no \
    "${MAPPING_EXEC}" \
    -configuration_directory "${CONFIG_DIR}" \
    -configuration_basename "dexe_slam_2d.lua" \
    -map_folder "${MAP_FOLDER}" \
    -map_name "${MAP_NAME}" \
    -collect_metrics false \
    --ros-args \
    -r scan_1:=laser_scan/front \
    -r scan_2:=laser_scan/rear \
    -r imu:=imu/chassis/refined \
    -r odom:=feedback/odom \
    -p use_sim_time:=false

# Capture exit status
EXIT_STATUS=$?

echo ""
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Memcheck Complete!${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Generate summary
echo -e "${YELLOW}Generating summary...${NC}"
{
    echo "=========================================="
    echo "Valgrind Memcheck Summary - Mapping Node"
    echo "=========================================="
    echo "Timestamp: $(date)"
    echo "Map Name: ${MAP_NAME}"
    echo "Exit Status: ${EXIT_STATUS}"
    echo ""
    echo "=========================================="
    echo "HEAP SUMMARY"
    echo "=========================================="
    grep -A 5 "HEAP SUMMARY" "${MEMCHECK_LOG}" || echo "No heap summary found"
    echo ""
    echo "=========================================="
    echo "LEAK SUMMARY"
    echo "=========================================="
    grep -A 20 "LEAK SUMMARY" "${MEMCHECK_LOG}" || echo "No leak summary found"
    echo ""
    echo "=========================================="
    echo "DEFINITE LEAKS (Top 20)"
    echo "=========================================="
    grep -B 5 -A 10 "definitely lost" "${MEMCHECK_LOG}" | head -100 || echo "No definite leaks found"
    echo ""
    echo "=========================================="
    echo "IKD-TREE RELATED LEAKS"
    echo "=========================================="
    grep -i "ikd.*tree\|STATIC_ROOT_NODE" "${MEMCHECK_LOG}" || echo "No IKD-Tree related issues found"
    echo ""
    echo "=========================================="
    echo "SUBMAP RELATED LEAKS"
    echo "=========================================="
    grep -i "submap\|Submap2D" "${MEMCHECK_LOG}" || echo "No Submap related issues found"
    echo ""
} > "${SUMMARY_LOG}"

echo -e "${GREEN}Summary generated: ${SUMMARY_LOG}${NC}"
echo ""

# Display quick results
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Quick Results:${NC}"
echo -e "${BLUE}========================================${NC}"

if grep -q "definitely lost: 0 bytes in 0 blocks" "${MEMCHECK_LOG}"; then
    echo -e "${GREEN}✓ No definite memory leaks detected!${NC}"
else
    echo -e "${RED}✗ Memory leaks detected!${NC}"
    LEAK_BYTES=$(grep "definitely lost:" "${MEMCHECK_LOG}" | tail -1 | grep -oP '\d+(?= bytes)')
    LEAK_BLOCKS=$(grep "definitely lost:" "${MEMCHECK_LOG}" | tail -1 | grep -oP '\d+(?= blocks)')
    echo -e "  Definitely lost: ${RED}${LEAK_BYTES} bytes in ${LEAK_BLOCKS} blocks${NC}"
fi

if grep -q "indirectly lost: 0 bytes in 0 blocks" "${MEMCHECK_LOG}"; then
    echo -e "${GREEN}✓ No indirect memory leaks detected!${NC}"
else
    echo -e "${YELLOW}⚠ Indirect memory leaks detected${NC}"
    INDIRECT_BYTES=$(grep "indirectly lost:" "${MEMCHECK_LOG}" | tail -1 | grep -oP '\d+(?= bytes)')
    echo -e "  Indirectly lost: ${YELLOW}${INDIRECT_BYTES} bytes${NC}"
fi

echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Output Files:${NC}"
echo -e "${BLUE}========================================${NC}"
echo -e "Full log: ${YELLOW}${MEMCHECK_LOG}${NC}"
echo -e "Summary:  ${YELLOW}${SUMMARY_LOG}${NC}"
echo ""

echo -e "${YELLOW}Useful Commands:${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""
echo -e "${YELLOW}1. View leak summary:${NC}"
echo -e "   cat ${SUMMARY_LOG}"
echo ""
echo -e "${YELLOW}2. View full memcheck log:${NC}"
echo -e "   less ${MEMCHECK_LOG}"
echo ""
echo -e "${YELLOW}3. Search for specific leaks:${NC}"
echo -e "   grep -i 'ikd.*tree' ${MEMCHECK_LOG}"
echo -e "   grep -i 'submap' ${MEMCHECK_LOG}"
echo -e "   grep -i 'STATIC_ROOT_NODE' ${MEMCHECK_LOG}"
echo ""
echo -e "${YELLOW}4. View all definite leaks:${NC}"
echo -e "   grep -B 5 -A 10 'definitely lost' ${MEMCHECK_LOG} | less"
echo ""
echo -e "${YELLOW}5. Compare with previous runs:${NC}"
echo -e "   ls -lht ${OUTPUT_DIR}/memcheck_mapping_*.log"
echo ""

# Check for IKD-Tree specific issues
if grep -qi "STATIC_ROOT_NODE\|ikd.*tree" "${MEMCHECK_LOG}"; then
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}⚠ IKD-Tree Memory Issues Detected!${NC}"
    echo -e "${RED}========================================${NC}"
    echo -e "IKD-Tree related memory issues were found."
    echo -e "Please review: ${YELLOW}${SUMMARY_LOG}${NC}"
    echo -e "For details on the recent IKD-Tree fix, see:"
    echo -e "  ${BLUE}IKD_TREE_MEMORY_LEAK_ANALYSIS.md${NC}"
    echo -e "  ${BLUE}IKD_TREE_MEMORY_FIX_COMPARISON.md${NC}"
    echo ""
fi

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Memcheck session completed!${NC}"
echo -e "${GREEN}========================================${NC}"
