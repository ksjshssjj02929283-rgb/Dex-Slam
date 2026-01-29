#!/bin/bash

# Combined Valgrind Analysis Script for Mapping/SLAM
# Usage: ./analyze_mapping_memory.sh [output_map_name] [duration_seconds]

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# Get the workspace directory dynamically
# When installed via ros2, the script is in install/cartographer_ros/lib/cartographer_ros/
# We need to go up to the workspace root (from install/... -> workspace root is 4 levels up)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
if [[ "$SCRIPT_DIR" == *"/install/cartographer_ros/lib/cartographer_ros"* ]]; then
    # Running from installed location (ros2 run)
    # From: <workspace>/install/cartographer_ros/lib/cartographer_ros
    # Up 4 levels => <workspace>
    WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../../../" && pwd)"
else
    # Running from source
    WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../../../../.." && pwd)"
fi

OUTPUT_MAP_NAME="${1:-test_mapping}"
DURATION="${2:-60}"  # Default 60 seconds for mapping (needs more time than localization)

echo -e "${GREEN}======================================${NC}"
echo -e "${GREEN}Mapping Node Memory Analysis${NC}"
echo -e "${GREEN}======================================${NC}"
echo ""
echo -e "${BLUE}This script will:${NC}"
echo -e "  1. Run SLAM node for ${DURATION} seconds with Massif profiling"
echo -e "  2. Collect memory usage data during map building"
echo -e "  3. Generate analysis reports"
echo -e "  4. Save map to: ~/.dexe/data/maps/${OUTPUT_MAP_NAME}"
echo ""
echo -e "${YELLOW}Note: You need to have sensor data (laser scans, odom) available.${NC}"
echo -e "${YELLOW}      Either play a rosbag or have the robot running.${NC}"
echo ""
read -p "Press Enter to start or Ctrl+C to cancel..."

# Run with timeout
# Prefer the debug script located next to this script (works both for installed and source runs).
timeout ${DURATION}s bash "${SCRIPT_DIR}/debug_mapping_memory.sh" "${OUTPUT_MAP_NAME}" || true

echo ""
echo -e "${GREEN}Analysis complete! Check the valgrind_output directory.${NC}"
echo -e "${GREEN}Map saved to: ~/.dexe/data/maps/${OUTPUT_MAP_NAME}${NC}"
