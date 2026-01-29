#!/bin/bash

# Combined Valgrind Analysis Script
# Usage: ./analyze_localization_memory.sh [map_name] [duration_seconds]

set -e

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

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

MAP_NAME="${1:-map_2}"
DURATION="${2:-20}"  # Default 20 seconds

echo -e "${GREEN}======================================${NC}"
echo -e "${GREEN}Localization Node Memory Analysis${NC}"
echo -e "${GREEN}======================================${NC}"
echo ""
echo -e "${BLUE}This script will:${NC}"
echo -e "  1. Run node for ${DURATION} seconds with Massif profiling"
echo -e "  2. Collect memory usage data"
echo -e "  3. Generate analysis reports"
echo ""
read -p "Press Enter to start or Ctrl+C to cancel..."

# Run with timeout
timeout ${DURATION}s bash "${WORKSPACE_DIR}/debug_localization_memory.sh" "${MAP_NAME}" || true

echo ""
echo -e "${GREEN}Analysis complete! Check the valgrind_output directory.${NC}"
