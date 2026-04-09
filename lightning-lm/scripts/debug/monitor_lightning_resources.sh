#!/bin/bash
# Monitor Lightning 3D SLAM / localization node CPU and memory usage
# Records data every 30 seconds and saves to CSV file
#
# Usage:
#   monitor_lightning_resources.sh [PATTERN] [INTERVAL]
#
# PATTERN examples:
#   /run_slam_online    - 在线建图（默认）
#   /run_loc_online     - 在线定位
#   /run_slam_offline   - 离线建图
#   /run_loc_offline    - 离线定位

# Parse command line arguments
PATTERN="${1:-/run_slam_online}"   # 默认监控在线建图节点
INTERVAL="${2:-30}"                # 默认采样间隔 30 秒

# Configuration
OUTPUT_DIR="$HOME/.dexe/lightning/assessment"
OUTPUT_FILE="$OUTPUT_DIR/lightning_resources.csv"

# CPU tick frequency (Hz), used for instantaneous CPU calculation
CLK_TCK=$(getconf CLK_TCK 2>/dev/null || echo 100)

# Create output directory if it doesn't exist
mkdir -p "$OUTPUT_DIR"

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Print usage information
print_usage() {
    echo "Usage: $0 [PATTERN] [INTERVAL]"
    echo ""
    echo "Arguments:"
    echo "  PATTERN   进程匹配模式 (default: /run_slam_online)"
    echo "  INTERVAL  采样间隔，单位秒 (default: 30)"
    echo ""
    echo "Examples:"
    echo "  $0                                 # 监控在线建图节点，每 30s 采样"
    echo "  $0 /run_loc_online                 # 监控在线定位节点"
    echo "  $0 /run_loc_online 10              # 监控在线定位节点，每 10s 采样"
    echo "  $0 /run_slam_offline               # 监控离线建图节点"
    echo "  $0 /run_loc_offline                # 监控离线定位节点"
    echo ""
    echo "  # 通过 ros2 run 调用："
    echo "  ros2 run lightning monitor_lightning_resources.sh /run_slam_online"
    echo "  ros2 run lightning monitor_lightning_resources.sh /run_loc_online 10"
    echo ""
}

# Check for help flag
if [ "$1" = "-h" ] || [ "$1" = "--help" ]; then
    print_usage
    exit 0
fi

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "Monitoring stopped."
    echo "Data saved to: $OUTPUT_FILE"

    # Automatically run visualization script
    echo ""
    echo "Generating visualizations..."
    VISUALIZE_SCRIPT="$SCRIPT_DIR/visualize_lightning_resources.py"

    if [ -f "$VISUALIZE_SCRIPT" ]; then
        python3 "$VISUALIZE_SCRIPT" "$OUTPUT_FILE"
    else
        echo "Warning: Visualization script not found at $VISUALIZE_SCRIPT"
        echo "You can manually run: python3 visualize_lightning_resources.py $OUTPUT_FILE"
    fi

    exit 0
}

# Trap Ctrl+C and other termination signals
trap cleanup SIGINT SIGTERM

# Read cumulative CPU ticks (utime + stime) from /proc/[pid]/stat
# This is the same source htop uses for instantaneous CPU calculation
get_cpu_ticks() {
    awk '{print $14+$15}' /proc/$1/stat 2>/dev/null || echo 0
}

# Find the target process
find_pid() {
    # 在 lib/lightning/ 下查找匹配的可执行文件，排除干扰进程
    PID=$(pgrep -a -f "$PATTERN" \
        | grep -v "grep" \
        | grep -v "monitor_lightning_resources.sh" \
        | grep -v "python3.*ros2 run" \
        | grep "lib/lightning${PATTERN}" \
        | head -1 \
        | awk '{print $1}')

    if [ -z "$PID" ]; then
        echo "Error: No process found matching pattern: lib/lightning${PATTERN}"
        echo "Please make sure the node is running before starting monitoring."
        echo ""
        echo "Available lightning processes:"
        pgrep -a -f "lib/lightning" | grep -v "grep" | grep -v "monitor_lightning_resources.sh" || echo "  None found"
        exit 1
    fi

    PROCESS_CMD=$(ps -p "$PID" -o args= 2>/dev/null)
    echo "Found process matching '${PATTERN}' with PID: $PID"
    echo "Command: $PROCESS_CMD"
}

# Initial PID search
find_pid

# Initialize tick baseline for first interval measurement
TICKS_PREV=$(get_cpu_ticks "$PID")
TIME_PREV_MS=$(date +%s%3N)

# Write CSV header
echo "timestamp,time,cpu_percent,rss_mb,mem_percent" > "$OUTPUT_FILE"
echo ""
echo "=========================================================================="
echo "Lightning Resource Monitoring Started"
echo "=========================================================================="
echo "Process:   $PATTERN"
echo "PID:       $PID"
echo "Interval:  ${INTERVAL}s"
echo "Output:    $OUTPUT_FILE"
echo "=========================================================================="
echo "Press Ctrl+C to stop monitoring and generate visualizations"
echo ""
printf "%-20s %-12s %-10s %-12s %-12s\n" "Timestamp" "Time" "CPU(%)" "RSS(MB)" "MEM(%)"
echo "--------------------------------------------------------------------------------"

# Main monitoring loop
while true; do
    sleep "$INTERVAL"

    # Check if process is still running
    if ! ps -p "$PID" > /dev/null 2>&1; then
        echo "Warning: Process $PID has exited. Searching for new process..."
        find_pid
        TICKS_PREV=$(get_cpu_ticks "$PID")
        TIME_PREV_MS=$(date +%s%3N)
        continue
    fi

    # Get current timestamp
    TIMESTAMP=$(date +%s)
    TIME=$(date +%H:%M:%S)

    # --- Instantaneous CPU (same algorithm as htop) ---
    # delta_ticks / (delta_seconds * CLK_TCK) * 100
    TICKS_NOW=$(get_cpu_ticks "$PID")
    TIME_NOW_MS=$(date +%s%3N)
    DELTA_TICKS=$((TICKS_NOW - TICKS_PREV))
    DELTA_MS=$((TIME_NOW_MS - TIME_PREV_MS))
    CPU=$(awk "BEGIN {printf \"%.2f\", ($DELTA_TICKS / ($DELTA_MS / 1000.0 * $CLK_TCK)) * 100}")
    TICKS_PREV=$TICKS_NOW
    TIME_PREV_MS=$TIME_NOW_MS

    # --- RSS and MEM (instantaneous, from ps) ---
    read -r RSS MEM < <(ps -p "$PID" -o rss=,%mem= 2>/dev/null \
        | awk '{printf "%.2f %.2f", $1/1024, $2}')

    # Check if we got valid data
    if [ -n "$CPU" ] && [ -n "$RSS" ]; then
        printf "%-20s %-12s %-10s %-12s %-12s\n" "$TIMESTAMP" "$TIME" "$CPU" "$RSS" "$MEM"
        echo "$TIMESTAMP,$TIME,$CPU,$RSS,$MEM" >> "$OUTPUT_FILE"
        COUNT=$((COUNT + 1))
    else
        echo "Warning: Failed to get resource data for PID $PID"
    fi
done
