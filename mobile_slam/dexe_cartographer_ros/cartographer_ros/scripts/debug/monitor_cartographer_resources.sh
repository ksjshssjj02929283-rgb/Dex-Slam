#!/bin/bash
# Monitor localization node CPU and memory usage
# Records data every 30 seconds and saves to CSV file

# Parse command line arguments
PATTERN="${1:-/localization_node}"  # Use first argument or default to /localization_node
INTERVAL="${2:-30}"  # Use second argument or default to 30 seconds

# Configuration
OUTPUT_DIR="$HOME/.dexe/localization/localization_assessment"
OUTPUT_FILE="$OUTPUT_DIR/localization_resources.csv"

# Create output directory if it doesn't exist
mkdir -p "$OUTPUT_DIR"

# CPU tick frequency (Hz), used for instantaneous CPU calculation
CLK_TCK=$(getconf CLK_TCK 2>/dev/null || echo 100)

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Print usage information
print_usage() {
    echo "Usage: $0 [PATTERN] [INTERVAL]"
    echo ""
    echo "Arguments:"
    echo "  PATTERN   Process pattern to monitor (default: /locaization_node)"
    echo "  INTERVAL  Monitoring interval in seconds (default: 30)"
    echo ""
    echo "Examples:"
    echo "  $0                              # Monitor /locaization_node every 30s"
    echo "  $0 /cartographer_node           # Monitor /cartographer_node every 30s"
    echo "  $0 /cartographer_node 10        # Monitor /cartographer_node every 10s"
    echo "  ros2 run cartographer_ros monitor_cartographer_resources.sh /locaization_node"
    echo "  ros2 run cartographer_ros monitor_cartographer_resources.sh /cartographer_node 10"
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
    VISUALIZE_SCRIPT="$SCRIPT_DIR/visualize_cartographer_resources.py"
    
    if [ -f "$VISUALIZE_SCRIPT" ]; then
        python3 "$VISUALIZE_SCRIPT" "$OUTPUT_FILE"
    else
        echo "Warning: Visualization script not found at $VISUALIZE_SCRIPT"
        echo "You can manually run: visualize_cartographer_resources.py $OUTPUT_FILE"
    fi
    
    exit 0
}

# Trap Ctrl+C and other termination signals
trap cleanup SIGINT SIGTERM

# Read cumulative CPU ticks (utime + stime) from /proc/[pid]/stat
get_cpu_ticks() {
    awk '{print $14+$15}' /proc/$1/stat 2>/dev/null || echo 0
}

# Find the target process
find_pid() {
    # Find all processes matching the pattern, excluding this script and grep itself
    # We look for the actual executable (not bash/python/grep processes)
    PID=$(pgrep -a -f "$PATTERN" | grep -v "grep" | grep -v "monitor_cartographer_resources.sh" | grep -v "python3.*ros2 run" | grep "lib/cartographer_ros${PATTERN}" | head -1 | awk '{print $1}')
    
    if [ -z "$PID" ]; then
        echo "Error: No process found matching pattern: $PATTERN"
        echo "Please make sure the process is running before starting monitoring."
        echo ""
        echo "Available processes matching '$PATTERN':"
        pgrep -a -f "$PATTERN" | grep -v "grep" | grep -v "monitor_cartographer_resources.sh" | grep -v "python3.*ros2 run" || echo "  None found"
        exit 1
    fi
    
    # Get the full command line for verification
    PROCESS_CMD=$(ps -p "$PID" -o args= 2>/dev/null)
    echo "Found process matching '$PATTERN' with PID: $PID"
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
echo "Resource Monitoring Started"
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

# Counter for monitoring
COUNT=0

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
