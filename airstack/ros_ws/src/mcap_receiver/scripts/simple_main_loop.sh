#!/bin/bash
# simple_main_loop.sh - Simplified main loop
# Designed for single-file single-target scenarios

set -euo pipefail

# Script directory
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Configuration parameters
SYNC_INTERVAL="${SYNC_INTERVAL:-30}"
LOG_FILE="${LOG_FILE:-$HOME/.local/log/simple_mcap_receiver.log}"

# Create log directory
mkdir -p "$(dirname "$LOG_FILE")"

# Load ROS environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "$(date): Loaded ROS 2 Humble environment" >> "$LOG_FILE"
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
    echo "$(date): Loaded ROS 2 Foxy environment" >> "$LOG_FILE"
else
    echo "$(date): WARNING: No ROS 2 environment found" >> "$LOG_FILE"
fi

# Load local workspace (if exists)
if [ -f "$SCRIPT_DIR/../../../../install/setup.bash" ]; then
    source "$SCRIPT_DIR/../../../../install/setup.bash"
    echo "$(date): Loaded local workspace" >> "$LOG_FILE"
fi

echo "$(date): Starting simple MCAP receiver main loop" >> "$LOG_FILE"

# Main loop
while true; do
    echo "$(date): Starting sync cycle" >> "$LOG_FILE"
    
    # 1. Synchronize files
    if "$SCRIPT_DIR/mcap_sync.sh"; then
        echo "$(date): Sync completed successfully" >> "$LOG_FILE"
    else
        echo "$(date): Sync failed, continuing..." >> "$LOG_FILE"
    fi
    
    # 2. Detect new files and process (one at a time)
    while IFS= read -r file_path; do
        if [ -n "$file_path" ]; then
            echo "$(date): Processing new file: $file_path" >> "$LOG_FILE"
            
            # Use ROS node processor
            if ros2 run mcap_receiver simple_mcap_processor "$file_path"; then
                echo "$(date): Successfully processed: $file_path" >> "$LOG_FILE"
            else
                echo "$(date): Failed to process: $file_path" >> "$LOG_FILE"
            fi
            
            # Process only one file at a time, continue after completion
            break
        fi
    done < <("$SCRIPT_DIR/mcap_watcher.sh")
    
    # 3. Wait for next cycle
    echo "$(date): Cycle completed, sleeping for ${SYNC_INTERVAL}s" >> "$LOG_FILE"
    sleep "$SYNC_INTERVAL"
done

