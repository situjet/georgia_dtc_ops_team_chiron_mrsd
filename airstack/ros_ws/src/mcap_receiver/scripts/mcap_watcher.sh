#!/bin/bash
# mcap_watcher.sh - Improved MCAP file monitoring script
# Validates file integrity first, outputs only complete file paths

set -euo pipefail

# Load configuration
CONFIG_FILE="$(dirname "$0")/../config/simple_config.env"
if [ -f "$CONFIG_FILE" ]; then
    source "$CONFIG_FILE"
else
    echo "ERROR: Configuration file not found: $CONFIG_FILE" >&2
    exit 1
fi

# Configuration parameters (with fallbacks)
WATCH_DIR="${LOCAL_MCAP_DIR:-/home/lance/mcap_received}"
PROCESSED_DIR="${PROCESSED_DIR:-${WATCH_DIR}/.processed}"

# Create processing marker directory
mkdir -p "$PROCESSED_DIR"

# Check if directory exists and has files
if [ ! -d "$WATCH_DIR" ]; then
    exit 0  # Directory doesn't exist, exit silently
fi

# Scan for new files, handle "empty directory" case
shopt -s nullglob  # Return empty when wildcards don't match, not literal value

# Look for both .db3 (rosbag) and .mcap.gz files
for pattern in "*.db3" "*.mcap.gz" "*.mcap"; do
    for file in "$WATCH_DIR"/$pattern; do
        # If no matching files, loop won't execute
        [ -f "$file" ] || continue
        
        basename=$(basename "$file")
        marker_file="$PROCESSED_DIR/${basename}.done"
        
        # Check if already processed
        if [ -f "$marker_file" ]; then
            continue
        fi
        
        # Validate file integrity based on type
        file_valid=false
        if [[ "$file" == *.gz ]]; then
            # For gzip files, test gzip integrity
            if gzip -t "$file" 2>/dev/null; then
                file_valid=true
            else
                echo "$(date): Corrupted gzip file: $file" >&2
            fi
        elif [[ "$file" == *.db3 ]]; then
            # For SQLite files, check if file is readable and not empty
            if [ -s "$file" ] && file "$file" | grep -q "SQLite"; then
                file_valid=true
            else
                echo "$(date): Invalid or corrupted db3 file: $file" >&2
            fi
        elif [[ "$file" == *.mcap ]]; then
            # For MCAP files, check if file is readable and not empty
            if [ -s "$file" ]; then
                file_valid=true
            else
                echo "$(date): Invalid or empty mcap file: $file" >&2
            fi
        fi
        
        # Output valid files for processing
        if [ "$file_valid" = true ]; then
            echo "$file"
        fi
    done
done

