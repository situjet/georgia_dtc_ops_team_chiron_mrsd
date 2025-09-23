#!/bin/bash
# mcap_watcher.sh - Improved MCAP file monitoring script
# Validates gzip integrity first, outputs only complete file paths

set -euo pipefail

# Configuration parameters
WATCH_DIR="${WATCH_DIR:-/home/lance/mcap_received}"
PROCESSED_DIR="${PROCESSED_DIR:-/home/lance/mcap_received/.processed}"

# Create processing marker directory
mkdir -p "$PROCESSED_DIR"

# Check if directory exists and has files
if [ ! -d "$WATCH_DIR" ]; then
    exit 0  # Directory doesn't exist, exit silently
fi

# Scan for new files, handle "empty directory" case
shopt -s nullglob  # Return empty when wildcards don't match, not literal value
for file in "$WATCH_DIR"/*.mcap.gz; do
    # If no matching files, loop won't execute
    [ -f "$file" ] || continue
    
    basename=$(basename "$file")
    marker_file="$PROCESSED_DIR/${basename}.done"
    
    # Check if already processed
    if [ -f "$marker_file" ]; then
        continue
    fi
    
    # Key improvement: validate gzip integrity first
    if gzip -t "$file" 2>/dev/null; then
        # Output only the path itself, for main_loop.sh to consume directly
        echo "$file"
    else
        # gzip corrupted, log but don't output
        echo "$(date): Corrupted gzip file: $file" >&2
    fi
done

