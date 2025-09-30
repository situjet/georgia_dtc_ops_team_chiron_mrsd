#!/bin/bash
set -euo pipefail

# Load configuration
CONFIG_FILE="$(dirname "$0")/../config/simple_config.env"
if [ -f "$CONFIG_FILE" ]; then
    source "$CONFIG_FILE"
else
    echo "ERROR: Configuration file not found: $CONFIG_FILE"
    exit 1
fi

# Ensure local directories exist
mkdir -p "$LOCAL_MCAP_DIR"
mkdir -p "$PARTIAL_DIR"

echo "Starting MCAP synchronization from ${REMOTE_USER}@${REMOTE_IP}:${REMOTE_MCAP_DIR} to ${LOCAL_MCAP_DIR}"

# Rsync command with radio-friendly options and sshpass
# --partial-dir: ensures incomplete files are stored in a separate directory
# --timeout: increases I/O timeout for unstable connections
# -e "sshpass + ssh ...": SSH options for keep-alive and connection stability
rsync -av --progress \
      --partial --partial-dir="$PARTIAL_DIR" \
      --timeout=60 \
      -e "sshpass -p 'passme24' ssh -o ConnectTimeout=10 -o ServerAliveInterval=10 -o ServerAliveCountMax=3 -o StrictHostKeyChecking=no" \
      "${REMOTE_USER}@${REMOTE_IP}:${REMOTE_MCAP_DIR}" \
      "$LOCAL_MCAP_DIR"

echo "MCAP synchronization completed."

