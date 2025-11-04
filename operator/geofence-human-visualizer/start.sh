#!/bin/bash
# Start script: publish test data, start bridge, build extension

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Source ROS2 environment (use system Python, not conda)
if [ -f /opt/ros/humble/setup.bash ]; then
    if [ -n "$CONDA_DEFAULT_ENV" ]; then
        OLD_CONDA_DEFAULT_ENV="$CONDA_DEFAULT_ENV"
        OLD_CONDA_PREFIX="$CONDA_PREFIX"
        OLD_PATH="$PATH"
        export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v conda | tr '\n' ':')
        unset CONDA_DEFAULT_ENV
        unset CONDA_PREFIX
    fi
    source /opt/ros/humble/setup.bash
fi

if ! command -v ros2 &> /dev/null; then
    echo "Error: ROS2 environment not loaded"
    exit 1
fi

# Start GPS publisher
echo "Starting GPS publisher..."
bash publish_target_gps.sh > /dev/null 2>&1 &
GPS_SCRIPT_PID=$!
sleep 1

# Stop existing bridge processes
echo "Stopping existing bridge processes..."
pkill -f "rosbridge" 2>/dev/null || true
pkill -f "foxglove_bridge" 2>/dev/null || true
sleep 2

# Wait for port to be free
for i in {1..5}; do
    if ! netstat -tlnp 2>/dev/null | grep -q ":8765 " && ! ss -tlnp 2>/dev/null | grep -q ":8765 "; then
        break
    fi
    sleep 1
done

# Start foxglove_bridge
echo "Starting foxglove_bridge..."
ros2 run foxglove_bridge foxglove_bridge --port 8765 > /tmp/foxglove_bridge.log 2>&1 &
BRIDGE_PID=$!
sleep 3

if ! ps -p $BRIDGE_PID > /dev/null 2>&1; then
    echo "Warning: foxglove_bridge may have failed, check logs:"
    cat /tmp/foxglove_bridge.log 2>/dev/null | tail -5
fi

# Publish geofence (after bridge starts)
echo "Publishing geofence data..."
bash publish_geofence.sh > /dev/null 2>&1 &
sleep 2

# Build and install extension
echo "Building extension..."
if [ ! -d "node_modules" ]; then
    echo "Installing dependencies..."
    npm install || {
        echo "npm install failed, cleaning and retrying..."
        rm -rf node_modules package-lock.json
        npm install
    }
fi
echo "Building..."
npm run build || {
    echo "Build failed, check errors"
    exit 1
}
echo "Installing to Foxglove..."
npm run local-install

echo "Setup complete"
echo ""
echo "=== Connection Info ==="
echo "foxglove_bridge: ws://localhost:8765 (PID: $BRIDGE_PID)"
echo "GPS publisher: $GPS_SCRIPT_PID"
echo ""
echo "=== Connect in Foxglove Studio ==="
echo "1. Click 'Open connection'"
echo "2. Select 'Foxglove WebSocket'"
echo "3. URL: ws://localhost:8765"
echo "4. Click 'Open'"
echo ""
echo "Press Ctrl+C to stop"

cleanup() {
    kill $GPS_SCRIPT_PID $BRIDGE_PID 2>/dev/null || true
    pkill -P $GPS_SCRIPT_PID 2>/dev/null || true
    if [ -n "$OLD_CONDA_DEFAULT_ENV" ]; then
        export CONDA_DEFAULT_ENV="$OLD_CONDA_DEFAULT_ENV"
        export CONDA_PREFIX="$OLD_CONDA_PREFIX"
        export PATH="$OLD_PATH"
    fi
    exit 0
}

trap cleanup INT TERM

while true; do
    sleep 1
done
