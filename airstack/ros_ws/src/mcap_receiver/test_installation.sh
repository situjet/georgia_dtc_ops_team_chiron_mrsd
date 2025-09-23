#!/bin/bash
# Test mcap_receiver installation

set -euo pipefail

echo "=== MCAP Receiver Installation Test ==="

# Enter ROS workspace
cd "$(dirname "$0")/../../.."
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "1. Check package installation..."
ros2 pkg list | grep mcap_receiver && echo "✅ mcap_receiver package installed" || echo "❌ mcap_receiver package not found"
ros2 pkg list | grep dell_publisher && echo "✅ dell_publisher package installed" || echo "❌ dell_publisher package not found"

echo ""
echo "2. Check executable files..."
echo "mcap_receiver nodes:"
ros2 pkg executables mcap_receiver

echo ""
echo "dell_publisher nodes:"
ros2 pkg executables dell_publisher

echo ""
echo "3. Test configuration file..."
CONFIG_FILE="src/mcap_receiver/config/simple_config.env"
if [ -f "$CONFIG_FILE" ]; then
    echo "✅ Configuration file exists: $CONFIG_FILE"
    echo "Configuration content:"
    head -5 "$CONFIG_FILE"
else
    echo "❌ Configuration file not found"
fi

echo ""
echo "4. Test script permissions..."
SCRIPTS_DIR="src/mcap_receiver/scripts"
for script in mcap_sync.sh mcap_watcher.sh simple_main_loop.sh; do
    if [ -x "$SCRIPTS_DIR/$script" ]; then
        echo "✅ $script executable"
    else
        echo "❌ $script not executable"
    fi
done

echo ""
echo "5. Create test MCAP file..."
TEST_DIR="/tmp/mcap_test_$(date +%s)"
mkdir -p "$TEST_DIR"
echo "MCAP_TEST_DATA_FOR_INSTALLATION_TEST" | gzip > "$TEST_DIR/test.mcap.gz"

echo "Testing simple_mcap_processor..."
if ros2 run mcap_receiver simple_mcap_processor "$TEST_DIR/test.mcap.gz" 2>/dev/null; then
    echo "✅ simple_mcap_processor can run"
else
    echo "⚠️  simple_mcap_processor has issues (possibly ROS environment)"
fi

# Cleanup
rm -rf "$TEST_DIR"

echo ""
echo "=== Installation Test Complete ==="
echo ""
echo "Next steps:"
echo "1. Configure SSH keys to remote machine"
echo "2. Update IP addresses in config/simple_config.env"
echo "3. Run: ros2 run mcap_receiver mcap_receiver_node"
