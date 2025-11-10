#!/bin/bash
# Activation script for Vision GPS Estimator environment
# =====================================================

echo "🔄 Activating Vision GPS Estimator environment..."

# Activate conda environment
source $(conda info --base)/etc/profile.d/conda.sh
conda activate vision_gps_estimator

# Source ROS2 (if available)
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
    echo "✓ ROS2 Humble sourced"
elif [ -f "/opt/ros/galactic/setup.bash" ]; then
    source /opt/ros/galactic/setup.bash
    echo "✓ ROS2 Galactic sourced"
elif [ -f "/opt/ros/foxy/setup.bash" ]; then
    source /opt/ros/foxy/setup.bash
    echo "✓ ROS2 Foxy sourced"
else
    echo "⚠️  ROS2 not found in standard locations"
fi

# Source workspace if built
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)"
if [ -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    source "$WORKSPACE_DIR/install/setup.bash"
    echo "✓ Workspace sourced"
fi

echo "✅ Environment ready!"
echo ""
echo "Available commands:"
echo "  - Test setup: python3 scripts/test_setup.py"
echo "  - Analyze MCAP: python3 scripts/analyze_mcap_data.py"
echo "  - Launch system: ros2 launch vision_gps_estimator vision_gps_estimator.launch.py"
echo ""
echo "To deactivate: conda deactivate"
