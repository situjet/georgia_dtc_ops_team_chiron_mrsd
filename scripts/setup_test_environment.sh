#!/bin/bash
# Setup Test Environment for Vision GPS Estimator
# ===============================================

set -e

echo "Setting up Vision GPS Estimator test environment..."

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${YELLOW}Warning: ROS2 environment not detected. Please source ROS2 setup.bash first:${NC}"
    echo "source /opt/ros/humble/setup.bash  # or your ROS2 distribution"
    exit 1
fi

echo -e "${GREEN}✓ ROS2 environment detected: $ROS_DISTRO${NC}"

# Project paths
PROJECT_ROOT="/home/yliu/Documents/github/georgia_dtc_ops_team_chiron_mrsd"
AIRSTACK_WS="$PROJECT_ROOT/airstack/ros_ws"
VISION_GPS_PKG="$AIRSTACK_WS/src/vision_gps_estimator"
MCAP_DATA_DIR="$PROJECT_ROOT/mil19"

# Check required directories
echo "Checking required directories..."

if [ ! -d "$MCAP_DATA_DIR" ]; then
    echo -e "${RED}✗ MCAP data directory not found: $MCAP_DATA_DIR${NC}"
    exit 1
fi
echo -e "${GREEN}✓ MCAP data directory found${NC}"

if [ ! -d "$VISION_GPS_PKG" ]; then
    echo -e "${RED}✗ Vision GPS package not found: $VISION_GPS_PKG${NC}"
    exit 1
fi
echo -e "${GREEN}✓ Vision GPS package found${NC}"

# Create models directory if it doesn't exist
MODELS_DIR="$VISION_GPS_PKG/models"
mkdir -p "$MODELS_DIR"
echo -e "${GREEN}✓ Models directory ready: $MODELS_DIR${NC}"

# Check for YOLO model
YOLO_MODEL="$VISION_GPS_PKG/yolo12s.pt"
if [ ! -f "$YOLO_MODEL" ]; then
    echo -e "${YELLOW}Warning: YOLO model not found at $YOLO_MODEL${NC}"
    echo "Please copy your yolo12s.pt model to:"
    echo "  $YOLO_MODEL"
    echo "Or update the model_path in config/params.yaml"
fi

# Check MCAP files
echo "Checking MCAP data files..."
MCAP_COUNT=$(find "$MCAP_DATA_DIR" -name "*.mcap" | wc -l)
if [ "$MCAP_COUNT" -eq 0 ]; then
    echo -e "${RED}✗ No MCAP files found in $MCAP_DATA_DIR${NC}"
    exit 1
fi
echo -e "${GREEN}✓ Found $MCAP_COUNT MCAP files${NC}"

# List available MCAP files
echo "Available MCAP datasets:"
for dataset in "$MCAP_DATA_DIR"/*/; do
    if [ -d "$dataset/recording" ]; then
        dataset_name=$(basename "$dataset")
        mcap_files=$(find "$dataset/recording" -name "*.mcap" | wc -l)
        echo "  - $dataset_name ($mcap_files files)"
    fi
done

# Check ROS2 dependencies
echo "Checking ROS2 dependencies..."

# Required packages
REQUIRED_PACKAGES=(
    "cv_bridge"
    "vision_msgs" 
    "mavros_msgs"
    "rosbag2_py"
    "tf2_ros"
)

for pkg in "${REQUIRED_PACKAGES[@]}"; do
    if ros2 pkg list | grep -q "^$pkg$"; then
        echo -e "${GREEN}✓ $pkg${NC}"
    else
        echo -e "${RED}✗ $pkg (missing)${NC}"
        echo "  Install with: sudo apt install ros-$ROS_DISTRO-${pkg//_/-}"
    fi
done

# Check Python dependencies
echo "Checking Python dependencies..."

PYTHON_PACKAGES=(
    "numpy"
    "opencv-python:cv2"
    "scipy"
    "ultralytics"
)

for pkg_spec in "${PYTHON_PACKAGES[@]}"; do
    pkg_name="${pkg_spec%%:*}"
    import_name="${pkg_spec##*:}"
    if [ "$import_name" = "$pkg_name" ]; then
        import_name="$pkg_name"
    fi
    
    if python3 -c "import $import_name" 2>/dev/null; then
        echo -e "${GREEN}✓ $pkg_name${NC}"
    else
        echo -e "${RED}✗ $pkg_name (missing)${NC}"
        echo "  Install with: pip install $pkg_name"
    fi
done

# Check GStreamer
echo "Checking GStreamer..."
if command -v gst-launch-1.0 &> /dev/null; then
    echo -e "${GREEN}✓ GStreamer tools${NC}"
else
    echo -e "${RED}✗ GStreamer tools (missing)${NC}"
    echo "  Install with: sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-*"
fi

# Build the package
echo "Building vision_gps_estimator package..."
cd "$AIRSTACK_WS"

if colcon build --packages-select vision_gps_estimator; then
    echo -e "${GREEN}✓ Package built successfully${NC}"
else
    echo -e "${RED}✗ Package build failed${NC}"
    exit 1
fi

# Source the workspace
source "$AIRSTACK_WS/install/setup.bash"

echo -e "${GREEN}🎉 Test environment setup complete!${NC}"
echo ""
echo "Next steps:"
echo "1. Copy your yolo12s.pt model to: $YOLO_MODEL"
echo "2. Run unit tests: cd $VISION_GPS_PKG && python3 -m pytest test/"
echo "3. Test MCAP info: ros2 bag info $MCAP_DATA_DIR/2025-05-08_00-12-29/recording/recording_0.mcap"
echo "4. Launch the system: ros2 launch vision_gps_estimator vision_gps_estimator.launch.py"
echo ""
echo "Environment variables:"
echo "  PROJECT_ROOT=$PROJECT_ROOT"
echo "  MCAP_DATA_DIR=$MCAP_DATA_DIR"
echo "  VISION_GPS_PKG=$VISION_GPS_PKG"
