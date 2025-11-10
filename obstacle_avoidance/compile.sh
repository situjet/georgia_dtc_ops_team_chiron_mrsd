#!/bin/bash
# This script compiles the path planner application.
# It's designed to be run from any directory.

# Stop on any error
set -e

# Get the directory where the script is located
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

echo "Compiling in directory: $SCRIPT_DIR"

# Compile the main.cpp file, using absolute paths
g++ -std=c++11 "$SCRIPT_DIR/main.cpp" -o "$SCRIPT_DIR/path_planner" \
-I/opt/homebrew/include/opencv4 \
-L/opt/homebrew/lib \
-lopencv_core \
-lopencv_highgui \
-lopencv_imgcodecs \
-lopencv_imgproc \
-lopencv_videoio

echo "Compilation successful. Run from the project root with:"
echo "cd '$SCRIPT_DIR' && ./path_planner"

"$SCRIPT_DIR/path_planner"