#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd /root/ros_ws
source install/setup.bash || true
# Launch robot bringup (starts mavros and domain_bridge)
ros2 launch robot_bringup robot.launch.xml
