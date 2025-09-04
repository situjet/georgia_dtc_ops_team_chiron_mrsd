#!/bin/bash
set -e
cd /root/ros_ws
colcon build --symlink-install
