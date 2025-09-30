#!/bin/bash

# ROS Domain Bridge 70 to 100 启动脚本
# 此脚本启动从域70到域100的ROS域桥接器

set -e

echo "=========================================="
echo "启动ROS域桥接器 (域70 -> 域100)"
echo "=========================================="

# 检查ROS环境
if [ -z "$ROS_DISTRO" ]; then
    echo "错误: ROS环境未设置。请先source ROS setup文件。"
    exit 1
fi

# 设置工作目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS_DIR="$(dirname "$SCRIPT_DIR")/ros_ws"

echo "ROS工作空间: $ROS_WS_DIR"

# 检查配置文件是否存在
CONFIG_FILE="$ROS_WS_DIR/src/robot_bringup/params/domain_bridge_70_to_100.yaml"
if [ ! -f "$CONFIG_FILE" ]; then
    echo "错误: 配置文件不存在: $CONFIG_FILE"
    exit 1
fi

echo "使用配置文件: $CONFIG_FILE"

# 检查launch文件是否存在
LAUNCH_FILE="$ROS_WS_DIR/src/robot_bringup/launch/domain_bridge_70_to_100.launch.xml"
if [ ! -f "$LAUNCH_FILE" ]; then
    echo "错误: Launch文件不存在: $LAUNCH_FILE"
    exit 1
fi

echo "使用Launch文件: $LAUNCH_FILE"

# 切换到ROS工作空间
cd "$ROS_WS_DIR"

# Source ROS环境
echo "设置ROS环境..."
source /opt/ros/humble/setup.bash
source install/setup.bash

# 设置默认域ID为70（桥接器需要能访问源域）
export ROS_DOMAIN_ID=70
echo "设置ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

# 显示当前ROS配置
echo "当前ROS配置:"
echo "  ROS_DISTRO: $ROS_DISTRO"
echo "  ROS_DOMAIN_ID: $ROS_DOMAIN_ID"

# 启动域桥接器
echo "=========================================="
echo "启动域桥接器..."
echo "=========================================="

# 使用ros2 launch启动
ros2 launch robot_bringup domain_bridge_70_to_100.launch.xml

# 或者直接使用domain_bridge命令（备选方案）
# ros2 run domain_bridge domain_bridge "$CONFIG_FILE"
