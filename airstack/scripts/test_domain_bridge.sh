#!/bin/bash

# ROS Domain Bridge 测试脚本
# 测试从域70到域100的话题桥接是否正常工作

set -e

echo "=========================================="
echo "ROS域桥接器测试脚本"
echo "测试域70 -> 域100的话题桥接"
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

# 切换到ROS工作空间
cd "$ROS_WS_DIR"

# Source ROS环境
echo "设置ROS环境..."
source /opt/ros/humble/setup.bash
source install/setup.bash

echo "=========================================="
echo "测试1: 检查域70中的话题"
echo "=========================================="

export ROS_DOMAIN_ID=70
echo "当前域ID: $ROS_DOMAIN_ID"
echo "域70中的话题列表:"
timeout 5 ros2 topic list || echo "域70中没有检测到话题或超时"

echo ""
echo "=========================================="
echo "测试2: 检查域100中的话题"
echo "=========================================="

export ROS_DOMAIN_ID=100
echo "当前域ID: $ROS_DOMAIN_ID"
echo "域100中的话题列表:"
timeout 5 ros2 topic list || echo "域100中没有检测到话题或超时"

echo ""
echo "=========================================="
echo "测试3: 发布测试消息到域70"
echo "=========================================="

export ROS_DOMAIN_ID=70
echo "在域70中发布测试消息到 /gimbal/teleop..."

# 在后台发布测试消息
ros2 topic pub --once /gimbal/teleop geometry_msgs/msg/Vector3 "{x: 1.0, y: 2.0, z: 3.0}" &
PUB_PID=$!

sleep 2

echo ""
echo "=========================================="
echo "测试4: 检查域100中是否收到桥接的消息"
echo "=========================================="

export ROS_DOMAIN_ID=100
echo "在域100中监听 /gimbal/teleop 话题..."

# 尝试接收桥接的消息
timeout 5 ros2 topic echo /gimbal/teleop geometry_msgs/msg/Vector3 --once || echo "未在域100中检测到桥接的消息"

# 清理后台进程
kill $PUB_PID 2>/dev/null || true

echo ""
echo "=========================================="
echo "测试完成"
echo "=========================================="
echo "如果看到桥接的消息，说明域桥接器工作正常。"
echo "如果没有看到消息，请检查："
echo "1. 域桥接器是否正在运行"
echo "2. 配置文件是否正确"
echo "3. 网络连接是否正常"
