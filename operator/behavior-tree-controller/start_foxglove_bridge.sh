#!/bin/bash
# 启动 foxglove_bridge 脚本（确保能找到 behavior_tree_msgs）

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# 设置 ROS2 domain ID（如果需要）
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}

# Source ROS2 环境
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Source 工作空间（包含 behavior_tree_msgs）
# 从 operator/behavior-tree-controller 到 airstack/ros_ws 的相对路径
WORKSPACE_PATH="../../airstack/ros_ws/install/setup.bash"
if [ -f "$WORKSPACE_PATH" ]; then
    source "$WORKSPACE_PATH"
elif [ -f "$(dirname "$SCRIPT_DIR")/../../airstack/ros_ws/install/setup.bash" ]; then
    source "$(dirname "$SCRIPT_DIR")/../../airstack/ros_ws/install/setup.bash"
else
    # 尝试使用绝对路径
    ABS_WORKSPACE="/home/yliu/Documents/github/georgia_dtc_ops_team_chiron_mrsd/airstack/ros_ws/install/setup.bash"
    if [ -f "$ABS_WORKSPACE" ]; then
        source "$ABS_WORKSPACE"
    else
        echo "警告: 找不到工作空间 setup.bash"
        echo "尝试查找工作空间..."
        find "$(dirname "$SCRIPT_DIR")/../.." -name "setup.bash" -path "*/airstack/ros_ws/install/*" 2>/dev/null | head -1 | while read ws; do
            echo "找到工作空间: $ws"
            source "$ws"
        done
    fi
fi

# 验证 behavior_tree_msgs 是否可用
if ! ros2 pkg list | grep -q "behavior_tree_msgs"; then
    echo "错误: 找不到 behavior_tree_msgs 包"
    echo "提示: 确保已构建并 source 了工作空间"
    echo "运行: cd airstack/ros_ws && colcon build --packages-select behavior_tree_msgs"
    exit 1
fi

# 验证 behavior_tree_msgs 接口是否可用
if ! ros2 interface show behavior_tree_msgs/msg/Active > /dev/null 2>&1; then
    echo "错误: 无法加载 behavior_tree_msgs/msg/Active"
    echo "提示: 确保工作空间已正确构建"
    exit 1
fi

echo "=========================================="
echo "启动 foxglove_bridge"
echo "=========================================="
echo "ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
echo "行为树消息包: 已找到"
echo "WebSocket 端口: 8765"
echo "客户端发布: 已启用 (clientPublish)"
echo ""
echo "在 Foxglove Studio 中连接: ws://localhost:8765"
echo "=========================================="
echo ""

# 启动 foxglove_bridge，启用 clientPublish 允许客户端广告和发布话题
ros2 run foxglove_bridge foxglove_bridge \
  --ros-args \
  -p port:=8765 \
  -p capabilities:="[clientPublish]"

