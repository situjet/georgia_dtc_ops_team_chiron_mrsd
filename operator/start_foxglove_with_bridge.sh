#!/bin/bash
# 统一启动 Foxglove Bridge 和 Domain Bridge
# 域100: Foxglove/Operator
# 域70: 无人机

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

echo "=========================================="
echo "Foxglove + Domain Bridge 启动脚本"
echo "=========================================="
echo "域100 (我们) ← Foxglove Bridge"
echo "域70  (无人机) ← Domain Bridge"
echo "=========================================="
echo ""

# 检查是否已经有实例在运行
check_running() {
    if pgrep -f "foxglove_bridge" > /dev/null; then
        echo "⚠️  Foxglove Bridge 已在运行"
        read -p "是否停止并重启? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            pkill -f foxglove_bridge
            sleep 2
        else
            exit 1
        fi
    fi
    
    if pgrep -f "domain_bridge.*70.*100" > /dev/null; then
        echo "⚠️  Domain Bridge 已在运行"
        read -p "是否停止并重启? (y/n) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            pkill -f "domain_bridge.*70.*100"
            sleep 2
        else
            exit 1
        fi
    fi
}

# 检查运行中的实例
check_running

# Source ROS2 环境
echo "1️⃣  设置 ROS2 环境..."
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "❌ 错误: 找不到 ROS2 Humble"
    exit 1
fi

# Source 工作空间
WORKSPACE="$PROJECT_ROOT/airstack/ros_ws/install/setup.bash"
if [ -f "$WORKSPACE" ]; then
    source "$WORKSPACE"
    echo "✓ 工作空间已加载: $WORKSPACE"
else
    echo "❌ 错误: 找不到工作空间 setup.bash"
    exit 1
fi

# 验证必要的包
echo ""
echo "2️⃣  验证 ROS2 包..."
if ! ros2 pkg list | grep -q "behavior_tree_msgs"; then
    echo "❌ 错误: 找不到 behavior_tree_msgs"
    exit 1
fi
echo "✓ behavior_tree_msgs 已找到"

if ! ros2 pkg list | grep -q "domain_bridge"; then
    echo "❌ 错误: 找不到 domain_bridge"
    echo "安装: sudo apt install ros-humble-domain-bridge"
    exit 1
fi
echo "✓ domain_bridge 已找到"

if ! ros2 pkg list | grep -q "foxglove_bridge"; then
    echo "❌ 错误: 找不到 foxglove_bridge"
    echo "安装: sudo apt install ros-humble-foxglove-bridge"
    exit 1
fi
echo "✓ foxglove_bridge 已找到"

# 验证配置文件 - 使用 operator 专用精简配置
CONFIG_FILE="$PROJECT_ROOT/operator/domain_bridge_operator.yaml"
if [ ! -f "$CONFIG_FILE" ]; then
    echo "❌ 错误: 找不到 domain bridge 配置文件"
    echo "   期望位置: $CONFIG_FILE"
    exit 1
fi
echo "✓ Domain bridge 配置已找到 (operator 专用精简版)"

echo ""
echo "=========================================="
echo "3️⃣  启动服务..."
echo "=========================================="

# 启动 Domain Bridge (域70 <-> 域100)
echo ""
echo "启动 Domain Bridge (域70 ↔ 域100)..."
export ROS_DOMAIN_ID=70
gnome-terminal --title="Domain Bridge 70↔100" -- bash -c "
    source /opt/ros/humble/setup.bash
    source $WORKSPACE
    export ROS_DOMAIN_ID=70
    echo '=========================================='
    echo 'Domain Bridge 70 ↔ 100'
    echo '=========================================='
    echo 'ROS_DOMAIN_ID: $ROS_DOMAIN_ID'
    echo 'Config: $CONFIG_FILE'
    echo ''
    echo 'Starting domain bridge...'
    ros2 run domain_bridge domain_bridge $CONFIG_FILE
    BRIDGE_EXIT=\$?
    echo ''
    echo '❌ Domain Bridge 已停止 (exit code: '$BRIDGE_EXIT')'
    read -p '按任意键关闭...'
" &

sleep 3

# 启动 Foxglove Bridge (域100)
echo ""
echo "启动 Foxglove Bridge (域100)..."
export ROS_DOMAIN_ID=100
gnome-terminal --title="Foxglove Bridge (域100)" -- bash -c "
    source /opt/ros/humble/setup.bash
    source $WORKSPACE
    export ROS_DOMAIN_ID=100
    echo '=========================================='
    echo 'Foxglove Bridge (域100)'
    echo '=========================================='
    echo 'ROS_DOMAIN_ID: $ROS_DOMAIN_ID'
    echo 'WebSocket 端口: 8765'
    echo 'clientPublish: 已启用'
    echo ''
    echo '在 Foxglove Studio 连接到: ws://localhost:8765'
    echo '=========================================='
    echo ''
    ros2 run foxglove_bridge foxglove_bridge \
      --ros-args \
      -p port:=8765 \
      -p capabilities:=\"[clientPublish]\"
    echo ''
    echo '❌ Foxglove Bridge 已停止'
    read -p '按任意键关闭...'
" &

sleep 2

echo ""
echo "=========================================="
echo "✅ 服务已启动！"
echo "=========================================="
echo ""
echo "📡 Domain Bridge:"
echo "   域70 (无人机) ↔ 域100 (Foxglove)"
echo ""
echo "🌐 Foxglove Bridge:"
echo "   端口: 8765"
echo "   域: 100"
echo "   连接: ws://localhost:8765"
echo ""
echo "📋 话题流向 (operator 专用精简配置):"
echo "   从无人机接收 (6个):"
echo "   • /dtc_mrsd_/mavros/global_position/global (位置)"
echo "   • /dtc_mrsd_/mavros/global_position/compass_hdg (航向)"
echo "   • /robot_1/mavros/geofence/fences (围栏)"
echo "   • /target_gps, /target_gps_list, /precise_target_gps (目标)"
echo ""
echo "   发送到无人机 (4个):"
echo "   • /selected_waypoint (航点选择)"
echo "   • /dtc_mrsd_/behavior_tree_commands (飞行控制)"
echo "   • /gimbal/teleop (云台控制)"
echo ""
echo "🔍 验证命令:"
echo "   ros2 topic list  # 在域100查看话题"
echo "   ROS_DOMAIN_ID=70 ros2 topic list  # 在域70查看话题"
echo ""
echo "🛑 停止服务:"
echo "   pkill -f foxglove_bridge"
echo "   pkill -f domain_bridge"
echo ""
echo "按 Ctrl+C 退出监控..."
echo "=========================================="

# 监控进程
while true; do
    sleep 5
    if ! pgrep -f "foxglove_bridge" > /dev/null; then
        echo "⚠️  Foxglove Bridge 已停止"
    fi
    # 检查 domain_bridge 进程（使用配置文件路径匹配）
    if ! pgrep -f "domain_bridge.*domain_bridge_operator.yaml" > /dev/null; then
        echo "⚠️  Domain Bridge 已停止"
    fi
done



