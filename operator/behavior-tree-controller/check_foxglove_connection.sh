#!/bin/bash
# 检查 Foxglove 连接和 ROS2 domain 配置

echo "=========================================="
echo "Foxglove ROS2 连接检查脚本"
echo "=========================================="
echo ""

# 1. 检查 foxglove_bridge 是否运行
echo "1. 检查 foxglove_bridge 进程..."
if pgrep -f "foxglove_bridge" > /dev/null; then
    echo "   ✓ foxglove_bridge 正在运行"
    BRIDGE_PID=$(pgrep -f "foxglove_bridge" | head -1)
    echo "   PID: $BRIDGE_PID"
    
    # 尝试获取 ROS_DOMAIN_ID
    BRIDGE_DOMAIN=$(cat /proc/$BRIDGE_PID/environ 2>/dev/null | tr '\0' '\n' | grep ROS_DOMAIN_ID | cut -d= -f2)
    if [ -n "$BRIDGE_DOMAIN" ]; then
        echo "   ROS_DOMAIN_ID: $BRIDGE_DOMAIN"
    else
        echo "   ROS_DOMAIN_ID: 未设置 (默认 0)"
        BRIDGE_DOMAIN=0
    fi
else
    echo "   ✗ foxglove_bridge 未运行"
    echo "   提示: 运行 'ros2 run foxglove_bridge foxglove_bridge --port 8765'"
    exit 1
fi

echo ""

# 2. 检查端口 8765
echo "2. 检查 WebSocket 端口 8765..."
if netstat -tlnp 2>/dev/null | grep -q ":8765 " || ss -tlnp 2>/dev/null | grep -q ":8765 "; then
    echo "   ✓ 端口 8765 正在监听"
else
    echo "   ✗ 端口 8765 未监听"
    echo "   提示: 确保 foxglove_bridge 正在运行"
fi

echo ""

# 3. 检查当前 ROS2 环境
echo "3. 检查当前 ROS2 环境..."
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash 2>/dev/null
    
    if [ -f ../../../airstack/ros_ws/install/setup.bash ]; then
        source ../../../airstack/ros_ws/install/setup.bash 2>/dev/null
    fi
    
    CURRENT_DOMAIN=${ROS_DOMAIN_ID:-0}
    echo "   当前 ROS_DOMAIN_ID: $CURRENT_DOMAIN"
    
    if [ "$CURRENT_DOMAIN" != "$BRIDGE_DOMAIN" ]; then
        echo "   ⚠ 警告: 当前 domain ID ($CURRENT_DOMAIN) 与 foxglove_bridge domain ID ($BRIDGE_DOMAIN) 不匹配"
        echo "   提示: 确保使用相同的 ROS_DOMAIN_ID"
    else
        echo "   ✓ domain ID 匹配"
    fi
    
    # 检查话题
    echo ""
    echo "4. 检查 ROS2 话题..."
    TOPICS=$(ros2 topic list 2>/dev/null | grep -E "behavior_tree|dtc_mrsd" || echo "")
    if [ -n "$TOPICS" ]; then
        echo "   ✓ 找到相关话题:"
        echo "$TOPICS" | sed 's/^/     /'
    else
        echo "   ✗ 未找到 behavior_tree 相关话题"
        echo "   提示: 确保 behavior_executive 节点正在运行"
        echo "   提示: 确保使用正确的 ROS_DOMAIN_ID"
    fi
else
    echo "   ✗ ROS2 环境未找到"
fi

echo ""
echo "=========================================="
echo "Foxglove Studio 连接检查"
echo "=========================================="
echo ""
echo "在 Foxglove Studio 中:"
echo "1. 点击 'Open connection' (或按 Ctrl+K)"
echo "2. 选择 'Foxglove WebSocket'"
echo "3. URL: ws://localhost:8765"
echo "4. 点击 'Open'"
echo ""
echo "连接后，应该能看到:"
echo "- 数据源显示为 'ros2' 而不是 'unknown'"
echo "- 话题列表显示 ROS2 话题"
echo "- 扩展面板显示 '已连接到' 状态"
echo ""
echo "如果仍然显示 'unknown':"
echo "- 检查 foxglove_bridge 日志: cat /tmp/foxglove_bridge.log"
echo "- 确保 foxglove_bridge 使用正确的 ROS_DOMAIN_ID"
echo "- 重新启动 foxglove_bridge 并设置 ROS_DOMAIN_ID"
echo ""

