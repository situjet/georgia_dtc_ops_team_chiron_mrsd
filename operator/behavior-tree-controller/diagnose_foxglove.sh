#!/bin/bash
# 诊断 Foxglove Bridge 和 behavior_tree_msgs 的连接问题

echo "=========================================="
echo "Foxglove Bridge 诊断脚本"
echo "=========================================="
echo ""

# 1. 检查当前环境
echo "1. 检查 ROS2 环境"
echo "---"
if command -v ros2 &> /dev/null; then
    echo "✓ ros2 命令可用"
    ros2 --version 2>&1 | head -1
else
    echo "✗ ros2 命令不可用"
    exit 1
fi
echo ""

# 2. 检查 behavior_tree_msgs 包
echo "2. 检查 behavior_tree_msgs 包"
echo "---"
if ros2 pkg list 2>/dev/null | grep -q "^behavior_tree_msgs$"; then
    echo "✓ behavior_tree_msgs 包已找到"
    ros2 pkg prefix behavior_tree_msgs 2>/dev/null
else
    echo "✗ behavior_tree_msgs 包未找到"
    echo "请运行: cd airstack/ros_ws && colcon build --packages-select behavior_tree_msgs"
    exit 1
fi
echo ""

# 3. 检查消息定义
echo "3. 检查消息定义"
echo "---"
if ros2 interface show behavior_tree_msgs/msg/BehaviorTreeCommands &> /dev/null; then
    echo "✓ BehaviorTreeCommands 消息定义可用"
    ros2 interface show behavior_tree_msgs/msg/BehaviorTreeCommands 2>/dev/null
else
    echo "✗ BehaviorTreeCommands 消息定义不可用"
    exit 1
fi
echo ""

# 4. 检查话题
echo "4. 检查 /behavior_tree_commands 话题"
echo "---"
if ros2 topic list 2>/dev/null | grep -q "^/behavior_tree_commands$"; then
    echo "✓ 话题存在"
    ros2 topic info /behavior_tree_commands 2>/dev/null
else
    echo "✗ 话题不存在（需要 behavior_executive 节点运行）"
fi
echo ""

# 5. 检查节点
echo "5. 检查 behavior_executive 节点"
echo "---"
if ros2 node list 2>/dev/null | grep -q "behavior_executive"; then
    echo "✓ behavior_executive 节点正在运行"
else
    echo "✗ behavior_executive 节点未运行"
    echo "请运行: ros2 run behavior_executive behavior_executive"
fi
echo ""

# 6. 检查 foxglove_bridge
echo "6. 检查 foxglove_bridge"
echo "---"
if pgrep -f foxglove_bridge > /dev/null; then
    echo "✓ foxglove_bridge 进程正在运行"
    echo "进程 PID:"
    pgrep -af foxglove_bridge | head -1
else
    echo "✗ foxglove_bridge 未运行"
    echo "请运行: ./start_foxglove_bridge.sh"
fi
echo ""

# 7. 检查 Python 环境
echo "7. 检查 Python 环境"
echo "---"
echo "当前 Python: $(which python3)"
python3 --version
if python3 -c "import behavior_tree_msgs" 2>/dev/null; then
    echo "✓ Python 可以导入 behavior_tree_msgs"
else
    echo "✗ Python 无法导入 behavior_tree_msgs"
    echo "可能原因: Python 版本不匹配或工作空间未 source"
fi
echo ""

# 8. 检查 AMENT_PREFIX_PATH
echo "8. 检查 AMENT_PREFIX_PATH"
echo "---"
if echo "$AMENT_PREFIX_PATH" | grep -q "behavior_tree_msgs"; then
    echo "✓ AMENT_PREFIX_PATH 包含 behavior_tree_msgs"
    echo "$AMENT_PREFIX_PATH" | tr ':' '\n' | grep behavior_tree
else
    echo "✗ AMENT_PREFIX_PATH 不包含 behavior_tree_msgs"
    echo "请 source 工作空间: source airstack/ros_ws/install/setup.bash"
fi
echo ""

# 9. 建议
echo "=========================================="
echo "诊断完成"
echo "=========================================="
echo ""
echo "如果所有检查都通过，请："
echo "1. 停止当前的 foxglove_bridge"
echo "2. 运行 ./start_foxglove_bridge.sh 重新启动"
echo "3. 在 Foxglove Studio 中重新连接到 ws://localhost:8765"
echo "4. 重新加载行为树控制器扩展"
echo ""

