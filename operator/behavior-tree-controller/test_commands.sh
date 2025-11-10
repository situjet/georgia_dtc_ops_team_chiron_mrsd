#!/bin/bash

# 测试 behavior_tree_commands 发布
# 用于验证消息格式是否与 AirStack_GCS 兼容

set -e

TOPIC="/dtc_mrsd_/behavior_tree_commands"
MSG_TYPE="behavior_tree_msgs/msg/BehaviorTreeCommands"

echo "=========================================="
echo "Behavior Tree Commands 测试脚本"
echo "=========================================="
echo ""

# 检查话题是否存在
echo "1. 检查话题是否存在..."
if ros2 topic list | grep -q "$TOPIC"; then
    echo "   ✓ 话题存在: $TOPIC"
else
    echo "   ✗ 话题不存在: $TOPIC"
    echo "   请确保 behavior_executive 正在运行"
    exit 1
fi

echo ""

# 检查话题类型
echo "2. 检查话题类型..."
ACTUAL_TYPE=$(ros2 topic type "$TOPIC" 2>/dev/null || echo "")
if [ "$ACTUAL_TYPE" == "$MSG_TYPE" ]; then
    echo "   ✓ 话题类型正确: $MSG_TYPE"
else
    echo "   话题类型: $ACTUAL_TYPE"
    echo "   (如果为空，可能是话题刚创建)"
fi

echo ""

# 发送测试命令：Arm
echo "3. 发送测试命令：Arm（与 AirStack_GCS 格式兼容）..."
echo "   发送所有 6 个命令的状态（只有 Arm 是 SUCCESS）"

ros2 topic pub "$TOPIC" "$MSG_TYPE" "{
  commands: [
    {condition_name: 'Arm Commanded', status: 2},
    {condition_name: 'Disarm Commanded', status: 0},
    {condition_name: 'Auto Takeoff Commanded', status: 0},
    {condition_name: 'AutoLand Commanded', status: 0},
    {condition_name: 'EStop Commanded', status: 0},
    {condition_name: 'Go to Waypoint Commanded', status: 0}
  ]
}" --once

echo "   ✓ 命令已发送"
echo ""

# 等待一下
sleep 2

# 发送测试命令：Auto Takeoff
echo "4. 发送测试命令：Auto Takeoff..."
echo "   发送所有 6 个命令的状态（只有 Auto Takeoff 是 SUCCESS）"

ros2 topic pub "$TOPIC" "$MSG_TYPE" "{
  commands: [
    {condition_name: 'Arm Commanded', status: 0},
    {condition_name: 'Disarm Commanded', status: 0},
    {condition_name: 'Auto Takeoff Commanded', status: 2},
    {condition_name: 'AutoLand Commanded', status: 0},
    {condition_name: 'EStop Commanded', status: 0},
    {condition_name: 'Go to Waypoint Commanded', status: 0}
  ]
}" --once

echo "   ✓ 命令已发送"
echo ""

# 监听话题（显示最近的消息）
echo "5. 监听话题（3 秒）..."
echo "   按 Ctrl+C 停止"
echo ""

timeout 3 ros2 topic echo "$TOPIC" || true

echo ""
echo "=========================================="
echo "测试完成"
echo "=========================================="
echo ""
echo "说明："
echo "- status: 2 = SUCCESS（激活命令）"
echo "- status: 0 = FAILURE（停用命令）"
echo "- 每次发送都包含所有 6 个命令的状态"
echo "- 只有一个命令的 status 为 2，其他都为 0"
echo ""
echo "这与 AirStack_GCS rqt 插件的行为一致"
echo ""

