# Behavior Tree Controller - 故障排除指南

## "Not advertised" 错误修复

### 问题描述
之前的实现在发布消息前会检查广告状态，如果未广告则显示错误。这会导致时序问题。

### 解决方案
已简化广告逻辑，现在像 gimbal 控制器一样工作：
- 在每次发布前调用 `ensureAdvertised()`
- 不再显式检查广告状态
- 失败时静默处理，不阻止后续操作

## 使用步骤

### 1. 在 Foxglove 中加载扩展

```bash
# 首次安装或更新后
cd operator/behavior-tree-controller
npm run build
```

在 Foxglove Studio 中：
1. 打开 Extensions 面板
2. 点击 "Import extension"
3. 选择 `operator/behavior-tree-controller/dist/extension.js`

### 2. 连接到 ROS 系统

确保 Foxglove 已连接到 ROS 2：
- 使用 Foxglove Bridge：`ros2 launch foxglove_bridge foxglove_bridge_launch.xml`
- 或使用 Rosbridge WebSocket

### 3. 配置扩展

在扩展设置中：
- **Robot Namespace**: `dtc_mrsd_` (不带斜杠)
- **Publish Topic**: `/dtc_mrsd_/behavior_tree_commands`

### 4. 验证连接

打开终端监听话题：
```bash
ros2 topic echo /dtc_mrsd_/behavior_tree_commands
```

在 Foxglove 中点击任意命令按钮，应该能看到消息被发布。

## 常见问题

### Q1: 点击按钮后没有反应

**检查项**：
1. 确认 Foxglove 已连接到 ROS 2
2. 检查浏览器控制台（F12）查看错误日志
3. 确认话题名称配置正确

**解决方法**：
```bash
# 检查话题是否存在
ros2 topic list | grep behavior_tree_commands

# 检查话题类型
ros2 topic info /dtc_mrsd_/behavior_tree_commands
```

### Q2: 状态显示 "发布功能不可用"

**原因**：Foxglove 未连接到数据源或使用的是播放模式

**解决方法**：
- 确保使用 Live 连接（不是播放录制的数据）
- 重新连接 Foxglove Bridge

### Q3: 行为树没有响应命令

**检查项**：
1. 确认 behavior_executive 节点正在运行
2. 检查话题是否正确订阅
3. 验证消息格式

**验证方法**：
```bash
# 检查节点是否运行
ros2 node list | grep behavior_executive

# 检查订阅
ros2 node info /dtc_mrsd_/behavior_executive

# 手动发布测试消息
ros2 topic pub /dtc_mrsd_/behavior_tree_commands behavior_tree_msgs/msg/BehaviorTreeCommands "{
  commands: [
    {condition_name: 'Arm Commanded', status: 2},
    {condition_name: 'Disarm Commanded', status: 0},
    {condition_name: 'Auto Takeoff Commanded', status: 0},
    {condition_name: 'AutoLand Commanded', status: 0},
    {condition_name: 'EStop Commanded', status: 0},
    {condition_name: 'Go to Waypoint Commanded', status: 0}
  ]
}" --once
```

### Q4: 扩展更新后还是旧版本

**解决方法**：
1. 在 Foxglove 中卸载旧扩展
2. 关闭 Foxglove
3. 重新构建：`npm run build`
4. 重新打开 Foxglove 并导入扩展

### Q5: 消息格式不正确

**正确格式示例**：
```json
{
  "commands": [
    { "condition_name": "Arm Commanded", "status": 2 },
    { "condition_name": "Disarm Commanded", "status": 0 },
    { "condition_name": "Auto Takeoff Commanded", "status": 0 },
    { "condition_name": "AutoLand Commanded", "status": 0 },
    { "condition_name": "EStop Commanded", "status": 0 },
    { "condition_name": "Go to Waypoint Commanded", "status": 0 }
  ]
}
```

**关键点**：
- `status: 2` = SUCCESS (激活命令)
- `status: 0` = FAILURE (停用命令)
- 必须发送**所有**命令的状态

## 调试技巧

### 启用详细日志

打开浏览器控制台（F12），所有日志都带有 `[BehaviorTree]` 前缀。

**正常的日志流程**：
```
[BehaviorTree] Attempting to advertise topic: /dtc_mrsd_/behavior_tree_commands
[BehaviorTree] Data source profile: ros2
[BehaviorTree] Mode: ros
[BehaviorTree] Calling context.advertise with behavior_tree_msgs/msg/BehaviorTreeCommands
[BehaviorTree] ✓ Advertised
[BehaviorTree] ✓ Successfully advertised: /dtc_mrsd_/behavior_tree_commands mode: ros
[BehaviorTree] sendCommand called: Arm Commanded {active: true}
[BehaviorTree] Publishing message to /dtc_mrsd_/behavior_tree_commands : {commands: Array(6)}
[BehaviorTree] ✓ Message published
```

### 检查消息内容

在控制台中展开发布的消息对象，确认：
- `commands` 数组包含 6 个元素
- 每个元素都有 `condition_name` 和 `status` 字段
- 只有一个命令的 `status` 是 2，其他都是 0

### 对比 rqt GUI

如果 Foxglove 扩展不工作，但 rqt GUI 可以：
1. 对比话题名称
2. 使用 `ros2 topic echo` 对比消息格式
3. 检查命令名称是否完全匹配（包括大小写和空格）

## 与 AirStack_GCS rqt GUI 的差异

| 特性 | rqt GUI | Foxglove 扩展 |
|------|---------|---------------|
| 话题格式 | `/{robot}/behavior/behavior_tree_commands` | `/{robot}/behavior_tree_commands` |
| 消息格式 | ✓ 相同 | ✓ 相同 |
| 互斥逻辑 | ✓ 支持 | ✓ 支持 |
| 支持的命令 | 可配置 | 硬编码 6 个命令 |
| Timeline 功能 | ✓ 支持 | ✗ 不支持 |

## 相关文件

- **扩展源码**: `src/BehaviorTreeControllerPanel.tsx`
- **构建输出**: `dist/extension.js`
- **消息定义**: `airstack/ros_ws/src/behavior_tree_system/behavior_tree_msgs/msg/`
- **兼容性文档**: `AIRSTACK_GCS_COMPATIBILITY.md`

## 获取帮助

如果问题仍未解决：
1. 检查浏览器控制台的完整错误信息
2. 检查 ROS 2 日志：`ros2 topic echo /rosout`
3. 验证消息定义：`ros2 interface show behavior_tree_msgs/msg/BehaviorTreeCommands`

