# AirStack_GCS Compatibility Fix

## 问题描述

原始的 Foxglove 扩展实现与 AirStack_GCS 的消息发送方式不一致，导致行为树系统无法正确接收命令。

## 关键差异

### ❌ **修复前的实现** (错误)

```typescript
// 只发送单个命令
const msg = {
  commands: [
    {
      condition_name: commandName,
      status: commandStatus,
    },
  ],
};
```

**问题**：
- 只发送被点击的单个命令
- 行为树系统无法获知其他条件的状态
- 缺少互斥逻辑

### ✓ **修复后的实现** (正确)

```typescript
// 发送所有命令的完整状态
const commandsList = BEHAVIOR_COMMANDS.map((cmd) => ({
  condition_name: cmd.name,
  status: cmd.name === commandName && options.active ? 2 : 0, // SUCCESS or FAILURE
}));

const msg = {
  commands: commandsList,
};
```

**修复内容**：
- 发送所有条件的状态
- 被选中的命令：`status = 2` (SUCCESS)
- 其他所有命令：`status = 0` (FAILURE)
- 实现互斥逻辑，确保只有一个命令被激活

## AirStack_GCS 参考实现

### Python rqt 插件的实现

```python
def get_click_function(group, button):
    def click_function():
        commands = BehaviorTreeCommands()
        for i in range(len(self.button_groups[group]['buttons'])):
            b = self.button_groups[group]['buttons'][i]
            command = BehaviorTreeCommand()
            command.condition_name = self.button_groups[group]['condition_names'][i]

            if b != button and b.isChecked():
                b.toggle()
                command.status = Status.FAILURE
            elif b == button and not b.isChecked():
                command.status = Status.FAILURE
            elif b == button and b.isChecked():
                command.status = Status.SUCCESS
            commands.commands.append(command)
        
        # 发布包含所有条件状态的消息
        self.settings['publishers'][self.robot_combo_box.currentText()]['command_pub'].publish(commands)
    return click_function
```

### 消息定义

```
# BehaviorTreeCommand.msg
string condition_name
int8 status

# BehaviorTreeCommands.msg  
BehaviorTreeCommand[] commands

# Status.msg
int8 FAILURE=0
int8 RUNNING=1
int8 SUCCESS=2
int8 status
uint64 id
```

## 消息示例

### 发送 "Auto Takeoff" 命令

```json
{
  "commands": [
    { "condition_name": "Arm Commanded", "status": 0 },
    { "condition_name": "Disarm Commanded", "status": 0 },
    { "condition_name": "Auto Takeoff Commanded", "status": 2 },
    { "condition_name": "AutoLand Commanded", "status": 0 },
    { "condition_name": "EStop Commanded", "status": 0 },
    { "condition_name": "Go to Waypoint Commanded", "status": 0 }
  ]
}
```

## 设计原理

### 为什么要发送所有命令？

1. **完整状态同步**
   - 行为树系统需要知道所有条件的当前状态
   - 避免部分状态更新导致的不一致

2. **互斥逻辑**
   - 确保同一时间只有一个任务命令被激活
   - 防止多个冲突的命令同时执行

3. **状态清除**
   - 激活新命令时，自动将其他命令设为 FAILURE
   - 保证状态机的确定性

4. **消息一致性**
   - 每条消息都包含完整的系统状态
   - 即使有消息丢失，下一条消息也能完整恢复状态

## 话题命名

```
AirStack_GCS:  /{robot_name}/behavior/behavior_tree_commands
当前系统:       /{robot_namespace}/behavior_tree_commands
```

注意：当前系统移除了中间的 `/behavior/` 层级。

## 测试验证

### 1. 检查发送的消息

```bash
# 监听话题
ros2 topic echo /dtc_mrsd_/behavior_tree_commands
```

### 2. 验证消息内容

点击 "Auto Takeoff" 按钮后，应该看到：
- 包含 6 个命令的数组
- 只有 "Auto Takeoff Commanded" 的 status 为 2
- 其他所有命令的 status 为 0

### 3. 功能测试

```bash
# 启动行为树系统
ros2 launch robot_bringup robot.launch.xml

# 在 Foxglove 中加载 behavior-tree-controller 扩展
# 点击各个命令按钮，观察无人机响应
```

## 相关文件

- **修复的文件**: `operator/behavior-tree-controller/src/BehaviorTreeControllerPanel.tsx`
- **参考实现**: `old/AirStack_GCS/ground_control_station/ros_ws/src/rqt_ground_control_station/src/rqt_ground_control_station/template.py`
- **消息定义**: `airstack/ros_ws/src/behavior_tree_system/behavior_tree_msgs/msg/`

## 修复日期

2025-11-04

## 影响

✅ 修复后，Foxglove 扩展的行为与 AirStack_GCS rqt 插件完全一致
✅ 行为树系统可以正确接收和处理命令
✅ 互斥逻辑确保系统状态一致性

