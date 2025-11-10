# 行为树控制器 - 操作员指南

## 快速开始（3 步）

### 1️⃣ 启动系统

在终端中运行：

```bash
# 终端 1: 启动 Foxglove Bridge
cd operator/behavior-tree-controller
./start_foxglove_bridge.sh

# 终端 2: 启动机器人系统（包括行为树）
cd airstack/ros_ws
source install/setup.bash
ros2 launch robot_bringup robot.launch.xml
```

### 2️⃣ 连接 Foxglove Studio

1. 打开 Foxglove Studio
2. 点击 **"Open connection"**
3. 选择连接类型：**Foxglove WebSocket**
4. WebSocket URL: `ws://localhost:8765`
5. 点击 **"Open"**

### 3️⃣ 添加控制面板

1. 在 Foxglove 中点击 **"+"** 添加面板
2. 搜索 **"行为树控制器"** 或 **"Behavior Tree Controller"**
3. 点击添加

## 🎮 使用控制面板

### 面板布局

```
┌─────────────────────────────────────────┐
│ Behavior Tree Controller                │
│─────────────────────────────────────────│
│ 状态: 已连接到: /behavior_tree_commands  │
│ Data source: ros2 · Publish: Available  │
│                                          │
│ [重新连接话题]                           │
│─────────────────────────────────────────│
│ Basic Control                            │
│  [Arm]  [Disarm]  [Auto Takeoff]        │
│  [Auto Land]  [EStop]                   │
│─────────────────────────────────────────│
│ Task Control                             │
│  [Survey]  [Geofence Mapping]           │
│  [Go to Waypoint]                       │
└─────────────────────────────────────────┘
```

### 支持的命令

#### 基础控制

| 按钮 | 功能 | 说明 |
|------|------|------|
| **Arm** | 解锁飞行器 | 允许电机启动 |
| **Disarm** | 上锁飞行器 | 停止电机，安全模式 |
| **Auto Takeoff** | 自动起飞 | 自动起飞到预设高度 |
| **Auto Land** | 自动降落 | 自动降落到地面 |
| **EStop** | 紧急停止 | 立即停止所有动作 |

#### 任务控制

| 按钮 | 功能 | 说明 |
|------|------|------|
| **Survey** | 勘测任务 | 执行预定义的勘测路线 |
| **Geofence Mapping** | 地理围栏映射 | 执行地理围栏边界映射 |
| **Go to Waypoint** | 前往航点 | 前往指定的航点位置 |

### 发送命令

1. **确认状态**：面板顶部应显示 `已连接到: /behavior_tree_commands`
2. **点击按钮**：直接点击任意命令按钮
3. **查看反馈**：
   - 面板显示 "Last sent: [命令名称]"
   - 机器人执行相应动作

## 🔧 常见操作场景

### 场景 1: 起飞任务

```
1. [Arm] - 解锁飞行器
2. 等待解锁确认
3. [Auto Takeoff] - 自动起飞
4. 飞行器起飞到预设高度
```

### 场景 2: 勘测任务

```
1. [Arm] - 解锁
2. [Auto Takeoff] - 起飞
3. [Survey] - 开始勘测
4. 等待任务完成
5. [Auto Land] - 降落
6. [Disarm] - 上锁
```

### 场景 3: 紧急情况

```
任何时候发生紧急情况：
[EStop] - 立即紧急停止
```

### 场景 4: 前往特定位置

```
1. [Go to Waypoint] - 发送航点命令
2. 飞行器自动导航到目标位置
```

## ⚠️ 故障排除

### 问题 1: 面板显示 "发布功能不可用"

**原因**: Foxglove 未连接或连接断开

**解决**:
1. 检查 Foxglove 是否连接到 `ws://localhost:8765`
2. 确认 Bridge 正在运行：`pgrep -af foxglove_bridge`
3. 如果 Bridge 未运行，重新启动：`./start_foxglove_bridge.sh`

### 问题 2: 点击按钮无反应

**原因**: 系统重启后话题未重新广告

**解决**:
1. 点击面板顶部的 **"重新连接话题"** 按钮
2. 等待状态显示 "已连接到: /behavior_tree_commands"
3. 重新尝试发送命令

### 问题 3: 命令发送后机器人无响应

**检查**:
```bash
# 终端中验证节点运行
ros2 node list | grep behavior

# 应该看到:
# /behavior_executive
# /behavior_tree_node

# 监听话题确认消息发送
ros2 topic echo /behavior_tree_commands
```

**解决**: 如果节点未运行，重启系统（步骤 1）

### 问题 4: Data source 显示 "unknown"

**不影响功能**：代码已自动处理，仍会使用 ROS 模式

**可选优化**:
1. 在 Foxglove 中断开连接
2. 重新创建连接，确保选择 **"Foxglove WebSocket"** 类型
3. 重新连接到 `ws://localhost:8765`

## 📊 监控和验证

### 验证命令发送（终端）

```bash
# 终端 1: 监听命令话题
cd airstack/ros_ws
source install/setup.bash
ros2 topic echo /behavior_tree_commands

# 终端 2: 查看话题信息
ros2 topic info /behavior_tree_commands
# 应该显示: Publisher count: 1 (foxglove_bridge)
#           Subscription count: 2 (behavior nodes)
```

### 查看所有行为树话题

```bash
ros2 topic list | grep -E "behavior|arm|takeoff|land"

# 输出示例:
# /behavior_tree_commands     - 命令输入
# /arm_active                 - Arm 状态
# /arm_status                 - Arm 执行状态
# /auto_takeoff_commanded_success
# /autoland_active
# ...
```

## 🎯 典型工作流程

### 日常飞行任务

```
1. 启动系统
   └─> ./start_foxglove_bridge.sh
   └─> ros2 launch robot_bringup robot.launch.xml

2. 连接 Foxglove
   └─> ws://localhost:8765

3. 打开行为树控制器面板

4. 执行飞行
   └─> [Arm]
   └─> [Auto Takeoff]
   └─> [任务命令: Survey/Go to Waypoint/等]
   └─> [Auto Land]
   └─> [Disarm]

5. 结束
   └─> 关闭 Foxglove
   └─> Ctrl+C 终止系统
```

### 系统重启后恢复

```
1. 确认 Bridge 运行中
   └─> pgrep -af foxglove_bridge

2. 如果未运行，启动 Bridge
   └─> ./start_foxglove_bridge.sh

3. 在 Foxglove 中
   └─> 点击 "重新连接话题" 按钮

4. 继续操作
```

## 📝 配置（可选）

### 修改话题名称

在面板设置（右上角齿轮图标）中：

- **Robot Namespace**: 机器人命名空间（默认为空）
  - 如果设置为 `robot_1`，话题变为 `/robot_1/behavior_tree_commands`
  
- **Publish Topic**: 完整话题路径（默认 `/behavior_tree_commands`）
  - 手动指定话题名称

### 默认配置

```
Robot Namespace: (空)
Publish Topic: /behavior_tree_commands
```

适用于大多数情况，无需修改。

## 🚨 安全注意事项

1. **紧急停止随时可用**: EStop 按钮在任何时候都可以使用
2. **确认状态**: 发送命令前确认面板状态为 "已连接"
3. **监控飞行器**: 始终监控飞行器实际状态
4. **测试环境**: 在安全环境中测试新命令
5. **记录操作**: 关键任务时记录发送的命令序列

## 📚 更多资源

- **系统架构**: `BEHAVIOR_TREE_INTEGRATION.md`
- **故障排除**: `USAGE.md`
- **快速修复**: `QUICK_FIX.md`
- **关键修复说明**: `CRITICAL_FIX.md`

## ✅ 检查清单

启动前检查：
- [ ] Bridge 正在运行 (`pgrep -af foxglove_bridge`)
- [ ] ROS2 节点正在运行 (`ros2 node list | grep behavior`)
- [ ] Foxglove 已连接到 `ws://localhost:8765`
- [ ] 面板显示 "已连接到: /behavior_tree_commands"

飞行前检查：
- [ ] 飞行器已上电
- [ ] MAVROS 连接正常
- [ ] GPS 信号良好（如需要）
- [ ] 电池电量充足
- [ ] 周围环境安全

---

**版本**: 1.0  
**最后更新**: 2025-11-04  
**维护**: Chiron Team

