# 行为树控制器使用指南

## 问题诊断

### 问题：重启节点后显示 "topic has not been advertised before"

**根本原因**：
- Foxglove 扩展在浏览器中缓存了广告状态（`advertisedRef`）
- 当 `foxglove_bridge` 或 ROS2 节点重启时，服务端的广告状态被清除
- 扩展认为话题已广告（因为缓存），但实际上 bridge 端已经丢失了这个状态

**解决方案**：

### 方法 1：点击"重新连接话题"按钮（推荐）

1. 在行为树控制器面板顶部，点击 **"重新连接话题"** 按钮
2. 查看状态栏，应显示 "已连接到: /behavior_tree_commands"
3. 现在可以发送命令了

### 方法 2：在 Foxglove 中断开重连

1. 在 Foxglove Studio 中，断开当前连接
2. 重新连接到 `ws://localhost:8765`
3. 打开行为树控制器面板

### 方法 3：重新加载扩展

1. 在 Foxglove Studio 中，关闭行为树控制器面板
2. 重新打开面板（扩展会重新初始化）

## 正确的启动顺序

### 1. 启动 foxglove_bridge（使用脚本）

```bash
cd operator/behavior-tree-controller
./start_foxglove_bridge.sh
```

**重要**：必须使用脚本启动，确保 bridge 能找到 `behavior_tree_msgs`。

脚本会：
- Source ROS2 环境（/opt/ros/humble/setup.bash）
- Source 工作空间（airstack/ros_ws/install/setup.bash）
- 验证 `behavior_tree_msgs` 包可用
- 启动 bridge 在端口 8765

### 2. 启动 behavior_executive 节点

```bash
cd airstack/ros_ws
source install/setup.bash
ros2 run behavior_executive behavior_executive
```

或者启动完整系统：

```bash
cd airstack/ros_ws
source install/setup.bash
ros2 launch robot_bringup robot.launch.xml
```

### 3. 在 Foxglove Studio 中连接

1. 打开 Foxglove Studio
2. 连接到 `ws://localhost:8765`
3. 数据源应显示为 "ros2"（不是 "unknown"）

### 4. 打开行为树控制器面板

1. 添加面板 → 行为树控制器
2. 查看状态栏，应显示 "已连接到: /behavior_tree_commands"
3. 数据源应显示为 "ros2"

## 验证设置

运行诊断脚本：

```bash
cd operator/behavior-tree-controller
./diagnose_foxglove.sh
```

所有检查应显示 ✓。

## 常见问题

### Q: 数据源显示 "unknown"
**A**: 
- 断开 Foxglove 连接
- 确保 `foxglove_bridge` 正在运行
- 重新连接到 `ws://localhost:8765`

### Q: 状态显示 "广告失败"
**A**:
- 检查 `foxglove_bridge` 是否使用脚本启动
- 检查 bridge 终端是否有错误日志
- 重启 bridge：`pkill -f foxglove_bridge && ./start_foxglove_bridge.sh`

### Q: 消息发送后没有反应
**A**:
- 检查 `behavior_executive` 节点是否运行：`ros2 node list | grep behavior`
- 查看话题订阅者：`ros2 topic info /behavior_tree_commands`
- 监听话题：`ros2 topic echo /behavior_tree_commands`

### Q: bridge 报错 "package 'behavior_tree_msgs' not found"
**A**:
- **不要**直接运行 `ros2 run foxglove_bridge foxglove_bridge`
- **必须**使用 `start_foxglove_bridge.sh` 脚本
- 脚本确保正确的环境变量（AMENT_PREFIX_PATH）

## 调试日志

在 Foxglove Studio 中按 F12 打开浏览器开发者工具，查看控制台日志：

**成功的日志**：
```
[BehaviorTree] Attempting to advertise topic: /behavior_tree_commands
[BehaviorTree] Data source profile: ros2
[BehaviorTree] Mode: ros
[BehaviorTree] ✓ Successfully advertised: /behavior_tree_commands mode: ros
[BehaviorTree] sendCommand called: Arm Commanded {active: true}
[BehaviorTree] Publishing message to /behavior_tree_commands : {...}
[BehaviorTree] ✓ Message published
```

**失败的日志示例**：
```
[BehaviorTree] Topic not yet advertised (topic match: false mode match: false), retrying...
```

## 支持的命令

- **Arm** - 解锁飞行器
- **Disarm** - 上锁飞行器
- **Auto Takeoff** - 自动起飞
- **Auto Land** - 自动降落
- **EStop** - 紧急停止
- **Survey** - 勘测任务
- **Geofence Mapping** - 地理围栏映射
- **Go to Waypoint** - 前往航点

## 配置

在面板设置中可以配置：

- **Robot Namespace** - 机器人命名空间（默认为空）
- **Publish Topic** - 发布话题路径（默认：`/behavior_tree_commands`）

## 技术说明

### 消息格式

```
behavior_tree_msgs/BehaviorTreeCommands:
  commands:
    - condition_name: "Arm Commanded"
      status: 2  # SUCCESS=2 (激活), FAILURE=0 (取消)
```

### 状态值

- `0` (FAILURE) - 取消/停用命令
- `2` (SUCCESS) - 激活命令

### 话题信息

- **话题名称**: `/behavior_tree_commands`
- **消息类型**: `behavior_tree_msgs/msg/BehaviorTreeCommands`
- **订阅者**: `behavior_executive` 节点

## 故障排除流程

1. **运行诊断**：`./diagnose_foxglove.sh`
2. **检查所有 ✓ 是否通过**
3. **如果失败**：
   - 停止 foxglove_bridge：`pkill -f foxglove_bridge`
   - 使用脚本重启：`./start_foxglove_bridge.sh`
   - 在 Foxglove 中断开重连
4. **在 Foxglove 中点击"重新连接话题"按钮**
5. **测试发送命令**

## 监控命令接收

在终端中监听话题：

```bash
cd airstack/ros_ws
source install/setup.bash
ros2 topic echo /behavior_tree_commands
```

应该能看到发送的命令消息。

