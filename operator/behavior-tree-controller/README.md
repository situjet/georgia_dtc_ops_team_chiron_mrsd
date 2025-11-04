# 行为树控制器 Foxglove 扩展

Foxglove Studio 扩展，用于通过 GUI 面板向行为树状态机发送命令控制飞行器。

## 📚 文档索引

### 🚀 操作员文档（开始这里！）
- **[QUICK_START.md](QUICK_START.md)** - 3 步快速启动
- **[OPERATOR_GUIDE.md](OPERATOR_GUIDE.md)** - 完整操作员指南（推荐）

### 🔧 技术文档
- **[AIRSTACK_GCS_COMPATIBILITY.md](AIRSTACK_GCS_COMPATIBILITY.md)** - ⭐ AirStack_GCS 消息格式兼容性说明
- **[TROUBLESHOOTING.md](TROUBLESHOOTING.md)** - ⭐ 故障排除完整指南
- **[CRITICAL_FIX.md](CRITICAL_FIX.md)** - clientPublish 配置说明
- **[USAGE.md](USAGE.md)** - 详细使用和故障排除
- **[QUICK_FIX.md](QUICK_FIX.md)** - 常见问题快速修复
- **[DEBUG_DATASOURCE.md](DEBUG_DATASOURCE.md)** - Data source 调试
- **[INTEGRATION_COMPLETE.md](INTEGRATION_COMPLETE.md)** - 系统集成总结

### 🏗️ 系统架构
- **[../airstack/ros_ws/src/behavior_tree_system/BEHAVIOR_TREE_INTEGRATION.md](../../airstack/ros_ws/src/behavior_tree_system/BEHAVIOR_TREE_INTEGRATION.md)** - 行为树系统架构（中文）

## 🎮 功能

### 支持的命令

**基础控制**:
- Arm - 解锁飞行器
- Disarm - 上锁飞行器
- Auto Takeoff - 自动起飞
- Auto Land - 自动降落
- EStop - 紧急停止

**任务控制**:
- Survey - 勘测任务
- Geofence Mapping - 地理围栏映射
- Go to Waypoint - 前往航点

## 📋 系统要求

- ROS2 Humble
- Foxglove Studio
- `foxglove_bridge` 包
- `behavior_tree_msgs` 消息包
- `behavior_executive` 节点

## 🛠️ 安装

### 开发安装

```bash
cd operator/behavior-tree-controller
npm install
npm run build
npm run local-install
```

### 打包发布

```bash
npm run package
# 生成 .foxe 文件在当前目录
```

## ⚡ 快速开始

```bash
# 1. 启动 Bridge (必须启用 clientPublish)
cd operator/behavior-tree-controller
./start_foxglove_bridge.sh

# 2. 启动系统
cd airstack/ros_ws
source install/setup.bash
ros2 launch robot_bringup robot.launch.xml

# 3. 在 Foxglove Studio 中连接
# ws://localhost:8765

# 4. 添加 "Behavior Tree Controller" 面板
```

## 🔍 诊断工具

```bash
# 诊断系统状态
./diagnose_foxglove.sh

# 测试命令发送（需要 Python）
./test_behavior_commands.sh
```

## 🚨 关键配置

### foxglove_bridge 必须启用 clientPublish

**正确**（已包含在脚本中）:
```bash
ros2 run foxglove_bridge foxglove_bridge \
  --ros-args \
  -p port:=8765 \
  -p capabilities:="[clientPublish]"
```

**错误**（不允许客户端发布）:
```bash
ros2 run foxglove_bridge foxglove_bridge --port 8765
```

## 📊 话题接口

### 发布话题

- **Topic**: `/behavior_tree_commands`
- **Type**: `behavior_tree_msgs/msg/BehaviorTreeCommands`
- **QoS**: Default

### 消息格式

```yaml
commands:
  - condition_name: "Arm Commanded"
    status: 2  # 2=激活, 0=取消
```

## 🐛 常见问题

### 重启后无法发布

**症状**: "topic has not been advertised before"

**解决**: 点击面板的 **"重新连接话题"** 按钮

### Data source 显示 "unknown"

**症状**: 扩展显示 `Data source: unknown`

**影响**: 无（代码已自动处理）

**优化**: 使用 "Foxglove WebSocket" 连接类型（不是 Rosbridge）

### Bridge 报错找不到 behavior_tree_msgs

**症状**: `package 'behavior_tree_msgs' not found`

**原因**: Bridge 未在正确的 ROS2 环境中启动

**解决**: 
```bash
# 不要直接运行
ros2 run foxglove_bridge foxglove_bridge  # ❌

# 使用脚本启动
./start_foxglove_bridge.sh  # ✅
```

## 🏗️ 开发

### 项目结构

```
behavior-tree-controller/
├── src/
│   ├── BehaviorTreeControllerPanel.tsx  # 主面板组件
│   └── index.ts                         # 扩展入口
├── package.json                         # 扩展配置
├── tsconfig.json                        # TypeScript 配置
├── eslint.config.js                     # Linting 配置
└── README.md                            # 本文件
```

### 构建和测试

```bash
# 开发构建
npm run build

# 本地安装（自动重新加载）
npm run local-install

# Lint 检查
npm run lint

# 修复 Lint 问题
npm run lint:fix
```

### 修改命令

编辑 `src/BehaviorTreeControllerPanel.tsx`:

```typescript
const BEHAVIOR_COMMANDS = [
  { name: "Arm Commanded", label: "Arm", description: "Arm the drone" },
  // 添加更多命令...
];
```

## 📄 许可

UNLICENSED

## 👥 维护者

Chiron MRSD Team

---

**开始使用**: 阅读 [OPERATOR_GUIDE.md](OPERATOR_GUIDE.md)  
**遇到问题**: 查看 [USAGE.md](USAGE.md) 的故障排除部分
