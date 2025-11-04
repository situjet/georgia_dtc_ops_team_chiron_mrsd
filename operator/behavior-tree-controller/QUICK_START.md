# 🚀 快速开始指南

## 3 步启动

### 1️⃣ 启动系统（终端）

```bash
# 终端 1: 启动 Foxglove Bridge
cd operator/behavior-tree-controller
./start_foxglove_bridge.sh

# 终端 2: 启动机器人系统
cd airstack/ros_ws
source install/setup.bash
ros2 launch robot_bringup robot.launch.xml
```

### 2️⃣ 连接 Foxglove Studio

1. 打开 Foxglove Studio
2. **Open connection** → **Foxglove WebSocket**
3. URL: `ws://localhost:8765`
4. 点击 **Open**

### 3️⃣ 添加控制面板

1. 点击 **"+"** 添加面板
2. 选择 **"Behavior Tree Controller"**
3. ✅ 完成！

## ✈️ 典型飞行流程

```
[Arm] → [Auto Takeoff] → [任务] → [Auto Land] → [Disarm]
```

## 🔧 重启后恢复

系统或节点重启后：

1. 点击面板顶部的 **"重新连接话题"** 按钮
2. 等待显示 "已连接到: /behavior_tree_commands"
3. 继续操作

## ⚠️ 紧急情况

任何时候点击 **[EStop]** 立即停止

---

**详细文档**: `OPERATOR_GUIDE.md`

