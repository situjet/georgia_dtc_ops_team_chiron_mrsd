# Geofence & Human GPS Visualizer + Waypoint Selector

Foxglove Studio 扩展，用于在交互式地图上可视化地理围栏边界和检测到的人员 GPS 坐标。

## 功能

- 可视化地理围栏边界
- 显示检测到的人员 GPS 位置
- **⭐ 点击地图选择航点并发送给导航系统**
- **🎯 自动居中到无人机GPS位置（缩放19）**
- **🔍 最大缩放级别适合小区域精确飞行**
- 交互式卫星地图界面

## ✈️ 航点导航工作流程

### 两个话题配合使用：

1. **`/selected_waypoint`** - 目标航点位置（本扩展发布）
   - 类型：`sensor_msgs/NavSatFix`
   - 用途：在地图上点击选择目标位置

2. **`/behavior_tree_commands`** - 导航命令（Behavior Tree Controller 发布）
   - 类型：`behavior_tree_msgs/msg/BehaviorTreeCommands`
   - 用途：激活"Navigate to Waypoint"命令

### 使用步骤：

1. **在本面板（地图）上点击选择航点**
   - 点击地图上的任意位置
   - 会出现蓝色的"W"标记
   - 航点自动发送到 `/selected_waypoint` 话题
   - behavior_executive 自动接收并设置目标位置

2. **在 Behavior Tree Controller 面板点击 "Navigate to Waypoint"**
   - 发送导航命令到 `/behavior_tree_commands`
   - 无人机开始飞往选定的航点

### 示例：

```
地图上点击 (40.414, -79.947) 
  ↓
发布到 /selected_waypoint 
  ↓
behavior_executive 接收并存储航点
  ↓
点击 "Navigate to Waypoint" 按钮
  ↓
发送命令到 /behavior_tree_commands
  ↓
无人机飞往目标位置
```

## 安装

```bash
npm install
npm run build
npm run local-install
```

## 使用

1. 在 Foxglove Studio 中添加 "Geofence & Human GPS Visualizer" 面板
2. 连接到 ROS2 系统
3. 面板将自动订阅相关话题并显示数据
4. **点击地图任意位置选择航点**
5. 在 Behavior Tree Controller 面板点击 "Navigate to Waypoint" 执行导航

## 订阅的话题

- `/robot_1/mavros/geofence/fences` - 地理围栏边界
- `/target_gps` - 目标 GPS 位置
- `/target_gps_list` - 目标 GPS 列表
- `/precise_target_gps` - 精确目标 GPS

## 发布的话题

- `/selected_waypoint` - 地图选择的航点（`sensor_msgs/NavSatFix`）

## 地图标记说明

- 🔴 **红色点**：地理围栏边界点
- 🟡 **黄色 "L"**：目标 GPS 列表检测点
- 🟢 **绿色 "P"**：精确目标 GPS
- 🟠 **橙色 "✈"**：无人机当前位置（实时更新）
- 🔵 **蓝色 "W"**：选定的航点（点击地图设置）

## 🔍 缩放功能

面板顶部有两个缩放按钮：

1. **✈ Drone** - 缩放到无人机当前位置
   - 自动跟踪无人机
   - 缩放级别：18（高精度）
   
2. **📍 Waypoint** - 缩放到选定的航点
   - 仅在选择航点后可用
   - 快速查看目标位置

### 使用缩放功能

```
1. 点击 "✈ Drone" → 地图自动移动到无人机位置并放大
2. 在地图上点击选择航点 → "📍 Waypoint" 按钮变为可用
3. 点击 "📍 Waypoint" → 地图自动移动到航点位置并放大
```

### 其他缩放方式

- **鼠标滚轮**：缩放地图
- **左上角 +/- 按钮**：Leaflet 默认缩放控制
- **双击地图**：放大
- **Shift + 拖动**：框选缩放

## 配合使用

此扩展与 **Behavior Tree Controller** 扩展配合使用实现完整的航点导航功能：

1. 使用本扩展选择目标位置
2. 使用 Behavior Tree Controller 激活导航命令
3. 无人机自动飞往目标位置

## 许可

UNLICENSED

