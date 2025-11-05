# 航点导航快速开始指南

## 🔄 重新加载扩展以看到新功能

**如果你看不到航点选择功能，需要重新加载扩展：**

### 方法 1: 在 Foxglove 中重新加载（推荐）

1. 打开 Foxglove Studio
2. 右上角点击 **Extensions** (扩展图标)
3. 找到 **Geofence & Human GPS Visualizer**
4. 点击 **卸载** (Uninstall)
5. 关闭 Foxglove Studio
6. 重新打开 Foxglove Studio
7. Extensions → **Import extension**
8. 选择：`operator/geofence-human-visualizer/dist/extension.js`
9. 添加面板到布局

### 方法 2: 直接导入新版本

1. 在 Foxglove 中，Extensions → **Import extension**
2. 选择：`operator/geofence-human-visualizer/dist/extension.js`
3. 如果提示已存在，选择 **替换**

---

## 🗺️ 使用航点导航功能

### 第一步：在地图上选择航点

1. **打开两个面板**：
   - `Geofence & Human GPS Visualizer` (地图面板)
   - `Behavior Tree Controller` (命令面板)

2. **在地图上点击**：
   - 点击地图上的任意位置
   - 会出现一个**蓝色的大圆圈**，里面有白色字母 **"W"**
   - 顶部状态栏会显示：📍 Selected Waypoint: lat, lon
   - 弹出窗口显示坐标和提示

### 第二步：发送导航命令

3. **切换到 Behavior Tree Controller 面板**
4. **点击 "Navigate to Waypoint" 按钮**
5. 无人机开始飞往选定的航点

---

## 🎯 功能特性

### 地图类型
- ✅ **卫星地图** (Esri World Imagery)
- 高分辨率卫星影像
- **自动居中到无人机GPS位置**
- **最大缩放级别 19**（~0.3m/像素精度）
- 适合小区域（< 100m）精确导航

### 地图标记
- 🔴 **红色圆点** - 地理围栏边界点
- 🟡 **黄色 "L"** - 目标 GPS 列表
- 🟢 **绿色 "P"** - 精确目标 GPS
- 🟠 **橙色 "✈"** - **无人机当前位置**（实时更新）← 新功能！
- 🔵 **蓝色 "W"** - **选定的航点** ← 新功能！

### 缩放按钮（右上角）
- **✈ Drone** - 快速缩放到无人机位置（级别 18）
- **📍 Waypoint** - 快速缩放到选定航点（级别 18）

### 两个话题协同工作

```
┌─────────────────────────────────────┐
│  地图面板                             │
│  点击地图 → 出现蓝色 "W" 标记          │
│  ↓                                   │
│  发布到 /selected_waypoint           │
└─────────────────────────────────────┘
              ↓
┌─────────────────────────────────────┐
│  behavior_executive (后台运行)        │
│  接收并存储航点坐标                   │
└─────────────────────────────────────┘
              ↓
┌─────────────────────────────────────┐
│  Behavior Tree Controller 面板       │
│  点击 "Navigate to Waypoint"         │
│  ↓                                   │
│  发布到 /behavior_tree_commands      │
└─────────────────────────────────────┘
              ↓
         无人机导航到目标
```

---

## 🔍 故障排除

### Q: 地图刷新频率太高，无法点击？

**✅ 已修复！** 
- 现已优化为 10 Hz 更新频率（每 100ms）
- 点击响应流畅
- 缩放操作平滑

**如果仍有问题**：
1. 重新加载扩展（见上方说明）
2. 检查 `operator/geofence-human-visualizer/PERFORMANCE.md` 性能文档

### Q: 我点击地图没有出现蓝色 "W" 标记？

**检查项**：
1. ✅ 确认已重新构建：`npm run build`
2. ✅ 确认已重新加载扩展到 Foxglove
3. ✅ 检查浏览器控制台 (F12) 查看错误
4. ✅ 确认 Foxglove Bridge 已启用 `clientPublish`

**查看日志**：
```
打开浏览器控制台 (F12)
应该看到：
[GeofenceMap] Advertised topic: /selected_waypoint
[GeofenceMap] Published waypoint: {lat: ..., lon: ..., alt: 10}
```

### Q: 发布航点但无人机没反应？

**检查项**：
1. ✅ 确认 `behavior_executive` 节点正在运行
2. ✅ 检查话题是否被订阅：
   ```bash
   ros2 topic info /selected_waypoint
   ```
3. ✅ 手动监听话题验证：
   ```bash
   ros2 topic echo /selected_waypoint
   ```
4. ✅ 确认点击了 "Navigate to Waypoint" 按钮

### Q: 如何修改默认高度？

编辑 `src/GeofenceHumanPanel.tsx` 第 191 行：
```typescript
publishSelectedWaypoint(lat, lng, 10.0); // 修改这里的高度值（米）
```

然后重新构建：
```bash
npm run build
```

---

## 📋 完整测试流程

```bash
# 1. 启动 Foxglove Bridge
ros2 run foxglove_bridge foxglove_bridge \
  --ros-args -p port:=8765 -p capabilities:="[clientPublish]"

# 2. 启动 behavior_executive
ros2 launch robot_bringup robot.launch.xml

# 3. 在另一个终端监听话题（用于验证）
ros2 topic echo /selected_waypoint
```

在 Foxglove Studio 中：
1. 连接到 ws://localhost:8765
2. 添加两个面板：
   - Geofence & Human GPS Visualizer
   - Behavior Tree Controller
3. 点击地图选择航点
4. 在监听终端应该看到消息
5. 点击 "Navigate to Waypoint" 按钮

---

## 🎥 预期效果

### 正确的视觉反馈：

1. **点击地图时**：
   - 立即出现蓝色大圆圈（30x30px）
   - 圆圈内有白色加粗字母 "W"
   - 有白色边框和阴影效果
   - 自动弹出信息窗口

2. **状态栏更新**：
   ```
   📍 Selected Waypoint: 40.414000, -79.947000 → Now click "Navigate to Waypoint" in Behavior Tree Controller
   ```

3. **控制台日志** (F12)：
   ```
   [GeofenceMap] Advertised topic: /selected_waypoint
   [GeofenceMap] Published waypoint: {lat: 40.414, lon: -79.947, alt: 10}
   ```

如果看到这些，说明功能正常！🎉

---

## 📞 需要帮助？

检查这些文件的日志输出：
- **浏览器控制台** (F12) - 前端 JavaScript 日志
- **ROS 日志** - `ros2 topic echo /rosout`
- **behavior_executive 日志** - 查看是否接收到航点

