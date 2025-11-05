# 自动居中到无人机功能

## 🎯 功能说明

地图现在会自动居中到无人机GPS位置，并使用最高缩放级别，适合小区域精确飞行操作。

## ✨ 新功能

### 1. **最大缩放级别**
- **初始缩放**: 19 级（最高精度）
- **适用场景**: 小区域（< 100m x 100m）精确操作
- **地面分辨率**: 约 0.3m/像素

### 2. **自动居中到无人机**
- 收到第一个GPS位置后自动居中
- 只在首次接收时居中（避免干扰用户操作）
- 保持缩放级别 19

## 📊 缩放级别对比

| 缩放级别 | 地面分辨率 | 可见范围 | 适用场景 |
|---------|-----------|---------|---------|
| 15 | ~4.8m/像素 | ~5km x 5km | 大区域概览 |
| 17 | ~1.2m/像素 | ~1.2km x 1.2km | 中等区域 |
| 18 | ~0.6m/像素 | ~600m x 600m | 小区域 |
| **19** | **~0.3m/像素** | **~300m x 300m** | **精确小区域** ← 当前 |

## 🔄 工作流程

```
1. 面板加载
   ↓
2. 地图初始化（默认位置，缩放19）
   ↓
3. 订阅无人机GPS话题 (/dtc_mrsd_/mavros/global_position/global)
   ↓
4. 收到第一个GPS位置
   ↓
5. 自动移动地图到无人机位置（缩放19）
   ↓
6. 显示橙色✈标记
   ↓
7. 用户可以自由操作地图
```

## 🎮 用户体验

### 启动序列

```
加载面板
  ↓ 1秒
[地图显示默认位置，缩放19]
  ↓ 1-2秒（等待GPS）
[地图自动跳转到无人机位置]
  ↓
状态显示: "Map centered on drone: 40.414123, -79.947456"
  ↓
[显示橙色✈标记]
  ↓
用户可以开始选择航点
```

### 状态消息

1. **初始化**: "Initializing..."
2. **居中**: "Map centered on drone: lat, lon"
3. **后续**: 显示其他操作状态

## 🔧 配置

### 修改初始缩放级别

编辑 `src/GeofenceHumanPanel.tsx` 第 189 行：

```typescript
const map = L.map(el, {
  center: [40.414, -79.947],
  zoom: 19,  // ← 修改这里 (15-19)
  zoomControl: true,
});
```

**推荐值**：
- **19**: 最精确（< 100m区域）
- **18**: 精确（100-300m区域）
- **17**: 中等（300-600m区域）
- **15**: 概览（> 1km区域）

### 居中缩放级别

编辑 `src/GeofenceHumanPanel.tsx` 第 365 行：

```typescript
// Center map on drone position on first GPS fix
if (!mapCenteredOnDroneRef.current && mapRef.current) {
  mapRef.current.setView([newLat, newLon], 19);  // ← 修改缩放级别
  mapCenteredOnDroneRef.current = true;
  // ...
}
```

## 🧪 测试验证

### 1. 启动测试

```bash
# 1. 启动系统（确保MAVROS在运行）
ros2 launch robot_bringup robot.launch.xml

# 2. 检查GPS话题
ros2 topic hz /dtc_mrsd_/mavros/global_position/global
# 应该看到 ~1-10 Hz

# 3. 检查GPS数据
ros2 topic echo /dtc_mrsd_/mavros/global_position/global --once
```

### 2. Foxglove 测试

```
1. 打开 Foxglove Studio
2. 连接到 ws://localhost:8765
3. 添加 "Geofence & Human GPS Visualizer" 面板
4. 观察：
   ✅ 地图以缩放19加载
   ✅ 1-2秒后自动跳转到无人机位置
   ✅ 显示橙色✈标记
   ✅ 状态显示 "Map centered on drone: ..."
```

### 3. 浏览器控制台验证

打开浏览器控制台 (F12)，应该看到：

```
[GeofenceMap] Advertised topic: /selected_waypoint
[GeofenceMap] Map centered on drone: 40.414123 -79.947456
```

## 📍 默认位置

### 后备位置
如果无人机GPS不可用，使用默认位置：
- **纬度**: 40.414
- **经度**: -79.947
- **位置**: Pittsburgh, Pennsylvania (CMU附近)

### 修改默认位置

编辑 `src/GeofenceHumanPanel.tsx` 第 188 行：

```typescript
const map = L.map(el, {
  center: [40.414, -79.947],  // ← [lat, lon]
  zoom: 19,
  zoomControl: true,
});
```

## 🔍 行为特点

### ✅ 只居中一次
- 只在收到第一个GPS位置时居中
- 后续GPS更新不会移动地图
- **原因**: 避免干扰用户手动操作

### ✅ 标记持续更新
- 无人机位置标记持续更新
- 即使地图不自动跟随
- 可以点击 "✈ Drone" 按钮手动跟随

### ✅ 保持用户控制
```
用户拖动地图
  ↓
地图保持在用户选择的位置
  ↓
无人机标记继续更新
  ↓
用户可以点击 "✈ Drone" 按钮回到无人机
```

## 🎯 使用场景

### 场景 1: 小区域精确任务
```
1. 启动系统
2. 地图自动居中到无人机（缩放19）
3. 清晰看到周围建筑物和细节
4. 点击地图选择精确航点（精度 ~30cm）
5. 发送 "Navigate to Waypoint" 命令
```

### 场景 2: 跟踪无人机飞行
```
1. 地图自动居中到无人机
2. 观察橙色✈标记移动
3. 如果需要跟随，点击 "✈ Drone" 按钮
4. 地图重新居中并缩放到无人机
```

### 场景 3: 大区域操作
```
1. 地图自动居中到无人机（缩放19）
2. 如果需要看更大范围：
   - 使用鼠标滚轮缩小
   - 或拖动地图到目标区域
3. 选择远处的航点
4. 点击 "📍 Waypoint" 查看选定位置
```

## 📊 性能考虑

### 高缩放级别的影响

**优点**：
- ✅ 精确选择航点（< 50cm精度）
- ✅ 清晰看到地面细节
- ✅ 适合小区域操作

**可能的问题**：
- ⚠️ 加载更多地图图块
- ⚠️ 初始加载稍慢（通常 < 2秒）

**优化**：
- 使用节流（10 Hz更新）
- 只在位置变化时更新
- 缓存地图图块

## 🔧 故障排除

### Q: 地图没有自动居中到无人机？

**检查**：
1. 无人机GPS话题是否发布？
   ```bash
   ros2 topic hz /dtc_mrsd_/mavros/global_position/global
   ```

2. GPS数据是否有效（非0）？
   ```bash
   ros2 topic echo /dtc_mrsd_/mavros/global_position/global --once
   ```

3. 浏览器控制台是否有错误？
   - 打开 F12
   - 查找 "Map centered on drone" 消息

### Q: 地图居中到错误的位置？

**可能原因**：
- GPS数据无效（0, 0）
- GPS精度不足
- 默认位置显示

**解决**：
1. 等待GPS固定（通常需要30-60秒）
2. 确保无人机在户外（GPS信号好）
3. 点击 "✈ Drone" 按钮手动居中

### Q: 缩放级别19太大/太小？

**调整方法**：
1. 编辑 `src/GeofenceHumanPanel.tsx`
2. 修改第 189 和 365 行的缩放值
3. 重新构建：`npm run build`
4. 重新加载扩展

## ✅ 功能清单

重新加载扩展后，验证：

- [ ] 地图以缩放19加载
- [ ] 1-2秒后自动跳转到无人机位置
- [ ] 显示 "Map centered on drone: ..." 状态
- [ ] 橙色✈标记出现在无人机位置
- [ ] 可以清晰看到地面细节
- [ ] 点击地图可以精确选择航点
- [ ] "✈ Drone" 按钮可以回到无人机

全部完成 ✅ → 自动居中功能正常工作！

## 📚 相关文档

- `ICON_SIZE_GUIDE.md` - 图标大小调整
- `PERFORMANCE.md` - 性能优化
- `TOPIC_ADVERTISE_FIX.md` - 话题广告修复
- `QUICK_START.md` - 快速开始指南

---

**新功能**：自动居中 + 最大缩放
**适用场景**：小区域（< 100m）精确飞行操作
**用户体验**：立即看到无人机位置，无需手动搜索

重新加载扩展即可体验！🚁🎯




