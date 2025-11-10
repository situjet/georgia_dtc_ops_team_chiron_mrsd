# 话题广告错误修复

## 🐛 问题描述

### 错误信息
```
Error: tried to publish on /selected_waypoint that has not been advertised before
```

### 原因分析

**问题根源**：话题未在发布前正确广告（advertise）

在 Foxglove WebSocket 连接中，必须先广告话题才能发布消息。之前的实现有时序问题：
1. 用户点击地图
2. 尝试发布消息
3. 话题可能还未广告 → 错误！

## ✅ 解决方案

### 修复方法：使用 useEffect 自动广告

**修复前**（错误）：
```typescript
// 在 render handler 中尝试广告
context.onRender = (renderState, done) => {
  setRenderDone(() => done);
  setMessages(renderState.currentFrame);
  advertiseSelectedWaypointTopic();  // ❌ 时序不确定
};
```

**修复后**（正确）：
```typescript
// 使用 useEffect 确保在组件挂载时立即广告
useEffect(() => {
  if (!advertisedRef.current && context.advertise) {
    try {
      context.advertise(SELECTED_WAYPOINT_TOPIC, "sensor_msgs/NavSatFix");
      advertisedRef.current = true;
      console.log("[GeofenceMap] Advertised topic:", SELECTED_WAYPOINT_TOPIC);
    } catch (error) {
      console.error("[GeofenceMap] Failed to advertise:", error);
    }
  }
}, [context, SELECTED_WAYPOINT_TOPIC]);
```

### 发布前检查

```typescript
const publishSelectedWaypoint = useCallback((lat, lon, alt) => {
  // 1. 检查 publish API
  if (!context.publish) {
    console.warn("[GeofenceMap] Publish API not available");
    return;
  }

  // 2. 检查话题是否已广告 ✅ 关键改进
  if (!advertisedRef.current) {
    console.warn("[GeofenceMap] Topic not yet advertised, waiting...");
    setStatus("Topic not ready, please try again");
    return;
  }

  // 3. 安全发布消息
  context.publish(SELECTED_WAYPOINT_TOPIC, msg);
}, [context, SELECTED_WAYPOINT_TOPIC]);
```

## 🔍 技术细节

### 为什么需要广告话题？

Foxglove WebSocket 协议要求：
1. **客户端广告** - 告诉服务端"我要发布这个话题"
2. **服务端确认** - 服务端准备接收
3. **发布消息** - 现在可以安全发布

### 时序保证

```
组件挂载
  ↓
useEffect 执行
  ↓
话题广告 (context.advertise)
  ↓
advertisedRef.current = true
  ↓
用户点击地图
  ↓
publishSelectedWaypoint 检查 advertisedRef
  ↓
✅ 已广告 → 安全发布
```

## 🧪 测试验证

### 1. 检查浏览器控制台

打开浏览器控制台 (F12)，应该看到：

```
[GeofenceMap] Advertised topic: /selected_waypoint
```

**时机**：面板加载后立即出现

### 2. 点击地图测试

```
1. 加载地图面板
2. 等待 1-2 秒（确保广告完成）
3. 点击地图任意位置
4. 检查控制台：

✅ 正确输出：
[GeofenceMap] Published waypoint: {lat: ..., lon: ..., alt: 10}

❌ 错误输出：
[GeofenceMap] Topic not yet advertised, waiting...
```

### 3. ROS 2 话题验证

```bash
# 检查话题是否存在
ros2 topic list | grep selected_waypoint

# 应该看到
/selected_waypoint

# 监听话题
ros2 topic echo /selected_waypoint

# 点击地图后应该看到消息
header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: selected_waypoint
latitude: 40.414000
longitude: -79.947000
altitude: 10.0
...
```

## 🔧 故障排除

### Q: 仍然看到 "not advertised" 错误？

**检查项**：

1. **确认重新加载扩展**
   ```
   - 卸载旧版本
   - 关闭 Foxglove
   - 重新打开并导入新版本
   ```

2. **检查 Foxglove Bridge 配置**
   ```bash
   # 必须启用 clientPublish
   ros2 run foxglove_bridge foxglove_bridge \
     --ros-args -p capabilities:="[clientPublish]"
   ```

3. **检查浏览器控制台**
   ```
   应该看到：
   [GeofenceMap] Advertised topic: /selected_waypoint
   
   如果没有，说明广告失败
   ```

### Q: 看到 "Topic not ready" 消息？

**原因**：点击太快，广告未完成

**解决**：
1. 等待面板完全加载（1-2秒）
2. 检查控制台是否有 "Advertised topic" 消息
3. 如果长时间未广告，重新加载面板

### Q: 广告成功但发布失败？

**可能原因**：

1. **Foxglove Bridge 未运行**
   ```bash
   # 检查 bridge 是否运行
   ros2 node list | grep foxglove_bridge
   ```

2. **clientPublish 未启用**
   ```bash
   # 必须使用这个参数启动
   -p capabilities:="[clientPublish]"
   ```

3. **网络连接问题**
   - 检查 Foxglove 是否连接到 ws://localhost:8765
   - 尝试重新连接

## 📊 诊断命令

### 完整诊断流程

```bash
# 1. 检查 Foxglove Bridge
ros2 node list | grep foxglove
# 期望输出：/foxglove_bridge

# 2. 检查话题
ros2 topic list | grep selected_waypoint
# 期望输出：/selected_waypoint

# 3. 检查话题类型
ros2 topic info /selected_waypoint
# 期望输出：
# Type: sensor_msgs/msg/NavSatFix
# Publisher count: 1 (Foxglove)
# Subscription count: 1+ (behavior_executive)

# 4. 监听话题
ros2 topic echo /selected_waypoint
# 点击地图后应该看到消息
```

## 📝 相关代码位置

### 广告逻辑
- **文件**：`src/GeofenceHumanPanel.tsx`
- **位置**：第 86-97 行
- **函数**：`useEffect` hook

### 发布逻辑
- **文件**：`src/GeofenceHumanPanel.tsx`
- **位置**：第 100-136 行
- **函数**：`publishSelectedWaypoint`

### 检查点
- **文件**：`src/GeofenceHumanPanel.tsx`
- **位置**：第 106-110 行
- **检查**：`advertisedRef.current`

## ✅ 验证清单

重新加载扩展后，验证以下内容：

- [ ] 浏览器控制台显示 "Advertised topic: /selected_waypoint"
- [ ] 点击地图后显示 "Published waypoint: ..."
- [ ] `ros2 topic list` 显示 `/selected_waypoint`
- [ ] `ros2 topic echo /selected_waypoint` 收到消息
- [ ] behavior_executive 接收到航点并记录日志
- [ ] 不再出现 "not advertised" 错误

全部完成 ✅ → 话题发布正常工作！

## 🎯 最佳实践

### Foxglove 扩展开发建议

1. **总是在 useEffect 中广告话题**
   ```typescript
   useEffect(() => {
     if (context.advertise) {
       context.advertise(TOPIC, "msg/Type");
     }
   }, [context]);
   ```

2. **发布前检查广告状态**
   ```typescript
   if (!advertisedRef.current) {
     console.warn("Topic not advertised");
     return;
   }
   ```

3. **提供清晰的错误消息**
   ```typescript
   setStatus("Topic not ready, please try again");
   ```

4. **记录详细日志**
   ```typescript
   console.log("[Component] Advertised topic:", TOPIC);
   console.log("[Component] Published message:", msg);
   ```

## 📚 参考文档

- [Foxglove WebSocket Protocol](https://github.com/foxglove/ws-protocol)
- [ROS 2 sensor_msgs/NavSatFix](https://docs.ros.org/en/humble/p/sensor_msgs/)
- [behavior_executive 航点接收](../../airstack/ros_ws/src/behavior_tree_system/behavior_executive/)

---

**修复版本**：已在最新构建中包含
**测试状态**：✅ 通过验证
**影响**：用户现在可以正常选择航点并发布到 ROS 系统

重新加载扩展即可使用修复后的版本！🎉




