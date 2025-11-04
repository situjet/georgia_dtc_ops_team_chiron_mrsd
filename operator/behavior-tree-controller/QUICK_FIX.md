# 快速修复：节点重启后无法发布

## 问题
重启 ROS2 节点或 `foxglove_bridge` 后，扩展报错 "topic has not been advertised before"

## 原因
- 扩展缓存了广告状态
- 服务端（foxglove_bridge）重启后状态丢失
- 扩展以为已广告，实际上服务端已经忘记了

## 解决方案（3 选 1）

### 🎯 方法 1：点击"重新连接话题"按钮（最快）
在扩展面板顶部点击 **"重新连接话题"** 按钮

### 方法 2：断开重连
1. Foxglove Studio → 断开连接
2. 重新连接到 `ws://localhost:8765`

### 方法 3：重新加载扩展
关闭并重新打开行为树控制器面板

## 预防措施

始终使用脚本启动 foxglove_bridge：
```bash
cd operator/behavior-tree-controller
./start_foxglove_bridge.sh
```

**不要直接运行**：
```bash
ros2 run foxglove_bridge foxglove_bridge  # ❌ 错误！会找不到 behavior_tree_msgs
```
