# GPS估计节点调试总结

## 为什么没有输出估计结果？

根据代码分析，GPS估计没有输出的主要原因可能包括：

### 1. **Burst模式未激活** (最可能的原因)
- GPS估计只有在 `burst_mode_enabled=True` 时才会执行
- 需要通过 `/burst_mode/control` 话题发送 `True` 来激活burst模式
- 如果没有收到burst模式激活信号，估计循环会跳过所有处理

### 2. **传感器数据缺失**
- 需要同时接收以下传感器数据：
  - GPS数据: `/dtc_mrsd_/mavros/global_position/global`
  - 高度数据: `/dtc_mrsd_/mavros/global_position/rel_alt`
  - 航向数据: `/dtc_mrsd_/mavros/global_position/compass_hdg`
  - 云台姿态: `/gimbal_attitude`
- 如果任何一个传感器数据缺失，GPS估计会被跳过

### 3. **图像数据缺失**
- 需要接收压缩图像数据: `/image_raw_compressed`
- 如果没有图像数据，估计循环无法进行

### 4. **时间同步问题**
- 传感器数据与图像帧的时间差超过0.5秒阈值
- 可能需要调整同步时间阈值参数

## 调试步骤

1. **检查burst模式状态**
   ```bash
   ros2 topic pub /burst_mode/control std_msgs/Bool "data: true"
   ```

2. **检查传感器话题是否存在**
   ```bash
   ros2 topic list | grep -E "(mavros|gimbal|image)"
   ```

3. **检查传感器数据是否发布**
   ```bash
   ros2 topic echo /dtc_mrsd_/mavros/global_position/global
   ros2 topic echo /image_raw_compressed
   ```

4. **查看节点日志**
   - 现在已添加详细的调试信息
   - 会显示接收到的传感器数据数量和缓冲区状态
   - 会显示为什么跳过GPS估计的具体原因

## 添加的调试信息

- 估计循环启动状态
- 传感器数据接收统计
- 图像帧接收统计
- burst模式状态变化
- 传感器数据同步状态
- GPS估计执行状态

现在运行节点时，会看到详细的调试信息来帮助诊断问题。
