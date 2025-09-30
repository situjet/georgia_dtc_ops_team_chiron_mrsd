# 中心像素GPS估计节点

## 概述

`center_pixel_gps_node` 是一个新的ROS2节点，假设目标始终位于图像平面的中心，并持续对中心像素进行GPS坐标估计。与原始的基于YOLO检测的节点不同，这个节点不需要目标检测，直接对图像中心进行GPS估计。

## 主要特性

- **无需目标检测**: 假设目标始终在图像中心，不使用YOLO或其他检测算法
- **持续GPS估计**: 以可配置的频率持续估计中心像素的GPS坐标
- **实时可视化**: 在发布的图像上绘制中心十字标记
- **多相机支持**: 支持EO和IR相机模式切换
- **传感器数据同步**: 同步GPS、高度、航向和云台姿态数据
- **集群管理**: 管理估计的GPS目标集群

## 节点架构

```
输入:
├── /image_raw_compressed (sensor_msgs/CompressedImage) - 压缩图像流
├── /dtc_mrsd/mavros/global_position/global (sensor_msgs/NavSatFix) - GPS位置
├── /dtc_mrsd/mavros/global_position/rel_alt (std_msgs/Float64) - 相对高度
├── /dtc_mrsd/mavros/global_position/compass_hdg (std_msgs/Float64) - 航向
├── /gimbal_attitude (geometry_msgs/Vector3) - 云台姿态
└── /camera_mode (std_msgs/String) - 相机模式

输出:
├── /center_gps/image_compressed (sensor_msgs/CompressedImage) - 带中心标记的图像
├── /center_gps/camera_info (sensor_msgs/CameraInfo) - 相机信息
├── /center_gps/target_gps (sensor_msgs/NavSatFix) - 中心像素GPS估计
├── /center_gps/target_gps_list (sensor_msgs/NavSatFix) - GPS目标集群列表
├── /center_gps/target_local_enu (geometry_msgs/PointStamped) - 本地ENU坐标
└── /center_gps/center_marker (geometry_msgs/PointStamped) - 中心标记点
```

## 配置参数

主要配置参数在 `config/center_pixel_params.yaml` 中：

### 估计参数
- `estimation.frequency`: GPS估计频率 (Hz)
- `estimation.enable_continuous`: 是否启用持续估计

### 图像参数
- `image.width`: 图像宽度
- `image.height`: 图像高度

### 相机参数
- `camera.mode`: 相机模式 ('EO' 或 'IR')
- `camera_intrinsics`: 相机内参矩阵

### 输出参数
- `output.publish_center_marker`: 是否发布中心标记
- `output.sigma_lat_lon_m`: GPS估计不确定性 (米)

## 启动节点

### 使用默认配置启动
```bash
ros2 launch vision_gps_estimator center_pixel_gps.launch.py
```

### 使用自定义参数启动
```bash
ros2 launch vision_gps_estimator center_pixel_gps.launch.py \
    estimation_frequency:=15.0 \
    log_level:=DEBUG \
    robot_namespace:=dtc_mrsd_
```

### 启用MCAP记录
```bash
ros2 launch vision_gps_estimator center_pixel_gps.launch.py \
    record_mcap:=true \
    mcap_output_dir:=/path/to/logs
```

## 直接运行节点

```bash
ros2 run vision_gps_estimator center_pixel_gps_node \
    --ros-args --params-file config/center_pixel_params.yaml
```

## 工作原理

1. **图像接收**: 节点订阅压缩图像流
2. **传感器同步**: 同步GPS、高度、航向和云台姿态数据
3. **中心像素计算**: 计算图像中心坐标 (width/2, height/2)
4. **GPS估计**: 使用相机内参和传感器数据估计中心像素的GPS坐标
5. **结果发布**: 发布估计的GPS坐标和可视化图像

## GPS估计算法

节点使用与原始系统相同的GPS估计算法：

1. 将像素坐标转换为归一化相机坐标
2. 应用云台旋转变换
3. 计算射线与地面的交点
4. 转换为世界坐标系 (ENU)
5. 计算GPS坐标偏移

## 监控和调试

### 性能统计
节点定期记录性能统计信息：
- 处理的帧数
- GPS估计次数
- 同步失败次数
- 平均处理时间

### 话题监控
```bash
# 监控GPS估计结果
ros2 topic echo /center_gps/target_gps

# 监控中心标记
ros2 topic echo /center_gps/center_marker

# 检查图像流
ros2 run rqt_image_view rqt_image_view /center_gps/image_compressed
```

### 参数调整
```bash
# 运行时调整估计频率
ros2 param set /center_pixel_gps_estimator estimation.frequency 20.0

# 切换相机模式
ros2 topic pub /camera_mode std_msgs/String "data: IR"
```

## 与原始节点的区别

| 特性 | 原始节点 (integrated_node) | 中心像素节点 (center_pixel_gps_node) |
|------|---------------------------|-------------------------------------|
| 目标检测 | YOLO检测 | 假设中心位置 |
| 处理复杂度 | 高 (AI推理) | 低 (直接计算) |
| 依赖项 | Ultralytics YOLO | 仅基础库 |
| 检测精度 | 依赖检测质量 | 固定中心位置 |
| 计算资源 | GPU密集型 | CPU轻量级 |
| 实时性能 | 受检测速度限制 | 高频率估计 |

## 故障排除

### 常见问题

1. **无GPS估计输出**
   - 检查传感器数据是否正常接收
   - 验证相机内参配置
   - 确认云台姿态数据有效

2. **同步失败**
   - 调整 `sync.max_sync_time_diff` 参数
   - 检查传感器数据时间戳

3. **估计精度问题**
   - 校准相机内参
   - 验证云台姿态校准
   - 检查高度数据准确性

### 日志级别
设置详细日志以进行调试：
```bash
ros2 launch vision_gps_estimator center_pixel_gps.launch.py log_level:=DEBUG
```

## 文件结构

```
vision_gps_estimator/
├── vision_gps_estimator/
│   ├── center_pixel_gps_node.py      # 主节点实现
│   ├── gps_manager.py                # GPS估计算法 (共享)
│   └── integrated_node.py            # 原始YOLO节点 (保留)
├── config/
│   ├── center_pixel_params.yaml      # 中心像素节点配置
│   └── params.yaml                   # 原始节点配置 (保留)
├── launch/
│   ├── center_pixel_gps.launch.py    # 中心像素节点启动文件
│   └── vision_gps_estimator.launch.py # 原始节点启动文件 (保留)
└── README_CENTER_PIXEL.md            # 本文档
```

## 未来改进

- 支持多目标假设 (例如，网格中的多个点)
- 添加目标区域配置 (而不是固定中心)
- 集成运动预测算法
- 支持动态相机内参校准

