# 中心像素GPS估计节点使用指南

## 快速开始

### 1. 构建包
```bash
cd /home/lance/mount/airstack/ros_ws
colcon build --packages-select vision_gps_estimator
source install/setup.bash
```

### 2. 启动节点
```bash
# 使用默认配置启动
ros2 launch vision_gps_estimator center_pixel_gps.launch.py

# 或者使用自定义频率
ros2 launch vision_gps_estimator center_pixel_gps.launch.py estimation_frequency:=10.0
```

### 3. 测试节点（可选）
在另一个终端中运行测试脚本：
```bash
cd /home/lance/mount/airstack/ros_ws
source install/setup.bash
python3 src/vision_gps_estimator/scripts/test_center_pixel_node.py
```

## 节点对比

### 原始节点 (integrated_node)
- 使用YOLO检测目标
- 需要GPU资源
- 检测精度依赖于模型质量
- 话题前缀: `/vision_gps/`

### 新节点 (center_pixel_gps_node)  
- 假设目标在图像中心
- 轻量级，仅需CPU
- 固定中心位置估计
- 话题前缀: `/center_gps/`

## 主要话题

### 输入话题
- `/image_raw_compressed` - 图像流
- `/dtc_mrsd/mavros/global_position/global` - GPS位置
- `/dtc_mrsd/mavros/global_position/rel_alt` - 高度
- `/dtc_mrsd/mavros/global_position/compass_hdg` - 航向
- `/gimbal_attitude` - 云台姿态

### 输出话题
- `/center_gps/target_gps` - 中心像素GPS估计
- `/center_gps/image_compressed` - 带中心标记的图像
- `/center_gps/center_marker` - 中心标记点

## 监控命令

```bash
# 查看GPS估计结果
ros2 topic echo /center_gps/target_gps

# 查看图像（需要安装rqt_image_view）
ros2 run rqt_image_view rqt_image_view /center_gps/image_compressed

# 检查节点状态
ros2 node info /center_pixel_gps_estimator

# 查看所有话题
ros2 topic list | grep center_gps
```

## 配置调整

### 运行时参数调整
```bash
# 调整估计频率
ros2 param set /center_pixel_gps_estimator estimation.frequency 10.0

# 查看所有参数
ros2 param list /center_pixel_gps_estimator
```

### 配置文件修改
编辑 `config/center_pixel_params.yaml` 文件，然后重启节点。

## 故障排除

### 1. 节点无法启动
```bash
# 检查依赖
ros2 pkg list | grep vision_gps_estimator

# 重新构建
colcon build --packages-select vision_gps_estimator --cmake-clean-cache
```

### 2. 无GPS输出
```bash
# 检查输入话题
ros2 topic list | grep mavros
ros2 topic echo /dtc_mrsd/mavros/global_position/global --once

# 检查图像流
ros2 topic echo /image_raw_compressed --once
```

### 3. 同步问题
调整同步参数：
```bash
ros2 param set /center_pixel_gps_estimator sync.max_sync_time_diff 0.2
```

## 性能优化

### 调整估计频率
```bash
# 降低频率以减少CPU使用
ros2 param set /center_pixel_gps_estimator estimation.frequency 5.0

# 提高频率以获得更高更新率
ros2 param set /center_pixel_gps_estimator estimation.frequency 10.0
```

### 禁用可选输出
在配置文件中设置：
```yaml
output:
  publish_local_enu: false
  publish_center_marker: false
```

## 与原始系统集成

两个节点可以同时运行，使用不同的话题前缀：

```bash
# 终端1: 启动原始YOLO节点
ros2 launch vision_gps_estimator vision_gps_estimator.launch.py

# 终端2: 启动中心像素节点  
ros2 launch vision_gps_estimator center_pixel_gps.launch.py
```

输出对比：
- YOLO节点: `/vision_gps/target_gps`
- 中心像素节点: `/center_gps/target_gps`
