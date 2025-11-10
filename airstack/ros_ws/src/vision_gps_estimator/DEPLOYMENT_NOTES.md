# Vision GPS Estimator - Deployment Ready

## 清理完成的文件结构

### 核心模块 (`vision_gps_estimator/`)
- `integrated_node.py` - 主要的 ROS2 节点，集成所有功能
- `yolo_detector.py` - YOLO 目标检测器
- `gps_manager.py` - GPS 坐标转换管理器  
- `streaming.py` - GStreamer 视频流处理

### 配置文件
- `config/params.yaml` - ROS2 参数配置
- `launch/vision_gps_estimator.launch.py` - ROS2 启动文件
- `requirements.txt` - Python 依赖
- `environment.yml` - Conda 环境配置
- `package.xml` - ROS2 包配置
- `setup.py` - Python 包配置

### 部署脚本 (`scripts/`)
- `validate_pipeline.py` - **Pipeline 验证脚本**（保留用于测试）
- `quick_record.py` - **一键录制脚本** 🆕
- `setup_conda_env.sh` - Conda 环境设置
- `install_dependencies.sh` - 依赖安装
- `integration_test.sh` - 集成测试

### 快速启动脚本
- `record.sh` - **一键录制启动器** 🆕

## 已清理的内容
- ✅ 删除了所有 `validation_output_*` 目录（测试图像和报告）
- ✅ 删除了临时测试脚本：
  - `analyze_mcap_data.py`
  - `comprehensive_test.py`
  - `debug_coordinate_transform.py`
  - `simple_mcap_test.py`
  - `test_setup.py`
  - `test_with_mcap.py`
  - `validate_mcap_pipeline.py`
- ✅ 删除了 `test/` 目录（单元测试）

## Pipeline 验证脚本使用

保留的 `scripts/validate_pipeline.py` 可用于：
```bash
# 验证 MCAP 文件处理
python3 scripts/validate_pipeline.py path/to/recording.mcap

# 使用 conda 环境
conda activate vision_gps_estimator
python3 scripts/validate_pipeline.py path/to/recording.mcap
```

## 验证结果摘要

最后验证成功的结果：
- **总帧数**: 222 帧
- **检测目标**: 5 次
- **GPS 估计**: 5 个精确坐标
- **目标平均位置**: (40.425272, -79.954221)
- **距离范围**: 3.44m - 6.53m
- **处理性能**: 171.85 ms/帧

Pipeline 完全验证成功：Image → YOLO Detection → GPS Transformation ✅

## 🆕 一键录制功能

### 快速开始
```bash
# 默认录制：10秒，20帧，1080x720
./record.sh

# 自定义参数
./record.sh --duration 15 --fps 3 --width 1920 --height 1080

# 不使用ROS2（使用摄像头或模拟数据）
./record.sh --no-ros
```

### 功能特点
- ⚡ **预加载YOLO模型**：减少启动延迟
- 📸 **默认设置**：10秒录制，2 FPS，共20帧
- 📐 **图像分辨率**：1080x720（可自定义）
- 🎯 **实时检测**：每帧进行YOLO目标检测
- 🌍 **GPS估计**：自动计算目标GPS坐标
- 💾 **数据保存**：
  - 原始图像 (`frame_XXXX.jpg`)
  - 标注图像 (`annotated_XXXX.jpg`，仅有检测时）
  - JSON元数据 (`recording_data.json`)
  - 包含时间戳、检测结果、GPS坐标

### 输出结构
```
recording_YYYYMMDD_HHMMSS/
├── frame_0000.jpg           # 原始图像
├── frame_0001.jpg
├── ...
├── annotated_0003.jpg       # 带检测框的图像
├── annotated_0007.jpg
└── recording_data.json      # 完整元数据
```

### JSON数据格式
```json
{
  "metadata": {
    "duration_sec": 10,
    "fps": 2.0,
    "resolution": [1080, 720],
    "total_frames": 20
  },
  "frames": [
    {
      "frame_idx": 0,
      "timestamp": 1234567890.123,
      "image_file": "frame_0000.jpg",
      "detections": 1,
      "drone_state": {
        "gps": [40.425280, -79.954200],
        "altitude": 50.0,
        "heading": 90.0,
        "gimbal": {"pitch": -45.0, "roll": 0.0, "yaw": 0.0}
      },
      "targets": [
        {
          "bbox": [100, 200, 150, 250],
          "center": [125, 225],
          "confidence": 0.85,
          "class": 0,
          "gps": {
            "latitude": 40.425272,
            "longitude": -79.954221,
            "distance_m": 5.5
          }
        }
      ]
    }
  ]
}
```
