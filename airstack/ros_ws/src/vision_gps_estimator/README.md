# Vision GPS Estimator

Real-time GPS target estimation from RTSP video streams integrated with MAVROS for AirStack autonomous systems.

## Overview

This package provides vision-based GPS coordinate estimation by combining:
- RTSP H.265 video streaming with hardware-accelerated decoding
- YOLO object detection (TensorRT optimized)
- Pixel-to-GPS coordinate transformation using drone pose data
- Target clustering and tracking

## Features

- **Low-latency streaming**: Hardware-accelerated H.265 decoding with GStreamer
- **Real-time detection**: TensorRT-optimized YOLO inference
- **Accurate GPS estimation**: Calibrated coordinate transformations with gimbal compensation
- **Time synchronization**: Frame-accurate sensor data alignment
- **MCAP recording**: Full pipeline recording for analysis and replay
- **ROS2 native**: Standard message types and QoS profiles

## Architecture

```
RTSP Stream → GStreamer → YOLO Detection → GPS Estimation → ROS2 Topics
     ↑                                           ↑
MAVROS Data ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ← ←
(GPS, Altitude, Heading, Gimbal Attitude)
```

## Installation

### Prerequisites

```bash
# ROS2 dependencies
sudo apt install ros-humble-cv-bridge ros-humble-vision-msgs ros-humble-mavros-msgs

# GStreamer dependencies  
sudo apt install gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
                 gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav

# Python dependencies
pip install ultralytics opencv-python numpy scipy
```

### Build

```bash
cd /path/to/airstack/ros_ws
colcon build --packages-select vision_gps_estimator
source install/setup.bash
```

## Configuration

### Camera Calibration

Update camera intrinsics in `config/params.yaml`:

```yaml
camera_intrinsics:
  EO:
    fx: 475.0    # Focal length X
    fy: 505.0    # Focal length Y  
    cx: 320.0    # Principal point X
    cy: 256.0    # Principal point Y
    distortion: [0.0, 0.0, 0.0, 0.0, 0.0]  # k1, k2, p1, p2, k3
```

### YOLO Model

Place your TensorRT engine file:
```bash
mkdir -p models/
cp your_model.engine models/yolo11s-pose.engine
```

## Usage

### Basic Launch

```bash
ros2 launch vision_gps_estimator vision_gps_estimator.launch.py
```

### With Custom Configuration

```bash
ros2 launch vision_gps_estimator vision_gps_estimator.launch.py \
  config_file:=/path/to/custom_params.yaml \
  robot_namespace:=robot_1 \
  log_level:=DEBUG
```

### With MCAP Recording

```bash
ros2 launch vision_gps_estimator vision_gps_estimator.launch.py \
  record_mcap:=true \
  mcap_output_dir:=/data/logs
```

## Topics

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/robot_1/mavros/global_position/global` | `NavSatFix` | Drone GPS position |
| `/robot_1/mavros/global_position/rel_alt` | `Float64` | Relative altitude (AGL) |
| `/robot_1/mavros/global_position/compass_hdg` | `Float64` | Compass heading (degrees) |
| `/gimbal_attitude` | `Vector3` | Gimbal attitude (roll, pitch, yaw in radians) |
| `/camera_mode` | `String` | Camera mode ("EO" or "IR") |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/vision_gps/image_compressed` | `CompressedImage` | Compressed video stream |
| `/vision_gps/camera_info` | `CameraInfo` | Camera calibration info |
| `/vision_gps/detections` | `Detection2DArray` | YOLO detection results |
| `/vision_gps/target_gps` | `NavSatFix` | Primary target GPS estimate |
| `/vision_gps/target_gps_list` | `NavSatFix` | All clustered targets |

## Parameters

Key parameters in `config/params.yaml`:

```yaml
streaming:
  rtsp_url: "rtsp://10.3.1.124:8554/ghadron"
  width: 640
  height: 512
  fps: 10

detector:
  model_path: "models/yolo11s-pose.engine"
  conf_threshold: 0.5
  iou_threshold: 0.45
  max_detections: 10

sync:
  max_sync_time_diff: 0.1  # seconds

gps_estimation:
  min_distance_threshold: 5.0  # meters between targets
```

## Coordinate Systems

### Frames
- **Camera Optical**: X=right, Y=down, Z=forward
- **Drone Body**: X=forward, Y=right, Z=down (NED)
- **World ENU**: X=east, Y=north, Z=up

### Conventions
- **Heading**: 0°=North, positive=clockwise
- **Gimbal**: Roll/Pitch/Yaw in radians, right-hand rule

## Testing

### Unit Tests

```bash
cd /path/to/airstack/ros_ws
colcon test --packages-select vision_gps_estimator
```

### Integration Test

```bash
# Terminal 1: Launch the system
ros2 launch vision_gps_estimator vision_gps_estimator.launch.py

# Terminal 2: Monitor topics
ros2 topic echo /vision_gps/target_gps

# Terminal 3: Check diagnostics
ros2 topic echo /diagnostics
```

## Performance

### Typical Performance
- **Streaming**: 10 FPS @ 640x512
- **Detection**: 7 FPS inference
- **GPS Estimation**: <10ms per target
- **Memory Usage**: <2GB
- **CPU Usage**: <80% (with TensorRT)

### Optimization Tips
1. Use TensorRT engines instead of PyTorch models
2. Enable hardware video decoding (NVDEC/VA-API)
3. Tune `max_sync_time_diff` for your scenario
4. Adjust JPEG quality for bandwidth vs. quality trade-off

## Troubleshooting

### Common Issues

**No video stream**:
```bash
# Test RTSP connection
gst-launch-1.0 rtspsrc location=rtsp://your.stream.url ! autovideosink
```

**GPS estimation errors**:
- Check gimbal attitude values are in radians
- Verify camera intrinsics match current resolution
- Ensure drone height > 0 and gimbal pointing downward

**Time sync failures**:
- Increase `max_sync_time_diff` parameter
- Check all MAVROS topics are publishing
- Verify system time synchronization

### Debug Mode

```bash
ros2 launch vision_gps_estimator vision_gps_estimator.launch.py log_level:=DEBUG
```

## Development

### Adding New Camera Types

1. Add intrinsics to `config/params.yaml`
2. Update `gps_manager.py` camera type handling
3. Test calibration accuracy

### Custom Detection Models

1. Export your model to TensorRT:
```python
from ultralytics import YOLO
model = YOLO('your_model.pt')
model.export(format='engine', device=0)
```

2. Update `model_path` parameter
3. Adjust confidence thresholds as needed

## Contributing

1. Follow ROS2 coding standards
2. Add unit tests for new functionality  
3. Update documentation
4. Test with hardware integration

## License

MIT License - see LICENSE file for details.
