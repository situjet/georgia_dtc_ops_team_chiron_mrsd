# RTSP to ROS2 Streamer

ROS2 package for converting RTSP streams to compressed image topics.

## Features

- Dual stream support (EO and IR cameras)
- Compressed image output for bandwidth efficiency
- Configurable via YAML file (mounted at runtime)
- Automatic reconnection on stream failure
- Docker deployment with host networking
- FastDDS configuration with UDP transport and disabled shared memory

## Quick Start

### Build and Run with Docker Compose

```bash
# Build the Docker image
docker-compose build

# Run the container
docker-compose up -d

# View logs
docker-compose logs -f

# Stop the container
docker-compose down
```

### Configuration

Edit `config.yaml` to customize:
- RTSP stream URLs
- ROS2 topic names  
- Enable/disable streams
- FPS and JPEG quality
- Reconnection settings

The config file is mounted at runtime, so changes take effect after container restart.

### Environment Variables

- `CONFIG_PATH`: Override default config file path
- `ROS_DOMAIN_ID`: ROS2 domain ID (default: 0)

### Topics

Default topics:
- `/vehicle_9/eo/image/compressed` - EO camera compressed images
- `/vehicle_9/ir/image/compressed` - IR camera compressed images

### Building from Source

```bash
cd rtsp_ros2_ws
colcon build
source install/setup.bash
ros2 launch rtsp_streamer rtsp_streamer.launch.py
```