#!/bin/bash

# Stop any existing container
docker stop rtsp_ros2_streamer 2>/dev/null
docker rm rtsp_ros2_streamer 2>/dev/null

echo "Starting RTSP ROS2 Streamer..."

docker run -d \
  --name rtsp_ros2_streamer \
  --network host \
  --ipc host \
  -e FASTRTPS_DEFAULT_PROFILES_FILE=/ros2_ws/fastdds_profile.xml \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  -e FASTDDS_BUILTIN_TRANSPORTS=UDPv4 \
  -e FASTRTPS_DISABLE_SHM=1 \
  -e ROS_DOMAIN_ID=0 \
  -v $(pwd)/config.yaml:/ros2_ws/install/rtsp_streamer/share/rtsp_streamer/config/config.yaml:ro \
  --restart unless-stopped \
  rtsp_ros2_streamer

echo "Container started. View logs with: docker logs -f rtsp_ros2_streamer"