#!/bin/bash

# 设置错误时退出
set -e

# echo "===== 1. 切换到Docker目录 ====="
# cd /home/dtc/humanflow/ros2_ghadron_gimbal/dockers
# cd /home/dtc/humanflow/ros2_ghadron_gimbal

# echo "===== 2. 重启Docker容器 ====="
# # 启动容器
# docker compose -f docker_compose_gimbal.yaml up -d

# echo "===== 3. 等待容器完全启动 ====="
# sleep 3

echo "===== 4. 在容器内构建ROS2工作空间 ====="
# 在容器中执行构建命令
docker exec -it ros2_gimbal_container bash

colcon build --cmake-args -DGHADRON=1

source install/setup.bash
   
ros2 launch gimbal_bringup gimbal_system.launch.py

ros2 run gimbal_angle_control gimbal_angle_control_node

ros2 topic pub /gimbal_angles geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: 0.0}"

ros2 topic pub /gimbal_angles geometry_msgs/msg/Vector3 "{x: -70.0, y: 0.0, z: 0.0}"