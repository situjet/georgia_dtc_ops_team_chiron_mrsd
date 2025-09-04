# ROS2 Gimbal Control Workspace

Only these commands are enough to launch the gimbal control. The rest are for the unit tests. 

set up the docker 
```bash
ssh dtc@10.3.1.32
sudo chown $(whoami):$(whoami) /home/dtc/humanflow/ros2_ghadron_gimbal/
cd /home/dtc/humanflow/ros2_ghadron_gimbal/dockers
&& docker compose -f docker-compose.yaml down
&& docker compose -f docker-compose.yaml up -d
docker exec -it ros2_gimbal_container bash
# rm the docker when needed
docker rm -f ros2_gimbal_container
```
run lauch file 
```bash
cd ros2_ghadron_gimbal 
colcon build --cmake-args -DGHADRON=1
source install/setup.bash
ros2 launch gimbal_bringup gimbal_system.launch.py
```


(By default it is already built) build the payload sdk from scratch
```bash
cd PayloadSdk
mkdir build
cd build
cmake -DGHADRON=1 ../
make -j6
```
## Unit Test and Function moduels

```bash
cd ros2_gremsy_gimbal_control
colcon build --cmake-args -DGHADRON=1
source install/setup.bash
```
```bash

ros2 run gimbal_angle_control gimbal_angle_control_node
ros2 run gimbal_status gimbal_status_node
ros2 run image_publisher image_publisher_node rtsp://10.3.1.124:8554/ghadron
ros2 run stream_publisher stream_node --ros-args -p rtsp_url:="rtsp://10.3.1.124:8554/ghadron" -p width:=1280 -p height:=720
ros2 run yolo_detection yolo_detection_node
ros2 run human_tracking tracking_node
ros2 run image_viewer image_viewer_node
```


### Gimbal bringup
launch all the packages in the gimbal system
```bash
ros2 launch gimbal_bringup gimbal_system.launch.py
```
The rest of the nodes are automatically launched by gimbal_system.launch.py

### Gimbal Angle Control
```bash
ros2 run gimbal_angle_control gimbal_angle_control_node
```
Send control commands (pitch=0, row=0, yaw=90).
The range of the pitch is -90 pointing down to 90 pointing up.
The range of the yaw is -120 to the left to 120 to the right. But the gimbal may lock if you switch the direction too fast like -120 to 120 directly. It is better to do it incrementally like by 10 degrees each time.
```bash
ros2 topic pub /gimbal_angles geometry_msgs/msg/Vector3 "{x: 0.0, y: 0.0, z: 90.0}"
```

### Get gimbal status
```bash
ros2 run gimbal_status gimbal_status_node
```

View topic information
```bash
ros2 topic echo /gimbal_attitude
```
### eo image
```bash
ros2 run eo_image eo_image_node
```
### ir image
```bash
ros2 run ir_image ir_image_node
```

### Zoom control for ir camera
```bash
ros2 run ir_zoom ir_zoom_node
```
set zoom range 50% for ir camera, 0 means no zoom in, 100 means 8x zoom in.
```bash
ros2 topic pub /ir_zoom/range std_msgs/msg/Float32 "data: 50.0" # 0.0% to 100.0%
```

### Zoom control for eo camera
```bash
ros2 run eo_zoom eo_zoom_node
```
set zoom range 50% for eo camera, 0 means no zoom in, 100 means 12x zoom in.
```bash
ros2 topic pub /eo_zoom/range std_msgs/msg/Float32 "data: 50.0" # 0.0% to 100.0%
```

### YOLO detection
```bash
ros2 run yolo_detection yolo_detection_node
```
check the published topics
```bash
ros2 topic info /detection_box
ros2 topic info /detection_box_center
```
### Human tracking
```bash
ros2 run human_tracking tracking_node
```
check the published topics
```bash
ros2 topic info /gimbal_angles
```
## Other functions
### Check the streaming video
image_publisher is a node that publishes the streaming video to the topic /image_raw, image_viewer is a node that subscribes to the topic /image_raw and displays the video. We don't use them for now because the streaming video contains ir and eo overlapped and not being able to display separately.
```bash
ros2 run image_publisher image_publisher_node rtsp://10.3.1.124:8554/ghadron
ffmpeg -y -i rtsp://10.3.1.124:8554/ghadron 1 do.jpg
ros2 topic echo /image_raw
```
## Warnings


### hard-coded path prefix
needed to be changed to if code is migrated to other machine
```bash
/home/dtc-mrsd/Downloads/ros2_gimbal_ws/
```
### docker folder auth
sudo chown -R $(id -u):$(id -g) /home/dtc/humanflow/ros2_ghadron_gimbal/src/eo_zoom
sudo chmod -R 775 /home/dtc/humanflow/ros2_ghadron_gimbal/src/eo_zoom

### command-line commands
ros2 bag record \
  -o /home/dtc/humanflow/ros2_ghadron_gimbal/mcap_recording \
  --storage mcap \
  --max-bag-duration 60 \
  /image_raw \
  /detection_box \
  /gimbal_attitude \
  /gimbal_angles \
  /waypoint_waiting


ros2 bag play mcap_data/2025-02-28/recording_20250228_053258/recording_20250228_053258_2.mcap

/image_raw
/gimbal_attitude
/detection_box

# 修改整个工作空间的权限
sudo chown -R $USER:$USER /home/dtc/humanflow/ros2_ghadron_gimbal

# 确保安装目录有正确的权限
sudo chmod -R 755 /home/dtc/humanflow/ros2_ghadron_gimbal/install

# 特别是对于 site-packages 目录
sudo chmod -R 755 /home/dtc/humanflow/ros2_ghadron_gimbal/install/detect_and_track/lib/python3.10/site-packages