#!/bin/bash
# Publish human GPS data continuously

if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
fi

# Publish /target_gps (single point per detection)
ros2 topic pub --rate 0.5 /target_gps sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, latitude: 40.4140, longitude: -79.9477, altitude: 0.0}" > /dev/null 2>&1 &
GPS1_PID=$!

# Publish /target_gps_list (multiple clusters, simulates real behavior)
# Each cluster published separately with frame_id cluster_X
(
  while true; do
    ros2 topic pub --once /target_gps_list sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'cluster_0'}, latitude: 40.4139, longitude: -79.9476, altitude: 0.0}" > /dev/null 2>&1
    sleep 0.1
    ros2 topic pub --once /target_gps_list sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'cluster_1'}, latitude: 40.4141, longitude: -79.9475, altitude: 0.0}" > /dev/null 2>&1
    sleep 0.1
    ros2 topic pub --once /target_gps_list sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'cluster_2'}, latitude: 40.4138, longitude: -79.9477, altitude: 0.0}" > /dev/null 2>&1
    sleep 1.0
  done
) > /dev/null 2>&1 &
GPS2_PID=$!

# Publish /precise_target_gps (single point when distance < 1m)
ros2 topic pub --rate 0.5 /precise_target_gps sensor_msgs/msg/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, latitude: 40.4141, longitude: -79.9478, altitude: 0.0}" > /dev/null 2>&1 &
GPS3_PID=$!

trap "kill $GPS1_PID $GPS2_PID $GPS3_PID 2>/dev/null; exit" INT TERM

while true; do
  sleep 1
done

