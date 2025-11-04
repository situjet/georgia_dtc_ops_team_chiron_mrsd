#!/bin/bash
# Publish geofence data once

ros2 topic pub --once /robot_1/mavros/geofence/fences mavros_msgs/msg/WaypointList "{current_seq: 0, waypoints: [
  {frame: 0, command: 5001, is_current: false, autocontinue: true, param1: 4.0, param2: 0.0, param3: 0.0, param4: 0.0, x_lat: 40.4141646, y_long: -79.9475044, z_alt: 0.0},
  {frame: 0, command: 5001, is_current: false, autocontinue: true, param1: 4.0, param2: 0.0, param3: 0.0, param4: 0.0, x_lat: 40.4138379, y_long: -79.9475000, z_alt: 0.0},
  {frame: 0, command: 5001, is_current: false, autocontinue: true, param1: 4.0, param2: 0.0, param3: 0.0, param4: 0.0, x_lat: 40.4137503, y_long: -79.9479319, z_alt: 0.0},
  {frame: 0, command: 5001, is_current: false, autocontinue: true, param1: 4.0, param2: 0.0, param3: 0.0, param4: 0.0, x_lat: 40.4140025, y_long: -79.9480247, z_alt: 0.0}
]}"

