# Gimbal Teleop Integration

This document describes the gimbal teleop integration between the operator system and the airstack.

## Overview

The system connects the operator's joystick/teleop commands to the airstack's gimbal control via PayloadSDK:

1. **Operator Side**: ROS2 node `gimbal_teleop_node` publishes joystick commands to `/gimbal/teleop` topic
2. **Airstack Side**: ROS2 node `gimbal_angle_control_node` subscribes to `/gimbal/teleop` and controls the gimbal via PayloadSDK

## Components

### Operator System (`operator/ros_ws/`)

- **Package**: `gimbal_teleop`
- **Node**: `gimbal_teleop_node`
- **Topic Published**: `/gimbal/teleop` (geometry_msgs/Vector3)
- **Input**: Joystick `/joy` topic
- **Domain**: ROS_DOMAIN_ID=70

The teleop node maps joystick inputs to gimbal commands:
- `msg.x` = pitch (right stick Y)  
- `msg.y` = yaw (right stick X)
- `msg.z` = zoom (trigger difference)

### Airstack System (`airstack/ros_ws/`)

- **Package**: `gimbal_control`  
- **Node**: `gimbal_angle_control_node`
- **Topics Subscribed**: 
  - `/gimbal/teleop` (geometry_msgs/Vector3) - for rate commands from operator
  - `gimbal_angles` (geometry_msgs/Vector3) - for absolute angle commands
- **Domain**: ROS_DOMAIN_ID=70
- **Hardware Interface**: PayloadSDK via UDP to gimbal hardware

The gimbal control node:
- Receives teleop commands and treats them as rate commands (INPUT_ANGULAR_RATE mode)
- Receives angle commands and treats them as absolute positioning (INPUT_ANGLE mode)
- Publishes current gimbal angles to `current_gimbal_angles`

## Communication

Both systems operate on **ROS_DOMAIN_ID=70**, allowing direct topic communication without domain bridging.

## Topic Flow

```
Operator Joystick → /joy → gimbal_teleop_node → /gimbal/teleop → gimbal_angle_control_node → PayloadSDK → Gimbal Hardware
```

## Usage

### Running the Operator System

1. Set environment for domain 70:
   ```bash
   export ROS_DOMAIN_ID=70
   ```

2. Build and run the operator workspace:
   ```bash
   cd operator/ros_ws
   colcon build
   source install/setup.bash
   ros2 run gimbal_teleop gimbal_teleop_node
   ```

3. Ensure joystick is connected and publishing to `/joy`

### Running the Airstack System

The gimbal control is automatically started as part of the unified airstack container:

```bash
cd airstack/docker
./run_unified.sh  # Uses ROS_DOMAIN_ID=70 by default
```

### Testing

1. Check topics are available:
   ```bash
   ros2 topic list | grep gimbal
   ```

2. Monitor gimbal commands:
   ```bash
   ros2 topic echo /gimbal/teleop
   ```

3. Check gimbal status:
   ```bash
   ros2 topic echo /current_gimbal_angles
   ```

## Troubleshooting

- Ensure both systems are on ROS_DOMAIN_ID=70
- Check network connectivity between operator and airstack systems  
- Verify PayloadSDK hardware connection (UDP to 192.168.12.240:14566)
- Check joystick is publishing to `/joy` topic