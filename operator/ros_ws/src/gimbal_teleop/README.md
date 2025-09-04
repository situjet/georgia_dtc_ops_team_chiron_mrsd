# gimbal_teleop

Teleoperation node for gimbal control using Xbox controller. This package subscribes to `/joy` and publishes gimbal commands to `/gimbal_angles`.

## Usage
1. Install the `joy` package:
   ```bash
   sudo apt-get install ros-<distro>-joy
   ```
2. Launch the joystick driver:
   ```bash
   rosrun joy joy_node
   ```
3. Build and run this package:
   ```bash
   cd ~/catkin_ws && catkin_make
   rosrun gimbal_teleop gimbal_teleop_node
   ```

## Axis Mapping
- Right stick Y (axes[4]): pitch
- Right stick X (axes[3]): yaw
- Triggers (axes[2], axes[5]): zoom (difference)
