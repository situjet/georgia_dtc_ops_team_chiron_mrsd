# Dell Publisher

Dell Publisher is a ROS2 package for forwarding ROS topic messages to remote hosts via UDP protocol. This package completely migrates all functionality from `rostopic_publisher.py`.

## Features

- **ROS2 Topic Subscription**: Subscribes to specified ROS topics
- **Dynamic Message Type Support**: Dynamically imports and processes different message types based on parameters
- **UDP Chunked Transmission**: Splits large messages into 512-byte chunks for UDP transmission
- **Performance Monitoring**: Real-time display of message processing speed and statistics
- **Parameterized Configuration**: Supports configuration of all key settings through ROS parameters

## Installation and Build

1. Ensure you are in the ROS2 workspace:
```bash
cd /home/lance/Documents/github/migration
```

2. Build the package:
```bash
colcon build --packages-select Dell_publisher
```

3. Source the environment:
```bash
source install/setup.bash
```

## Usage

### 1. Run Node Directly

Using default parameters:
```bash
ros2 run Dell_publisher rostopic_publisher
```

Using custom parameters:
```bash
ros2 run Dell_publisher rostopic_publisher --ros-args \
  -p topic_name:=/your/topic \
  -p topic_type:=std_msgs/msg/String \
  -p udp_ip:=10.3.1.100 \
  -p udp_port:=6000
```

### 2. Using Launch Files

Using default parameters:
```bash
ros2 launch Dell_publisher rostopic_publisher.launch.py
```

Using custom parameters:
```bash
ros2 launch Dell_publisher rostopic_publisher.launch.py \
  topic_name:=/drone/data \
  topic_type:=dtc_network_msgs/msg/HumanDataMsg \
  udp_ip:=10.3.1.106 \
  udp_port:=5005
```

## Parameter Configuration

| Parameter Name | Type | Default Value | Description |
|----------------|------|---------------|-------------|
| `topic_name` | string | "drone/data" | ROS topic name to subscribe to |
| `topic_type` | string | "dtc_network_msgs/msg/HumanDataMsg" | ROS message type |
| `udp_ip` | string | "10.3.1.106" | UDP target IP address |
| `udp_port` | integer | 5005 | UDP target port |

## UDP Protocol Format

Messages are sent via UDP using the following format:
1. Send `__start__` marker to begin transmission
2. Split message into 512-byte chunks for transmission
3. 10ms delay between each chunk to prevent data loss
4. Send `__end__` marker to end transmission

## Dependencies

- rclpy
- std_msgs
- sensor_msgs
- geometry_msgs
- builtin_interfaces
- cv_bridge
- python3-opencv
- dtc_network_msgs
- straps_msgs

## Testing

Run tests:
```bash
colcon test --packages-select Dell_publisher
```

## License

Apache-2.0