# Behavior Governor

A modular behavior governor for vehicle autonomy, replacing the rigid C++ behavior tree with a flexible, extensible system.

## Overview
- **C++ Node**: Handles core vehicle actions (waypoint, takeoff, arming, etc.) and exposes topics/services for extension.
- **Python Integration**: Easily add Python scripts in `scripts/` to publish/subscribe to behavior topics for complex behaviors.

## Topics
- `behavior_governor/command` (`std_msgs/String`): Send high-level commands (e.g., `takeoff`, `arm`, `goto_waypoint`).
- `behavior_governor/status` (`std_msgs/String`): Status updates from the governor.

## Extending in Python
Add scripts to the `scripts/` directory. Example:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ExampleBehavior(Node):
    def __init__(self):
        super().__init__('example_behavior')
        self.cmd_pub = self.create_publisher(String, 'behavior_governor/command', 10)
        self.status_sub = self.create_subscription(String, 'behavior_governor/status', self.status_callback, 10)
        self.timer = self.create_timer(2.0, self.send_takeoff)
        self.sent = False

    def send_takeoff(self):
        if not self.sent:
            msg = String()
            msg.data = 'takeoff'
            self.cmd_pub.publish(msg)
            self.sent = True

    def status_callback(self, msg):
        print(f'Status: {msg.data}')

if __name__ == '__main__':
    rclpy.init()
    node = ExampleBehavior()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
```

## Build & Run
1. Add `behavior_governor` to your workspace and rebuild:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ros2 run behavior_governor behavior_governor_node
   ```
2. Run your Python script:
   ```bash
   ros2 run behavior_governor example_behavior.py
   ```

## Notes
- Core actions (takeoff, arming, etc.) are implemented in C++ for safety and performance.
- Complex/mission behaviors can be rapidly prototyped in Python.
