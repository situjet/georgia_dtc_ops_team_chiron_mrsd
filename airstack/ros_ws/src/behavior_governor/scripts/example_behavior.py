#!/usr/bin/env python3
"""
Example Python extension for the behavior_governor node.
This script publishes high-level behavior commands to the governor and listens for status updates.
"""
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
            self.get_logger().info('Sent takeoff command')
            self.sent = True

    def status_callback(self, msg):
        self.get_logger().info(f'Status: {msg.data}')

if __name__ == '__main__':
    rclpy.init()
    node = ExampleBehavior()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
