#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import time

class TopicTester(Node):
    def __init__(self):
        super().__init__('topic_tester')
        self.count = 0
        self.subscription = self.create_subscription(
            CompressedImage,
            '/image_raw_compressed',
            self.listener_callback,
            10)
        self.timer = self.create_timer(1.0, self.print_status)
        self.start_time = time.time()

    def listener_callback(self, msg):
        self.count += 1
        self.get_logger().info(f'Received image {self.count}, size: {len(msg.data)} bytes, format: {msg.format}')

    def print_status(self):
        elapsed = time.time() - self.start_time
        rate = self.count / elapsed if elapsed > 0 else 0
        self.get_logger().info(f'Total: {self.count} messages, Rate: {rate:.2f} Hz')

def main():
    rclpy.init()
    tester = TopicTester()
    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()