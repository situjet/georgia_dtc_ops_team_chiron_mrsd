import cv2
import time
import os
import threading
import subprocess
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Bool

class BurstModeRecorder(Node):
    def __init__(self, rtsp_url, mcap_dir, duration_sec=10, fps=24, topics_to_record=None):
        super().__init__('burst_recorder_node')
        self.declare_parameter('fps', fps)
        self.declare_parameter('duration_sec', duration_sec)
        default_topics = [
            '/camera/image_raw',
            '/robot_1/mavros/global_position/global',
            '/robot_1/mavros/global_position/rel_alt', 
            '/robot_1/mavros/imu/data',
            '/robot_1/mavros/global_position/compass_hdg',
            '/gimbal_attitude'
        ]
        self.declare_parameter('topics_to_record', default_topics)
        self.rtsp_url = rtsp_url
        self.mcap_dir = mcap_dir
        self.duration_sec = self.get_parameter('duration_sec').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.topics_to_record = self.get_parameter('topics_to_record').get_parameter_value().string_array_value or default_topics
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        os.makedirs(self.mcap_dir, exist_ok=True)
        self.burst_mode = False
        self.bag_process = None
        self.burst_thread = None
        self.control_sub = self.create_subscription(
            Bool,
            '/burst_mode/control',
            self.control_callback,
            10
        )

    def control_callback(self, msg):
        if msg.data:
            self.get_logger().info("Received burst mode START command.")
            self.start_burst_mode()
        else:
            self.get_logger().info("Received burst mode STOP command.")
            self.stop_burst_mode()

    def start_burst_mode(self):
        if self.burst_mode:
            self.get_logger().info('Burst mode already running.')
            return
        self.burst_mode = True
        self.burst_thread = threading.Thread(target=self._burst_loop)
        self.burst_thread.start()

    def stop_burst_mode(self):
        self.burst_mode = False
        if self.burst_thread:
            self.burst_thread.join()

    def _burst_loop(self):
        cap = cv2.VideoCapture(self.rtsp_url)
        if not cap.isOpened():
            self.get_logger().error(f'Failed to open RTSP stream: {self.rtsp_url}')
            return
        try:
            while self.burst_mode:
                timestamp = time.strftime('%Y%m%d_%H%M%S')
                mcap_path = os.path.join(self.mcap_dir, f'burst_{timestamp}')
                bag_cmd = [
                    'ros2', 'bag', 'record', '-o', mcap_path, '-s', 'mcap'
                ] + self.topics_to_record
                self.bag_process = subprocess.Popen(bag_cmd)
                self.get_logger().info(f'Started ros2 bag record: {mcap_path} topics: {self.topics_to_record}')
                start_time = time.time()
                while time.time() - start_time < self.duration_sec and self.burst_mode:
                    ret, frame = cap.read()
                    if not ret:
                        self.get_logger().warning('Frame grab failed.')
                        continue
                    img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                    img_msg.header.stamp = self.get_clock().now().to_msg()
                    img_msg.header.frame_id = 'camera'
                    self.publisher.publish(img_msg)
                    time.sleep(1.0 / self.fps)
                if self.bag_process:
                    self.bag_process.terminate()
                    self.bag_process.wait()
                    self.get_logger().info('Stopped ros2 bag record.')
        finally:
            cap.release()
            self.bag_process = None

def main():
    rclpy.init()
    RTSP_URL = "rtsp://10.3.1.124:8554/ghadron"  # Replace with your RTSP stream
    MCAP_DIR = "./mcap"
    recorder = BurstModeRecorder(RTSP_URL, MCAP_DIR)
    try:
        recorder.get_logger().info("BurstRecorder node started. Use /burst_mode/control topic to control burst mode.")
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.stop_burst_mode()
    finally:
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
