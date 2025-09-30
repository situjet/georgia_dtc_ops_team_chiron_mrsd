#!/usr/bin/env python3
import time
import os
import threading
import subprocess
from rclpy.node import Node
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CompressedImage, NavSatFix
from std_msgs.msg import Bool, Header, Float64
from dtc_network_msgs.msg import HumanDataMsg
from builtin_interfaces.msg import Time
import numpy as np

# Check OpenCV availability
CV_AVAILABLE = False
try:
    import cv2
    from cv_bridge import CvBridge
    CV_AVAILABLE = True
except ImportError as e:
    print(f"OpenCV not available: {e}")
    CV_AVAILABLE = False

class HumanDataPublisherRecorder(Node):
    def __init__(self):
        super().__init__('human_data_publisher_recorder')
        
        # Check if OpenCV is available
        if not CV_AVAILABLE:
            self.get_logger().fatal("OpenCV is not available! Node cannot function without OpenCV. Shutting down.")
            raise RuntimeError("OpenCV not available")
        
        # Declare all parameters with defaults
        self.declare_parameter('compressed_image_topic', '/image_raw_compressed')
        self.declare_parameter('mcap_dir', '/root/ros_ws/mcap_recordings')
        self.declare_parameter('burst_fps', 24)  # FPS during burst recording
        self.declare_parameter('publish_fps', 5)  # Normal publishing FPS
        self.declare_parameter('burst_duration_sec', 10)
        self.declare_parameter('system_name', 'drone_system')
        self.declare_parameter('enable_continuous_publish', True)  # Enable continuous publishing
        # HumanDataMsg will be recorded instead of individual topics
        default_topics = ['/human_data']
        self.declare_parameter('topics_to_record', default_topics)
        
        # Get parameters
        self.compressed_image_topic = self.get_parameter('compressed_image_topic').get_parameter_value().string_value
        self.mcap_dir = self.get_parameter('mcap_dir').get_parameter_value().string_value
        self.burst_duration_sec = self.get_parameter('burst_duration_sec').get_parameter_value().integer_value
        self.burst_fps = self.get_parameter('burst_fps').get_parameter_value().integer_value
        self.publish_fps = self.get_parameter('publish_fps').get_parameter_value().integer_value
        self.system_name = self.get_parameter('system_name').get_parameter_value().string_value
        self.enable_continuous_publish = self.get_parameter('enable_continuous_publish').get_parameter_value().bool_value
        self.topics_to_record = self.get_parameter('topics_to_record').get_parameter_value().string_array_value or default_topics
        
        # Initialize OpenCV components
        self.bridge = CvBridge()
        
        # Publisher for HumanDataMsg
        self.human_data_publisher = self.create_publisher(HumanDataMsg, '/human_data', 10)
        
        # State variables for GPS data
        self.latest_gps = None
        self.gps_lock = threading.Lock()
        
        # Create output directory
        try:
            os.makedirs(self.mcap_dir, exist_ok=True)
            self.get_logger().info(f"Created/verified MCAP directory: {self.mcap_dir}")
        except Exception as e:
            self.get_logger().fatal(f"Failed to create MCAP directory {self.mcap_dir}: {e}")
            raise RuntimeError(f"Cannot create MCAP directory: {e}")
        
        # Burst mode state
        self.burst_mode = False
        self.bag_process = None
        self.burst_thread = None
        
        # Image processing state
        self.latest_frame = None
        self.latest_compressed_msg = None
        self.frame_lock = threading.Lock()
        self.last_frame_time = 0
        
        # Create QoS profile for MAVROS compatibility
        mavros_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribe to compressed image from rtsp_streamer
        self.image_sub = self.create_subscription(
            CompressedImage,
            self.compressed_image_topic,
            self.compressed_image_callback,
            10
        )
        
        # Subscribe to GPS data from MAVROS
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/dtc_mrsd_/mavros/global_position/global',
            self.gps_callback,
            mavros_qos
        )
        
        # Control subscription for burst mode
        self.control_sub = self.create_subscription(
            Bool,
            '/burst_mode/control',
            self.control_callback,
            10
        )
        
        self.get_logger().info(f"Subscribed to compressed image topic: {self.compressed_image_topic}")
        self.get_logger().info(f"Publishing HumanDataMsg to /human_data topic")
        self.get_logger().info(f"System name: {self.system_name}")
        self.get_logger().info(f"Continuous publishing: {'Enabled' if self.enable_continuous_publish else 'Disabled'}")
        
        # Timer for continuous publishing
        if self.enable_continuous_publish:
            publish_period = 1.0 / self.publish_fps
            self.publish_timer = self.create_timer(publish_period, self.publish_human_data)
            self.get_logger().info(f"Continuous publishing at {self.publish_fps} Hz")
        
        # Auto-start burst mode on initialization (optional)
        # self.start_burst_mode()

    def compressed_image_callback(self, msg):
        """Callback for compressed image from rtsp_streamer"""
        try:
            # Decompress image
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if frame is not None:
                with self.frame_lock:
                    self.latest_frame = frame
                    self.last_frame_time = time.time()
                    # Store the compressed image message for HumanDataMsg
                    self.latest_compressed_msg = msg
        except Exception as e:
            self.get_logger().warning(f"Failed to decompress image: {e}")
    
    def gps_callback(self, msg):
        """Callback for GPS data from MAVROS"""
        with self.gps_lock:
            self.latest_gps = msg
    
    def publish_human_data(self):
        """Publish HumanDataMsg with current data (for continuous publishing)"""
        with self.frame_lock:
            compressed_msg = self.latest_compressed_msg
            frame_time = self.last_frame_time
        
        with self.gps_lock:
            gps_data = self.latest_gps
        
        # Only publish if we have image data
        if compressed_msg is not None:
            current_time = time.time()
            # Check if frame is recent (within 2 seconds)
            if current_time - frame_time < 2.0:
                human_data_msg = self.create_human_data_msg(compressed_msg, gps_data)
                self.human_data_publisher.publish(human_data_msg)
    
    def create_human_data_msg(self, compressed_image_msg, gps_data):
        """Create HumanDataMsg from compressed image and GPS data"""
        human_data_msg = HumanDataMsg()
        
        # Set header
        human_data_msg.header = Header()
        human_data_msg.header.stamp = self.get_clock().now().to_msg()
        human_data_msg.header.frame_id = 'base_link'
        
        # Set timestamp
        human_data_msg.stamp = self.get_clock().now().to_msg()
        
        # Set system name
        human_data_msg.system = self.system_name
        
        # Set GPS data (if available)
        if gps_data is not None:
            human_data_msg.gps_data = gps_data
        else:
            # Create empty GPS data if not available
            human_data_msg.gps_data = NavSatFix()
            human_data_msg.gps_data.header.stamp = self.get_clock().now().to_msg()
            human_data_msg.gps_data.header.frame_id = 'base_link'
            human_data_msg.gps_data.status.status = NavSatFix.STATUS_NO_FIX
            human_data_msg.gps_data.status.service = NavSatFix.SERVICE_GPS
        
        # Set compressed image data
        human_data_msg.compressed_images = [compressed_image_msg]
        
        # Leave raw images empty (as per message specification)
        human_data_msg.raw_images = []
        
        return human_data_msg

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
        self.get_logger().info(f'Starting burst recording with high-rate publishing at {self.burst_fps} Hz')
        
        try:
            while self.burst_mode:
                # Wait for first frame
                self.get_logger().info('Waiting for compressed image data...')
                timeout_start = time.time()
                while self.latest_compressed_msg is None and self.burst_mode:
                    time.sleep(0.1)
                    if time.time() - timeout_start > 10.0:  # 10 second timeout
                        self.get_logger().warning('Timeout waiting for compressed image data')
                        return
                
                if not self.burst_mode:
                    break
                    
                self.get_logger().info('Starting burst recording session')
                
                timestamp = time.strftime('%Y%m%d_%H%M%S')
                mcap_path = os.path.join(self.mcap_dir, f'burst_{timestamp}')
                bag_cmd = [
                    'ros2', 'bag', 'record', '-o', mcap_path, '-s', 'sqlite3'
                ] + self.topics_to_record
                self.bag_process = subprocess.Popen(bag_cmd)
                self.get_logger().info(f'Started ros2 bag record: {mcap_path} topics: {self.topics_to_record}')
                
                start_time = time.time()
                frame_count = 0
                last_publish_time = 0
                
                # High-rate publishing during burst mode
                while time.time() - start_time < self.burst_duration_sec and self.burst_mode:
                    current_time = time.time()
                    
                    # Check if we should publish a frame based on burst FPS
                    if current_time - last_publish_time >= (1.0 / self.burst_fps):
                        with self.frame_lock:
                            compressed_msg = self.latest_compressed_msg
                            frame_time = self.last_frame_time
                        
                        with self.gps_lock:
                            gps_data = self.latest_gps
                        
                        if compressed_msg is not None:
                            # Check if frame is recent (within 2 seconds)
                            if current_time - frame_time < 2.0:
                                frame_count += 1
                                if frame_count % max(1, self.burst_fps) == 1:  # Log every second worth of frames
                                    self.get_logger().info(f'Burst publishing frame {frame_count}, elapsed: {current_time - start_time:.1f}s')
                                
                                # Create and publish HumanDataMsg
                                human_data_msg = self.create_human_data_msg(compressed_msg, gps_data)
                                self.human_data_publisher.publish(human_data_msg)
                                
                                last_publish_time = current_time
                            else:
                                self.get_logger().warning(f'Frame too old ({current_time - frame_time:.1f}s), skipping')
                    
                    time.sleep(0.01)  # Small sleep to prevent busy waiting
                
                self.get_logger().info(f'Burst recording completed. Total HumanDataMsg published: {frame_count}')
                if self.bag_process:
                    self.bag_process.terminate()
                    self.bag_process.wait()
                    self.get_logger().info('Stopped ros2 bag record.')
        
        finally:
            if self.bag_process:
                self.bag_process.terminate()
                self.bag_process.wait()
            self.bag_process = None
            self.get_logger().info('Burst loop finished.')

def main():
    rclpy.init()
    recorder = HumanDataPublisherRecorder()
    try:
        recorder.get_logger().info("HumanDataPublisherRecorder node started.")
        recorder.get_logger().info("Continuously publishing HumanDataMsg at configured rate.")
        recorder.get_logger().info("Use /burst_mode/control topic to trigger high-rate burst recording.")
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.stop_burst_mode()
    finally:
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()