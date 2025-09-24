import time
import os
import threading
import subprocess
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Bool
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

class BurstModeRecorder(Node):
    def __init__(self):
        super().__init__('burst_recorder_node')
        
        # Check if OpenCV is available
        if not CV_AVAILABLE:
            self.get_logger().fatal("OpenCV is not available! Node cannot function without OpenCV. Shutting down.")
            raise RuntimeError("OpenCV not available")
        
        # Declare all parameters with defaults
        self.declare_parameter('compressed_image_topic', '/image_raw_compressed')
        self.declare_parameter('mcap_dir', '/root/ros_ws/mcap_recordings')
        self.declare_parameter('fps', 24)
        self.declare_parameter('duration_sec', 10)
        default_topics = [
            '/camera/image_raw',
            '/dtc_mrsd_/mavros/global_position/global',
            '/dtc_mrsd_/mavros/global_position/rel_alt', 
            '/dtc_mrsd_/mavros/imu/data',
            '/dtc_mrsd_/mavros/global_position/compass_hdg'
        ]
        self.declare_parameter('topics_to_record', default_topics)
        
        # Get parameters
        self.compressed_image_topic = self.get_parameter('compressed_image_topic').get_parameter_value().string_value
        self.mcap_dir = self.get_parameter('mcap_dir').get_parameter_value().string_value
        self.duration_sec = self.get_parameter('duration_sec').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.topics_to_record = self.get_parameter('topics_to_record').get_parameter_value().string_array_value or default_topics
        
        # Initialize OpenCV components
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        
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
        self.frame_lock = threading.Lock()
        self.last_frame_time = 0
        
        # Subscribe to compressed image from rtsp_streamer
        self.image_sub = self.create_subscription(
            CompressedImage,
            self.compressed_image_topic,
            self.compressed_image_callback,
            10
        )
        
        # Control subscription
        self.control_sub = self.create_subscription(
            Bool,
            '/burst_mode/control',
            self.control_callback,
            10
        )
        
        self.get_logger().info(f"Subscribed to compressed image topic: {self.compressed_image_topic}")
        
        # Auto-start burst mode on initialization
        self.get_logger().info("Auto-starting burst mode...")
        self.start_burst_mode()

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
        except Exception as e:
            self.get_logger().warning(f"Failed to decompress image: {e}")

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
        self.get_logger().info(f'Starting burst loop, using compressed images from: {self.compressed_image_topic}')
        
        try:
            while self.burst_mode:
                # Wait for first frame
                self.get_logger().info('Waiting for compressed image data...')
                timeout_start = time.time()
                while self.latest_frame is None and self.burst_mode:
                    time.sleep(0.1)
                    if time.time() - timeout_start > 10.0:  # 10 second timeout
                        self.get_logger().warning('Timeout waiting for compressed image data')
                        return
                
                if not self.burst_mode:
                    break
                    
                self.get_logger().info('Compressed image data received, starting recording session')
                
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
                
                while time.time() - start_time < self.duration_sec and self.burst_mode:
                    current_time = time.time()
                    
                    # Check if we should publish a frame based on target FPS
                    if current_time - last_publish_time >= (1.0 / self.fps):
                        with self.frame_lock:
                            if self.latest_frame is not None:
                                frame = self.latest_frame.copy()
                                frame_time = self.last_frame_time
                            else:
                                frame = None
                                frame_time = 0
                        
                        if frame is not None:
                            # Check if frame is recent (within 2 seconds)
                            if current_time - frame_time < 2.0:
                                frame_count += 1
                                if frame_count % max(1, self.fps) == 1:  # Log every second worth of frames
                                    self.get_logger().info(f'Processing frame {frame_count}, elapsed: {current_time - start_time:.1f}s')
                                
                                img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                                img_msg.header.stamp = self.get_clock().now().to_msg()
                                img_msg.header.frame_id = 'camera'
                                self.publisher.publish(img_msg)
                                
                                last_publish_time = current_time
                            else:
                                self.get_logger().warning(f'Frame too old ({current_time - frame_time:.1f}s), skipping')
                    
                    time.sleep(0.01)  # Small sleep to prevent busy waiting
                
                self.get_logger().info(f'Burst recording completed. Total frames: {frame_count}')
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
    recorder = BurstModeRecorder()
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
