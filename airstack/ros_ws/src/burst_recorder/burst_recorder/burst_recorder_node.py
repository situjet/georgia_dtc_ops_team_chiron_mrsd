import time
import os
import threading
import subprocess
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

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
        
        # Force OpenCV to prioritize FFmpeg backend over GStreamer. This will make it use CPU only.
        os.environ["OPENCV_VIDEOIO_PRIORITY_FFMPEG"] = "9999"
        
        # Check if OpenCV is available
        if not CV_AVAILABLE:
            self.get_logger().fatal("OpenCV is not available! Node cannot function without OpenCV. Shutting down.")
            raise RuntimeError("OpenCV not available")
        
        # Declare all parameters with defaults
        self.declare_parameter('rtsp_url', 'rtsp://10.3.1.124:8556/ghadron')
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
        self.rtsp_url = self.get_parameter('rtsp_url').get_parameter_value().string_value
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
        self.burst_mode = False
        self.bag_process = None
        self.burst_thread = None
        self.control_sub = self.create_subscription(
            Bool,
            '/burst_mode/control',
            self.control_callback,
            10
        )
        
        # Auto-start burst mode on initialization
        self.get_logger().info("Auto-starting burst mode...")
        self.start_burst_mode()

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
        self.get_logger().info(f'Starting burst loop, connecting to RTSP: {self.rtsp_url}')
        
        # Force TCP transport for RTSP
        os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS'] = 'rtsp_transport;tcp'
        self.get_logger().info("RTSP: Setting FFMPEG capture options to prefer TCP.")

        cap = cv2.VideoCapture(self.rtsp_url)
        
        if not cap.isOpened():
            self.get_logger().error(f'Failed to open RTSP stream: {self.rtsp_url}')
            # Clean up env var on failure
            if 'OPENCV_FFMPEG_CAPTURE_OPTIONS' in os.environ:
                del os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS']
            return
        
        self.get_logger().info('RTSP stream connected successfully.')

        try:
            while self.burst_mode:
                timestamp = time.strftime('%Y%m%d_%H%M%S')
                mcap_path = os.path.join(self.mcap_dir, f'burst_{timestamp}')
                bag_cmd = [
                    'ros2', 'bag', 'record', '-o', mcap_path, '-s', 'sqlite3'
                ] + self.topics_to_record
                self.bag_process = subprocess.Popen(bag_cmd)
                self.get_logger().info(f'Started ros2 bag record: {mcap_path} topics: {self.topics_to_record}')
                
                start_time = time.time()
                frame_count = 0
                while time.time() - start_time < self.duration_sec and self.burst_mode:
                    ret, frame = cap.read()
                    if not ret:
                        self.get_logger().warning(f'Frame grab failed at frame {frame_count}')
                        continue
                    
                    frame_count += 1
                    if frame_count % 30 == 1:  # Log every 30 frames
                        self.get_logger().info(f'Processing frame {frame_count}, elapsed: {time.time() - start_time:.1f}s')
                    
                    img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
                    img_msg.header.stamp = self.get_clock().now().to_msg()
                    img_msg.header.frame_id = 'camera'
                    self.publisher.publish(img_msg)
                    
                    # Approximate frame rate control
                    time.sleep(1.0 / self.fps)
                
                self.get_logger().info(f'Burst recording completed. Total frames: {frame_count}')
                if self.bag_process:
                    self.bag_process.terminate()
                    self.bag_process.wait()
                    self.get_logger().info('Stopped ros2 bag record.')
        
        finally:
            if cap:
                cap.release()
            if self.bag_process:
                self.bag_process.terminate()
                self.bag_process.wait()
            self.bag_process = None
            # Clean up environment variable
            if 'OPENCV_FFMPEG_CAPTURE_OPTIONS' in os.environ:
                del os.environ['OPENCV_FFMPEG_CAPTURE_OPTIONS']
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
