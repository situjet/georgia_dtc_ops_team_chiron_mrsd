#!/usr/bin/env python3
import time
import os
import threading
import subprocess
from rclpy.node import Node
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CompressedImage, NavSatFix, NavSatStatus
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
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('mcap_dir', '/root/ros_ws/mcap_recordings')
        self.declare_parameter('burst_fps', 2)  # FPS during burst recording
        self.declare_parameter('publish_fps', 3)  # Normal publishing FPS
        self.declare_parameter('burst_duration_sec', 5)  # Base recording duration
        self.declare_parameter('max_burst_duration_sec', 8)  # Maximum recording duration
        self.declare_parameter('bad_frame_extension_sec', 0.5)  # Extension per bad frame
        self.declare_parameter('system_name', 'mrsd_uav_1')
        self.declare_parameter('enable_continuous_publish', True)  # Enable continuous publishing
        # HumanDataMsg will be recorded instead of individual topics
        # Using absolute topic name to avoid namespace prefix
        default_topics = ['/manual_targets/geolocated']
        self.declare_parameter('topics_to_record', default_topics)
        
        # Get parameters
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.mcap_dir = self.get_parameter('mcap_dir').get_parameter_value().string_value
        self.burst_duration_sec = self.get_parameter('burst_duration_sec').get_parameter_value().integer_value
        self.max_burst_duration_sec = self.get_parameter('max_burst_duration_sec').get_parameter_value().integer_value
        self.bad_frame_extension_sec = self.get_parameter('bad_frame_extension_sec').get_parameter_value().double_value
        self.burst_fps = self.get_parameter('burst_fps').get_parameter_value().integer_value
        self.publish_fps = self.get_parameter('publish_fps').get_parameter_value().integer_value
        self.system_name = self.get_parameter('system_name').get_parameter_value().string_value
        self.enable_continuous_publish = self.get_parameter('enable_continuous_publish').get_parameter_value().bool_value
        self.topics_to_record = self.get_parameter('topics_to_record').get_parameter_value().string_array_value or default_topics
        
        # Initialize OpenCV components
        self.bridge = CvBridge()
        
        # Publisher for HumanDataMsg (using absolute topic name to avoid namespace prefix)
        topic_name = '/manual_targets/geolocated'
        # Ensure the topic name is absolute and won't be affected by namespace
        if not topic_name.startswith('/'):
            topic_name = '/' + topic_name
        self.human_data_publisher = self.create_publisher(HumanDataMsg, topic_name, 10)
        
        # Get the full topic name with namespace
        self.human_data_topic = self.human_data_publisher.topic_name
        self.get_logger().info(f"Full topic name with namespace: {self.human_data_topic}")
        
        # State variables for GPS data
        self.latest_gps = None
        self.gps_lock = threading.Lock()
        
        # GPS warning tracking
        self.last_gps_warning_time = 0
        self.gps_warning_interval = 5.0  # Warn every 5 seconds
        self.skipped_frames_count = 0
        
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
        
        # Bad frame tracking for recording extension
        self.bad_frames_count = 0
        self.current_recording_duration = 0
        
        # Image processing state
        self.latest_frame = None
        self.latest_raw_msg = None
        self.frame_lock = threading.Lock()
        self.last_frame_time = 0
        
        # Create QoS profile for MAVROS compatibility
        center_gps_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        
        # Create QoS profile for image data (match rtsp_streamer publisher)
        image_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Subscribe to raw image from rtsp_streamer
        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            image_qos
        )
        
        # Subscribe to GPS data from MAVROS
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/center_gps/target_gps',
            self.gps_callback,
            center_gps_qos
        )
        
        # Control subscription for burst mode
        self.control_sub = self.create_subscription(
            Bool,
            '/burst_mode/control',
            self.control_callback,
            10
        )
        
        self.get_logger().info(f"Subscribed to raw image topic: {self.image_topic}")
        self.get_logger().info(f"Publishing HumanDataMsg to: {self.human_data_topic}")
        self.get_logger().info(f"System name: {self.system_name}")
        self.get_logger().info(f"Continuous publishing: {'Enabled' if self.enable_continuous_publish else 'Disabled'}")
        
        # Timer for continuous publishing
        if self.enable_continuous_publish:
            publish_period = 1.0 / self.publish_fps
            self.publish_timer = self.create_timer(publish_period, self.publish_human_data)
            self.get_logger().info(f"Continuous publishing at {self.publish_fps} Hz")
        
        # Auto-start burst mode on initialization (optional)
        # self.start_burst_mode()

    def image_callback(self, msg):
        """Callback for raw image from rtsp_streamer"""
        try:
            # Convert ROS Image message to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            if frame is not None:
                with self.frame_lock:
                    self.latest_frame = frame
                    self.last_frame_time = time.time()
                    # Store the raw image message for HumanDataMsg
                    self.latest_raw_msg = msg
        except Exception as e:
            self.get_logger().warning(f"Failed to convert image: {e}")
    
    def gps_callback(self, msg):
        """Callback for GPS data"""
        with self.gps_lock:
            self.latest_gps = msg
    
    def publish_human_data(self):
        """Publish HumanDataMsg with current data (for continuous publishing)"""
        with self.frame_lock:
            raw_msg = self.latest_raw_msg
            frame_time = self.last_frame_time
        
        with self.gps_lock:
            gps_data = self.latest_gps
        
        # Only publish if we have image data
        if raw_msg is not None:
            current_time = time.time()
            # Check if frame is recent (within 2 seconds)
            if current_time - frame_time < 2.0:
                # Check if GPS data is valid before publishing
                if self._is_valid_gps_data(gps_data):
                    human_data_msg = self.create_human_data_msg(raw_msg, gps_data)
                    self.human_data_publisher.publish(human_data_msg)
                else:
                    # Skip publishing and warn about missing GPS
                    self._handle_missing_gps_warning("continuous publishing")
    
    def _is_valid_gps_data(self, gps_data):
        """Check if GPS data is valid and usable"""
        if gps_data is None:
            self.get_logger().warning("GPS data none")
            return False
        
        # # Check GPS fix status
        # if gps_data.status.status == NavSatStatus.STATUS_NO_FIX:
        #     return False
        
        # Check if coordinates are reasonable (not at origin)
        if abs(gps_data.latitude) < 0.0001 and abs(gps_data.longitude) < 0.0001:
            print("GPS data is 0, 0")
            return False
        
        return True
    
    def _handle_missing_gps_warning(self, context):
        """Handle missing GPS data with appropriate warnings"""
        current_time = time.time()
        self.skipped_frames_count += 1
        
        # If in burst recording mode, count bad frames and extend recording time
        if context == "burst recording" and self.burst_mode:
            self.bad_frames_count += 1
            # Extend recording duration by 0.5 seconds per bad frame, but not exceed max duration
            potential_new_duration = self.burst_duration_sec + (self.bad_frames_count * self.bad_frame_extension_sec)
            self.current_recording_duration = min(potential_new_duration, self.max_burst_duration_sec)
            
            self.get_logger().info(
                f"Bad frame #{self.bad_frames_count} detected. "
                f"Recording duration extended to {self.current_recording_duration:.1f}s "
                f"(max: {self.max_burst_duration_sec}s)"
            )
        
        # Only warn periodically to avoid spam
        if current_time - self.last_gps_warning_time > self.gps_warning_interval:
            self.get_logger().warning(
                f"GPS data unavailable during {context}. "
                f"Skipped {self.skipped_frames_count} frames since last warning. "
                f"Data recording paused until valid GPS signal is restored."
            )
            self.last_gps_warning_time = current_time
            # Reset counter after warning
            self.skipped_frames_count = 0
    
    def create_human_data_msg(self, raw_image_msg, gps_data):
        """Create HumanDataMsg from raw image and GPS data
        
        Note: This function should only be called with valid GPS data.
        GPS validation should be done before calling this function.
        """
        human_data_msg = HumanDataMsg()
        
        # Set header
        human_data_msg.header = Header()
        human_data_msg.header.stamp = self.get_clock().now().to_msg()
        human_data_msg.header.frame_id = 'base_link'
        
        # Set timestamp
        human_data_msg.stamp = self.get_clock().now().to_msg()
        
        # Set system name
        human_data_msg.system = self.system_name
        
        # Set GPS data (should be valid at this point)
        human_data_msg.gps_data = gps_data
        
        # Set raw image data
        human_data_msg.raw_images = [raw_image_msg]
        
        # Leave compressed images empty (since we're using raw images)
        human_data_msg.compressed_images = []
        
        return human_data_msg

    def control_callback(self, msg):
        if msg.data:
            print("RECEIVED RECORDING COMMAND: START")
            self.get_logger().info("Received burst mode START command.")
            self.start_burst_mode()
        else:
            print("RECEIVED RECORDING COMMAND: STOP")
            self.get_logger().info("Received burst mode STOP command.")
            self.stop_burst_mode()

    def start_burst_mode(self):
        if self.burst_mode:
            print("RECORDING STATUS: Already recording")
            self.get_logger().info('Burst mode already running.')
            return
        print("RECORDING STATUS: Starting recording")
        self.burst_mode = True
        # Reset bad frame tracking for new recording session
        self.bad_frames_count = 0
        self.current_recording_duration = self.burst_duration_sec
        self.burst_thread = threading.Thread(target=self._burst_loop)
        self.burst_thread.start()

    def stop_burst_mode(self):
        print("RECORDING STATUS: Stopping recording")
        self.burst_mode = False
        if self.burst_thread:
            self.burst_thread.join()
        print("RECORDING STATUS: Recording stopped")

    def _burst_loop(self):
        print("RECORDING STATUS: Recording in progress")
        self.get_logger().info(f'Starting burst recording with high-rate publishing at {self.burst_fps} Hz')
        
        try:
            # Wait for first frame (removed outer while loop)
            self.get_logger().info('Waiting for raw image data...')
            timeout_start = time.time()
            while self.latest_raw_msg is None and self.burst_mode:
                time.sleep(0.1)
                if time.time() - timeout_start > 10.0:  # 10 second timeout
                    print("ERROR: Timeout waiting for raw image data")
                    self.get_logger().warning('Timeout waiting for raw image data')
                    return
            
            if not self.burst_mode:
                return
                
            self.get_logger().info('Starting burst recording session')
            
            # Check if the topic exists
            try:
                topic_list = subprocess.check_output(['ros2', 'topic', 'list'], text=True)
                self.get_logger().info(f'Available topics: {topic_list.strip()}')
            except Exception as e:
                self.get_logger().warning(f'Could not list topics: {e}')
            
            timestamp = time.strftime('%Y%m%d_%H%M%S')
            mcap_path = os.path.join(self.mcap_dir, f'burst_{timestamp}')
            
            # Use absolute topic name without namespace prefix for recording
            absolute_topic_name = '/manual_targets/geolocated'
            topics_to_record_with_ns = [absolute_topic_name]
            self.get_logger().info(f'Will record topic: {absolute_topic_name} (publisher topic: {self.human_data_topic})')
            
            bag_cmd = [
                'ros2', 'bag', 'record', '-o', mcap_path, '-s', 'sqlite3'
            ] + topics_to_record_with_ns
            try:
                self.bag_process = subprocess.Popen(bag_cmd)
                print(f"RECORDING STATUS: Started ros2 bag record: {mcap_path}")
                self.get_logger().info(f'Started ros2 bag record: {mcap_path} topics: {topics_to_record_with_ns}')
            except Exception as e:
                print(f"ERROR: Failed to start ros2 bag record: {e}")
                self.get_logger().error(f'Failed to start ros2 bag record: {e}')
                return
            
            start_time = time.time()
            frame_count = 0
            last_publish_time = 0
            
            # High-rate publishing during burst mode
            while time.time() - start_time < self.current_recording_duration and self.burst_mode:
                current_time = time.time()
                
                # Check if we should publish a frame based on burst FPS
                if current_time - last_publish_time >= (1.0 / self.burst_fps):
                    with self.frame_lock:
                        raw_msg = self.latest_raw_msg
                        frame_time = self.last_frame_time
                    
                    with self.gps_lock:
                        gps_data = self.latest_gps
                    
                    if raw_msg is not None:
                        # Check if frame is recent (within 2 seconds)
                        if current_time - frame_time < 2.0:
                            # Check if GPS data is valid before publishing
                            if self._is_valid_gps_data(gps_data):
                                frame_count += 1
                                if frame_count % max(1, self.burst_fps) == 1:  # Log every second worth of frames
                                    self.get_logger().info(f'Burst publishing frame {frame_count}, elapsed: {current_time - start_time:.1f}s')
                                
                                # Create and publish HumanDataMsg
                                human_data_msg = self.create_human_data_msg(raw_msg, gps_data)
                                self.human_data_publisher.publish(human_data_msg)
                                
                                # Debug log to confirm message was published
                                if frame_count == 1:
                                    self.get_logger().info(f'Publishing HumanDataMsg to {self.human_data_topic}')
                                
                                last_publish_time = current_time
                            else:
                                # Skip frame due to invalid GPS and warn
                                self._handle_missing_gps_warning("burst recording")
                        else:
                            self.get_logger().warning(f'Frame too old ({current_time - frame_time:.1f}s), skipping')
                
                time.sleep(0.01)  # Small sleep to prevent busy waiting
            
            actual_duration = time.time() - start_time
            print(f"RECORDING STATUS: Recording completed. Total HumanDataMsg published: {frame_count}")
            self.get_logger().info(
                f'Burst recording completed. Total HumanDataMsg published: {frame_count}, '
                f'Bad frames: {self.bad_frames_count}, '
                f'Actual duration: {actual_duration:.1f}s, '
                f'Target duration: {self.current_recording_duration:.1f}s'
            )
            if self.bag_process:
                try:
                    self.bag_process.terminate()
                    self.bag_process.wait()
                    print("RECORDING STATUS: Stopped ros2 bag record successfully")
                    self.get_logger().info('Stopped ros2 bag record.')
                except Exception as e:
                    print(f"ERROR: Failed to stop ros2 bag record: {e}")
                    self.get_logger().error(f'Failed to stop ros2 bag record: {e}')
        
        except Exception as e:
            print(f"ERROR: Recording failed: {e}")
            self.get_logger().error(f'Recording failed: {e}')
        finally:
            if self.bag_process:
                try:
                    self.bag_process.terminate()
                    self.bag_process.wait()
                    print("RECORDING STATUS: Cleanup completed")
                except Exception as e:
                    print(f"ERROR: Failed to cleanup recording process: {e}")
                    self.get_logger().error(f'Failed to cleanup recording process: {e}')
            self.bag_process = None
            # Reset recording status after recording completes
            self.burst_mode = False
            print("RECORDING STATUS: Recording completed and status reset - ready for next command")
            print("RECORDING STATUS: Burst loop finished")
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