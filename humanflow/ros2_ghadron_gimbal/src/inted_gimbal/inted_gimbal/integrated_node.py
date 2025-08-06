#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, NavSatFix, CompressedImage
from vision_msgs.msg import Detection2D, Detection2DArray
from geometry_msgs.msg import Point, PoseStamped, Vector3
from std_msgs.msg import Bool, Float32, String, Float64, Header
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, SetParametersResult
import cv2
import threading
from cv_bridge import CvBridge
from .streaming import Streaming
from .yolo import Detection
# from .track import SimpleTracker
from .gps_manager import gps_manager
import time
import os
import math
from datetime import datetime
import numpy as np
# from .pose_estimation import PoseEstimation
from humanflow_msgs.msg import HumanPoseArray17
from collections import deque
# Import MultiThreadedExecutor
from rclpy.executors import MultiThreadedExecutor
import traceback # <<< ADDED IMPORT

# import the profiler module
from .profiler import ProfileManager, FunctionProfiler, ResourceMonitor

# --- Add Haversine distance function here ---
def calculate_haversine_distance(lat1, lon1, lat2, lon2):
    """
    Calculate the great circle distance in meters between two points 
    on the earth (specified in decimal degrees)
    """
    # Convert decimal degrees to radians 
    lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])

    # Haversine formula 
    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a)) 
    r = 6371000 # Radius of earth in meters. Use 6371 for kilometers
    return c * r
# --- End of Haversine distance function ---

class IntegratedNode(Node):
    def __init__(self):
        super().__init__('integrated_node')

        # initialize the profiler manager
        self.profile_manager = ProfileManager(output_dir='/home/dtc/humanflow/ros2_ghadron_gimbal/profiling_results')
        self.function_profiler = self.profile_manager.function_profiler
        
        self.declare_parameter('enable_profiling', True, 
            ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, 
                                description='Enable performance profiling'))
        self.profiling_enabled = self.get_parameter('enable_profiling').value
        
        self.declare_parameter('profiling_report_interval', 300.0, 
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, 
                                description='Interval (seconds) between profiling reports'))
        self.profiling_report_interval = self.get_parameter('profiling_report_interval').value
        
        if self.profiling_enabled:
            self.get_logger().info("Starting performance profiling")
            self.profile_manager.start_profiling()
            # create a timer to generate profiling reports periodically
            self.profiling_timer = self.create_timer(self.profiling_report_interval, self._generate_profiling_report)

        # QoS profile for image transport - use BEST_EFFORT for lower latency
        image_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        # Use standard ROS2 compressed image topics instead of image_transport
        # This is the raw image publisher (usually not used with high latency links)
        self.image_pub = self.create_publisher(
            Image, '/camera/image_raw', 
            qos_profile=image_qos
        )
        
        # This is the compressed image publisher - most subscribers will use this
        self.compressed_image_pub = self.create_publisher(
            CompressedImage, '/image_raw/compressed', 
            qos_profile=image_qos
        )

        self.detection_pub = self.create_publisher(Detection2DArray, '/detection', 10)
        # add a new publisher for publishing the detection center coordinates
        self.detection_center_pub = self.create_publisher(Vector3, '/detection_center', 10)
        # self.pose_pub = self.create_publisher(HumanPoseArray17, '/human_pose', 10)

        # Add navigation data subscribers
        sensor_qos = QoSProfile(
            depth=10, # Increased depth for sensor data
            reliability=QoSReliabilityPolicy.BEST_EFFORT, # BEST_EFFORT is often fine for sensor data
            history=QoSHistoryPolicy.KEEP_LAST
        )
        
        self.gps_sub = self.create_subscription(NavSatFix, '/robot_1/mavros/global_position/global', self.gps_callback, qos_profile=sensor_qos)
        self.height_sub = self.create_subscription(Float64, '/robot_1/mavros/global_position/rel_alt', self.height_callback, qos_profile=sensor_qos)
        # Add subscription for heading
        self.heading_sub = self.create_subscription(Float64, '/robot_1/mavros/global_position/compass_hdg', self.heading_callback, qos_profile=sensor_qos)
        
        # Add subscription for camera mode
        self.camera_mode_sub = self.create_subscription(String, '/camera_mode', self.camera_mode_callback, 10)
        self.current_camera_mode = 'EO'  # Default camera mode
        
        # Add subscription for gimbal mode
        self.gimbal_mode_sub = self.create_subscription(String, '/gimbal_mode', self.gimbal_mode_callback, 10)

        # Add subscription for gimbal attitude
        self.gimbal_attitude_sub = self.create_subscription(Vector3, '/gimbal_attitude', self.gimbal_attitude_callback, sensor_qos)
        self.current_gimbal_attitude = Vector3()  # Initialize with zeros

        # Comment out the subscriptions for gimbal control
        # self.spin_sub = self.create_subscription(Bool, 'gimbal/spin', self.spin_loop, 10)
        # self.lockon_sub = self.create_subscription(Bool, 'gimbal/lockon', self.lockon_loop, 10)
        
        self.streaming_module = Streaming()
        self.detection_model = Detection()
        self.bridge = CvBridge()
        # self.direction = -1.0  # Not needed anymore

        # Add a separate thread for detection to not block image publishing
        self.detection_thread = None
        self.frame_lock = threading.Lock()

        # Add structure to store combined frame and GPS data captured at the same time
        # This ensures consistency between the frame used for detection and the associated sensor data
        self.latest_processed_frame_info = None # <<< RENAMED and will store combined data

        # We'll initialize GPSEstimator later after getting the first frame

        # Current drone state (keep for potential direct use elsewhere)
        self.current_gps = None
        self.current_height = None
        self.current_heading = None
        self.latest_frame = None
        self.latest_frame_timestamp = None
        
        # Deques to store recent sensor readings with timestamps
        self.gps_readings = deque(maxlen=20)
        self.height_readings = deque(maxlen=20)
        self.heading_readings = deque(maxlen=20)
        self.gimbal_attitude_readings = deque(maxlen=20)
        
        # Gimbal mode state
        self.current_gimbal_mode = "OFF" # Default mode, adjust if needed
        
        # Local position related state
        self.current_local_pose = None
        self.local_pose_lock = threading.Lock()
        
        # IMU data for checking if drone is horizontal
        self.is_horizontal = False  # Flag to track if drone is horizontal
        self.max_tilt_angle = 50.0  # Maximum tilt angle in degrees for "horizontal" (adjust as needed)
        self.current_roll = 0.0
        self.current_pitch = 0.0

        # Add parameters for recording the stream
        self.declare_parameter('record_stream', False, 
            ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, 
                               description='Whether to record the stream locally'))
        self.record_stream = self.get_parameter('record_stream').value
        
        self.declare_parameter('record_path', '/home/dtc/humanflow/ros2_ghadron_gimbal', 
            ParameterDescriptor(type=ParameterType.PARAMETER_STRING, 
                               description='Directory to save recorded streams'))
        self.record_path = self.get_parameter('record_path').value
        
        # Initialize video recording only after streaming has started
        # (Move this to start() method instead of here)
        self.video_writer = None
        
        # Add compression quality parameter
        self.declare_parameter('jpeg_quality', 10, 
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER, 
                               description='JPEG compression quality (0-100)'))
        self.jpeg_quality = self.get_parameter('jpeg_quality').value

        # Parameters for center threshold 
        self.declare_parameter('center_threshold', 300,
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER,
                              description='Maximum pixel distance from center to consider a detection "centered"'))
        self.center_threshold = self.get_parameter('center_threshold').value

        # Add parameter to controPubll minimum distance between targets
        self.declare_parameter('min_target_distance', 5.0,
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                              description='Minimum distance in meters between distinct targets'))
        self.min_target_distance = self.get_parameter('min_target_distance').value

        # Add parameter to control maximum tilt angle
        self.declare_parameter('max_tilt_angle', 20.0,
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                              description='Maximum tilt angle in degrees for the drone to be considered horizontal'))
        self.max_tilt_angle = self.get_parameter('max_tilt_angle').value

        # <<< ADDED PARAMETER >>>
        # Parameter to control maximum allowed total time difference between frame and sensors
        self.declare_parameter('max_sync_time_diff', 0.2, # Default to 0.2 seconds 
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                              description='Maximum total time difference (seconds) allowed between frame and sensor data for processing'))
        self.max_sync_time_diff = self.get_parameter('max_sync_time_diff').value
        # <<< END ADDED PARAMETER >>>

        # Publisher for target GPS coordinates
        self.gps_target_pub = self.create_publisher(NavSatFix, '/target_gps', 10)
        self.precise_gps_target_pub = self.create_publisher(NavSatFix, '/precise_target_gps', 10)

        # Publisher for the list of stored GPS targets
        self.gps_targets_list_pub = self.create_publisher(NavSatFix, '/target_gps_list', 10)
        # Create a timer to publish GPS targets list every 1 second - Corrected: Create timer here
        self.gps_list_timer = self.create_timer(1.0, self._publish_gps_targets_callback)
        
 
        # Logging for image publishing rate
        self.image_publish_counter = 0
        self.last_image_log_time = time.time()
        self.log_interval = 5.0  # Log every 5 seconds

        # Add variables to control frame rate
        self.detection_fps = 10  # Target FPS for detection
        self.last_detection_time = 0  # Timestamp of last detection
        
        # GPSEstimator will be initialized in start() after getting image dimensions
        self.gps_manager = None
        self.image_center = None

        self.start()

    # add a function to generate and save the profiling report
    def _generate_profiling_report(self):
        """generate and save the profiling report"""
        if not self.profiling_enabled:
            return
            
        try:
            report_path = self.profile_manager.save_results()
            self.get_logger().info(f"Generated profiling report: {report_path}")
            
            # print the key performance metrics to the log
            res_stats = self.profile_manager.resource_monitor.get_stats()
            self.get_logger().info(f"performance metrics - CPU: {res_stats['cpu']['avg']:.2f}% (max: {res_stats['cpu']['max']:.2f}%), "
                                 f"memory: {res_stats['memory']['avg']:.2f}MB (max: {res_stats['memory']['max']:.2f}MB), "
                                 f"thread count: {res_stats['threads']['latest']}")
        except Exception as e:
            self.get_logger().error(f"Error generating profiling report: {e}")
            traceback.print_exc()

    # use the decorator to profile the key functions
    @property
    def profile(self):
        return self.function_profiler

    # Navigation data callbacks - simplified to just update current values
    @FunctionProfiler()
    def gps_callback(self, msg):
        callback_start_time = time.time()
        # Just store the latest GPS data
        self.current_gps = (msg.latitude, msg.longitude)
        callback_duration = time.time() - callback_start_time
        self.get_logger().debug(f"gps_callback: Total processing time: {callback_duration:.6f}s")

        # Store timestamped reading
        ros_time_sec = self.get_clock().now().nanoseconds / 1e9
        self.gps_readings.append((ros_time_sec, (msg.latitude, msg.longitude)))

    @FunctionProfiler()
    def height_callback(self, msg):
        callback_start_time = time.time()
        # Just store the latest height data
        self.current_height = msg.data
        callback_duration = time.time() - callback_start_time
        self.get_logger().debug(f"height_callback: Total processing time: {callback_duration:.6f}s")

        # Store timestamped reading
        ros_time_sec = self.get_clock().now().nanoseconds / 1e9
        self.height_readings.append((ros_time_sec, msg.data))

    @FunctionProfiler()
    def heading_callback(self, msg):
        callback_start_time = time.time()
        # Just store the latest heading data
        self.current_heading = msg.data
        callback_duration = time.time() - callback_start_time
        self.get_logger().debug(f"heading_callback: Total processing time: {callback_duration:.6f}s")

        # Store timestamped reading
        ros_time_sec = self.get_clock().now().nanoseconds / 1e9
        self.heading_readings.append((ros_time_sec, msg.data))

    def start(self):
        start_method_time = time.time() # <<< TIMER START
        
        streaming_start_time = time.time() # <<< TIMER START
        self.streaming_module.start_pipeline()
        streaming_start_duration = time.time() - streaming_start_time # <<< TIMER END
        self.get_logger().debug(f"start: Streaming module initialization took {streaming_start_duration:.6f}s")
        
        model_load_start_time = time.time() # <<< TIMER START
        self.detection_model.load_model()
        model_load_duration = time.time() - model_load_start_time # <<< TIMER END
        self.get_logger().debug(f"start: Detection model loading took {model_load_duration:.6f}s")
        
        # Give streaming some time to initialize before trying to record
        time.sleep(2.0)  # Add a 2-second delay
        
        # Get a frame to determine image dimensions
        img_dim_start_time = time.time() # <<< TIMER START
        self.init_image_dimensions()
        img_dim_duration = time.time() - img_dim_start_time # <<< TIMER END
        self.get_logger().debug(f"start: Image dimension initialization took {img_dim_duration:.6f}s")
        
        # Initialize video recording if enabled
        # if self.record_stream:
        #     self.init_video_recording()

        # Separate threads for streaming and detection
        # self.get_logger().info("Creating stream thread...") # <<< ADDED PRINT
        thread_start_time = time.time() # <<< TIMER START
        self.stream_thread = threading.Thread(target=self.process_stream)
        self.stream_thread.daemon = True
        # self.get_logger().info("Starting stream thread...") # <<< ADDED PRINT
        self.stream_thread.start()
        # self.get_logger().info("Stream thread started.") # <<< ADDED PRINT

        # self.get_logger().info("Creating detection thread...") # <<< ADDED PRINT
        self.detection_thread = threading.Thread(target=self.process_detection)
        self.detection_thread.daemon = True
        # self.get_logger().info("Starting detection thread...") # <<< ADDED PRINT
        self.detection_thread.start()
        thread_start_duration = time.time() - thread_start_time # <<< TIMER END
        self.get_logger().debug(f"start: Thread creation and startup took {thread_start_duration:.6f}s")
        #  self.get_logger().info("Detection thread started.") # <<< ADDED PRINT
        
        total_start_duration = time.time() - start_method_time # <<< TIMER END
        self.get_logger().info(f"start: Total initialization time: {total_start_duration:.6f}s")

    def init_image_dimensions(self):
        init_start_time = time.time() # <<< TIMER START
        # Try multiple times to get the first frame to determine image dimensions
        max_attempts = 10
        retrieved = False
        for attempt in range(max_attempts):
            # self.get_logger().info(f"Trying to get first frame for dimension detection (attempt {attempt+1}/{max_attempts})")
            retrieve_attempt_start = time.time() # <<< TIMER START
            result = self.streaming_module.retrieve_image()
            retrieve_attempt_duration = time.time() - retrieve_attempt_start # <<< TIMER END
            self.get_logger().debug(f"init_image_dimensions: Attempt {attempt+1} retrieve took {retrieve_attempt_duration:.6f}s")
            
            frame = None
            if result is not None:
                frame, timestamp = result
                # self.get_logger().info(f"First frame received with timestamp: {timestamp:.6f}")
            if frame is not None:
                height, width = frame.shape[:2]
                self.image_center = (width // 2, height // 2)
                # self.get_logger().info(f"Image dimensions: {width}x{height}, center: {self.image_center}")

                # First create gps_manager instance, without passing configuration
                # self.get_logger().info("Initializing gps_manager...") # <<< ADDED PRINT
                gps_mgr_start = time.time() # <<< TIMER START
                self.gps_manager = gps_manager()
                gps_mgr_duration = time.time() - gps_mgr_start # <<< TIMER END
                self.get_logger().info("gps_manager initialized.") # <<< ADDED PRINT
                self.get_logger().debug(f"init_image_dimensions: gps_manager initialization took {gps_mgr_duration:.6f}s")

                # Store this frame initially? No, let process_stream handle the first update.
                # self.get_logger().info("Storing initial frame...") # <<< ADDED PRINT
                # with self.frame_lock:
                #     self.latest_frame = frame.copy() # <<< REMOVED Initial storage
                # self.get_logger().info("Initial frame stored.") # <<< ADDED PRINT
                retrieved = True
                break
            # self.get_logger().warn("Failed to get frame, retrying...")
            time.sleep(1.0)  # Wait a second before trying again
        
        # If we couldn't get image dimensions, use defaults
        if not retrieved:
            self.get_logger().error("Could not determine image dimensions - using default values")
            self.image_center = (640, 512)  # Default for 1280x720
            # self.get_logger().info("Initializing gps_manager (with defaults)...") # <<< ADDED PRINT
            gps_mgr_start = time.time() # <<< TIMER START
            self.gps_manager = gps_manager()
            gps_mgr_duration = time.time() - gps_mgr_start # <<< TIMER END
            # self.get_logger().info("gps_manager initialized (with defaults).") # <<< ADDED PRINT
            self.get_logger().debug(f"init_image_dimensions: gps_manager initialization with defaults took {gps_mgr_duration:.6f}s")
            
        init_total_duration = time.time() - init_start_time # <<< TIMER END
        self.get_logger().debug(f"init_image_dimensions: Total initialization time: {init_total_duration:.6f}s")

    def init_video_recording(self):
        init_rec_start_time = time.time() # <<< TIMER START
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"stream_{timestamp}.mkv"
        filepath = os.path.join(self.record_path, filename)
        
        # Ensure directory exists
        os.makedirs(self.record_path, exist_ok=True)
        
        # Try multiple times to get the first frame
        max_attempts = 5
        initialized = False
        for attempt in range(max_attempts):
            # self.get_logger().info(f"Trying to get first frame for recording (attempt {attempt+1}/{max_attempts})")
            frame_retrieve_start = time.time() # <<< TIMER START
            result = self.streaming_module.retrieve_image()
            frame_retrieve_duration = time.time() - frame_retrieve_start # <<< TIMER END
            self.get_logger().debug(f"init_video_recording: Attempt {attempt+1} retrieve took {frame_retrieve_duration:.6f}s")
            
            frame = None
            if result is not None:
                frame, frame_timestamp = result
                # self.get_logger().info(f"First frame for recording received with timestamp: {frame_timestamp:.6f}")
            if frame is not None:
                height, width = frame.shape[:2]
                # Use a common codec like MP4V or XVID. MKV supports various codecs.
                writer_start_time = time.time() # <<< TIMER START
                fourcc = cv2.VideoWriter_fourcc(*'MP4V') # Changed codec
                self.video_writer = cv2.VideoWriter(filepath, fourcc, 30.0, (width, height))
                writer_init_duration = time.time() - writer_start_time # <<< TIMER END
                self.get_logger().debug(f"init_video_recording: VideoWriter initialization took {writer_init_duration:.6f}s")
                # self.get_logger().info(f"Recording stream to {filepath}")
                # Store this frame as latest_frame so we don't lose it
                frame_lock_start = time.time() # <<< TIMER START
                with self.frame_lock:
                    self.latest_processed_frame_info = {
                        'frame': frame.copy(),
                        'gps_data': None,
                        'height_data': None,
                        'heading_data': None,
                        'exact_timestamp': frame_timestamp
                    }
                frame_lock_duration = time.time() - frame_lock_start # <<< TIMER END
                self.get_logger().debug(f"init_video_recording: Frame lock operation took {frame_lock_duration:.6f}s")
                initialized = True
                break
            # self.get_logger().warn("Failed to get frame, retrying...")
            time.sleep(1.0)  # Wait a second before trying again
        
        if not initialized:
            # self.get_logger().error("Could not initialize video recording - no frame received after multiple attempts")
            pass
            
        init_rec_total_duration = time.time() - init_rec_start_time # <<< TIMER END
        self.get_logger().debug(f"init_video_recording: Total initialization time: {init_rec_total_duration:.6f}s (success: {initialized})")

    @FunctionProfiler()
    def process_stream(self):
        # Target FPS for retrieving frames
        stream_rate = 5 # set to retrieve 5 frames per second
        frame_time = 1.0 / stream_rate

        # Use rclpy.Rate for rate control
        rate = self.create_rate(stream_rate, self.get_clock())

        # record performance data
        frame_count = 0
        total_loop_time = 0
        max_loop_time = 0
        
        while rclpy.ok():
            loop_start_time = time.time()

            # Get the latest image frame from the streaming module
            try:
                result = self.streaming_module.retrieve_image()
                
                if result is not None:
                    frame, frame_timestamp = result # frame_timestamp is from streaming_module
                    
                    if frame is not None:
                        # self.get_logger().info(f"Retrieve Image Timestamp: {frame_timestamp:.6f}")

                        # Find closest sensor readings
                        closest_gps_reading = self._get_closest_reading(self.gps_readings, frame_timestamp)
                        closest_height_reading = self._get_closest_reading(self.height_readings, frame_timestamp)
                        closest_heading_reading = self._get_closest_reading(self.heading_readings, frame_timestamp)
                        closest_gimbal_attitude_reading = self._get_closest_reading(self.gimbal_attitude_readings, frame_timestamp)

                        # Store combined info and update latest_frame for compatibility
                        with self.frame_lock:
                            self.latest_frame = frame.copy()
                            self.latest_frame_timestamp = frame_timestamp
                            
                            self.latest_processed_frame_info = {
                                'frame': frame.copy(),
                                'frame_timestamp': frame_timestamp,
                                'gps_reading': closest_gps_reading,
                                'height_reading': closest_height_reading,
                                'heading_reading': closest_heading_reading,
                                'gimbal_attitude_reading': closest_gimbal_attitude_reading,
                            }
                            
                        # Publish the frame to ROS topics
                        try:
                            # publish the original image
                            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                            img_msg.header.stamp = self.get_clock().now().to_msg()
                            self.image_pub.publish(img_msg)
                            
                            # publish compressed image
                            compressed_img = CompressedImage()
                            compressed_img.header.stamp = self.get_clock().now().to_msg()
                            compressed_img.format = "jpeg"
                            compressed_img.data = np.array(cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])[1]).tobytes()
                            self.compressed_image_pub.publish(compressed_img)
                        except Exception as e:
                            self.get_logger().error(f"Error publishing image: {str(e)}")
            except Exception as e:
                self.get_logger().error(f"Exception during retrieve_image: {type(e).__name__} - {e}", exc_info=True)
                time.sleep(1.0)
                
            # update performance metrics
            loop_duration = time.time() - loop_start_time
            frame_count += 1
            total_loop_time += loop_duration
            max_loop_time = max(max_loop_time, loop_duration)
            
            rate.sleep()

    @FunctionProfiler()
    def process_detection(self):
        try:
            # Target detection rate (per second)
            detection_rate = 7
            rate = self.create_rate(detection_rate, self.get_clock())

            while rclpy.ok():
                loop_start_time = time.time()
                frame_for_detection = None
                
                current_frame_info = None
                with self.frame_lock:
                    if self.latest_processed_frame_info:
                        # Perform a shallow copy of the dictionary. The frame itself is copied when stored.
                        current_frame_info = self.latest_processed_frame_info.copy()
                        self.latest_processed_frame_info = None # Mark as consumed
                    # No else needed, current_frame_info remains None if the if condition is false
                
                if current_frame_info is None:
                    self.get_logger().debug("No new frame available for detection, waiting...")
                    rate.sleep()
                    continue

                frame_for_detection = current_frame_info['frame']
                frame_timestamp = current_frame_info['frame_timestamp']
                gps_reading = current_frame_info['gps_reading']
                height_reading = current_frame_info['height_reading']
                heading_reading = current_frame_info['heading_reading']
                gimbal_attitude_reading = current_frame_info['gimbal_attitude_reading']
                
                # Check if all necessary data is present
                if not all([frame_for_detection is not None, gps_reading, height_reading, heading_reading, gimbal_attitude_reading]):
                    missing_data = []
                    if frame_for_detection is None: missing_data.append("frame")
                    if not gps_reading: missing_data.append("GPS reading")
                    if not height_reading: missing_data.append("height reading")
                    if not heading_reading: missing_data.append("heading reading")
                    if not gimbal_attitude_reading: missing_data.append("gimbal attitude reading")
                    # self.get_logger().warn(f"Missing data for detection: {', '.join(missing_data)} from latest_processed_frame_info.")
                    rate.sleep()
                    continue

                # Extract actual data values and their timestamps
                gps_ts, current_gps_val = gps_reading
                height_ts, current_height_val = height_reading
                heading_ts, current_heading_val = heading_reading # This is in degrees
                gimbal_ts, current_gimbal_attitude_val = gimbal_attitude_reading # This is Vector3 msg (radians)

                # Synchronization check
                time_diff_gps = abs(frame_timestamp - gps_ts)
                time_diff_height = abs(frame_timestamp - height_ts)
                time_diff_heading = abs(frame_timestamp - heading_ts)
                time_diff_gimbal = abs(frame_timestamp - gimbal_ts)

                max_diff = self.max_sync_time_diff
                if (time_diff_gps > max_diff or
                    time_diff_height > max_diff or
                    time_diff_heading > max_diff or
                    time_diff_gimbal > max_diff):
                    self.get_logger().warn(
                        f"Data stale for detection. Frame_ts: {frame_timestamp:.3f}, "
                        f"GPS_ts: {gps_ts:.3f} (diff: {time_diff_gps:.3f}), "
                        f"Height_ts: {height_ts:.3f} (diff: {time_diff_height:.3f}), "
                        f"Heading_ts: {heading_ts:.3f} (diff: {time_diff_heading:.3f}), "
                        f"Gimbal_ts: {gimbal_ts:.3f} (diff: {time_diff_gimbal:.3f}). "
                        f"Max allowed: {max_diff:.3f}s."
                    )
                    rate.sleep()
                    continue
                
                self.get_logger().debug(
                    f"Data synchronized. Frame_ts: {frame_timestamp:.3f}, "
                    f"GPS_diff: {time_diff_gps:.3f}, Height_diff: {time_diff_height:.3f}, "
                    f"Heading_diff: {time_diff_heading:.3f}, Gimbal_diff: {time_diff_gimbal:.3f}."
                )
                        
                # Process the frame for detection
                processed_frame = frame_for_detection # Already a copy
                
                # Run YOLO detection
                yolo_result = self.detection_model.yolo_detect(processed_frame)
                
                # Unpack detection results
                detection_array = None
                if isinstance(yolo_result, tuple) and len(yolo_result) >= 1:
                    detection_array = yolo_result[0]
                elif isinstance(yolo_result, Detection2DArray):
                    detection_array = yolo_result
                else:
                    self.get_logger().warn(f"Unexpected return type from yolo_detect: {type(yolo_result)}")
                
                # Process detection results
                if detection_array is not None:
                    # Set current state in gps_manager
                    try:
                        self.gps_manager.set_current_state(
                            gps=current_gps_val, # (lat, lon)
                            height=current_height_val, # meters
                            heading=math.radians(current_heading_val), # Convert degrees to radians
                            gimbal_attitude=current_gimbal_attitude_val # Vector3 (radians)
                        )
                        
                        # Process each detection
                        for detection in detection_array.detections:
                            center_x = detection.bbox.center.position.x
                            center_y = detection.bbox.center.position.y

                            bbox_center_input = (center_x, center_y)
                            
                            if 200 <= center_x <= 440 and 150 <= center_y <= 362: # TODO: This bounding box might need re-evaluation based on image size or use case
                                try:
                                    self.get_logger().info(f"Detection bounding box center: x={center_x:.2f}, y={center_y:.2f} (pixel coordinates)")

                                    # Estimate GPS from bounding box
                                    # Assuming calculate_lateral_distance uses the state set by set_current_state
                                    # If it requires explicit arguments, ensure they are correctly formatted (e.g., degrees)
                                    # The original code passed: gimbal_attitude_deg=self.current_gimbal_attitude (Vector3), drone_height_m=self.current_height, drone_heading_deg=self.current_heading
                                    # For consistency, if calculate_lateral_distance needs these, use synchronized values:
                                    gimbal_attitude_for_calc_deg_tuple = (
                                        current_gimbal_attitude_val.x, # roll
                                        current_gimbal_attitude_val.y, # pitch
                                        current_gimbal_attitude_val.z  # yaw
                                    )
                                    result = self.gps_manager.calculate_lateral_distance(
                                        bbox_center=bbox_center_input,
                                        gimbal_attitude_deg=gimbal_attitude_for_calc_deg_tuple, # Pass as tuple of degrees
                                        drone_height_m=current_height_val,
                                        drone_heading_deg=current_heading_val # Already in degrees
                                        )
                                    estimated_lat = result['estimated_latitude']
                                    estimated_lon = result['estimated_longitude']
                                    distance_meters = result['lateral_distance_m']

                                    
                                    # Create and publish GPS message
                                    gps_msg = NavSatFix()
                                    timestamp = self.get_clock().now().to_msg()
                                    gps_msg.header.stamp = timestamp
                                    gps_msg.header.frame_id = "estimated_casualty"
                                    gps_msg.latitude = float(estimated_lat)
                                    gps_msg.longitude = float(estimated_lon)
                                    gps_msg.altitude = distance_meters
                                    
                                    self.gps_target_pub.publish(gps_msg)
                                    self.gps_manager.add_target(float(estimated_lat), float(estimated_lon))
                                    
                                    self.get_logger().info(f"Estimated Target GPS: Lat={estimated_lat:.8f}, Lon={estimated_lon:.8f}")
                                    self.get_logger().info(f"Distance from drone to estimated target: {distance_meters:.2f} meters")
                                    
                                    if distance_meters < 1.0:
                                        self.precise_gps_target_pub.publish(gps_msg)
                                        self.get_logger().info(f"PRECISE TARGET FOUND AT {estimated_lat:.8f}, {estimated_lon:.8f}")
                                except Exception as e:
                                    tb_str = traceback.format_exc()
                                    self.get_logger().error(f"Error in GPS estimation: {e}\n{tb_str}")
                        
                        # Publish detection array
                        if len(detection_array.detections) > 0:
                            self.get_logger().info(f"Detection: {len(detection_array.detections)} detection(s) found.")
                            frame_time_msg = self.get_clock().now().to_msg()
                            detection_array.header.stamp = frame_time_msg
                            self.detection_pub.publish(detection_array)
                            
                            # Publish detection center if in LOCK_ON mode
                            if len(detection_array.detections) > 0 and self.current_gimbal_mode == 'LOCK_ON':
                                detection = detection_array.detections[0]
                                center = Vector3()
                                center.x = detection.bbox.center.position.x
                                center.y = detection.bbox.center.position.y
                                center.z = 0.0
                                self.detection_center_pub.publish(center)
                                self.get_logger().info(f"Published detection center: x={center.x:.2f}, y={center.y:.2f}")
                    except Exception as e:
                        tb_str = traceback.format_exc()
                        self.get_logger().error(f"Error processing detection: {e}\n{tb_str}")
                else:
                    # This else block for the outer if (frame_for_detection and sensor data check) is now handled earlier.
                    # We can log if yolo_result was None or detection_array was None after yolo_detect call if needed.
                    if yolo_result is None or detection_array is None :
                         self.get_logger().debug("No detections from yolo_detect or result was None.")
                
                loop_duration = time.time() - loop_start_time
                rate.sleep()
        except Exception as e:
            tb_str = traceback.format_exc()
            self.get_logger().error(f"Exception in process_detection thread: {e}\nTraceback:\n{tb_str}")

    def _publish_gps_targets_callback(self):
        """Callback function to publish all GPS targets (called by timer)."""
        callback_start_time = time.time() # <<< TIMER START
        # Check gps_manager exists and is initialized
        if not hasattr(self, 'gps_manager') or self.gps_manager is None:
            # It's possible this callback runs before gps_manager is fully initialized in start()
            # Log a warning or debug message instead of erroring out immediately after startup
            self.get_logger().debug('GPS Manager not yet initialized, skipping target publication cycle.')
            return
            
        # Get active clusters of GPS targets
        get_clusters_start = time.time() # <<< TIMER START
        clusters = self.gps_manager.get_active_clusters()
        get_clusters_duration = time.time() - get_clusters_start # <<< TIMER END
        
        if not clusters:
            callback_duration = time.time() - callback_start_time # <<< TIMER END
            self.get_logger().debug(f"_publish_gps_targets_callback: No clusters to publish. Duration={callback_duration:.6f}s")
            return
            
        # Import necessary message type
        from sensor_msgs.msg import NavSatFix
        from std_msgs.msg import Header
        
        # Create and publish a NavSatFix message for each cluster center
        publish_start = time.time() # <<< TIMER START
        published_count = 0
        for cluster in clusters:
            msg = NavSatFix()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = f"casualty_{cluster['id']}"
            
            # Set the GPS coordinates
            lat, lon = cluster["center"]
            msg.latitude = lat
            msg.longitude = lon
            
            # Set some reasonable covariance values (diagonal elements for lat, lon)
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
            msg.position_covariance = [0.0] * 9  # Initialize with zeros
            msg.position_covariance[0] = 1.0  # Latitude variance
            msg.position_covariance[4] = 1.0  # Longitude variance
            
            # Add the number of targets in this cluster as a suffix to the frame_id
            targets_count = len(cluster["targets"])
            msg.header.frame_id += f"_{targets_count}"
            ####
            #### 1. no stop geofence mapping
            #### 2. ray cast gps 
            #### 3. stream lock on
            #### 4. /target_gps_list natsatfix
            #### 5. /robot_1/mavros/global_position/global

            self.gps_targets_list_pub.publish(msg)
            published_count += 1
            
        publish_duration = time.time() - publish_start # <<< TIMER END
        callback_duration = time.time() - callback_start_time # <<< TIMER END
        # self.get_logger().debug(f"_publish_gps_targets_callback: Published {published_count} clusters. Duration={callback_duration:.6f}s (clusters get: {get_clusters_duration:.6f}s, publish: {publish_duration:.6f}s)")

    def camera_mode_callback(self, msg):
        """receive the camera mode ('EO' or 'IR') and update GPSEstimator"""
        callback_start_time = time.time() # <<< TIMER START
        camera_mode = msg.data
        
        # Ensure camera mode is valid
        if camera_mode not in ['EO', 'IR']:
            self.get_logger().warn(f"Received invalid camera mode: {camera_mode}, should be 'EO' or 'IR'")
            return
            
        # If camera mode has changed, update it
        if camera_mode != self.current_camera_mode:
            self.current_camera_mode = camera_mode
            
            try:
                # Update the camera type in GPSEstimator
                set_camera_start = time.time() # <<< TIMER START
                self.gps_manager.set_camera_type(camera_mode)
                set_camera_duration = time.time() - set_camera_start # <<< TIMER END
                self.get_logger().info(f"Camera mode updated to: {camera_mode}")
                self.get_logger().debug(f"camera_mode_callback: set_camera_type took {set_camera_duration:.6f}s")
            except Exception as e:
                self.get_logger().error(f"Error updating camera mode: {str(e)}")
                
        callback_duration = time.time() - callback_start_time # <<< TIMER END
        # self.get_logger().debug(f"camera_mode_callback: Total processing time: {callback_duration:.6f}s")

    # Callback for gimbal mode
    def gimbal_mode_callback(self, msg):
        """Callback function to update the current gimbal mode."""
        callback_start_time = time.time() # <<< TIMER START
        self.current_gimbal_mode = msg.data
        # self.get_logger().debug(f"Received gimbal mode: {self.current_gimbal_mode}")
        callback_duration = time.time() - callback_start_time # <<< TIMER END
        # self.get_logger().debug(f"gimbal_mode_callback: Total processing time: {callback_duration:.6f}s")

    @FunctionProfiler()
    def gimbal_attitude_callback(self, msg):
        """Callback function to update the current gimbal attitude."""
        callback_start_time = time.time()
        # Update the current gimbal attitude
        self.current_gimbal_attitude = msg
        callback_duration = time.time() - callback_start_time
        self.get_logger().debug(f"gimbal_attitude_callback: Total processing time: {callback_duration:.6f}s")

        # Store timestamped reading
        ros_time_sec = self.get_clock().now().nanoseconds / 1e9
        self.gimbal_attitude_readings.append((ros_time_sec, msg))

    def destroy_node(self):
        # stop profiling
        if hasattr(self, 'profiling_enabled') and self.profiling_enabled:
            self.get_logger().info("Stopping performance profiling and generating final report")
            self.profile_manager.stop_profiling()
            final_report_path = self.profile_manager.save_results()
            self.get_logger().info(f"Final profiling report saved to: {final_report_path}")
        
        super().destroy_node()

    # Helper function to find the closest reading from a deque
    def _get_closest_reading(self, readings_deque, target_timestamp):
        if not readings_deque:
            return None
        # Find the reading with the minimum absolute time difference
        # readings_deque contains tuples of (timestamp, data)
        closest_reading = min(readings_deque, key=lambda reading: abs(reading[0] - target_timestamp))
        return closest_reading # Returns (timestamp, data) or None if deque is empty

def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None # Define executor
    try:
        node = IntegratedNode()
        # Use MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin() # Spin the executor
    except KeyboardInterrupt:
        pass
    finally:
        # Ensure video writer is released properly
        if node is not None and hasattr(node, 'video_writer') and node.video_writer is not None:
            node.video_writer.release()
            node.get_logger().info("Video writer released.")
        
        # Shutdown executor if it exists
        if executor is not None:
            executor.shutdown()
            
        # Destroy node explicitly if it exists (good practice)
        if node is not None:
            node.destroy_node()

        # Shutdown rclpy only if it's still running
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main() 
