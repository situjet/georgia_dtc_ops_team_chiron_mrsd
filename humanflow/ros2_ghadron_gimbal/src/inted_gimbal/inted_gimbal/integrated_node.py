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

# import performance analysis module
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

        # initialize performance analysis manager
        self.profile_manager = ProfileManager(output_dir='/home/dtc/humanflow/ros2_ghadron_gimbal/profiling_results')
        self.function_profiler = self.profile_manager.function_profiler
        
        # add performance analysis parameter
        self.declare_parameter('enable_profiling', True, 
            ParameterDescriptor(type=ParameterType.PARAMETER_BOOL, 
                                description='Enable performance profiling'))
        self.profiling_enabled = self.get_parameter('enable_profiling').value
        
        # add performance analysis report frequency parameter
        self.declare_parameter('profiling_report_interval', 300.0, 
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE, 
                                description='Interval (seconds) between profiling reports'))
        self.profiling_report_interval = self.get_parameter('profiling_report_interval').value
        
        # if performance profiling is enabled, start profiling
        if self.profiling_enabled:
            self.get_logger().info("Starting performance profiling")
            self.profile_manager.start_profiling()
            # create a timer to generate performance reports periodically
            self.profiling_timer = self.create_timer(self.profiling_report_interval, self._generate_profiling_report)

        # QoS profile for image transport - use BEST_EFFORT for lower latency
        image_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST
        )

        # Use standard ROS2 compressed image topics instead of image_transport
        # This is the raw image publisher (usually not used with high latency links)
        # self.image_pub = self.create_publisher(
        #     Image, 'image_raw', 
        #     qos_profile=image_qos
        # )
        
        # This is the compressed image publisher - most subscribers will use this
        self.compressed_image_pub = self.create_publisher(
            CompressedImage, '/image_raw/compressed', 
            qos_profile=image_qos
        )

        self.detection_pub = self.create_publisher(Detection2DArray, '/detection', 10)
        # add a new publisher to publish the detection center coordinates
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
        self.declare_parameter('jpeg_quality', 30, 
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
        self.declare_parameter('max_sync_time_diff', 1.5, # Default to 1.5 seconds (Increased from 0.5)
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
        self.detection_fps = 3  # Target FPS for detection
        self.last_detection_time = 0  # Timestamp of last detection
        
        # GPSEstimator will be initialized in start() after getting image dimensions
        self.gps_manager = None
        self.image_center = None

        # Add buffer to store timestamped sensor data
        # Parameter for sensor buffer size
        self.declare_parameter('sensor_buffer_size', 200,
            ParameterDescriptor(type=ParameterType.PARAMETER_INTEGER,
                              description='Maximum number of sensor readings to buffer'))
        self.sensor_buffer_size = self.get_parameter('sensor_buffer_size').value
        self.max_timestamp_diff = 0.1  # Maximum timestamp difference (seconds) - warn if exceeded
        
        # Unified sensor data buffer, each element is a tuple: (timestamp, sensor_type, data)
        # sensor_type can be 'gps', 'height', 'heading', etc.
        # data format depends on sensor_type: (lat, lon) for 'gps', float for 'height'/'heading'
        self.sensor_buffer = deque(maxlen=self.sensor_buffer_size)

        # Sensor data lock to prevent buffer modification during search
        self.sensor_buffer_lock = threading.Lock()

        # Add variable to record the GPS estimation method used
        self.last_gps_estimation_method = "unknown"

        # buffer to store the latest 6 frames
        self.frame_buffer = deque(maxlen=12)  # 2 fps per second, 3 seconds
        self.frame_buffer_lock = threading.Lock()
        
        # declare parameter to control the frame buffer size
        self.declare_parameter('frame_buffer_duration', 3.0,
            ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE,
                                description='duration (seconds) to store frames in the buffer'))
        self.frame_buffer_duration = self.get_parameter('frame_buffer_duration').value

        self.start()

    #add performance report generation function
    def _generate_profiling_report(self):
        """generate and save performance analysis report"""
        if not self.profiling_enabled:
            return
            
        try:
            report_path = self.profile_manager.save_results()
            self.get_logger().info(f"Generated profiling report: {report_path}")
            
            # print key performance metrics to log
            res_stats = self.profile_manager.resource_monitor.get_stats()
            self.get_logger().info(f"performance metrics summary - CPU: {res_stats['cpu']['avg']:.2f}% (max: {res_stats['cpu']['max']:.2f}%), "
                                 f"memory: {res_stats['memory']['avg']:.2f}MB (max: {res_stats['memory']['max']:.2f}MB), "
                                 f"thread count: {res_stats['threads']['latest']}")
        except Exception as e:
            self.get_logger().error(f"Error generating profiling report: {e}")
            traceback.print_exc()

    # use decorator to profile key functions
    @property
    def profile(self):
        return self.function_profiler

    # Navigation data callbacks
    @FunctionProfiler()
    def gps_callback(self, msg):
        callback_start_time = time.time() # <<< TIMER START
        # Use time.time() as the current timestamp
        current_timestamp = time.time()
        self.latest_gps_timestamp = current_timestamp  # record the timestamp of the latest GPS data
        
        # Store timestamped GPS data in the unified sensor buffer
        buffer_lock_start = time.time() # <<< TIMER START
        with self.sensor_buffer_lock:
            # Add current GPS data to buffer
            gps_data = (msg.latitude, msg.longitude)
            self.sensor_buffer.append((current_timestamp, 'gps', gps_data))
            # Optionally update current value for immediate access if needed elsewhere
            self.current_gps = gps_data
        buffer_lock_duration = time.time() - buffer_lock_start # <<< TIMER END
        
        callback_duration = time.time() - callback_start_time # <<< TIMER END
        self.get_logger().debug(f"gps_callback: Total processing time: {callback_duration:.6f}s (buffer lock: {buffer_lock_duration:.6f}s)")

    @FunctionProfiler()
    def height_callback(self, msg):
        callback_start_time = time.time() # <<< TIMER START
        current_timestamp = time.time()
        self.latest_height_timestamp = current_timestamp  # record the timestamp of the latest height data
        
        # Store timestamped Height data in the unified sensor buffer
        buffer_lock_start = time.time() # <<< TIMER START
        with self.sensor_buffer_lock:
            height_data = msg.data
            self.sensor_buffer.append((current_timestamp, 'height', height_data))
            # Optionally update current value
            self.current_height = height_data
        buffer_lock_duration = time.time() - buffer_lock_start # <<< TIMER END
        
        callback_duration = time.time() - callback_start_time # <<< TIMER END
        self.get_logger().debug(f"height_callback: Total processing time: {callback_duration:.6f}s (buffer lock: {buffer_lock_duration:.6f}s)")

    @FunctionProfiler()
    def heading_callback(self, msg):
        """Callback for heading data."""
        callback_start_time = time.time() # <<< TIMER START
        current_timestamp = time.time()
        self.latest_heading_timestamp = current_timestamp  # record the timestamp of the latest heading data
        
        # Store timestamped Heading data in the unified sensor buffer
        buffer_lock_start = time.time() # <<< TIMER START
        with self.sensor_buffer_lock:
            heading_data = msg.data
            self.sensor_buffer.append((current_timestamp, 'heading', heading_data))
            # Optionally update current value
            self.current_heading = heading_data
        buffer_lock_duration = time.time() - buffer_lock_start # <<< TIMER END
        
        callback_duration = time.time() - callback_start_time # <<< TIMER END
        self.get_logger().debug(f"heading_callback: Total processing time: {callback_duration:.6f}s (buffer lock: {buffer_lock_duration:.6f}s)")

    # return the closest data, timestamp, time difference, and whether the data is ahead of the target timestamp
    def _find_closest_data(self, buffer_copy, target_timestamp, sensor_type):
        """Find the data of a specific sensor_type in buffer_copy closest to the target timestamp"""
        func_start_time = time.time() # <<< TIMER START
        closest_data = None
        closest_timestamp = None
        min_diff = float('inf')
        found = False
        is_ahead = None  #
        search_iterations = 0 # <<< COUNTER
        sensor_entries = 0 # record the number of entries for the specific type of sensor
        iteration_time = 0 # iteration time

        # calculate the buffer size
        buffer_size = len(buffer_copy)

        # Iterate backwards over the provided buffer copy
        iterate_start = time.time()
        for ts, stype, data in reversed(buffer_copy):
            search_iterations += 1 # <<< COUNTER
            iter_start = time.time()
            
            if stype == sensor_type:
                sensor_entries += 1
                diff = abs(ts - target_timestamp)
                if diff < min_diff:
                    min_diff = diff
                    closest_data = data
                    closest_timestamp = ts
                    is_ahead = (ts > target_timestamp)  # determine if the sensor timestamp is ahead of the target timestamp
                    found = True
                elif found and ts < target_timestamp - min_diff:
                     break
            
            iter_time = time.time() - iter_start
            iteration_time += iter_time

        iterate_time = time.time() - iterate_start
        func_duration = time.time() - func_start_time # <<< TIMER END
        
        # only record detailed time for debug level
        if self.get_logger().get_effective_level() <= 10:  # DEBUG=10
            self.get_logger().debug(f"sensor query({sensor_type}, ts={target_timestamp:.4f}): found={found}, min diff={min_diff:.4f}s, ahead_behind={'ahead' if is_ahead else 'behind'} frame, iterations={search_iterations}/{buffer_size}, sensor entries={sensor_entries}, duration={func_duration:.6f}s (iteration time={iterate_time:.6f}s)")

        if not found:
             self.get_logger().debug(f"sensor query({sensor_type}): no data found near timestamp {target_timestamp:.4f}")
             return None, None, None, None  # return None tuple, add is_ahead

        # Warn if the time difference is large, but still return the found data
        if min_diff > self.max_timestamp_diff:
            ahead_behind = "ahead" if is_ahead else "behind"
            self.get_logger().warn(f"sensor query({sensor_type}): large time difference ({min_diff:.4f}s > {self.max_timestamp_diff}s), target timestamp={target_timestamp:.4f}, closest:{closest_timestamp:.4f}({ahead_behind} the frame). still use this data.")

        return closest_data, closest_timestamp, min_diff, is_ahead  # return the sensor data whether it is ahead of the frame timestamp

    # Updated get_current_gps_data method, using buffer to find closest sensor data
    def find_closest_gps_data(self, timestamp=None):
        """Find the GPS data closest to the given timestamp using the sensor buffer."""
        if timestamp is None:
            timestamp = time.time() # Default to now if no timestamp provided
        with self.sensor_buffer_lock:
            return self._find_closest_data(self.sensor_buffer, timestamp, 'gps')

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
        max_attempts = 5
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
            self.image_center = (640, 360)  # Default for 1280x720
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
        # self.get_logger().info("process_stream thread entered.") # <<< ADDED PRINT
        # Target FPS for retrieving frames
        streaming_fps = 5 # Set to retrieve frames at a rate of 1 frame per second
        frame_time = 1.0 / streaming_fps

        # Use rclpy.Rate for rate control
        rate = self.create_rate(streaming_fps, self.get_clock())

        # record performance data
        frame_count = 0
        total_loop_time = 0
        max_loop_time = 0
        
        while rclpy.ok():
            loop_start_time = time.time() # <<< TIMER START

            # Get the latest image frame and timestamp from the streaming module
            result = None # Initialize result to None
            try:
                # <<< ADDED LOG >>>
                # self.get_logger().info("process_stream: Attempting retrieve_image...")
                # retrieve_start_time = time.time() # <<< TIMER START
                result = self.streaming_module.retrieve_image()
                # retrieve_duration = time.time() - retrieve_start_time # <<< TIMER END
                # self.get_logger().info(f"process_stream: retrieve_image took {retrieve_duration:.6f} seconds.")
                # <<< ADDED LOG >>>
                self.get_logger().info(f"Retrieve Image Timestamp: {result[1]:.6f}")
                if result is not None:
                    # self.get_logger().info("process_stream: retrieve_image returned successfully.")
                    pass
                else:
                    # self.get_logger().warn("process_stream: retrieve_image returned None.") # Changed to WARN
                    pass
            except Exception as e:
                # <<< ADDED EXCEPTION LOGGING >>>
                self.get_logger().error(f"Exception during retrieve_image: {type(e).__name__} - {e}", exc_info=True)
                # Optionally add a small sleep here if retrieve_image fails rapidly
                time.sleep(1.0)
                rate.sleep() # Still sleep according to rate even if there's an error
                continue # Skip the rest of the loop iteration on error

            if result is not None:
                frame, timestamp = result
                # <<< START ADDED LOGGING >>>
                if frame is None:
                    # self.get_logger().warn("process_stream: Frame extracted from result is None!")
                    pass
                else:
                    # self.get_logger().info(f"process_stream: Frame extracted successfully, shape={frame.shape}, ts={timestamp:.4f}")
                # <<< END ADDED LOGGING >>>
                    pass

                if frame is not None:
                    # publish the frame to the compressed image topic for debugging
                    publish_start_time = time.time() # <<< TIMER START
                    try:
                        # convert the image to compressed format
                        compressed_img = CompressedImage()
                        compressed_img.header.stamp = self.get_clock().now().to_msg()
                        compressed_img.format = "jpeg"
                        compress_start_time = time.time() # <<< TIMER START
                        compressed_img.data = np.array(cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])[1]).tobytes()
                        compress_duration = time.time() - compress_start_time # <<< TIMER END
                        self.get_logger().debug(f"process_stream: JPEG compression took {compress_duration:.8f} seconds.") 
                        # normally 0.001s
                        # publish the compressed image
                        self.compressed_image_pub.publish(compressed_img)
                        publish_duration = time.time() - publish_start_time # <<< TIMER END
                        # self.get_logger().info(f"Published frame to compressed image topic for debugging")
                        # self.get_logger().info(f"process_stream: Publishing compressed image took {publish_duration:.6f} seconds (incl. compression).")
                    except Exception as e:
                        self.get_logger().error(f"Error publishing compressed image: {str(e)}")

                    # add the new frame to the buffer
                    buffer_add_start_time = time.time() # <<< TIMER START
                    with self.frame_buffer_lock:
                        self.frame_buffer.append({
                            'frame': frame.copy(),
                            'timestamp': timestamp,
                            'processed': False  # mark this frame as not processed
                        })
                    # buffer_add_duration = time.time() - buffer_add_start_time # <<< TIMER END
                    # self.get_logger().debug(f"process_stream: Adding frame to buffer took {buffer_add_duration:.8f} seconds.")

                    current_time = time.time()
                    # cutoff_time = current_time - self.frame_buffer_duration

                    # find the closest GPS, height and heading data
                    # gps_data, gps_ts, gps_diff, _ = self._find_closest_data(self.sensor_buffer, timestamp, 'gps')
                    # height_data, height_ts, height_diff, _ = self._find_closest_data(self.sensor_buffer, timestamp, 'height')
                    # heading_data, heading_ts, heading_diff, _ = self._find_closest_data(self.sensor_buffer, timestamp, 'heading')

                    # only consider this frame when all data is available
                    # if gps_data is None or height_data is None or heading_data is None:
                    #     continue

                    #
                    # total_diff = gps_diff + height_diff + heading_diff

                    #
                    # if total_diff < min_total_diff:
                    #     min_total_diff = total_diff
                    #     best_frame_info = {
                    #         'frame': frame.copy(),
                    #         'gps_data': gps_data,
                    #         'height_data': height_data,
                    #         'heading_data': heading_data,
                    #         'exact_timestamp': timestamp,
                    #         'total_time_diff': total_diff
                    #     }
                    #     # 标记帧为已处理
                    #     frame_info['processed'] = True

                    # # 继续发布当前帧用于显示
                    # # ... existing compressed image publishing code ...

            # <<< ADDED ELSE for result is None >>>
            else:
                 self.get_logger().warn("process_stream: retrieve_image returned None, skipping frame processing.")

            # loop_duration = time.time() - loop_start_time # <<< TIMER END
            # self.get_logger().info(f"process_stream: Full loop iteration took {loop_duration:.6f} seconds.")
            
            # update performance metrics
            loop_duration = time.time() - loop_start_time
            frame_count += 1
            total_loop_time += loop_duration
            max_loop_time = max(max_loop_time, loop_duration)
            
            if frame_count % 100 == 0:
                self.get_logger().info(f"stream processing performance: average loop time={total_loop_time/frame_count:.6f} seconds, max loop time={max_loop_time:.6f} seconds, processed frames={frame_count}")
            
            rate.sleep()

    @FunctionProfiler()
    def process_detection(self):
        # self.get_logger().info("process_detection thread entered.") # <<< ADDED PRINT
        try: # Add try block for better error reporting if something fails early
            # Target detection rate (per second)
            detection_rate = 7.0  # Adjust as needed
            detection_time = 1.0 / detection_rate

            # Initialize detection_array to None
            detection_array = None

            # Use rclpy.Rate for rate control
            # self.get_logger().info("process_detection: Creating rate...") # <<< ADDED PRINT
            rate = self.create_rate(detection_rate, self.get_clock())
            # self.get_logger().info("process_detection: Rate created.") # <<< ADDED PRINT

            while rclpy.ok():
                loop_start_time = time.time() # <<< TIMER START
                # Modify to find best frame first
                find_best_start_time = time.time() # <<< TIMER START
                # self.get_logger().info(f"finding best frame...")
                # best_match = self._find_best_frame_for_sensor_data()
                find_best_duration = time.time() - find_best_start_time # <<< TIMER END
                # self.get_logger().info(f"find best frame took {find_best_duration:.6f} seconds, result: {'success' if best_match else 'not found'}")
                # best_match = True
                # If best match is found, process it

                # <<< TEMP DEBUG: Get latest frame directly >>>
                best_match = None
                with self.frame_buffer_lock:
                    if self.frame_buffer:
                        latest_frame_info = self.frame_buffer[-1] # Get the last item
                        # create a matching structure containing the latest sensor data
                        best_match = {
                            'frame': latest_frame_info['frame'].copy(),
                            'gps_data': self.current_gps, # use the latest GPS data
                            'height_data': self.current_height, # use the latest height data
                            'heading_data': self.current_heading, # use the latest heading data
                            'exact_timestamp': latest_frame_info['timestamp'],
                            'total_time_diff': 0.0 # placeholder value
                        }
                        detection_frame_timestamp = time.time()
                        self.get_logger().info(f"Detection Frame Time Diff: {detection_frame_timestamp-latest_frame_info['timestamp']:.6f}")
                        # check the difference between the sensor data and the frame timestamp
                        frame_timestamp = latest_frame_info['timestamp']
                        if hasattr(self, 'latest_gps_timestamp') and self.latest_gps_timestamp:
                            gps_diff = abs(self.latest_gps_timestamp - frame_timestamp)
                            if gps_diff > 0.1:
                                self.get_logger().info(f"GPS data timestamp difference: {gps_diff:.3f} seconds")
                        
                        if hasattr(self, 'latest_height_timestamp') and self.latest_height_timestamp:
                            height_diff = abs(self.latest_height_timestamp - frame_timestamp)
                            if height_diff > 0.1:
                                self.get_logger().info(f"height data timestamp difference: {height_diff:.3f} seconds")
                        
                        if hasattr(self, 'latest_heading_timestamp') and self.latest_heading_timestamp:
                            heading_diff = abs(self.latest_heading_timestamp - frame_timestamp)
                            if heading_diff > 0.1:
                                self.get_logger().info(f"heading data timestamp difference: {heading_diff:.3f} seconds")
                        # self.get_logger().info(f"Processing best match frame, total time difference: {best_match['total_time_diff']:.6f} seconds")
                    else:
                        self.get_logger().debug("TEMP DEBUG: Frame buffer empty, cannot get latest frame.")
                # <<< END TEMP DEBUG >>>

                if best_match:
                    # Log the match quality
                    # self.get_logger().info(f"Processing best match frame, total time difference: {best_match['total_time_diff']:.6f} seconds")

                    # Process detection using existing code, but directly use best_match instead of re-retrieving
                    frame_for_detection = best_match['frame']
                    captured_data = best_match

                    # Proceed only if we have a valid frame
                    if frame_for_detection is not None and captured_data is not None:
                        # Explicitly apply preprocessing to reduce overexposure before detection
                        preprocess_start_time = time.time() # <<< TIMER START
                        processed_frame = frame_for_detection#self.detection_model.preprocess_image(frame_for_detection)

                        # Resize frame to width 640, maintaining aspect ratio

                        original_height, original_width = processed_frame.shape[:2]
                        target_width = 640
                        target_height = 512
                        dim = (target_width, target_height)

                        # Log before resizing (changed message slightly for clarity)
                        # self.get_logger().info(f"process_detection: Preparing to resize frame from {original_width}x{original_height} to {target_width}x{target_height}") # <<< REVISED PRINT
                        # processed_frame = cv2.resize(processed_frame, dim, interpolation = cv2.INTER_AREA) # <<< SUSPECTED HANG POINT
                        # Log immediately after resizing
                        # self.get_logger().info("process_detection: Frame resize complete.") # <<< ADDED PRINT
                        # self.get_logger().debug(f"Resized frame dimensions: {processed_frame.shape}") # <<< ADDED PRINT
                        # preprocess_duration = time.time() - preprocess_start_time # <<< TIMER END
                        # self.get_logger().info(f"process_detection: Preprocessing (resize) took {preprocess_duration:.6f} seconds.")


                        # Run YOLO detection - Assume it returns (detections, poses)
                        # detection_array, pose_array = self.detection_model.yolo_detect(processed_frame)
                        # self.get_logger().info("process_detection: Calling yolo_detect...") # <<< ADDED PRINT
                        # yolo_start_time = time.time() # <<< TIMER START
                        yolo_result = self.detection_model.yolo_detect(processed_frame)
                        # yolo_duration = time.time() - yolo_start_time # <<< TIMER END
                        # self.get_logger().info(f"process_detection: yolo_detect took {yolo_duration:.6f} seconds.")
                        # self.get_logger().info("process_detection: yolo_detect returned.") # <<< ADDED PRINT

                        # Unpack carefully, handling potential None or non-tuple results
                        detection_array = None
                        # pose_array = None # Keep pose_array for potential future use
                        if isinstance(yolo_result, tuple) and len(yolo_result) >= 1:
                            detection_array = yolo_result[0]
                            if len(yolo_result) > 1:
                                pose_array = yolo_result[1]
                        elif isinstance(yolo_result, Detection2DArray): # Handle case where it only returns detections
                            detection_array = yolo_result
                        else:
                            self.get_logger().warn(f"Unexpected return type from yolo_detect: {type(yolo_result)}")

                        # Process detection results and determine if bounding box center is in target area
                        if detection_array is not None:
                            # Get GPS data corresponding to the frame
                            gps_info = best_match['gps_data']
                            height_data = best_match['height_data']
                            heading_data = best_match['heading_data']
                            # timestamp = best_match['exact_timestamp']
                            # <<< START ADDED LOGGING >>>
                            # self.get_logger().info(f"process_detection: 1")
                            # self.get_logger().info(f"process_detection: Extracted data for frame ts={timestamp:.4f}: gps={gps_info}, height={height_data}, heading={heading_data}")
                            # <<< END ADDED LOGGING >>>

                            # Check if required data is available for GPS estimation
                            if gps_info is not None and height_data is not None and heading_data is not None:
                                current_lat, current_lon = gps_info
                                current_alt = height_data
                                # Convert heading from degrees to radians
                                current_heading_rad = math.radians(heading_data)
                                # self.get_logger().info(f"process_detection: 2")
                                # Set the current state in gps_manager
                                try:
                                    self.gps_manager.set_current_state(
                                        gps=(current_lat, current_lon),
                                        height=current_alt,
                                        heading=current_heading_rad
                                    )
                                    state_set = True
                                except Exception as e:
                                    self.get_logger().error(f"Error setting gps_manager state: {e}")
                                    state_set = False

                                # Check if center point is within specified region (480-800, 270-450)
                                # window at 10m height
                                # Width 4.0m, height 3.93m
                                # 640 by 512
                                # 230-410, 260-460
                                # if 230 <= center_x <= 410 and 260 <= center_y <= 460:
                                #     # Create GPS message
                                #     gps_msg = NavSatFix()
                                #     gps_msg.header.stamp = self.get_clock().now().to_msg()
                                #     gps_msg.header.frame_id = "casulty"
                                #     # Ensure latitude and longitude are float type
                                #     gps_msg.latitude = float(gps_data[0])  # latitude
                                #     gps_msg.longitude = float(gps_data[1])  # longitude
                                #     gps_msg.altitude = 0.0  # assume target is on ground

                                #     # Publish GPS message
                                #     self.gps_target_pub.publish(gps_msg)
                                #     self.get_logger().info(f"Target in Center of Image: Publishing GPS at {gps_data[0]:.8f}, {gps_data[1]:.8f}")

                                #     #add to gps manager
                                #     self.gps_manager.add_target(float(gps_data[0]), float(gps_data[1]))

                                #     # for precise search
                                #     # 100 x 100 pixels
                                #     # width 2.11m, height 2.10m
                                #     # if 220 <= center_x <= 420 and 206 <= center_y <= 306:
                                #     #     self.precise_gps_target_pub.publish(gps_msg)
                                #     #     self.get_logger().info(f"PRECISE TARGET FOUND AT {gps_data[0]:.8f}, {gps_data[1]:.8f}")
                                if state_set:
                                    # Iterate through all detection results
                                    estimation_total_duration = 0.0 # <<< TIMER INIT
                                    estimation_count = 0 # <<< COUNTER INIT
                                    for detection in detection_array.detections:
                                        # Get bounding box center coordinates and size

                                        center_x = detection.bbox.center.position.x
                                        center_y = detection.bbox.center.position.y
                                        width = detection.bbox.size_x
                                        height = detection.bbox.size_y

                                        # Calculate bbox (x1, y1, x2, y2)
                                        x1 = center_x - width / 2
                                        y1 = center_y - height / 2
                                        x2 = center_x + width / 2
                                        y2 = center_y + height / 2
                                        bbox = (x1, y1, x2, y2)
                                        # 230-410, 260-460

                                        if 10 <= center_x <= 630 and 15 <= center_y <= 500:
                                            try:
                                                # Estimate GPS from bounding box
                                                # self.get_logger().info(f"process_detection: GPS estimation start")
                                                # Store the result in a single variable first
                                                estimation_start_time = time.time() # <<< TIMER START
                                                estimated_coords = self.gps_manager.estimate_gps_from_bbox(bbox)
                                                estimation_duration = time.time() - estimation_start_time # <<< TIMER END
                                                estimation_total_duration += estimation_duration # <<< TIMER ACCUMULATE
                                                estimation_count += 1 # <<< COUNTER INCREMENT

                                                if estimated_coords is not None:
                                                    # Unpack only if the result is not None
                                                    estimated_lat, estimated_lon = estimated_coords

                                                    # --- Calculate and log distance ---
                                                    distance_meters = calculate_haversine_distance(
                                                        current_lat, current_lon,
                                                        estimated_lat, estimated_lon
                                                    )
                                                    # --- End of distance calculation ---

                                                    # Create GPS message for the estimated target location
                                                    gps_msg = NavSatFix()
                                                    timestamp = self.get_clock().now().to_msg()
                                                    gps_msg.header.stamp = timestamp
                                                    # Use a distinct frame_id for estimated targets
                                                    gps_msg.header.frame_id = "estimated_casualty"
                                                    gps_msg.latitude = float(estimated_lat)
                                                    gps_msg.longitude = float(estimated_lon)
                                                    # Set altitude if available, otherwise 0.0 or drone's altitude
                                                    gps_msg.altitude = distance_meters # Storing distance here, as per your code

                                                    # Publish estimated GPS message
                                                    self.gps_target_pub.publish(gps_msg)

                                                    # Add the *estimated* target GPS to the manager for clustering
                                                    self.gps_manager.add_target(float(estimated_lat), float(estimated_lon))
                                                    self.get_logger().info(f"Estimated Target GPS: Lat={estimated_lat:.8f}, Lon={estimated_lon:.8f}, Timestamp={(timestamp.sec + timestamp.nanosec * 1e-9):.6f}")
                                                    self.get_logger().info(f"Distance from drone to estimated target: {distance_meters:.2f} meters")

                                                    # Optional: Refine logic for precise target publishing
                                                    if distance_meters < 1.5:
                                                        self.precise_gps_target_pub.publish(gps_msg)
                                                        self.get_logger().info(f"PRECISE TARGET FOUND AT {estimated_lat:.8f}, {estimated_lon:.8f}")

                                                else:
                                                    # Log that estimation failed for this bbox
                                                    self.get_logger().warn(f"estimate_gps_from_bbox returned None for bbox {bbox}. Skipping GPS estimation for this detection.")

                                            except ValueError as e:
                                                # Log the specific ValueError message
                                                self.get_logger().warn(f"Could not estimate GPS for bbox {bbox}: {e}")
                                            except Exception as e:
                                                # Log the exception type and message, and include traceback
                                                tb_str = traceback.format_exc() # Format the traceback string
                                                self.get_logger().error(f"Unexpected error during GPS estimation: {type(e).__name__} - {e}\nTraceback:\n{tb_str}")
                                        else:
                                            self.get_logger().info("Detection Not Close Enough")
                                    # Log total estimation time
                                    if estimation_count > 0:
                                        avg_estimation_time = estimation_total_duration / estimation_count
                                        self.get_logger().info(f"process_detection: GPS estimation for {estimation_count} detections took {estimation_total_duration:.6f} seconds (avg {avg_estimation_time:.6f}s per detection).")
                                    else:
                                        self.get_logger().debug("process_detection: No valid detections for GPS estimation in this cycle.")
                        else:
                            # Extracted data for logging missing sensor parts
                            gps_info = best_match.get('gps_data')
                            height_data = best_match.get('height_data')
                            heading_data = best_match.get('heading_data')
                            timestamp = best_match.get('exact_timestamp', 'N/A') # Get timestamp or default

                            missing_data = []
                            if gps_info is None: missing_data.append("GPS")
                            if height_data is None: missing_data.append("Height")
                            if heading_data is None: missing_data.append("Heading")
                            # Log timestamp even if data is missing
                            ts_str = f"{timestamp:.4f}" if isinstance(timestamp, float) else str(timestamp)
                            self.get_logger().warn(f"Skipping GPS estimation due to missing sensor data: {', '.join(missing_data)} for frame timestamp {ts_str}")

                else:
                    self.get_logger().warn("No best match found for sensor data")

                # Process and publish detection array (Moved logging inside the condition)
                # Ensure detection_array is not None before checking its length
                detection_publish_start_time = time.time() # <<< TIMER START
                detections_published = 0
                if detection_array is not None and len(detection_array.detections) > 0:
                    self.get_logger().info(f"Detection: {len(detection_array.detections)} detection(s) found.")
                    # update the last detection timestamp
                    self.last_detection_time = time.time()

                    # Add timestamp for synchronization
                    frame_time_msg = self.get_clock().now().to_msg()
                    detection_array.header.stamp = frame_time_msg
                    self.detection_pub.publish(detection_array)
                    detections_published = len(detection_array.detections)

                    # extract the center coordinates of the first detection and publish
                    if len(detection_array.detections) > 0:
                        detection = detection_array.detections[0]
                        center = Vector3()
                        center.x = detection.bbox.center.position.x
                        center.y = detection.bbox.center.position.y
                        center.z = 0.0  # optional: use z to store confidence

                        # Publish detection center only if gimbal mode is LOCK_ON
                        if self.current_gimbal_mode == 'LOCK_ON':
                            self.detection_center_pub.publish(center)
                            self.get_logger().info(f"Published detection center (LOCK_ON): x={center.x:.2f}, y={center.y:.2f}")
                        else:
                             self.get_logger().debug(f"Gimbal mode is not LOCK_ON ({self.current_gimbal_mode}), skipping detection center publish.")

                detection_publish_duration = time.time() - detection_publish_start_time # <<< TIMER END
                if detections_published > 0:
                    self.get_logger().debug(f"process_detection: Publishing {detections_published} detections took {detection_publish_duration:.6f} seconds.")
                # Wait for next detection cycle
                # elapsed = time.time() - start_time # No longer needed for sleep calculation
                # sleep_time = max(0, detection_time - elapsed) # No longer needed
                # self.get_logger().info("process_detection: Loop end, calling rate.sleep().") # <<< ADDED PRINT
                loop_duration = time.time() - loop_start_time # <<< TIMER END
                # self.get_logger().info(f" Full Detection loop iteration took {loop_duration:.6f} seconds.")
                rate.sleep()
                # self.get_logger().info("process_detection: rate.sleep() returned.") # <<< ADDED PRINT

        except Exception as e:
            # Log any exception that occurs within the thread, including traceback
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

    #   
    def _find_best_frame_for_sensor_data(self):
        func_start_time = time.time() # <<< TIMER START
        best_frame_info = None
        min_total_diff = float('inf')
        frames_checked = 0 # <<< COUNTER
        
        # time statistics for each step
        lock_time = 0
        sensor_search_time = 0
        frame_copy_time = 0
        sensor_searches = 0

        with self.frame_buffer_lock, self.sensor_buffer_lock:
            lock_start = time.time()
            # create a buffer copy to avoid being modified during processing
            frame_buffer_copy = list(self.frame_buffer)
            sensor_buffer_copy = list(self.sensor_buffer)
            lock_time = time.time() - lock_start

            # <<< ADDED LOGGING >>>
            self.get_logger().info(f"frame matching: frame buffer={len(frame_buffer_copy)} frames, sensor buffer={len(sensor_buffer_copy)} entries, lock time={lock_time:.6f}s")
            # <<< END ADDED LOGGING >>>

            # if the buffer is empty, return None
            if not frame_buffer_copy or not sensor_buffer_copy:
                func_duration = time.time() - func_start_time # <<< TIMER END
                self.get_logger().info(f"frame matching: buffer is empty, return None. duration={func_duration:.6f}s")
                return None

            # iterate through each frame, find the best match
            for frame_info in frame_buffer_copy:
                frames_checked += 1 # <<< COUNTER
                if frame_info['processed']:
                    continue  # skip the processed frame

                frame_timestamp = frame_info['timestamp']
                
                # record the time for finding the sensor data
                search_start = time.time()
                # find the closest GPS, height and heading data
                gps_data, gps_ts, gps_diff, _ = self._find_closest_data(sensor_buffer_copy, frame_timestamp, 'gps')
                height_data, height_ts, height_diff, _ = self._find_closest_data(sensor_buffer_copy, frame_timestamp, 'height')
                heading_data, heading_ts, heading_diff, _ = self._find_closest_data(sensor_buffer_copy, frame_timestamp, 'heading')
                search_time = time.time() - search_start
                sensor_search_time += search_time
                sensor_searches += 3

                # only consider this frame when all data is available
                if gps_data is None or height_data is None or heading_data is None:
                    # <<< ADDED LOGGING >>>
                    missing = []
                    if gps_data is None: missing.append('gps')
                    if height_data is None: missing.append('height')
                    if heading_data is None: missing.append('heading')
                    self.get_logger().debug(f"frame matching: frame {frame_timestamp:.4f} skipped: missing sensor data [{', '.join(missing)}]")
                    # <<< END ADDED LOGGING >>>
                    continue

                # calculate the total time difference as a match quality metric
                total_diff = gps_diff + height_diff + heading_diff

                # <<< ADDED LOGGING >>>
                self.get_logger().debug(f"frame matching: frame {frame_timestamp:.4f} found all sensors, total time difference={total_diff:.4f}s (GPS:{gps_diff:.4f}s, height:{height_diff:.4f}s, heading:{heading_diff:.4f}s)")
                # <<< END ADDED LOGGING >>>

                # <<< ADDED CHECK >>>
                # Check if the total time difference exceeds the acceptable threshold
                if total_diff > self.max_sync_time_diff:
                    self.get_logger().debug(f"frame matching: frame {frame_timestamp:.4f} skipped: total sensor time difference {total_diff:.4f}s > threshold {self.max_sync_time_diff:.4f}s")
                    continue # Skip this frame, its sensor data is too far off
                # <<< END ADDED CHECK >>>

                # if this is the best match so far, save it
                if total_diff < min_total_diff:
                    min_total_diff = total_diff
                    copy_start = time.time()
                    best_frame_info = {
                        'frame': frame_info['frame'].copy(),
                        'gps_data': gps_data,
                        'height_data': height_data,
                        'heading_data': heading_data,
                        'exact_timestamp': frame_timestamp,
                        'total_time_diff': total_diff
                    }
                    frame_copy_time += time.time() - copy_start
                    
                    # mark the frame as processed
                    frame_info['processed'] = True

        # if a match is found, update the latest processed frame information
        update_start = time.time()
        update_time = 0
        if best_frame_info:
            with self.frame_lock:
                self.latest_processed_frame_info = best_frame_info
            update_time = time.time() - update_start
        # <<< ADDED LOGGING >>>
        else:
            update_time = time.time() - update_start
            self.get_logger().warn("frame matching: no best match found in this cycle.") # Changed log level and wording
        # <<< END ADDED LOGGING >>>

        func_duration = time.time() - func_start_time # <<< TIMER END
        found_str = "found" if best_frame_info else "not found"
        self.get_logger().info(f"frame matching: {found_str} best frame. checked frames:{frames_checked}, sensor searches:{sensor_searches} times, duration={func_duration:.6f}s (lock get:{lock_time:.6f}s, sensor search:{sensor_search_time:.6f}s, frame copy:{frame_copy_time:.6f}s, update latest frame:{update_time:.6f}s)")

        return best_frame_info

    def destroy_node(self):
        # stop performance profiling and generate final report
        if hasattr(self, 'profiling_enabled') and self.profiling_enabled:
            self.get_logger().info("Stopping performance profiling and generating final report")
            self.profile_manager.stop_profiling()
            final_report_path = self.profile_manager.save_results()
            self.get_logger().info(f"Final profiling report saved to: {final_report_path}")
        
        super().destroy_node()

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
# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import CompressedImage
# from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
# import cv2
# import threading
# import time
# import numpy as np
# import struct
# from .streaming import Streaming

# class StreamNode(Node):
#     def __init__(self):
#         super().__init__('stream_node')
        
#         # QoS profile for image transport - use BEST_EFFORT for lower latency
#         image_qos = QoSProfile(
#             depth=1,
#             reliability=QoSReliabilityPolicy.BEST_EFFORT,
#             history=QoSHistoryPolicy.KEEP_LAST
#         )
        
#         # Compressed image publisher
#         self.compressed_image_pub = self.create_publisher(
#             CompressedImage, '/image_raw/compressed', 
#             qos_profile=image_qos
#         )
        
#         # Initialize streaming module
#         self.streaming_module = Streaming()
        
#         # Add compression quality parameter
#         self.declare_parameter('jpeg_quality', 5)
#         self.jpeg_quality = self.get_parameter('jpeg_quality').value
        
#         # Statistics tracking
#         self.frame_count = 0
#         self.start_time = time.time()
#         self.last_stats_time = self.start_time
        
#         # Unique ID counter for images
#         self.image_id = 0
        
#         # Start streaming
#         self.start()
        
#     def start(self):
#         # Start the streaming module
#         self.streaming_module.start_pipeline()
        
#         # Give streaming some time to initialize
#         time.sleep(1.0)
        
#         # Start the streaming thread
#         self.stream_thread = threading.Thread(target=self.process_stream)
#         self.stream_thread.daemon = True
#         self.stream_thread.start()
        
#         self.get_logger().info("Stream node started successfully")
        
#     def process_stream(self):
#         # Target FPS for streaming
#         streaming_fps = 5
        
#         # Use rclpy.Rate for rate control
#         rate = self.create_rate(streaming_fps, self.get_clock())
        
#         while rclpy.ok():
#             # Get the latest image frame and timestamp
#             result = self.streaming_module.retrieve_image()
            
#             if result is not None:
#                 frame, timestamp = result
                
#                 if frame is not None:
#                     # Convert to compressed image and publish
#                     try:
#                         # Generate a unique ID for this image
#                         self.image_id += 1
#                         current_image_id = self.image_id
                        
#                         compressed_img = CompressedImage()
#                         # Use both ROS timestamp and our own timestamp for comparison
#                         now = self.get_clock().now()
#                         compressed_img.header.stamp = now.to_msg()
#                         # Add the unique ID to the frame_id field
#                         compressed_img.header.frame_id = f"frame_{current_image_id}"
#                         compressed_img.format = "jpeg"
                        
#                         # Embed the source timestamp and image ID in the first part of the image data
#                         # Format: [SYNC(4bytes)][timestamp(8bytes)][image_id(4bytes)][image_data]
#                         compressed_img.data = np.array(cv2.imencode(
#                             '.jpg', 
#                             frame, 
#                             [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
#                         )[1]).tobytes()
#                         # Publish the compressed image
#                         self.compressed_image_pub.publish(compressed_img)
                        
#                         # Print the unique ID after publishing
#                         self.get_logger().info(f"Published image with ID: {current_image_id}, timestamp: {timestamp:.6f}")
                        
                            
#                     except Exception as e:
#                         self.get_logger().error(f"Error publishing compressed image: {str(e)}")
            
#             # Sleep to maintain the desired publishing rate
#             rate.sleep()

# def main(args=None):
#     rclpy.init(args=args)
#     node = StreamNode()
    
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()