#!/usr/bin/env python3
"""
Center Pixel GPS Estimation Node - Simplified Version
===================================================

Assumes the target is always located at the center of the image plane, continuously performs GPS estimation and publishes results.
Does not use YOLO detection, directly performs GPS coordinate estimation on the center pixel of the image.

Simplified logic:
- Does not use clustering algorithms
- Always publishes estimated GPS position
- Real-time processing, no caching or historical data management
"""

from pickle import FALSE
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import cv2
import math
import time
import threading
import numpy as np
from collections import deque
from typing import Dict, List, Tuple, Optional, Any
import logging

# ROS2 message types
from sensor_msgs.msg import Image, NavSatFix, CompressedImage, CameraInfo
from geometry_msgs.msg import Point, PoseStamped, Vector3, PointStamped
from std_msgs.msg import Bool, Float32, String, Float64, Header
from cv_bridge import CvBridge

# Local modules
from .gps_manager import GPSManagerFixed

logger = logging.getLogger(__name__)


class CenterPixelGPSNode(Node):
    """Center Pixel GPS Estimation Node - Simplified version, no clustering, always publishes GPS estimates"""
    
    def __init__(self):
        super().__init__('center_pixel_gps_estimator')
        
        # Initialize logging
        self._setup_logging()
        
        # Load parameters
        self._load_parameters()
        
        # Initialize components
        self._initialize_components()
        
        # Setup ROS2 interfaces
        self._setup_subscriptions()
        self._setup_publishers()
        self._setup_timers()
        
        # State management
        self._initialize_state()
        
        # Start processing threads
        self._start_processing_threads()
        
        self.get_logger().info("CenterPixelGPSNode initialization successful")

    def _setup_logging(self):
        """Configure logging"""
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )

    def _load_parameters(self):
        """Load and validate ROS2 parameters"""
        # Declare all parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                # Node identification and diagnostics
                ('node_name', 'center_pixel_gps_estimator'),
                ('publish_diagnostics', False),
                ('diagnostics_period', 10.0),
                
                # Estimation frequency parameters
                ('estimation.frequency', 10.0),  # Hz - GPS estimation frequency
                ('estimation.enable_continuous', False),  # Whether to enable continuous estimation
                
                # Image parameters
                ('image.width', 640),
                ('image.height', 360),
                
                # Synchronization parameters
                ('sync.max_sync_time_diff', 0.5),
                ('sync.buffer_size', 10),
                
                # Output parameters
                ('output.jpeg_quality', 10),
                ('output.publish_local_enu', False),
                ('output.publish_camera_info', False),
                ('output.sigma_lat_lon_m', 5.0),
                ('output.publish_center_marker', False),  # Whether to publish center marker
                
                # Camera parameters
                ('camera.mode', 'EO'),
                ('camera.auto_scale_with_resolution', True),
                ('camera.digital_zoom_factor', 1.0),
                
                # Camera intrinsics - EO
                ('camera_intrinsics.EO.fx', 515.1042901585477),
                ('camera_intrinsics.EO.fy', 515.1042901585475),
                ('camera_intrinsics.EO.cx', 320.0),
                ('camera_intrinsics.EO.cy', 256.0),
                ('camera_intrinsics.EO.distortion', [0.0, 0.0, 0.0, 0.0, 0.0]),
                ('camera_intrinsics.EO.calibration_resolution', [640, 512]),
                
                # Camera intrinsics - IR
                ('camera_intrinsics.IR.fx', 2267.0),
                ('camera_intrinsics.IR.fy', 1593.0),
                ('camera_intrinsics.IR.cx', 640.0),
                ('camera_intrinsics.IR.cy', 360.0),
                ('camera_intrinsics.IR.distortion', [0.0, 0.0, 0.0, 0.0, 0.0]),
                ('camera_intrinsics.IR.calibration_resolution', [1280, 720]),
                
                # GPS estimation - simplified version, no clustering
                ('gps_estimation.simple_mode', True),
                
                # Topic names
                ('topics.mavros_gps', '/dtc_mrsd_/mavros/global_position/global'),
                ('topics.mavros_altitude', '/dtc_mrsd_/mavros/global_position/rel_alt'),
                ('topics.mavros_heading', '/dtc_mrsd_/mavros/global_position/compass_hdg'),
                ('topics.gimbal_attitude', '/gimbal_attitude'),
                ('topics.camera_mode', '/camera_mode'),
                ('topics.image_compressed', '/center_gps/image_compressed'),
                ('topics.camera_info', '/center_gps/camera_info'),
                ('topics.target_gps', '/center_gps/target_gps'),
                ('topics.target_local_enu', '/center_gps/target_local_enu'),
                ('topics.center_marker', '/center_gps/center_marker'),
                
                # QoS settings - sensor data
                ('qos.sensor_data.reliability', 'best_effort'),
                ('qos.sensor_data.durability', 'volatile'),
                ('qos.sensor_data.depth', 1),
                
                # QoS settings - state data
                ('qos.state_data.reliability', 'reliable'),
                ('qos.state_data.durability', 'volatile'),
                ('qos.state_data.depth', 10),
                
                # QoS settings - target data
                ('qos.target_data.reliability', 'reliable'),
                ('qos.target_data.durability', 'volatile'),
                ('qos.target_data.depth', 5),
                
                # Performance monitoring
                ('performance.enable_profiling', False),
                ('performance.profiling_report_interval', 300.0),
                ('performance.log_performance_stats', False),
                ('performance.stats_log_interval', 30.0),
                
                # Logging configuration
                ('logging.level', 'INFO'),
                ('logging.log_to_file', False),
                ('logging.log_file_path', '/tmp/center_pixel_gps.log'),
            ]
        )
        
        # Extract parameter values
        self.config = self._extract_config()
        
        # Configure logging based on parameters
        self._configure_logging()
        
        self.get_logger().info(f"Loaded {self.config['camera']['mode']} camera configuration")

    def _extract_config(self) -> Dict[str, Any]:
        """Extract parameters into organized configuration dictionary"""
        config = {}
        
        # Parameter groups
        param_groups = ['diagnostics', 'estimation', 'image', 'sync', 'output', 
                       'camera', 'gps_estimation', 'topics', 'qos', 'performance', 'logging']
        
        for group in param_groups:
            config[group] = {}
            
        # Add node-level parameters
        config['node_name'] = self.get_parameter('node_name').value
        config['publish_diagnostics'] = self.get_parameter('publish_diagnostics').value
        config['diagnostics_period'] = self.get_parameter('diagnostics_period').value
        
        # Extract parameters
        for param_descriptor in self._parameters.values():
            param_name = param_descriptor.name
            value = self.get_parameter(param_name).value
            
            # Skip already processed node-level parameters
            if param_name in ['node_name', 'publish_diagnostics', 'diagnostics_period']:
                continue
            
            # Group parameters
            if param_name.startswith('estimation.'):
                config['estimation'][param_name[11:]] = value
            elif param_name.startswith('image.'):
                config['image'][param_name[6:]] = value
            elif param_name.startswith('sync.'):
                config['sync'][param_name[5:]] = value
            elif param_name.startswith('output.'):
                config['output'][param_name[7:]] = value
            elif param_name.startswith('camera.') and not param_name.startswith('camera_intrinsics.'):
                config['camera'][param_name[7:]] = value
            elif param_name.startswith('gps_estimation.'):
                config['gps_estimation'][param_name[15:]] = value
            elif param_name.startswith('topics.'):
                config['topics'][param_name[7:]] = value
            elif param_name.startswith('qos.'):
                # Parse QoS hierarchy: qos.sensor_data.reliability
                parts = param_name.split('.')
                if len(parts) >= 3:
                    qos_category = parts[1]  # sensor_data, state_data, target_data
                    qos_param = parts[2]     # reliability, durability, depth
                    if qos_category not in config['qos']:
                        config['qos'][qos_category] = {}
                    config['qos'][qos_category][qos_param] = value
            elif param_name.startswith('performance.'):
                config['performance'][param_name[12:]] = value
            elif param_name.startswith('logging.'):
                config['logging'][param_name[8:]] = value
        
        # Add camera intrinsics
        config['camera_intrinsics'] = {
            'EO': {
                'fx': self.get_parameter('camera_intrinsics.EO.fx').value,
                'fy': self.get_parameter('camera_intrinsics.EO.fy').value,
                'cx': self.get_parameter('camera_intrinsics.EO.cx').value,
                'cy': self.get_parameter('camera_intrinsics.EO.cy').value,
                'distortion': self.get_parameter('camera_intrinsics.EO.distortion').value,
                'calibration_resolution': self.get_parameter('camera_intrinsics.EO.calibration_resolution').value,
            },
            'IR': {
                'fx': self.get_parameter('camera_intrinsics.IR.fx').value,
                'fy': self.get_parameter('camera_intrinsics.IR.fy').value,
                'cx': self.get_parameter('camera_intrinsics.IR.cx').value,
                'cy': self.get_parameter('camera_intrinsics.IR.cy').value,
                'distortion': self.get_parameter('camera_intrinsics.IR.distortion').value,
                'calibration_resolution': self.get_parameter('camera_intrinsics.IR.calibration_resolution').value,
            }
        }
        
        return config

    def _configure_logging(self):
        """Configure logging based on parameter settings"""
        try:
            # Map log level strings to logging constants
            level_map = {
                'DEBUG': logging.DEBUG,
                'INFO': logging.INFO,
                'WARN': logging.WARNING,
                'WARNING': logging.WARNING,
                'ERROR': logging.ERROR,
                'CRITICAL': logging.CRITICAL
            }
            
            log_level = level_map.get(self.config['logging']['level'], logging.INFO)
            
            # Configure root logger
            logger = logging.getLogger(__name__)
            logger.setLevel(log_level)
            
            # Add file handler if needed
            if self.config['logging']['log_to_file']:
                file_handler = logging.FileHandler(self.config['logging']['log_file_path'])
                file_handler.setLevel(log_level)
                formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
                file_handler.setFormatter(formatter)
                logger.addHandler(file_handler)
                
        except Exception as e:
            self.get_logger().warn(f"Failed to configure logging: {e}")

    def _create_qos_profile(self, qos_type: str) -> QoSProfile:
        """Create QoS profile from configuration"""
        qos_config = self.config['qos'].get(qos_type, {})
        
        # Map string values to ROS2 enums
        reliability_map = {
            'reliable': QoSReliabilityPolicy.RELIABLE,
            'best_effort': QoSReliabilityPolicy.BEST_EFFORT
        }
        
        durability_map = {
            'volatile': QoSDurabilityPolicy.VOLATILE,
            'transient_local': QoSDurabilityPolicy.TRANSIENT_LOCAL
        }
        
        return QoSProfile(
            reliability=reliability_map.get(qos_config.get('reliability', 'reliable'), QoSReliabilityPolicy.RELIABLE),
            durability=durability_map.get(qos_config.get('durability', 'volatile'), QoSDurabilityPolicy.VOLATILE),
            depth=qos_config.get('depth', 10)
        )

    def _initialize_components(self):
        """Initialize core processing components"""
        # Initialize GPS manager - simplified version
        gps_config = self.config['gps_estimation']
        self.gps_manager = GPSManagerFixed(config=gps_config)
        
        # Initialize CV bridge
        self.cv_bridge = CvBridge()
        
        # Load camera intrinsics
        self._setup_camera_intrinsics()

    def _setup_camera_intrinsics(self):
        """Setup camera intrinsics based on current mode and resolution"""
        camera_mode = self.config['camera']['mode']
        width = self.config['image']['width']
        height = self.config['image']['height']
        
        # Load intrinsics from config, including digital zoom factor
        digital_zoom = self.config['camera']['digital_zoom_factor']
        self.gps_manager.load_intrinsics_from_config(
            {
                'mode': camera_mode,
                'intrinsics': self.config['camera_intrinsics']
            },
            width, height, digital_zoom
        )

    def _setup_subscriptions(self):
        """Setup ROS2 subscriptions with appropriate QoS settings"""
        # Create QoS profiles from configuration
        sensor_qos = self._create_qos_profile('sensor_data')
        state_qos = self._create_qos_profile('state_data')
        
        # Create callback group for parallel processing
        self.sensor_callback_group = MutuallyExclusiveCallbackGroup()
        
        # MAVROS subscriptions
        self.gps_sub = self.create_subscription(
            NavSatFix,
            self.config['topics']['mavros_gps'],
            self._gps_callback,
            sensor_qos,
            callback_group=self.sensor_callback_group
        )
        
        self.altitude_sub = self.create_subscription(
            Float64,
            self.config['topics']['mavros_altitude'],
            self._altitude_callback,
            sensor_qos,
            callback_group=self.sensor_callback_group
        )
        
        self.heading_sub = self.create_subscription(
            Float64,
            self.config['topics']['mavros_heading'],
            self._heading_callback,
            sensor_qos,
            callback_group=self.sensor_callback_group
        )
        
        # Gimbal and camera subscriptions
        self.gimbal_attitude_sub = self.create_subscription(
            Vector3,
            self.config['topics']['gimbal_attitude'],
            self._gimbal_attitude_callback,
            sensor_qos,
            callback_group=self.sensor_callback_group
        )
        
        self.camera_mode_sub = self.create_subscription(
            String,
            self.config['topics']['camera_mode'],
            self._camera_mode_callback,
            state_qos
        )
        
        # Image subscription - directly subscribe to compressed image topic
        self.image_sub = self.create_subscription(
            CompressedImage,
            '/image_raw_compressed',
            self._image_callback,
            sensor_qos,
            callback_group=self.sensor_callback_group
        )
        
        # Burst mode control subscription
        self.burst_mode_sub = self.create_subscription(
            Bool,
            '/burst_mode/control',
            self._burst_mode_callback,
            state_qos
        )

    def _setup_publishers(self):
        """Setup ROS2 publishers with appropriate QoS settings"""
        # Create QoS profiles from configuration
        target_qos = self._create_qos_profile('target_data')
        
        # Only publish GPS targets
        self.target_gps_pub = self.create_publisher(
            NavSatFix,
            self.config['topics']['target_gps'],
            target_qos
        )

    def _setup_timers(self):
        """Setup periodic timers for maintenance tasks"""
        # Performance statistics timer
        if self.config['performance']['log_performance_stats']:
            self.stats_timer = self.create_timer(
                self.config['performance']['stats_log_interval'],
                self._log_performance_stats
            )
        
        # Note: Removed GPS target list publisher and cluster cleanup timer since clustering is no longer used

    def _initialize_state(self):
        """Initialize node state variables"""
        # Timestamped sensor data buffers
        buffer_size = self.config['sync']['buffer_size']
        self.gps_readings = deque(maxlen=buffer_size)
        self.altitude_readings = deque(maxlen=buffer_size)
        self.heading_readings = deque(maxlen=buffer_size)
        self.gimbal_attitude_readings = deque(maxlen=buffer_size)
        
        # Current state (for compatibility)
        self.current_gps = None
        self.current_altitude = None
        self.current_heading = None
        self.current_gimbal_attitude = Vector3()
        self.current_camera_mode = self.config['camera']['mode']
        
        # Burst mode control
        self.burst_mode_enabled = False
        self.burst_mode_timer = None
        self.burst_mode_duration = 7.0  # 10 seconds
        
        # Frame processing
        self.latest_frame_info = None
        self.frame_lock = threading.Lock()
        self.current_frame = None
        self.current_frame_timestamp = None
        
        # Performance tracking
        self.stats = {
            'frames_processed': 0,
            'gps_estimates': 0,
            'sync_failures': 0,
            'last_frame_time': 0,
            'average_processing_time': 0.0
        }

    def _start_processing_threads(self):
        """Start background processing threads"""
        # Start GPS estimation processing thread
        self.estimation_thread = threading.Thread(
            target=self._estimation_loop,
            daemon=True
        )
        self.estimation_thread.start()

    def _estimation_loop(self):
        """Main GPS estimation loop"""
        target_fps = self.config['estimation']['frequency']
        frame_interval = 1.0 / target_fps
        last_estimation_time = 0
        
        while rclpy.ok():
            try:
                current_time = time.time()
                
                # Frequency limiting
                if current_time - last_estimation_time < frame_interval:
                    time.sleep(0.01)
                    continue
                
                # Get latest frame
                frame = None
                frame_timestamp = None
                with self.frame_lock:
                    if self.current_frame is not None:
                        frame = self.current_frame.copy()
                        frame_timestamp = self.current_frame_timestamp
                
                if frame is None:
                    continue
                
                # Check if burst mode is enabled
                if not self.burst_mode_enabled:
                    time.sleep(0.1)  # Sleep longer when not in burst mode
                    continue
                
                # Find synchronized sensor readings
                sync_data = self._get_synchronized_sensor_data(frame_timestamp)
                
                last_estimation_time = current_time
                processing_start = time.time()
                
                # Process center pixel GPS estimation
                frame_info = {
                    'frame': frame,
                    'timestamp': frame_timestamp,
                    'sync_data': sync_data
                }
                self._process_center_pixel_estimation(frame_info)
                
                # Update performance statistics
                processing_time = time.time() - processing_start
                self._update_processing_stats(processing_time)
                
            except Exception as e:
                self.get_logger().error(f"Estimation loop error: {e}")
                time.sleep(1.0)

    def _get_synchronized_sensor_data(self, frame_timestamp: float) -> Optional[Dict[str, Any]]:
        """Get sensor data synchronized with frame timestamp"""
        max_time_diff = self.config['sync']['max_sync_time_diff']
        
        # Find closest readings
        gps_reading = self._find_closest_reading(self.gps_readings, frame_timestamp)
        altitude_reading = self._find_closest_reading(self.altitude_readings, frame_timestamp)
        heading_reading = self._find_closest_reading(self.heading_readings, frame_timestamp)
        gimbal_reading = self._find_closest_reading(self.gimbal_attitude_readings, frame_timestamp)
        
        # Check synchronization quality
        if not all([gps_reading, altitude_reading, heading_reading, gimbal_reading]):
            # Log missing sensor data for debugging
            missing_sensors = []
            if not gps_reading:
                missing_sensors.append("GPS")
            if not altitude_reading:
                missing_sensors.append("altitude")
            if not heading_reading:
                missing_sensors.append("heading")
            if not gimbal_reading:
                missing_sensors.append("gimbal_attitude")
            
            self.get_logger().info(f"Missing sensor data: {', '.join(missing_sensors)}")
            return None
        
        # Check time differences
        time_diffs = [
            abs(frame_timestamp - gps_reading[0]),
            abs(frame_timestamp - altitude_reading[0]),
            abs(frame_timestamp - heading_reading[0]),
            abs(frame_timestamp - gimbal_reading[0])
        ]
        
        if max(time_diffs) > max_time_diff:
            self.stats['sync_failures'] += 1
            return None
        
        return {
            'gps': gps_reading[1],
            'altitude': altitude_reading[1],
            'heading': heading_reading[1],
            'gimbal_attitude': gimbal_reading[1],
            'max_time_diff': max(time_diffs)
        }

    def _find_closest_reading(self, readings_deque: deque, target_timestamp: float) -> Optional[Tuple[float, Any]]:
        """Find sensor reading closest to target timestamp"""
        if not readings_deque:
            return None
        
        return min(readings_deque, key=lambda x: abs(x[0] - target_timestamp))

    def _process_center_pixel_estimation(self, frame_info: Dict[str, Any]):
        """Process center pixel GPS estimation"""
        frame_timestamp = frame_info['timestamp']
        sync_data = frame_info['sync_data']
        frame = frame_info['frame']
        
        if sync_data is None:
            return
        
        # Calculate image center coordinates
        height, width = frame.shape[:2]
        center_x = width / 2.0
        center_y = height / 2.0
        
        # Set GPS manager state
        gps_lat, gps_lon = sync_data['gps']
        heading_rad = math.radians(sync_data['heading'])  # Convert degrees to radians
        gimbal_vec = sync_data['gimbal_attitude']
        gimbal_attitude = (gimbal_vec.x, gimbal_vec.y, gimbal_vec.z)  # Already in radians
        
        try:
            self.gps_manager.set_current_state(
                gps=(gps_lat, gps_lon),
                height=sync_data['altitude'],
                heading=heading_rad,
                gimbal_attitude=gimbal_attitude
            )
            
            # Perform GPS estimation on center pixel
            gps_result = self.gps_manager.estimate_target_gps(center_x, center_y)
            
            # Create GPS message
            gps_msg = self._create_gps_message(gps_result, frame_timestamp)
            
            # Publish estimated GPS position
            self.target_gps_pub.publish(gps_msg)
            
            self.stats['gps_estimates'] += 1
            
            self.get_logger().debug(
                f"Center pixel GPS estimate: ({gps_result['estimated_latitude']:.6f}, "
                f"{gps_result['estimated_longitude']:.6f}), "
                f"distance: {gps_result['lateral_distance_m']:.2f}m"
            )
            
        except Exception as e:
            self.get_logger().warning(f"Center pixel GPS estimation failed: {e}")

    def _create_gps_message(self, gps_result: Dict[str, Any], timestamp: float) -> NavSatFix:
        """Create ROS2 NavSatFix message from GPS estimation result"""
        msg = NavSatFix()
        msg.header.stamp = self._to_ros_time(timestamp)
        msg.header.frame_id = "estimated_center_target"
        
        msg.latitude = gps_result['estimated_latitude']
        msg.longitude = gps_result['estimated_longitude']
        msg.altitude = 0.0  # Not estimated
        
        # Set status to indicate this is an estimate, not real GNSS
        msg.status.status = -1  # Custom status for estimate
        msg.status.service = 0
        
        # Set covariance to indicate uncertainty
        sigma = self.config['output']['sigma_lat_lon_m']
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        msg.position_covariance = [0.0] * 9
        msg.position_covariance[0] = sigma ** 2  # Latitude variance
        msg.position_covariance[4] = sigma ** 2  # Longitude variance
        msg.position_covariance[8] = 1000.0      # Large altitude variance
        
        return msg



    def _to_ros_time(self, timestamp: float):
        """Convert timestamp to ROS2 time message"""
        sec = int(timestamp)
        nanosec = int((timestamp - sec) * 1e9)
        
        time_msg = self.get_clock().now().to_msg()
        time_msg.sec = sec
        time_msg.nanosec = nanosec
        
        return time_msg

    # Sensor callbacks
    def _gps_callback(self, msg: NavSatFix):
        """Process GPS message"""
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.gps_readings.append((timestamp, (msg.latitude, msg.longitude)))
        self.current_gps = (msg.latitude, msg.longitude)

    def _altitude_callback(self, msg: Float64):
        """Process altitude message"""
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.altitude_readings.append((timestamp, msg.data))
        self.current_altitude = msg.data

    def _heading_callback(self, msg: Float64):
        """Process heading message"""
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.heading_readings.append((timestamp, msg.data))
        self.current_heading = msg.data

    def _gimbal_attitude_callback(self, msg: Vector3):
        """Process gimbal attitude message"""
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.gimbal_attitude_readings.append((timestamp, msg))
        self.current_gimbal_attitude = msg

    def _camera_mode_callback(self, msg: String):
        """Process camera mode change"""
        new_mode = msg.data.upper()
        if new_mode != self.current_camera_mode and new_mode in ['EO', 'IR']:
            self.get_logger().info(f"Camera mode switched from {self.current_camera_mode} to {new_mode}")
            self.current_camera_mode = new_mode
            
            # Update GPS manager intrinsics
            try:
                self.gps_manager.set_camera_type(new_mode)
                self._setup_camera_intrinsics()  # Reload appropriate scaling
            except Exception as e:
                self.get_logger().error(f"Failed to update camera mode: {e}")

    def _image_callback(self, msg: CompressedImage):
        """Process incoming compressed image message"""
        try:
            # Convert compressed image to OpenCV format
            np_arr = np.frombuffer(msg.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if frame is None:
                self.get_logger().warning("Failed to decode compressed image")
                return
            
            # Store frame with timestamp for processing
            timestamp = self.get_clock().now().nanoseconds / 1e9
            
            # Add to frame buffer for processing
            with self.frame_lock:
                self.current_frame = frame
                self.current_frame_timestamp = timestamp
                
            # Update statistics
            self.stats['frames_processed'] += 1
            
        except Exception as e:
            self.get_logger().error(f"Image processing error: {e}")

    def _burst_mode_callback(self, msg: Bool):
        """Process burst mode control message"""
        previous_state = self.burst_mode_enabled
        self.burst_mode_enabled = msg.data
        
        if previous_state != self.burst_mode_enabled:
            if self.burst_mode_enabled:
                self.get_logger().info("burst mode enabled - start GPS estimation and publishing for 10 seconds")
                # Cancel any existing timer
                if self.burst_mode_timer is not None:
                    self.burst_mode_timer.cancel()
                # Start new timer for 10 seconds
                self.burst_mode_timer = self.create_timer(
                    self.burst_mode_duration,
                    self._reset_burst_mode
                )
            else:
                self.get_logger().info("burst mode disabled - stop GPS estimation and publishing")
                # Cancel timer if burst mode is manually disabled
                if self.burst_mode_timer is not None:
                    self.burst_mode_timer.cancel()
                    self.burst_mode_timer = None

    def _reset_burst_mode(self):
        """Reset burst mode after timeout"""
        self.burst_mode_enabled = False
        if self.burst_mode_timer is not None:
            self.burst_mode_timer.cancel()
            self.burst_mode_timer = None
        self.get_logger().info("burst mode auto-reset after 10 seconds - GPS estimation stopped")

    # Performance monitoring
    def _update_processing_stats(self, processing_time: float):
        """Update processing time statistics"""
        n = self.stats['gps_estimates']
        if n == 0:
            self.stats['average_processing_time'] = processing_time
        else:
            old_avg = self.stats['average_processing_time']
            self.stats['average_processing_time'] = (old_avg * (n - 1) + processing_time) / n

    def _log_performance_stats(self):
        """Log performance statistics"""
        stats = self.stats.copy()
        
        # Add component statistics
        gps_stats = self.gps_manager.get_stats()
        
        # Add sensor data statistics
        sensor_stats = f"Sensor buffers - GPS: {len(self.gps_readings)}, altitude: {len(self.altitude_readings)}, heading: {len(self.heading_readings)}, gimbal: {len(self.gimbal_attitude_readings)}"
        
        self.get_logger().info(
            f"Performance stats - "
            f"frames: {stats['frames_processed']}, "
            f"GPS estimates: {stats['gps_estimates']}, "
            f"sync failures: {stats['sync_failures']}, "
            f"avg processing time: {stats['average_processing_time']:.3f}s, "
            f"GPS success rate: {gps_stats.get('success_rate', 0):.2f}"
        )
        self.get_logger().info(sensor_stats)


    def destroy_node(self):
        """Clean shutdown of the node"""
        self.get_logger().info("Shutting down CenterPixelGPSNode...")
        
        # Cancel burst mode timer if active
        if self.burst_mode_timer is not None:
            self.burst_mode_timer.cancel()
            self.burst_mode_timer = None
            
        super().destroy_node()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        node = CenterPixelGPSNode()
        
        # Use MultiThreadedExecutor for parallel callback processing
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()
            node.destroy_node()
            
    except Exception as e:
        print(f"Failed to start node: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
