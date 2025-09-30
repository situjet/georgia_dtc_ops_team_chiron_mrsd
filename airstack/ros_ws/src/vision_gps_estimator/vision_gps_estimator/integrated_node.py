#!/usr/bin/env python3
"""
Enhanced Integrated Node for AirStack Vision GPS Estimation
==========================================================

Refactored from original humanflow integrated_node.py with:
- Clean ROS2 node architecture
- Unified timestamp management
- Enhanced error handling and diagnostics
- Streamlined topic management
- Performance monitoring
"""

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
from vision_msgs.msg import Detection2D, Detection2DArray
from geometry_msgs.msg import Point, PoseStamped, Vector3, PointStamped
from std_msgs.msg import Bool, Float32, String, Float64, Header
from cv_bridge import CvBridge

# Local modules
from .yolo_detector import YOLODetector, DetectionResult
from .gps_manager import GPSManagerFixed

logger = logging.getLogger(__name__)


class VisionGPSEstimatorNode(Node):
    """Enhanced integrated node for vision-based GPS target estimation."""
    
    def __init__(self):
        super().__init__('vision_gps_estimator')
        
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
        
        self.get_logger().info("VisionGPSEstimatorNode initialized successfully")

    def _setup_logging(self):
        """Configure logging for the node."""
        # Basic setup first - will be updated after parameters are loaded
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
        )

    def _load_parameters(self):
        """Load and validate ROS2 parameters."""
        # Declare all parameters with defaults matching params.yaml
        self.declare_parameters(
            namespace='',
            parameters=[
                # Node identification and diagnostics
                ('node_name', 'vision_gps_estimator'),
                ('publish_diagnostics', True),
                ('diagnostics_period', 10.0),
                
                # Streaming parameters
                ('streaming.rtsp_url', 'rtsp://10.3.1.124:8556/ghadron'),
                ('streaming.width', 640),
                ('streaming.height', 512),
                ('streaming.fps', 5),
                ('streaming.retry_max', 10),
                ('streaming.retry_delay', 2.0),
                ('streaming.buffer_size', 10),
                
                # Detector parameters
                ('detector.model_path', '/root/ros_ws/src/vision_gps_estimator/yolo12s.pt'),
                ('detector.conf_threshold', 0.5),
                ('detector.iou_threshold', 0.45),
                ('detector.max_detections', 10),
                ('detector.input_size', 640),
                ('detector.target_classes', [0]),
                ('detector.enable_preprocessing', False),
                
                # Synchronization parameters
                ('sync.max_sync_time_diff', 0.1),
                ('sync.buffer_size', 20),
                
                # Output parameters
                ('output.jpeg_quality', 10),
                ('output.publish_local_enu', False),
                ('output.publish_camera_info', True),
                ('output.sigma_lat_lon_m', 5.0),
                
                # Target selection
                ('target_selection.method', 'score'),
                ('target_selection.center_bias_weight', 0.3),
                
                # Camera parameters
                ('camera.mode', 'EO'),
                ('camera.auto_scale_with_resolution', True),
                
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
                
                # GPS estimation
                ('gps_estimation.min_distance_threshold', 5.0),
                ('gps_estimation.max_cluster_age_hours', 24.0),
                ('gps_estimation.cluster_buffer_size', 30),
                
                # Topic names
                ('topics.mavros_gps', '/dtc_mrsd/mavros/global_position/global'),
                ('topics.mavros_altitude', '/dtc_mrsd/mavros/global_position/rel_alt'),
                ('topics.mavros_heading', '/dtc_mrsd/mavros/global_position/compass_hdg'),
                ('topics.gimbal_attitude', '/gimbal_attitude'),
                ('topics.camera_mode', '/camera_mode'),
                ('topics.image_compressed', '/vision_gps/image_compressed'),
                ('topics.camera_info', '/vision_gps/camera_info'),
                ('topics.detections', '/vision_gps/detections'),
                ('topics.target_gps', '/vision_gps/target_gps'),
                ('topics.target_gps_list', '/vision_gps/target_gps_list'),
                ('topics.target_local_enu', '/vision_gps/target_local_enu'),
                
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
                ('performance.log_performance_stats', True),
                ('performance.stats_log_interval', 30.0),
                
                # Logging configuration
                ('logging.level', 'INFO'),
                ('logging.log_to_file', False),
                ('logging.log_file_path', '/tmp/vision_gps_estimator.log'),
            ]
        )
        
        # Extract parameter values
        self.config = self._extract_config()
        
        # Configure logging based on parameters
        self._configure_logging()
        
        self.get_logger().info(f"Loaded configuration for {self.config['camera']['mode']} camera")

    def _extract_config(self) -> Dict[str, Any]:
        """Extract parameters into organized configuration dictionary."""
        config = {}
        
        # Group parameters by category
        param_groups = ['diagnostics', 'streaming', 'detector', 'sync', 'output', 'target_selection', 
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
            
            # Skip node-level params already handled
            if param_name in ['node_name', 'publish_diagnostics', 'diagnostics_period']:
                continue
            
            # Group parameters
            if param_name.startswith('streaming.'):
                config['streaming'][param_name[10:]] = value
            elif param_name.startswith('detector.'):
                config['detector'][param_name[9:]] = value
            elif param_name.startswith('sync.'):
                config['sync'][param_name[5:]] = value
            elif param_name.startswith('output.'):
                config['output'][param_name[7:]] = value
            elif param_name.startswith('target_selection.'):
                config['target_selection'][param_name[17:]] = value
            elif param_name.startswith('camera.') and not param_name.startswith('camera.intrinsics.'):
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
        
        # Add camera intrinsics with full parameters
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
        """Configure logging based on parameter settings."""
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
            
            # Add file handler if requested
            if self.config['logging']['log_to_file']:
                file_handler = logging.FileHandler(self.config['logging']['log_file_path'])
                file_handler.setLevel(log_level)
                formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
                file_handler.setFormatter(formatter)
                logger.addHandler(file_handler)
                
        except Exception as e:
            self.get_logger().warn(f"Failed to configure logging: {e}")

    def _create_qos_profile(self, qos_type: str) -> QoSProfile:
        """Create QoS profile from configuration."""
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
        """Initialize core processing components."""
        # Initialize YOLO detector
        self.yolo_detector = YOLODetector(self.config['detector'])
        
        # Initialize GPS manager
        gps_config = self.config['gps_estimation']
        self.gps_manager = GPSManagerFixed(
            config=gps_config,
            buffer_size=gps_config['cluster_buffer_size']
        )
        
        # Initialize CV bridge
        self.cv_bridge = CvBridge()
        
        # Load camera intrinsics
        self._setup_camera_intrinsics()

    def _setup_camera_intrinsics(self):
        """Setup camera intrinsics based on current mode and resolution."""
        camera_mode = self.config['camera']['mode']
        width = self.config['streaming']['width']
        height = self.config['streaming']['height']
        
        # Load intrinsics from configuration
        self.gps_manager.load_intrinsics_from_config(
            {
                'mode': camera_mode,
                'intrinsics': self.config['camera_intrinsics']
            },
            width, height
        )

    def _setup_subscriptions(self):
        """Setup ROS2 subscriptions with appropriate QoS."""
        # Create QoS profiles from configuration
        sensor_qos = self._create_qos_profile('sensor_data')
        state_qos = self._create_qos_profile('state_data')
        
        # Create callback groups for parallel processing
        self.sensor_callback_group = MutuallyExclusiveCallbackGroup()
        
        # MAVROS subscriptions
        self.gps_sub = self.create_subscription(
            NavSatFix,
            self.config['topics']['mavros_gps'],
            self._gps_callback,
            state_qos,
            callback_group=self.sensor_callback_group
        )
        
        self.altitude_sub = self.create_subscription(
            Float64,
            self.config['topics']['mavros_altitude'],
            self._altitude_callback,
            state_qos,
            callback_group=self.sensor_callback_group
        )
        
        self.heading_sub = self.create_subscription(
            Float64,
            self.config['topics']['mavros_heading'],
            self._heading_callback,
            state_qos,
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

    def _setup_publishers(self):
        """Setup ROS2 publishers with appropriate QoS."""
        # Create QoS profiles from configuration
        sensor_qos = self._create_qos_profile('sensor_data')
        target_qos = self._create_qos_profile('target_data')
        
        # Image publishers
        self.compressed_image_pub = self.create_publisher(
            CompressedImage,
            self.config['topics']['image_compressed'],
            sensor_qos
        )
        
        if self.config['output']['publish_camera_info']:
            self.camera_info_pub = self.create_publisher(
                CameraInfo,
                self.config['topics']['camera_info'],
                sensor_qos
            )
        
        # Detection publisher
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            self.config['topics']['detections'],
            target_qos
        )
        
        # GPS target publishers
        self.target_gps_pub = self.create_publisher(
            NavSatFix,
            self.config['topics']['target_gps'],
            target_qos
        )
        
        self.target_gps_list_pub = self.create_publisher(
            NavSatFix,
            self.config['topics']['target_gps_list'],
            target_qos
        )
        
        # Optional ENU point publisher
        if self.config['output']['publish_local_enu']:
            self.target_enu_pub = self.create_publisher(
                PointStamped,
                self.config['topics']['target_local_enu'],
                target_qos
            )

    def _setup_timers(self):
        """Setup periodic timers for maintenance tasks."""
        # Performance statistics timer
        if self.config['performance']['log_performance_stats']:
            self.stats_timer = self.create_timer(
                self.config['performance']['stats_log_interval'],
                self._log_performance_stats
            )
        
        # GPS targets list publisher timer
        self.gps_list_timer = self.create_timer(1.0, self._publish_gps_targets_list)
        
        # Cluster cleanup timer
        self.cleanup_timer = self.create_timer(300.0, self._cleanup_old_clusters)  # Every 5 minutes

    def _initialize_state(self):
        """Initialize node state variables."""
        # Sensor data buffers with timestamps
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
        
        # Frame processing
        self.latest_frame_info = None
        self.frame_lock = threading.Lock()
        self.current_frame = None
        self.current_frame_timestamp = None
        
        # Performance tracking
        self.stats = {
            'frames_processed': 0,
            'detections_made': 0,
            'gps_estimates': 0,
            'sync_failures': 0,
            'last_frame_time': 0,
            'average_processing_time': 0.0
        }

    def _start_processing_threads(self):
        """Start background processing threads."""
        # Load YOLO model
        if not self.yolo_detector.load_model():
            self.get_logger().error("Failed to load YOLO model")
            return
        
        # Start detection processing thread
        self.detection_thread = threading.Thread(
            target=self._detection_loop,
            daemon=True
        )
        self.detection_thread.start()

    def _detection_loop(self):
        """Main detection loop for YOLO inference and GPS estimation."""
        target_fps = 7  # Detection rate
        frame_interval = 1.0 / target_fps
        last_detection_time = 0
        
        while rclpy.ok():
            try:
                current_time = time.time()
                
                # Rate limiting
                if current_time - last_detection_time < frame_interval:
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
                
                # Find synchronized sensor readings
                sync_data = self._get_synchronized_sensor_data(frame_timestamp)
                
                last_detection_time = current_time
                processing_start = time.time()
                
                # Run YOLO detection
                detection_result = self.yolo_detector.infer(frame)
                if detection_result is None or len(detection_result.boxes) == 0:
                    continue
                
                # Process detections
                frame_info = {
                    'frame': frame,
                    'timestamp': frame_timestamp,
                    'sync_data': sync_data
                }
                self._process_detections(detection_result, frame_info)
                
                # Publish compressed image
                self._publish_compressed_image(frame, frame_timestamp)
                
                # Publish camera info
                if self.config['output']['publish_camera_info']:
                    self._publish_camera_info(frame_timestamp)
                
                # Update performance stats
                processing_time = time.time() - processing_start
                self._update_processing_stats(processing_time)
                
            except Exception as e:
                self.get_logger().error(f"Error in detection loop: {e}")
                time.sleep(1.0)

    def _get_synchronized_sensor_data(self, frame_timestamp: float) -> Optional[Dict[str, Any]]:
        """Get sensor data synchronized to frame timestamp."""
        max_time_diff = self.config['sync']['max_sync_time_diff']
        
        # Find closest readings
        gps_reading = self._find_closest_reading(self.gps_readings, frame_timestamp)
        altitude_reading = self._find_closest_reading(self.altitude_readings, frame_timestamp)
        heading_reading = self._find_closest_reading(self.heading_readings, frame_timestamp)
        gimbal_reading = self._find_closest_reading(self.gimbal_attitude_readings, frame_timestamp)
        
        # Check synchronization quality
        if not all([gps_reading, altitude_reading, heading_reading, gimbal_reading]):
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
        """Find closest sensor reading to target timestamp."""
        if not readings_deque:
            return None
        
        return min(readings_deque, key=lambda x: abs(x[0] - target_timestamp))

    def _process_detections(self, detection_result: DetectionResult, frame_info: Dict[str, Any]):
        """Process YOLO detections and estimate GPS coordinates."""
        frame_timestamp = frame_info['timestamp']
        sync_data = frame_info['sync_data']
        
        if sync_data is None:
            return
        
        # Publish detection array
        detection_msg = self._create_detection_message(detection_result, frame_timestamp)
        self.detection_pub.publish(detection_msg)
        
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
            
            # Process each detection
            primary_target = None
            primary_distance = float('inf')
            
            for i, (center_x, center_y) in enumerate(detection_result.centers):
                try:
                    # Estimate GPS coordinates
                    gps_result = self.gps_manager.estimate_target_gps(center_x, center_y)
                    
                    # Create GPS message
                    gps_msg = self._create_gps_message(gps_result, frame_timestamp)
                    
                    # Select primary target (closest or highest confidence)
                    if self._is_primary_target(detection_result, i, gps_result, primary_distance):
                        primary_target = gps_msg
                        primary_distance = gps_result['lateral_distance_m']
                    
                    # Add to cluster management
                    self.gps_manager.add_target(
                        gps_result['estimated_latitude'],
                        gps_result['estimated_longitude']
                    )
                    
                    # Publish ENU point if enabled
                    if self.config['output']['publish_local_enu']:
                        self._publish_enu_point(gps_result, frame_timestamp)
                    
                    self.stats['gps_estimates'] += 1
                    
                except Exception as e:
                    self.get_logger().warning(f"GPS estimation failed for detection {i}: {e}")
            
            # Publish primary target
            if primary_target is not None:
                self.target_gps_pub.publish(primary_target)
            
            self.stats['detections_made'] += 1
            
        except Exception as e:
            self.get_logger().error(f"Error processing detections: {e}")

    def _is_primary_target(self, detection_result: DetectionResult, index: int, 
                          gps_result: Dict[str, Any], current_best_distance: float) -> bool:
        """Determine if this detection should be the primary target."""
        method = self.config['target_selection']['method']
        
        if method == 'score':
            # Highest confidence score
            return index == 0  # Already sorted by confidence
        elif method == 'score_center':
            # Weighted combination of score and center proximity
            score = detection_result.scores[index]
            
            # Calculate distance from image center
            img_center_x = detection_result.image_shape[1] / 2
            img_center_y = detection_result.image_shape[0] / 2
            center_x, center_y = detection_result.centers[index]
            
            center_distance = np.sqrt(
                (center_x - img_center_x)**2 + (center_y - img_center_y)**2
            )
            
            # Normalize center distance (0-1, where 0 is center)
            max_center_distance = np.sqrt(img_center_x**2 + img_center_y**2)
            normalized_center_distance = center_distance / max_center_distance
            
            # Combined score (higher is better)
            bias_weight = self.config['target_selection']['center_bias_weight']
            combined_score = score - (bias_weight * normalized_center_distance)
            
            # For now, use simple distance-based selection
            return gps_result['lateral_distance_m'] < current_best_distance
        else:
            # Default to closest target
            return gps_result['lateral_distance_m'] < current_best_distance

    def _create_detection_message(self, detection_result: DetectionResult, 
                                timestamp: float) -> Detection2DArray:
        """Create ROS2 Detection2DArray message from YOLO results."""
        msg = Detection2DArray()
        msg.header.stamp = self._to_ros_time(timestamp)
        msg.header.frame_id = "camera_optical_frame"
        
        for i in range(len(detection_result.boxes)):
            detection = Detection2D()
            
            # Bounding box
            x1, y1, x2, y2 = detection_result.boxes[i]
            center_x, center_y = detection_result.centers[i]
            
            detection.bbox.center.position.x = float(center_x)
            detection.bbox.center.position.y = float(center_y)
            detection.bbox.size_x = float(x2 - x1)
            detection.bbox.size_y = float(y2 - y1)
            detection.bbox.center.theta = float(detection_result.scores[i])
            
            # Detection ID
            detection.id = str(int(detection_result.classes[i]))
            
            msg.detections.append(detection)
        
        return msg

    def _create_gps_message(self, gps_result: Dict[str, Any], timestamp: float) -> NavSatFix:
        """Create ROS2 NavSatFix message from GPS estimation result."""
        msg = NavSatFix()
        msg.header.stamp = self._to_ros_time(timestamp)
        msg.header.frame_id = "estimated_target"
        
        msg.latitude = gps_result['estimated_latitude']
        msg.longitude = gps_result['estimated_longitude']
        msg.altitude = 0.0  # Not estimated
        
        # Set status to indicate this is an estimate, not real GNSS
        msg.status.status = -1  # Custom status for estimation
        msg.status.service = 0
        
        # Set covariance to indicate uncertainty
        sigma = self.config['output']['sigma_lat_lon_m']
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        msg.position_covariance = [0.0] * 9
        msg.position_covariance[0] = sigma ** 2  # Latitude variance
        msg.position_covariance[4] = sigma ** 2  # Longitude variance
        msg.position_covariance[8] = 1000.0      # Large altitude variance
        
        return msg

    def _publish_compressed_image(self, frame: np.ndarray, timestamp: float):
        """Publish compressed image."""
        try:
            compressed_msg = CompressedImage()
            compressed_msg.header.stamp = self._to_ros_time(timestamp)
            compressed_msg.header.frame_id = "camera_optical_frame"
            compressed_msg.format = "jpeg"
            
            # Compress image
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.config['output']['jpeg_quality']]
            _, buffer = cv2.imencode('.jpg', frame, encode_param)
            compressed_msg.data = np.array(buffer).tobytes()
            
            self.compressed_image_pub.publish(compressed_msg)
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish compressed image: {e}")

    def _publish_camera_info(self, timestamp: float):
        """Publish camera info message."""
        try:
            msg = CameraInfo()
            msg.header.stamp = self._to_ros_time(timestamp)
            msg.header.frame_id = "camera_optical_frame"
            
            # Image dimensions
            msg.width = self.config['streaming']['width']
            msg.height = self.config['streaming']['height']
            
            # Camera intrinsics
            K = self.gps_manager.current_intrinsic
            msg.k = K.flatten().tolist()
            
            # Distortion (assuming no distortion for now)
            msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
            msg.distortion_model = "plumb_bob"
            
            # Rectification and projection matrices
            msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            msg.p = [K[0,0], 0.0, K[0,2], 0.0,
                     0.0, K[1,1], K[1,2], 0.0,
                     0.0, 0.0, 1.0, 0.0]
            
            self.camera_info_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish camera info: {e}")

    def _publish_enu_point(self, gps_result: Dict[str, Any], timestamp: float):
        """Publish ENU point for visualization."""
        try:
            msg = PointStamped()
            msg.header.stamp = self._to_ros_time(timestamp)
            msg.header.frame_id = "map"  # or "enu" frame
            
            msg.point.x = gps_result['east_offset_m']
            msg.point.y = gps_result['north_offset_m']
            msg.point.z = 0.0
            
            self.target_enu_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Failed to publish ENU point: {e}")

    def _publish_gps_targets_list(self):
        """Publish list of clustered GPS targets."""
        try:
            clusters = self.gps_manager.get_active_clusters()
            
            for cluster in clusters:
                msg = NavSatFix()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = f"cluster_{cluster['id']}"
                
                lat, lon = cluster['center']
                msg.latitude = lat
                msg.longitude = lon
                msg.altitude = 0.0
                
                # Set covariance based on cluster size and age
                target_count = len(cluster['targets'])
                uncertainty = max(1.0, 10.0 / target_count)  # Less uncertainty with more targets
                
                msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                msg.position_covariance = [0.0] * 9
                msg.position_covariance[0] = uncertainty ** 2
                msg.position_covariance[4] = uncertainty ** 2
                msg.position_covariance[8] = 1000.0
                
                self.target_gps_list_pub.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f"Failed to publish GPS targets list: {e}")

    def _to_ros_time(self, timestamp: float):
        """Convert timestamp to ROS2 time message."""
        sec = int(timestamp)
        nanosec = int((timestamp - sec) * 1e9)
        
        time_msg = self.get_clock().now().to_msg()
        time_msg.sec = sec
        time_msg.nanosec = nanosec
        
        return time_msg

    # Sensor callbacks
    def _gps_callback(self, msg: NavSatFix):
        """Handle GPS messages."""
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.gps_readings.append((timestamp, (msg.latitude, msg.longitude)))
        self.current_gps = (msg.latitude, msg.longitude)

    def _altitude_callback(self, msg: Float64):
        """Handle altitude messages."""
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.altitude_readings.append((timestamp, msg.data))
        self.current_altitude = msg.data

    def _heading_callback(self, msg: Float64):
        """Handle heading messages."""
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.heading_readings.append((timestamp, msg.data))
        self.current_heading = msg.data

    def _gimbal_attitude_callback(self, msg: Vector3):
        """Handle gimbal attitude messages."""
        timestamp = self.get_clock().now().nanoseconds / 1e9
        self.gimbal_attitude_readings.append((timestamp, msg))
        self.current_gimbal_attitude = msg

    def _camera_mode_callback(self, msg: String):
        """Handle camera mode changes."""
        new_mode = msg.data.upper()
        if new_mode != self.current_camera_mode and new_mode in ['EO', 'IR']:
            self.get_logger().info(f"Switching camera mode from {self.current_camera_mode} to {new_mode}")
            self.current_camera_mode = new_mode
            
            # Update GPS manager intrinsics
            try:
                self.gps_manager.set_camera_type(new_mode)
                self._setup_camera_intrinsics()  # Reload with proper scaling
            except Exception as e:
                self.get_logger().error(f"Failed to update camera mode: {e}")

    def _image_callback(self, msg: CompressedImage):
        """Handle incoming compressed image messages."""
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
            with threading.Lock():
                self.current_frame = frame
                self.current_frame_timestamp = timestamp
                
            # Update statistics
            self.stats['frames_processed'] += 1
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    # Performance monitoring
    def _update_processing_stats(self, processing_time: float):
        """Update processing time statistics."""
        n = self.stats['detections_made']
        if n == 0:
            self.stats['average_processing_time'] = processing_time
        else:
            old_avg = self.stats['average_processing_time']
            self.stats['average_processing_time'] = (old_avg * (n - 1) + processing_time) / n

    def _log_performance_stats(self):
        """Log performance statistics."""
        stats = self.stats.copy()
        
        # Add component stats
        detector_stats = self.yolo_detector.get_stats()
        gps_stats = self.gps_manager.get_stats()
        
        self.get_logger().info(
            f"Performance Stats - "
            f"Frames: {stats['frames_processed']}, "
            f"Detections: {stats['detections_made']}, "
            f"GPS Estimates: {stats['gps_estimates']}, "
            f"Sync Failures: {stats['sync_failures']}, "
            f"Avg Processing: {stats['average_processing_time']:.3f}s, "
            f"Detection Time: {detector_stats.get('average_inference_time', 0):.1f}ms, "
            f"GPS Success Rate: {gps_stats.get('success_rate', 0):.2f}"
        )

    def _cleanup_old_clusters(self):
        """Clean up old GPS target clusters."""
        max_age = self.config['gps_estimation']['max_cluster_age_hours']
        self.gps_manager.cleanup_old_clusters(max_age)

    def destroy_node(self):
        """Clean shutdown of the node."""
        self.get_logger().info("Shutting down VisionGPSEstimatorNode...")
        
        # Stop detector
        self.yolo_detector.shutdown()
        
        super().destroy_node()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    try:
        node = VisionGPSEstimatorNode()
        
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
