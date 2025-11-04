#!/usr/bin/env python3
"""
Pipeline Validation Script for MCAP-YOLO-GPS
===============
Validate the entire YOLO detection + GPS conversion pipeline
Use the data from the MCAP file to test
"""

import sys
import os
import time
import json
import numpy as np
import cv2
import math
from pathlib import Path
from typing import Dict, List, Any, Optional, Tuple
import logging
from dataclasses import dataclass
from datetime import datetime

# Add parent directory to path for imports
sys.path.append(str(Path(__file__).parent.parent))

# Import modules
from vision_gps_estimator.yolo_detector import YOLODetector, DetectionResult
from vision_gps_estimator.gps_manager import GPSManagerFixed

# Try to import mcap reader
try:
    from mcap_ros2.reader import read_ros2_messages
    MCAP_AVAILABLE = True
except ImportError as e:
    MCAP_AVAILABLE = False
    print(f"Warning: mcap libraries import error: {e}")

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


@dataclass
class ValidationResult:
    """Store validation results"""
    timestamp: float
    frame_id: int
    detections: List[Dict[str, Any]]
    gps_estimates: List[Dict[str, Any]]
    processing_time: float
    errors: List[str]


class PipelineValidator:
    """Pipeline validator"""
    
    def __init__(self, mcap_file: str, output_dir: str = "./validation_output"):
        """
        Initialize validator
        
        Args:
            mcap_file: MCAP file path
            output_dir: Output directory
        """
        self.mcap_file = mcap_file
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Initialize components
        self._init_detector()
        self._init_gps_manager()
        
        # Validation results
        self.results = []
        
        # Statistics
        self.stats = {
            'total_frames': 0,
            'frames_with_detections': 0,
            'total_detections': 0,
            'total_gps_estimates': 0,
            'processing_errors': 0,
            'average_processing_time': 0.0
        }
        
        logger.info(f"Pipeline validator initialized")
        logger.info(f"MCAP file: {mcap_file}")
        logger.info(f"Output directory: {output_dir}")
    
    def _init_detector(self):
        """Initialize YOLO detector"""
        detector_config = {
            'model_path': '/home/yliu/Documents/github/georgia_dtc_ops_team_chiron_mrsd/airstack/ros_ws/src/yolo12s.pt',
            'conf_threshold': 0.1,  # Further reduce confidence threshold
            'iou_threshold': 0.45,
            'max_detections': 10,
            'input_size': 640,
            'target_classes': [0],  # Person class
            'enable_preprocessing': False
        }
        
        # If model file does not exist, use default path
        if not Path(detector_config['model_path']).exists():
            logger.warning(f"Model not found at {detector_config['model_path']}")
            detector_config['model_path'] = 'models/yolo11s-pose.engine'
        
        self.detector = YOLODetector(detector_config)
        
        # Try to load model
        if not self.detector.load_model():
            logger.warning("Failed to load YOLO model - detection will be simulated")
            self.detector = None
    
    def _init_gps_manager(self):
        """Initialize GPS manager"""
        gps_config = {
            'min_distance_threshold': 5.0,
            'max_cluster_age_hours': 24.0,
            'cluster_buffer_size': 30
        }
        
        self.gps_manager = GPSManagerFixed(config=gps_config)
        
        # Set default camera intrinsics (EO camera)
        self.gps_manager.set_camera_intrinsics(
            fx=475.0, fy=505.0,
            cx=320.0, cy=256.0
        )
    
    def validate_from_mcap(self):
        """Validate pipeline from MCAP file"""
        if not MCAP_AVAILABLE:
            logger.error("MCAP libraries not available")
            return self._validate_with_test_data()
        
        try:
            # Extract MCAP data
            logger.info("Extracting data from MCAP file...")
            image_messages, sensor_data = self._extract_mcap_data()
            
            if not image_messages:
                logger.warning("No image messages found in MCAP")
                return self._validate_with_test_data()
            
            logger.info(f"Found {len(image_messages)} image messages")
            
            # Process each image frame - process all frames to find targets
            frames_to_process = len(image_messages)  # 处理所有可用帧
            logger.info(f"Will process {frames_to_process} frames to find targets")
            
            for idx, (timestamp, image_data) in enumerate(image_messages[:frames_to_process]):
                logger.info(f"\nProcessing frame {idx+1}/{frames_to_process}")
                logger.debug(f"Image data type: {type(image_data)}")
                
                # Get synchronized sensor data
                sync_data = self._get_synchronized_data(timestamp, sensor_data)
                
                # Process frame
                result = self._process_frame(
                    image_data, 
                    timestamp, 
                    idx,
                    sync_data
                )
                
                self.results.append(result)
                self._update_statistics(result)
                
                # Save visualization - save first 10 frames and all frames with detections
                should_save_viz = (idx < 10 or len(result.detections) > 0) and not result.errors
                if should_save_viz:
                    # Re-decode image for visualization
                    vis_image = None
                    if isinstance(image_data, bytes):
                        # Process raw byte data
                        if len(image_data) == 983040:  # 640x512x3
                            vis_image = np.frombuffer(image_data, dtype=np.uint8).reshape((512, 640, 3))
                    elif hasattr(image_data, 'data') and hasattr(image_data, 'encoding'):
                        # ROS Image message
                        vis_image = self._decode_ros_image(image_data)
                    
                    if vis_image is not None:
                        self._save_visualization(vis_image, result, idx)
            
            # Generate report
            self._generate_report()
            
        except Exception as e:
            logger.error(f"Error validating from MCAP: {e}")
            return self._validate_with_test_data()
    
    def _extract_mcap_data(self) -> Tuple[List, Dict]:
        """Extract data from MCAP file"""
        image_messages = []
        sensor_data = {
            'gps': [],
            'altitude': [],
            'heading': [],
            'gimbal': []
        }
        
        try:
            # Count all topics
            topic_count = {}
            
            # Directly pass file path to read_ros2_messages
            for message in read_ros2_messages(self.mcap_file):
                topic = message.channel.topic
                data = message.ros_msg
                                  # log_time is nanosecond timestamp
                timestamp = message.log_time / 1e9 if isinstance(message.log_time, (int, float)) else message.log_time.timestamp()
                
                # Count topics
                if topic not in topic_count:
                    topic_count[topic] = 0
                topic_count[topic] += 1
                
                # Extract image messages
                if 'image' in topic.lower() or 'camera' in topic.lower():
                    if hasattr(data, 'format'):
                        # CompressedImage
                        logger.info(f"Found CompressedImage topic: {topic}, format: {data.format}")
                        image_messages.append((timestamp, data.data))
                    elif hasattr(data, 'encoding'):
                        # Image (raw)
                        logger.info(f"Found Image topic: {topic}, encoding: {data.encoding}, size: {data.width}x{data.height}, step: {data.step if hasattr(data, 'step') else 'N/A'}")
                        image_messages.append((timestamp, data))
                    elif hasattr(data, 'data'):
                        # Unknown format with data
                        logger.warning(f"Found unknown image topic: {topic}, data size: {len(data.data)} bytes")
                        image_messages.append((timestamp, data.data))
                
                # Extract altitude data - must be checked before GPS because topic name may contain 'global_position'
                elif 'rel_alt' in topic.lower():
                    if hasattr(data, 'data'):
                        sensor_data['altitude'].append((timestamp, data.data))
                
                # Extract heading data - must be checked before GPS because topic name may contain 'global_position'
                elif 'compass_hdg' in topic.lower():
                    if hasattr(data, 'data'):
                        sensor_data['heading'].append((timestamp, data.data))
                
                # Extract GPS data
                elif ('gps' in topic.lower() or 'global_position' in topic.lower()) and 'rel_alt' not in topic.lower() and 'compass' not in topic.lower():
                    if hasattr(data, 'latitude') and hasattr(data, 'longitude'):
                        sensor_data['gps'].append((timestamp, data.latitude, data.longitude))
                
                # Other altitude data format
                elif 'altitude' in topic.lower():
                    if hasattr(data, 'data'):
                        sensor_data['altitude'].append((timestamp, data.data))
                
                # Extract gimbal attitude
                elif 'gimbal' in topic.lower():
                    if hasattr(data, 'x') and hasattr(data, 'y') and hasattr(data, 'z'):
                        sensor_data['gimbal'].append((timestamp, data.x, data.y, data.z))
            
            # Print all topic statistics
            logger.info("\nAll topics in MCAP file:")
            for topic, count in sorted(topic_count.items()):
                logger.info(f"  {topic}: {count} messages")
            
            logger.info(f"\nExtracted {len(image_messages)} images, "
                       f"{len(sensor_data['gps'])} GPS, "
                       f"{len(sensor_data['altitude'])} altitude, "
                       f"{len(sensor_data['heading'])} heading, "
                       f"{len(sensor_data['gimbal'])} gimbal messages")
            
            return image_messages, sensor_data
            
        except Exception as e:
            logger.error(f"Error reading MCAP file: {e}")
            return [], {}
    
    def _validate_with_test_data(self):
        """Validate pipeline with synthetic test data"""
        logger.info("Validating with synthetic test data...")
        
        # Create test images
        test_images = []
        for i in range(5):
            # Create test images with rectangles
            img = np.zeros((512, 640, 3), dtype=np.uint8)
            cv2.rectangle(img, (200 + i*50, 150), (300 + i*50, 350), (255, 255, 255), -1)
            cv2.putText(img, f"Test Frame {i+1}", (50, 50), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            test_images.append(img)
        
        # Test sensor data
        test_sensor_data = {
            'gps': (40.7128, -74.0060),  # New York
            'altitude': 50.0,  # 50 meters
            'heading': 45.0,  # Northeast  
            'gimbal': (-45.0, 0.0, 0.0)  # 45 degree downward
        }
        
        # Process each test image
        for idx, img in enumerate(test_images):
            timestamp = time.time() + idx
            
            logger.info(f"\nProcessing test frame {idx+1}/{len(test_images)}")
            
            # Set GPS manager state
            self.gps_manager.set_current_state(
                gps=test_sensor_data['gps'],
                height=test_sensor_data['altitude'],
                heading=math.radians(test_sensor_data['heading']),
                gimbal_attitude=tuple(math.radians(a) for a in test_sensor_data['gimbal'])
            )
            
            # Process frame
            result = self._process_frame(img, timestamp, idx, test_sensor_data)
            self.results.append(result)
            self._update_statistics(result)
            
            # Save visualization
            self._save_visualization(img, result, idx)
        
        # Generate report
        self._generate_report()
    
    def _get_synchronized_data(self, timestamp: float, sensor_data: Dict) -> Dict:
        """Get synchronized sensor data"""
        sync_data = {}
        max_time_diff = 0.1  # 100ms
        
        # Find closest GPS data
        if sensor_data['gps']:
            closest_gps = min(sensor_data['gps'], 
                            key=lambda x: abs(x[0] - timestamp))
            if abs(closest_gps[0] - timestamp) < max_time_diff:
                sync_data['gps'] = (closest_gps[1], closest_gps[2])
        
        # Find closest altitude data
        if sensor_data['altitude']:
            closest_alt = min(sensor_data['altitude'],
                            key=lambda x: abs(x[0] - timestamp))
            if abs(closest_alt[0] - timestamp) < max_time_diff:
                sync_data['altitude'] = closest_alt[1]
        
        # Find closest heading data
        if sensor_data['heading']:
            closest_heading = min(sensor_data['heading'],
                                key=lambda x: abs(x[0] - timestamp))
            if abs(closest_heading[0] - timestamp) < max_time_diff:
                sync_data['heading'] = closest_heading[1]
        
        # Find closest gimbal data
        if sensor_data['gimbal']:
            closest_gimbal = min(sensor_data['gimbal'],
                               key=lambda x: abs(x[0] - timestamp))
            if abs(closest_gimbal[0] - timestamp) < max_time_diff:
                sync_data['gimbal'] = (closest_gimbal[1], closest_gimbal[2], closest_gimbal[3])
        
        # Use default values to fill missing data
        if 'gps' not in sync_data:
            logger.warning("No GPS data synchronized, using default")
            sync_data['gps'] = (40.7128, -74.0060)
        if 'altitude' not in sync_data:
            logger.warning("No altitude data synchronized, using default 50m")
            sync_data['altitude'] = 50.0
        if 'heading' not in sync_data:
            logger.warning("No heading data synchronized, using default 0°")
            sync_data['heading'] = 0.0
        if 'gimbal' not in sync_data:
            logger.warning("No gimbal data synchronized, using default")
            sync_data['gimbal'] = (-90.0, 0.0, 0.0)
        
        return sync_data
    
    def _process_frame(self, image_data: Any, timestamp: float, 
                      frame_id: int, sensor_data: Dict) -> ValidationResult:
        """Process single frame"""
        start_time = time.time()
        errors = []
        detections = []
        gps_estimates = []
        
        try:
            # Decode image
            if isinstance(image_data, bytes):
                logger.info(f"Image data type: bytes, size: {len(image_data)} bytes")
                
                # Check if this might be raw pixel data
                # Common raw image sizes: 640x512x3=983040, 1280x720x3=2764800, 1920x1080x3=6220800
                raw_formats = [
                    (983040, 640, 512, 3),    # 640x512 RGB/BGR
                    (327680, 640, 512, 1),    # 640x512 Grayscale
                    (2764800, 1280, 720, 3),  # 1280x720 RGB/BGR
                    (921600, 1280, 720, 1),   # 1280x720 Grayscale
                    (6220800, 1920, 1080, 3), # 1920x1080 RGB/BGR
                    (2073600, 1920, 1080, 1), # 1920x1080 Grayscale
                ]
                
                is_raw = False
                for expected_size, width, height, channels in raw_formats:
                    if len(image_data) == expected_size:
                        logger.info(f"Detected possible raw image format: {width}x{height}, {channels} channel(s)")
                        
                        # Try to reshape as raw pixel data
                        try:
                            if channels == 3:
                                # Try BGR (OpenCV default)
                                image = np.frombuffer(image_data, dtype=np.uint8).reshape((height, width, channels))
                                logger.info(f"Successfully decoded as raw BGR image: {image.shape}")
                                is_raw = True
                            elif channels == 1:
                                # Grayscale
                                image = np.frombuffer(image_data, dtype=np.uint8).reshape((height, width))
                                image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
                                logger.info(f"Successfully decoded as raw grayscale image: {image.shape}")
                                is_raw = True
                        except Exception as e:
                            logger.warning(f"Failed to reshape as {width}x{height}x{channels}: {e}")
                        
                        if is_raw:
                            break
                
                # If not raw format, try standard image decoding
                if not is_raw:
                    logger.info(f"First 16 bytes (hex): {image_data[:16].hex()}")
                    logger.info(f"First 4 bytes (ASCII): {repr(image_data[:4])}")
                    
                    # Check for common image format signatures
                    if image_data[:4] == b'\xff\xd8\xff\xe0':
                        logger.info("Detected JPEG format (FFD8FFE0)")
                    elif image_data[:4] == b'\xff\xd8\xff\xe1':
                        logger.info("Detected JPEG format (FFD8FFE1)")
                    elif image_data[:8] == b'\x89PNG\r\n\x1a\n':
                        logger.info("Detected PNG format")
                    else:
                        logger.warning(f"Unknown image format. First 32 bytes: {image_data[:32].hex()}")
                    
                    nparr = np.frombuffer(image_data, np.uint8)
                    image = cv2.imdecode(nparr, cv2.IMREAD_COLOR)
                    
                    if image is None:
                        logger.error(f"Failed to decode image data")
                        logger.error(f"Data size: {len(image_data)} bytes")
                        logger.error(f"First 64 bytes (hex): {image_data[:64].hex()}")
                        errors.append(f"Failed to decode image: size={len(image_data)}, header={image_data[:4].hex()}")
                        return ValidationResult(
                            timestamp=timestamp,
                            frame_id=frame_id,
                            detections=[],
                            gps_estimates=[],
                            processing_time=time.time() - start_time,
                            errors=errors
                        )
            elif isinstance(image_data, np.ndarray):
                image = image_data
            elif hasattr(image_data, 'data') and hasattr(image_data, 'encoding'):
                # ROS Image message
                logger.info(f"Processing ROS Image message with encoding: {image_data.encoding}")
                image = self._decode_ros_image(image_data)
            else:
                # Unknown format
                logger.error(f"Unknown image data type: {type(image_data)}")
                errors.append(f"Unknown image data type: {type(image_data)}")
                return ValidationResult(
                    timestamp=timestamp,
                    frame_id=frame_id,
                    detections=[],
                    gps_estimates=[],
                    processing_time=time.time() - start_time,
                    errors=errors
                )
            
            # Run YOLO detection
            if self.detector:
                detection_result = self.detector.infer(image)
                
                if detection_result:
                    if len(detection_result.boxes) > 0:
                        logger.info(f"✓ Found {len(detection_result.boxes)} detections!")
                        for i, score in enumerate(detection_result.scores):
                            logger.info(f"  Detection {i+1}: confidence={score:.3f}")
                    
                    # Set GPS manager state
                    self.gps_manager.set_current_state(
                        gps=sensor_data['gps'],
                        height=sensor_data['altitude'],
                        heading=math.radians(sensor_data['heading']),
                        gimbal_attitude=tuple(math.radians(a) for a in sensor_data['gimbal'])
                    )
                    
                    # Process each detection
                    for i in range(len(detection_result.boxes)):
                        box = detection_result.boxes[i]
                        center = detection_result.centers[i]
                        score = detection_result.scores[i]
                        
                        detection = {
                            'box': box.tolist(),
                            'center': center.tolist(),
                            'score': float(score),
                            'class': int(detection_result.classes[i])
                        }
                        detections.append(detection)
                        
                        # Estimate GPS coordinates
                        try:
                            gps_result = self.gps_manager.estimate_target_gps(
                                center[0], center[1]
                            )
                            
                            gps_estimate = {
                                'pixel': center.tolist(),
                                'latitude': gps_result['estimated_latitude'],
                                'longitude': gps_result['estimated_longitude'],
                                'distance_m': gps_result['lateral_distance_m'],
                                'forward_offset_m': gps_result['forward_offset_m'],
                                'right_offset_m': gps_result['right_offset_m']
                            }
                            gps_estimates.append(gps_estimate)
                            
                            logger.info(f"  Detection {i+1}: center=({center[0]:.1f}, {center[1]:.1f}), "
                                      f"GPS=({gps_result['estimated_latitude']:.6f}, "
                                      f"{gps_result['estimated_longitude']:.6f}), "
                                      f"distance={gps_result['lateral_distance_m']:.2f}m")
                            
                            # Record drone position for reference
                            if i == 0:  # Only record first detection
                                logger.info(f"  无人机 GPS: ({sensor_data['gps'][0]:.6f}, {sensor_data['gps'][1]:.6f}), "
                                          f"高度: {sensor_data['altitude']:.2f}m, "
                                          f"航向: {sensor_data['heading']:.1f}°")
                            
                        except Exception as e:
                            error_msg = f"GPS estimation failed for detection {i}: {e}"
                            logger.error(error_msg)
                            errors.append(error_msg)
            else:
                # Simulate detection result
                detection = {
                    'box': [200, 150, 300, 350],
                    'center': [250, 250],
                    'score': 0.95,
                    'class': 0
                }
                detections.append(detection)
                
                # Simulate GPS estimation
                gps_estimate = {
                    'pixel': [250, 250],
                    'latitude': sensor_data['gps'][0] + 0.0001,
                    'longitude': sensor_data['gps'][1] + 0.0001,
                    'distance_m': 10.0,
                    'forward_offset_m': 7.0,
                    'right_offset_m': 7.0
                }
                gps_estimates.append(gps_estimate)
                
        except Exception as e:
            error_msg = f"Frame processing error: {e}"
            logger.error(error_msg)
            errors.append(error_msg)
        
        processing_time = time.time() - start_time
        
        return ValidationResult(
            timestamp=timestamp,
            frame_id=frame_id,
            detections=detections,
            gps_estimates=gps_estimates,
            processing_time=processing_time,
            errors=errors
        )
    
    def _decode_ros_image(self, img_msg) -> np.ndarray:
        """Decode ROS Image message - encoding aware version"""
        # Process CompressedImage
        if hasattr(img_msg, "format"):  # CompressedImage
            buf = np.frombuffer(img_msg.data, dtype=np.uint8)
            cv_img = cv2.imdecode(buf, cv2.IMREAD_ANYCOLOR)
            if cv_img is None:
                raise ValueError("cv2.imdecode failed for CompressedImage")
            return cv_img

        # sensor_msgs/Image (uncompressed)
        H, W = img_msg.height, img_msg.width
        step = img_msg.step if hasattr(img_msg, 'step') else W * 3  # Default assume RGB
        enc = img_msg.encoding.lower() if hasattr(img_msg, 'encoding') else 'bgr8'
        big_endian = getattr(img_msg, "is_bigendian", 0) == 1

        logger.info(f"Decoding ROS Image: {W}x{H}, encoding={enc}, step={step}")

        # Map common encodings to dtype and channel count
        if enc in ("bgr8", "rgb8"):
            dtype, ch = np.uint8, 3
        elif enc in ("mono8", "8uc1"):
            dtype, ch = np.uint8, 1
        elif enc in ("mono16", "16uc1"):
            dtype, ch = np.uint16, 1
        elif enc in ("bgra8", "rgba8"):
            dtype, ch = np.uint8, 4
        elif enc in ("32fc1",):
            dtype, ch = np.float32, 1
        elif enc in ("yuv422", "yuyv"):
            # YUV422/YUYV format
            return self._decode_yuv422(img_msg)
        elif enc in ("uyvy",):
            # UYVY format
            return self._decode_uyvy(img_msg)
        elif enc in ("nv12", "nv21"):
            # NV12/NV21 format
            return self._decode_nv12_nv21(img_msg, enc)
        else:
            logger.warning(f"Unsupported encoding: {img_msg.encoding}, trying default bgr8")
            dtype, ch = np.uint8, 3

        row_bytes = step
        buf = np.frombuffer(img_msg.data, dtype=dtype)
        
        if big_endian and dtype != np.uint8:
            buf = buf.byteswap().newbyteorder()

        # Process possible row padding (using step)
        expected_row_bytes = W * ch * np.dtype(dtype).itemsize
        
        if row_bytes == expected_row_bytes:
            # No padding, directly reshape
            if ch > 1:
                cv_img = buf.reshape(H, W, ch)
            else:
                cv_img = buf.reshape(H, W)
        else:
            # Padding detected, need to process row by row
            logger.debug(f"Row padding detected: step={row_bytes}, expected={expected_row_bytes}")
            items_per_row = row_bytes // np.dtype(dtype).itemsize
            expected_items = expected_row_bytes // np.dtype(dtype).itemsize
            
            rows = []
            for i in range(H):
                start = i * items_per_row
                end = start + expected_items
                rows.append(buf[start:end])
            
            cv_img = np.vstack(rows)
            if ch > 1:
                cv_img = cv_img.reshape(H, W, ch)
            else:
                cv_img = cv_img.reshape(H, W)

        # Color format conversion (to BGR for OpenCV use)
        if enc == "rgb8":
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGB2BGR)
        elif enc == "rgba8":
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_RGBA2BGR)
        elif enc == "bgra8":
            cv_img = cv2.cvtColor(cv_img, cv2.COLOR_BGRA2BGR)

        return cv_img
    
    def _decode_yuv422(self, img_msg) -> np.ndarray:
        """Decode YUV422/YUYV format"""
        H, W = img_msg.height, img_msg.width
        buf = np.frombuffer(img_msg.data, dtype=np.uint8)
        
        # YUYV format: Y0 U Y1 V
        yuyv = buf.reshape((H, W * 2))
        bgr = cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUYV)
        return bgr
    
    def _decode_uyvy(self, img_msg) -> np.ndarray:
        """Decode UYVY format"""
        H, W = img_msg.height, img_msg.width
        buf = np.frombuffer(img_msg.data, dtype=np.uint8)
        
        # UYVY format: U Y0 V Y1
        uyvy = buf.reshape((H, W * 2))
        bgr = cv2.cvtColor(uyvy, cv2.COLOR_YUV2BGR_UYVY)
        return bgr
    
    def _decode_nv12_nv21(self, img_msg, encoding: str) -> np.ndarray:
        """Decode NV12/NV21 format"""
        H, W = img_msg.height, img_msg.width
        buf = np.frombuffer(img_msg.data, dtype=np.uint8)
        
        # NV12: Y plane + UV interleaved
        # NV21: Y plane + VU interleaved
        yuv = buf.reshape((H * 3 // 2, W))
        
        if encoding == "nv12":
            bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV12)
        else:  # nv21
            bgr = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_NV21)
        
        return bgr
    
    def _save_visualization(self, image: np.ndarray, result: ValidationResult, idx: int):
        """Save visualization result"""
        try:
            # Ensure image is not bytes
            if isinstance(image, bytes):
                logger.warning("Image is still in bytes format, skipping visualization")
                return
            
            vis_image = image.copy()
            
            # Draw detection boxes
            for detection in result.detections:
                box = detection['box']
                center = detection['center']
                score = detection['score']
                
                # Draw bounding box
                cv2.rectangle(vis_image, 
                            (int(box[0]), int(box[1])),
                            (int(box[2]), int(box[3])),
                            (0, 255, 0), 2)
                
                # Draw center point
                cv2.circle(vis_image, 
                          (int(center[0]), int(center[1])),
                          5, (255, 0, 0), -1)
                
                # Add label
                label = f"Score: {score:.2f}"
                cv2.putText(vis_image, label,
                          (int(box[0]), int(box[1]) - 10),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            # Add GPS information
            y_offset = 30
            for i, gps_est in enumerate(result.gps_estimates):
                text = f"Target {i+1}: ({gps_est['latitude']:.6f}, {gps_est['longitude']:.6f}), {gps_est['distance_m']:.1f}m"
                cv2.putText(vis_image, text,
                          (10, y_offset + i*25),
                          cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            # Save image
            output_path = self.output_dir / f"frame_{idx:04d}.jpg"
            cv2.imwrite(str(output_path), vis_image)
            logger.info(f"Saved visualization to {output_path}")
            
        except Exception as e:
            logger.error(f"Failed to save visualization: {e}")
    
    def _update_statistics(self, result: ValidationResult):
        """Update statistics"""
        self.stats['total_frames'] += 1
        
        if result.detections:
            self.stats['frames_with_detections'] += 1
            self.stats['total_detections'] += len(result.detections)
        
        self.stats['total_gps_estimates'] += len(result.gps_estimates)
        
        if result.errors:
            self.stats['processing_errors'] += len(result.errors)
        
        # Update average processing time
        n = self.stats['total_frames']
        old_avg = self.stats['average_processing_time']
        self.stats['average_processing_time'] = (
            (old_avg * (n - 1) + result.processing_time) / n
        )
    
    def _generate_report(self):
        """Generate validation report"""
        report = {
            'timestamp': datetime.now().isoformat(),
            'mcap_file': self.mcap_file,
            'statistics': self.stats,
            'results': []
        }
        
        # Add result summary
        for result in self.results:
            summary = {
                'frame_id': result.frame_id,
                'timestamp': result.timestamp,
                'num_detections': len(result.detections),
                'num_gps_estimates': len(result.gps_estimates),
                'processing_time_ms': result.processing_time * 1000,
                'has_errors': len(result.errors) > 0
            }
            
            if result.gps_estimates:
                # Add first GPS estimate as example
                gps = result.gps_estimates[0]
                summary['example_gps'] = {
                    'latitude': gps['latitude'],
                    'longitude': gps['longitude'],
                    'distance_m': gps['distance_m']
                }
            
            report['results'].append(summary)
        
        # Save JSON report
        report_path = self.output_dir / "validation_report.json"
        with open(report_path, 'w') as f:
            json.dump(report, f, indent=2)
        
        # Print summary
        print("\n" + "="*60)
        print("PIPELINE VALIDATION REPORT")
        print("="*60)
        print(f"MCAP File: {self.mcap_file}")
        print(f"Total Frames Processed: {self.stats['total_frames']}")
        print(f"Frames with Detections: {self.stats['frames_with_detections']}")
        print(f"Total Detections: {self.stats['total_detections']}")
        print(f"Total GPS Estimates: {self.stats['total_gps_estimates']}")
        print(f"Processing Errors: {self.stats['processing_errors']}")
        print(f"Average Processing Time: {self.stats['average_processing_time']*1000:.2f} ms")
        print(f"\nReport saved to: {report_path}")
        print(f"Visualizations saved to: {self.output_dir}")
        print("="*60)


def main():
    """Main function"""
    # Check command line parameters
    if len(sys.argv) < 2:
        mcap_file = "/home/yliu/Documents/github/georgia_dtc_ops_team_chiron_mrsd/2025-05-07_06-04-01/recording/recording_4.mcap"
    else:
        mcap_file = sys.argv[1]
    
    # Check if file exists
    if not Path(mcap_file).exists():
        logger.error(f"MCAP file not found: {mcap_file}")
        logger.info("Using test data instead...")
        mcap_file = None
    
    # Create validator
    output_dir = f"./validation_output_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    validator = PipelineValidator(mcap_file or "test", output_dir)
    
    # Run validation
    if mcap_file and MCAP_AVAILABLE:
        validator.validate_from_mcap()
    else:
        validator._validate_with_test_data()


if __name__ == "__main__":
    main()
