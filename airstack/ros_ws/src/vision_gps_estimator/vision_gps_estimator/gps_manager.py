#!/usr/bin/env python3
"""
Enhanced GPS Manager with Standardized Coordinate Systems for AirStack
=====================================================================

Refactored from original humanflow gps_manager.py with:
- Enhanced parameter loading and validation
- Improved error handling and logging  
- Deprecated method annotations
- Performance optimizations

Coordinate System Definitions:
-----------------------------

1. CAMERA OPTICAL FRAME (OpenCV/ROS standard):
   - X_cam: Right in image (positive = right direction in image)
   - Y_cam: Down in image (positive = down direction in image) 
   - Z_cam: Forward along optical axis (positive = into scene)

2. DRONE BODY FRAME (NED-based):
   - X_drone: Forward (positive = nose direction)
   - Y_drone: Right (positive = right wing direction)
   - Z_drone: Down (positive = toward ground)

3. GIMBAL ATTITUDE (from gimbal_status_node.cpp):
   - Pitch: Rotation around Y-axis (positive = nose up)
   - Roll: Rotation around X-axis (positive = right wing down)
   - Yaw: Rotation around Z-axis (positive = nose right/clockwise from above)

4. WORLD FRAME (ENU - East North Up):
   - X_world: East (positive = eastward)
   - Y_world: North (positive = northward) 
   - Z_world: Up (positive = upward)

5. DRONE HEADING:
   - 0° = North, 90° = East, 180° = South, 270° = West
   - Positive rotation is clockwise when viewed from above
"""

from typing import Dict, Tuple, List, Optional, Any
import math
import time
import logging
import warnings
import numpy as np
from scipy.spatial.transform import Rotation

logger = logging.getLogger(__name__)


class GPSManagerFixed:
    """
    Simplified GPS Manager with fixed coordinate system handling.
    
    This version removes clustering functionality and focuses on direct GPS estimation.
    Always publishes estimated GPS coordinates without buffering or clustering.
    """
    
    def __init__(self, config: Optional[Dict[str, Any]] = None, buffer_size: int = 30):
        """
        Initialize GPS Manager with configuration.
        
        Args:
            config: Configuration dictionary with camera and estimation parameters
            buffer_size: Not used anymore (kept for compatibility)
        """
        # Configuration
        self.config = config or {}
        
        # Camera intrinsics - will be set by configuration
        self.current_intrinsic = None
        self.current_camera_type = None
        self._load_default_intrinsics()
        
        # Current state for GPS estimation
        self.current_gps: Optional[Tuple[float, float]] = None
        self.current_height: Optional[float] = None
        self.current_heading: Optional[float] = None  # radians, 0=North, +ve=clockwise
        self.current_gimbal_attitude: Optional[Tuple[float, float, float]] = None  # (pitch, roll, yaw) in radians
        
        # Performance tracking
        self.stats = {
            'total_estimations': 0,
            'successful_estimations': 0,
            'failed_estimations': 0
        }
        
        logger.info("GPSManagerFixed initialized with enhanced coordinate system handling")

    def _load_default_intrinsics(self):
        """Load default camera intrinsics for EO and IR cameras."""
        # Default EO camera intrinsics
        self.intrinsics_eo = np.array([
            [475.0, 0.0, 320.0],
            [0.0, 505.0, 256.0],
            [0.0, 0.0, 1.0]
        ])
        
        # Default IR camera intrinsics  
        self.intrinsics_ir = np.array([
            [2267.0, 0.0, 640.0],
            [0.0, 1593.0, 360.0],
            [0.0, 0.0, 1.0]
        ])
        
        # Set default to EO
        self.current_intrinsic = self.intrinsics_eo.copy()
        self.current_camera_type = "EO"

    def set_camera_intrinsics(self, fx: float, fy: float, cx: float, cy: float):
        """
        Set camera intrinsic parameters.
        
        Args:
            fx, fy: Focal lengths in pixels
            cx, cy: Principal point coordinates
        """
        self.current_intrinsic = np.array([
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0]
        ])
        logger.info(f"Camera intrinsics updated: fx={fx}, fy={fy}, cx={cx}, cy={cy}")

    def set_camera_type(self, camera_type: str):
        """
        Set camera type and update intrinsics accordingly.
        
        Args:
            camera_type: "EO" or "IR"
        """
        camera_type = camera_type.upper()
        
        if camera_type == "EO":
            self.current_intrinsic = self.intrinsics_eo.copy()
        elif camera_type == "IR":
            self.current_intrinsic = self.intrinsics_ir.copy()
        else:
            logger.error(f"Unknown camera type: {camera_type}")
            raise ValueError(f"Unknown camera type: {camera_type}")
        
        self.current_camera_type = camera_type
        logger.info(f"Camera type set to: {camera_type}")

    def load_intrinsics_from_config(self, camera_config: Dict[str, Any], 
                                   image_width: int, image_height: int, 
                                   digital_zoom_factor: float = 1.0):
        """
        Load camera intrinsics from configuration with resolution scaling and digital zoom compensation.
        
        Args:
            camera_config: Configuration dictionary with camera parameters
            image_width: Current image width
            image_height: Current image height
            digital_zoom_factor: Digital zoom multiplier (1.0 = no zoom, 2.0 = 2x zoom)
        """
        try:
            camera_type = camera_config.get('mode', 'EO').upper()
            intrinsics_config = camera_config.get('intrinsics', {}).get(camera_type, {})
            
            if not intrinsics_config:
                logger.warning(f"No intrinsics found for camera type {camera_type}, using defaults")
                self.set_camera_type(camera_type)
                return
            
            # Extract intrinsic parameters
            fx = intrinsics_config.get('fx', 475.0)
            fy = intrinsics_config.get('fy', 505.0)
            cx = intrinsics_config.get('cx', 320.0)
            cy = intrinsics_config.get('cy', 256.0)
            
            # Scale intrinsics if resolution differs from calibration resolution
            # Assume calibration was done at standard resolutions
            if camera_type == "EO":
                cal_width, cal_height = 640, 512
            else:  # IR
                cal_width, cal_height = 1280, 720
            
            # Scale intrinsics for resolution changes
            if image_width != cal_width or image_height != cal_height:
                scale_x = image_width / cal_width
                scale_y = image_height / cal_height
                
                fx *= scale_x
                fy *= scale_y
                cx *= scale_x
                cy *= scale_y
                
                logger.info(f"Scaled intrinsics for resolution {image_width}x{image_height} "
                           f"(from {cal_width}x{cal_height})")
            
            # Apply digital zoom compensation
            if digital_zoom_factor != 1.0:
                fx *= digital_zoom_factor
                fy *= digital_zoom_factor
                # Note: cx, cy remain unchanged as zoom center stays at image center
                
                logger.info(f"Applied digital zoom factor: {digital_zoom_factor}x")
            
            self.set_camera_intrinsics(fx, fy, cx, cy)
            self.current_camera_type = camera_type
            
        except Exception as e:
            logger.error(f"Failed to load intrinsics from config: {e}")
            self.set_camera_type(camera_type)

    def set_current_state(self, gps: Tuple[float, float], height: float, 
                         heading: float, gimbal_attitude: Tuple[float, float, float]):
        """
        Set the current state needed for GPS estimation.
        
        Args:
            gps: (latitude, longitude) in degrees
            height: Height above ground in meters
            heading: Drone heading in radians (0=North, positive=clockwise)
            gimbal_attitude: (pitch, roll, yaw) in radians
        """
        # Validate inputs
        if not self._validate_gps_coordinates(gps[0], gps[1]):
            raise ValueError(f"Invalid GPS coordinates: {gps}")
        
        if height < 0:
            logger.warning(f"Negative height value: {height}")
        
        if not (-math.pi <= heading <= math.pi):
            logger.warning(f"Heading outside [-π, π]: {heading}")
        
        # Validate gimbal angles (reasonable ranges)
        pitch, roll, yaw = gimbal_attitude
        if not (-math.pi/2 <= pitch <= math.pi/2):
            logger.warning(f"Gimbal pitch outside [-π/2, π/2]: {pitch}")
        if not (-math.pi <= roll <= math.pi):
            logger.warning(f"Gimbal roll outside [-π, π]: {roll}")
        if not (-math.pi <= yaw <= math.pi):
            logger.warning(f"Gimbal yaw outside [-π, π]: {yaw}")
        
        self.current_gps = gps
        self.current_height = height
        self.current_heading = heading
        self.current_gimbal_attitude = gimbal_attitude

    def _validate_gps_coordinates(self, lat: float, lon: float) -> bool:
        """Validate GPS coordinates are within valid ranges."""
        return (-90 <= lat <= 90) and (-180 <= lon <= 180)

    def estimate_target_gps(self, pixel_x: float, pixel_y: float) -> Dict[str, Any]:
        """
        Estimate GPS coordinates of a target from its pixel coordinates.
        
        This is the main GPS estimation function with corrected coordinate transformations.
        
        Args:
            pixel_x: X coordinate in image (0 = left edge)
            pixel_y: Y coordinate in image (0 = top edge)
            
        Returns:
            Dictionary containing estimation results
        """
        try:
            # Validate inputs and state
            if not self._validate_state():
                raise ValueError("GPS estimation state not properly initialized")
            
            if self.current_intrinsic is None:
                raise ValueError("Camera intrinsics not set")
            
            self.stats['total_estimations'] += 1
            
            # Step 1: Convert pixel to normalized camera coordinates
            pixel_coords = np.array([pixel_x, pixel_y, 1.0])
            K_inv = np.linalg.inv(self.current_intrinsic)
            normalized_coords = K_inv @ pixel_coords  # [x_norm, y_norm, 1.0]
            
            # Step 2: Define camera-to-drone transformation at gimbal zero position
            # When gimbal is at (0,0,0), camera optical axis should point forward (drone +X)
            R_camera_to_drone_base = np.array([
                [0, 0, 1],  # Drone X from Camera Z
                [1, 0, 0],  # Drone Y from Camera X  
                [0, 1, 0]   # Drone Z from Camera Y
            ])
            
            # Step 3: Apply gimbal rotations
            pitch, roll, yaw = self.current_gimbal_attitude
            
            # Create rotation matrices for gimbal movement
            R_pitch = Rotation.from_euler('y', pitch, degrees=False).as_matrix()
            R_roll = Rotation.from_euler('x', roll, degrees=False).as_matrix()  
            R_yaw = Rotation.from_euler('z', yaw, degrees=False).as_matrix()
            
            # Apply gimbal rotations: Yaw first, then Pitch, then Roll
            R_gimbal = R_yaw @ R_pitch @ R_roll
            
            # Step 4: Transform ray direction to drone body frame
            ray_camera = normalized_coords
            ray_drone = R_gimbal @ R_camera_to_drone_base @ ray_camera
            
            # Step 5: Check ray is pointing downward (essential for ground intersection)
            if ray_drone[2] <= 0:
                error_msg = f"Ray not pointing downward (Z={ray_drone[2]:.4f}). Check gimbal angles."
                logger.error(error_msg)
                self.stats['failed_estimations'] += 1
                raise ValueError(error_msg)
            
            # Step 6: Calculate ground intersection in drone body frame
            scale_factor = self.current_height / ray_drone[2]
            ground_point_drone = scale_factor * ray_drone
            
            forward_offset = ground_point_drone[0]   # meters forward from drone
            right_offset = ground_point_drone[1]     # meters right from drone
            
            # Step 7: Transform to world coordinates (ENU)
            cos_heading = np.cos(self.current_heading)
            sin_heading = np.sin(self.current_heading)
            
            # Rotation matrix from drone body to ENU world frame
            east_offset = forward_offset * sin_heading + right_offset * cos_heading
            north_offset = forward_offset * cos_heading - right_offset * sin_heading
            
            # Step 8: Calculate lateral distance and GPS coordinates
            lateral_distance = np.sqrt(east_offset**2 + north_offset**2)
            
            # Convert ENU offsets to GPS coordinate offsets
            lat, lon = self.current_gps
            lat_rad = np.radians(lat)
            
            # More accurate conversion using WGS84 approximation
            meters_per_deg_lat = 111111.0  # approximately constant
            meters_per_deg_lon = 111111.0 * np.cos(lat_rad)
            
            lat_offset_deg = north_offset / meters_per_deg_lat
            lon_offset_deg = east_offset / meters_per_deg_lon
            
            estimated_lat = lat + lat_offset_deg
            estimated_lon = lon + lon_offset_deg
            
            # Validate estimated coordinates
            if not self._validate_gps_coordinates(estimated_lat, estimated_lon):
                logger.warning(f"Estimated GPS coordinates out of range: {estimated_lat}, {estimated_lon}")
            
            self.stats['successful_estimations'] += 1
            
            return {
                "estimated_latitude": estimated_lat,
                "estimated_longitude": estimated_lon,
                "lateral_distance_m": lateral_distance,
                "forward_offset_m": forward_offset,
                "right_offset_m": right_offset,
                "east_offset_m": east_offset,
                "north_offset_m": north_offset,
                "ray_direction_drone": ray_drone.tolist(),
                "ground_point_drone": ground_point_drone.tolist(),
                "pixel_coordinates": [pixel_x, pixel_y],
                "camera_type": self.current_camera_type
            }
            
        except Exception as e:
            self.stats['failed_estimations'] += 1
            logger.error(f"GPS estimation failed for pixel ({pixel_x}, {pixel_y}): {e}")
            raise

    def calculate_lateral_distance(self, bbox_center: Tuple[float, float],
                                 gimbal_attitude_deg: Tuple[float, float, float],
                                 drone_height_m: float,
                                 drone_heading_deg: float = 0.0) -> Dict[str, Any]:
        """
        Legacy interface for backward compatibility.
        
        **DEPRECATED**: Use estimate_target_gps() instead for better error handling
        and consistent state management.
        
        Args:
            bbox_center: (x, y) pixel coordinates
            gimbal_attitude_deg: (pitch, roll, yaw) in degrees
            drone_height_m: Height above ground
            drone_heading_deg: Heading in degrees (0=North, positive=clockwise)
            
        Returns:
            Dictionary with estimation results
        """
        warnings.warn(
            "calculate_lateral_distance() is deprecated. Use estimate_target_gps() instead.",
            DeprecationWarning,
            stacklevel=2
        )
        
        try:
            # Convert inputs to internal format
            self.current_height = drone_height_m
            self.current_heading = np.radians(drone_heading_deg)
            self.current_gimbal_attitude = tuple(np.radians(angle) for angle in gimbal_attitude_deg)
            
            # Use default GPS if not set
            if self.current_gps is None:
                logger.warning("No GPS coordinates set, using default (0, 0)")
                self.current_gps = (0.0, 0.0)
                
            # Call main estimation function
            result = self.estimate_target_gps(bbox_center[0], bbox_center[1])
            
            # Return in expected legacy format
            return {
                "estimated_latitude": result["estimated_latitude"],
                "estimated_longitude": result["estimated_longitude"], 
                "lateral_distance_m": result["lateral_distance_m"],
                "enu_offsets_m": (result["east_offset_m"], result["north_offset_m"]),
                "fwd_right_offsets_m": (result["forward_offset_m"], result["right_offset_m"])
            }
            
        except Exception as e:
            logger.error(f"Legacy GPS calculation failed: {e}")
            # Return safe defaults
            return {
                "estimated_latitude": 0.0,
                "estimated_longitude": 0.0,
                "lateral_distance_m": 0.0,
                "enu_offsets_m": (0.0, 0.0),
                "fwd_right_offsets_m": (0.0, 0.0)
            }

    def _validate_state(self) -> bool:
        """Validate that all required state is set for GPS estimation."""
        required_state = [
            (self.current_gps is not None, "GPS coordinates"),
            (self.current_height is not None, "height"),
            (self.current_heading is not None, "heading"),
            (self.current_gimbal_attitude is not None, "gimbal attitude"),
            (self.current_intrinsic is not None, "camera intrinsics")
        ]
        
        missing = [name for valid, name in required_state if not valid]
        
        if missing:
            logger.error(f"Missing required state for GPS estimation: {', '.join(missing)}")
            return False
            
        return True

    # Simplified GPS estimation - no clustering
    def add_target(self, latitude: float, longitude: float) -> str:
        """
        Legacy method for compatibility - now just validates coordinates.
        
        Args:
            latitude: Target latitude in degrees
            longitude: Target longitude in degrees
            
        Returns:
            Always returns "simple" since no clustering is used
        """
        if not self._validate_gps_coordinates(latitude, longitude):
            logger.warning(f"Invalid target coordinates: {latitude}, {longitude}")
            return "invalid"
        
        logger.debug(f"Target added (no clustering): {latitude:.6f}, {longitude:.6f}")
        return "simple"

    def get_active_clusters(self) -> List[Dict]:
        """
        Return empty list since clustering is disabled.
        
        Returns:
            Empty list for compatibility
        """
        return []

    def get_cluster_count(self) -> int:
        """Return 0 since clustering is disabled."""
        return 0

    def get_stats(self) -> Dict[str, Any]:
        """
        Get GPS manager statistics for monitoring.
        
        Returns:
            Dictionary with GPS manager statistics
        """
        stats = self.stats.copy()
        stats['success_rate'] = (
            stats['successful_estimations'] / max(stats['total_estimations'], 1)
        )
        stats['active_clusters'] = 0  # No clustering
        stats['current_camera_type'] = self.current_camera_type
        return stats

    def cleanup_old_clusters(self, max_age_hours: float = 24.0):
        """
        No-op method for compatibility - no clusters to clean up.
        
        Args:
            max_age_hours: Ignored since no clustering is used
        """
        logger.debug("Cluster cleanup called but no clustering is enabled")


# Alias for backward compatibility
gps_manager = GPSManagerFixed
