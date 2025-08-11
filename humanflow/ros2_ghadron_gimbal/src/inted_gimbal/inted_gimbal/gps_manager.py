"""
Fixed GPS Manager with Standardized Coordinate Systems
====================================================

This module provides a corrected GPS estimation algorithm that addresses the
coordinate system inconsistencies found in the original implementation.

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
import numpy as np
from scipy.spatial.transform import Rotation


class GPSManagerFixed:
    def __init__(self, buffer_size: int = 30):
        """
        Initialize GPS Manager with fixed coordinate system handling.
        
        Args:
            buffer_size: Maximum number of targets to keep per cluster
        """
        # Target clustering
        self.target_clusters = []
        self.min_distance_threshold = 5.0  # meters
        self.buffer_size = buffer_size
        
        # Camera intrinsics (default to EO camera)
        self.current_intrinsic = np.array([
            [475.0, 0.0, 320.0],
            [0.0, 505.0, 256.0],
            [0.0, 0.0, 1.0]
        ])
        
        # Current state for GPS estimation
        self.current_gps: Optional[Tuple[float, float]] = None
        self.current_height: Optional[float] = None
        self.current_heading: Optional[float] = None  # radians, 0=North, +ve=clockwise
        self.current_gimbal_attitude: Optional[Tuple[float, float, float]] = None  # (pitch, roll, yaw) in radians

    def set_camera_intrinsics(self, fx: float, fy: float, cx: float, cy: float):
        """Set camera intrinsic parameters."""
        self.current_intrinsic = np.array([
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0]
        ])

    def set_camera_type(self, camera_type: str):
        """Set camera type and update intrinsics accordingly."""
        if camera_type.upper() == "EO":
            self.set_camera_intrinsics(475.0, 505.0, 320.0, 256.0)
        elif camera_type.upper() == "IR":
            self.set_camera_intrinsics(2267.0, 1593.0, 640.0, 360.0)
        else:
            raise ValueError(f"Unknown camera type: {camera_type}")

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
        self.current_gps = gps
        self.current_height = height
        self.current_heading = heading
        self.current_gimbal_attitude = gimbal_attitude

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
        # Validate inputs
        if not self._validate_state():
            raise ValueError("GPS estimation state not properly initialized")
            
        # Step 1: Convert pixel to normalized camera coordinates
        pixel_coords = np.array([pixel_x, pixel_y, 1.0])
        K_inv = np.linalg.inv(self.current_intrinsic)
        normalized_coords = K_inv @ pixel_coords  # [x_norm, y_norm, 1.0]
        
        # Step 2: Define camera-to-drone transformation at gimbal zero position
        # When gimbal is at (0,0,0), camera optical axis should point forward (drone +X)
        # This transformation assumes:
        # - Camera Z (optical forward) → Drone X (forward)
        # - Camera X (image right) → Drone Y (right) 
        # - Camera Y (image down) → Drone Z (down)
        R_camera_to_drone_base = np.array([
            [0, 0, 1],  # Drone X from Camera Z
            [1, 0, 0],  # Drone Y from Camera X  
            [0, 1, 0]   # Drone Z from Camera Y
        ])
        
        # Step 3: Apply gimbal rotations
        pitch, roll, yaw = self.current_gimbal_attitude
        
        # Create rotation matrices for gimbal movement
        # Note: We use the convention where positive rotations follow right-hand rule
        R_pitch = Rotation.from_euler('y', pitch, degrees=False).as_matrix()
        R_roll = Rotation.from_euler('x', roll, degrees=False).as_matrix()  
        R_yaw = Rotation.from_euler('z', yaw, degrees=False).as_matrix()
        
        # Apply gimbal rotations: Yaw first (around vertical), then Pitch, then Roll
        # This order matches typical gimbal mechanical construction
        R_gimbal = R_yaw @ R_pitch @ R_roll
        
        # Step 4: Transform ray direction to drone body frame
        ray_camera = normalized_coords
        ray_drone = R_gimbal @ R_camera_to_drone_base @ ray_camera
        
        # Step 5: Check ray is pointing downward (essential for ground intersection)
        if ray_drone[2] <= 0:
            raise ValueError(f"Ray not pointing downward (Z={ray_drone[2]:.4f}). Check gimbal angles.")
        
        # Step 6: Calculate ground intersection in drone body frame
        scale_factor = self.current_height / ray_drone[2]
        ground_point_drone = scale_factor * ray_drone
        
        forward_offset = ground_point_drone[0]   # meters forward from drone
        right_offset = ground_point_drone[1]     # meters right from drone
        
        # Step 7: Transform to world coordinates (ENU)
        # Convert drone body offsets to East/North using drone heading
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
        
        # Approximate conversion (accurate for small offsets)
        meters_per_deg_lat = 111111.0  # approximately constant
        meters_per_deg_lon = 111111.0 * np.cos(lat_rad)
        
        lat_offset_deg = north_offset / meters_per_deg_lat
        lon_offset_deg = east_offset / meters_per_deg_lon
        
        estimated_lat = lat + lat_offset_deg
        estimated_lon = lon + lon_offset_deg
        
        return {
            "estimated_latitude": estimated_lat,
            "estimated_longitude": estimated_lon,
            "lateral_distance_m": lateral_distance,
            "forward_offset_m": forward_offset,
            "right_offset_m": right_offset,
            "east_offset_m": east_offset,
            "north_offset_m": north_offset,
            "ray_direction_drone": ray_drone.tolist(),
            "ground_point_drone": ground_point_drone.tolist()
        }

    def calculate_lateral_distance(self, bbox_center: Tuple[float, float],
                                 gimbal_attitude_deg: Tuple[float, float, float],
                                 drone_height_m: float,
                                 drone_heading_deg: float = 0.0) -> Dict[str, Any]:
        """
        Legacy interface for backward compatibility.
        
        Args:
            bbox_center: (x, y) pixel coordinates
            gimbal_attitude_deg: (pitch, roll, yaw) in degrees
            drone_height_m: Height above ground
            drone_heading_deg: Heading in degrees (0=North, positive=clockwise)
            
        Returns:
            Dictionary with estimation results
        """
        # Convert inputs to internal format
        self.current_height = drone_height_m
        self.current_heading = np.radians(drone_heading_deg)
        self.current_gimbal_attitude = tuple(np.radians(angle) for angle in gimbal_attitude_deg)
        
        # Use default GPS if not set
        if self.current_gps is None:
            self.current_gps = (0.0, 0.0)
            
        # Call main estimation function
        result = self.estimate_target_gps(bbox_center[0], bbox_center[1])
        
        # Return in expected format
        return {
            "estimated_latitude": result["estimated_latitude"],
            "estimated_longitude": result["estimated_longitude"], 
            "lateral_distance_m": result["lateral_distance_m"],
            "enu_offsets_m": (result["east_offset_m"], result["north_offset_m"]),
            "fwd_right_offsets_m": (result["forward_offset_m"], result["right_offset_m"])
        }

    def _validate_state(self) -> bool:
        """Validate that all required state is set for GPS estimation."""
        return all([
            self.current_gps is not None,
            self.current_height is not None,
            self.current_heading is not None,
            self.current_gimbal_attitude is not None,
            self.current_intrinsic is not None
        ])

    # Target clustering methods (unchanged from original)
    def add_target(self, latitude: float, longitude: float) -> str:
        """Add a target to GPS manager and return cluster ID."""
        new_target = (latitude, longitude, time.time())
        
        if not self.target_clusters:
            self._create_new_cluster(new_target)
            return "0"
            
        closest_idx, min_distance = self._find_closest_cluster(latitude, longitude)
        
        if min_distance > self.min_distance_threshold:
            self._create_new_cluster(new_target)
            self._rank_clusters_by_density()
            for cluster in self.target_clusters:
                if new_target in cluster["targets"]:
                    return cluster["id"]
        else:
            self._add_to_cluster(closest_idx, new_target)
            self._rank_clusters_by_density()
            for cluster in self.target_clusters:
                if new_target in cluster["targets"]:
                    return cluster["id"]
        
        return "unknown"

    def _create_new_cluster(self, target: Tuple[float, float, float]):
        """Create a new cluster with the given target."""
        new_idx = len(self.target_clusters)
        self.target_clusters.append({
            "id": f"{new_idx}",
            "center": (target[0], target[1]),
            "targets": [target]
        })

    def _add_to_cluster(self, cluster_idx: int, target: Tuple[float, float, float]):
        """Add target to existing cluster and update center."""
        cluster = self.target_clusters[cluster_idx]
        cluster["targets"].append(target)
        
        if len(cluster["targets"]) > self.buffer_size:
            cluster["targets"].sort(key=lambda t: t[2])
            cluster["targets"] = cluster["targets"][-self.buffer_size:]
        
        latitudes = [t[0] for t in cluster["targets"]]
        longitudes = [t[1] for t in cluster["targets"]]
        
        cluster["center"] = (
            sum(latitudes) / len(latitudes),
            sum(longitudes) / len(longitudes)
        )

    def _rank_clusters_by_density(self):
        """Rank clusters by target count."""
        self.target_clusters.sort(key=lambda c: len(c["targets"]), reverse=True)
        for i, cluster in enumerate(self.target_clusters):
            cluster["id"] = f"{i}"

    def _find_closest_cluster(self, latitude: float, longitude: float) -> Tuple[int, float]:
        """Find closest cluster to given coordinates."""
        min_distance = float('inf')
        closest_idx = -1
        
        for idx, cluster in enumerate(self.target_clusters):
            center_lat, center_lon = cluster["center"]
            distance = self._calculate_haversine_distance(
                latitude, longitude, center_lat, center_lon
            )
            
            if distance < min_distance:
                min_distance = distance
                closest_idx = idx
                
        return closest_idx, min_distance

    def _calculate_haversine_distance(self, lat1: float, lon1: float, 
                                    lat2: float, lon2: float) -> float:
        """Calculate Haversine distance between two points in meters."""
        R = 6371000.0  # Earth radius in meters
        
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        dlon = lon2_rad - lon1_rad
        dlat = lat2_rad - lat1_rad
        a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        
        return R * c

    def get_active_clusters(self) -> List[Dict]:
        """Return all active target clusters."""
        return self.target_clusters

    def get_cluster_count(self) -> int:
        """Return number of clusters."""
        return len(self.target_clusters)


# Alias for backward compatibility
gps_manager = GPSManagerFixed