from typing import Dict, Tuple, List, Optional
import math
import time
import numpy as np
from typing import Tuple, Dict, Any
from scipy.spatial.transform import Rotation # For robust Euler angle transformations

class gps_manager:
    def __init__(self, buffer_size: int = 30):
        # List of target clusters, each containing:
        # {
        #   "id": "0", "1", "2", etc.,
        #   "center": (latitude, longitude),
        #   "targets": [(latitude, longitude, timestamp), ...] 
        # }
        self.target_clusters = []
        self.min_distance_threshold = 5.0  # meters
        self.buffer_size = buffer_size  # Maximum number of targets to keep per cluster
        self.current_intrinsic = np.array([[475.0, 0.0, 320.0], [0.0, 505.0, 256.0], [0.0, 0.0, 1.0]])

        # Add state attributes required for estimation
        self.current_gps: Optional[Tuple[float, float]] = None
        self.current_height: Optional[float] = None
        self.current_heading: Optional[float] = None # Assuming heading is in radians relative to North/East? Needs clarification.


    def set_current_state(self, gps: Tuple[float, float], height: float, heading: float, gimbal_attitude: Tuple[float, float, float]):
        """Set the current state needed for estimation."""
        self.current_gps = gps
        self.current_height = height
        self.current_heading = heading # Ensure heading is in radians as expected by np.cos/sin
        self.current_gimbal_attitude = gimbal_attitude
        self.current_intrinsic = self.current_intrinsic

    def add_target(self, latitude: float, longitude: float) -> str:
        """
        Add a new target to the GPS manager.
        
        Args:
            latitude: Target latitude
            longitude: Target longitude
            
        Returns:
            str: ID of the cluster the target was added to
        """
        new_target = (latitude, longitude, time.time())
        
        # Check if this is the first target
        if not self.target_clusters:
            self._create_new_cluster(new_target)
            return "0"
            
        # Find closest cluster and its distance
        closest_idx, min_distance = self._find_closest_cluster(latitude, longitude)
        
        # If distance is greater than threshold, create a new cluster
        if min_distance > self.min_distance_threshold:
            self._create_new_cluster(new_target)
            # Re-rank clusters by density
            self._rank_clusters_by_density()
            # Return the ID of the newly created cluster
            for cluster in self.target_clusters:
                if new_target in cluster["targets"]:
                    return cluster["id"]
        else:
            # Add to existing cluster and update center
            self._add_to_cluster(closest_idx, new_target)
            # Re-rank clusters by density
            self._rank_clusters_by_density()
            # Return the ID of the updated cluster
            for cluster in self.target_clusters:
                if new_target in cluster["targets"]:
                    return cluster["id"]
        
        # This should not happen, but just in case
        return "unknown"
    
    def _create_new_cluster(self, target: Tuple[float, float, float]):
        """Create a new cluster with the given target."""
        # Temporarily assign a placeholder ID
        new_idx = len(self.target_clusters)
        self.target_clusters.append({
            "id": f"{new_idx}",
            "center": (target[0], target[1]),
            "targets": [target]
        })
    
    def _add_to_cluster(self, cluster_idx: int, target: Tuple[float, float, float]):
        """Add target to an existing cluster and update its center."""
        cluster = self.target_clusters[cluster_idx]
        cluster["targets"].append(target)
        
        # Maintain buffer size by keeping only the most recent targets
        if len(cluster["targets"]) > self.buffer_size:
            # Sort by timestamp (which is the third element in each target tuple)
            cluster["targets"].sort(key=lambda t: t[2])
            # Keep only the most recent ones
            cluster["targets"] = cluster["targets"][-self.buffer_size:]
        
        # Recalculate center as moving average of buffered targets
        latitudes = [t[0] for t in cluster["targets"]]
        longitudes = [t[1] for t in cluster["targets"]]
        
        new_center = (
            sum(latitudes) / len(latitudes),
            sum(longitudes) / len(longitudes)
        )
        
        cluster["center"] = new_center
    
    def _rank_clusters_by_density(self):
        """
        Rank clusters by density (number of targets in cluster).
        Most dense cluster gets id "0", second most dense gets id "1", etc.
        """
        # Sort clusters by number of targets (in descending order)
        sorted_clusters = sorted(
            self.target_clusters,
            key=lambda c: len(c["targets"]),
            reverse=True
        )
        
        # Update IDs based on new ranking
        for i, cluster in enumerate(sorted_clusters):
            cluster["id"] = f"{i}"
        
        # Update the target_clusters list with the newly sorted and re-identified clusters
        self.target_clusters = sorted_clusters
    
    def _find_closest_cluster(self, latitude: float, longitude: float) -> Tuple[int, float]:
        """
        Find the closest cluster to a given coordinate.
        
        Returns:
            Tuple containing (cluster_index, distance_in_meters)
        """
        min_distance = float('inf')
        closest_idx = -1
        
        for idx, cluster in enumerate(self.target_clusters):
            center_lat, center_lon = cluster["center"]
            distance = self._calculate_distance(
                latitude, longitude, center_lat, center_lon
            )
            
            if distance < min_distance:
                min_distance = distance
                closest_idx = idx
                
        return closest_idx, min_distance
    
    def _calculate_distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """
        Calculate the Haversine distance between two points in meters.
        """
        # Earth radius in meters
        R = 6371000.0
        
        # Convert latitude and longitude from degrees to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        
        # Haversine formula
        dlon = lon2_rad - lon1_rad
        dlat = lat2_rad - lat1_rad
        a = math.sin(dlat/2)**2 + math.cos(lat1_rad) * math.cos(lat2_rad) * math.sin(dlon/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1-a))
        distance = R * c
        
        return distance
    
    def get_all_clusters(self) -> List[Dict]:
        """Return all target clusters."""
        return self.target_clusters
    
    def get_active_clusters(self) -> List[Dict]:
        """Return all active target clusters.
        
        Currently returns all clusters, but could be modified to filter
        based on recency or other activity criteria.
        """
        return self.target_clusters
    
    def get_cluster_count(self) -> int:
        """Return the number of clusters."""
        return len(self.target_clusters)
        
    def get_cluster_by_id(self, cluster_id: str) -> Optional[Dict]:
        """
        Get a cluster by its ID.
        
        Args:
            cluster_id: ID of the cluster to retrieve
            
        Returns:
            Dict or None: The cluster if found, None otherwise
        """
        for cluster in self.target_clusters:
            if cluster["id"] == cluster_id:
                return cluster
        return None

    def calculate_lateral_distance(
        self,
        bbox_center: Tuple[float, float],
        gimbal_attitude_deg: Tuple[float, float, float],
        drone_height_m: float,
        drone_heading_deg: float = 0.0 # Default heading to North
        ) -> Dict[str, Any]:
        """
        Estimates GPS coordinates and calculates lateral distance from bounding box center.
        This is a convenience wrapper around estimate_gps_from_bbox_detailed.

        Args:
            bbox_center: (x, y) pixel coordinates of the bounding box center.
            gimbal_attitude_deg: (pitch, roll, yaw) of the gimbal in degrees.
            drone_height_m: Height of the drone above ground in meters.
            drone_heading_deg: Heading of the drone in degrees (0 is North, positive clockwise/East).

        Returns:
            A dictionary containing estimated latitude, longitude, lateral distance,
            ENU offsets, and forward/right offsets.
        """
        # Set the current state of the estimator based on inputs
        self.current_gimbal_attitude = (
            np.radians(gimbal_attitude_deg[0]), # Convert pitch to radians
            np.radians(gimbal_attitude_deg[1]), # Convert roll to radians
            np.radians(gimbal_attitude_deg[2])  # Convert yaw to radians
        )
        self.current_height = drone_height_m
        self.current_heading = np.radians(drone_heading_deg) # Convert heading to radians

        # Set default GPS if not already set (though not strictly needed for lateral distance if GPS output is ignored)
        if self.current_gps is None: self.current_gps = (0.0,0.0)
        # Intrinsic matrix is assumed to be set during initialization or externally

        # Call the detailed estimation function
        details = self.estimate_gps_from_bbox_detailed(bbox_center)
        return {
            "estimated_latitude": details["estimated_latitude"],
            "estimated_longitude": details["estimated_longitude"],
            "lateral_distance_m": details["lateral_distance_m"],
            "enu_offsets_m": details["enu_offsets_m"],
            "fwd_right_offsets_m": details["fwd_right_offsets_m"]
            # Add other details if needed from the 'details' dictionary
        }


    def estimate_gps_from_bbox_detailed(self, bbox_center: Tuple[float, float]) -> Dict[str, Any]:
        """
        Performs the core calculation for GPS estimation and related distances.

        Args:
            bbox_center: (x, y) pixel coordinates of the bounding box center.

        Returns:
            A dictionary with detailed calculation results.
        """
        # Check for None on essential attributes that must be set
        if self.current_gps is None or \
           self.current_height is None or \
           self.current_heading is None or \
           self.current_intrinsic is None or \
           self.current_gimbal_attitude is None:
            raise ValueError("Current state (GPS, height, heading, intrinsic, gimbal) must be set and not None.")

        center_x, center_y = bbox_center
        pixel_coords = np.array([center_x, center_y, 1.0]) # Homogeneous pixel coordinates

        # Invert the intrinsic matrix
        try:
            K_inv = np.linalg.inv(self.current_intrinsic)
        except np.linalg.LinAlgError:
            raise ValueError("Cannot invert intrinsic matrix K. It might be singular.")

        # Convert pixel coordinates to normalized camera coordinates
        # normalized_coords are in Camera Optical Frame: [X_cam_right, Y_cam_down, Z_cam_fwd]
        normalized_coords = K_inv @ pixel_coords # [xc, yc, zc]

        gimbal_pitch_input, gimbal_roll_input, gimbal_yaw_input = self.current_gimbal_attitude

        # Define the base transformation: Camera Optical Frame -> Drone Body Frame (at gimbal 0,0,0)
        # Drone Body Frame: X-axis forward, Y-axis right, Z-axis down (NED-like local frame)
        # Camera Optical Frame: X_cam-axis right in image, Y_cam-axis down in image, Z_cam-axis forward (optical axis)
        # This specific base alignment assumes:
        #   At gimbal (0,0,0), Camera Y (image down) aligns with Drone X (forward).
        #   At gimbal (0,0,0), Camera Z (optical fwd) aligns with Drone Y (right).
        #   At gimbal (0,0,0), Camera X (image right) aligns with Drone Z (down).
        # The columns of R_cam_optical_to_drone_body_at_0 are CamX, CamY, CamZ axes expressed in Drone frame coords:
        #   Col1 (CamX_right_in_Drone): [0,0,1]^T (aligns with DroneZ)
        #   Col2 (CamY_down_in_Drone):  [1,0,0]^T (aligns with DroneX)
        #   Col3 (CamZ_fwd_in_Drone):   [0,1,0]^T (aligns with DroneY)
        R_cam_optical_to_drone_body_at_0 = np.array([
            [0, 1, 0],  # DroneX component from CamY_down
            [0, 0, 1],  # DroneY component from CamZ_fwd
            [1, 0, 0]   # DroneZ component from CamX_right
        ])

        # Calculate gimbal rotation matrix using ZYX intrinsic Euler angle sequence
        # Note: -gimbal_pitch_input is used because a positive input pitch typically means camera tilts upwards,
        # while a positive rotation around the Y-axis (if Y is right) in a right-handed system would tilt Z-fwd downwards.
        # This negation aligns intuitive pitch direction with the mathematical rotation.
        R_gimbal_movement = Rotation.from_euler(
            'zyx', # Intrinsic: Yaw around Z_base, then Pitch around new Y, then Roll around new X
            [gimbal_yaw_input, -gimbal_pitch_input, gimbal_roll_input],
            degrees=False # Input gimbal attitudes are already in radians here
        ).as_matrix()

        # Combine base alignment with gimbal movement to get total rotation from Camera Optical to Drone Body frame
        R_total = R_cam_optical_to_drone_body_at_0 @ R_gimbal_movement
        
        # Transform normalized camera coordinates to a ray direction in the Drone Body Frame
        # ray_direction_drone_body is the ray vector [X_fwd, Y_right, Z_down]
        ray_direction_drone_body = R_total @ normalized_coords

        # Check if the ray is pointing downwards (positive Z component in Z-down frame)
        if ray_direction_drone_body[2] < 1e-9: # Allow for minor floating point inaccuracies
            raise ValueError(
                f"Ray is not pointing downwards relative to drone (Z-component: {ray_direction_drone_body[2]:.4f}). "
                "Check gimbal angles or coordinate system definitions."
            )

        # Calculate the scale factor to project the ray onto the ground plane
        # Ground plane is at Z = self.current_height in the drone's Z-down frame
        distance_scale = self.current_height / ray_direction_drone_body[2]

        # Calculate the intersection point of the ray with the ground in the Drone Body Frame
        # [Forward_offset, Right_offset, Height_check (should be self.current_height)]
        ground_point_drone_body = distance_scale * ray_direction_drone_body

        fwd_offset = ground_point_drone_body[0] # Offset along drone's forward X-axis
        right_offset = ground_point_drone_body[1] # Offset along drone's right Y-axis

        # Convert drone body frame Fwd/Right offsets to ENU (East, North, Up) offsets
        # self.current_heading = 0 means drone X-axis (Forward) points North.
        # self.current_heading = pi/2 (90 deg) means drone X-axis (Forward) points East.
        enu_x = fwd_offset * np.sin(self.current_heading) + right_offset * np.cos(self.current_heading)  # East offset
        enu_y = fwd_offset * np.cos(self.current_heading) - right_offset * np.sin(self.current_heading)  # North offset

        # Lateral distance is the magnitude of the horizontal displacement
        lateral_distance_m = np.sqrt(enu_x**2 + enu_y**2)
        # This is equivalent to np.sqrt(fwd_offset**2 + right_offset**2) as heading is a 2D rotation

        # GPS coordinate calculation (can be omitted if only lateral distance is needed)
        current_lat, current_lon = self.current_gps
        # Handle if self.current_gps is a NumPy array
        if isinstance(self.current_gps, np.ndarray) and self.current_gps.size >= 2:
             current_lat, current_lon = self.current_gps[0], self.current_gps[1]

        lat_rad = np.radians(current_lat)
        meters_per_deg_lon = 111111.0 * np.cos(lat_rad) # Approx meters per degree longitude
        
        estimated_lat, estimated_lon = None, None # Initialize
        if abs(meters_per_deg_lon) < 1e-9: # Avoid division by zero near the poles
            # If at poles, longitude offset is problematic. Latitude offset can still be calculated.
            if self.current_gps is not None: # Check if GPS info is available
                 lat_offset = enu_y / 111111.0 # Approx meters per degree latitude
                 estimated_lat = current_lat + lat_offset
                 # estimated_lon remains None or could be set to current_lon if no change is assumed
                 estimated_lon = current_lon # Or raise error as before
            # else: # No GPS info, so cannot estimate
        else:
            lat_offset = enu_y / 111111.0
            lon_offset = enu_x / meters_per_deg_lon
            estimated_lat = current_lat + lat_offset
            estimated_lon = current_lon + lon_offset
            
        return {
            "estimated_latitude": estimated_lat,
            "estimated_longitude": estimated_lon,
            "lateral_distance_m": lateral_distance_m,
            "enu_offsets_m": (enu_x, enu_y),
            "fwd_right_offsets_m": (fwd_offset, right_offset),
            "ray_direction_drone_body": ray_direction_drone_body.tolist(), # For debugging
            "R_total": R_total.tolist(), # For debugging
            "normalized_coords": normalized_coords.tolist() # For debugging
        }