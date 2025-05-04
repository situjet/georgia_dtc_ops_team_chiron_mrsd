from typing import Dict, Tuple, List, Optional
import math
import time
import numpy as np

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
        # self.current_intrinsic: Optional[np.ndarray] = None # Assuming 3x3 numpy array

    def set_current_state(self, gps: Tuple[float, float], height: float, heading: float):
        """Set the current state needed for estimation."""
        self.current_gps = gps
        self.current_height = height
        self.current_heading = heading # Ensure heading is in radians as expected by np.cos/sin
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
    

    def estimate_gps_from_bbox(self, bbox):
        if None in (self.current_gps, self.current_height, self.current_heading):
            raise ValueError("State not set")

        u = (bbox[0] + bbox[2]) * 0.5
        v = (bbox[1] + bbox[3]) * 0.5
        dir_c = np.linalg.inv(self.current_intrinsic) @ np.array([u, v, 1.0])

        # Ground point in camera frame (Z_c = +height, nadir assumption)
        ground_c = dir_c * self.current_height        # (x_c, y_c, z_c=height)

        # Rotate camera‑to‑ENU (yaw only)
        c, s = math.cos(self.current_heading), math.sin(self.current_heading)
        enu_x =  ground_c[0]*c - ground_c[1]*s         # east
        enu_y =  ground_c[0]*s + ground_c[1]*c         # north

        # Convert to LLA (flat earth)
        lat, lon = self.current_gps
        lat +=  enu_y / 111111.0
        lon +=  enu_x / (111111.0 * math.cos(math.radians(lat)))
        return lat, lon