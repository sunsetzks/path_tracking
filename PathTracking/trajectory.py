"""
Trajectory class for path tracking with interpolation and projection capabilities.

This module provides a Trajectory class that can:
- Store waypoints with coordinates, angles, and directions
- Perform interpolation between waypoints
- Find nearest projection points for given poses
- Calculate longitudinal and lateral coordinates relative to the path

Author: Assistant
"""

import numpy as np
from typing import List, Tuple, Optional
import math
from scipy.interpolate import interp1d
from scipy.spatial.distance import cdist


class Waypoint:
    """Represents a single waypoint in the trajectory."""
    
    def __init__(self, x: float, y: float, yaw: float, direction: int = 1):
        """
        Initialize a waypoint.
        
        Args:
            x: X coordinate
            y: Y coordinate  
            yaw: Heading angle in radians
            direction: Movement direction (1 for forward, -1 for backward)
        """
        self.x = x
        self.y = y
        self.yaw = yaw
        self.direction = direction


class Trajectory:
    """
    A trajectory class that supports interpolation and projection operations.
    
    The trajectory consists of waypoints with coordinates, heading angles, and directions.
    It provides methods for interpolation and finding projection points.
    """
    
    def __init__(self, waypoints: List[Waypoint] = None):
        """
        Initialize the trajectory.
        
        Args:
            waypoints: List of Waypoint objects defining the trajectory
        """
        self.waypoints = waypoints if waypoints is not None else []
        self._s_values = None  # Cumulative distances along the path
        self._interpolators = {}  # Cached interpolation functions
        self._update_path_parameters()
    
    def add_waypoint(self, x: float, y: float, yaw: float, direction: int = 1):
        """
        Add a waypoint to the trajectory.
        
        Args:
            x: X coordinate
            y: Y coordinate
            yaw: Heading angle in radians
            direction: Movement direction (1 for forward, -1 for backward)
        """
        waypoint = Waypoint(x, y, yaw, direction)
        self.waypoints.append(waypoint)
        self._update_path_parameters()
    
    def add_waypoints_from_arrays(self, x_coords: List[float], y_coords: List[float], 
                                  yaw_angles: List[float], directions: List[int] = None):
        """
        Add multiple waypoints from coordinate arrays.
        
        Args:
            x_coords: List of X coordinates
            y_coords: List of Y coordinates
            yaw_angles: List of heading angles in radians
            directions: List of movement directions (default: all forward)
        """
        if directions is None:
            directions = [1] * len(x_coords)
        
        if not (len(x_coords) == len(y_coords) == len(yaw_angles) == len(directions)):
            raise ValueError("All input arrays must have the same length")
        
        for x, y, yaw, direction in zip(x_coords, y_coords, yaw_angles, directions):
            self.add_waypoint(x, y, yaw, direction)
    
    def _update_path_parameters(self):
        """Update internal path parameters after waypoint changes."""
        if len(self.waypoints) < 2:
            self._s_values = None
            self._interpolators = {}
            return
        
        # Calculate cumulative distances along the path
        self._s_values = [0.0]
        for i in range(1, len(self.waypoints)):
            prev_wp = self.waypoints[i-1]
            curr_wp = self.waypoints[i]
            distance = math.sqrt((curr_wp.x - prev_wp.x)**2 + (curr_wp.y - prev_wp.y)**2)
            self._s_values.append(self._s_values[-1] + distance)
        
        # Create interpolation functions
        if len(self.waypoints) >= 2:
            x_coords = [wp.x for wp in self.waypoints]
            y_coords = [wp.y for wp in self.waypoints]
            yaw_angles = [wp.yaw for wp in self.waypoints]
            directions = [wp.direction for wp in self.waypoints]
            
            # Handle angle wrapping for yaw interpolation
            yaw_unwrapped = self._unwrap_angles(yaw_angles)
            
            self._interpolators = {
                'x': interp1d(self._s_values, x_coords, kind='linear', 
                            bounds_error=False, fill_value='extrapolate'),
                'y': interp1d(self._s_values, y_coords, kind='linear',
                            bounds_error=False, fill_value='extrapolate'),
                'yaw': interp1d(self._s_values, yaw_unwrapped, kind='linear',
                              bounds_error=False, fill_value='extrapolate'),
                'direction': interp1d(self._s_values, directions, kind='nearest',
                                    bounds_error=False, fill_value='extrapolate')
            }
    
    def _unwrap_angles(self, angles: List[float]) -> List[float]:
        """Unwrap angles to avoid discontinuities in interpolation."""
        if len(angles) <= 1:
            return angles
        
        unwrapped = [angles[0]]
        for i in range(1, len(angles)):
            diff = angles[i] - angles[i-1]
            # Wrap difference to [-pi, pi]
            while diff > math.pi:
                diff -= 2 * math.pi
            while diff < -math.pi:
                diff += 2 * math.pi
            unwrapped.append(unwrapped[-1] + diff)
        
        return unwrapped
    
    def interpolate_at_distance(self, s: float) -> Tuple[float, float, float, int]:
        """
        Interpolate trajectory at a given cumulative distance.
        
        Args:
            s: Cumulative distance along the trajectory
            
        Returns:
            Tuple of (x, y, yaw, direction)
        """
        if not self._interpolators:
            raise ValueError("Trajectory must have at least 2 waypoints for interpolation")
        
        x = float(self._interpolators['x'](s))
        y = float(self._interpolators['y'](s))
        yaw = float(self._interpolators['yaw'](s))
        direction = int(self._interpolators['direction'](s))
        
        # Wrap yaw angle to [-pi, pi]
        yaw = math.atan2(math.sin(yaw), math.cos(yaw))
        
        return x, y, yaw, direction
    
    def find_nearest_point(self, pose_x: float, pose_y: float) -> Tuple[float, float, float, int, float]:
        """
        Find the nearest point on the trajectory to a given pose.
        
        Args:
            pose_x: X coordinate of the query pose
            pose_y: Y coordinate of the query pose
            
        Returns:
            Tuple of (nearest_x, nearest_y, nearest_yaw, nearest_direction, distance_along_path)
        """
        if not self._interpolators:
            raise ValueError("Trajectory must have at least 2 waypoints")
        
        # Sample points along the trajectory for nearest point search
        total_length = self._s_values[-1]
        num_samples = max(100, len(self.waypoints) * 10)
        s_samples = np.linspace(0, total_length, num_samples)
        
        # Get interpolated points
        sampled_points = []
        for s in s_samples:
            x, y, yaw, direction = self.interpolate_at_distance(s)
            sampled_points.append([x, y])
        
        sampled_points = np.array(sampled_points)
        
        # Find nearest point
        query_point = np.array([[pose_x, pose_y]])
        distances = cdist(query_point, sampled_points)[0]
        nearest_idx = np.argmin(distances)
        nearest_s = s_samples[nearest_idx]
        
        # Refine the search using local optimization
        refined_s = self._refine_nearest_point(pose_x, pose_y, nearest_s, total_length)
        
        nearest_x, nearest_y, nearest_yaw, nearest_direction = self.interpolate_at_distance(refined_s)
        
        return nearest_x, nearest_y, nearest_yaw, nearest_direction, refined_s
    
    def _refine_nearest_point(self, pose_x: float, pose_y: float, initial_s: float, 
                             total_length: float, tolerance: float = 0.01) -> float:
        """
        Refine nearest point search using golden section search.
        
        Args:
            pose_x: X coordinate of query pose
            pose_y: Y coordinate of query pose
            initial_s: Initial guess for nearest point distance
            total_length: Total trajectory length
            tolerance: Search tolerance
            
        Returns:
            Refined distance along path to nearest point
        """
        def distance_squared(s):
            s = max(0, min(s, total_length))  # Clamp to valid range
            x, y, _, _ = self.interpolate_at_distance(s)
            return (x - pose_x)**2 + (y - pose_y)**2
        
        # Golden section search
        phi = (1 + math.sqrt(5)) / 2  # Golden ratio
        
        # Search window around initial guess
        window_size = min(total_length * 0.1, 10.0)  # 10% of path or 10 units
        left = max(0, initial_s - window_size)
        right = min(total_length, initial_s + window_size)
        
        # Golden section search
        while (right - left) > tolerance:
            s1 = right - (right - left) / phi
            s2 = left + (right - left) / phi
            
            if distance_squared(s1) > distance_squared(s2):
                left = s1
            else:
                right = s2
        
        return (left + right) / 2
    
    def get_frenet_coordinates(self, pose_x: float, pose_y: float) -> Tuple[float, float]:
        """
        Get Frenet coordinates (longitudinal and lateral) for a given pose.
        
        Args:
            pose_x: X coordinate of the query pose
            pose_y: Y coordinate of the query pose
            
        Returns:
            Tuple of (longitudinal_distance, lateral_distance)
            - longitudinal_distance: Distance along the trajectory (s-coordinate)
            - lateral_distance: Signed distance from trajectory (d-coordinate, positive = left)
        """
        nearest_x, nearest_y, nearest_yaw, _, s_coordinate = self.find_nearest_point(pose_x, pose_y)
        
        # Calculate lateral distance (signed)
        # Vector from nearest point to query pose
        dx = pose_x - nearest_x
        dy = pose_y - nearest_y
        
        # Normal vector to the trajectory (pointing left)
        normal_x = -math.sin(nearest_yaw)
        normal_y = math.cos(nearest_yaw)
        
        # Project displacement onto normal vector
        lateral_distance = dx * normal_x + dy * normal_y
        
        return s_coordinate, lateral_distance
    
    def get_trajectory_length(self) -> float:
        """Get the total length of the trajectory."""
        if self._s_values is None:
            return 0.0
        return self._s_values[-1]
    
    def get_waypoint_distances(self) -> List[float]:
        """Get the cumulative distances to each waypoint."""
        if self._s_values is None:
            return []
        return self._s_values.copy()
    
    def clear(self):
        """Clear all waypoints from the trajectory."""
        self.waypoints = []
        self._s_values = None
        self._interpolators = {}
    
    def __len__(self) -> int:
        """Return the number of waypoints in the trajectory."""
        return len(self.waypoints)
    
    def __str__(self) -> str:
        """String representation of the trajectory."""
        return f"Trajectory with {len(self.waypoints)} waypoints, length: {self.get_trajectory_length():.2f}"


# Example usage and testing
if __name__ == "__main__":
    # Create a simple circular trajectory for testing
    trajectory = Trajectory()
    
    # Add waypoints for a quarter circle
    n_points = 20
    radius = 10.0
    for i in range(n_points):
        angle = i * math.pi / 2 / (n_points - 1)  # Quarter circle
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        yaw = angle + math.pi / 2  # Tangent direction
        trajectory.add_waypoint(x, y, yaw, direction=1)
    
    print(f"Created {trajectory}")
    
    # Test interpolation
    test_s = trajectory.get_trajectory_length() / 2
    x, y, yaw, direction = trajectory.interpolate_at_distance(test_s)
    print(f"Interpolated point at s={test_s:.2f}: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")
    
    # Test nearest point finding
    query_x, query_y = 5.0, 8.0
    nearest_x, nearest_y, nearest_yaw, nearest_dir, s_coord = trajectory.find_nearest_point(query_x, query_y)
    print(f"Nearest point to ({query_x}, {query_y}): ({nearest_x:.2f}, {nearest_y:.2f}), s={s_coord:.2f}")
    
    # Test Frenet coordinates
    s_frenet, d_frenet = trajectory.get_frenet_coordinates(query_x, query_y)
    print(f"Frenet coordinates: s={s_frenet:.2f}, d={d_frenet:.2f}") 