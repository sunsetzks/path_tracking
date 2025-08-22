"""
Utility functions for Foxglove channels

This module provides helper functions and utilities for creating and working
with different types of Foxglove channels and their data structures.

Features:
- Transform utilities (TF helpers)
- Grid/map creation helpers  
- Point cloud utilities
- Primitive creation helpers
- Data conversion utilities

Author: Generated for path_tracking project
Date: 2025-01-27
"""

import math
import time
import numpy as np
from typing import List, Tuple, Optional, Union, Any
from enum import Enum

# Import Foxglove schemas
try:
    from foxglove.schemas import (
        # Transforms
        FrameTransform, Vector3, Vector2, Quaternion, Timestamp,
        # Primitives
        CubePrimitive, SpherePrimitive, LinePrimitive, ArrowPrimitive,
        Color, Point3, Pose,
        # Advanced types
        Grid, PointCloud, LaserScan, PackedElementField, PackedElementFieldNumericType
    )
    FOXGLOVE_AVAILABLE = True
except ImportError:
    print("Warning: Foxglove SDK not available")
    FOXGLOVE_AVAILABLE = False


class GridType(Enum):
    """Types of grid data"""
    OCCUPANCY = "occupancy"
    COST = "cost"
    HEIGHT = "height"
    INTENSITY = "intensity"


class TransformUtils:
    """Utilities for working with coordinate transformations"""
    
    @staticmethod
    def create_identity_transform(parent_frame: str, child_frame: str, 
                                timestamp: Optional[float] = None) -> FrameTransform:
        """Create an identity transform between two frames"""
        if not FOXGLOVE_AVAILABLE:
            raise ImportError("Foxglove SDK not available")
        
        ts = Timestamp(sec=int(timestamp or time.time()), nsec=0)
        return FrameTransform(
            timestamp=ts,
            parent_frame_id=parent_frame,
            child_frame_id=child_frame,
            translation=Vector3(x=0.0, y=0.0, z=0.0),
            rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
    
    @staticmethod
    def create_translation_transform(parent_frame: str, child_frame: str,
                                   x: float, y: float, z: float,
                                   timestamp: Optional[float] = None) -> FrameTransform:
        """Create a translation-only transform"""
        if not FOXGLOVE_AVAILABLE:
            raise ImportError("Foxglove SDK not available")
        
        ts = Timestamp(sec=int(timestamp or time.time()), nsec=0)
        return FrameTransform(
            timestamp=ts,
            parent_frame_id=parent_frame,
            child_frame_id=child_frame,
            translation=Vector3(x=x, y=y, z=z),
            rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
    
    @staticmethod
    def create_rotation_transform(parent_frame: str, child_frame: str,
                                roll: float, pitch: float, yaw: float,
                                timestamp: Optional[float] = None) -> FrameTransform:
        """
        Create a rotation-only transform from Euler angles
        
        Args:
            parent_frame: Parent frame ID
            child_frame: Child frame ID
            roll: Roll angle in radians
            pitch: Pitch angle in radians
            yaw: Yaw angle in radians
            timestamp: Optional timestamp
        """
        if not FOXGLOVE_AVAILABLE:
            raise ImportError("Foxglove SDK not available")
        
        # Convert Euler angles to quaternion
        qx, qy, qz, qw = TransformUtils.euler_to_quaternion(roll, pitch, yaw)
        
        ts = Timestamp(sec=int(timestamp or time.time()), nsec=0)
        return FrameTransform(
            timestamp=ts,
            parent_frame_id=parent_frame,
            child_frame_id=child_frame,
            translation=Vector3(x=0.0, y=0.0, z=0.0),
            rotation=Quaternion(x=qx, y=qy, z=qz, w=qw)
        )
    
    @staticmethod
    def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Tuple[float, float, float, float]:
        """
        Convert Euler angles to quaternion
        
        Args:
            roll: Roll angle in radians
            pitch: Pitch angle in radians
            yaw: Yaw angle in radians
            
        Returns:
            Tuple of (qx, qy, qz, qw)
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return qx, qy, qz, qw


class PrimitiveUtils:
    """Utilities for creating 3D primitives"""
    
    @staticmethod
    def create_cube(position: Tuple[float, float, float],
                   size: Tuple[float, float, float] = (1.0, 1.0, 1.0),
                   color: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 1.0),
                   orientation: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)) -> CubePrimitive:
        """Create a cube primitive with given parameters"""
        if not FOXGLOVE_AVAILABLE:
            raise ImportError("Foxglove SDK not available")
        
        return CubePrimitive(
            pose=Pose(
                position=Vector3(x=position[0], y=position[1], z=position[2]),
                orientation=Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])
            ),
            size=Vector3(x=size[0], y=size[1], z=size[2]),
            color=Color(r=color[0], g=color[1], b=color[2], a=color[3])
        )
    
    @staticmethod
    def create_sphere(position: Tuple[float, float, float],
                     radius: float = 0.5,
                     color: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 1.0)) -> SpherePrimitive:
        """Create a sphere primitive"""
        if not FOXGLOVE_AVAILABLE:
            raise ImportError("Foxglove SDK not available")
        
        return SpherePrimitive(
            pose=Pose(
                position=Vector3(x=position[0], y=position[1], z=position[2]),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            ),
            size=Vector3(x=radius*2, y=radius*2, z=radius*2),
            color=Color(r=color[0], g=color[1], b=color[2], a=color[3])
        )
    
    @staticmethod
    def create_line(points: List[Tuple[float, float, float]],
                   thickness: float = 0.05,
                   color: Tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0)) -> LinePrimitive:
        """Create a line primitive through given points"""
        if not FOXGLOVE_AVAILABLE:
            raise ImportError("Foxglove SDK not available")
        
        point3_list = [Point3(x=p[0], y=p[1], z=p[2]) for p in points]
        return LinePrimitive(
            pose=Pose(
                position=Vector3(x=0.0, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            ),
            thickness=thickness,
            scale_invariant=False,
            points=point3_list,
            color=Color(r=color[0], g=color[1], b=color[2], a=color[3])
        )
    
    @staticmethod
    def create_arrow(position: Tuple[float, float, float],
                    direction: float,
                    length: float = 1.5,
                    thickness: float = 0.1,
                    color: Tuple[float, float, float, float] = (1.0, 0.0, 0.0, 1.0)) -> ArrowPrimitive:
        """Create an arrow primitive pointing in a given direction"""
        if not FOXGLOVE_AVAILABLE:
            raise ImportError("Foxglove SDK not available")
        
        quat_z = math.sin(direction / 2.0)
        quat_w = math.cos(direction / 2.0)
        
        return ArrowPrimitive(
            pose=Pose(
                position=Vector3(x=position[0], y=position[1], z=position[2]),
                orientation=Quaternion(x=0.0, y=0.0, z=quat_z, w=quat_w)
            ),
            shaft_length=length,
            shaft_diameter=thickness,
            head_length=length * 0.3,
            head_diameter=thickness * 2,
            color=Color(r=color[0], g=color[1], b=color[2], a=color[3])
        )


class GridUtils:
    """Utilities for creating grid/map data"""
    
    @staticmethod
    def create_occupancy_grid(width: int, height: int, resolution: float,
                            origin_x: float = 0.0, origin_y: float = 0.0,
                            data: Optional[np.ndarray] = None,
                            frame_id: str = "map",
                            timestamp: Optional[float] = None) -> Grid:
        """
        Create an occupancy grid
        
        Args:
            width: Grid width in cells
            height: Grid height in cells  
            resolution: Cell size in meters
            origin_x: Grid origin X coordinate
            origin_y: Grid origin Y coordinate
            data: Optional numpy array of grid data (0-100 for occupancy)
            frame_id: Frame ID for the grid
            timestamp: Optional timestamp
        """
        if not FOXGLOVE_AVAILABLE:
            raise ImportError("Foxglove SDK not available")
        
        # Create default data if not provided
        if data is None:
            data = np.zeros((height, width), dtype=np.uint8)
        
        # Ensure data is the right shape and type
        if data.shape != (height, width):
            raise ValueError(f"Data shape {data.shape} doesn't match grid size ({height}, {width})")
        
        # Create packed element field for occupancy data
        field = PackedElementField(
            name="occupancy",
            offset=0,
            type=PackedElementFieldNumericType.Uint8
        )
        
        ts = Timestamp(sec=int(timestamp or time.time()), nsec=0)
        
        return Grid(
            timestamp=ts,
            frame_id=frame_id,
            pose=Pose(
                position=Vector3(x=origin_x, y=origin_y, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            ),
            column_count=width,
            cell_size=Vector2(x=resolution, y=resolution),
            row_stride=width,
            cell_stride=1,
            fields=[field],
            data=data.tobytes()
        )
    
    @staticmethod
    def create_simple_test_grid(size: int = 100, resolution: float = 0.1) -> Grid:
        """Create a simple test grid with some obstacles"""
        if not FOXGLOVE_AVAILABLE:
            raise ImportError("Foxglove SDK not available")
        
        # Create a grid with some obstacles
        data = np.zeros((size, size), dtype=np.uint8)
        
        # Add some rectangular obstacles
        data[20:30, 20:30] = 100  # Obstacle 1
        data[70:80, 70:80] = 100  # Obstacle 2
        data[40:50, 10:90] = 100  # Wall
        
        return GridUtils.create_occupancy_grid(
            width=size, height=size, resolution=resolution,
            data=data, frame_id="map"
        )


class PointCloudUtils:
    """Utilities for creating point cloud data"""
    
    @staticmethod
    def create_point_cloud(points: np.ndarray,
                          colors: Optional[np.ndarray] = None,
                          intensities: Optional[np.ndarray] = None,
                          frame_id: str = "lidar",
                          timestamp: Optional[float] = None) -> PointCloud:
        """
        Create a point cloud from numpy arrays
        
        Args:
            points: Nx3 array of (x, y, z) coordinates
            colors: Optional Nx3 array of RGB colors (0-255)
            intensities: Optional Nx1 array of intensities
            frame_id: Frame ID for the point cloud
            timestamp: Optional timestamp
        """
        if not FOXGLOVE_AVAILABLE:
            raise ImportError("Foxglove SDK not available")
        
        if points.shape[1] != 3:
            raise ValueError("Points array must be Nx3")
        
        n_points = points.shape[0]
        
        # Define fields
        fields = [
            PackedElementField(name="x", offset=0, type=PackedElementFieldNumericType.Float32),
            PackedElementField(name="y", offset=4, type=PackedElementFieldNumericType.Float32),
            PackedElementField(name="z", offset=8, type=PackedElementFieldNumericType.Float32),
        ]
        
        # Start with XYZ data
        point_data = points.astype(np.float32)
        point_stride = 12  # 3 floats * 4 bytes each
        
        # Add colors if provided
        if colors is not None:
            if colors.shape != (n_points, 3):
                raise ValueError("Colors array must be Nx3")
            fields.extend([
                PackedElementField(name="r", offset=point_stride, type=PackedElementFieldNumericType.Uint8),
                PackedElementField(name="g", offset=point_stride+1, type=PackedElementFieldNumericType.Uint8),
                PackedElementField(name="b", offset=point_stride+2, type=PackedElementFieldNumericType.Uint8),
            ])
            colors_data = colors.astype(np.uint8)
            point_data = np.hstack([point_data, colors_data])
            point_stride += 3
        
        # Add intensities if provided
        if intensities is not None:
            if intensities.shape[0] != n_points:
                raise ValueError("Intensities array must have same length as points")
            fields.append(
                PackedElementField(name="intensity", offset=point_stride, type=PackedElementFieldNumericType.Float32)
            )
            intensities_data = intensities.astype(np.float32).reshape(-1, 1)
            point_data = np.hstack([point_data, intensities_data])
            point_stride += 4
        
        ts = Timestamp(sec=int(timestamp or time.time()), nsec=0)
        
        return PointCloud(
            timestamp=ts,
            frame_id=frame_id,
            pose=Pose(
                position=Vector3(x=0.0, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            ),
            point_stride=point_stride,
            fields=fields,
            data=point_data.tobytes()
        )
    
    @staticmethod
    def create_test_point_cloud(n_points: int = 1000, radius: float = 5.0) -> PointCloud:
        """Create a test point cloud with random points in a sphere"""
        if not FOXGLOVE_AVAILABLE:
            raise ImportError("Foxglove SDK not available")
        
        # Generate random points in a sphere
        phi = np.random.uniform(0, 2*np.pi, n_points)
        costheta = np.random.uniform(-1, 1, n_points)
        u = np.random.uniform(0, 1, n_points)
        
        theta = np.arccos(costheta)
        r = radius * (u ** (1/3))
        
        x = r * np.sin(theta) * np.cos(phi)
        y = r * np.sin(theta) * np.sin(phi)
        z = r * np.cos(theta)
        
        points = np.column_stack([x, y, z])
        
        # Generate colors based on height
        colors = np.zeros((n_points, 3), dtype=np.uint8)
        colors[:, 0] = np.clip((z + radius) / (2 * radius) * 255, 0, 255)  # Red based on height
        colors[:, 1] = np.clip(128 + 127 * np.sin(phi), 0, 255)  # Green based on angle
        colors[:, 2] = 100  # Constant blue
        
        return PointCloudUtils.create_point_cloud(points, colors, frame_id="lidar")


class LaserScanUtils:
    """Utilities for creating laser scan data"""
    
    @staticmethod
    def create_laser_scan(ranges: List[float],
                         start_angle: float = -math.pi,
                         end_angle: float = math.pi,
                         intensities: Optional[List[float]] = None,
                         frame_id: str = "laser",
                         timestamp: Optional[float] = None) -> LaserScan:
        """
        Create a laser scan
        
        Args:
            ranges: List of range measurements in meters
            start_angle: Start angle in radians
            end_angle: End angle in radians
            intensities: Optional list of intensity values
            frame_id: Frame ID for the scan
            timestamp: Optional timestamp
        """
        if not FOXGLOVE_AVAILABLE:
            raise ImportError("Foxglove SDK not available")
        
        ts = Timestamp(sec=int(timestamp or time.time()), nsec=0)
        
        return LaserScan(
            timestamp=ts,
            frame_id=frame_id,
            pose=Pose(
                position=Vector3(x=0.0, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            ),
            start_angle=start_angle,
            end_angle=end_angle,
            ranges=ranges,
            intensities=intensities or []
        )
    
    @staticmethod
    def create_test_laser_scan(n_rays: int = 360, max_range: float = 10.0) -> LaserScan:
        """Create a test laser scan with simulated data"""
        if not FOXGLOVE_AVAILABLE:
            raise ImportError("Foxglove SDK not available")
        
        # Create simulated ranges (simple room with some obstacles)
        angles = np.linspace(-math.pi, math.pi, n_rays)
        ranges = []
        
        for angle in angles:
            # Simulate a rectangular room with some obstacles
            if abs(angle) < math.pi/4:  # Front
                range_val = max_range * 0.8
            elif abs(angle) > 3*math.pi/4:  # Back
                range_val = max_range * 0.6
            else:  # Sides
                range_val = max_range * 0.9
            
            # Add some noise and occasional obstacles
            range_val += np.random.normal(0, 0.1)
            if np.random.random() < 0.1:  # 10% chance of obstacle
                range_val *= 0.3
            
            ranges.append(max(0.1, min(max_range, range_val)))
        
        return LaserScanUtils.create_laser_scan(
            ranges=ranges,
            start_angle=-math.pi,
            end_angle=math.pi,
            frame_id="laser"
        )
