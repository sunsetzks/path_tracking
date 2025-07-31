"""
Dubins Path Implementation

This module implements the Dubins path algorithm for finding the shortest
path between two poses with curvature constraints.
"""

import math
from typing import List, Tuple, Optional
from .types import Pose, DubinsSegment, DubinsPathResult


class DubinsPath:
    """
    Implementation of Dubins path algorithm
    
    Dubins paths are the shortest paths between two points with constraints
    on the minimum turning radius. There are 6 possible path types:
    - LSL: Left-Straight-Left
    - RSR: Right-Straight-Right  
    - LSR: Left-Straight-Right
    - RSL: Right-Straight-Left
    - RLR: Right-Left-Right
    - LRL: Left-Right-Left
    """
    
    def __init__(self, turning_radius: float = 1.0):
        """
        Initialize Dubins path calculator
        
        Args:
            turning_radius: Minimum turning radius (default: 1.0)
        """
        self.turning_radius = turning_radius
        self.step_size = 0.1  # For path discretization
    
    def path(self, start: Pose, end: Pose) -> DubinsPathResult:
        """
        Calculate the shortest Dubins path between two poses
        
        Args:
            start: Starting pose
            end: Ending pose
            
        Returns:
            DubinsPathResult containing the path information
        """
        # Transform to start-centered coordinate system
        dx = end.x - start.x
        dy = end.y - start.y
        D = math.sqrt(dx*dx + dy*dy)
        
        # Transform to start frame
        theta = math.atan2(dy, dx)
        
        # Calculate relative end pose in start frame
        x = D * math.cos(theta - start.theta)
        y = D * math.sin(theta - start.theta)
        
        # Calculate all possible path types
        path_types = ['LSL', 'RSR', 'LSR', 'RSL', 'RLR', 'LRL']
        results = []
        
        for path_type in path_types:
            try:
                result = self._calculate_path_type(x, y, path_type)
                if result.success:
                    # Transform back to global frame
                    segments = self._transform_segments_to_global(result.segments, start)
                    results.append(DubinsPathResult(
                        path_type=path_type,
                        total_length=result.total_length,
                        segments=segments,
                        success=True
                    ))
            except Exception as e:
                # Skip invalid paths
                continue
        
        if not results:
            return DubinsPathResult(
                path_type="",
                total_length=0.0,
                segments=[],
                success=False,
                error_message="No valid Dubins path found"
            )
        
        # Return the shortest path
        shortest = min(results, key=lambda r: r.total_length)
        return shortest
    
    def _calculate_path_type(self, x: float, y: float, path_type: str) -> DubinsPathResult:
        """Calculate a specific path type"""
        if path_type == 'LSL':
            t, p, q = self._lsl(x, y)
        elif path_type == 'RSR':
            t, p, q = self._rsr(x, y)
        elif path_type == 'LSR':
            t, p, q = self._lsr(x, y)
        elif path_type == 'RSL':
            t, p, q = self._rsl(x, y)
        elif path_type == 'RLR':
            t, p, q = self._rlr(x, y)
        elif path_type == 'LRL':
            t, p, q = self._lrl(x, y)
        else:
            raise ValueError(f"Unknown path type: {path_type}")
        
        if t < 0 or p < 0 or q < 0:
            return DubinsPathResult(
                path_type=path_type,
                total_length=0.0,
                segments=[],
                success=False,
                error_message="Invalid path parameters"
            )
        
        total_length = t + p + q
        segments = self._create_segments(x, y, t, p, q, path_type)
        
        return DubinsPathResult(
            path_type=path_type,
            total_length=total_length,
            segments=segments,
            success=True
        )
    
    def _lsl(self, x: float, y: float) -> Tuple[float, float, float]:
        """Calculate LSL (Left-Straight-Left) path parameters"""
        tmp = x - y
        p_squared = 4 * self.turning_radius**2 + tmp**2
        if p_squared < 0:
            return -1, -1, -1
        
        p = math.sqrt(p_squared)
        angle = math.atan2(-y, x - self.turning_radius)
        t = self._mod_2pi(-angle)
        q = self._mod_2pi(-angle)
        
        return t, p, q
    
    def _rsr(self, x: float, y: float) -> Tuple[float, float, float]:
        """Calculate RSR (Right-Straight-Right) path parameters"""
        tmp = x + y
        p_squared = 4 * self.turning_radius**2 + tmp**2
        if p_squared < 0:
            return -1, -1, -1
        
        p = math.sqrt(p_squared)
        angle = math.atan2(y, x + self.turning_radius)
        t = self._mod_2pi(angle)
        q = self._mod_2pi(angle)
        
        return t, p, q
    
    def _lsr(self, x: float, y: float) -> Tuple[float, float, float]:
        """Calculate LSR (Left-Straight-Right) path parameters"""
        tmp = x + y
        p_squared = 4 * self.turning_radius**2 - tmp**2
        if p_squared < 0:
            return -1, -1, -1
        
        p = math.sqrt(p_squared)
        angle = math.atan2(x - self.turning_radius, y)
        t = self._mod_2pi(angle)
        q = self._mod_2pi(-angle)
        
        return t, p, q
    
    def _rsl(self, x: float, y: float) -> Tuple[float, float, float]:
        """Calculate RSL (Right-Straight-Left) path parameters"""
        tmp = x - y
        p_squared = 4 * self.turning_radius**2 - tmp**2
        if p_squared < 0:
            return -1, -1, -1
        
        p = math.sqrt(p_squared)
        angle = math.atan2(x + self.turning_radius, y)
        t = self._mod_2pi(-angle)
        q = self._mod_2pi(angle)
        
        return t, p, q
    
    def _rlr(self, x: float, y: float) -> Tuple[float, float, float]:
        """Calculate RLR (Right-Left-Right) path parameters"""
        tmp = x**2 + y**2 - 6 * self.turning_radius**2
        if tmp < 0:
            return -1, -1, -1
        
        p_squared = 4 * self.turning_radius**2 - tmp
        if p_squared < 0:
            return -1, -1, -1
        
        p = math.sqrt(p_squared)
        angle = math.atan2(y, x - 2 * self.turning_radius)
        t = self._mod_2pi(-angle + math.atan2(-p, 2 * self.turning_radius))
        q = self._mod_2pi(angle - math.atan2(-p, 2 * self.turning_radius))
        
        return t, p, q
    
    def _lrl(self, x: float, y: float) -> Tuple[float, float, float]:
        """Calculate LRL (Left-Right-Left) path parameters"""
        tmp = x**2 + y**2 - 6 * self.turning_radius**2
        if tmp < 0:
            return -1, -1, -1
        
        p_squared = 4 * self.turning_radius**2 - tmp
        if p_squared < 0:
            return -1, -1, -1
        
        p = math.sqrt(p_squared)
        angle = math.atan2(y, x + 2 * self.turning_radius)
        t = self._mod_2pi(angle + math.atan2(-p, 2 * self.turning_radius))
        q = self._mod_2pi(-angle - math.atan2(-p, 2 * self.turning_radius))
        
        return t, p, q
    
    def _mod_2pi(self, angle: float) -> float:
        """Normalize angle to [0, 2π)"""
        return angle % (2 * math.pi)
    
    def _create_segments(self, x: float, y: float, t: float, p: float, q: float, path_type: str) -> List[DubinsSegment]:
        """Create path segments from parameters"""
        segments = []
        
        # Calculate intermediate poses
        if path_type == 'LSL':
            # First left turn
            start_pose = Pose(0, 0, 0)
            mid1_pose = Pose(self.turning_radius * (1 - math.cos(t)), 
                           self.turning_radius * math.sin(t), t)
            # Straight line
            mid2_pose = Pose(mid1_pose.x + p * math.cos(mid1_pose.theta + math.pi/2),
                           mid1_pose.y + p * math.sin(mid1_pose.theta + math.pi/2),
                           mid1_pose.theta)
            # Second left turn
            end_pose = Pose(x, y, math.atan2(y, x))
            
            segments.extend([
                DubinsSegment('L', t, start_pose, mid1_pose),
                DubinsSegment('S', p, mid1_pose, mid2_pose),
                DubinsSegment('L', q, mid2_pose, end_pose)
            ])
        
        elif path_type == 'RSR':
            # First right turn
            start_pose = Pose(0, 0, 0)
            mid1_pose = Pose(-self.turning_radius * (1 - math.cos(t)), 
                           -self.turning_radius * math.sin(t), -t)
            # Straight line
            mid2_pose = Pose(mid1_pose.x + p * math.cos(mid1_pose.theta - math.pi/2),
                           mid1_pose.y + p * math.sin(mid1_pose.theta - math.pi/2),
                           mid1_pose.theta)
            # Second right turn
            end_pose = Pose(x, y, math.atan2(y, x))
            
            segments.extend([
                DubinsSegment('R', t, start_pose, mid1_pose),
                DubinsSegment('S', p, mid1_pose, mid2_pose),
                DubinsSegment('R', q, mid2_pose, end_pose)
            ])
        
        # Add other path types as needed...
        elif path_type == 'LSR':
            # First left turn
            start_pose = Pose(0, 0, 0)
            mid1_pose = Pose(self.turning_radius * (1 - math.cos(t)),
                           self.turning_radius * math.sin(t), t)
            # Straight line
            mid2_pose = Pose(mid1_pose.x + p * math.cos(mid1_pose.theta + math.pi/2),
                           mid1_pose.y + p * math.sin(mid1_pose.theta + math.pi/2),
                           mid1_pose.theta)
            # Second right turn
            end_pose = Pose(x, y, math.atan2(y, x))
            
            segments.extend([
                DubinsSegment('L', t, start_pose, mid1_pose),
                DubinsSegment('S', p, mid1_pose, mid2_pose),
                DubinsSegment('R', q, mid2_pose, end_pose)
            ])
        
        elif path_type == 'RSL':
            # First right turn
            start_pose = Pose(0, 0, 0)
            mid1_pose = Pose(-self.turning_radius * (1 - math.cos(t)),
                           -self.turning_radius * math.sin(t), -t)
            # Straight line
            mid2_pose = Pose(mid1_pose.x + p * math.cos(mid1_pose.theta - math.pi/2),
                           mid1_pose.y + p * math.sin(mid1_pose.theta - math.pi/2),
                           mid1_pose.theta)
            # Second left turn
            end_pose = Pose(x, y, math.atan2(y, x))
            
            segments.extend([
                DubinsSegment('R', t, start_pose, mid1_pose),
                DubinsSegment('S', p, mid1_pose, mid2_pose),
                DubinsSegment('L', q, mid2_pose, end_pose)
            ])
        
        elif path_type == 'RLR':
            # First right turn
            start_pose = Pose(0, 0, 0)
            mid1_pose = Pose(-self.turning_radius * (1 - math.cos(t)),
                           -self.turning_radius * math.sin(t), -t)
            # Second left turn
            mid2_pose = Pose(mid1_pose.x + 2 * self.turning_radius * math.sin(t) - self.turning_radius * math.sin(t + p),
                           mid1_pose.y - 2 * self.turning_radius * math.cos(t) + self.turning_radius * math.cos(t + p),
                           t + p)
            # Third right turn
            end_pose = Pose(x, y, math.atan2(y, x))
            
            segments.extend([
                DubinsSegment('R', t, start_pose, mid1_pose),
                DubinsSegment('L', p, mid1_pose, mid2_pose),
                DubinsSegment('R', q, mid2_pose, end_pose)
            ])
        
        elif path_type == 'LRL':
            # First left turn
            start_pose = Pose(0, 0, 0)
            mid1_pose = Pose(self.turning_radius * (1 - math.cos(t)),
                           self.turning_radius * math.sin(t), t)
            # Second right turn
            mid2_pose = Pose(mid1_pose.x - 2 * self.turning_radius * math.sin(t) + self.turning_radius * math.sin(t - p),
                           mid1_pose.y + 2 * self.turning_radius * math.cos(t) - self.turning_radius * math.cos(t - p),
                           t - p)
            # Third left turn
            end_pose = Pose(x, y, math.atan2(y, x))
            
            segments.extend([
                DubinsSegment('L', t, start_pose, mid1_pose),
                DubinsSegment('R', p, mid1_pose, mid2_pose),
                DubinsSegment('L', q, mid2_pose, end_pose)
            ])
        
        return segments
    
    def _transform_segments_to_global(self, segments: List[DubinsSegment], start_pose: Pose) -> List[DubinsSegment]:
        """Transform segments from local to global coordinate frame"""
        global_segments = []
        
        for segment in segments:
            # Transform start pose
            global_start = self._transform_pose(segment.start_pose, start_pose)
            
            # Transform end pose
            global_end = self._transform_pose(segment.end_pose, start_pose)
            
            global_segments.append(DubinsSegment(
                segment_type=segment.segment_type,
                length=segment.length,
                start_pose=global_start,
                end_pose=global_end
            ))
        
        return global_segments
    
    def _transform_pose(self, local_pose: Pose, global_start: Pose) -> Pose:
        """Transform pose from local to global coordinate frame"""
        # Rotation matrix
        cos_theta = math.cos(global_start.theta)
        sin_theta = math.sin(global_start.theta)
        
        # Transform position
        x = global_start.x + local_pose.x * cos_theta - local_pose.y * sin_theta
        y = global_start.y + local_pose.x * sin_theta + local_pose.y * cos_theta
        theta = global_start.theta + local_pose.theta
        
        return Pose(x, y, theta)
    
    def get_path_points(self, path_result: DubinsPathResult, num_points: int = 100) -> List[Tuple[float, float]]:
        """
        Get discretized points along the complete path
        
        Args:
            path_result: Dubins path result
            num_points: Number of points to generate
            
        Returns:
            List of (x, y) points along the path
        """
        if not path_result.success:
            return []
        
        points = []
        total_length = path_result.total_length
        
        if total_length == 0:
            return []
        
        for i in range(num_points + 1):
            s = i / num_points * total_length
            point = self._get_point_at_distance(path_result, s)
            if point:
                points.append(point)
        
        return points
    
    def _get_point_at_distance(self, path_result: DubinsPathResult, distance: float) -> Optional[Tuple[float, float]]:
        """Get point at specific distance along path"""
        if not path_result.success:
            return None
        
        current_distance = 0
        
        for segment in path_result.segments:
            if current_distance + segment.length >= distance:
                # Point is in this segment
                segment_distance = distance - current_distance
                return self._get_point_on_segment(segment, segment_distance)
            
            current_distance += segment.length
        
        return None
    
    def _get_point_on_segment(self, segment: DubinsSegment, distance: float) -> Optional[Tuple[float, float]]:
        """Get point at specific distance along a segment"""
        if segment.segment_type == 'S':
            # Straight line
            t = distance / segment.length if segment.length > 0 else 0
            x = segment.start_pose.x + t * (segment.end_pose.x - segment.start_pose.x)
            y = segment.start_pose.y + t * (segment.end_pose.y - segment.start_pose.y)
            return (x, y)
        elif segment.segment_type in ['L', 'R']:
            # Circular arc
            return self._get_point_on_arc(segment, distance)
        
        return None
    
    def _get_point_on_arc(self, segment: DubinsSegment, distance: float) -> Optional[Tuple[float, float]]:
        """Get point on circular arc segment"""
        if segment.length == 0:
            return (segment.start_pose.x, segment.start_pose.y)
        
        # Calculate angle swept
        angle = distance / self.turning_radius
        
        # Determine direction
        if segment.segment_type == 'L':
            direction = 1
        else:
            direction = -1
        
        # Calculate center of circle
        if segment.segment_type == 'L':
            # Left turn: center is to the left of start direction
            center_x = segment.start_pose.x - self.turning_radius * math.sin(segment.start_pose.theta)
            center_y = segment.start_pose.y + self.turning_radius * math.cos(segment.start_pose.theta)
        else:
            # Right turn: center is to the right of start direction
            center_x = segment.start_pose.x + self.turning_radius * math.sin(segment.start_pose.theta)
            center_y = segment.start_pose.y - self.turning_radius * math.cos(segment.start_pose.theta)
        
        # Calculate point on circle
        start_angle = segment.start_pose.theta - direction * math.pi/2
        current_angle = start_angle + direction * angle
        
        x = center_x + self.turning_radius * math.cos(current_angle)
        y = center_y + self.turning_radius * math.sin(current_angle)
        
        return (x, y)