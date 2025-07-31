"""
Type definitions for Dubins curves
"""

from dataclasses import dataclass
from typing import Tuple, List
import math


@dataclass
class Pose:
    """2D pose with position and orientation"""
    x: float
    y: float
    theta: float  # orientation in radians
    
    def __post_init__(self):
        # Normalize theta to [-pi, pi]
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
    
    def distance_to(self, other: 'Pose') -> float:
        """Calculate Euclidean distance to another pose"""
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def angle_to(self, other: 'Pose') -> float:
        """Calculate angle to another pose"""
        return math.atan2(other.y - self.y, other.x - self.x)


@dataclass
class DubinsSegment:
    """A single segment of a Dubins path"""
    segment_type: str  # 'L', 'R', or 'S' (left, right, straight)
    length: float
    start_pose: Pose
    end_pose: Pose
    
    def get_points(self, num_points: int = 20) -> List[Tuple[float, float]]:
        """Get points along this segment for visualization"""
        points = []
        for i in range(num_points + 1):
            t = i / num_points
            if self.segment_type == 'S':
                # Straight line
                x = self.start_pose.x + t * (self.end_pose.x - self.start_pose.x)
                y = self.start_pose.y + t * (self.end_pose.y - self.start_pose.y)
            elif self.segment_type in ['L', 'R']:
                # Circular arc
                points.extend(self._circular_arc_points(t))
                continue
            points.append((x, y))
        return points
    
    def _circular_arc_points(self, t: float) -> List[Tuple[float, float]]:
        """Generate points for a circular arc segment"""
        # This will be implemented in the main DubinsPath class
        # For now, return empty list
        return []


@dataclass
class DubinsPathResult:
    """Result of Dubins path calculation"""
    path_type: str  # e.g., 'LSL', 'RSR', etc.
    total_length: float
    segments: List[DubinsSegment]
    success: bool
    error_message: str = ""
    
    def __post_init__(self):
        # Ensure total_length is positive
        if self.total_length < 0:
            self.total_length = 0.0