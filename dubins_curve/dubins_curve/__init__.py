"""
Dubins Curve Package

This package provides implementations of Dubins curves for path planning.
Dubins curves are the shortest paths between two points with constraints on
the curvature of the path.
"""

from .dubins import DubinsPath
from .types import Pose, DubinsSegment

__version__ = "0.1.0"
__all__ = ["DubinsPath", "Pose", "DubinsSegment"]