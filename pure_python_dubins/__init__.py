"""
Pure Python implementation of Dubins path planning algorithms.

This module provides a pure Python implementation of the Dubins path algorithms
without requiring Cython or C compilation. It implements the same API as the
original pydubins library.

Author: Rewritten from pydubins by Andrew Walker
License: MIT License
"""

from .dubins import (
    DubinsPath,
    shortest_path,
    path,
    path_sample,
    norm_path,
    backward_path_sample,
    shortest_backward_path,
    LSL, LSR, RSL, RSR, RLR, LRL
)

__all__ = [
    'DubinsPath',
    'shortest_path', 
    'path',
    'path_sample',
    'norm_path',
    'backward_path_sample',
    'shortest_backward_path',
    'LSL', 'LSR', 'RSL', 'RSR', 'RLR', 'LRL'
]

__version__ = "1.0.0"