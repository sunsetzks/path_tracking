"""
Pure Python implementation of Dubins path planning algorithms.

This module implements the core Dubins path algorithms in pure Python,
translating the C implementation from the original pydubins library.

Author: Rewritten from pydubins by Andrew Walker
License: MIT License
"""

import math
from typing import Tuple, List, Optional, Callable

# Path type constants
LSL = 0
LSR = 1
RSL = 2
RSR = 3
RLR = 4
LRL = 5

# Error codes
EDUBOK = 0
EDUBCOCONFIGS = 1
EDUBPARAM = 2
EDUBBADRHO = 3
EDUBNOPATH = 4

# Segment types
L_SEG = 0
S_SEG = 1
R_SEG = 2

# Segment types for each path type
DIRDATA = [
    [L_SEG, S_SEG, L_SEG],  # LSL
    [L_SEG, S_SEG, R_SEG],  # LSR
    [R_SEG, S_SEG, L_SEG],  # RSL
    [R_SEG, S_SEG, R_SEG],  # RSR
    [R_SEG, L_SEG, R_SEG],  # RLR
    [L_SEG, R_SEG, L_SEG]   # LRL
]


class DubinsPath:
    """
    A Dubins path represents the shortest path between two configurations
    with a minimum turning radius constraint.
    """
    
    def __init__(self, q0: Tuple[float, float, float], q1: Tuple[float, float, float], 
                 rho: float, path_type: int, param: Tuple[float, float, float]):
        """
        Initialize a Dubins path.
        
        Args:
            q0: Initial configuration (x, y, theta)
            q1: Final configuration (x, y, theta) 
            rho: Turning radius
            path_type: Type of path (LSL, LSR, etc.)
            param: Parameters for the three segments
        """
        self.qi = q0  # Initial configuration
        self.q1 = q1  # Final configuration
        self.rho = rho
        self.type = path_type
        self.param = param  # Three segment parameters
        
    def path_length(self) -> float:
        """Calculate the total length of the path."""
        return sum(self.param) * self.rho
    
    def segment_length(self, i: int) -> float:
        """Calculate the length of the i-th segment."""
        if i < 0 or i > 2:
            return float('inf')
        return self.param[i] * self.rho
    
    def segment_length_normalized(self, i: int) -> float:
        """Calculate the normalized length of the i-th segment."""
        if i < 0 or i > 2:
            return float('inf')
        return self.param[i]
    
    def path_type(self) -> int:
        """Return the type of path."""
        return self.type
    
    def path_endpoint(self) -> Tuple[float, float, float]:
        """Calculate the endpoint of the path."""
        return self.q1
    
    def sample(self, t: float) -> Tuple[float, float, float]:
        """
        Sample the path at distance t from the start.
        
        Args:
            t: Distance along the path (0 <= t <= path_length)
            
        Returns:
            Configuration (x, y, theta) at distance t
        """
        if t < 0 or t > self.path_length():
            raise ValueError("t is out of range")
        
        # Normalize t
        t_prime = t / self.rho
        
        # Start from origin with initial orientation
        qi = [0.0, 0.0, self.qi[2]]
        q = [0.0, 0.0, 0.0]
        
        # Get segment types for this path
        segment_types = DIRDATA[self.type]
        
        # Calculate segment endpoints
        p1 = self.param[0]
        p2 = self.param[1]
        
        # Sample based on which segment t falls into
        if t_prime < p1:
            _dubins_segment(t_prime, qi, q, segment_types[0])
        elif t_prime < (p1 + p2):
            q1_seg = [0.0, 0.0, 0.0]
            _dubins_segment(p1, qi, q1_seg, segment_types[0])
            _dubins_segment(t_prime - p1, q1_seg, q, segment_types[1])
        else:
            q1_seg = [0.0, 0.0, 0.0]
            q2_seg = [0.0, 0.0, 0.0]
            _dubins_segment(p1, qi, q1_seg, segment_types[0])
            _dubins_segment(p2, q1_seg, q2_seg, segment_types[1])
            _dubins_segment(t_prime - p1 - p2, q2_seg, q, segment_types[2])
        
        # Scale and translate back to original coordinate system
        q[0] = q[0] * self.rho + self.qi[0]
        q[1] = q[1] * self.rho + self.qi[1]
        q[2] = _mod2pi(q[2])
        
        return (q[0], q[1], q[2])
    
    def sample_many(self, step_size: float, callback: Optional[Callable] = None) -> Tuple[List[Tuple[float, float, float]], List[float]]:
        """
        Sample the entire path at regular intervals.
        
        Args:
            step_size: Distance between samples
            callback: Optional callback function called for each sample
            
        Returns:
            Tuple of (configurations, distances)
        """
        if callback is None:
            configurations = []
            distances = []
            
            def default_callback(q, t):
                configurations.append(tuple(q))
                distances.append(t)
                return 0
            
            callback = default_callback
        
        x = 0.0
        length = self.path_length()
        
        while x < length:
            # Sample the actual point on the path
            q = list(self.sample(x))
            retcode = callback(q, x)
            if retcode != 0:
                break
            x += step_size
        
        return configurations, distances
    
    def extract_subpath(self, t: float) -> 'DubinsPath':
        """
        Extract a subpath from the start to distance t.
        
        Args:
            t: Distance along the path (0 < t < path_length)
            
        Returns:
            New DubinsPath object representing the subpath
        """
        if t <= 0 or t >= self.path_length():
            raise ValueError("t is out of range")
        
        t_prime = t / self.rho
        
        # Create new path with same start and type
        param0 = min(self.param[0], t_prime)
        param1 = min(self.param[1], t_prime - param0)
        param2 = min(self.param[2], t_prime - param0 - param1)
        new_param = (param0, param1, param2)
        
        # Calculate new endpoint
        new_q1 = self.sample(t)
        
        return DubinsPath(self.qi, new_q1, self.rho, self.type, new_param)


def _fmodr(x: float, y: float) -> float:
    """Floating point modulus suitable for rings."""
    return x - y * math.floor(x / y)


def _mod2pi(theta: float) -> float:
    """Normalize angle to [0, 2π)."""
    return _fmodr(theta, 2 * math.pi)


def _dubins_segment(t: float, qi: List[float], qt: List[float], segment_type: int):
    """
    Calculate a segment of a Dubins path.
    
    Args:
        t: Segment length
        qi: Initial configuration [x, y, theta]
        qt: Target configuration [x, y, theta] (output)
        segment_type: Type of segment (L_SEG, S_SEG, R_SEG)
    """
    st = math.sin(qi[2])
    ct = math.cos(qi[2])
    
    if segment_type == L_SEG:
        qt[0] = math.sin(qi[2] + t) - st
        qt[1] = -math.cos(qi[2] + t) + ct
        qt[2] = t
    elif segment_type == R_SEG:
        qt[0] = -math.sin(qi[2] - t) + st
        qt[1] = math.cos(qi[2] - t) - ct
        qt[2] = -t
    elif segment_type == S_SEG:
        qt[0] = ct * t
        qt[1] = st * t
        qt[2] = 0.0
    
    qt[0] += qi[0]
    qt[1] += qi[1]
    qt[2] += qi[2]


class _DubinsIntermediateResults:
    """Intermediate results for Dubins path calculations."""
    
    def __init__(self):
        self.alpha = 0.0
        self.beta = 0.0
        self.d = 0.0
        self.sa = 0.0
        self.sb = 0.0
        self.ca = 0.0
        self.cb = 0.0
        self.c_ab = 0.0
        self.d_sq = 0.0


def _dubins_intermediate_results(in_results: _DubinsIntermediateResults, 
                                q0: Tuple[float, float, float], 
                                q1: Tuple[float, float, float], 
                                rho: float) -> int:
    """
    Calculate intermediate results for Dubins path planning.
    
    Args:
        in_results: Object to store intermediate results
        q0: Initial configuration
        q1: Final configuration
        rho: Turning radius
        
    Returns:
        Error code (EDUBOK on success)
    """
    if rho <= 0.0:
        return EDUBBADRHO
    
    dx = q1[0] - q0[0]
    dy = q1[1] - q0[1]
    D = math.sqrt(dx * dx + dy * dy)
    d = D / rho
    theta = 0.0
    
    # Prevent domain errors if dx=0 and dy=0
    if d > 0:
        theta = _mod2pi(math.atan2(dy, dx))
    
    alpha = _mod2pi(q0[2] - theta)
    beta = _mod2pi(q1[2] - theta)
    
    in_results.alpha = alpha
    in_results.beta = beta
    in_results.d = d
    in_results.sa = math.sin(alpha)
    in_results.sb = math.sin(beta)
    in_results.ca = math.cos(alpha)
    in_results.cb = math.cos(beta)
    in_results.c_ab = math.cos(alpha - beta)
    in_results.d_sq = d * d
    
    return EDUBOK


def _dubins_LSL(in_results: _DubinsIntermediateResults, out: List[float]) -> int:
    """Calculate LSL path parameters."""
    tmp0 = in_results.d + in_results.sa - in_results.sb
    p_sq = 2 + in_results.d_sq - (2 * in_results.c_ab) + (2 * in_results.d * (in_results.sa - in_results.sb))
    
    if p_sq >= 0:
        tmp1 = math.atan2(in_results.cb - in_results.ca, tmp0)
        out[0] = _mod2pi(tmp1 - in_results.alpha)
        out[1] = math.sqrt(p_sq)
        out[2] = _mod2pi(in_results.beta - tmp1)
        return EDUBOK
    return EDUBNOPATH


def _dubins_RSR(in_results: _DubinsIntermediateResults, out: List[float]) -> int:
    """Calculate RSR path parameters."""
    tmp0 = in_results.d - in_results.sa + in_results.sb
    p_sq = 2 + in_results.d_sq - (2 * in_results.c_ab) + (2 * in_results.d * (in_results.sb - in_results.sa))
    
    if p_sq >= 0:
        tmp1 = math.atan2(in_results.ca - in_results.cb, tmp0)
        out[0] = _mod2pi(in_results.alpha - tmp1)
        out[1] = math.sqrt(p_sq)
        out[2] = _mod2pi(tmp1 - in_results.beta)
        return EDUBOK
    return EDUBNOPATH


def _dubins_LSR(in_results: _DubinsIntermediateResults, out: List[float]) -> int:
    """Calculate LSR path parameters."""
    p_sq = -2 + in_results.d_sq + (2 * in_results.c_ab) + (2 * in_results.d * (in_results.sa + in_results.sb))
    
    if p_sq >= 0:
        p = math.sqrt(p_sq)
        tmp0 = math.atan2(-in_results.ca - in_results.cb, in_results.d + in_results.sa + in_results.sb) - math.atan2(-2.0, p)
        out[0] = _mod2pi(tmp0 - in_results.alpha)
        out[1] = p
        out[2] = _mod2pi(tmp0 - _mod2pi(in_results.beta))
        return EDUBOK
    return EDUBNOPATH


def _dubins_RSL(in_results: _DubinsIntermediateResults, out: List[float]) -> int:
    """Calculate RSL path parameters."""
    p_sq = -2 + in_results.d_sq + (2 * in_results.c_ab) - (2 * in_results.d * (in_results.sa + in_results.sb))
    
    if p_sq >= 0:
        p = math.sqrt(p_sq)
        tmp0 = math.atan2(in_results.ca + in_results.cb, in_results.d - in_results.sa - in_results.sb) - math.atan2(2.0, p)
        out[0] = _mod2pi(in_results.alpha - tmp0)
        out[1] = p
        out[2] = _mod2pi(in_results.beta - tmp0)
        return EDUBOK
    return EDUBNOPATH


def _dubins_RLR(in_results: _DubinsIntermediateResults, out: List[float]) -> int:
    """Calculate RLR path parameters."""
    tmp0 = (6. - in_results.d_sq + 2 * in_results.c_ab + 2 * in_results.d * (in_results.sa - in_results.sb)) / 8.0
    phi = math.atan2(in_results.ca - in_results.cb, in_results.d - in_results.sa + in_results.sb)
    
    if abs(tmp0) <= 1:
        p = _mod2pi((2 * math.pi) - math.acos(tmp0))
        t = _mod2pi(in_results.alpha - phi + _mod2pi(p / 2.0))
        out[0] = t
        out[1] = p
        out[2] = _mod2pi(in_results.alpha - in_results.beta - t + _mod2pi(p))
        return EDUBOK
    return EDUBNOPATH


def _dubins_LRL(in_results: _DubinsIntermediateResults, out: List[float]) -> int:
    """Calculate LRL path parameters."""
    tmp0 = (6. - in_results.d_sq + 2 * in_results.c_ab + 2 * in_results.d * (in_results.sb - in_results.sa)) / 8.0
    phi = math.atan2(in_results.ca - in_results.cb, in_results.d + in_results.sa - in_results.sb)
    
    if abs(tmp0) <= 1:
        p = _mod2pi(2 * math.pi - math.acos(tmp0))
        t = _mod2pi(-in_results.alpha - phi + p / 2.0)
        out[0] = t
        out[1] = p
        out[2] = _mod2pi(_mod2pi(in_results.beta) - in_results.alpha - t + _mod2pi(p))
        return EDUBOK
    return EDUBNOPATH


def _dubins_word(in_results: _DubinsIntermediateResults, path_type: int, out: List[float]) -> int:
    """Calculate path parameters for a specific path type."""
    if path_type == LSL:
        return _dubins_LSL(in_results, out)
    elif path_type == RSR:
        return _dubins_RSR(in_results, out)
    elif path_type == LSR:
        return _dubins_LSR(in_results, out)
    elif path_type == RSL:
        return _dubins_RSL(in_results, out)
    elif path_type == RLR:
        return _dubins_RLR(in_results, out)
    elif path_type == LRL:
        return _dubins_LRL(in_results, out)
    else:
        return EDUBNOPATH


def shortest_path(q0: Tuple[float, float, float], q1: Tuple[float, float, float], 
                 rho: float) -> DubinsPath:
    """
    Find the shortest Dubins path between two configurations.
    
    Args:
        q0: Initial configuration (x, y, theta)
        q1: Final configuration (x, y, theta)
        rho: Turning radius
        
    Returns:
        DubinsPath object representing the shortest path
        
    Raises:
        RuntimeError: If no path exists
    """
    in_results = _DubinsIntermediateResults()
    errcode = _dubins_intermediate_results(in_results, q0, q1, rho)
    if errcode != EDUBOK:
        raise RuntimeError(f"Invalid parameters: {errcode}")
    
    best_cost = float('inf')
    best_word = -1
    best_params = (0.0, 0.0, 0.0)
    
    # Try all path types
    for path_type in range(6):
        params = [0.0, 0.0, 0.0]
        errcode = _dubins_word(in_results, path_type, params)
        if errcode == EDUBOK:
            cost = sum(params)
            if cost < best_cost:
                best_cost = cost
                best_word = path_type
                best_params = tuple(params)
    
    if best_word == -1:
        raise RuntimeError("No path exists between configurations")
    
    return DubinsPath(q0, q1, rho, best_word, (best_params[0], best_params[1], best_params[2]))


def path(q0: Tuple[float, float, float], q1: Tuple[float, float, float], 
         rho: float, path_type: int) -> Optional[DubinsPath]:
    """
    Find a Dubins path with a specific path type.
    
    Args:
        q0: Initial configuration (x, y, theta)
        q1: Final configuration (x, y, theta)
        rho: Turning radius
        path_type: Specific path type (LSL, LSR, etc.)
        
    Returns:
        DubinsPath object if path exists, None otherwise
    """
    in_results = _DubinsIntermediateResults()
    errcode = _dubins_intermediate_results(in_results, q0, q1, rho)
    if errcode != EDUBOK:
        return None
    
    params = [0.0, 0.0, 0.0]
    errcode = _dubins_word(in_results, path_type, params)
    if errcode != EDUBOK:
        return None
    
    return DubinsPath(q0, q1, rho, path_type, (params[0], params[1], params[2]))


def norm_path(alpha: float, beta: float, delta: float, path_type: int) -> Optional[DubinsPath]:
    """
    Find a Dubins path in normalized coordinates.
    
    Args:
        alpha: Initial orientation
        beta: Final orientation
        delta: Distance between configurations
        path_type: Specific path type
        
    Returns:
        DubinsPath object if path exists, None otherwise
    """
    q0 = (0.0, 0.0, alpha)
    q1 = (delta, 0.0, beta)
    return path(q0, q1, 1.0, path_type)


def path_sample(q0: Tuple[float, float, float], q1: Tuple[float, float, float], 
               rho: float, step_size: float) -> Tuple[List[Tuple[float, float, float]], List[float]]:
    """
    Generate points along a Dubins path sampled at regular intervals.
    
    Args:
        q0: Initial configuration
        q1: Final configuration
        rho: Turning radius
        step_size: Distance between samples
        
    Returns:
        Tuple of (configurations, distances)
    """
    path_obj = shortest_path(q0, q1, rho)
    return path_obj.sample_many(step_size)


def backward_path_sample(q0: Tuple[float, float, float], q1: Tuple[float, float, float], 
                        rho: float, step_size: float) -> Tuple[List[Tuple[float, float, float]], List[float]]:
    """
    Generate points along a Dubins path for backward motion.
    
    Args:
        q0: Initial configuration
        q1: Final configuration
        rho: Turning radius
        step_size: Distance between samples
        
    Returns:
        Tuple of (configurations, distances)
    """
    # Swap start and end points
    q0_swapped = q1
    q1_swapped = q0
    
    # Plan path from swapped points
    path_obj = shortest_path(q0_swapped, q1_swapped, rho)
    configurations, distances = path_obj.sample_many(step_size)
    
    # Reverse the path but keep original orientations
    reversed_configs = []
    reversed_distances = []
    
    # Process in reverse order
    for i in range(len(configurations) - 1, -1, -1):
        config = configurations[i]
        distance = distances[i]
        
        # Keep the original orientation, only reverse direction
        reversed_config = config
        
        # Adjust distance to be increasing in backward direction
        reversed_distance = path_obj.path_length() - distance
        
        reversed_configs.append(reversed_config)
        reversed_distances.append(reversed_distance)
    
    return reversed_configs, reversed_distances


def shortest_backward_path(q0: Tuple[float, float, float], q1: Tuple[float, float, float], 
                          rho: float) -> DubinsPath:
    """
    Find the shortest Dubins path for backward motion.
    
    Args:
        q0: Initial configuration
        q1: Final configuration
        rho: Turning radius
        
    Returns:
        DubinsPath object for backward motion
    """
    # Swap start and end points
    q0_swapped = q1
    q1_swapped = q0
    
    # Plan path from swapped points
    return shortest_path(q0_swapped, q1_swapped, rho)