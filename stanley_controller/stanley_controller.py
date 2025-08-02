"""
Stanley Controller Implementation

An enhanced Stanley controller for autonomous vehicle path tracking with improved
performance and features compared to the basic implementation.

Author: Enhanced implementation for Path Tracking Project
Reference: 
    - Stanley: The robot that won the DARPA grand challenge
    - Autonomous Automobile Path Tracking
"""

import numpy as np
from typing import List, Tuple, Optional
from dataclasses import dataclass
import math

from .utils.angle import angle_mod
from .utils.se2 import SE2


@dataclass
class ControlParams:
    """Stanley controller parameters"""
    k_cross_track: float = 0.5  # Cross-track error gain
    k_heading: float = 1.0      # Heading error gain
    max_steer_angle: float = np.radians(30.0)  # Maximum steering angle [rad]
    max_acceleration: float = 2.0  # Maximum acceleration [m/s^2]
    max_deceleration: float = -3.0  # Maximum deceleration [m/s^2]
    wheelbase: float = 2.9  # Vehicle wheelbase [m]
    lookahead_distance: float = 5.0  # Lookahead distance for target selection [m]


class StanleyController:
    """
    Enhanced Stanley controller for path tracking.
    
    The Stanley controller combines cross-track error correction and heading error
    correction to provide smooth path following for autonomous vehicles.
    """
    
    def __init__(self, params: ControlParams):
        """
        Initialize Stanley controller.
        
        Args:
            params: Control parameters
        """
        self.params = params
        self.target_index = 0
        self.last_target_index = 0
        
    def compute_control(self, 
                       state: SE2, 
                       path_points: np.ndarray, 
                       path_yaw: np.ndarray,
                       target_speed: float,
                       current_speed: float = 0.0) -> Tuple[float, float, int]:
        """
        Compute steering and acceleration commands.
        
        Args:
            state: Current vehicle state (SE2 transformation)
            path_points: Path points as numpy array of shape (N, 2)
            path_yaw: Path yaw angles as numpy array of shape (N,)
            target_speed: Target speed [m/s]
            current_speed: Current vehicle speed [m/s]
            
        Returns:
            Tuple of (steering_angle, acceleration, target_index)
        """
        # Find target index
        target_index = self._find_target_index(state, path_points)
        
        # Ensure we don't go backwards
        if target_index <= self.last_target_index:
            target_index = self.last_target_index
        else:
            self.last_target_index = target_index
            
        # Compute errors
        cross_track_error = self._compute_cross_track_error(state, path_points, target_index)
        heading_error = self._compute_heading_error(state, path_yaw[target_index])
        
        # Stanley control law - use current_speed instead of state.x
        steering_angle = self._compute_steering_angle(cross_track_error, heading_error, current_speed)
        
        # Simple speed control - use current_speed instead of state.x
        acceleration = self._speed_control(target_speed, current_speed)
        
        # Clamp control inputs
        steering_angle = np.clip(steering_angle, 
                               -self.params.max_steer_angle, 
                               self.params.max_steer_angle)
        acceleration = np.clip(acceleration, 
                             self.params.max_deceleration, 
                             self.params.max_acceleration)
        
        return steering_angle, acceleration, target_index
    
    def _find_target_index(self, state: SE2, path_points: np.ndarray) -> int:
        """
        Find the target index along the path.
        
        Args:
            state: Current vehicle state
            path_points: Path points
            
        Returns:
            Target index
        """
        # Find closest point on path
        distances = np.linalg.norm(path_points - np.array([state.x, state.y]), axis=1)
        closest_idx = np.argmin(distances)
        
        # Look ahead based on speed
        lookahead_idx = closest_idx
        for i in range(closest_idx, len(path_points)):
            if np.linalg.norm(path_points[i] - np.array([state.x, state.y])) > self.params.lookahead_distance:
                lookahead_idx = i
                break
                
        return min(lookahead_idx, len(path_points) - 1)
    
    def _compute_cross_track_error(self, state: SE2, path_points: np.ndarray, target_idx: int) -> float:
        """
        Compute cross-track error.
        
        Args:
            state: Current vehicle state
            path_points: Path points
            target_idx: Target index
            
        Returns:
            Cross-track error [m]
        """
        if target_idx >= len(path_points):
            return 0.0
            
        # Vector from vehicle to target point
        target_point = path_points[target_idx]
        to_target = target_point - np.array([state.x, state.y])
        
        # Project onto vehicle heading direction
        heading_vector = np.array([np.cos(state.theta), np.sin(state.theta)])
        cross_track_error = np.dot(to_target, np.array([-heading_vector[1], heading_vector[0]]))
        
        return cross_track_error
    
    def _compute_heading_error(self, state: SE2, target_yaw: float) -> float:
        """
        Compute heading error.
        
        Args:
            state: Current vehicle state
            target_yaw: Target yaw angle
            
        Returns:
            Heading error [rad]
        """
        return angle_mod(target_yaw - state.theta)
    
    def _compute_steering_angle(self, cross_track_error: float, heading_error: float, speed: float) -> float:
        """
        Compute steering angle using Stanley control law.
        
        Args:
            cross_track_error: Cross-track error
            heading_error: Heading error
            speed: Current speed
            
        Returns:
            Steering angle [rad]
        """
        # Cross-track error term
        cross_track_term = np.arctan2(self.params.k_cross_track * cross_track_error, 
                                    max(speed, 1.0))  # Avoid division by zero
        
        # Total steering angle
        steering_angle = heading_error + cross_track_term
        
        return steering_angle
    
    def _speed_control(self, target_speed: float, current_speed: float) -> float:
        """
        Simple proportional speed control.
        
        Args:
            target_speed: Target speed
            current_speed: Current speed
            
        Returns:
            Acceleration command
        """
        speed_error = target_speed - current_speed
        kp_speed = 2.0  # Increased speed control gain from 1.0 to 2.0
        return kp_speed * speed_error
    
    def reset(self):
        """Reset controller state."""
        self.target_index = 0
        self.last_target_index = 0


class PathTracker:
    """
    Path tracking utility class that manages the complete tracking process.
    """
    
    def __init__(self, controller: StanleyController):
        """
        Initialize path tracker.
        
        Args:
            controller: Stanley controller instance
        """
        self.controller = controller
        self.trajectory_history = []
        
    def track_path(self, 
                   initial_state: SE2,
                   path_points: np.ndarray,
                   path_yaw: np.ndarray,
                   target_speed: float,
                   max_iterations: int = 1000,
                   dt: float = 0.1) -> dict:
        """
        Track a complete path.
        
        Args:
            initial_state: Initial vehicle state
            path_points: Path points
            path_yaw: Path yaw angles
            target_speed: Target speed
            max_iterations: Maximum simulation steps
            dt: Time step
            
        Returns:
            Dictionary with tracking results
        """
        # Reset controller
        self.controller.reset()
        
        # Initialize state
        state = SE2(initial_state.x, initial_state.y, initial_state.theta)
        current_speed = 0.0
        
        # Storage for results
        trajectory = []
        controls = []
        errors = []
        
        for i in range(max_iterations):
            # Compute control
            steering, acceleration, target_idx = self.controller.compute_control(
                state, path_points, path_yaw, target_speed)
            
            # Update state (simple bicycle model)
            state, current_speed = self._update_dynamics(state, current_speed, steering, acceleration, dt)
            
            # Store results
            trajectory.append([state.x, state.y, state.theta, current_speed])
            controls.append([steering, acceleration])
            
            # Compute errors
            if target_idx < len(path_points):
                target_point = path_points[target_idx]
                distance_error = np.linalg.norm([state.x - target_point[0], state.y - target_point[1]])
                errors.append([distance_error, abs(angle_mod(state.theta - path_yaw[target_idx]))])
            
            # Check if we've reached the end
            if target_idx >= len(path_points) - 1:
                break
                
        return {
            'trajectory': np.array(trajectory),
            'controls': np.array(controls),
            'errors': np.array(errors) if errors else np.array([]),
            'success': target_idx >= len(path_points) - 1,
            'iterations': i + 1
        }
    
    def _update_dynamics(self, state: SE2, speed: float, steering: float, acceleration: float, dt: float) -> Tuple[SE2, float]:
        """
        Update vehicle dynamics using bicycle model.
        
        Args:
            state: Current state
            speed: Current speed
            steering: Steering angle
            acceleration: Acceleration
            dt: Time step
            
        Returns:
            Updated state and speed
        """
        # Update speed
        new_speed = speed + acceleration * dt
        new_speed = max(0, new_speed)  # Prevent negative speed
        
        # Update position and heading
        if new_speed > 0:
            # Bicycle model dynamics
            beta = np.arctan(0.5 * np.tan(steering))  # Slip angle
            dx = new_speed * np.cos(state.theta + beta) * dt
            dy = new_speed * np.sin(state.theta + beta) * dt
            dtheta = (new_speed / self.controller.params.wheelbase) * np.sin(beta) * dt
            
            new_state = SE2(
                state.x + dx,
                state.y + dy,
                state.theta + dtheta
            )
        else:
            new_state = SE2(state.x, state.y, state.theta)
            
        return new_state, new_speed