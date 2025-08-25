"""
Vehicle dynamics model for MPC car tracking using bicycle model.

This module implements the kinematic bicycle model for vehicle dynamics,
which is commonly used in path tracking and autonomous driving applications.
"""

import numpy as np
from typing import Tuple, Optional
from dataclasses import dataclass


@dataclass
class VehicleState:
    """Vehicle state representation.
    
    Attributes:
        x: Position in x-direction (m)
        y: Position in y-direction (m)
        yaw: Heading angle (rad)
        v: Velocity (m/s)
    """
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    v: float = 0.0
    
    def to_array(self) -> np.ndarray:
        """Convert state to numpy array."""
        return np.array([self.x, self.y, self.yaw, self.v])
    
    @classmethod
    def from_array(cls, state_array: np.ndarray) -> 'VehicleState':
        """Create VehicleState from numpy array."""
        return cls(x=state_array[0], y=state_array[1], 
                  yaw=state_array[2], v=state_array[3])


@dataclass
class VehicleControl:
    """Vehicle control inputs.
    
    Attributes:
        acceleration: Longitudinal acceleration (m/s²)
        steering_angle: Front wheel steering angle (rad)
    """
    acceleration: float = 0.0
    steering_angle: float = 0.0
    
    def to_array(self) -> np.ndarray:
        """Convert control to numpy array."""
        return np.array([self.acceleration, self.steering_angle])


@dataclass
class VehicleParameters:
    """Vehicle physical parameters.
    
    Attributes:
        wheelbase: Distance between front and rear axles (m)
        max_speed: Maximum vehicle speed (m/s)
        max_acceleration: Maximum acceleration (m/s²)
        max_deceleration: Maximum deceleration (m/s²)
        max_steering_angle: Maximum steering angle (rad)
        max_steering_rate: Maximum steering rate (rad/s)
    """
    wheelbase: float = 2.7  # Typical car wheelbase
    max_speed: float = 30.0  # 30 m/s ≈ 108 km/h
    max_acceleration: float = 3.0  # m/s²
    max_deceleration: float = -5.0  # m/s²
    max_steering_angle: float = np.pi / 3  # 60 degrees
    max_steering_rate: float = np.pi / 2  # 90 degrees/s


class BicycleModel:
    """Kinematic bicycle model for vehicle dynamics.
    
    The bicycle model is a simplified vehicle model that assumes:
    - No tire slip
    - Constant velocity or controlled acceleration
    - Front wheel steering
    - Kinematic (not dynamic) behavior
    """
    
    def __init__(self, params: Optional[VehicleParameters] = None):
        """Initialize bicycle model.
        
        Args:
            params: Vehicle parameters. If None, default parameters are used.
        """
        self.params = params or VehicleParameters()
    
    def step(self, state: VehicleState, control: VehicleControl, dt: float) -> VehicleState:
        """Simulate one time step of vehicle dynamics.
        
        Args:
            state: Current vehicle state
            control: Control inputs
            dt: Time step (s)
            
        Returns:
            New vehicle state after one time step
        """
        # Apply control constraints
        control = self._apply_constraints(control, state)
        
        # Kinematic bicycle model equations
        x_dot = state.v * np.cos(state.yaw)
        y_dot = state.v * np.sin(state.yaw)
        yaw_dot = (state.v / self.params.wheelbase) * np.tan(control.steering_angle)
        v_dot = control.acceleration
        
        # Update state using Euler integration
        new_state = VehicleState(
            x=state.x + x_dot * dt,
            y=state.y + y_dot * dt,
            yaw=state.yaw + yaw_dot * dt,
            v=state.v + v_dot * dt
        )
        
        # Normalize yaw angle to [-pi, pi]
        new_state.yaw = self._normalize_angle(new_state.yaw)
        
        # Apply velocity constraints
        new_state.v = np.clip(new_state.v, 0.0, self.params.max_speed)
        
        return new_state
    
    def linearize(self, state: VehicleState, control: VehicleControl, dt: float) -> Tuple[np.ndarray, np.ndarray]:
        """Linearize the bicycle model around the given state and control.
        
        This is used for MPC to create linear approximations of the dynamics.
        
        Args:
            state: Operating point state
            control: Operating point control
            dt: Time step
            
        Returns:
            Tuple of (A_matrix, B_matrix) for linearized system x_{k+1} = A*x_k + B*u_k
        """
        # State: [x, y, yaw, v]
        # Control: [acceleration, steering_angle]
        
        # Continuous time Jacobians
        cos_yaw = np.cos(state.yaw)
        sin_yaw = np.sin(state.yaw)
        v = state.v
        L = self.params.wheelbase
        delta = control.steering_angle
        
        # A matrix (4x4) - derivative w.r.t. state
        A_c = np.array([
            [0, 0, -v * sin_yaw, cos_yaw],
            [0, 0, v * cos_yaw, sin_yaw],
            [0, 0, 0, np.tan(delta) / L],
            [0, 0, 0, 0]
        ])
        
        # B matrix (4x2) - derivative w.r.t. control
        B_c = np.array([
            [0, 0],
            [0, 0],
            [0, v / (L * np.cos(delta)**2)],
            [1, 0]
        ])
        
        # Convert to discrete time using Euler integration
        A_d = np.eye(4) + A_c * dt
        B_d = B_c * dt
        
        return A_d, B_d
    
    def _apply_constraints(self, control: VehicleControl, state: VehicleState) -> VehicleControl:
        """Apply physical constraints to control inputs."""
        # Steering angle constraints
        steering_angle = np.clip(control.steering_angle, 
                               -self.params.max_steering_angle, 
                               self.params.max_steering_angle)
        
        # Acceleration constraints (considering current velocity)
        if state.v <= 0 and control.acceleration < 0:
            # Don't allow negative acceleration when stopped
            acceleration = 0.0
        else:
            acceleration = np.clip(control.acceleration,
                                 self.params.max_deceleration,
                                 self.params.max_acceleration)
        
        return VehicleControl(acceleration=acceleration, steering_angle=steering_angle)
    
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi] range."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def get_state_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get state variable bounds for optimization.
        
        Returns:
            Tuple of (lower_bounds, upper_bounds) for [x, y, yaw, v]
        """
        lower_bounds = np.array([-np.inf, -np.inf, -np.pi, 0.0])
        upper_bounds = np.array([np.inf, np.inf, np.pi, self.params.max_speed])
        return lower_bounds, upper_bounds
    
    def get_control_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        """Get control input bounds for optimization.
        
        Returns:
            Tuple of (lower_bounds, upper_bounds) for [acceleration, steering_angle]
        """
        lower_bounds = np.array([self.params.max_deceleration, -self.params.max_steering_angle])
        upper_bounds = np.array([self.params.max_acceleration, self.params.max_steering_angle])
        return lower_bounds, upper_bounds