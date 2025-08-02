"""
Vehicle Dynamics Model

A comprehensive vehicle dynamics model for simulation and control testing.
Includes bicycle model, realistic vehicle parameters, and actuator dynamics.
"""

import numpy as np
from typing import Tuple, Optional
from dataclasses import dataclass
import math

from .utils.se2 import SE2


@dataclass
class VehicleParameters:
    """Vehicle physical parameters"""
    # Dimensions
    wheelbase: float = 2.9  # Distance between front and rear axles [m]
    track_width: float = 1.6  # Distance between left and right wheels [m]
    front_overhang: float = 1.0  # Distance from front axle to front bumper [m]
    rear_overhang: float = 1.0  # Distance from rear axle to rear bumper [m]
    vehicle_width: float = 2.0  # Vehicle width [m]
    
    # Mass properties
    mass: float = 1500.0  # Vehicle mass [kg]
    inertia_z: float = 2875.0  # Yaw moment of inertia [kg*m^2]
    
    # Aerodynamic properties
    drag_coefficient: float = 0.3  # Aerodynamic drag coefficient
    frontal_area: float = 2.5  # Frontal area [m^2]
    air_density: float = 1.225  # Air density [kg/m^3]
    
    # Tire properties
    cornering_stiffness_front: float = 80000.0  # Front tire cornering stiffness [N/rad]
    cornering_stiffness_rear: float = 100000.0  # Rear tire cornering stiffness [N/rad]
    friction_coefficient: float = 0.9  # Tire-road friction coefficient
    
    # Actuator limits
    max_steering_angle: float = np.radians(30.0)  # Maximum steering angle [rad]
    max_steering_rate: float = np.radians(60.0)  # Maximum steering rate [rad/s]
    max_acceleration: float = 3.0  # Maximum acceleration [m/s^2]
    max_deceleration: float = -8.0  # Maximum deceleration [m/s^2]
    max_brake_torque: float = 3000.0  # Maximum brake torque [N*m]


class VehicleDynamics:
    """
    Vehicle dynamics model using bicycle model with enhanced features.
    
    This model includes:
    - Bicycle kinematic model
    - Tire slip dynamics
    - Actuator dynamics
    - Aerodynamic drag
    - Realistic vehicle parameters
    """
    
    def __init__(self, params: VehicleParameters):
        """
        Initialize vehicle dynamics.
        
        Args:
            params: Vehicle parameters
        """
        self.params = params
        
        # State variables
        self.reset()
        
    def reset(self):
        """Reset vehicle state to initial conditions."""
        self.x = 0.0  # x position [m]
        self.y = 0.0  # y position [m]
        self.yaw = 0.0  # yaw angle [rad]
        self.vx = 0.0  # longitudinal velocity [m/s]
        self.vy = 0.0  # lateral velocity [m/s]
        self.yaw_rate = 0.0  # yaw rate [rad/s]
        
        # Control inputs
        self.steering_angle = 0.0  # steering angle [rad]
        self.throttle = 0.0  # throttle input [0, 1]
        self.brake = 0.0  # brake input [0, 1]
        
        # Actuator states
        self.actual_steering_angle = 0.0  # actual steering angle [rad]
        self.actual_throttle = 0.0  # actual throttle [0, 1]
        self.actual_brake = 0.0  # actual brake [0, 1]
        
    def update(self, dt: float, steering_command: float, throttle_command: float, brake_command: float):
        """
        Update vehicle state for one time step.
        
        Args:
            dt: Time step [s]
            steering_command: Desired steering angle [rad]
            throttle_command: Desired throttle [0, 1]
            brake_command: Desired brake [0, 1]
        """
        # Update actuator dynamics (first-order lag)
        self._update_actuators(dt, steering_command, throttle_command, brake_command)
        
        # Calculate forces and moments
        fx, fy, mz = self._calculate_forces_moments()
        
        # Update velocities using bicycle model
        self._update_velocities(dt, fx, fy, mz)
        
        # Update position and orientation
        self._update_position(dt)
        
    def _update_actuators(self, dt: float, steering_command: float, throttle_command: float, brake_command: float):
        """Update actuator states with realistic dynamics."""
        # Steering actuator dynamics
        steering_error = steering_command - self.actual_steering_angle
        max_steering_change = self.params.max_steering_rate * dt
        steering_change = np.clip(steering_error, -max_steering_change, max_steering_change)
        self.actual_steering_angle += steering_change
        self.actual_steering_angle = np.clip(self.actual_steering_angle, 
                                           -self.params.max_steering_angle, 
                                           self.params.max_steering_angle)
        
        # Throttle actuator dynamics
        throttle_error = throttle_command - self.actual_throttle
        throttle_rate = 2.0  # Throttle response rate [1/s]
        self.actual_throttle += throttle_error * throttle_rate * dt
        self.actual_throttle = np.clip(self.actual_throttle, 0.0, 1.0)
        
        # Brake actuator dynamics
        brake_error = brake_command - self.actual_brake
        brake_rate = 3.0  # Brake response rate [1/s]
        self.actual_brake += brake_error * brake_rate * dt
        self.actual_brake = np.clip(self.actual_brake, 0.0, 1.0)
        
    def _calculate_forces_moments(self) -> Tuple[float, float, float]:
        """
        Calculate forces and moments acting on the vehicle.
        
        Returns:
            Tuple of (fx, fy, mz) - longitudinal force, lateral force, yaw moment
        """
        # Vehicle speed
        speed = np.sqrt(self.vx**2 + self.vy**2)
        
        # Tire slip angles
        alpha_f = self._calculate_front_slip_angle()
        alpha_r = self._calculate_rear_slip_angle()
        
        # Tire forces (linear tire model)
        Fyf = self.params.cornering_stiffness_front * alpha_f
        Fyr = self.params.cornering_stiffness_rear * alpha_r
        
        # Longitudinal forces
        # Engine force from throttle
        engine_force = self.actual_throttle * 5000.0  # Max engine force [N]
        
        # Brake force
        brake_force = self.actual_brake * self.params.max_brake_torque / (self.params.wheelbase / 2)
        
        # Aerodynamic drag
        drag_force = 0.5 * self.params.air_density * self.params.drag_coefficient * \
                    self.params.frontal_area * speed * self.vx
        
        # Total longitudinal force
        fx = engine_force + brake_force - drag_force
        
        # Lateral forces
        fy = Fyf + Fyr
        
        # Yaw moment
        mz = self.params.wheelbase * (Fyf - Fyr) / 2.0
        
        # Apply friction limits
        max_friction_force = self.params.friction_coefficient * self.params.mass * 9.81
        if abs(fx) > max_friction_force:
            fx = np.sign(fx) * max_friction_force
        if abs(fy) > max_friction_force:
            fy = np.sign(fy) * max_friction_force
            
        return fx, fy, mz
    
    def _calculate_front_slip_angle(self) -> float:
        """Calculate front tire slip angle."""
        if abs(self.vx) < 0.1:  # Avoid division by zero
            return 0.0
        
        # Slip angle at front tires
        alpha_f = self.actual_steering_angle - np.arctan2(self.vy + self.params.wheelbase/2 * self.yaw_rate, self.vx)
        return alpha_f
    
    def _calculate_rear_slip_angle(self) -> float:
        """Calculate rear tire slip angle."""
        if abs(self.vx) < 0.1:  # Avoid division by zero
            return 0.0
        
        # Slip angle at rear tires
        alpha_r = -np.arctan2(self.vy - self.params.wheelbase/2 * self.yaw_rate, self.vx)
        return alpha_r
    
    def _update_velocities(self, dt: float, fx: float, fy: float, mz: float):
        """Update vehicle velocities using Newton's laws."""
        # Accelerations
        ax = fx / self.params.mass
        ay = fy / self.params.mass
        alpha_z = mz / self.params.inertia_z
        
        # Update velocities
        self.vx += ax * dt
        self.vy += ay * dt
        self.yaw_rate += alpha_z * dt
        
        # Prevent negative speed (vehicle can't go backwards in this simple model)
        speed = np.sqrt(self.vx**2 + self.vy**2)
        if speed < 0.1:
            self.vx = 0.1
            self.vy = 0.0
    
    def _update_position(self, dt: float):
        """Update vehicle position and orientation."""
        # Convert body velocities to world frame
        cos_yaw = np.cos(self.yaw)
        sin_yaw = np.sin(self.yaw)
        
        # World frame velocities
        vx_world = self.vx * cos_yaw - self.vy * sin_yaw
        vy_world = self.vx * sin_yaw + self.vy * cos_yaw
        
        # Update position
        self.x += vx_world * dt
        self.y += vy_world * dt
        
        # Update orientation
        self.yaw += self.yaw_rate * dt
        self.yaw = self._normalize_angle(self.yaw)
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def get_state(self) -> SE2:
        """
        Get current vehicle state as SE2 transformation.
        
        Returns:
            SE2 transformation representing vehicle pose
        """
        return SE2(self.x, self.y, self.yaw)
    
    def get_speed(self) -> float:
        """Get current vehicle speed [m/s]."""
        return np.sqrt(self.vx**2 + self.vy**2)
    
    def get_acceleration_limits(self) -> Tuple[float, float]:
        """
        Get acceleration limits based on current speed.
        
        Returns:
            Tuple of (max_acceleration, max_deceleration)
        """
        speed = self.get_speed()
        
        # Reduce acceleration at high speeds
        max_acc = self.params.max_acceleration * (1.0 - 0.1 * min(speed / 30.0, 1.0))
        max_dec = self.params.max_deceleration * (1.0 - 0.1 * min(speed / 30.0, 1.0))
        
        return max_acc, max_dec
    
    def get_vehicle_corners(self) -> np.ndarray:
        """
        Get vehicle corner coordinates in world frame.
        
        Returns:
            Array of corner points [4 x 2]
        """
        # Vehicle corners in vehicle frame
        half_length = (self.params.wheelbase + self.params.front_overhang + self.params.rear_overhang) / 2
        half_width = self.params.vehicle_width / 2
        
        corners = np.array([
            [-half_length + self.params.rear_overhang, -half_width],  # Rear left
            [half_length - self.params.front_overhang, -half_width],   # Front left
            [half_length - self.params.front_overhang, half_width],    # Front right
            [-half_length + self.params.rear_overhang, half_width],    # Rear right
        ])
        
        # Transform to world frame
        cos_yaw = np.cos(self.yaw)
        sin_yaw = np.sin(self.yaw)
        rotation_matrix = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
        
        world_corners = corners @ rotation_matrix.T
        world_corners[:, 0] += self.x
        world_corners[:, 1] += self.y
        
        return world_corners


class SimpleVehicleDynamics:
    """
    Simplified vehicle dynamics model for basic simulation.
    
    Uses a simple bicycle model without tire dynamics or actuator lag.
    """
    
    def __init__(self, wheelbase: float = 2.9):
        """
        Initialize simplified vehicle dynamics.
        
        Args:
            wheelbase: Vehicle wheelbase [m]
        """
        self.wheelbase = wheelbase
        self.reset()
        
    def reset(self):
        """Reset vehicle state."""
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.speed = 0.0
        
    def set_state(self, state: SE2, speed: float = 0.0):
        """Set vehicle state."""
        self.x = state.x
        self.y = state.y
        self.yaw = state.theta
        self.speed = speed
        
    def update(self, dt: float, steering: float, acceleration: float):
        """
        Update vehicle state with simple bicycle model.
        
        Args:
            dt: Time step [s]
            steering: Steering angle [rad]
            acceleration: Acceleration [m/s^2]
        """
        # Update speed
        self.speed += acceleration * dt
        self.speed = max(0, self.speed)  # Prevent negative speed
        
        # Update position and heading
        if self.speed > 0:
            # Standard bicycle model
            dx = self.speed * np.cos(self.yaw) * dt
            dy = self.speed * np.sin(self.yaw) * dt
            dtheta = (self.speed / self.wheelbase) * np.tan(steering) * dt
            
            self.x += dx
            self.y += dy
            self.yaw += dtheta
            self.yaw = self._normalize_angle(self.yaw)
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-pi, pi]."""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def get_state(self) -> SE2:
        """Get current vehicle state as SE2 transformation."""
        return SE2(self.x, self.y, self.yaw)
    
    def get_speed(self) -> float:
        """Get current vehicle speed [m/s]."""
        return self.speed