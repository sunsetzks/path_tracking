"""
Vehicle Dynamics Simulation using Bicycle Model

This module implements a vehicle dynamics simulation based on the bicycle model,
which is commonly used in vehicle dynamics and control applications.
"""

import numpy as np
from typing import Tuple, Optional
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle


class VehicleParameters:
    """Vehicle physical parameters"""
    def __init__(self, 
                 length: float = 4.0,      # Vehicle length [m]
                 width: float = 1.8,       # Vehicle width [m]
                 wheelbase: float = 2.7,   # Wheelbase (distance between front and rear axles) [m]
                 mass: float = 1500.0,     # Vehicle mass [kg]
                 max_steer: float = 0.6,   # Maximum steering angle [rad]
                 max_accel: float = 3.0,   # Maximum acceleration [m/s^2]
                 max_decel: float = -6.0,  # Maximum deceleration [m/s^2]
                 max_speed: float = 50.0,  # Maximum speed [m/s]
                 dt: float = 0.05):        # Simulation timestep [s]
        self.length = length
        self.width = width
        self.wheelbase = wheelbase
        self.mass = mass
        self.max_steer = max_steer
        self.max_accel = max_accel
        self.max_decel = max_decel
        self.max_speed = max_speed
        self.dt = dt


class VehicleState:
    """Vehicle state representation"""
    def __init__(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0,
                 v: float = 0.0, steer: float = 0.0):
        self.x = x           # X position [m]
        self.y = y           # Y position [m]
        self.yaw = yaw       # Yaw angle [rad]
        self.v = v           # Velocity [m/s]
        self.steer = steer   # Steering angle [rad]
    
    def update(self, a: float, delta_dot: float, params: VehicleParameters):
        """
        Update vehicle state using bicycle model dynamics
        
        Args:
            a: Acceleration [m/s^2]
            delta_dot: Steering angular velocity [rad/s]
            params: Vehicle parameters
        """
        # Limit acceleration and steering rate
        a = np.clip(a, params.max_decel, params.max_accel)
        v = np.clip(self.v + a * params.dt, 0, params.max_speed)
        
        # Update steering angle with rate limit
        self.steer += delta_dot * params.dt
        self.steer = np.clip(self.steer, -params.max_steer, params.max_steer)
        
        # Update velocity
        self.v = v
        
        # Update position and yaw using bicycle model
        self.x += self.v * np.cos(self.yaw) * params.dt
        self.y += self.v * np.sin(self.yaw) * params.dt
        self.yaw += (self.v / params.wheelbase) * np.tan(self.steer) * params.dt
        
        # Normalize yaw angle to [-pi, pi]
        self.yaw = np.arctan2(np.sin(self.yaw), np.cos(self.yaw))
    
    def copy(self):
        """Create a copy of the vehicle state"""
        return VehicleState(self.x, self.y, self.yaw, self.v, self.steer)
        """
        Update vehicle state using bicycle model dynamics
        
        Args:
            a: Acceleration [m/s^2]
            delta_dot: Steering angular velocity [rad/s]
            params: Vehicle parameters
        """
        # Limit acceleration and steering rate
        a = np.clip(a, params.max_decel, params.max_accel)
        v = np.clip(self.v + a * params.dt, 0, params.max_speed)
        
        # Update steering angle with rate limit
        self.steer += delta_dot * params.dt
        self.steer = np.clip(self.steer, -params.max_steer, params.max_steer)
        
        # Update velocity
        self.v = v
        
        # Update position and yaw using bicycle model
        self.x += self.v * np.cos(self.yaw) * params.dt
        self.y += self.v * np.sin(self.yaw) * params.dt
        self.yaw += (self.v / params.wheelbase) * np.tan(self.steer) * params.dt
        
        # Normalize yaw angle to [-pi, pi]
        self.yaw = np.arctan2(np.sin(self.yaw), np.cos(self.yaw))


def kinematic_bicycle_model(state: VehicleState, a: float, delta_dot: float, 
                           params: VehicleParameters) -> VehicleState:
    """
    Kinematic bicycle model for vehicle dynamics
    
    Args:
        state: Current vehicle state
        a: Acceleration [m/s^2]
        delta_dot: Steering angular velocity [rad/s]
        params: Vehicle parameters
        
    Returns:
        Updated vehicle state
    """
    new_state = VehicleState(state.x, state.y, state.yaw, state.v, state.steer)
    new_state.update(a, delta_dot, params)
    return new_state


class VehicleSimulation:
    """Vehicle simulation environment"""
    def __init__(self, params: Optional[VehicleParameters] = None):
        self.params = params or VehicleParameters()
        self.state = VehicleState()
        self.history = {
            't': [0.0],
            'x': [0.0],
            'y': [0.0],
            'yaw': [0.0],
            'v': [0.0],
            'steer': [0.0],
            'a': [0.0],
            'delta_dot': [0.0]
        }
    
    def reset(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0, 
              v: float = 0.0, steer: float = 0.0):
        """Reset vehicle state"""
        self.state = VehicleState(x, y, yaw, v, steer)
        self.history = {
            't': [0.0],
            'x': [x],
            'y': [y],
            'yaw': [yaw],
            'v': [v],
            'steer': [steer],
            'a': [0.0],
            'delta_dot': [0.0]
        }
    
    def step(self, a: float, delta_dot: float) -> VehicleState:
        """
        Execute one simulation step
        
        Args:
            a: Acceleration [m/s^2]
            delta_dot: Steering angular velocity [rad/s]
            
        Returns:
            Updated vehicle state
        """
        # Store control inputs
        current_t = self.history['t'][-1] + self.params.dt
        
        # Update state
        self.state.update(a, delta_dot, self.params)
        
        # Store history
        self.history['t'].append(current_t)
        self.history['x'].append(self.state.x)
        self.history['y'].append(self.state.y)
        self.history['yaw'].append(self.state.yaw)
        self.history['v'].append(self.state.v)
        self.history['steer'].append(self.state.steer)
        self.history['a'].append(a)
        self.history['delta_dot'].append(delta_dot)
        
        return self.state.copy() if hasattr(self.state, 'copy') else self.state
    
    def get_state_array(self) -> np.ndarray:
        """Get current state as numpy array [x, y, yaw, v, steer]"""
        return np.array([self.state.x, self.state.y, self.state.yaw, 
                        self.state.v, self.state.steer])
    
    def get_history_array(self) -> np.ndarray:
        """Get simulation history as numpy array"""
        return np.array([self.history['t'], self.history['x'], self.history['y'], 
                        self.history['yaw'], self.history['v'], self.history['steer'],
                        self.history['a'], self.history['delta_dot']]).T


def simulate_trajectory(sim: VehicleSimulation, T: float, 
                       control_func) -> VehicleSimulation:
    """
    Simulate vehicle trajectory for a given time period
    
    Args:
        sim: Vehicle simulation instance
        T: Simulation duration [s]
        control_func: Function that takes (t, state) and returns (a, delta_dot)
        
    Returns:
        Simulated vehicle simulation instance
    """
    n_steps = int(T / sim.params.dt)
    
    for i in range(n_steps):
        t = i * sim.params.dt
        a, delta_dot = control_func(t, sim.state)
        sim.step(a, delta_dot)
    
    return sim