"""
Vehicle State Estimators

This module implements various vehicle state estimators for simulating different
sensor characteristics and noise models:
- OdometryEstimator: Dead reckoning with accumulated drift
- GlobalLocalizationEstimator: GPS-like positioning with bounded noise
- SimpleNoiseEstimator: Basic Gaussian noise without accumulation
- VehicleStateManager: Unified manager for all state types

Author: Assistant
"""

import math
from collections import deque
from typing import Tuple, Optional, TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    from .config import EstimatorConfig

from .vehicle_state import VehicleState


class OdometryEstimator:
    """
    Odometry-based state estimator for dead reckoning
    
    This estimator simulates odometry sensors (wheel encoders, IMU) that compute
    motion increments and accumulate small errors over time, leading to drift in 
    position estimation. This is the correct implementation of odometry that:
    1. Calculates motion deltas from the previous state
    2. Adds noise to these deltas
    3. Integrates the noisy deltas to get the new odometry state
    """

    def __init__(self, config: "EstimatorConfig"):
        """
        Initialize odometry estimator

        Args:
            config (EstimatorConfig): Estimator configuration containing noise parameters
        """
        self.config = config
        self.noise_enabled = True  # Odometry noise is always enabled

        # Initialize random number generator with seed for reproducibility
        if config.noise_seed is not None:
            self.rng = np.random.RandomState(config.noise_seed)
        else:
            self.rng = np.random.RandomState()

        # Odometry state (accumulates errors over time)
        self.odometry_state = VehicleState()
        
        # Previous true state for delta calculation
        self.previous_true_state = VehicleState()

    def update_odometry_state(self, true_state: "VehicleState", time_step: float) -> "VehicleState":
        """
        Update odometry state based on motion deltas with noise

        Args:
            true_state (VehicleState): True vehicle state (without noise)
            time_step (float): Time step for integration

        Returns:
            VehicleState: Updated odometry state
        """
        if not self.noise_enabled:
            self.previous_true_state = true_state.copy()
            return true_state.copy()

        # Calculate motion deltas from previous true state
        delta_x = true_state.position_x - self.previous_true_state.position_x
        delta_y = true_state.position_y - self.previous_true_state.position_y
        delta_yaw = self._normalize_angle(true_state.yaw_angle - self.previous_true_state.yaw_angle)
        
        # Add noise to motion deltas (this is how odometry errors accumulate)
        # Position noise proportional to distance traveled
        distance_traveled = math.sqrt(delta_x**2 + delta_y**2)
        position_noise_std = self.config.odometry_position_noise_std * max(distance_traveled, time_step)
        
        delta_x_noise = self.rng.normal(0, position_noise_std)
        delta_y_noise = self.rng.normal(0, position_noise_std)
        
        # Yaw noise proportional to angular change
        yaw_noise_std = self.config.odometry_yaw_noise_std * max(abs(delta_yaw), time_step)
        delta_yaw_noise = self.rng.normal(0, yaw_noise_std)
        
        # Apply noisy deltas to odometry state
        self.odometry_state.position_x += delta_x + delta_x_noise
        self.odometry_state.position_y += delta_y + delta_y_noise
        self.odometry_state.yaw_angle = self._normalize_angle(
            self.odometry_state.yaw_angle + delta_yaw + delta_yaw_noise
        )
        
        # Velocity and steering angle with direct noise (measured locally)
        velocity_noise = self.rng.normal(0, self.config.odometry_velocity_noise_std)
        self.odometry_state.velocity = true_state.velocity + velocity_noise
        self.odometry_state.steering_angle = true_state.steering_angle  # Assume directly measured
        
        # Update previous state for next iteration
        self.previous_true_state = true_state.copy()
        
        return self.odometry_state.copy()

    @staticmethod
    def _normalize_angle(angle):
        """
        Normalize angle to [-pi, pi]

        Args:
            angle (float): angle in radians

        Returns:
            float: normalized angle
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def reset_state(self, initial_state: "VehicleState"):
        """
        Reset odometry estimator to initial state

        Args:
            initial_state (VehicleState): Initial state to reset to
        """
        self.odometry_state = initial_state.copy()
        self.previous_true_state = initial_state.copy()

    def set_noise_enabled(self, enabled: bool):
        """
        Enable or disable noise generation

        Args:
            enabled (bool): Whether to enable noise
        """
        self.noise_enabled = enabled

    def reset_seed(self, seed: Optional[int] = None):
        """
        Reset the random number generator with a new seed

        Args:
            seed (Optional[int]): New seed value, None for random seed
        """
        if seed is not None:
            self.rng = np.random.RandomState(seed)
        else:
            self.rng = np.random.RandomState()


class GlobalLocalizationEstimator:
    """
    Global localization estimator for GPS-like positioning systems
    
    This estimator simulates global positioning systems (GPS, GNSS) characteristics:
    - Bounded noise that doesn't accumulate over time
    - Periodic corrections to prevent drift
    - Measurement delays and update frequencies
    - Separate noise characteristics for position and orientation
    """

    def __init__(self, config: "EstimatorConfig"):
        """
        Initialize global localization estimator

        Args:
            config (EstimatorConfig): Estimator configuration containing noise parameters
        """
        self.config = config
        self.noise_enabled = True  # Global localization noise is always enabled
        
        # Initialize random number generator with seed for reproducibility
        if config.noise_seed is not None:
            self.rng = np.random.RandomState(config.noise_seed)
        else:
            self.rng = np.random.RandomState()
        
        # Global localization state
        self.global_state = VehicleState()
        self.last_global_measurement_time = 0.0
        self.global_measurement_interval = 1.0 / config.global_measurement_frequency
        
        # Persistent noise offsets (simulate GPS bias)
        self.position_bias = self.rng.normal(0, config.global_position_noise_std * 0.3, 2)
        self.yaw_bias = self.rng.normal(0, config.global_yaw_noise_std * 0.3)
        
        # Measurement buffer for delayed measurements
        self.measurement_buffer = deque()

    def update_global_state(self, true_state: "VehicleState", current_time: float) -> "VehicleState":
        """
        Update global localization state with GPS-like characteristics

        Args:
            true_state (VehicleState): True vehicle state
            current_time (float): Current simulation time

        Returns:
            VehicleState: Updated global localization state
        """
        if not self.noise_enabled:
            return true_state.copy()

        # Generate global measurement if it's time for one
        if current_time - self.last_global_measurement_time >= self.global_measurement_interval:
            self.last_global_measurement_time = current_time
            
            # Generate noisy global measurement
            position_noise = self.rng.normal(0, self.config.global_position_noise_std, 2)
            yaw_noise = self.rng.normal(0, self.config.global_yaw_noise_std)
            
            # Add to measurement buffer with delay
            measurement = (
                true_state.position_x + self.position_bias[0] + position_noise[0],
                true_state.position_y + self.position_bias[1] + position_noise[1],
                true_state.yaw_angle + self.yaw_bias + yaw_noise,
                current_time + self.config.global_measurement_delay
            )
            self.measurement_buffer.append(measurement)

        # Process delayed measurements
        while self.measurement_buffer and self.measurement_buffer[0][3] <= current_time:
            delayed_measurement = self.measurement_buffer.popleft()
            
            # Update global state with delayed measurement
            self.global_state.position_x = delayed_measurement[0]
            self.global_state.position_y = delayed_measurement[1]
            self.global_state.yaw_angle = delayed_measurement[2]

        # Use current velocity and steering angle (assume these are measured locally)
        # Note: Global localization doesn't typically provide velocity directly
        self.global_state.velocity = true_state.velocity
        self.global_state.steering_angle = true_state.steering_angle

        return self.global_state.copy()

    def reset_state(self, initial_state: "VehicleState"):
        """
        Reset global localization estimator to initial state

        Args:
            initial_state (VehicleState): Initial state to reset to
        """
        self.global_state = initial_state.copy()
        self.last_global_measurement_time = 0.0
        self.measurement_buffer.clear()
        
        # Reset bias
        self.position_bias = self.rng.normal(0, self.config.global_position_noise_std * 0.3, 2)
        self.yaw_bias = self.rng.normal(0, self.config.global_yaw_noise_std * 0.3)

    def set_noise_enabled(self, enabled: bool):
        """
        Enable or disable noise generation

        Args:
            enabled (bool): Whether to enable noise
        """
        self.noise_enabled = enabled

    def reset_seed(self, seed: Optional[int] = None):
        """
        Reset the random number generator with a new seed

        Args:
            seed (Optional[int]): New seed value, None for random seed
        """
        if seed is not None:
            self.rng = np.random.RandomState(seed)
        else:
            self.rng = np.random.RandomState()


class SimpleNoiseEstimator:
    """
    Simple noise estimator that adds Gaussian noise to true position
    
    This estimator provides a basic noise model that simply adds independent
    Gaussian noise to the position coordinates at each time step. Unlike
    odometry noise, this noise doesn't accumulate over time, and unlike
    global localization, it doesn't have delays or measurement intervals.
    """

    def __init__(self, config: "EstimatorConfig"):
        """
        Initialize simple noise estimator

        Args:
            config (EstimatorConfig): Estimator configuration containing noise parameters
        """
        self.config = config
        self.noise_enabled = True  # Simple noise is always enabled when used
        
        # Initialize random number generator with seed for reproducibility
        if config.noise_seed is not None:
            self.rng = np.random.RandomState(config.noise_seed)
        else:
            self.rng = np.random.RandomState()
        
        # Simple noise state
        self.simple_state = VehicleState()

    def update_simple_state(self, true_state: "VehicleState") -> "VehicleState":
        """
        Update simple noise state by adding Gaussian noise to true position

        Args:
            true_state (VehicleState): True vehicle state

        Returns:
            VehicleState: Updated simple noise state
        """
        if not self.noise_enabled:
            return true_state.copy()

        # Copy true state
        self.simple_state = true_state.copy()
        
        # Add Gaussian noise to position coordinates
        position_noise = self.rng.normal(0, self.config.simple_position_noise_std, 2)
        self.simple_state.position_x += position_noise[0]
        self.simple_state.position_y += position_noise[1]
        
        # Add Gaussian noise to yaw angle
        yaw_noise = self.rng.normal(0, self.config.simple_yaw_noise_std)
        self.simple_state.yaw_angle += yaw_noise
        
        # Add Gaussian noise to velocity
        velocity_noise = self.rng.normal(0, self.config.simple_velocity_noise_std)
        self.simple_state.velocity += velocity_noise
        
        # Steering angle remains the same (assume directly measured)
        # self.simple_state.steering_angle = true_state.steering_angle

        return self.simple_state.copy()

    def reset_state(self, initial_state: "VehicleState"):
        """
        Reset simple noise estimator to initial state

        Args:
            initial_state (VehicleState): Initial state to reset to
        """
        self.simple_state = initial_state.copy()

    def set_noise_enabled(self, enabled: bool):
        """
        Enable or disable noise generation

        Args:
            enabled (bool): Whether to enable noise
        """
        self.noise_enabled = enabled

    def reset_seed(self, seed: Optional[int] = None):
        """
        Reset the random number generator with a new seed

        Args:
            seed (Optional[int]): New seed value, None for random seed
        """
        if seed is not None:
            self.rng = np.random.RandomState(seed)
        else:
            self.rng = np.random.RandomState()


class VehicleStateManager:
    """
    Manages all four types of vehicle states: true, odometry, global localization, and simple noise
    
    This class coordinates the different state estimators and provides a unified interface
    for accessing different state types.
    """

    def __init__(self, config: "EstimatorConfig"):
        """
        Initialize vehicle state manager

        Args:
            config (EstimatorConfig): Estimator configuration containing noise parameters
        """
        self.config = config
        
        # Initialize state estimators
        self.odometry_estimator = OdometryEstimator(config)
        self.global_estimator = GlobalLocalizationEstimator(config)
        self.simple_estimator = SimpleNoiseEstimator(config)
        
        # Control input noise generator
        self.control_noise_enabled = config.control_input_noise_enabled
        if config.noise_seed is not None:
            self.control_rng = np.random.RandomState(config.noise_seed)
        else:
            self.control_rng = np.random.RandomState()
        
        # True state (clean state without noise)
        self.true_state = VehicleState()
        
        # Current simulation time
        self.simulation_time = 0.0

    def update_all_states(self, true_state: "VehicleState", time_step: float):
        """
        Update all state estimates based on the true state

        Args:
            true_state (VehicleState): True vehicle state (without noise)
            time_step (float): Time step for integration
        """
        self.true_state = true_state.copy()
        self.simulation_time += time_step
        
        # Update odometry, global, and simple estimates
        self.odometry_estimator.update_odometry_state(true_state, time_step)
        self.global_estimator.update_global_state(true_state, self.simulation_time)
        self.simple_estimator.update_simple_state(true_state)

    def get_state(self, state_type: Optional[str] = None) -> "VehicleState":
        """
        Get vehicle state of specified type

        Args:
            state_type (Optional[str]): Type of state to return ("true", "odometry", "global", "simple")
                                       If None, uses default from config

        Returns:
            VehicleState: Requested state type
        """
        if state_type is None:
            state_type = self.config.default_state_type
            
        if state_type == "true":
            return self.true_state.copy()
        elif state_type == "odometry":
            return self.odometry_estimator.odometry_state.copy()
        elif state_type == "global":
            return self.global_estimator.global_state.copy()
        elif state_type == "simple":
            return self.simple_estimator.simple_state.copy()
        else:
            raise ValueError(f"Unknown state type: {state_type}. Must be 'true', 'odometry', 'global', or 'simple'")

    def get_true_state(self) -> "VehicleState":
        """Get true vehicle state (without noise)"""
        return self.true_state.copy()

    def get_odometry_state(self) -> "VehicleState":
        """Get odometry-based state estimate"""
        return self.odometry_estimator.odometry_state.copy()

    def get_global_state(self) -> "VehicleState":
        """Get global localization state estimate"""
        return self.global_estimator.global_state.copy()

    def get_simple_state(self) -> "VehicleState":
        """Get simple noise state estimate"""
        return self.simple_estimator.simple_state.copy()

    def generate_control_input_noise(self) -> Tuple[float, float]:
        """
        Generate control input noise

        Returns:
            Tuple[float, float]: (steering_rate_noise, acceleration_noise)
        """
        if not self.control_noise_enabled:
            return 0.0, 0.0

        # Use steering noise std as control input noise standard deviation
        steering_rate_noise = self.control_rng.normal(0, self.config.steering_noise_std)
        acceleration_noise = self.control_rng.normal(0, self.config.steering_noise_std)

        return steering_rate_noise, acceleration_noise

    def generate_measurement_noise(self, measurement: float) -> float:
        """
        Generate measurement noise for sensor readings
        
        Note: Measurement noise has been removed from the configuration.
        This method now returns the original measurement without noise.

        Args:
            measurement (float): Original measurement value

        Returns:
            float: Original measurement (no noise added)
        """
        return measurement

    def reset_state(self, initial_state: "VehicleState"):
        """
        Reset all state estimators to initial state

        Args:
            initial_state (VehicleState): Initial state to reset to
        """
        self.true_state = initial_state.copy()
        self.odometry_estimator.reset_state(initial_state)
        self.global_estimator.reset_state(initial_state)
        self.simple_estimator.reset_state(initial_state)
        self.simulation_time = 0.0

    def set_noise_enabled(self, enabled: bool):
        """
        Enable or disable noise generation for all estimators
        
        Note: Individual estimators now manage their own noise settings.
        This method controls the estimator-level noise switches.

        Args:
            enabled (bool): Whether to enable noise
        """
        self.odometry_estimator.set_noise_enabled(enabled)
        self.global_estimator.set_noise_enabled(enabled)
        self.simple_estimator.set_noise_enabled(enabled)

    def set_control_input_noise_enabled(self, enabled: bool):
        """
        Enable or disable control input noise specifically

        Args:
            enabled (bool): Whether to enable control input noise
        """
        self.control_noise_enabled = enabled

    def reset_seed(self, seed: Optional[int] = None):
        """
        Reset random seeds for all estimators

        Args:
            seed (Optional[int]): New seed value, None for random seed
        """
        if seed is not None:
            self.control_rng = np.random.RandomState(seed)
        else:
            self.control_rng = np.random.RandomState()
            
        self.odometry_estimator.reset_seed(seed)
        self.global_estimator.reset_seed(seed)
        self.simple_estimator.reset_seed(seed) 