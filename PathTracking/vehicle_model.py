"""
Vehicle Kinematic Model

This module implements a vehicle kinematic model with the following components:
- VehicleState: Structured representation of vehicle state
- DelayBuffer: Handles actuator delays with command buffering
- OdometryEstimator: Maintains odometry-based position estimation with accumulated errors
- GlobalLocalizationEstimator: Maintains global localization estimation with GPS-like characteristics
- VehicleStateManager: Manages all three state types (true, odometry, global)
- BicycleKinematicModel: Core bicycle model kinematics (pure, no delays)
- VehicleModel: Main vehicle model combining kinematics with delays and control methods

Control methods:
- Rate-based control: Direct steering rate and acceleration inputs
- Direct control: Target steering angle and velocity with automatic conversion

Author: Assistant
"""

import math
from collections import deque
from dataclasses import dataclass
from typing import Tuple, Union, Optional, TYPE_CHECKING

import numpy as np

# Import configuration management
if TYPE_CHECKING:
    from .config import VehicleConfig, SimulationConfig


@dataclass
class VehicleState:
    """
    Structured representation of vehicle state

    Provides a clear interface for accessing and manipulating vehicle state components.
    All angles are in radians, distances in meters, velocities in m/s.
    """

    position_x: float = 0.0  # Vehicle position X coordinate [m]
    position_y: float = 0.0  # Vehicle position Y coordinate [m]
    yaw_angle: float = 0.0  # Vehicle heading angle [rad]
    velocity: float = 0.0  # Vehicle velocity [m/s]
    steering_angle: float = 0.0  # Steering wheel angle [rad]

    def to_array(self) -> np.ndarray:
        """
        Convert state to numpy array format

        Returns:
            np.ndarray: State as [position_x, position_y, yaw_angle, velocity, steering_angle]
        """
        return np.array(
            [
                self.position_x,
                self.position_y,
                self.yaw_angle,
                self.velocity,
                self.steering_angle,
            ]
        )

    @classmethod
    def from_array(cls, state_array: Union[np.ndarray, list]) -> "VehicleState":
        """
        Create VehicleState from array format

        Args:
            state_array: Array with [position_x, position_y, yaw_angle, velocity, steering_angle]

        Returns:
            VehicleState: New VehicleState instance
        """
        if len(state_array) != 5:
            raise ValueError("State array must have exactly 5 elements")

        return cls(
            position_x=float(state_array[0]),
            position_y=float(state_array[1]),
            yaw_angle=float(state_array[2]),
            velocity=float(state_array[3]),
            steering_angle=float(state_array[4]),
        )

    def get_position(self) -> Tuple[float, float]:
        """
        Get position as tuple

        Returns:
            Tuple[float, float]: (position_x, position_y)
        """
        return (self.position_x, self.position_y)

    def get_pose(self) -> Tuple[float, float, float]:
        """
        Get pose (position + orientation) as tuple

        Returns:
            Tuple[float, float, float]: (position_x, position_y, yaw_angle)
        """
        return (self.position_x, self.position_y, self.yaw_angle)

    def copy(self) -> "VehicleState":
        """
        Create a copy of the current state

        Returns:
            VehicleState: Copy of current state
        """
        return VehicleState(
            position_x=self.position_x,
            position_y=self.position_y,
            yaw_angle=self.yaw_angle,
            velocity=self.velocity,
            steering_angle=self.steering_angle,
        )

    def __str__(self) -> str:
        """String representation of vehicle state"""
        return (
            f"VehicleState(pos=({self.position_x:.2f}, {self.position_y:.2f}), "
            f"yaw={math.degrees(self.yaw_angle):.1f}°, "
            f"vel={self.velocity:.2f}m/s, "
            f"steer={math.degrees(self.steering_angle):.1f}°)"
        )


class DelayBuffer:
    """
    Command delay buffer for actuator delays

    Buffers commands with timestamps and retrieves them after specified delays.
    """

    def __init__(self, delay=0.0):
        """
        Initialize delay buffer

        Args:
            delay (float): Time delay [s]
        """
        self.delay = max(0.0, delay)
        self.command_buffer = deque()  # Store tuples of (command, timestamp)
        self.last_command = None  # Changed: Initialize with None instead of 0.0

    def add_command(self, command, current_time):
        """
        Add command to buffer with delay

        Args:
            command (float): Command value
            current_time (float): Current simulation time [s]
        """
        self.command_buffer.append((command, current_time + self.delay))

    def get_effective_command(self, command, current_time):
        """
        Get effective command considering delay

        Args:
            command (float): Current input command
            current_time (float): Current simulation time [s]

        Returns:
            float or None: Effective command after delay processing, or None if no command is available yet
        """
        if self.delay <= 0:
            return command

        # Add current command to buffer
        self.add_command(command, current_time)

        # Retrieve commands that have completed their delay
        while self.command_buffer and self.command_buffer[0][1] <= current_time:
            self.last_command = self.command_buffer.popleft()[0]

        return self.last_command  # Can be None during initial delay period

    def set_delay(self, delay):
        """
        Update delay and clear buffer

        Args:
            delay (float): New delay [s]
        """
        self.delay = max(0.0, delay)
        self.command_buffer.clear()
        self.last_command = None  # Changed: Reset to None instead of 0.0

    def clear(self):
        """Clear buffer and reset last command"""
        self.command_buffer.clear()
        self.last_command = None  # Changed: Reset to None instead of 0.0


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

    def __init__(self, config: "VehicleConfig"):
        """
        Initialize odometry estimator

        Args:
            config (VehicleConfig): Vehicle configuration containing noise parameters
        """
        self.config = config
        self.noise_enabled = config.noise_enabled

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

    def __init__(self, config: "VehicleConfig"):
        """
        Initialize global localization estimator

        Args:
            config (VehicleConfig): Vehicle configuration containing noise parameters
        """
        self.config = config
        self.noise_enabled = config.noise_enabled
        
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
        velocity_noise = self.rng.normal(0, self.config.velocity_noise_std) if self.noise_enabled else 0.0
        self.global_state.velocity = true_state.velocity + velocity_noise
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


class VehicleStateManager:
    """
    Manages all three types of vehicle states: true, odometry, and global localization
    
    This class coordinates the different state estimators and provides a unified interface
    for accessing different state types.
    """

    def __init__(self, config: "VehicleConfig"):
        """
        Initialize vehicle state manager

        Args:
            config (VehicleConfig): Vehicle configuration containing noise parameters
        """
        self.config = config
        
        # Initialize state estimators
        self.odometry_estimator = OdometryEstimator(config)
        self.global_estimator = GlobalLocalizationEstimator(config)
        
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
        
        # Update odometry and global estimates
        self.odometry_estimator.update_odometry_state(true_state, time_step)
        self.global_estimator.update_global_state(true_state, self.simulation_time)

    def get_state(self, state_type: Optional[str] = None) -> "VehicleState":
        """
        Get vehicle state of specified type

        Args:
            state_type (Optional[str]): Type of state to return ("true", "odometry", "global")
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
        else:
            raise ValueError(f"Unknown state type: {state_type}. Must be 'true', 'odometry', or 'global'")

    def get_true_state(self) -> "VehicleState":
        """Get true vehicle state (without noise)"""
        return self.true_state.copy()

    def get_odometry_state(self) -> "VehicleState":
        """Get odometry-based state estimate"""
        return self.odometry_estimator.odometry_state.copy()

    def get_global_state(self) -> "VehicleState":
        """Get global localization state estimate"""
        return self.global_estimator.global_state.copy()

    def generate_control_input_noise(self) -> Tuple[float, float]:
        """
        Generate control input noise

        Returns:
            Tuple[float, float]: (steering_rate_noise, acceleration_noise)
        """
        if not self.config.noise_enabled or not self.control_noise_enabled:
            return 0.0, 0.0

        steering_rate_noise = self.control_rng.normal(0, self.config.process_noise_std)
        acceleration_noise = self.control_rng.normal(0, self.config.process_noise_std)

        return steering_rate_noise, acceleration_noise

    def generate_measurement_noise(self, measurement: float) -> float:
        """
        Generate measurement noise for sensor readings

        Args:
            measurement (float): Original measurement value

        Returns:
            float: Noisy measurement
        """
        if not self.config.noise_enabled:
            return measurement

        noise = self.control_rng.normal(0, self.config.measurement_noise_std)
        return measurement + noise

    def reset_state(self, initial_state: "VehicleState"):
        """
        Reset all state estimators to initial state

        Args:
            initial_state (VehicleState): Initial state to reset to
        """
        self.true_state = initial_state.copy()
        self.odometry_estimator.reset_state(initial_state)
        self.global_estimator.reset_state(initial_state)
        self.simulation_time = 0.0

    def set_noise_enabled(self, enabled: bool):
        """
        Enable or disable noise generation for all estimators

        Args:
            enabled (bool): Whether to enable noise
        """
        self.config.noise_enabled = enabled
        self.odometry_estimator.set_noise_enabled(enabled)
        self.global_estimator.set_noise_enabled(enabled)

    def set_control_input_noise_enabled(self, enabled: bool):
        """
        Enable or disable control input noise specifically

        Args:
            enabled (bool): Whether to enable control input noise
        """
        self.control_noise_enabled = enabled
        self.config.control_input_noise_enabled = enabled

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


class BicycleKinematicModel:
    """
    Pure bicycle kinematic model without delays

    Implements basic bicycle model kinematics for vehicle motion.
    State vector: [position_x, position_y, yaw_angle, velocity, steering_angle]
    """

    def __init__(self, config: "VehicleConfig", initial_state: Optional[VehicleState] = None):
        """
        Initialize bicycle model parameters

        Args:
            config (VehicleConfig): Configuration object for the vehicle.
            initial_state (VehicleState, optional): Initial state of the vehicle.
        """
        self.config = config
        self.wheelbase = self.config.wheelbase
        self.max_steering_angle = self.config.get_max_steering_angle_rad()
        self.max_velocity = self.config.max_velocity
        self.min_velocity = self.config.min_velocity
        self.max_steering_rate = self.config.get_max_steering_rate_rad()
        self.max_acceleration = self.config.max_acceleration
        self.max_deceleration = -self.config.max_deceleration  # Convert to negative value

        # Initialize vehicle state manager for handling all state types
        self.state_manager = VehicleStateManager(config)

        if initial_state:
            self.state_manager.reset_state(initial_state)
        else:
            self.state_manager.reset_state(VehicleState())

    def update(self, steering_rate: float, acceleration: float, time_step: float):
        """
        Update vehicle state using bicycle model kinematics

        Args:
            steering_rate (float): Steering angle velocity [rad/s]
            acceleration (float): Acceleration [m/s²]
            time_step (float): Time step [s]

        Returns:
            VehicleState: Updated vehicle state (type depends on config.default_state_type)
        """
        # Get current true state for physics calculations
        current_true_state = self.state_manager.get_true_state()
        
        # Step 1: Add process noise to control inputs
        steering_rate_noise, acceleration_noise = self.state_manager.generate_control_input_noise()
        steering_rate += steering_rate_noise
        acceleration += acceleration_noise

        # Step 2: Apply control limits to inputs
        steering_rate = np.clip(steering_rate, -self.max_steering_rate, self.max_steering_rate)
        acceleration = np.clip(acceleration, self.max_deceleration, self.max_acceleration)

        # Step 3: Update steering angle with physical limits (use true state for physics)
        new_steering_angle = np.clip(
            current_true_state.steering_angle + steering_rate * time_step,
            -self.max_steering_angle,
            self.max_steering_angle,
        )

        # Step 4: Update velocity with acceleration and limits (use true state for physics)
        new_velocity = np.clip(
            current_true_state.velocity + acceleration * time_step, 
            self.min_velocity, 
            self.max_velocity
        )

        # Step 5: Update position using bicycle model kinematics (use true state for physics)
        new_position_x = current_true_state.position_x + new_velocity * math.cos(current_true_state.yaw_angle) * time_step
        new_position_y = current_true_state.position_y + new_velocity * math.sin(current_true_state.yaw_angle) * time_step

        # Step 6: Update yaw angle using bicycle model (use true state for physics)
        new_yaw_angle = self.normalize_angle(
            current_true_state.yaw_angle + (new_velocity / self.wheelbase) * math.tan(new_steering_angle) * time_step
        )

        # Step 7: Create new true state (without measurement noise)
        new_true_state = VehicleState(
            position_x=new_position_x,
            position_y=new_position_y,
            yaw_angle=new_yaw_angle,
            velocity=new_velocity,
            steering_angle=new_steering_angle,
        )

        # Step 8: Update all state estimates
        self.state_manager.update_all_states(new_true_state, time_step)

        # Return the requested state type
        return self.state_manager.get_state()

    def set_state(self, state):
        """
        Set vehicle state

        Args:
            state (Union[VehicleState, array, list]): Vehicle state as VehicleState object or
                     array [position_x, position_y, yaw_angle, velocity, steering_angle]
        """
        # Convert to VehicleState if needed
        if isinstance(state, VehicleState):
            vehicle_state = state
        else:
            vehicle_state = VehicleState.from_array(state)
            
        # Reset state manager with new state
        self.state_manager.reset_state(vehicle_state)

    def get_state(self, state_type: Optional[str] = None):
        """
        Get current vehicle state

        Args:
            state_type (Optional[str]): Type of state to return ("true", "odometry", "global")
                                       If None, uses default from config

        Returns:
            VehicleState: Current vehicle state of requested type
        """
        return self.state_manager.get_state(state_type)

    def get_state_array(self, state_type: Optional[str] = None):
        """
        Get current vehicle state as numpy array (for backward compatibility)

        Args:
            state_type (Optional[str]): Type of state to return ("true", "odometry", "global")
                                       If None, uses default from config

        Returns:
            np.array: Current state [position_x, position_y, yaw_angle, velocity, steering_angle]
        """
        return self.state_manager.get_state(state_type).to_array()

    def get_true_state(self) -> VehicleState:
        """
        Get true vehicle state (without noise)

        Returns:
            VehicleState: True vehicle state
        """
        return self.state_manager.get_true_state()

    def get_odometry_state(self) -> VehicleState:
        """
        Get odometry-based state estimate

        Returns:
            VehicleState: Odometry state estimate
        """
        return self.state_manager.get_odometry_state()

    def get_global_state(self) -> VehicleState:
        """
        Get global localization state estimate

        Returns:
            VehicleState: Global localization state estimate
        """
        return self.state_manager.get_global_state()

    def set_noise_enabled(self, enabled: bool):
        """
        Enable or disable noise generation

        Args:
            enabled (bool): Whether to enable noise
        """
        self.state_manager.set_noise_enabled(enabled)

    def get_noise_enabled(self) -> bool:
        """
        Get current noise enabled status

        Returns:
            bool: Whether noise is enabled
        """
        return self.config.noise_enabled

    def reset_noise_seed(self, seed: Optional[int] = None):
        """
        Reset noise random seed for reproducibility

        Args:
            seed (Optional[int]): New seed value, None for random seed
        """
        self.state_manager.reset_seed(seed)

    def get_clean_state(self) -> VehicleState:
        """
        Get current vehicle state without noise (for debugging/analysis)
        
        Note: This is an alias for get_true_state() for backward compatibility

        Returns:
            VehicleState: True vehicle state without noise
        """
        return self.state_manager.get_true_state()

    def get_noisy_measurement(self, measurement: float) -> float:
        """
        Apply measurement noise to a sensor reading

        Args:
            measurement (float): Original measurement value

        Returns:
            float: Noisy measurement
        """
        return self.state_manager.generate_measurement_noise(measurement)

    def set_control_input_noise_enabled(self, enabled: bool):
        """
        Enable or disable control input noise specifically

        Args:
            enabled (bool): Whether to enable control input noise
        """
        self.state_manager.set_control_input_noise_enabled(enabled)

    def get_control_input_noise_enabled(self) -> bool:
        """
        Get current control input noise enabled status

        Returns:
            bool: Whether control input noise is enabled
        """
        return self.config.control_input_noise_enabled

    @staticmethod
    def normalize_angle(angle):
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


class VehicleModel:
    """
    Main vehicle model with kinematics, delays, and control interface

    This class provides a comprehensive vehicle model that includes:
    - Bicycle kinematics for motion prediction
    - Actuator delays for steering and acceleration
    - Two control modes: rate-based and direct control
    """

    def __init__(
        self,
        config: "VehicleConfig",
        initial_state: Optional[VehicleState] = None,
    ):
        """
        Initialize the complete vehicle model.

        Args:
            config (VehicleConfig): Configuration object for the vehicle.
            initial_state (VehicleState, optional): Initial state of the vehicle.
        """
        self.config = config
        self.kinematic_model = BicycleKinematicModel(config=self.config, initial_state=initial_state)

        # Initialize delay buffers
        self.steering_delay_buffer = DelayBuffer(self.config.steering_delay)
        self.acceleration_delay_buffer = DelayBuffer(self.config.acceleration_delay)

        self.time = 0.0

    def update_with_rates(self, control_input: Tuple[float, float], time_step: float):
        """
        Update vehicle state using steering rate and acceleration with actuator delays

        Args:
            control_input (array): [steering_rate, acceleration] - steering angle velocity and acceleration
            time_step (float): time step [s]

        Returns:
            VehicleState: Updated vehicle state
        """
        steering_rate_cmd, acceleration_cmd = control_input

        # Update time
        self.time += time_step

        # Apply delays - use 0.0 as default during delay period
        effective_steering_rate = self.steering_delay_buffer.get_effective_command(steering_rate_cmd, self.time)
        if effective_steering_rate is None:
            effective_steering_rate = 0.0  # No steering command during delay period

        effective_acceleration = self.acceleration_delay_buffer.get_effective_command(acceleration_cmd, self.time)
        if effective_acceleration is None:
            effective_acceleration = 0.0  # No acceleration command during delay period

        # Update kinematics
        return self.kinematic_model.update(effective_steering_rate, effective_acceleration, time_step)

    def update_with_direct_control(self, control_input, time_step):
        """
        Update vehicle state using direct steering angle and velocity commands with delays

        Args:
            control_input (array): [steering_angle, velocity] - target steering angle and velocity
            time_step (float): time step [s]

        Returns:
            VehicleState: Updated vehicle state
        """
        # Get current state and update time
        current_state = self.kinematic_model.get_state()
        self.time += time_step

        # Add commands to delay buffers first (with clipping)
        target_steering_angle, target_velocity = control_input
        clipped_steering = np.clip(
            target_steering_angle,
            -self.kinematic_model.max_steering_angle,
            self.kinematic_model.max_steering_angle,
        )
        clipped_velocity = np.clip(
            target_velocity, self.kinematic_model.min_velocity, self.kinematic_model.max_velocity
        )

        # Add commands to buffers with current time
        self.steering_delay_buffer.add_command(clipped_steering, self.time)
        self.acceleration_delay_buffer.add_command(clipped_velocity, self.time)

        # Get delayed commands from buffers - handle None values during delay period
        effective_steering = self.steering_delay_buffer.get_effective_command(clipped_steering, self.time)
        if effective_steering is None:
            effective_steering = current_state.steering_angle  # Maintain current steering during delay

        effective_velocity = self.acceleration_delay_buffer.get_effective_command(clipped_velocity, self.time)
        if effective_velocity is None:
            effective_velocity = current_state.velocity  # Maintain current velocity during delay

        # Calculate rates based on delayed commands vs current state
        steering_error = effective_steering - current_state.steering_angle
        desired_steering_rate = steering_error * self.config.steering_rate_gain

        # Calculate acceleration based on velocity error
        velocity_error = effective_velocity - current_state.velocity
        desired_acceleration = velocity_error * self.config.acceleration_gain

        # Clip acceleration to vehicle limits
        desired_acceleration = np.clip(
            desired_acceleration,
            self.kinematic_model.max_deceleration,
            self.kinematic_model.max_acceleration,
        )

        # Update kinematics with calculated rates
        return self.kinematic_model.update(desired_steering_rate, desired_acceleration, time_step)

    def update(self, control_input, time_step):
        """
        Update vehicle state using rate-based control (default method)

        Args:
            control_input (array): [steering_rate, acceleration] - steering angle velocity and acceleration
            time_step (float): time step [s]

        Returns:
            VehicleState: Updated vehicle state
        """
        return self.update_with_rates(control_input, time_step)

    def set_state(self, state):
        """
        Set vehicle state and reset delay buffers

        Args:
            state (Union[VehicleState, array, list]): Vehicle state as VehicleState object or
                     array [position_x, position_y, yaw_angle, velocity, steering_angle]
        """
        self.kinematic_model.set_state(state)
        self.steering_delay_buffer.clear()
        self.acceleration_delay_buffer.clear()
        self.time = 0.0

    def get_state(self, state_type: Optional[str] = None):
        """
        Get current vehicle state

        Args:
            state_type (Optional[str]): Type of state to return ("true", "odometry", "global")
                                       If None, uses default from config

        Returns:
            VehicleState: Current vehicle state of requested type
        """
        return self.kinematic_model.get_state(state_type)

    def get_state_array(self, state_type: Optional[str] = None):
        """
        Get current vehicle state as numpy array (for backward compatibility)

        Args:
            state_type (Optional[str]): Type of state to return ("true", "odometry", "global")
                                       If None, uses default from config

        Returns:
            np.array: Current state [position_x, position_y, yaw_angle, velocity, steering_angle]
        """
        return self.kinematic_model.get_state_array(state_type)

    def get_true_state(self) -> VehicleState:
        """
        Get true vehicle state (without noise)

        Returns:
            VehicleState: True vehicle state
        """
        return self.kinematic_model.get_true_state()

    def get_odometry_state(self) -> VehicleState:
        """
        Get odometry-based state estimate

        Returns:
            VehicleState: Odometry state estimate
        """
        return self.kinematic_model.get_odometry_state()

    def get_global_state(self) -> VehicleState:
        """
        Get global localization state estimate

        Returns:
            VehicleState: Global localization state estimate
        """
        return self.kinematic_model.get_global_state()

    def get_position(self, state_type: Optional[str] = None):
        """
        Get vehicle position

        Args:
            state_type (Optional[str]): Type of state to return ("true", "odometry", "global")
                                       If None, uses default from config

        Returns:
            tuple: (position_x, position_y) position
        """
        state = self.kinematic_model.get_state(state_type)
        return state.get_position()

    def get_orientation(self, state_type: Optional[str] = None):
        """
        Get vehicle orientation

        Args:
            state_type (Optional[str]): Type of state to return ("true", "odometry", "global")
                                       If None, uses default from config

        Returns:
            float: yaw_angle [rad]
        """
        return self.kinematic_model.get_state(state_type).yaw_angle

    def get_velocity(self, state_type: Optional[str] = None):
        """
        Get vehicle velocity

        Args:
            state_type (Optional[str]): Type of state to return ("true", "odometry", "global")
                                       If None, uses default from config

        Returns:
            float: velocity [m/s]
        """
        return self.kinematic_model.get_state(state_type).velocity

    def get_steering_angle(self, state_type: Optional[str] = None):
        """
        Get steering angle

        Args:
            state_type (Optional[str]): Type of state to return ("true", "odometry", "global")
                                       If None, uses default from config

        Returns:
            float: steering_angle [rad]
        """
        return self.kinematic_model.get_state(state_type).steering_angle

    def set_delays(self, steering_delay=None, acceleration_delay=None):
        """
        Update delay parameters

        Args:
            steering_delay (float, optional): New steering delay [s]
            acceleration_delay (float, optional): New acceleration delay [s]
        """
        if steering_delay is not None:
            self.steering_delay_buffer.set_delay(steering_delay)
        if acceleration_delay is not None:
            self.acceleration_delay_buffer.set_delay(acceleration_delay)

    def get_delays(self):
        """
        Get current delay parameters

        Returns:
            tuple: (steering_delay, acceleration_delay) in seconds
        """
        return self.steering_delay_buffer.delay, self.acceleration_delay_buffer.delay

    def set_noise_enabled(self, enabled: bool):
        """
        Enable or disable noise generation

        Args:
            enabled (bool): Whether to enable noise
        """
        self.kinematic_model.set_noise_enabled(enabled)

    def get_noise_enabled(self) -> bool:
        """
        Get current noise enabled status

        Returns:
            bool: Whether noise is enabled
        """
        return self.kinematic_model.get_noise_enabled()

    def reset_noise_seed(self, seed: Optional[int] = None):
        """
        Reset noise random seed for reproducibility

        Args:
            seed (Optional[int]): New seed value, None for random seed
        """
        self.kinematic_model.reset_noise_seed(seed)

    def get_clean_state(self) -> VehicleState:
        """
        Get current vehicle state without noise (for debugging/analysis)
        
        Note: This is an alias for get_true_state() for backward compatibility

        Returns:
            VehicleState: True vehicle state without noise
        """
        return self.kinematic_model.get_true_state()

    def get_noisy_measurement(self, measurement: float) -> float:
        """
        Apply measurement noise to a sensor reading

        Args:
            measurement (float): Original measurement value

        Returns:
            float: Noisy measurement
        """
        return self.kinematic_model.get_noisy_measurement(measurement)

    def set_control_input_noise_enabled(self, enabled: bool):
        """
        Enable or disable control input noise specifically

        Args:
            enabled (bool): Whether to enable control input noise
        """
        self.kinematic_model.set_control_input_noise_enabled(enabled)

    def get_control_input_noise_enabled(self) -> bool:
        """
        Get current control input noise enabled status

        Returns:
            bool: Whether control input noise is enabled
        """
        return self.kinematic_model.get_control_input_noise_enabled()


# Legacy compatibility - keep original class name as alias
VehicleKinematicModel = VehicleModel


def simulate_vehicle_motion(
    initial_state: VehicleState, control_sequence: list, config: "VehicleConfig", sim_config: "SimulationConfig"
):
    """
    Simulate vehicle motion over a sequence of control inputs.

    Args:
        initial_state (VehicleState): Initial state of the vehicle.
        control_sequence (list): List of control inputs, where each element is
                                 a tuple (steering_rate, acceleration).
        config (VehicleConfig): Configuration for the vehicle model.
        sim_config (SimulationConfig): Configuration for the simulation.

    Returns:
        list: History of vehicle states throughout the simulation.
    """
    vehicle = VehicleModel(config=config, initial_state=initial_state)
    state_history = [vehicle.get_state().copy()]
    time_step = sim_config.time_step

    for control in control_sequence:
        vehicle.update_with_rates(control, time_step)
        state_history.append(vehicle.get_state().copy())

    return state_history


# Note: For comprehensive examples and testing, see vehicle_model_example.py
# which includes forward driving, reverse driving, and combined maneuvers.
