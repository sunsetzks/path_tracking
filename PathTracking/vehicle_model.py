"""
Vehicle Kinematic Model

This module implements a vehicle kinematic model with the following components:
- VehicleState: Structured representation of vehicle state
- DelayBuffer: Handles actuator delays with command buffering
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
    from .config import VehicleConfig

# Try multiple import strategies for robustness
get_vehicle_config = None
try:
    from .config import get_vehicle_config
except ImportError:
    try:
        # Fallback for direct execution or when run as a script
        import sys
        import os
        current_dir = os.path.dirname(__file__)
        if current_dir not in sys.path:
            sys.path.insert(0, current_dir)
        import config  # type: ignore
        get_vehicle_config = getattr(config, 'get_vehicle_config', None)
    except ImportError:
        # Final fallback for standalone usage
        get_vehicle_config = None


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


class BicycleKinematicModel:
    """
    Pure bicycle kinematic model without delays

    Implements basic bicycle model kinematics for vehicle motion.
    State vector: [position_x, position_y, yaw_angle, velocity, steering_angle]
    """

    def __init__(
        self, 
        wheelbase: Optional[float] = None, 
        max_steering_angle: Optional[float] = None, 
        max_velocity: Optional[float] = None, 
        min_velocity: Optional[float] = None,
        config: Optional['VehicleConfig'] = None
    ):
        """
        Initialize bicycle model parameters

        Args:
            wheelbase (float, optional): Distance between front and rear axles [m]
            max_steering_angle (float, optional): Maximum steering angle [rad]
            max_velocity (float, optional): Maximum forward velocity [m/s]
            min_velocity (float, optional): Maximum reverse velocity [m/s] (negative value)
            config (VehicleConfig, optional): Configuration object. If None, uses global config
        """
        # Get configuration
        if config is None and get_vehicle_config is not None:
            config = get_vehicle_config()
        
        # Set parameters with fallback to defaults
        if config is not None:
            self.wheelbase = wheelbase if wheelbase is not None else config.wheelbase
            self.max_steering_angle = max_steering_angle if max_steering_angle is not None else config.get_max_steering_angle_rad()
            self.max_velocity = max_velocity if max_velocity is not None else config.max_velocity
            self.min_velocity = min_velocity if min_velocity is not None else config.min_velocity
            self.max_steering_rate = config.get_max_steering_rate_rad()
            self.max_acceleration = config.max_acceleration
            self.max_deceleration = -config.max_deceleration  # Convert to negative value
        else:
            # Fallback defaults for standalone usage
            self.wheelbase = wheelbase if wheelbase is not None else 2.9
            self.max_steering_angle = max_steering_angle if max_steering_angle is not None else np.deg2rad(45.0)
            self.max_velocity = max_velocity if max_velocity is not None else 50.0
            self.min_velocity = min_velocity if min_velocity is not None else -10.0
            self.max_steering_rate = np.deg2rad(30)  # rad/s
            self.max_acceleration = 3.0  # m/s²
            self.max_deceleration = -5.0  # m/s²

        # Vehicle state using structured representation
        self.state = VehicleState()

    def update(self, steering_rate, acceleration, time_step):
        """
        Update vehicle state using bicycle model kinematics

        Args:
            steering_rate (float): Steering angle velocity [rad/s]
            acceleration (float): Acceleration [m/s²]
            time_step (float): Time step [s]

        Returns:
            VehicleState: Updated vehicle state
        """
        # Apply control limits
        steering_rate = np.clip(
            steering_rate, -self.max_steering_rate, self.max_steering_rate
        )
        acceleration = np.clip(
            acceleration, self.max_deceleration, self.max_acceleration
        )

        # Update steering angle with limits
        new_steering_angle = np.clip(
            self.state.steering_angle + steering_rate * time_step,
            -self.max_steering_angle,
            self.max_steering_angle,
        )

        # Update velocity with limits (allow reverse driving)
        new_velocity = np.clip(
            self.state.velocity + acceleration * time_step, self.min_velocity, self.max_velocity
        )

        # Update position using bicycle model
        new_position_x = (
            self.state.position_x
            + new_velocity * math.cos(self.state.yaw_angle) * time_step
        )
        new_position_y = (
            self.state.position_y
            + new_velocity * math.sin(self.state.yaw_angle) * time_step
        )

        # Update yaw angle using bicycle model
        new_yaw_angle = self.normalize_angle(
            self.state.yaw_angle
            + (new_velocity / self.wheelbase) * math.tan(new_steering_angle) * time_step
        )

        # Update state using VehicleState structure
        self.state = VehicleState(
            position_x=new_position_x,
            position_y=new_position_y,
            yaw_angle=new_yaw_angle,
            velocity=new_velocity,
            steering_angle=new_steering_angle,
        )

        return self.state.copy()

    def set_state(self, state):
        """
        Set vehicle state

        Args:
            state (Union[VehicleState, array, list]): Vehicle state as VehicleState object or
                     array [position_x, position_y, yaw_angle, velocity, steering_angle]
        """
        if isinstance(state, VehicleState):
            self.state = state.copy()
        else:
            self.state = VehicleState.from_array(state)

    def get_state(self):
        """
        Get current vehicle state

        Returns:
            VehicleState: Current vehicle state
        """
        return self.state.copy()

    def get_state_array(self):
        """
        Get current vehicle state as numpy array (for backward compatibility)

        Returns:
            np.array: Current state [position_x, position_y, yaw_angle, velocity, steering_angle]
        """
        return self.state.to_array()

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
    Complete vehicle model with delays and control methods

    Combines bicycle kinematics with actuator delays and provides two control interfaces:
    1. Rate-based control: Direct steering rate and acceleration inputs
    2. Direct control: Target steering angle and velocity with automatic conversion
    """

    def __init__(
        self,
        wheelbase: Optional[float] = None,
        max_steering_angle: Optional[float] = None,
        max_velocity: Optional[float] = None,
        min_velocity: Optional[float] = None,
        steering_delay: Optional[float] = None,
        acceleration_delay: Optional[float] = None,
        steering_rate_gain: Optional[float] = None,
        acceleration_gain: Optional[float] = None,
        config: Optional['VehicleConfig'] = None,
    ):
        """
        Initialize vehicle model

        Args:
            wheelbase (float, optional): Distance between front and rear axles [m]
            max_steering_angle (float, optional): Maximum steering angle [rad]
            max_velocity (float, optional): Maximum forward velocity [m/s]
            min_velocity (float, optional): Maximum reverse velocity [m/s] (negative value)
            steering_delay (float, optional): Time delay for steering commands [s]
            acceleration_delay (float, optional): Time delay for acceleration commands [s]
            steering_rate_gain (float, optional): Gain for converting steering error to steering rate [rad/s per rad]
            acceleration_gain (float, optional): Gain for converting velocity error to acceleration [(m/s²) per (m/s)]
            config (VehicleConfig, optional): Configuration object. If None, uses global config
        """
        # Get configuration
        if config is None and get_vehicle_config is not None:
            config = get_vehicle_config()
        
        # Set parameters with fallback to defaults
        if config is not None:
            _wheelbase = wheelbase if wheelbase is not None else config.wheelbase
            _max_steering_angle = max_steering_angle if max_steering_angle is not None else config.get_max_steering_angle_rad()
            _max_velocity = max_velocity if max_velocity is not None else config.max_velocity
            _min_velocity = min_velocity if min_velocity is not None else config.min_velocity
            _steering_delay = steering_delay if steering_delay is not None else config.steering_delay
            _acceleration_delay = acceleration_delay if acceleration_delay is not None else config.acceleration_delay
            _steering_rate_gain = steering_rate_gain if steering_rate_gain is not None else config.steering_rate_gain
            _acceleration_gain = acceleration_gain if acceleration_gain is not None else config.acceleration_gain
        else:
            # Fallback defaults for standalone usage
            _wheelbase = wheelbase if wheelbase is not None else 2.9
            _max_steering_angle = max_steering_angle if max_steering_angle is not None else np.deg2rad(45.0)
            _max_velocity = max_velocity if max_velocity is not None else 50.0
            _min_velocity = min_velocity if min_velocity is not None else -10.0
            _steering_delay = steering_delay if steering_delay is not None else 0.0
            _acceleration_delay = acceleration_delay if acceleration_delay is not None else 0.0
            _steering_rate_gain = steering_rate_gain if steering_rate_gain is not None else 5.0
            _acceleration_gain = acceleration_gain if acceleration_gain is not None else 2.0

        # Core kinematic model
        self.kinematics = BicycleKinematicModel(
            _wheelbase, _max_steering_angle, _max_velocity, _min_velocity, config
        )

        # Delay buffers
        self.steering_delay_buffer = DelayBuffer(_steering_delay)
        self.acceleration_delay_buffer = DelayBuffer(_acceleration_delay)

        # Control gains for direct control method
        self.steering_rate_gain = _steering_rate_gain
        self.acceleration_gain = _acceleration_gain

        # Time tracking
        self.current_time = 0.0

    def update_with_rates(self, control_input, time_step):
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
        self.current_time += time_step

        # Apply delays - use 0.0 as default during delay period
        effective_steering_rate = self.steering_delay_buffer.get_effective_command(
            steering_rate_cmd, self.current_time
        )
        if effective_steering_rate is None:
            effective_steering_rate = 0.0  # No steering command during delay period

        effective_acceleration = self.acceleration_delay_buffer.get_effective_command(
            acceleration_cmd, self.current_time
        )
        if effective_acceleration is None:
            effective_acceleration = 0.0  # No acceleration command during delay period

        # Update kinematics
        return self.kinematics.update(
            effective_steering_rate, effective_acceleration, time_step
        )

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
        current_state = self.kinematics.get_state()
        self.current_time += time_step

        # Add commands to delay buffers first (with clipping)
        target_steering_angle, target_velocity = control_input
        clipped_steering = np.clip(
            target_steering_angle,
            -self.kinematics.max_steering_angle,
            self.kinematics.max_steering_angle,
        )
        clipped_velocity = np.clip(target_velocity, self.kinematics.min_velocity, self.kinematics.max_velocity)

        # Add commands to buffers with current time
        self.steering_delay_buffer.add_command(clipped_steering, self.current_time)
        self.acceleration_delay_buffer.add_command(clipped_velocity, self.current_time)

        # Get delayed commands from buffers - handle None values during delay period
        effective_steering = self.steering_delay_buffer.get_effective_command(
            clipped_steering, self.current_time
        )
        if effective_steering is None:
            effective_steering = (
                current_state.steering_angle
            )  # Maintain current steering during delay

        effective_velocity = self.acceleration_delay_buffer.get_effective_command(
            clipped_velocity, self.current_time
        )
        if effective_velocity is None:
            effective_velocity = (
                current_state.velocity
            )  # Maintain current velocity during delay

        # Calculate rates based on delayed commands vs current state
        steering_error = effective_steering - current_state.steering_angle
        desired_steering_rate = self.steering_rate_gain * steering_error

        velocity_error = effective_velocity - current_state.velocity
        desired_acceleration = self.acceleration_gain * velocity_error

        # Update kinematics with calculated rates
        return self.kinematics.update(
            desired_steering_rate, desired_acceleration, time_step
        )

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
        self.kinematics.set_state(state)
        self.steering_delay_buffer.clear()
        self.acceleration_delay_buffer.clear()
        self.current_time = 0.0

    def get_state(self):
        """
        Get current vehicle state

        Returns:
            VehicleState: Current vehicle state
        """
        return self.kinematics.get_state()

    def get_state_array(self):
        """
        Get current vehicle state as numpy array (for backward compatibility)

        Returns:
            np.array: Current state [position_x, position_y, yaw_angle, velocity, steering_angle]
        """
        return self.kinematics.get_state_array()

    def get_position(self):
        """
        Get vehicle position

        Returns:
            tuple: (position_x, position_y) position
        """
        state = self.kinematics.get_state()
        return state.get_position()

    def get_orientation(self):
        """
        Get vehicle orientation

        Returns:
            float: yaw_angle [rad]
        """
        return self.kinematics.get_state().yaw_angle

    def get_velocity(self):
        """
        Get vehicle velocity

        Returns:
            float: velocity [m/s]
        """
        return self.kinematics.get_state().velocity

    def get_steering_angle(self):
        """
        Get steering angle

        Returns:
            float: steering_angle [rad]
        """
        return self.kinematics.get_state().steering_angle

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


# Legacy compatibility - keep original class name as alias
VehicleKinematicModel = VehicleModel


def simulate_vehicle_motion(
    initial_state,
    control_sequence,
    time_step: Optional[float] = None,
    wheelbase: Optional[float] = None,
    min_velocity: Optional[float] = None,
    max_velocity: Optional[float] = None,
    steering_delay: Optional[float] = None,
    acceleration_delay: Optional[float] = None,
    config: Optional['VehicleConfig'] = None,
):
    """
    Simulate vehicle motion with given control sequence and actuator delays

    Args:
        initial_state (Union[VehicleState, array, list]): Initial state as VehicleState object or
                     array [position_x, position_y, yaw_angle, velocity, steering_angle]
        control_sequence (array): Control inputs [[steering_rate1, acceleration1], [steering_rate2, acceleration2], ...]
        time_step (float, optional): Time step [s]. If None, uses config default
        wheelbase (float, optional): Vehicle wheelbase [m]. If None, uses config default
        min_velocity (float, optional): Maximum reverse velocity [m/s] (negative value). If None, uses config default
        max_velocity (float, optional): Maximum forward velocity [m/s]. If None, uses config default
        steering_delay (float, optional): Time delay for steering commands [s]. If None, uses config default
        acceleration_delay (float, optional): Time delay for acceleration commands [s]. If None, uses config default
        config (VehicleConfig, optional): Configuration object. If None, uses global config

    Returns:
        np.array: State trajectory as array for backward compatibility
    """
    # Get configuration for time_step if not provided
    if time_step is None:
        try:
            from .config import get_simulation_config
            sim_config = get_simulation_config()
            time_step = sim_config.time_step
        except ImportError:
            time_step = 0.1  # Fallback default

    vehicle = VehicleModel(
        wheelbase=wheelbase,
        min_velocity=min_velocity,
        max_velocity=max_velocity,
        steering_delay=steering_delay,
        acceleration_delay=acceleration_delay,
        config=config,
    )
    vehicle.set_state(initial_state)

    trajectory = [
        vehicle.get_state_array()
    ]  # Convert to array for backward compatibility

    for control_input in control_sequence:
        state = vehicle.update(control_input, time_step)
        trajectory.append(
            state.to_array()
        )  # Convert to array for backward compatibility

    return np.array(trajectory)


# Note: For comprehensive examples and testing, see vehicle_model_example.py
# which includes forward driving, reverse driving, and combined maneuvers.
