"""
Vehicle Kinematic Model

This module implements a vehicle kinematic model with the following components:
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
from typing import Tuple, Union, Optional, TYPE_CHECKING

import numpy as np

# Import configuration management and vehicle state
if TYPE_CHECKING:
    from .config import VehicleConfig, SimulationConfig, EstimatorConfig

from .vehicle_state import VehicleState
from .estimators import VehicleStateManager


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

    def __init__(self, config: "VehicleConfig", estimator_config: "EstimatorConfig", initial_state: Optional[VehicleState] = None):
        """
        Initialize bicycle model parameters

        Args:
            config (VehicleConfig): Configuration object for the vehicle.
            estimator_config (EstimatorConfig): Configuration object for the estimators.
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
        self.state_manager = VehicleStateManager(estimator_config)

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
            VehicleState: Updated vehicle state (type depends on estimator_config.default_state_type)
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
            state_type (Optional[str]): Type of state to return ("true", "odometry", "global", "simple")
                                       If None, uses default from config

        Returns:
            VehicleState: Current vehicle state of requested type
        """
        return self.state_manager.get_state(state_type)

    def get_state_array(self, state_type: Optional[str] = None):
        """
        Get current vehicle state as numpy array (for backward compatibility)

        Args:
            state_type (Optional[str]): Type of state to return ("true", "odometry", "global", "simple")
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

    def get_simple_state(self) -> VehicleState:
        """
        Get simple noise state estimate

        Returns:
            VehicleState: Simple noise state estimate
        """
        return self.state_manager.get_simple_state()

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
        return self.state_manager.odometry_estimator.noise_enabled

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
        return self.state_manager.control_noise_enabled

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
        estimator_config: "EstimatorConfig",
        initial_state: Optional[VehicleState] = None,
    ):
        """
        Initialize the complete vehicle model.

        Args:
            config (VehicleConfig): Configuration object for the vehicle.
            estimator_config (EstimatorConfig): Configuration object for the estimators.
            initial_state (VehicleState, optional): Initial state of the vehicle.
        """
        self.config = config
        self.kinematic_model = BicycleKinematicModel(config=self.config, estimator_config=estimator_config, initial_state=initial_state)

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
            state_type (Optional[str]): Type of state to return ("true", "odometry", "global", "simple")
                                       If None, uses default from config

        Returns:
            VehicleState: Current vehicle state of requested type
        """
        return self.kinematic_model.get_state(state_type)

    def get_state_array(self, state_type: Optional[str] = None):
        """
        Get current vehicle state as numpy array (for backward compatibility)

        Args:
            state_type (Optional[str]): Type of state to return ("true", "odometry", "global", "simple")
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

    def get_simple_state(self) -> VehicleState:
        """
        Get simple noise state estimate

        Returns:
            VehicleState: Simple noise state estimate
        """
        return self.kinematic_model.get_simple_state()

    def get_position(self, state_type: Optional[str] = None):
        """
        Get vehicle position

        Args:
            state_type (Optional[str]): Type of state to return ("true", "odometry", "global", "simple")
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
            state_type (Optional[str]): Type of state to return ("true", "odometry", "global", "simple")
                                       If None, uses default from config

        Returns:
            float: yaw_angle [rad]
        """
        return self.kinematic_model.get_state(state_type).yaw_angle

    def get_velocity(self, state_type: Optional[str] = None):
        """
        Get vehicle velocity

        Args:
            state_type (Optional[str]): Type of state to return ("true", "odometry", "global", "simple")
                                       If None, uses default from config

        Returns:
            float: velocity [m/s]
        """
        return self.kinematic_model.get_state(state_type).velocity

    def get_steering_angle(self, state_type: Optional[str] = None):
        """
        Get steering angle

        Args:
            state_type (Optional[str]): Type of state to return ("true", "odometry", "global", "simple")
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
    # Create a default estimator config for backward compatibility
    from .config import EstimatorConfig
    estimator_config = EstimatorConfig()
    
    vehicle = VehicleModel(config=config, estimator_config=estimator_config, initial_state=initial_state)
    state_history = [vehicle.get_state().copy()]
    time_step = sim_config.time_step

    for control in control_sequence:
        vehicle.update_with_rates(control, time_step)
        state_history.append(vehicle.get_state().copy())

    return state_history


# Note: For comprehensive examples and testing, see vehicle_model_example.py
# which includes forward driving, reverse driving, and combined maneuvers.
