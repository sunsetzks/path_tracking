"""
Configuration for PathTracking

This module defines dataclasses for configuration of different components.
It provides a simple function to load configuration from a YAML file.

Author: Assistant
"""

import os
import yaml
from dataclasses import dataclass, field, fields, is_dataclass
from typing import Dict, Any, Optional, Type
import numpy as np


@dataclass
class VehicleConfig:
    """Vehicle model configuration parameters"""

    # Physical parameters
    wheelbase: float = 2.9  # Distance between front and rear axles [m]
    vehicle_length: float = 4.5  # Total vehicle length [m]
    vehicle_width: float = 2.0  # Total vehicle width [m]
    wheel_length: float = 0.4  # Wheel length [m]
    wheel_width: float = 0.2  # Wheel width [m]
    wheel_track: float = 1.6  # Distance between left and right wheels [m]

    # Kinematic limits
    max_steering_angle: float = 45.0  # Maximum steering angle [degrees]
    max_velocity: float = 50.0  # Maximum forward velocity [m/s]
    min_velocity: float = -10.0  # Maximum reverse velocity [m/s] (negative value)
    max_steering_rate: float = 30.0  # Maximum steering rate [degrees/s]
    max_acceleration: float = 3.0  # Maximum acceleration [m/s²]
    max_deceleration: float = 5.0  # Maximum deceleration [m/s²] (positive value)

    # Delay parameters
    steering_delay: float = 0.0  # Time delay for steering commands [s]
    acceleration_delay: float = 0.0  # Time delay for acceleration commands [s]

    # Control gains for direct control
    steering_rate_gain: float = 5.0  # Gain for converting steering error to steering rate
    acceleration_gain: float = 2.0  # Gain for converting velocity error to acceleration

    # Noise parameters
    noise_enabled: bool = False  # Enable/disable noise simulation
    control_input_noise_enabled: bool = True  # Enable/disable control input noise (master switch)
    position_noise_std: float = 0.01  # Standard deviation for position noise [m]
    yaw_noise_std: float = 0.005  # Standard deviation for yaw angle noise [rad]
    velocity_noise_std: float = 0.02  # Standard deviation for velocity noise [m/s]
    steering_noise_std: float = 0.01  # Standard deviation for steering angle noise [rad]
    process_noise_std: float = 0.001  # Standard deviation for process noise [various units]
    measurement_noise_std: float = 0.005  # Standard deviation for measurement noise [various units]
    noise_seed: Optional[int] = None  # Random seed for reproducible noise (None for random)
    
    # Odometry noise parameters (for dead reckoning estimation)
    odometry_position_noise_std: float = 0.01  # Standard deviation for odometry position noise [m]
    odometry_yaw_noise_std: float = 0.005  # Standard deviation for odometry yaw angle noise [rad]
    odometry_velocity_noise_std: float = 0.02  # Standard deviation for odometry velocity noise [m/s]
    
    # Global localization noise parameters (for GPS-like positioning)
    global_position_noise_std: float = 0.5  # Standard deviation for global position noise [m]
    global_yaw_noise_std: float = 0.02  # Standard deviation for global yaw angle noise [rad]
    global_measurement_frequency: float = 1.0  # Frequency of global measurements [Hz]
    global_measurement_delay: float = 0.1  # Delay of global measurements [s]
    
    # Default state type to return when get_state() is called
    default_state_type: str = "true"  # Options: "true", "odometry", "global"

    def get_max_steering_angle_rad(self) -> float:
        """Get maximum steering angle in radians"""
        return np.deg2rad(self.max_steering_angle)

    def get_max_steering_rate_rad(self) -> float:
        """Get maximum steering rate in radians/s"""
        return np.deg2rad(self.max_steering_rate)


@dataclass
class PurePursuitConfig:
    """Pure Pursuit controller configuration parameters"""

    min_lookahead: float = 1.0  # Minimum lookahead distance [m]
    k_gain: float = 10.0  # Lookahead distance gain (lookahead = k_gain * velocity + min_lookahead)
    max_steering_angle: float = 45.0  # Maximum steering angle [degrees]
    goal_tolerance: float = 0.5  # Distance tolerance to consider goal reached [m]
    velocity_tolerance: float = 0.1  # Velocity tolerance to consider vehicle stopped [m/s]

    def get_max_steering_angle_rad(self) -> float:
        """Get maximum steering angle in radians"""
        return np.deg2rad(self.max_steering_angle)


@dataclass
class VelocityControllerConfig:
    """Velocity controller configuration parameters"""

    max_forward_velocity: float = 5.0  # Maximum forward velocity [m/s]
    max_backward_velocity: float = 2.0  # Maximum backward velocity [m/s]
    max_acceleration: float = 1.0  # Maximum acceleration magnitude [m/s²]
    max_deceleration: float = 2.0  # Maximum deceleration magnitude [m/s²]
    goal_tolerance: float = 0.5  # Distance tolerance to consider goal reached [m]
    velocity_tolerance: float = 0.1  # Velocity tolerance to consider vehicle stopped [m/s]
    conservative_braking_factor: float = 1.2  # Safety factor for deceleration distance
    min_velocity: float = 0.1  # Minimum velocity magnitude [m/s]


@dataclass
class TrajectoryConfig:
    """Trajectory configuration parameters"""

    discretization_distance: float = 0.1  # Default distance between discretized points [m]
    default_sample_count: int = 100  # Initial default sample count


@dataclass
class SimulationConfig:
    """Simulation configuration parameters"""

    time_step: float = 0.1  # Default simulation time step [s]
    max_time: float = 60.0  # Default maximum simulation time [s]

    # Visualization parameters
    plot_margin: float = 5.0  # Margin around trajectory for plot view [m]
    figure_size: tuple = (16, 8)  # Default figure size (width, height)

    # Animation parameters
    animation_interval: int = 50  # Matplotlib animation interval [ms]


@dataclass
class PathTrackingConfig:
    """Main configuration container for the entire PathTracking project"""

    vehicle: VehicleConfig = field(default_factory=VehicleConfig)
    pure_pursuit: PurePursuitConfig = field(default_factory=PurePursuitConfig)
    velocity_controller: VelocityControllerConfig = field(default_factory=VelocityControllerConfig)
    trajectory: TrajectoryConfig = field(default_factory=TrajectoryConfig)
    simulation: SimulationConfig = field(default_factory=SimulationConfig)


def _update_config_from_dict(config_obj: Any, config_dict: Dict[str, Any]) -> None:
    """
    Recursively update dataclass fields from a dictionary.
    """
    for f in fields(config_obj):
        if f.name in config_dict:
            value = config_dict[f.name]
            if is_dataclass(f.type) and isinstance(value, dict):
                _update_config_from_dict(getattr(config_obj, f.name), value)
            else:
                setattr(config_obj, f.name, value)


def load_config(config_file: Optional[str] = None) -> PathTrackingConfig:
    """
    Load PathTracking configuration.

    This function loads configuration from a YAML file and merges it with the default
    configuration. If no file is provided, it returns the default configuration.

    Args:
        config_file (Optional[str]): Path to the YAML configuration file.

    Returns:
        PathTrackingConfig: The fully populated configuration object.
    """
    config = PathTrackingConfig()

    if config_file and os.path.exists(config_file):
        with open(config_file, "r") as f:
            yaml_config = yaml.safe_load(f)

        if yaml_config:
            _update_config_from_dict(config, yaml_config)

    return config
