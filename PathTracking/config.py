"""
Global Configuration Management for PathTracking

This module provides a centralized configuration management system for the entire PathTracking project.
It supports loading configuration from YAML files, environment variables, and provides default values
for all parameters used across different components.

Author: Assistant
"""

import os
import yaml
from dataclasses import dataclass, field
from typing import Dict, Any, Optional
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
class PresetConfigs:
    """Predefined configuration presets for different scenarios"""
    
    @staticmethod
    def get_forward_driving_config() -> Dict[str, Any]:
        """Get configuration optimized for forward driving"""
        return {
            'vehicle': VehicleConfig(
                max_velocity=15.0,  # Higher max velocity for forward driving
                min_velocity=-5.0,
                max_acceleration=2.5,
                max_deceleration=3.0,
            ),
            'velocity_controller': VelocityControllerConfig(
                max_forward_velocity=6.0,
                max_backward_velocity=2.0,
                max_acceleration=2.0,
                max_deceleration=2.5,
                goal_tolerance=1.0,
                velocity_tolerance=0.2,
                conservative_braking_factor=1.2,
                min_velocity=0.5,
            ),
            'pure_pursuit': PurePursuitConfig(
                min_lookahead=2.5,
                k_gain=1.2,
                max_steering_angle=45.0,
            )
        }
    
    @staticmethod
    def get_reverse_driving_config() -> Dict[str, Any]:
        """Get configuration optimized for reverse driving"""
        return {
            'vehicle': VehicleConfig(
                max_velocity=8.0,  # Moderate max velocity for reverse driving
                min_velocity=-8.0,  # More reverse capability
                max_acceleration=1.5,
                max_deceleration=2.0,
                max_steering_angle=40.0,  # Slightly more steering for tight maneuvers
            ),
            'velocity_controller': VelocityControllerConfig(
                max_forward_velocity=3.0,
                max_backward_velocity=2.5,
                max_acceleration=1.0,
                max_deceleration=1.5,
                goal_tolerance=0.8,
                velocity_tolerance=0.1,
                conservative_braking_factor=1.5,
                min_velocity=0.3,
            ),
            'pure_pursuit': PurePursuitConfig(
                min_lookahead=1.5,
                k_gain=0.8,
                max_steering_angle=35.0,
            )
        }
    
    @staticmethod
    def get_parking_config() -> Dict[str, Any]:
        """Get configuration optimized for parking maneuvers"""
        return {
            'vehicle': VehicleConfig(
                max_velocity=5.0,  # Low max velocity for parking
                min_velocity=-5.0,  # Equal reverse capability for parking
                max_acceleration=1.0,
                max_deceleration=1.5,
                max_steering_angle=45.0,  # Full steering for tight maneuvers
                max_steering_rate=20.0,  # Slower steering for precision
            ),
            'velocity_controller': VelocityControllerConfig(
                max_forward_velocity=2.0,
                max_backward_velocity=2.0,
                max_acceleration=0.8,
                max_deceleration=1.0,
                goal_tolerance=0.3,
                velocity_tolerance=0.05,
                conservative_braking_factor=1.8,
                min_velocity=0.2,
            ),
            'pure_pursuit': PurePursuitConfig(
                min_lookahead=1.0,
                k_gain=0.5,
                max_steering_angle=40.0,
            )
        }
    
    @staticmethod
    def get_high_speed_config() -> Dict[str, Any]:
        """Get configuration optimized for high speed driving"""
        return {
            'vehicle': VehicleConfig(
                max_velocity=30.0,  # High max velocity for highway driving
                min_velocity=-10.0,
                max_acceleration=4.0,
                max_deceleration=6.0,
                max_steering_angle=35.0,  # Reduced steering for stability at speed
                max_steering_rate=40.0,  # Faster steering response
            ),
            'velocity_controller': VelocityControllerConfig(
                max_forward_velocity=15.0,
                max_backward_velocity=5.0,
                max_acceleration=3.0,
                max_deceleration=4.0,
                goal_tolerance=2.0,
                velocity_tolerance=0.5,
                conservative_braking_factor=1.0,
                min_velocity=1.0,
            ),
            'pure_pursuit': PurePursuitConfig(
                min_lookahead=5.0,
                k_gain=2.0,
                max_steering_angle=30.0,
            )
        }


@dataclass
class PathTrackingConfig:
    """Main configuration container for the entire PathTracking project"""
    vehicle: VehicleConfig = field(default_factory=VehicleConfig)
    pure_pursuit: PurePursuitConfig = field(default_factory=PurePursuitConfig)
    velocity_controller: VelocityControllerConfig = field(default_factory=VelocityControllerConfig)
    trajectory: TrajectoryConfig = field(default_factory=TrajectoryConfig)
    simulation: SimulationConfig = field(default_factory=SimulationConfig)


class ConfigManager:
    """
    Configuration manager for loading and managing PathTracking configurations
    
    This class handles loading configurations from files, environment variables,
    and provides access to configuration parameters throughout the application.
    """
    
    def __init__(self, config_file: Optional[str] = None):
        """
        Initialize configuration manager
        
        Args:
            config_file (Optional[str]): Path to YAML configuration file
        """
        self._config = PathTrackingConfig()
        self._config_file = config_file
        
        # Load configuration from file if provided
        if config_file and os.path.exists(config_file):
            self.load_from_file(config_file)
        
        # Override with environment variables if present
        self._load_from_environment()
    
    def load_from_file(self, config_file: str) -> None:
        """
        Load configuration from YAML file
        
        Args:
            config_file (str): Path to YAML configuration file
        """
        try:
            with open(config_file, 'r', encoding='utf-8') as f:
                config_data = yaml.safe_load(f)
            
            if config_data:
                self._update_config_from_dict(config_data)
                
        except FileNotFoundError:
            print(f"Warning: Configuration file {config_file} not found. Using default values.")
        except yaml.YAMLError as e:
            print(f"Error parsing YAML configuration file: {e}")
        except Exception as e:
            print(f"Error loading configuration: {e}")
    
    def save_to_file(self, config_file: str) -> None:
        """
        Save current configuration to YAML file
        
        Args:
            config_file (str): Path to save YAML configuration file
        """
        try:
            config_dict = self._config_to_dict()
            with open(config_file, 'w', encoding='utf-8') as f:
                yaml.dump(config_dict, f, default_flow_style=False, indent=2)
        except Exception as e:
            print(f"Error saving configuration: {e}")
    
    def load_preset(self, preset_name: str) -> None:
        """
        Load a predefined configuration preset
        
        Args:
            preset_name (str): Name of preset ('forward', 'reverse', 'parking', 'high_speed')
        """
        preset_configs = {
            'forward': PresetConfigs.get_forward_driving_config,
            'reverse': PresetConfigs.get_reverse_driving_config,
            'parking': PresetConfigs.get_parking_config,
            'high_speed': PresetConfigs.get_high_speed_config,
        }
        
        if preset_name not in preset_configs:
            raise ValueError(f"Unknown preset: {preset_name}. Available presets: {list(preset_configs.keys())}")
        
        preset_config = preset_configs[preset_name]()
        self._update_config_from_dict(preset_config)
    
    def _update_config_from_dict(self, config_dict: Dict[str, Any]) -> None:
        """Update configuration from dictionary"""
        for section_name, section_data in config_dict.items():
            if hasattr(self._config, section_name):
                if isinstance(section_data, dict):
                    # Handle dictionary format (from YAML files)
                    section_config = getattr(self._config, section_name)
                    for key, value in section_data.items():
                        if hasattr(section_config, key):
                            setattr(section_config, key, value)
                else:
                    # Handle config object format (from presets)
                    setattr(self._config, section_name, section_data)
    
    def _config_to_dict(self) -> Dict[str, Any]:
        """Convert configuration to dictionary"""
        result = {}
        for field_name in ['vehicle', 'pure_pursuit', 'velocity_controller', 'trajectory', 'simulation']:
            field_obj = getattr(self._config, field_name)
            result[field_name] = {
                key: value for key, value in field_obj.__dict__.items()
                if not key.startswith('_')
            }
        return result
    
    def _load_from_environment(self) -> None:
        """Load configuration overrides from environment variables"""
        # Example: PT_VEHICLE_WHEELBASE, PT_PUREPURSUIT_MIN_LOOKAHEAD, etc.
        env_prefix = "PT_"
        
        for env_var, env_value in os.environ.items():
            if env_var.startswith(env_prefix):
                try:
                    # Parse environment variable name
                    var_parts = env_var[len(env_prefix):].lower().split('_')
                    if len(var_parts) >= 2:
                        section_name = var_parts[0]
                        param_name = '_'.join(var_parts[1:])
                        
                        # Convert string value to appropriate type
                        if hasattr(self._config, section_name):
                            section_config = getattr(self._config, section_name)
                            if hasattr(section_config, param_name):
                                current_value = getattr(section_config, param_name)
                                
                                # Convert based on current type
                                if isinstance(current_value, bool):
                                    new_value = env_value.lower() in ('true', '1', 'yes', 'on')
                                elif isinstance(current_value, int):
                                    new_value = int(env_value)
                                elif isinstance(current_value, float):
                                    new_value = float(env_value)
                                else:
                                    new_value = env_value
                                
                                setattr(section_config, param_name, new_value)
                                
                except (ValueError, AttributeError) as e:
                    print(f"Warning: Could not parse environment variable {env_var}: {e}")
    
    @property
    def vehicle(self) -> VehicleConfig:
        """Get vehicle configuration"""
        return self._config.vehicle
    
    @property
    def pure_pursuit(self) -> PurePursuitConfig:
        """Get pure pursuit configuration"""
        return self._config.pure_pursuit
    
    @property
    def velocity_controller(self) -> VelocityControllerConfig:
        """Get velocity controller configuration"""
        return self._config.velocity_controller
    
    @property
    def trajectory(self) -> TrajectoryConfig:
        """Get trajectory configuration"""
        return self._config.trajectory
    
    @property
    def simulation(self) -> SimulationConfig:
        """Get simulation configuration"""
        return self._config.simulation
    
    def get_config(self) -> PathTrackingConfig:
        """Get complete configuration object"""
        return self._config
    
    def __str__(self) -> str:
        """String representation of configuration"""
        return f"PathTracking Configuration:\n{yaml.dump(self._config_to_dict(), default_flow_style=False)}"


# Global configuration manager instance
_global_config_manager: Optional[ConfigManager] = None


def get_config_manager(config_file: Optional[str] = None) -> ConfigManager:
    """
    Get global configuration manager instance
    
    Args:
        config_file (Optional[str]): Path to configuration file (only used on first call)
    
    Returns:
        ConfigManager: Global configuration manager instance
    """
    global _global_config_manager
    
    if _global_config_manager is None:
        # Look for default config file if not specified
        if config_file is None:
            default_config_path = os.path.join(os.path.dirname(__file__), 'config.yaml')
            if os.path.exists(default_config_path):
                config_file = default_config_path
        
        _global_config_manager = ConfigManager(config_file)
    
    return _global_config_manager


def reset_config_manager() -> None:
    """Reset global configuration manager (useful for testing)"""
    global _global_config_manager
    _global_config_manager = None


# Convenience functions for common configuration access
def get_vehicle_config() -> VehicleConfig:
    """Get vehicle configuration"""
    return get_config_manager().vehicle


def get_pure_pursuit_config() -> PurePursuitConfig:
    """Get pure pursuit configuration"""
    return get_config_manager().pure_pursuit


def get_velocity_controller_config() -> VelocityControllerConfig:
    """Get velocity controller configuration"""
    return get_config_manager().velocity_controller


def get_trajectory_config() -> TrajectoryConfig:
    """Get trajectory configuration"""
    return get_config_manager().trajectory


def get_simulation_config() -> SimulationConfig:
    """Get simulation configuration"""
    return get_config_manager().simulation


def load_preset_config(preset_name: str) -> None:
    """
    Load a predefined configuration preset
    
    Args:
        preset_name (str): Name of preset ('forward', 'reverse', 'parking', 'high_speed')
    """
    get_config_manager().load_preset(preset_name) 