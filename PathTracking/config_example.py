#!/usr/bin/env python3
"""
Configuration Management Example

This example demonstrates how to use the PathTracking configuration management system
including loading configurations, using presets, and customizing parameters.

Author: Assistant
"""

import os
import sys
import numpy as np

# Add the parent directory to the path so we can import the modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PathTracking.config import (
    ConfigManager, 
    get_config_manager, 
    load_preset_config,
    get_vehicle_config,
    get_pure_pursuit_config,
    get_velocity_controller_config,
    reset_config_manager
)
from PathTracking.vehicle_model import VehicleModel
from PathTracking.pure_pursuit import PurePursuitController
from PathTracking.velocity_planning import VelocityController
from PathTracking.trajectory import Trajectory


def demonstrate_basic_config_usage():
    """Demonstrate basic configuration loading and usage"""
    print("=== Basic Configuration Usage ===")
    
    # Reset config manager to start fresh
    reset_config_manager()
    
    # Get the global configuration manager
    config_manager = get_config_manager()
    
    # Print current configuration
    print("Default configuration:")
    print(f"Vehicle wheelbase: {config_manager.vehicle.wheelbase}m")
    print(f"Max steering angle: {config_manager.vehicle.max_steering_angle}°")
    print(f"Pure pursuit min lookahead: {config_manager.pure_pursuit.min_lookahead}m")
    print(f"Velocity controller max forward velocity: {config_manager.velocity_controller.max_forward_velocity}m/s")
    print()
    
    # Create components using configuration
    vehicle = VehicleModel()  # Will use global config automatically
    velocity_controller = VelocityController()  # Will use global config automatically
    
    print(f"Vehicle model wheelbase: {vehicle.kinematics.wheelbase}m")
    print(f"Velocity controller max forward velocity: {velocity_controller.max_forward_velocity}m/s")
    print()


def demonstrate_config_from_file():
    """Demonstrate loading configuration from YAML file"""
    print("=== Configuration from File ===")
    
    # Reset to start fresh
    reset_config_manager()
    
    # Create a custom config file
    custom_config = """
# Custom PathTracking Configuration
vehicle:
  wheelbase: 3.2
  max_steering_angle: 35.0
  max_velocity: 40.0

pure_pursuit:
  min_lookahead: 2.0
  k_gain: 8.0

velocity_controller:
  max_forward_velocity: 8.0
  max_backward_velocity: 3.0
"""
    
    config_file_path = "custom_config.yaml"
    with open(config_file_path, 'w') as f:
        f.write(custom_config)
    
    try:
        # Load configuration from file
        config_manager = ConfigManager(config_file_path)
        
        print("Custom configuration loaded:")
        print(f"Vehicle wheelbase: {config_manager.vehicle.wheelbase}m")
        print(f"Max steering angle: {config_manager.vehicle.max_steering_angle}°")
        print(f"Pure pursuit min lookahead: {config_manager.pure_pursuit.min_lookahead}m")
        print(f"Velocity controller max forward velocity: {config_manager.velocity_controller.max_forward_velocity}m/s")
        print()
        
        # Create components with this configuration
        vehicle = VehicleModel()
        print(f"Vehicle model using custom wheelbase: {vehicle.kinematics.wheelbase}m")
        print()
        
    finally:
        # Clean up
        if os.path.exists(config_file_path):
            os.remove(config_file_path)


def demonstrate_preset_configs():
    """Demonstrate using preset configurations"""
    print("=== Preset Configurations ===")
    
    presets = ['forward', 'reverse', 'parking', 'high_speed']
    
    for preset_name in presets:
        print(f"\n--- {preset_name.upper()} Preset ---")
        
        # Reset and load preset
        reset_config_manager()
        load_preset_config(preset_name)
        
        # Show configuration values
        config = get_config_manager()
        print(f"Velocity Controller:")
        print(f"  Max Forward Velocity: {config.velocity_controller.max_forward_velocity}m/s")
        print(f"  Max Backward Velocity: {config.velocity_controller.max_backward_velocity}m/s")
        print(f"  Max Acceleration: {config.velocity_controller.max_acceleration}m/s²")
        print(f"  Goal Tolerance: {config.velocity_controller.goal_tolerance}m")
        
        print(f"Pure Pursuit:")
        print(f"  Min Lookahead: {config.pure_pursuit.min_lookahead}m")
        print(f"  K Gain: {config.pure_pursuit.k_gain}")
        print(f"  Max Steering Angle: {config.pure_pursuit.max_steering_angle}°")


def demonstrate_mixed_config_usage():
    """Demonstrate mixing configuration with custom parameters"""
    print("\n=== Mixed Configuration Usage ===")
    
    # Reset and use forward driving preset
    reset_config_manager()
    load_preset_config('forward')
    
    # Create vehicle with default config parameters
    vehicle_default = VehicleModel()
    
    # Create vehicle with some custom parameters (others from config)
    vehicle_custom = VehicleModel(
        wheelbase=3.5,  # Custom wheelbase
        max_velocity=60.0,  # Custom max velocity
        # Other parameters will use config defaults
    )
    
    print("Default vehicle (from config):")
    print(f"  Wheelbase: {vehicle_default.kinematics.wheelbase}m")
    print(f"  Max velocity: {vehicle_default.kinematics.max_velocity}m/s")
    print(f"  Steering delay: {vehicle_default.steering_delay_buffer.delay}s")
    
    print("Custom vehicle (mixed parameters):")
    print(f"  Wheelbase: {vehicle_custom.kinematics.wheelbase}m (custom)")
    print(f"  Max velocity: {vehicle_custom.kinematics.max_velocity}m/s (custom)")
    print(f"  Steering delay: {vehicle_custom.steering_delay_buffer.delay}s (from config)")
    print()


def demonstrate_environment_variables():
    """Demonstrate environment variable configuration"""
    print("=== Environment Variable Configuration ===")
    
    # Set some environment variables
    os.environ['PT_VEHICLE_WHEELBASE'] = '3.8'
    os.environ['PT_PUREPURSUIT_MIN_LOOKAHEAD'] = '3.0'
    os.environ['PT_VELOCITY_MAX_FORWARD_VELOCITY'] = '12.0'
    
    try:
        # Reset and create new config manager (will pick up env vars)
        reset_config_manager()
        config_manager = get_config_manager()
        
        print("Configuration with environment variables:")
        print(f"Vehicle wheelbase: {config_manager.vehicle.wheelbase}m (from PT_VEHICLE_WHEELBASE)")
        print(f"Pure pursuit min lookahead: {config_manager.pure_pursuit.min_lookahead}m (from PT_PUREPURSUIT_MIN_LOOKAHEAD)")
        print(f"Velocity controller max forward velocity: {config_manager.velocity_controller.max_forward_velocity}m/s (from PT_VELOCITY_MAX_FORWARD_VELOCITY)")
        print()
        
    finally:
        # Clean up environment variables
        for var in ['PT_VEHICLE_WHEELBASE', 'PT_PUREPURSUIT_MIN_LOOKAHEAD', 'PT_VELOCITY_MAX_FORWARD_VELOCITY']:
            if var in os.environ:
                del os.environ[var]


def demonstrate_save_and_load_config():
    """Demonstrate saving and loading configuration"""
    print("=== Save and Load Configuration ===")
    
    # Reset and set up custom configuration
    reset_config_manager()
    config_manager = get_config_manager()
    
    # Modify some values
    config_manager.vehicle.wheelbase = 3.1
    config_manager.vehicle.max_steering_angle = 40.0
    config_manager.pure_pursuit.min_lookahead = 1.8
    config_manager.velocity_controller.max_forward_velocity = 7.5
    
    # Save configuration
    saved_config_path = "saved_config.yaml"
    config_manager.save_to_file(saved_config_path)
    print(f"Configuration saved to {saved_config_path}")
    
    try:
        # Reset and load the saved configuration
        reset_config_manager()
        new_config_manager = ConfigManager(saved_config_path)
        
        print("Loaded configuration:")
        print(f"Vehicle wheelbase: {new_config_manager.vehicle.wheelbase}m")
        print(f"Max steering angle: {new_config_manager.vehicle.max_steering_angle}°")
        print(f"Pure pursuit min lookahead: {new_config_manager.pure_pursuit.min_lookahead}m")
        print(f"Velocity controller max forward velocity: {new_config_manager.velocity_controller.max_forward_velocity}m/s")
        print()
        
    finally:
        # Clean up
        if os.path.exists(saved_config_path):
            os.remove(saved_config_path)


def demonstrate_trajectory_configuration():
    """Demonstrate trajectory configuration usage"""
    print("=== Trajectory Configuration ===")
    
    # Reset and modify trajectory config
    reset_config_manager()
    config_manager = get_config_manager()
    
    print("Default trajectory configuration:")
    print(f"Discretization distance: {config_manager.trajectory.discretization_distance}m")
    print(f"Default sample count: {config_manager.trajectory.default_sample_count}")
    
    # Create trajectory with default config
    trajectory_default = Trajectory()
    
    # Create trajectory with custom discretization distance
    trajectory_custom = Trajectory(discretization_distance=0.05)
    
    print(f"Default trajectory discretization: {trajectory_default._discretization_distance}m")
    print(f"Custom trajectory discretization: {trajectory_custom._discretization_distance}m")
    print()


def main():
    """Run all configuration examples"""
    print("PathTracking Configuration Management Examples")
    print("=" * 50)
    print()
    
    try:
        demonstrate_basic_config_usage()
        demonstrate_config_from_file()
        demonstrate_preset_configs()
        demonstrate_mixed_config_usage()
        demonstrate_environment_variables()
        demonstrate_save_and_load_config()
        demonstrate_trajectory_configuration()
        
        print("=" * 50)
        print("All configuration examples completed successfully!")
        
    except Exception as e:
        print(f"Error in configuration examples: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Reset config manager
        reset_config_manager()


if __name__ == "__main__":
    main() 