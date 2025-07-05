#!/usr/bin/env python3
"""
Configuration Management Example

This example demonstrates how to use the new simplified PathTracking configuration system.
It shows how to load default configurations, load from a YAML file, and customize
parameters before passing them to different components.

Author: Assistant
"""

import os
import sys
import yaml
from dataclasses import asdict

# Add the parent directory to the path so we can import the modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PathTracking.config import (
    load_config,
    PathTrackingConfig,
    VehicleConfig,
    PurePursuitConfig,
    VelocityControllerConfig,
    TrajectoryConfig,
)
from PathTracking.vehicle_model import VehicleModel, VehicleState
from PathTracking.pure_pursuit import PurePursuitController
from PathTracking.velocity_planning import VelocityController
from PathTracking.trajectory import Trajectory, Waypoint


def demonstrate_basic_config_usage():
    """Demonstrate basic configuration loading and usage"""
    print("=== Basic Configuration Usage ===")

    # Load default configuration
    config = load_config()

    # Print some default configuration values
    print("Default configuration:")
    print(f"Vehicle wheelbase: {config.vehicle.wheelbase}m")
    print(f"Max steering angle: {config.vehicle.max_steering_angle}°")
    print(f"Pure pursuit min lookahead: {config.pure_pursuit.min_lookahead}m")
    print()

    # Create components using the configuration objects
    # Note: Each component now requires its specific config to be passed explicitly.
    vehicle = VehicleModel(config=config.vehicle, initial_state=VehicleState())
    velocity_controller = VelocityController(config=config.velocity_controller)

    print(f"Vehicle model wheelbase: {vehicle.kinematic_model.wheelbase}m")
    print(f"Velocity controller max forward velocity: {velocity_controller.max_forward_velocity}m/s")
    print()


def demonstrate_config_from_file():
    """Demonstrate loading configuration from a YAML file"""
    print("=== Configuration from File ===")

    # Create a custom config file
    custom_config_dict = {
        "vehicle": {"wheelbase": 3.2, "max_steering_angle": 35.0},
        "pure_pursuit": {"min_lookahead": 2.0, "k_gain": 8.0},
    }

    config_file_path = "custom_config.yaml"
    with open(config_file_path, "w") as f:
        yaml.dump(custom_config_dict, f)

    try:
        # Load configuration from the file
        config = load_config(config_file_path)

        print("Custom configuration loaded from file:")
        print(f"Vehicle wheelbase: {config.vehicle.wheelbase}m")
        print(f"Pure pursuit min lookahead: {config.pure_pursuit.min_lookahead}m")
        print()

        # Create a component with this custom configuration
        vehicle = VehicleModel(config=config.vehicle)
        print(f"Vehicle model using custom wheelbase: {vehicle.kinematic_model.wheelbase}m")
        print()

    finally:
        # Clean up the created file
        if os.path.exists(config_file_path):
            os.remove(config_file_path)


def demonstrate_customizing_config():
    """Demonstrate customizing a configuration object in code"""
    print("=== Customizing Configuration in Code ===")

    # Load the default configuration
    config = load_config()

    # Modify parameters directly on the config object
    config.vehicle.wheelbase = 3.5
    config.vehicle.max_velocity = 60.0
    config.pure_pursuit.k_gain = 0.7

    print("Customized configuration:")
    print(f"  Vehicle wheelbase: {config.vehicle.wheelbase}m (customized)")
    print(f"  Max velocity: {config.vehicle.max_velocity}m/s (customized)")
    print(f"  Pure pursuit k_gain: {config.pure_pursuit.k_gain} (customized)")
    print()

    # Create components with the customized config
    vehicle = VehicleModel(config=config.vehicle)

    # For PurePursuitController, it needs the wheelbase, which is in vehicle config.
    # We pass the specific part of the config it needs.
    pp_controller = PurePursuitController(wheelbase=vehicle.kinematic_model.wheelbase, config=config.pure_pursuit)

    print(f"Vehicle wheelbase from model: {vehicle.kinematic_model.wheelbase}m")
    print(f"Pure pursuit controller k_gain: {pp_controller.k_gain}")
    print()


def demonstrate_trajectory_configuration():
    """Demonstrate providing configuration to the Trajectory class"""
    print("=== Trajectory Configuration ===")

    # Load config and customize trajectory parameters
    config = load_config()
    config.trajectory.discretization_distance = 0.05

    print("Custom trajectory configuration:")
    print(f"Discretization distance: {config.trajectory.discretization_distance}m")

    # Create a trajectory with the custom config
    # The Trajectory class now requires its config object.
    trajectory = Trajectory(config=config.trajectory)

    # You can still add waypoints as before
    trajectory.add_waypoint(0, 0, 0)
    trajectory.add_waypoint(10, 5, 0.4)

    print(f"Created a trajectory with {len(trajectory.waypoints)} waypoints.")
    print(f"Internal discretization distance: {trajectory._discretization_distance}m")
    print()


def main():
    """Run all demonstration functions"""
    demonstrate_basic_config_usage()
    demonstrate_config_from_file()
    demonstrate_customizing_config()
    demonstrate_trajectory_configuration()


if __name__ == "__main__":
    main()
