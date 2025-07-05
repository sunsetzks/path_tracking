#!/usr/bin/env python3
"""
Estimator Configuration Example

This example demonstrates how to use the new separated estimator configuration
and shows the difference between the vehicle model and estimator configurations.

Author: Assistant
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import sys

# Add the parent directory to the path to import PathTracking modules
sys.path.append(str(Path(__file__).parent.parent))

from PathTracking import (
    VehicleModel, 
    VehicleConfig, 
    EstimatorConfig, 
    VehicleState,
    load_config
)


def demonstrate_new_structure():
    """Demonstrate the new separated configuration structure"""
    
    print("=== New Estimator Configuration Structure ===\n")
    
    # Method 1: Create configurations separately
    print("Method 1: Separate configuration creation")
    vehicle_config = VehicleConfig()
    estimator_config = EstimatorConfig()
    
    # Configure vehicle parameters
    vehicle_config.wheelbase = 2.5
    vehicle_config.max_velocity = 20.0
    vehicle_config.steering_delay = 0.1
    
    # Configure estimator parameters
    estimator_config.noise_seed = 42
    estimator_config.odometry_position_noise_std = 0.05
    estimator_config.global_position_noise_std = 1.0
    estimator_config.control_input_noise_enabled = True
    estimator_config.default_state_type = "odometry"
    
    # Create vehicle model
    initial_state = VehicleState(position_x=0.0, position_y=0.0, yaw_angle=0.0)
    vehicle1 = VehicleModel(vehicle_config, estimator_config, initial_state)
    
    print(f"Vehicle wheelbase: {vehicle_config.wheelbase} m")
    print(f"Estimator noise seed: {estimator_config.noise_seed}")
    print(f"Default state type: {estimator_config.default_state_type}")
    print()
    
    # Method 2: Use the main configuration loader
    print("Method 2: Using main configuration loader")
    config = load_config()
    
    # Modify both vehicle and estimator settings
    config.vehicle.max_velocity = 15.0
    config.estimator.odometry_position_noise_std = 0.02
    config.estimator.default_state_type = "true"
    
    vehicle2 = VehicleModel(config.vehicle, config.estimator, initial_state)
    
    print(f"Vehicle max velocity: {config.vehicle.max_velocity} m/s")
    print(f"Estimator position noise: {config.estimator.odometry_position_noise_std} m")
    print(f"Default state type: {config.estimator.default_state_type}")
    print()
    
    return vehicle1, vehicle2


def compare_configurations():
    """Compare different estimator configurations"""
    
    print("=== Comparing Different Estimator Configurations ===\n")
    
    # Base vehicle configuration
    vehicle_config = VehicleConfig()
    initial_state = VehicleState()
    
    # Configuration 1: High noise
    high_noise_config = EstimatorConfig()
    high_noise_config.noise_seed = 42
    high_noise_config.odometry_position_noise_std = 0.1
    high_noise_config.odometry_yaw_noise_std = 0.05
    high_noise_config.control_input_noise_enabled = True
    high_noise_config.default_state_type = "odometry"
    
    # Configuration 2: Low noise
    low_noise_config = EstimatorConfig()
    low_noise_config.noise_seed = 42
    low_noise_config.odometry_position_noise_std = 0.01
    low_noise_config.odometry_yaw_noise_std = 0.005
    low_noise_config.control_input_noise_enabled = False
    low_noise_config.default_state_type = "odometry"
    
    # Configuration 3: No noise (true state)
    no_noise_config = EstimatorConfig()
    no_noise_config.default_state_type = "true"
    
    # Create vehicles with different configurations
    vehicle_high_noise = VehicleModel(vehicle_config, high_noise_config, initial_state)
    vehicle_low_noise = VehicleModel(vehicle_config, low_noise_config, initial_state)
    vehicle_no_noise = VehicleModel(vehicle_config, no_noise_config, initial_state)
    
    # Simulate motion
    control_input = (0.2, 1.0)  # steering_rate, acceleration
    time_step = 0.1
    steps = 50
    
    trajectories = {"High Noise": [], "Low Noise": [], "No Noise": []}
    
    for i in range(steps):
        # Update all vehicles with the same control input
        state_high = vehicle_high_noise.update_with_rates(control_input, time_step)
        state_low = vehicle_low_noise.update_with_rates(control_input, time_step)
        state_none = vehicle_no_noise.update_with_rates(control_input, time_step)
        
        trajectories["High Noise"].append((state_high.position_x, state_high.position_y))
        trajectories["Low Noise"].append((state_low.position_x, state_low.position_y))
        trajectories["No Noise"].append((state_none.position_x, state_none.position_y))
    
    # Plot comparison
    plt.figure(figsize=(12, 8))
    
    colors = {"High Noise": "red", "Low Noise": "orange", "No Noise": "blue"}
    
    for config_name, traj in trajectories.items():
        x_coords = [point[0] for point in traj]
        y_coords = [point[1] for point in traj]
        plt.plot(x_coords, y_coords, color=colors[config_name], 
                linewidth=2, label=config_name, alpha=0.8)
    
    plt.xlabel("X Position [m]")
    plt.ylabel("Y Position [m]")
    plt.title("Vehicle Trajectories with Different Estimator Configurations")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis("equal")
    plt.show()
    
    # Print final positions
    print("Final positions after 50 steps:")
    for config_name, traj in trajectories.items():
        final_x, final_y = traj[-1]
        print(f"  {config_name}: ({final_x:.3f}, {final_y:.3f})")
    print()


def demonstrate_state_access():
    """Demonstrate accessing different state types"""
    
    print("=== State Type Access with New Structure ===\n")
    
    # Create vehicle with noise enabled
    vehicle_config = VehicleConfig()
    estimator_config = EstimatorConfig()
    estimator_config.noise_seed = 42
    estimator_config.odometry_position_noise_std = 0.05
    estimator_config.global_position_noise_std = 0.5
    estimator_config.simple_position_noise_std = 0.1
    estimator_config.control_input_noise_enabled = True
    estimator_config.default_state_type = "true"
    
    initial_state = VehicleState()
    vehicle = VehicleModel(vehicle_config, estimator_config, initial_state)
    
    # Run a few simulation steps
    control_input = (0.1, 0.5)
    time_step = 0.1
    
    print("State comparison after each simulation step:")
    print("Step | True State      | Odometry State  | Global State    | Simple State")
    print("-----|-----------------|-----------------|-----------------|------------------")
    
    for i in range(5):
        vehicle.update_with_rates(control_input, time_step)
        
        true_state = vehicle.get_true_state()
        odom_state = vehicle.get_odometry_state()
        global_state = vehicle.get_global_state()
        simple_state = vehicle.get_simple_state()
        
        print(f"{i+1:4d} | ({true_state.position_x:5.2f}, {true_state.position_y:5.2f}) | "
              f"({odom_state.position_x:5.2f}, {odom_state.position_y:5.2f}) | "
              f"({global_state.position_x:5.2f}, {global_state.position_y:5.2f}) | "
              f"({simple_state.position_x:5.2f}, {simple_state.position_y:5.2f})")
    
    print(f"\nDefault state type: {estimator_config.default_state_type}")
    print(f"get_state() returns: ({vehicle.get_state().position_x:.3f}, {vehicle.get_state().position_y:.3f})")
    print()


if __name__ == "__main__":
    print("PathTracking Estimator Configuration Example\n")
    
    # Demonstrate the new structure
    vehicle1, vehicle2 = demonstrate_new_structure()
    
    # Compare different configurations
    compare_configurations()
    
    # Demonstrate state access
    demonstrate_state_access()
    
    print("=== Summary ===")
    print("Key changes in the new structure:")
    print("1. VehicleConfig: Contains physical vehicle parameters and control settings")
    print("2. EstimatorConfig: Contains all noise and state estimation parameters")
    print("3. VehicleModel: Now requires both configs: VehicleModel(vehicle_config, estimator_config)")
    print("4. State types: 'true', 'odometry', 'global', 'simple' - all configurable via EstimatorConfig")
    print("5. Cleaner separation: Vehicle physics vs. sensor/estimation characteristics")
    print("\nThis separation makes it easier to:")
    print("- Configure different noise models independently")
    print("- Reuse vehicle configurations with different sensor setups")
    print("- Test estimation algorithms with consistent vehicle dynamics")
    print("- Maintain and extend the codebase") 