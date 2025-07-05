#!/usr/bin/env python3
"""
Vehicle Model Noise Example

This example demonstrates the noise functionality in the vehicle model.
It shows how to:
1. Enable/disable noise
2. Configure noise parameters
3. Compare clean vs noisy vehicle behavior
4. Use different noise seeds for reproducibility

Author: Assistant
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import sys

# Add the parent directory to the path to import PathTracking modules
sys.path.append(str(Path(__file__).parent))

from PathTracking.config import load_config
from PathTracking.vehicle_model import VehicleModel, VehicleState


def demonstrate_noise_functionality():
    """Demonstrate vehicle model noise functionality"""

    print("=== Vehicle Model Noise Demonstration ===\n")

    # Load configuration and configure noise
    config = load_config()
    config.vehicle.control_input_noise_enabled = True
    config.vehicle.steering_noise_std = 0.02  # Control input noise
    config.vehicle.odometry_position_noise_std = 0.02  # Odometry position noise
    config.vehicle.odometry_yaw_noise_std = 0.01  # Odometry yaw noise
    config.vehicle.odometry_velocity_noise_std = 0.05  # Odometry velocity noise
    config.vehicle.global_position_noise_std = 0.5  # Global position noise
    config.vehicle.global_yaw_noise_std = 0.02  # Global yaw noise
    config.vehicle.noise_seed = 42  # For reproducible results

    print("Noise Configuration:")
    print(f"  Control input noise enabled: {config.vehicle.control_input_noise_enabled}")
    print(f"  Steering noise std: {config.vehicle.steering_noise_std} rad")
    print(f"  Odometry position noise std: {config.vehicle.odometry_position_noise_std} m")
    print(f"  Odometry yaw noise std: {config.vehicle.odometry_yaw_noise_std} rad")
    print(f"  Odometry velocity noise std: {config.vehicle.odometry_velocity_noise_std} m/s")
    print(f"  Global position noise std: {config.vehicle.global_position_noise_std} m")
    print(f"  Global yaw noise std: {config.vehicle.global_yaw_noise_std} rad")
    print(f"  Noise seed: {config.vehicle.noise_seed}")
    print()

    # Create initial state
    initial_state = VehicleState(position_x=0.0, position_y=0.0, yaw_angle=0.0, velocity=0.0, steering_angle=0.0)

    # Create vehicle model with noise enabled
    vehicle = VehicleModel(config.vehicle, initial_state)
    vehicle.reset_noise_seed(42)  # For reproducible results

    # Simulation parameters
    time_step = 0.1
    total_time = 10.0
    steps = int(total_time / time_step)

    # Control sequence: accelerate, turn, and maintain speed
    control_sequence = []
    for i in range(steps):
        t = i * time_step
        if t < 2.0:
            # Accelerate forward
            steering_rate = 0.0
            acceleration = 1.0
        elif t < 5.0:
            # Turn left while maintaining speed
            steering_rate = 0.3
            acceleration = 0.0
        elif t < 7.0:
            # Straighten out
            steering_rate = -0.2
            acceleration = 0.0
        else:
            # Maintain course
            steering_rate = 0.0
            acceleration = 0.0

        control_sequence.append((steering_rate, acceleration))

    # Simulate vehicle with different state types
    print("Running simulation...")
    true_trajectory = []
    odometry_trajectory = []
    global_trajectory = []

    for control in control_sequence:
        # Update vehicle with control input
        vehicle.update_with_rates(control, time_step)
        
        # Get different state types
        true_state = vehicle.get_true_state()
        odometry_state = vehicle.get_odometry_state()
        global_state = vehicle.get_global_state()

        true_trajectory.append(true_state.copy())
        odometry_trajectory.append(odometry_state.copy())
        global_trajectory.append(global_state.copy())

    # Extract trajectories for plotting
    true_x = [state.position_x for state in true_trajectory]
    true_y = [state.position_y for state in true_trajectory]
    odometry_x = [state.position_x for state in odometry_trajectory]
    odometry_y = [state.position_y for state in odometry_trajectory]
    global_x = [state.position_x for state in global_trajectory]
    global_y = [state.position_y for state in global_trajectory]

    # Plot results
    plt.figure(figsize=(12, 8))

    # Trajectory comparison
    plt.subplot(2, 2, 1)
    plt.plot(true_x, true_y, "b-", linewidth=2, label="True State")
    plt.plot(odometry_x, odometry_y, "r-", linewidth=1, alpha=0.7, label="Odometry")
    plt.plot(global_x, global_y, "g-", linewidth=1, alpha=0.7, label="Global Localization")
    plt.xlabel("X Position [m]")
    plt.ylabel("Y Position [m]")
    plt.title("Vehicle Trajectory Comparison")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis("equal")

    # Position error over time
    plt.subplot(2, 2, 2)
    time_array = np.array(range(len(true_trajectory))) * time_step
    odometry_error = np.sqrt(
        (np.array(odometry_x) - np.array(true_x)) ** 2 + (np.array(odometry_y) - np.array(true_y)) ** 2
    )
    global_error = np.sqrt(
        (np.array(global_x) - np.array(true_x)) ** 2 + (np.array(global_y) - np.array(true_y)) ** 2
    )
    plt.plot(time_array, odometry_error, "r-", linewidth=2, label="Odometry Error")
    plt.plot(time_array, global_error, "g-", linewidth=2, label="Global Error")
    plt.xlabel("Time [s]")
    plt.ylabel("Position Error [m]")
    plt.title("Position Error Due to Noise")
    plt.legend()
    plt.grid(True, alpha=0.3)

    # Velocity comparison
    plt.subplot(2, 2, 3)
    true_vel = [state.velocity for state in true_trajectory]
    odometry_vel = [state.velocity for state in odometry_trajectory]
    global_vel = [state.velocity for state in global_trajectory]
    plt.plot(time_array, true_vel, "b-", linewidth=2, label="True")
    plt.plot(time_array, odometry_vel, "r-", linewidth=1, alpha=0.7, label="Odometry")
    plt.plot(time_array, global_vel, "g-", linewidth=1, alpha=0.7, label="Global")
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity [m/s]")
    plt.title("Velocity Comparison")
    plt.legend()
    plt.grid(True, alpha=0.3)

    # Steering angle comparison
    plt.subplot(2, 2, 4)
    true_steer = [np.degrees(state.steering_angle) for state in true_trajectory]
    odometry_steer = [np.degrees(state.steering_angle) for state in odometry_trajectory]
    global_steer = [np.degrees(state.steering_angle) for state in global_trajectory]
    plt.plot(time_array, true_steer, "b-", linewidth=2, label="True")
    plt.plot(time_array, odometry_steer, "r-", linewidth=1, alpha=0.7, label="Odometry")
    plt.plot(time_array, global_steer, "g-", linewidth=1, alpha=0.7, label="Global")
    plt.xlabel("Time [s]")
    plt.ylabel("Steering Angle [deg]")
    plt.title("Steering Angle Comparison")
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

    # Print statistics
    print("\nSimulation Results:")
    print(f"Final position (true): ({true_x[-1]:.2f}, {true_y[-1]:.2f})")
    print(f"Final position (odometry): ({odometry_x[-1]:.2f}, {odometry_y[-1]:.2f})")
    print(f"Final position (global): ({global_x[-1]:.2f}, {global_y[-1]:.2f})")
    print(f"Final odometry error: {odometry_error[-1]:.3f} m")
    print(f"Final global error: {global_error[-1]:.3f} m")
    print(f"Mean odometry error: {np.mean(odometry_error):.3f} m")
    print(f"Mean global error: {np.mean(global_error):.3f} m")
    print(f"Max odometry error: {np.max(odometry_error):.3f} m")
    print(f"Max global error: {np.max(global_error):.3f} m")
    print(f"RMS odometry error: {np.sqrt(np.mean(odometry_error**2)):.3f} m")
    print(f"RMS global error: {np.sqrt(np.mean(global_error**2)):.3f} m")


def demonstrate_noise_control():
    """Demonstrate noise control functionality"""

    print("\n=== Noise Control Demonstration ===\n")

    # Load configuration
    config = load_config()
    config.vehicle.control_input_noise_enabled = True
    config.vehicle.odometry_position_noise_std = 0.1

    # Create vehicle model
    vehicle = VehicleModel(config.vehicle)

    print("Testing noise control functions:")
    print(f"Initial noise enabled: {vehicle.get_noise_enabled()}")

    # Test enabling/disabling noise
    vehicle.set_noise_enabled(False)
    print(f"After disabling: {vehicle.get_noise_enabled()}")

    vehicle.set_noise_enabled(True)
    print(f"After enabling: {vehicle.get_noise_enabled()}")

    # Test measurement noise
    original_measurement = 5.0
    noisy_measurement = vehicle.get_noisy_measurement(original_measurement)
    print(f"Original measurement: {original_measurement}")
    print(f"Noisy measurement: {noisy_measurement:.3f}")

    # Test seed reset for reproducibility
    print("\nTesting seed reproducibility:")
    vehicle.reset_noise_seed(123)
    measurement1 = vehicle.get_noisy_measurement(10.0)

    vehicle.reset_noise_seed(123)  # Reset to same seed
    measurement2 = vehicle.get_noisy_measurement(10.0)

    print(f"First measurement with seed 123: {measurement1:.6f}")
    print(f"Second measurement with seed 123: {measurement2:.6f}")
    print(f"Measurements are identical: {abs(measurement1 - measurement2) < 1e-10}")


if __name__ == "__main__":
    # Run demonstrations
    demonstrate_noise_functionality()
    demonstrate_noise_control()

    print("\n=== Noise Demonstration Complete ===")
    print("The vehicle model now supports realistic noise simulation!")
    print("You can configure noise parameters in config.yaml or programmatically.")
