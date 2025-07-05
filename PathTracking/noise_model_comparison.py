#!/usr/bin/env python3
"""
Noise Model Comparison Example

This example demonstrates the difference between odometry noise model and 
global localization noise model. It shows how:
1. Odometry noise accumulates over time (drift)
2. Global localization noise is bounded and corrected periodically
3. Different noise characteristics for different positioning systems

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


def compare_noise_models():
    """Compare odometry and global localization noise models"""

    print("=== Noise Model Comparison ===\n")

    # Create initial state
    initial_state = VehicleState(position_x=0.0, position_y=0.0, yaw_angle=0.0, velocity=0.0, steering_angle=0.0)

    # Configure odometry noise model
    config_odometry = load_config()
    config_odometry.vehicle.noise_enabled = True
    config_odometry.vehicle.noise_model = "odometry"
    config_odometry.vehicle.position_noise_std = 0.05  # Increase noise for visibility
    config_odometry.vehicle.yaw_noise_std = 0.02
    config_odometry.vehicle.velocity_noise_std = 0.1
    config_odometry.vehicle.steering_noise_std = 0.02
    config_odometry.vehicle.process_noise_std = 0.01
    config_odometry.vehicle.noise_seed = 42  # For reproducible results

    # Configure global localization noise model
    config_global = load_config()
    config_global.vehicle.noise_enabled = True
    config_global.vehicle.noise_model = "global_localization"
    config_global.vehicle.control_input_noise_enabled = False
    config_global.vehicle.position_noise_std = 0.05  # Same base noise as odometry
    config_global.vehicle.yaw_noise_std = 0.02
    config_global.vehicle.velocity_noise_std = 0.1
    config_global.vehicle.steering_noise_std = 0.02
    config_global.vehicle.process_noise_std = 0.01
    config_global.vehicle.global_position_noise_std = 0.8  # GPS-like accuracy
    config_global.vehicle.global_yaw_noise_std = 0.03
    config_global.vehicle.global_measurement_frequency = 1.0  # 1 Hz GPS updates
    config_global.vehicle.global_measurement_delay = 0.1  # 100ms delay
    config_global.vehicle.noise_seed = 42  # Same seed for fair comparison

    print("Odometry Noise Model Configuration:")
    print(f"  Position noise std: {config_odometry.vehicle.position_noise_std} m")
    print(f"  Yaw noise std: {config_odometry.vehicle.yaw_noise_std} rad")
    print(f"  Process noise std: {config_odometry.vehicle.process_noise_std}")
    print()

    print("Global Localization Noise Model Configuration:")
    print(f"  Position noise std: {config_global.vehicle.position_noise_std} m")
    print(f"  Yaw noise std: {config_global.vehicle.yaw_noise_std} rad")
    print(f"  Global position noise std: {config_global.vehicle.global_position_noise_std} m")
    print(f"  Global yaw noise std: {config_global.vehicle.global_yaw_noise_std} rad")
    print(f"  Global measurement frequency: {config_global.vehicle.global_measurement_frequency} Hz")
    print(f"  Global measurement delay: {config_global.vehicle.global_measurement_delay} s")
    print()

    # Create vehicle models
    vehicle_clean = VehicleModel(config_odometry.vehicle, initial_state)
    vehicle_clean.set_noise_enabled(False)

    vehicle_odometry = VehicleModel(config_odometry.vehicle, initial_state)
    vehicle_global = VehicleModel(config_global.vehicle, initial_state)

    # Simulation parameters
    time_step = 0.1
    total_time = 30.0  # Longer simulation to show drift accumulation
    steps = int(total_time / time_step)

    # Create a more complex control sequence
    control_sequence = []
    for i in range(steps):
        t = i * time_step
        if t < 3.0:
            # Accelerate forward
            steering_rate = 0.0
            acceleration = 1.0
        elif t < 8.0:
            # Turn left while maintaining speed
            steering_rate = 0.3
            acceleration = 0.0
        elif t < 12.0:
            # Straighten out
            steering_rate = -0.2
            acceleration = 0.0
        elif t < 18.0:
            # Turn right
            steering_rate = -0.4
            acceleration = 0.0
        elif t < 22.0:
            # Straighten out again
            steering_rate = 0.3
            acceleration = 0.0
        else:
            # Maintain course
            steering_rate = 0.0
            acceleration = 0.0

        control_sequence.append((steering_rate, acceleration))

    # Simulate all vehicles
    print("Running simulation...")
    clean_trajectory = []
    odometry_trajectory = []
    global_trajectory = []

    for control in control_sequence:
        # Update all vehicles with the same control input
        clean_state = vehicle_clean.update_with_rates(control, time_step)
        odometry_state = vehicle_odometry.update_with_rates(control, time_step)
        global_state = vehicle_global.update_with_rates(control, time_step)

        clean_trajectory.append(clean_state.copy())
        odometry_trajectory.append(odometry_state.copy())
        global_trajectory.append(global_state.copy())

    # Extract trajectories for plotting
    clean_x = [state.position_x for state in clean_trajectory]
    clean_y = [state.position_y for state in clean_trajectory]
    odometry_x = [state.position_x for state in odometry_trajectory]
    odometry_y = [state.position_y for state in odometry_trajectory]
    global_x = [state.position_x for state in global_trajectory]
    global_y = [state.position_y for state in global_trajectory]

    # Calculate errors
    time_array = np.array(range(len(clean_trajectory))) * time_step
    odometry_error = np.sqrt(
        (np.array(odometry_x) - np.array(clean_x)) ** 2 + 
        (np.array(odometry_y) - np.array(clean_y)) ** 2
    )
    global_error = np.sqrt(
        (np.array(global_x) - np.array(clean_x)) ** 2 + 
        (np.array(global_y) - np.array(clean_y)) ** 2
    )

    # Plot results
    plt.figure(figsize=(16, 12))

    # Trajectory comparison
    plt.subplot(2, 3, 1)
    plt.plot(clean_x, clean_y, "b-", linewidth=3, label="Clean (no noise)", alpha=0.8)
    plt.plot(odometry_x, odometry_y, "r-", linewidth=2, label="Odometry noise", alpha=0.7)
    plt.plot(global_x, global_y, "g-", linewidth=2, label="Global localization", alpha=0.7)
    plt.xlabel("X Position [m]")
    plt.ylabel("Y Position [m]")
    plt.title("Vehicle Trajectory Comparison")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis("equal")

    # Position error over time
    plt.subplot(2, 3, 2)
    plt.plot(time_array, odometry_error, "r-", linewidth=2, label="Odometry error")
    plt.plot(time_array, global_error, "g-", linewidth=2, label="Global localization error")
    plt.xlabel("Time [s]")
    plt.ylabel("Position Error [m]")
    plt.title("Position Error Over Time")
    plt.legend()
    plt.grid(True, alpha=0.3)

    # Error statistics comparison
    plt.subplot(2, 3, 3)
    error_stats = {
        'Mean Error': [np.mean(odometry_error), np.mean(global_error)],
        'Max Error': [np.max(odometry_error), np.max(global_error)],
        'Final Error': [odometry_error[-1], global_error[-1]],
        'RMS Error': [np.sqrt(np.mean(odometry_error**2)), np.sqrt(np.mean(global_error**2))]
    }
    
    x_pos = np.arange(len(error_stats))
    width = 0.35
    
    odometry_values = [error_stats[key][0] for key in error_stats.keys()]
    global_values = [error_stats[key][1] for key in error_stats.keys()]
    
    plt.bar(x_pos - width/2, odometry_values, width, label='Odometry', color='red', alpha=0.7)
    plt.bar(x_pos + width/2, global_values, width, label='Global Localization', color='green', alpha=0.7)
    
    plt.xlabel('Error Metric')
    plt.ylabel('Error [m]')
    plt.title('Error Statistics Comparison')
    plt.xticks(x_pos, list(error_stats.keys()), rotation=45)
    plt.legend()
    plt.grid(True, alpha=0.3)

    # X position over time
    plt.subplot(2, 3, 4)
    plt.plot(time_array, clean_x, "b-", linewidth=3, label="Clean", alpha=0.8)
    plt.plot(time_array, odometry_x, "r-", linewidth=2, label="Odometry", alpha=0.7)
    plt.plot(time_array, global_x, "g-", linewidth=2, label="Global localization", alpha=0.7)
    plt.xlabel("Time [s]")
    plt.ylabel("X Position [m]")
    plt.title("X Position Over Time")
    plt.legend()
    plt.grid(True, alpha=0.3)

    # Y position over time
    plt.subplot(2, 3, 5)
    plt.plot(time_array, clean_y, "b-", linewidth=3, label="Clean", alpha=0.8)
    plt.plot(time_array, odometry_y, "r-", linewidth=2, label="Odometry", alpha=0.7)
    plt.plot(time_array, global_y, "g-", linewidth=2, label="Global localization", alpha=0.7)
    plt.xlabel("Time [s]")
    plt.ylabel("Y Position [m]")
    plt.title("Y Position Over Time")
    plt.legend()
    plt.grid(True, alpha=0.3)

    # Yaw angle comparison
    plt.subplot(2, 3, 6)
    clean_yaw = [np.degrees(state.yaw_angle) for state in clean_trajectory]
    odometry_yaw = [np.degrees(state.yaw_angle) for state in odometry_trajectory]
    global_yaw = [np.degrees(state.yaw_angle) for state in global_trajectory]
    
    plt.plot(time_array, clean_yaw, "b-", linewidth=3, label="Clean", alpha=0.8)
    plt.plot(time_array, odometry_yaw, "r-", linewidth=2, label="Odometry", alpha=0.7)
    plt.plot(time_array, global_yaw, "g-", linewidth=2, label="Global localization", alpha=0.7)
    plt.xlabel("Time [s]")
    plt.ylabel("Yaw Angle [degrees]")
    plt.title("Yaw Angle Over Time")
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

    # Print detailed statistics
    print("\n=== Simulation Results ===")
    print(f"Simulation time: {total_time} s")
    print(f"Time steps: {steps}")
    print()
    
    print("Final Positions:")
    print(f"  Clean: ({clean_x[-1]:.2f}, {clean_y[-1]:.2f})")
    print(f"  Odometry: ({odometry_x[-1]:.2f}, {odometry_y[-1]:.2f})")
    print(f"  Global localization: ({global_x[-1]:.2f}, {global_y[-1]:.2f})")
    print()
    
    print("Odometry Noise Model Statistics:")
    print(f"  Final position error: {odometry_error[-1]:.3f} m")
    print(f"  Mean position error: {np.mean(odometry_error):.3f} m")
    print(f"  Max position error: {np.max(odometry_error):.3f} m")
    print(f"  RMS position error: {np.sqrt(np.mean(odometry_error**2)):.3f} m")
    print(f"  Error growth rate: {(odometry_error[-1] - odometry_error[0]) / total_time:.3f} m/s")
    print()
    
    print("Global Localization Noise Model Statistics:")
    print(f"  Final position error: {global_error[-1]:.3f} m")
    print(f"  Mean position error: {np.mean(global_error):.3f} m")
    print(f"  Max position error: {np.max(global_error):.3f} m")
    print(f"  RMS position error: {np.sqrt(np.mean(global_error**2)):.3f} m")
    print(f"  Error growth rate: {(global_error[-1] - global_error[0]) / total_time:.3f} m/s")
    print()
    
    print("Improvement with Global Localization:")
    improvement_final = ((odometry_error[-1] - global_error[-1]) / odometry_error[-1]) * 100
    improvement_mean = ((np.mean(odometry_error) - np.mean(global_error)) / np.mean(odometry_error)) * 100
    improvement_max = ((np.max(odometry_error) - np.max(global_error)) / np.max(odometry_error)) * 100
    
    print(f"  Final error reduction: {improvement_final:.1f}%")
    print(f"  Mean error reduction: {improvement_mean:.1f}%")
    print(f"  Max error reduction: {improvement_max:.1f}%")


def demonstrate_noise_model_switching():
    """Demonstrate switching between noise models"""
    
    print("\n=== Noise Model Switching Demo ===\n")
    
    # Create a vehicle with odometry noise initially
    config = load_config()
    config.vehicle.noise_enabled = True
    config.vehicle.noise_model = "odometry"
    config.vehicle.position_noise_std = 0.05
    config.vehicle.noise_seed = 42
    
    vehicle = VehicleModel(config.vehicle)
    
    print(f"Initial noise model: {config.vehicle.noise_model}")
    print(f"Noise enabled: {vehicle.get_noise_enabled()}")
    
    # Test with a simple control input
    control = (0.1, 0.5)  # steering_rate, acceleration
    time_step = 0.1
    
    # Run a few steps with odometry noise
    print("\nRunning with odometry noise model:")
    for i in range(3):
        state = vehicle.update_with_rates(control, time_step)
        print(f"  Step {i+1}: pos=({state.position_x:.3f}, {state.position_y:.3f})")
    
    # Switch to global localization noise model
    # Note: In practice, you would create a new vehicle model with different config
    print("\nSwitching to global localization noise model would require:")
    print("  1. Creating new configuration with noise_model='global_localization'")
    print("  2. Creating new VehicleModel instance with the new configuration")
    print("  3. Transferring current state to the new model")
    print("This ensures proper initialization of the noise generator.")


if __name__ == "__main__":
    # Run demonstrations
    compare_noise_models()
    demonstrate_noise_model_switching()

    print("\n=== Noise Model Comparison Complete ===")
    print("Key Differences:")
    print("1. Odometry noise accumulates over time (unbounded drift)")
    print("2. Global localization noise is bounded and corrected periodically")
    print("3. Global localization simulates GPS/GNSS characteristics:")
    print("   - Periodic measurements with delays")
    print("   - Bounded error that doesn't grow indefinitely")
    print("   - Persistent bias plus measurement noise")
    print("4. Choose odometry for pure dead-reckoning scenarios")
    print("5. Choose global localization for GPS-aided navigation") 