#!/usr/bin/env python3
"""
Noise Model Comparison Example

This example demonstrates the difference between true state, odometry state, and 
global localization state. It shows how:
1. True state: Perfect state without any noise
2. Odometry state: Dead reckoning with accumulating drift over time
3. Global localization state: GPS-like positioning with bounded, periodic corrections

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


def compare_state_estimators():
    """Compare true state, odometry state, and global localization state"""

    print("=== State Estimator Comparison ===\n")

    # Create initial state
    initial_state = VehicleState(position_x=0.0, position_y=0.0, yaw_angle=0.0, velocity=0.0, steering_angle=0.0)

    # Configure vehicle with all noise types enabled
    config = load_config()
    config.vehicle.noise_seed = 42  # For reproducible results
    
    # Odometry noise parameters (dead reckoning)
    config.vehicle.odometry_position_noise_std = 0.01  # 1cm noise per meter traveled
    config.vehicle.odometry_yaw_noise_std = 0.005  # ~0.3 degree noise per radian turned
    config.vehicle.odometry_velocity_noise_std = 0.05  # 5cm/s velocity noise
    
    # Global localization noise parameters (GPS-like)
    config.vehicle.global_position_noise_std = 1.0  # 1m GPS accuracy
    config.vehicle.global_yaw_noise_std = 0.02  # ~1 degree GPS heading accuracy
    config.vehicle.global_measurement_frequency = 1.0  # 1 Hz GPS updates
    config.vehicle.global_measurement_delay = 0.2  # 200ms GPS delay
    
    # Control input noise
    config.vehicle.control_input_noise_enabled = True
    config.vehicle.steering_noise_std = 0.01  # Control input noise
    
    # Set default state type to "true" for clean comparison
    config.vehicle.default_state_type = "true"

    print("Configuration:")
    print(f"  Odometry position noise std: {config.vehicle.odometry_position_noise_std} m")
    print(f"  Odometry yaw noise std: {config.vehicle.odometry_yaw_noise_std} rad ({np.degrees(config.vehicle.odometry_yaw_noise_std):.2f}°)")
    print(f"  Odometry velocity noise std: {config.vehicle.odometry_velocity_noise_std} m/s")
    print(f"  Global position noise std: {config.vehicle.global_position_noise_std} m")
    print(f"  Global yaw noise std: {config.vehicle.global_yaw_noise_std} rad ({np.degrees(config.vehicle.global_yaw_noise_std):.2f}°)")
    print(f"  Global measurement frequency: {config.vehicle.global_measurement_frequency} Hz")
    print(f"  Global measurement delay: {config.vehicle.global_measurement_delay} s")
    print(f"  Control input noise enabled: {config.vehicle.control_input_noise_enabled}")
    print()

    # Create vehicle model
    vehicle = VehicleModel(config.vehicle, initial_state)

    # Simulation parameters
    time_step = 0.1
    total_time = 30.0  # Longer simulation to show drift accumulation
    steps = int(total_time / time_step)

    # Create a complex control sequence - figure-8 pattern
    control_sequence = []
    for i in range(steps):
        t = i * time_step
        if t < 5.0:
            # Accelerate forward
            steering_rate = 0.0
            acceleration = 1.0
        elif t < 10.0:
            # Turn left (first loop)
            steering_rate = 0.4
            acceleration = 0.0
        elif t < 15.0:
            # Continue left turn
            steering_rate = 0.3
            acceleration = 0.0
        elif t < 20.0:
            # Turn right (second loop)
            steering_rate = -0.6
            acceleration = 0.0
        elif t < 25.0:
            # Complete the figure-8
            steering_rate = -0.3
            acceleration = 0.0
        else:
            # Straighten out
            steering_rate = 0.1
            acceleration = 0.0

        control_sequence.append((steering_rate, acceleration))

    # Simulate vehicle and collect all three state types
    print("Running simulation...")
    true_states = []
    odometry_states = []
    global_states = []
    times = []

    for i, control in enumerate(control_sequence):
        current_time = i * time_step
        
        # Update vehicle with control input
        vehicle.update_with_rates(control, time_step)
        
        # Get all three state types
        true_state = vehicle.get_true_state()
        odometry_state = vehicle.get_odometry_state()
        global_state = vehicle.get_global_state()
        
        # Store states
        true_states.append(true_state.copy())
        odometry_states.append(odometry_state.copy())
        global_states.append(global_state.copy())
        times.append(current_time)

    # Extract trajectories for plotting
    true_x = [state.position_x for state in true_states]
    true_y = [state.position_y for state in true_states]
    odometry_x = [state.position_x for state in odometry_states]
    odometry_y = [state.position_y for state in odometry_states]
    global_x = [state.position_x for state in global_states]
    global_y = [state.position_y for state in global_states]

    # Calculate errors relative to true state
    time_array = np.array(times)
    odometry_error = np.sqrt(
        (np.array(odometry_x) - np.array(true_x)) ** 2 + 
        (np.array(odometry_y) - np.array(true_y)) ** 2
    )
    global_error = np.sqrt(
        (np.array(global_x) - np.array(true_x)) ** 2 + 
        (np.array(global_y) - np.array(true_y)) ** 2
    )

    # Calculate yaw errors
    true_yaw = [state.yaw_angle for state in true_states]
    odometry_yaw = [state.yaw_angle for state in odometry_states]
    global_yaw = [state.yaw_angle for state in global_states]
    
    odometry_yaw_error = []
    global_yaw_error = []
    for i in range(len(true_yaw)):
        # Handle angle wrapping
        odom_err = abs(odometry_yaw[i] - true_yaw[i])
        if odom_err > np.pi:
            odom_err = 2*np.pi - odom_err
        odometry_yaw_error.append(np.degrees(odom_err))
        
        global_err = abs(global_yaw[i] - true_yaw[i])
        if global_err > np.pi:
            global_err = 2*np.pi - global_err
        global_yaw_error.append(np.degrees(global_err))

    # Plot results
    plt.figure(figsize=(18, 12))

    # Trajectory comparison
    plt.subplot(3, 3, 1)
    plt.plot(true_x, true_y, "b-", linewidth=3, label="True state", alpha=0.9)
    plt.plot(odometry_x, odometry_y, "r-", linewidth=2, label="Odometry", alpha=0.7)
    plt.plot(global_x, global_y, "g-", linewidth=2, label="Global localization", alpha=0.7)
    plt.xlabel("X Position [m]")
    plt.ylabel("Y Position [m]")
    plt.title("Vehicle Trajectory Comparison")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis("equal")

    # Position error over time
    plt.subplot(3, 3, 2)
    plt.plot(time_array, odometry_error, "r-", linewidth=2, label="Odometry error")
    plt.plot(time_array, global_error, "g-", linewidth=2, label="Global localization error")
    plt.xlabel("Time [s]")
    plt.ylabel("Position Error [m]")
    plt.title("Position Error Over Time")
    plt.legend()
    plt.grid(True, alpha=0.3)

    # Yaw error over time
    plt.subplot(3, 3, 3)
    plt.plot(time_array, odometry_yaw_error, "r-", linewidth=2, label="Odometry yaw error")
    plt.plot(time_array, global_yaw_error, "g-", linewidth=2, label="Global yaw error")
    plt.xlabel("Time [s]")
    plt.ylabel("Yaw Error [degrees]")
    plt.title("Yaw Error Over Time")
    plt.legend()
    plt.grid(True, alpha=0.3)

    # X position over time
    plt.subplot(3, 3, 4)
    plt.plot(time_array, true_x, "b-", linewidth=3, label="True", alpha=0.9)
    plt.plot(time_array, odometry_x, "r-", linewidth=2, label="Odometry", alpha=0.7)
    plt.plot(time_array, global_x, "g-", linewidth=2, label="Global localization", alpha=0.7)
    plt.xlabel("Time [s]")
    plt.ylabel("X Position [m]")
    plt.title("X Position Over Time")
    plt.legend()
    plt.grid(True, alpha=0.3)

    # Y position over time
    plt.subplot(3, 3, 5)
    plt.plot(time_array, true_y, "b-", linewidth=3, label="True", alpha=0.9)
    plt.plot(time_array, odometry_y, "r-", linewidth=2, label="Odometry", alpha=0.7)
    plt.plot(time_array, global_y, "g-", linewidth=2, label="Global localization", alpha=0.7)
    plt.xlabel("Time [s]")
    plt.ylabel("Y Position [m]")
    plt.title("Y Position Over Time")
    plt.legend()
    plt.grid(True, alpha=0.3)

    # Yaw angle comparison
    plt.subplot(3, 3, 6)
    true_yaw_deg = [np.degrees(yaw) for yaw in true_yaw]
    odometry_yaw_deg = [np.degrees(yaw) for yaw in odometry_yaw]
    global_yaw_deg = [np.degrees(yaw) for yaw in global_yaw]
    
    plt.plot(time_array, true_yaw_deg, "b-", linewidth=3, label="True", alpha=0.9)
    plt.plot(time_array, odometry_yaw_deg, "r-", linewidth=2, label="Odometry", alpha=0.7)
    plt.plot(time_array, global_yaw_deg, "g-", linewidth=2, label="Global localization", alpha=0.7)
    plt.xlabel("Time [s]")
    plt.ylabel("Yaw Angle [degrees]")
    plt.title("Yaw Angle Over Time")
    plt.legend()
    plt.grid(True, alpha=0.3)

    # Velocity comparison
    plt.subplot(3, 3, 7)
    true_vel = [state.velocity for state in true_states]
    odometry_vel = [state.velocity for state in odometry_states]
    global_vel = [state.velocity for state in global_states]
    
    plt.plot(time_array, true_vel, "b-", linewidth=3, label="True", alpha=0.9)
    plt.plot(time_array, odometry_vel, "r-", linewidth=2, label="Odometry", alpha=0.7)
    plt.plot(time_array, global_vel, "g-", linewidth=2, label="Global localization", alpha=0.7)
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity [m/s]")
    plt.title("Velocity Over Time")
    plt.legend()
    plt.grid(True, alpha=0.3)

    # Error statistics comparison
    plt.subplot(3, 3, 8)
    error_stats = {
        'Mean': [np.mean(odometry_error), np.mean(global_error)],
        'Max': [np.max(odometry_error), np.max(global_error)],
        'Final': [odometry_error[-1], global_error[-1]],
        'RMS': [np.sqrt(np.mean(odometry_error**2)), np.sqrt(np.mean(global_error**2))]
    }
    
    x_pos = np.arange(len(error_stats))
    width = 0.35
    
    odometry_values = [error_stats[key][0] for key in error_stats.keys()]
    global_values = [error_stats[key][1] for key in error_stats.keys()]
    
    plt.bar(x_pos - width/2, odometry_values, width, label='Odometry', color='red', alpha=0.7)
    plt.bar(x_pos + width/2, global_values, width, label='Global Localization', color='green', alpha=0.7)
    
    plt.xlabel('Error Metric')
    plt.ylabel('Position Error [m]')
    plt.title('Position Error Statistics')
    plt.xticks(x_pos, list(error_stats.keys()))
    plt.legend()
    plt.grid(True, alpha=0.3)

    # Drift analysis
    plt.subplot(3, 3, 9)
    # Calculate moving average of errors to show drift trends
    window_size = 50
    if len(odometry_error) >= window_size:
        odometry_trend = np.convolve(odometry_error, np.ones(window_size)/window_size, mode='valid')
        global_trend = np.convolve(global_error, np.ones(window_size)/window_size, mode='valid')
        trend_time = time_array[window_size-1:]
        
        plt.plot(trend_time, odometry_trend, "r-", linewidth=3, label="Odometry drift trend")
        plt.plot(trend_time, global_trend, "g-", linewidth=3, label="Global localization trend")
        plt.xlabel("Time [s]")
        plt.ylabel("Average Error [m]")
        plt.title("Error Drift Trends (Moving Average)")
        plt.legend()
        plt.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

    # Print detailed statistics
    print("\n=== Simulation Results ===")
    print(f"Simulation time: {total_time} s")
    print(f"Time steps: {steps}")
    print()
    
    print("Final States:")
    print(f"  True state: pos=({true_x[-1]:.2f}, {true_y[-1]:.2f}), yaw={np.degrees(true_yaw[-1]):.1f}°, vel={true_vel[-1]:.2f}m/s")
    print(f"  Odometry:   pos=({odometry_x[-1]:.2f}, {odometry_y[-1]:.2f}), yaw={np.degrees(odometry_yaw[-1]):.1f}°, vel={odometry_vel[-1]:.2f}m/s")
    print(f"  Global loc: pos=({global_x[-1]:.2f}, {global_y[-1]:.2f}), yaw={np.degrees(global_yaw[-1]):.1f}°, vel={global_vel[-1]:.2f}m/s")
    print()
    
    print("Odometry (Dead Reckoning) Statistics:")
    print(f"  Final position error: {odometry_error[-1]:.3f} m")
    print(f"  Mean position error: {np.mean(odometry_error):.3f} m")
    print(f"  Max position error: {np.max(odometry_error):.3f} m")
    print(f"  RMS position error: {np.sqrt(np.mean(odometry_error**2)):.3f} m")
    print(f"  Final yaw error: {odometry_yaw_error[-1]:.2f}°")
    print(f"  Mean yaw error: {np.mean(odometry_yaw_error):.2f}°")
    print(f"  Error growth rate: {(odometry_error[-1] - odometry_error[0]) / total_time:.3f} m/s")
    print()
    
    print("Global Localization (GPS-like) Statistics:")
    print(f"  Final position error: {global_error[-1]:.3f} m")
    print(f"  Mean position error: {np.mean(global_error):.3f} m")
    print(f"  Max position error: {np.max(global_error):.3f} m")
    print(f"  RMS position error: {np.sqrt(np.mean(global_error**2)):.3f} m")
    print(f"  Final yaw error: {global_yaw_error[-1]:.2f}°")
    print(f"  Mean yaw error: {np.mean(global_yaw_error):.2f}°")
    print(f"  Error growth rate: {(global_error[-1] - global_error[0]) / total_time:.3f} m/s")
    print()
    
    print("Improvement with Global Localization:")
    improvement_final = ((odometry_error[-1] - global_error[-1]) / odometry_error[-1]) * 100
    improvement_mean = ((np.mean(odometry_error) - np.mean(global_error)) / np.mean(odometry_error)) * 100
    improvement_rms = ((np.sqrt(np.mean(odometry_error**2)) - np.sqrt(np.mean(global_error**2))) / np.sqrt(np.mean(odometry_error**2))) * 100
    
    print(f"  Final error reduction: {improvement_final:.1f}%")
    print(f"  Mean error reduction: {improvement_mean:.1f}%")
    print(f"  RMS error reduction: {improvement_rms:.1f}%")


def demonstrate_state_types():
    """Demonstrate accessing different state types from the same vehicle"""
    
    print("\n=== State Type Access Demo ===\n")
    
    # Create a vehicle with noise enabled
    config = load_config()
    config.vehicle.noise_seed = 42
    config.vehicle.odometry_position_noise_std = 0.02
    config.vehicle.global_position_noise_std = 0.5
    config.vehicle.control_input_noise_enabled = True
    
    vehicle = VehicleModel(config.vehicle)
    
    print("Running vehicle with noise enabled...")
    print("State comparison at each step:")
    print("Step | True State          | Odometry State      | Global State")
    print("-----|---------------------|---------------------|---------------------")
    
    # Test with a simple control input
    control = (0.1, 0.5)  # steering_rate, acceleration
    time_step = 0.1
    
    # Run several steps
    for i in range(5):
        vehicle.update_with_rates(control, time_step)
        
        true_state = vehicle.get_true_state()
        odom_state = vehicle.get_odometry_state()
        global_state = vehicle.get_global_state()
        
        print(f"{i+1:4d} | ({true_state.position_x:6.3f}, {true_state.position_y:6.3f}) | ({odom_state.position_x:6.3f}, {odom_state.position_y:6.3f}) | ({global_state.position_x:6.3f}, {global_state.position_y:6.3f})")
    
    print("\nDemonstrating state type parameter:")
    print(f"get_state('true'):     {vehicle.get_state('true').position_x:.3f}")
    print(f"get_state('odometry'): {vehicle.get_state('odometry').position_x:.3f}")
    print(f"get_state('global'):   {vehicle.get_state('global').position_x:.3f}")
    print(f"get_state():           {vehicle.get_state().position_x:.3f} (default: {config.vehicle.default_state_type})")


if __name__ == "__main__":
    # Run demonstrations
    compare_state_estimators()
    demonstrate_state_types()

    print("\n=== State Estimator Comparison Complete ===")
    print("Key Characteristics:")
    print("1. True State:")
    print("   - Perfect state without any noise or errors")
    print("   - Represents the actual vehicle motion")
    print("   - Used as ground truth for error analysis")
    print()
    print("2. Odometry State (Dead Reckoning):")
    print("   - Based on wheel encoders and IMU sensors")
    print("   - Errors accumulate over time (unbounded drift)")
    print("   - Noise proportional to distance traveled and rotation")
    print("   - Good for short-term, high-frequency state estimation")
    print()
    print("3. Global Localization State (GPS-like):")
    print("   - Based on GPS/GNSS or other global positioning systems")
    print("   - Bounded error that doesn't grow indefinitely")
    print("   - Periodic measurements with delays")
    print("   - Lower frequency but prevents long-term drift")
    print()
    print("Usage Guidelines:")
    print("- Use 'true' for simulation ground truth and algorithm testing")
    print("- Use 'odometry' for high-frequency control and short-term planning")
    print("- Use 'global' for long-term navigation and drift correction")
    print("- Combine both in real applications for optimal performance") 