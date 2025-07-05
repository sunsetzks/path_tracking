#!/usr/bin/env python3
"""
Simple Noise Estimator Example

This example demonstrates how to use the SimpleNoiseEstimator which adds
basic Gaussian noise to the true vehicle state without accumulation.

Author: Assistant
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import sys

# Add the parent directory to the path to import PathTracking modules
sys.path.append(str(Path(__file__).parent.parent))

from PathTracking.config import load_config
from PathTracking.vehicle_model import VehicleModel, VehicleState


def simple_noise_example():
    """Demonstrate the simple noise estimator"""
    
    print("=== Simple Noise Estimator Example ===\n")
    
    # Create initial state
    initial_state = VehicleState(
        position_x=0.0, 
        position_y=0.0, 
        yaw_angle=0.0, 
        velocity=0.0, 
        steering_angle=0.0
    )
    
    # Configure vehicle with simple noise
    config = load_config()
    config.vehicle.noise_seed = 42  # For reproducible results
    
    # Simple noise parameters
    config.vehicle.simple_position_noise_std = 0.2  # 20cm position noise
    config.vehicle.simple_yaw_noise_std = 0.05  # ~3 degree yaw noise
    config.vehicle.simple_velocity_noise_std = 0.1  # 10cm/s velocity noise
    
    # Set default state type to use simple noise
    config.vehicle.default_state_type = "simple"
    
    print("Configuration:")
    print(f"  Simple position noise std: {config.vehicle.simple_position_noise_std} m")
    print(f"  Simple yaw noise std: {config.vehicle.simple_yaw_noise_std} rad ({np.degrees(config.vehicle.simple_yaw_noise_std):.2f}°)")
    print(f"  Simple velocity noise std: {config.vehicle.simple_velocity_noise_std} m/s")
    print(f"  Default state type: {config.vehicle.default_state_type}")
    print()
    
    # Create vehicle model
    vehicle = VehicleModel(config.vehicle, initial_state)
    
    # Simulation parameters
    time_step = 0.1
    total_time = 5.0
    steps = int(total_time / time_step)
    
    # Control sequence - circular motion
    control_sequence = []
    for i in range(steps):
        t = i * time_step
        steering_rate = 0.1 * np.sin(0.5 * t)  # Sinusoidal steering
        acceleration = 0.5 if t < 2.0 else 0.0  # Accelerate for first 2 seconds
        control_sequence.append((steering_rate, acceleration))
    
    # Storage for results
    true_states = []
    simple_states = []
    default_states = []  # This will be simple states since default_state_type = "simple"
    times = []
    
    # Run simulation
    print("Running simulation...")
    for i, control in enumerate(control_sequence):
        # Update vehicle
        vehicle.update_with_rates(control, time_step)
        
        # Store states
        true_states.append(vehicle.get_true_state())
        simple_states.append(vehicle.get_simple_state())
        default_states.append(vehicle.get_state())  # Uses default_state_type
        times.append(i * time_step)
    
    print(f"Simulation completed: {len(true_states)} steps")
    print()
    
    # Verify that default state is simple state
    print("=== Verification ===")
    print(f"Default state type: {config.vehicle.default_state_type}")
    print(f"Final true state:    {true_states[-1]}")
    print(f"Final simple state:  {simple_states[-1]}")
    print(f"Final default state: {default_states[-1]}")
    print(f"Simple == Default: {simple_states[-1].to_array().tolist() == default_states[-1].to_array().tolist()}")
    print()
    
    # Calculate noise statistics
    position_errors = []
    for true_state, simple_state in zip(true_states, simple_states):
        pos_error = np.sqrt(
            (simple_state.position_x - true_state.position_x)**2 + 
            (simple_state.position_y - true_state.position_y)**2
        )
        position_errors.append(pos_error)
    
    print("=== Noise Statistics ===")
    print(f"Position error mean: {np.mean(position_errors):.4f} m")
    print(f"Position error std:  {np.std(position_errors):.4f} m")
    print(f"Expected noise std:  {config.vehicle.simple_position_noise_std} m")
    print()
    
    # Plot results
    plt.figure(figsize=(12, 8))
    
    # Position trajectory
    plt.subplot(2, 2, 1)
    true_x = [state.position_x for state in true_states]
    true_y = [state.position_y for state in true_states]
    simple_x = [state.position_x for state in simple_states]
    simple_y = [state.position_y for state in simple_states]
    
    plt.plot(true_x, true_y, 'b-', label='True Position', linewidth=2)
    plt.plot(simple_x, simple_y, 'r.', label='Simple Noise Position', alpha=0.7, markersize=4)
    plt.xlabel('X Position [m]')
    plt.ylabel('Y Position [m]')
    plt.title('Vehicle Trajectory')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    
    # Position error over time
    plt.subplot(2, 2, 2)
    plt.plot(times, position_errors, 'g-', linewidth=1.5)
    plt.axhline(y=config.vehicle.simple_position_noise_std, color='r', linestyle='--', 
                label=f'Expected Noise Std ({config.vehicle.simple_position_noise_std}m)')
    plt.xlabel('Time [s]')
    plt.ylabel('Position Error [m]')
    plt.title('Position Error Over Time')
    plt.legend()
    plt.grid(True)
    
    # Velocity comparison
    plt.subplot(2, 2, 3)
    true_vel = [state.velocity for state in true_states]
    simple_vel = [state.velocity for state in simple_states]
    
    plt.plot(times, true_vel, 'b-', label='True Velocity', linewidth=2)
    plt.plot(times, simple_vel, 'r.', label='Simple Noise Velocity', alpha=0.7, markersize=4)
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [m/s]')
    plt.title('Velocity Comparison')
    plt.legend()
    plt.grid(True)
    
    # Yaw angle comparison
    plt.subplot(2, 2, 4)
    true_yaw = [np.degrees(state.yaw_angle) for state in true_states]
    simple_yaw = [np.degrees(state.yaw_angle) for state in simple_states]
    
    plt.plot(times, true_yaw, 'b-', label='True Yaw', linewidth=2)
    plt.plot(times, simple_yaw, 'r.', label='Simple Noise Yaw', alpha=0.7, markersize=4)
    plt.xlabel('Time [s]')
    plt.ylabel('Yaw Angle [degrees]')
    plt.title('Yaw Angle Comparison')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()
    
    print("Example completed successfully!")
    print("\nKey Features of Simple Noise Estimator:")
    print("- Adds independent Gaussian noise to each state component")
    print("- Noise does not accumulate over time (unlike odometry)")
    print("- No measurement delays or intervals (unlike global localization)")
    print("- Suitable for basic sensor noise simulation")


if __name__ == "__main__":
    simple_noise_example() 