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
    
    # Load configuration and enable noise
    config = load_config()
    config.vehicle.noise_enabled = True
    config.vehicle.position_noise_std = 0.05  # Increase noise for visibility
    config.vehicle.yaw_noise_std = 0.02
    config.vehicle.velocity_noise_std = 0.1
    config.vehicle.steering_noise_std = 0.02
    config.vehicle.process_noise_std = 0.01
    config.vehicle.noise_seed = 42  # For reproducible results
    
    print("Noise Configuration:")
    print(f"  Noise enabled: {config.vehicle.noise_enabled}")
    print(f"  Position noise std: {config.vehicle.position_noise_std} m")
    print(f"  Yaw noise std: {config.vehicle.yaw_noise_std} rad")
    print(f"  Velocity noise std: {config.vehicle.velocity_noise_std} m/s")
    print(f"  Steering noise std: {config.vehicle.steering_noise_std} rad")
    print(f"  Process noise std: {config.vehicle.process_noise_std}")
    print(f"  Noise seed: {config.vehicle.noise_seed}")
    print()
    
    # Create initial state
    initial_state = VehicleState(
        position_x=0.0,
        position_y=0.0,
        yaw_angle=0.0,
        velocity=0.0,
        steering_angle=0.0
    )
    
    # Create two vehicle models - one with noise, one without
    vehicle_clean = VehicleModel(config.vehicle, initial_state)
    vehicle_clean.set_noise_enabled(False)
    
    vehicle_noisy = VehicleModel(config.vehicle, initial_state)
    vehicle_noisy.set_noise_enabled(True)
    vehicle_noisy.reset_noise_seed(42)  # Same seed for reproducibility
    
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
    
    # Simulate both vehicles
    print("Running simulation...")
    clean_trajectory = []
    noisy_trajectory = []
    
    for control in control_sequence:
        # Update both vehicles with the same control input
        clean_state = vehicle_clean.update_with_rates(control, time_step)
        noisy_state = vehicle_noisy.update_with_rates(control, time_step)
        
        clean_trajectory.append(clean_state.copy())
        noisy_trajectory.append(noisy_state.copy())
    
    # Extract trajectories for plotting
    clean_x = [state.position_x for state in clean_trajectory]
    clean_y = [state.position_y for state in clean_trajectory]
    noisy_x = [state.position_x for state in noisy_trajectory]
    noisy_y = [state.position_y for state in noisy_trajectory]
    
    # Plot results
    plt.figure(figsize=(12, 8))
    
    # Trajectory comparison
    plt.subplot(2, 2, 1)
    plt.plot(clean_x, clean_y, 'b-', linewidth=2, label='Clean (no noise)')
    plt.plot(noisy_x, noisy_y, 'r-', linewidth=1, alpha=0.7, label='Noisy')
    plt.xlabel('X Position [m]')
    plt.ylabel('Y Position [m]')
    plt.title('Vehicle Trajectory Comparison')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
    # Position error over time
    plt.subplot(2, 2, 2)
    time_array = np.array(range(len(clean_trajectory))) * time_step
    position_error = np.sqrt((np.array(noisy_x) - np.array(clean_x))**2 + 
                            (np.array(noisy_y) - np.array(clean_y))**2)
    plt.plot(time_array, position_error, 'g-', linewidth=2)
    plt.xlabel('Time [s]')
    plt.ylabel('Position Error [m]')
    plt.title('Position Error Due to Noise')
    plt.grid(True, alpha=0.3)
    
    # Velocity comparison
    plt.subplot(2, 2, 3)
    clean_vel = [state.velocity for state in clean_trajectory]
    noisy_vel = [state.velocity for state in noisy_trajectory]
    plt.plot(time_array, clean_vel, 'b-', linewidth=2, label='Clean')
    plt.plot(time_array, noisy_vel, 'r-', linewidth=1, alpha=0.7, label='Noisy')
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [m/s]')
    plt.title('Velocity Comparison')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Steering angle comparison
    plt.subplot(2, 2, 4)
    clean_steer = [np.degrees(state.steering_angle) for state in clean_trajectory]
    noisy_steer = [np.degrees(state.steering_angle) for state in noisy_trajectory]
    plt.plot(time_array, clean_steer, 'b-', linewidth=2, label='Clean')
    plt.plot(time_array, noisy_steer, 'r-', linewidth=1, alpha=0.7, label='Noisy')
    plt.xlabel('Time [s]')
    plt.ylabel('Steering Angle [deg]')
    plt.title('Steering Angle Comparison')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    # Print statistics
    print("\nSimulation Results:")
    print(f"Final position (clean): ({clean_x[-1]:.2f}, {clean_y[-1]:.2f})")
    print(f"Final position (noisy): ({noisy_x[-1]:.2f}, {noisy_y[-1]:.2f})")
    print(f"Final position error: {position_error[-1]:.3f} m")
    print(f"Mean position error: {np.mean(position_error):.3f} m")
    print(f"Max position error: {np.max(position_error):.3f} m")
    print(f"RMS position error: {np.sqrt(np.mean(position_error**2)):.3f} m")


def demonstrate_noise_control():
    """Demonstrate noise control functionality"""
    
    print("\n=== Noise Control Demonstration ===\n")
    
    # Load configuration
    config = load_config()
    config.vehicle.noise_enabled = True
    config.vehicle.position_noise_std = 0.1
    
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