#!/usr/bin/env python3
"""
Control Input Noise Master Switch Example

This example demonstrates the new control_input_noise_enabled feature that allows
users to enable/disable control input noise independently from other noise types.

Key Features:
- Master switch for control input noise (process noise)
- Independent control from other noise types
- Useful for testing control algorithms with/without actuator noise

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


def demonstrate_control_input_noise_switch():
    """Demonstrate the control input noise master switch functionality"""

    print("=== Control Input Noise Master Switch Demonstration ===\n")

    # Load configuration
    config = load_config()
    config.vehicle.noise_enabled = True
    config.vehicle.control_input_noise_enabled = True
    config.vehicle.process_noise_std = 0.05  # Moderate noise level
    config.vehicle.position_noise_std = 0.01  # Small state noise
    config.vehicle.yaw_noise_std = 0.005
    config.vehicle.velocity_noise_std = 0.02
    config.vehicle.steering_noise_std = 0.01
    config.vehicle.noise_seed = 42

    print("Configuration:")
    print(f"  Overall noise enabled: {config.vehicle.noise_enabled}")
    print(f"  Control input noise enabled: {config.vehicle.control_input_noise_enabled}")
    print(f"  Process noise std: {config.vehicle.process_noise_std}")
    print(f"  Position noise std: {config.vehicle.position_noise_std}")
    print()

    # Create initial state
    initial_state = VehicleState(
        position_x=0.0,
        position_y=0.0,
        yaw_angle=0.0,
        velocity=0.0,
        steering_angle=0.0
    )

    # Create three vehicle models for comparison
    print("Creating vehicle models:")
    
    # 1. No noise at all
    config_clean = load_config()
    config_clean.vehicle.noise_enabled = False
    vehicle_clean = VehicleModel(config_clean.vehicle, initial_state)
    print(f"  Clean vehicle - All noise: {vehicle_clean.get_noise_enabled()}")
    
    # 2. All noise enabled (including control input noise)
    config_full = load_config()
    config_full.vehicle.noise_enabled = True
    config_full.vehicle.control_input_noise_enabled = True
    config_full.vehicle.process_noise_std = 0.05
    config_full.vehicle.position_noise_std = 0.01
    config_full.vehicle.yaw_noise_std = 0.005
    config_full.vehicle.velocity_noise_std = 0.02
    config_full.vehicle.steering_noise_std = 0.01
    config_full.vehicle.noise_seed = 42
    vehicle_full_noise = VehicleModel(config_full.vehicle, initial_state)
    print(f"  Full noise vehicle - All noise: {vehicle_full_noise.get_noise_enabled()}, "
          f"Control input noise: {vehicle_full_noise.get_control_input_noise_enabled()}")
    
    # 3. State noise only (no control input noise)
    config_state = load_config()
    config_state.vehicle.noise_enabled = True
    config_state.vehicle.control_input_noise_enabled = False
    config_state.vehicle.process_noise_std = 0.05
    config_state.vehicle.position_noise_std = 0.01
    config_state.vehicle.yaw_noise_std = 0.005
    config_state.vehicle.velocity_noise_std = 0.02
    config_state.vehicle.steering_noise_std = 0.01
    config_state.vehicle.noise_seed = 123  # Different seed for different behavior
    vehicle_state_noise_only = VehicleModel(config_state.vehicle, initial_state)
    print(f"  State noise only vehicle - All noise: {vehicle_state_noise_only.get_noise_enabled()}, "
          f"Control input noise: {vehicle_state_noise_only.get_control_input_noise_enabled()}")
    print()

    # Simulation parameters
    time_step = 0.1
    total_time = 10.0
    steps = int(total_time / time_step)

    # Create control sequence: accelerate, turn, maintain
    control_sequence = []
    for i in range(steps):
        t = i * time_step
        if t < 2.0:
            # Accelerate forward
            steering_rate = 0.0
            acceleration = 1.0
        elif t < 6.0:
            # Turn left while maintaining speed
            steering_rate = 0.3
            acceleration = 0.0
        else:
            # Maintain course
            steering_rate = 0.0
            acceleration = 0.0

        control_sequence.append((steering_rate, acceleration))

    print("Running simulation...")
    
    # Store trajectories
    trajectories = {
        'clean': [],
        'full_noise': [],
        'state_noise_only': []
    }
    
    vehicles = {
        'clean': vehicle_clean,
        'full_noise': vehicle_full_noise,
        'state_noise_only': vehicle_state_noise_only
    }

    # Run simulation
    for control in control_sequence:
        for name, vehicle in vehicles.items():
            updated_state = vehicle.update_with_rates(control, time_step)
            trajectories[name].append(updated_state.copy())

    print("Simulation completed.\n")

    # Debug: Check trajectory lengths and some sample points
    print("=== Trajectory Debug Info ===")
    for name, trajectory in trajectories.items():
        print(f"{name}: {len(trajectory)} points")
        if len(trajectory) > 0:
            print(f"  Start: ({trajectory[0].position_x:.3f}, {trajectory[0].position_y:.3f})")
            if len(trajectory) > 1:
                print(f"  End: ({trajectory[-1].position_x:.3f}, {trajectory[-1].position_y:.3f})")
    print()

    # Analyze results
    print("=== Analysis ===")
    
    # Extract final positions
    final_positions = {}
    for name, trajectory in trajectories.items():
        final_state = trajectory[-1]
        final_positions[name] = (final_state.position_x, final_state.position_y)
        print(f"{name.replace('_', ' ').title()} final position: "
              f"({final_state.position_x:.3f}, {final_state.position_y:.3f})")

    # Calculate differences
    clean_pos = final_positions['clean']
    full_noise_pos = final_positions['full_noise']
    state_noise_pos = final_positions['state_noise_only']

    full_noise_diff = np.sqrt((full_noise_pos[0] - clean_pos[0])**2 + 
                             (full_noise_pos[1] - clean_pos[1])**2)
    state_noise_diff = np.sqrt((state_noise_pos[0] - clean_pos[0])**2 + 
                              (state_noise_pos[1] - clean_pos[1])**2)

    print(f"\nPosition differences from clean trajectory:")
    print(f"  Full noise: {full_noise_diff:.3f} m")
    print(f"  State noise only: {state_noise_diff:.3f} m")
    print(f"  Control input noise contribution: {abs(full_noise_diff - state_noise_diff):.3f} m")

    # Demonstrate noise generator behavior
    print("\n=== Noise Generator Behavior ===")
    
    # Test noise generation directly
    from PathTracking.vehicle_model import NoiseGenerator
    
    # With control input noise enabled
    noise_gen_enabled = NoiseGenerator(config_full.vehicle)
    noise_gen_enabled.set_noise_enabled(True)
    
    steering_noise, accel_noise = noise_gen_enabled.generate_process_noise()
    print(f"Control input noise enabled:")
    print(f"  Steering rate noise: {steering_noise:.4f} rad/s")
    print(f"  Acceleration noise: {accel_noise:.4f} m/s²")
    
    # With control input noise disabled
    noise_gen_disabled = NoiseGenerator(config_state.vehicle)
    noise_gen_disabled.set_noise_enabled(True)
    
    steering_noise, accel_noise = noise_gen_disabled.generate_process_noise()
    print(f"Control input noise disabled:")
    print(f"  Steering rate noise: {steering_noise:.4f} rad/s")
    print(f"  Acceleration noise: {accel_noise:.4f} m/s²")

    # Plot results
    plot_trajectories(trajectories)
    
    print("\n=== Summary ===")
    print("✓ Control input noise can be enabled/disabled independently")
    print("✓ When disabled, only state noise affects the vehicle")
    print("✓ When enabled, both control input and state noise affect the vehicle")
    print("✓ This allows testing control algorithms with different noise scenarios")
    print("✓ Useful for evaluating robustness to actuator vs sensor noise")


def plot_trajectories(trajectories):
    """Plot the trajectories for comparison"""
    
    plt.figure(figsize=(12, 8))
    
    colors = {
        'clean': 'blue',
        'full_noise': 'red',
        'state_noise_only': 'green'
    }
    
    labels = {
        'clean': 'Clean (No Noise)',
        'full_noise': 'Full Noise (State + Control Input)',
        'state_noise_only': 'State Noise Only'
    }
    
    print("=== Plotting Debug Info ===")
    for name, trajectory in trajectories.items():
        x_coords = [state.position_x for state in trajectory]
        y_coords = [state.position_y for state in trajectory]
        
        print(f"{name}: {len(x_coords)} points")
        if len(x_coords) > 0:
            print(f"  X range: [{min(x_coords):.3f}, {max(x_coords):.3f}]")
            print(f"  Y range: [{min(y_coords):.3f}, {max(y_coords):.3f}]")
        
        plt.plot(x_coords, y_coords, color=colors[name], linewidth=2, 
                label=labels[name], alpha=0.8)
        
        # Mark start and end points
        if len(x_coords) > 0:
            plt.plot(x_coords[0], y_coords[0], 'o', color=colors[name], markersize=8)
            plt.plot(x_coords[-1], y_coords[-1], 's', color=colors[name], markersize=8)
    print()
    
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Vehicle Trajectories: Control Input Noise Comparison')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
    # Add annotations
    plt.annotate('Start', xy=(0, 0), xytext=(0.2, 0.2),
                arrowprops=dict(arrowstyle='->', color='black', alpha=0.5))
    
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    demonstrate_control_input_noise_switch() 