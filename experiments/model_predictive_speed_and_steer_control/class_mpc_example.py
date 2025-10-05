#!/usr/bin/env python3
"""
Example demonstrating how to use the MPCSolver class.

This script shows how to:
1. Create and configure an MPCSolver instance
2. Use the class methods for MPC optimization
3. Update parameters dynamically
4. Get solver information and statistics
"""

import math
import numpy as np
import matplotlib.pyplot as plt
import sys
import pathlib

# Add parent directory to path for imports
sys.path.append(str(pathlib.Path(__file__).parent.parent))

from mpc_core import MPCSolver


def create_test_reference_trajectory():
    """Create a test reference trajectory."""
    prediction_horizon = 5
    reference_trajectory = np.zeros((4, prediction_horizon + 1))
    reference_steering = np.zeros((1, prediction_horizon + 1))
    
    # Create a curved path
    for i in range(prediction_horizon + 1):
        t = i * 0.2  # time step
        reference_trajectory[0, i] = t * 3.0  # x position
        reference_trajectory[1, i] = 0.1 * t**2  # y position (curve)
        reference_trajectory[2, i] = 2.0  # velocity
        reference_trajectory[3, i] = math.atan2(0.2 * t, 3.0)  # yaw angle
        reference_steering[0, i] = 0.0  # steering reference
    
    return reference_trajectory, reference_steering


def create_linearization_trajectory(initial_state, reference_trajectory):
    """Create linearization trajectory."""
    prediction_horizon = reference_trajectory.shape[1] - 1
    linearization_trajectory = np.zeros((4, prediction_horizon + 1))
    initial_state_array = np.array(initial_state)
    
    # Start with initial state
    linearization_trajectory[:, 0] = initial_state_array
    
    # Interpolate towards reference trajectory
    for i in range(1, prediction_horizon + 1):
        alpha = i / prediction_horizon
        linearization_trajectory[:, i] = (1 - alpha) * initial_state_array + alpha * reference_trajectory[:, i]
    
    return linearization_trajectory


def demonstrate_basic_usage():
    """Demonstrate basic usage of MPCSolver class."""
    print("=== Basic MPCSolver Usage ===")
    
    # 1. Create MPC solver with default parameters
    print("1. Creating MPC solver with default parameters...")
    mpc_solver = MPCSolver()
    print(f"   Prediction horizon: {mpc_solver.prediction_horizon}")
    print(f"   Time step: {mpc_solver.time_step}")
    print(f"   Max velocity: {mpc_solver.max_velocity:.2f} m/s")
    
    # 2. Create test data
    print("\n2. Creating test reference trajectory...")
    reference_trajectory, reference_steering = create_test_reference_trajectory()
    initial_state = [0.0, 0.0, 0.0, 0.0]
    linearization_trajectory = create_linearization_trajectory(initial_state, reference_trajectory)
    
    # 3. Solve MPC
    print("\n3. Solving MPC optimization...")
    result = mpc_solver.solve(
        reference_trajectory,
        linearization_trajectory,
        initial_state,
        reference_steering
    )
    
    acceleration_sequence, steering_sequence, predicted_x, predicted_y, predicted_yaw, predicted_velocity = result
    
    # 4. Display results
    if acceleration_sequence is not None:
        print("   ✓ MPC solved successfully!")
        print(f"   Acceleration sequence: {acceleration_sequence}")
        steering_deg = [f"{math.degrees(s):.2f}" for s in steering_sequence]
        print(f"   Steering sequence: {steering_deg}°")
        
        # Get solver information
        solver_info = mpc_solver.get_solver_info()
        print(f"   Solve time: {solver_info['solve_time']:.4f} seconds")
        print(f"   Solve status: {solver_info['status']}")
    else:
        print("   ✗ MPC failed to solve!")
        return None
    
    return mpc_solver, result


def demonstrate_custom_parameters():
    """Demonstrate custom parameter configuration."""
    print("\n=== Custom Parameter Configuration ===")
    
    # Create MPC solver with custom parameters
    print("1. Creating MPC solver with custom parameters...")
    mpc_solver = MPCSolver(
        prediction_horizon=8,  # Longer prediction horizon
        time_step=0.1,         # Smaller time step
        max_velocity=10.0,     # Lower max velocity
        max_acceleration=0.5,  # Lower max acceleration
        input_cost_weights=[0.1, 0.1],  # Higher control cost
        state_cost_weights=[2.0, 2.0, 1.0, 1.0]  # Higher state tracking cost
    )
    
    print(f"   Prediction horizon: {mpc_solver.prediction_horizon}")
    print(f"   Time step: {mpc_solver.time_step}")
    print(f"   Max velocity: {mpc_solver.max_velocity:.2f} m/s")
    print(f"   Max acceleration: {mpc_solver.max_acceleration:.2f} m/s²")
    
    # Create test data with longer horizon
    reference_trajectory, reference_steering = create_test_reference_trajectory()
    # Extend trajectory for longer horizon
    extended_reference = np.zeros((4, 9))
    extended_steering = np.zeros((1, 9))
    for i in range(9):
        t = i * 0.1
        extended_reference[0, i] = t * 3.0
        extended_reference[1, i] = 0.1 * t**2
        extended_reference[2, i] = 2.0
        extended_reference[3, i] = math.atan2(0.2 * t, 3.0)
        extended_steering[0, i] = 0.0
    
    initial_state = [0.0, 0.0, 0.0, 0.0]
    linearization_trajectory = create_linearization_trajectory(initial_state, extended_reference)
    
    # Solve MPC
    print("\n2. Solving MPC with custom parameters...")
    result = mpc_solver.solve(
        extended_reference,
        linearization_trajectory,
        initial_state,
        extended_steering
    )
    
    if result[0] is not None:
        print("   ✓ MPC solved successfully!")
        solver_info = mpc_solver.get_solver_info()
        print(f"   Solve time: {solver_info['solve_time']:.4f} seconds")
    else:
        print("   ✗ MPC failed to solve!")
    
    return mpc_solver, result


def demonstrate_parameter_updates():
    """Demonstrate dynamic parameter updates."""
    print("\n=== Dynamic Parameter Updates ===")
    
    # Create solver
    mpc_solver = MPCSolver()
    print("1. Initial parameters:")
    print(f"   Max velocity: {mpc_solver.max_velocity:.2f} m/s")
    print(f"   Max acceleration: {mpc_solver.max_acceleration:.2f} m/s²")
    
    # Update parameters
    print("\n2. Updating parameters...")
    mpc_solver.update_parameters(
        max_velocity=5.0,
        max_acceleration=0.8,
        state_cost_weights=[2.0, 2.0, 1.0, 1.0]
    )
    
    print("   Updated parameters:")
    print(f"   Max velocity: {mpc_solver.max_velocity:.2f} m/s")
    print(f"   Max acceleration: {mpc_solver.max_acceleration:.2f} m/s²")
    print(f"   State cost matrix: {np.diag(mpc_solver.state_cost_matrix)}")
    
    return mpc_solver


def demonstrate_multiple_solves():
    """Demonstrate multiple solves with the same solver instance."""
    print("\n=== Multiple Solves ===")
    
    mpc_solver = MPCSolver()
    reference_trajectory, reference_steering = create_test_reference_trajectory()
    
    # Test different initial states
    initial_states = [
        [0.0, 0.0, 0.0, 0.0],      # Start at origin
        [1.0, 0.5, 1.0, 0.1],      # Offset position
        [0.0, 0.0, 3.0, 0.0],      # Moving forward
        [0.0, 0.0, 0.0, math.pi/4], # Rotated
    ]
    
    solve_times = []
    
    for i, initial_state in enumerate(initial_states):
        print(f"\nSolve {i+1}: Initial state {initial_state}")
        
        linearization_trajectory = create_linearization_trajectory(initial_state, reference_trajectory)
        
        result = mpc_solver.solve(
            reference_trajectory,
            linearization_trajectory,
            initial_state,
            reference_steering
        )
        
        if result[0] is not None:
            solver_info = mpc_solver.get_solver_info()
            solve_times.append(solver_info['solve_time'])
            print(f"   ✓ Solved in {solver_info['solve_time']:.4f} seconds")
        else:
            print("   ✗ Failed to solve!")
    
    if solve_times:
        print(f"\nAverage solve time: {np.mean(solve_times):.4f} seconds")
        print(f"Min solve time: {np.min(solve_times):.4f} seconds")
        print(f"Max solve time: {np.max(solve_times):.4f} seconds")


def visualize_results(mpc_solver, result, title="MPC Results"):
    """Visualize MPC results."""
    if result is None or result[0] is None:
        print("No results to visualize!")
        return
    
    acceleration_sequence, steering_sequence, predicted_x, predicted_y, predicted_yaw, predicted_velocity = result
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle(title, fontsize=14)
    
    # Plot 1: Trajectory
    ax1 = axes[0, 0]
    ax1.plot(predicted_x, predicted_y, 'b-s', label='Predicted trajectory', markersize=6, linewidth=2)
    ax1.plot(predicted_x[0], predicted_y[0], 'go', label='Start', markersize=10)
    ax1.plot(predicted_x[-1], predicted_y[-1], 'ro', label='End', markersize=10)
    ax1.set_xlabel('X position [m]')
    ax1.set_ylabel('Y position [m]')
    ax1.legend()
    ax1.grid(True)
    ax1.axis('equal')
    ax1.set_title('Predicted Trajectory')
    
    # Plot 2: Velocity profile
    ax2 = axes[0, 1]
    time_steps = np.arange(len(predicted_velocity)) * mpc_solver.time_step
    ax2.plot(time_steps, predicted_velocity, 'b-s', label='Predicted velocity', markersize=6, linewidth=2)
    ax2.axhline(y=mpc_solver.max_velocity, color='r', linestyle='--', label='Max velocity')
    ax2.axhline(y=mpc_solver.min_velocity, color='r', linestyle='--', label='Min velocity')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Velocity [m/s]')
    ax2.legend()
    ax2.grid(True)
    ax2.set_title('Velocity Profile')
    
    # Plot 3: Control inputs
    ax3 = axes[1, 0]
    control_time_steps = np.arange(len(acceleration_sequence)) * mpc_solver.time_step
    ax3.plot(control_time_steps, acceleration_sequence, 'g-s', label='Acceleration', markersize=6, linewidth=2)
    ax3.axhline(y=mpc_solver.max_acceleration, color='r', linestyle='--', label='Max acceleration')
    ax3.axhline(y=-mpc_solver.max_acceleration, color='r', linestyle='--', label='Min acceleration')
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Acceleration [m/s²]')
    ax3.legend()
    ax3.grid(True)
    ax3.set_title('Acceleration Control')
    
    # Plot 4: Steering angle
    ax4 = axes[1, 1]
    ax4.plot(control_time_steps, [math.degrees(s) for s in steering_sequence], 'm-s', 
             label='Steering angle', markersize=6, linewidth=2)
    ax4.axhline(y=math.degrees(mpc_solver.max_steering_angle), color='r', linestyle='--', label='Max steering')
    ax4.axhline(y=-math.degrees(mpc_solver.max_steering_angle), color='r', linestyle='--', label='Min steering')
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Steering angle [deg]')
    ax4.legend()
    ax4.grid(True)
    ax4.set_title('Steering Control')
    
    plt.tight_layout()
    plt.show()


def main():
    """Main demonstration function."""
    print("MPCSolver Class Demonstration")
    print("=" * 50)
    
    # Basic usage
    mpc_solver, result = demonstrate_basic_usage()
    
    # Custom parameters
    custom_solver, custom_result = demonstrate_custom_parameters()
    
    # Parameter updates
    updated_solver = demonstrate_parameter_updates()
    
    # Multiple solves
    demonstrate_multiple_solves()
    
    # Visualization
    if result is not None:
        print("\n=== Visualization ===")
        visualize_results(mpc_solver, result, "Basic MPC Results")
    
    print("\n" + "=" * 50)
    print("Class demonstration completed!")


if __name__ == "__main__":
    main()
