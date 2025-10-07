#!/usr/bin/env python3
"""
Simple example demonstrating how to use the MPC solver independently.

This script shows how to:
1. Create reference trajectories
2. Set up initial conditions
3. Solve MPC optimization
4. Visualize results
"""

import math
import numpy as np
import matplotlib.pyplot as plt
import sys
import pathlib

# Add parent directory to path for imports
sys.path.append(str(pathlib.Path(__file__).parent.parent))

from mpc_core import MPCSolver


def create_simple_reference_trajectory(prediction_horizon=5, time_step=0.2):
    """Create a simple reference trajectory for testing."""
    reference_trajectory = np.zeros((4, prediction_horizon + 1))
    reference_steering = np.zeros((1, prediction_horizon + 1))
    
    # Create a simple curved path
    for i in range(prediction_horizon + 1):
        t = i * time_step
        reference_trajectory[0, i] = t * 3.0  # x position: move forward
        reference_trajectory[1, i] = 0.1 * t**2  # y position: slight curve
        reference_trajectory[2, i] = 2.0  # velocity: constant speed
        reference_trajectory[3, i] = math.atan2(0.2 * t, 3.0)  # yaw: follow curve
        reference_steering[0, i] = 0.0  # no steering reference
    
    return reference_trajectory, reference_steering


def create_linearization_trajectory(initial_state, reference_trajectory):
    """Create linearization trajectory by interpolating between initial and reference."""
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


def visualize_mpc_results(reference_trajectory, reference_steering, initial_state, result, mpc_solver):
    """Visualize MPC results."""
    if result is None or result[0] is None:
        print("No results to visualize!")
        return
    
    acceleration_sequence, steering_sequence, predicted_x, predicted_y, predicted_yaw, predicted_velocity = result
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle("MPC Solver Results", fontsize=14)
    
    # Plot 1: Trajectory comparison
    ax1 = axes[0, 0]
    ax1.plot(reference_trajectory[0, :], reference_trajectory[1, :], 'r-o', 
             label='Reference trajectory', markersize=6, linewidth=2)
    ax1.plot(predicted_x, predicted_y, 'b-s', 
             label='Predicted trajectory', markersize=6, linewidth=2)
    ax1.plot(initial_state[0], initial_state[1], 'go', 
             label='Initial position', markersize=10)
    ax1.set_xlabel('X position [m]')
    ax1.set_ylabel('Y position [m]')
    ax1.legend()
    ax1.grid(True)
    ax1.axis('equal')
    ax1.set_title('Trajectory Comparison')
    
    # Plot 2: Velocity profile
    ax2 = axes[0, 1]
    time_steps = np.arange(mpc_solver.prediction_horizon + 1) * mpc_solver.time_step
    ax2.plot(time_steps, reference_trajectory[2, :], 'r-o', 
             label='Reference velocity', markersize=6, linewidth=2)
    ax2.plot(time_steps, predicted_velocity, 'b-s', 
             label='Predicted velocity', markersize=6, linewidth=2)
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Velocity [m/s]')
    ax2.legend()
    ax2.grid(True)
    ax2.set_title('Velocity Profile')
    
    # Plot 3: Control inputs
    ax3 = axes[1, 0]
    control_time_steps = np.arange(mpc_solver.prediction_horizon) * mpc_solver.time_step
    ax3.plot(control_time_steps, acceleration_sequence, 'g-s', 
             label='Acceleration', markersize=6, linewidth=2)
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Acceleration [m/s²]')
    ax3.legend()
    ax3.grid(True)
    ax3.set_title('Acceleration Control')
    
    # Plot 4: Steering angle
    ax4 = axes[1, 1]
    ax4.plot(control_time_steps, [math.degrees(s) for s in steering_sequence], 'm-s', 
             label='Steering angle', markersize=6, linewidth=2)
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Steering angle [deg]')
    ax4.legend()
    ax4.grid(True)
    ax4.set_title('Steering Control')
    
    plt.tight_layout()
    plt.show()


def main():
    """Main example function."""
    print("MPC Solver Simple Example")
    print("=" * 40)
    
    # 1. Create MPC solver
    print("1. Creating MPC solver...")
    mpc_solver = MPCSolver()
    print(f"   Prediction horizon: {mpc_solver.prediction_horizon}")
    print(f"   Time step: {mpc_solver.time_step}")
    print(f"   Max velocity: {mpc_solver.max_velocity:.2f} m/s")
    
    # 2. Create reference trajectory
    print("\n2. Creating reference trajectory...")
    reference_trajectory, reference_steering = create_simple_reference_trajectory(
        mpc_solver.prediction_horizon, mpc_solver.time_step
    )
    print(f"   Reference trajectory shape: {reference_trajectory.shape}")
    print(f"   Reference points: {mpc_solver.prediction_horizon + 1}")
    
    # 3. Set initial state
    print("\n3. Setting initial state...")
    initial_state = [0.0, 0.0, 0.0, 0.0]  # [x, y, velocity, yaw]
    print(f"   Initial state: {initial_state}")
    
    # 4. Create linearization trajectory
    print("\n4. Creating linearization trajectory...")
    linearization_trajectory = create_linearization_trajectory(initial_state, reference_trajectory)
    print(f"   Linearization trajectory shape: {linearization_trajectory.shape}")
    
    # 5. Solve MPC with kinematic comparison
    print("\n5. Solving MPC optimization with kinematic comparison...")
    result = mpc_solver.solve_with_kinematic_comparison(
        reference_trajectory,
        linearization_trajectory,
        initial_state,
        reference_steering
    )
    
    acceleration_sequence, steering_sequence, predicted_x, predicted_y, predicted_yaw, predicted_velocity, kinematic_prediction = result
    
    # 6. Display results
    if acceleration_sequence is not None:
        print("   ✓ MPC solved successfully!")
        print(f"   Acceleration sequence: {acceleration_sequence}")
        steering_deg = [f"{math.degrees(s):.2f}" for s in steering_sequence]
        print(f"   Steering sequence: {steering_deg}°")
        print(f"   Final predicted position: ({predicted_x[-1]:.2f}, {predicted_y[-1]:.2f})")
        print(f"   Final predicted velocity: {predicted_velocity[-1]:.2f} m/s")
        print(f"   Final predicted yaw: {math.degrees(predicted_yaw[-1]):.2f}°")
        
        # Get solver info
        solver_info = mpc_solver.get_solver_info()
        print(f"   Solve time: {solver_info['solve_time']:.4f} seconds")
        print(f"   Solve status: {solver_info['status']}")
        
        # Compare with kinematic model if available
        if kinematic_prediction is not None:
            print("\n   === Kinematic Model Comparison ===")
            print(f"   Kinematic final position: ({kinematic_prediction['x'][-1]:.2f}, {kinematic_prediction['y'][-1]:.2f})")
            print(f"   Kinematic final velocity: {kinematic_prediction['velocity'][-1]:.2f} m/s")
            print(f"   Kinematic final yaw: {math.degrees(kinematic_prediction['yaw'][-1]):.2f}°")
            
            # Calculate differences
            position_diff = np.sqrt((predicted_x[-1] - kinematic_prediction['x'][-1])**2 + (predicted_y[-1] - kinematic_prediction['y'][-1])**2)
            velocity_diff = abs(predicted_velocity[-1] - kinematic_prediction['velocity'][-1])
            yaw_diff = abs(predicted_yaw[-1] - kinematic_prediction['yaw'][-1])
            
            print(f"   Position difference: {position_diff:.4f} m")
            print(f"   Velocity difference: {velocity_diff:.4f} m/s")
            print(f"   Yaw difference: {math.degrees(yaw_diff):.4f}°")
        
        # 7. Visualize results
        print("\n6. Visualizing results...")
        visualize_mpc_results(reference_trajectory, reference_steering, initial_state, result, mpc_solver)
        
        # 8. Show kinematic comparison if available
        if kinematic_prediction is not None:
            print("\n7. Showing kinematic model comparison...")
            linearized_prediction = {
                'x': predicted_x,
                'y': predicted_y,
                'velocity': predicted_velocity,
                'yaw': predicted_yaw
            }
            mpc_solver.compare_predictions(
                linearized_prediction,
                kinematic_prediction,
                "Linearized vs Kinematic Model Comparison",
                None,  # acceleration_sequence
                None,  # steering_sequence
                reference_trajectory
            )
        
    else:
        print("   ✗ MPC failed to solve!")
        return
    
    print("\n" + "=" * 40)
    print("Example completed successfully!")


def test_different_initial_states():
    """Test MPC with different initial states."""
    print("\n" + "=" * 40)
    print("Testing with different initial states...")
    
    # Create MPC solver
    mpc_solver = MPCSolver()
    reference_trajectory, reference_steering = create_simple_reference_trajectory(
        mpc_solver.prediction_horizon, mpc_solver.time_step
    )
    
    # Test different initial states
    test_states = [
        [0.0, 0.0, 0.0, 0.0],      # Start at origin
        [1.0, 0.5, 1.0, 0.1],      # Offset position
        [0.0, 0.0, 3.0, 0.0],      # Moving forward
        [0.0, 0.0, 0.0, math.pi/4], # Rotated
    ]
    
    for i, initial_state in enumerate(test_states):
        print(f"\nTest {i+1}: Initial state {initial_state}")
        
        linearization_trajectory = create_linearization_trajectory(initial_state, reference_trajectory)
        
        result = mpc_solver.solve(
            reference_trajectory,
            linearization_trajectory,
            initial_state,
            reference_steering
        )
        
        if result[0] is not None:
            print("   ✓ Solved successfully")
            solver_info = mpc_solver.get_solver_info()
            print(f"   Solve time: {solver_info['solve_time']:.4f} seconds")
        else:
            print("   ✗ Failed to solve")


if __name__ == "__main__":
    # Run main example
    main()
    
    # Test different initial states
    test_different_initial_states()
    
    print("\nAll examples completed!")
