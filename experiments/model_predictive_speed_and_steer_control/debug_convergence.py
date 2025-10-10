#!/usr/bin/env python3
"""
Debug script to understand MPC convergence behavior.

This script helps diagnose why MPC might not converge and shows the
iteration process in detail.
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import pathlib

# Add parent directory to path for imports
sys.path.append(str(pathlib.Path(__file__).parent.parent))

from mpc_core import MPCSolver


def create_simple_test_trajectory(prediction_horizon=5, time_step=0.2):
    """Create a simple test trajectory."""
    reference_trajectory = np.zeros((4, prediction_horizon + 1))
    reference_steering = np.zeros((1, prediction_horizon + 1))
    
    for i in range(prediction_horizon + 1):
        t = i * time_step
        reference_trajectory[0, i] = t * 2.0  # x position
        reference_trajectory[1, i] = 0.0      # y position
        reference_trajectory[2, i] = 2.0      # velocity
        reference_trajectory[3, i] = 0.0      # yaw
        reference_steering[0, i] = 0.0        # steering
    
    return reference_trajectory, reference_steering


def test_linearization_trajectory():
    """Test the linearization trajectory creation."""
    print("=== Testing Linearization Trajectory Creation ===")
    
    # Create test data
    reference_trajectory, _ = create_simple_test_trajectory()
    initial_state = [0.0, 0.0, 0.0, 0.0]
    
    # Create linearization trajectory
    prediction_horizon = reference_trajectory.shape[1] - 1
    linearization_trajectory = np.zeros((4, prediction_horizon + 1))
    initial_state_array = np.array(initial_state)
    
    # Start with initial state
    linearization_trajectory[:, 0] = initial_state_array
    
    # Interpolate towards reference trajectory
    for i in range(1, prediction_horizon + 1):
        alpha = i / prediction_horizon
        linearization_trajectory[:, i] = (1 - alpha) * initial_state_array + alpha * reference_trajectory[:, i]
    
    print("Reference trajectory (first 3 points):")
    for i in range(3):
        print(f"  Point {i}: x={reference_trajectory[0, i]:.2f}, y={reference_trajectory[1, i]:.2f}, "
              f"v={reference_trajectory[2, i]:.2f}, yaw={reference_trajectory[3, i]:.2f}")
    
    print("\nLinearization trajectory (first 3 points):")
    for i in range(3):
        print(f"  Point {i}: x={linearization_trajectory[0, i]:.2f}, y={linearization_trajectory[1, i]:.2f}, "
              f"v={linearization_trajectory[2, i]:.2f}, yaw={linearization_trajectory[3, i]:.2f}")
    
    print(f"\nInitial state: {initial_state}")
    print(f"Linearization trajectory starts with: {linearization_trajectory[:, 0].tolist()}")
    print(f"Linearization trajectory ends with: {linearization_trajectory[:, -1].tolist()}")
    
    return linearization_trajectory


def debug_convergence_with_different_thresholds():
    """Test convergence with different thresholds."""
    print("\n=== Testing Convergence with Different Thresholds ===")
    
    # Create solver
    mpc_solver = MPCSolver(prediction_horizon=5, time_step=0.2)
    
    # Create test trajectory
    reference_trajectory, reference_steering = create_simple_test_trajectory()
    initial_state = [0.0, 0.0, 0.0, 0.0]
    
    # Test different thresholds
    thresholds = [1.0, 0.5, 0.2, 0.1, 0.05, 0.01]
    
    print(f"Initial state: {initial_state}")
    print(f"Reference trajectory: x=[{reference_trajectory[0, 0]:.1f}, {reference_trajectory[0, -1]:.1f}], "
          f"y=[{reference_trajectory[1, 0]:.1f}, {reference_trajectory[1, -1]:.1f}]")
    
    for threshold in thresholds:
        print(f"\n--- Testing threshold: {threshold} ---")
        
        iteration_results = mpc_solver.solve_iterative(
            reference_trajectory,
            initial_state,
            reference_steering,
            max_iterations=8,
            convergence_threshold=threshold
        )
        
        if iteration_results['iterations']:
            print(f"  Iterations: {len(iteration_results['iterations'])}")
            print(f"  Converged: {iteration_results['converged']}")
            print(f"  Total time: {iteration_results['total_solve_time']:.4f}s")
            
            # Show control changes
            for i, iteration in enumerate(iteration_results['iterations']):
                print(f"    Iter {i}: control_change={iteration['control_change']:.6f}")
        else:
            print("  Failed to solve!")


def debug_challenging_scenario():
    """Test a more challenging scenario."""
    print("\n=== Testing Challenging Scenario ===")
    
    # Create solver
    mpc_solver = MPCSolver(prediction_horizon=8, time_step=0.15)
    
    # Create challenging trajectory (S-curve)
    reference_trajectory = np.zeros((4, 9))
    reference_steering = np.zeros((1, 9))
    
    for i in range(9):
        t = i * 0.15
        reference_trajectory[0, i] = t * 3.0  # x position
        reference_trajectory[1, i] = 2.0 * np.sin(0.5 * t)  # y position (S-curve)
        reference_trajectory[2, i] = 3.0  # velocity
        reference_trajectory[3, i] = np.arctan2(2.0 * 0.5 * np.cos(0.5 * t), 3.0)  # yaw
        reference_steering[0, i] = 0.0
    
    # Test with different initial states
    initial_states = [
        [0.0, 0.0, 0.0, 0.0],      # Stationary at origin
        [1.0, 1.0, 0.0, 0.5],      # Offset position and angle
        [0.0, 0.0, 5.0, 0.0],      # High initial velocity
    ]
    
    for i, initial_state in enumerate(initial_states):
        print(f"\n--- Challenging Test {i+1}: {initial_state} ---")
        
        iteration_results = mpc_solver.solve_iterative(
            reference_trajectory,
            initial_state,
            reference_steering,
            max_iterations=10,
            convergence_threshold=0.05
        )
        
        if iteration_results['iterations']:
            print(f"  ✓ Solved with {len(iteration_results['iterations'])} iterations")
            print(f"  Converged: {iteration_results['converged']}")
            
            final_iter = iteration_results['iterations'][-1]
            if final_iter['predicted_x'] is not None:
                final_pos = (final_iter['predicted_x'][-1], final_iter['predicted_y'][-1])
                ref_pos = (reference_trajectory[0, -1], reference_trajectory[1, -1])
                error = np.sqrt((final_pos[0] - ref_pos[0])**2 + (final_pos[1] - ref_pos[1])**2)
                print(f"  Final position: ({final_pos[0]:.2f}, {final_pos[1]:.2f})")
                print(f"  Reference position: ({ref_pos[0]:.2f}, {ref_pos[1]:.2f})")
                print(f"  Position error: {error:.3f} m")
            
            # Show iteration plot
            mpc_solver.plot_iteration_comparison(
                iteration_results,
                reference_trajectory,
                f"Challenging Scenario {i+1} - Convergence Analysis"
            )
        else:
            print("  ✗ Failed to solve!")


def main():
    """Main debug function."""
    print("MPC Convergence Debug Tool")
    print("=" * 50)
    
    # Test linearization trajectory
    test_linearization_trajectory()
    
    # Test different convergence thresholds
    debug_convergence_with_different_thresholds()
    
    # Test challenging scenarios
    debug_challenging_scenario()
    
    print("\n" + "=" * 50)
    print("Debug analysis completed!")
    print("\nKey Insights:")
    print("1. Linearization trajectory should interpolate from initial to reference")
    print("2. Convergence threshold affects number of iterations needed")
    print("3. Challenging scenarios may need more iterations or looser thresholds")
    print("4. Control change should decrease monotonically for convergence")


if __name__ == "__main__":
    main()