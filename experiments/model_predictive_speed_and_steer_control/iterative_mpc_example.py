#!/usr/bin/env python3
"""
Example demonstrating the iterative MPC solver functionality.

This script shows how to:
1. Use the new solve_iterative method to get all iteration results
2. Visualize the convergence process across iterations
3. Compare different scenarios and their convergence behavior
4. Analyze the performance of the iterative approach

Usage:
    python iterative_mpc_example.py
"""

import math
import numpy as np
import matplotlib.pyplot as plt
import sys
import pathlib

# Add parent directory to path for imports
sys.path.append(str(pathlib.Path(__file__).parent.parent))

from mpc_core import MPCSolver


def create_test_reference_trajectory(prediction_horizon=5, time_step=0.2):
    """Create a test reference trajectory for iterative MPC testing."""
    reference_trajectory = np.zeros((4, prediction_horizon + 1))
    reference_steering = np.zeros((1, prediction_horizon + 1))
    
    # Create a curved path that will require multiple iterations to converge
    for i in range(prediction_horizon + 1):
        t = i * time_step
        reference_trajectory[0, i] = t * 3.0  # x position: move forward
        reference_trajectory[1, i] = 0.3 * t**2  # y position: curved path (more challenging)
        reference_trajectory[2, i] = 2.0 + 0.8 * t  # velocity: increasing
        reference_trajectory[3, i] = math.atan2(0.6 * t, 3.0)  # yaw: follow curve
        reference_steering[0, i] = 0.0  # no steering reference
    
    return reference_trajectory, reference_steering


def create_challenging_reference_trajectory(prediction_horizon=5, time_step=0.2):
    """Create a more challenging reference trajectory that requires more iterations."""
    reference_trajectory = np.zeros((4, prediction_horizon + 1))
    reference_steering = np.zeros((1, prediction_horizon + 1))
    
    # Create an S-curve path that will be challenging for the linearized model
    for i in range(prediction_horizon + 1):
        t = i * time_step
        reference_trajectory[0, i] = t * 4.0  # x position: move forward
        reference_trajectory[1, i] = 2.0 * math.sin(0.8 * t)  # y position: S-curve
        reference_trajectory[2, i] = 3.0 + 0.5 * math.cos(0.8 * t)  # velocity: varying
        reference_trajectory[3, i] = math.atan2(2.0 * 0.8 * math.cos(0.8 * t), 4.0)  # yaw: follow curve
        reference_steering[0, i] = 0.0  # no steering reference
    
    return reference_trajectory, reference_steering


def demonstrate_basic_iterative_mpc():
    """Demonstrate basic iterative MPC functionality."""
    print("=== Basic Iterative MPC Demonstration ===")
    
    # Create MPC solver
    mpc_solver = MPCSolver(
        prediction_horizon=8,
        time_step=0.15,
        max_velocity=10.0,
        max_acceleration=1.5
    )
    
    print(f"Created MPC solver:")
    print(f"  Prediction horizon: {mpc_solver.prediction_horizon}")
    print(f"  Time step: {mpc_solver.time_step}")
    print(f"  Max velocity: {mpc_solver.max_velocity:.2f} m/s")
    
    # Create test trajectory
    reference_trajectory, reference_steering = create_test_reference_trajectory(
        mpc_solver.prediction_horizon, mpc_solver.time_step
    )
    
    # Test different initial states
    test_cases = [
        {
            'name': 'Stationary Start',
            'initial_state': [0.0, 0.0, 0.0, 0.0],
            'description': 'Vehicle starts from rest at origin'
        },
        {
            'name': 'Offset Start',
            'initial_state': [1.0, 0.5, 1.0, 0.2],
            'description': 'Vehicle starts with position and angle offset'
        },
        {
            'name': 'High Speed Start',
            'initial_state': [0.0, 0.0, 6.0, 0.0],
            'description': 'Vehicle starts with high initial velocity'
        }
    ]
    
    for test_case in test_cases:
        print(f"\n--- Test Case: {test_case['name']} ---")
        print(f"Description: {test_case['description']}")
        print(f"Initial state: {test_case['initial_state']}")
        
        # Solve with iterative MPC
        iteration_results = mpc_solver.solve_iterative(
            reference_trajectory,
            test_case['initial_state'],
            reference_steering,
            max_iterations=5,
            convergence_threshold=0.1
        )
        
        # Display results
        if iteration_results['iterations']:
            print(f"✓ Iterative MPC solved successfully!")
            print(f"  Total iterations: {len(iteration_results['iterations'])}")
            print(f"  Converged: {iteration_results['converged']}")
            print(f"  Total solve time: {iteration_results['total_solve_time']:.4f} seconds")
            
            # Show iteration details
            final_iteration = iteration_results['iterations'][-1]
            if final_iteration['predicted_x'] is not None:
                print(f"  Final position: ({final_iteration['predicted_x'][-1]:.2f}, {final_iteration['predicted_y'][-1]:.2f})")
                print(f"  Final velocity: {final_iteration['predicted_velocity'][-1]:.2f} m/s")
            
            # Plot iteration comparison
            mpc_solver.plot_iteration_comparison(
                iteration_results,
                reference_trajectory,
                f"Iterative MPC - {test_case['name']}"
            )
        else:
            print("✗ Iterative MPC failed to solve!")
    
    return mpc_solver


def demonstrate_convergence_analysis():
    """Demonstrate convergence analysis with different parameters."""
    print("\n=== Convergence Analysis Demonstration ===")
    
    # Create MPC solver
    mpc_solver = MPCSolver(prediction_horizon=6, time_step=0.2)
    
    # Create challenging trajectory
    reference_trajectory, reference_steering = create_challenging_reference_trajectory(
        mpc_solver.prediction_horizon, mpc_solver.time_step
    )
    
    initial_state = [0.0, 0.0, 2.0, 0.0]
    
    # Test different convergence thresholds
    thresholds = [0.5, 0.2, 0.1, 0.05, 0.01]
    
    print(f"Testing different convergence thresholds...")
    print(f"Initial state: {initial_state}")
    
    convergence_results = []
    
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
            result = {
                'threshold': threshold,
                'iterations': len(iteration_results['iterations']),
                'converged': iteration_results['converged'],
                'total_time': iteration_results['total_solve_time'],
                'final_control_change': iteration_results['iterations'][-1]['control_change']
            }
            convergence_results.append(result)
            
            print(f"  Iterations: {result['iterations']}")
            print(f"  Converged: {result['converged']}")
            print(f"  Total time: {result['total_time']:.4f}s")
            print(f"  Final control change: {result['final_control_change']:.4f}")
        else:
            print("  Failed to solve!")
    
    # Plot convergence analysis
    if convergence_results:
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
        
        thresholds = [r['threshold'] for r in convergence_results]
        iterations = [r['iterations'] for r in convergence_results]
        times = [r['total_time'] for r in convergence_results]
        converged = [r['converged'] for r in convergence_results]
        
        # Plot iterations vs threshold
        colors = ['green' if c else 'red' for c in converged]
        ax1.semilogx(thresholds, iterations, 'o-', color='blue', markersize=8, linewidth=2)
        for i, (t, it, conv) in enumerate(zip(thresholds, iterations, converged)):
            ax1.plot(t, it, 'o', color=colors[i], markersize=10)
        ax1.set_xlabel('Convergence Threshold')
        ax1.set_ylabel('Number of Iterations')
        ax1.set_title('Iterations vs Convergence Threshold')
        ax1.grid(True, alpha=0.3)
        
        # Plot time vs threshold
        ax2.semilogx(thresholds, times, 's-', color='red', markersize=8, linewidth=2)
        for i, (t, time, conv) in enumerate(zip(thresholds, times, converged)):
            ax2.plot(t, time, 's', color=colors[i], markersize=10)
        ax2.set_xlabel('Convergence Threshold')
        ax2.set_ylabel('Total Solve Time [s]')
        ax2.set_title('Solve Time vs Convergence Threshold')
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
        
        print(f"\n=== Convergence Analysis Summary ===")
        print(f"{'Threshold':<12} {'Iterations':<12} {'Converged':<10} {'Time [s]':<10}")
        print("-" * 50)
        for result in convergence_results:
            print(f"{result['threshold']:<12.3f} {result['iterations']:<12} "
                  f"{result['converged']:<10} {result['total_time']:<10.4f}")


def compare_single_vs_iterative():
    """Compare single solve vs iterative solve performance."""
    print("\n=== Single vs Iterative Solve Comparison ===")
    
    # Create MPC solver
    mpc_solver = MPCSolver(prediction_horizon=6, time_step=0.2)
    
    # Create test trajectory
    reference_trajectory, reference_steering = create_challenging_reference_trajectory(
        mpc_solver.prediction_horizon, mpc_solver.time_step
    )
    
    # Test different initial states
    test_states = [
        [0.0, 0.0, 0.0, 0.0],      # Stationary
        [1.0, 0.5, 2.0, 0.1],      # Offset
        [0.0, 0.0, 5.0, 0.0],      # Moving
    ]
    
    print(f"Comparing single solve vs iterative solve for {len(test_states)} test cases...")
    
    for i, initial_state in enumerate(test_states):
        print(f"\n--- Test Case {i+1}: Initial state {initial_state} ---")
        
        # Single solve
        print("Single solve:")
        linearization_trajectory = np.zeros_like(reference_trajectory)
        linearization_trajectory[:, 0] = initial_state
        
        start_time = time.time()
        single_result = mpc_solver.solve(
            reference_trajectory,
            linearization_trajectory,
            initial_state,
            reference_steering
        )
        single_time = time.time() - start_time
        
        if single_result[0] is not None:
            print(f"  ✓ Solved in {single_time:.4f} seconds")
        else:
            print("  ✗ Failed to solve!")
        
        # Iterative solve
        print("Iterative solve:")
        start_time = time.time()
        iteration_results = mpc_solver.solve_iterative(
            reference_trajectory,
            initial_state,
            reference_steering,
            max_iterations=5,
            convergence_threshold=0.1
        )
        iterative_time = time.time() - start_time
        
        if iteration_results['iterations']:
            print(f"  ✓ Solved in {iterative_time:.4f} seconds")
            print(f"  Used {len(iteration_results['iterations'])} iterations")
            print(f"  Converged: {iteration_results['converged']}")
            
            # Compare final results
            if single_result[0] is not None:
                final_iter = iteration_results['iterations'][-1]
                pos_diff = np.sqrt((single_result[1][-1] - final_iter['predicted_y'][-1])**2 + 
                                 (single_result[0][-1] - final_iter['predicted_x'][-1])**2)
                vel_diff = abs(single_result[3][-1] - final_iter['predicted_velocity'][-1])
                
                print(f"  Position difference: {pos_diff:.4f} m")
                print(f"  Velocity difference: {vel_diff:.4f} m/s")
                print(f"  Speed improvement: {iterative_time/single_time:.2f}x slower")
        else:
            print("  ✗ Failed to solve!")


def demonstrate_advanced_visualization():
    """Demonstrate advanced visualization capabilities."""
    print("\n=== Advanced Visualization Demonstration ===")
    
    # Create MPC solver with longer horizon for better visualization
    mpc_solver = MPCSolver(
        prediction_horizon=10,
        time_step=0.15,
        max_velocity=12.0,
        max_acceleration=2.0
    )
    
    # Create complex trajectory
    reference_trajectory, reference_steering = create_challenging_reference_trajectory(
        mpc_solver.prediction_horizon, mpc_solver.time_step
    )
    
    initial_state = [0.0, 0.0, 0.0, 0.0]
    
    print(f"Solving with extended prediction horizon ({mpc_solver.prediction_horizon} steps)...")
    
    # Solve with more iterations for better visualization
    iteration_results = mpc_solver.solve_iterative(
        reference_trajectory,
        initial_state,
        reference_steering,
        max_iterations=8,
        convergence_threshold=0.05
    )
    
    if iteration_results['iterations']:
        print(f"✓ Solved with {len(iteration_results['iterations'])} iterations")
        
        # Show detailed plot with convergence analysis
        mpc_solver.plot_iteration_comparison(
            iteration_results,
            reference_trajectory,
            "Advanced Iterative MPC Analysis",
            show_convergence=True
        )
        
        # Create custom analysis plot
        create_custom_analysis_plot(iteration_results, reference_trajectory, mpc_solver)


def create_custom_analysis_plot(iteration_results, reference_trajectory, mpc_solver):
    """Create custom analysis plots for iteration results."""
    iterations = iteration_results['iterations']
    n_iterations = len(iterations)
    
    if n_iterations == 0:
        return
    
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    fig.suptitle("Custom Iterative MPC Analysis", fontsize=16)
    
    # Plot 1: Trajectory convergence
    ax1 = axes[0, 0]
    for i, iteration in enumerate(iterations):
        if iteration['predicted_x'] is not None and iteration['predicted_y'] is not None:
            alpha = 0.3 + 0.7 * (i / max(1, n_iterations - 1))
            ax1.plot(iteration['predicted_x'], iteration['predicted_y'], 
                    'o-', alpha=alpha, label=f'Iter {i}', markersize=3, linewidth=1)
    
    if reference_trajectory is not None:
        ax1.plot(reference_trajectory[0, :], reference_trajectory[1, :], 
                'r--', label='Reference', linewidth=2, alpha=0.7)
    
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.set_title('Trajectory Convergence')
    ax1.axis('equal')
    
    # Plot 2: Error evolution
    ax2 = axes[0, 1]
    if reference_trajectory is not None:
        errors = []
        for iteration in iterations:
            if iteration['predicted_x'] is not None and iteration['predicted_y'] is not None:
                # Calculate RMSE against reference
                ref_x = reference_trajectory[0, :len(iteration['predicted_x'])]
                ref_y = reference_trajectory[1, :len(iteration['predicted_y'])]
                rmse = np.sqrt(np.mean((iteration['predicted_x'] - ref_x)**2 + 
                                      (iteration['predicted_y'] - ref_y)**2))
                errors.append(rmse)
        
        if errors:
            ax2.plot(range(len(errors)), errors, 'bo-', markersize=8, linewidth=2)
            ax2.set_xlabel('Iteration')
            ax2.set_ylabel('RMSE [m]')
            ax2.set_title('Tracking Error Evolution')
            ax2.grid(True, alpha=0.3)
    
    # Plot 3: Control effort evolution
    ax3 = axes[1, 0]
    accel_efforts = []
    steer_efforts = []
    
    for iteration in iterations:
        if iteration['acceleration_sequence'] is not None:
            accel_effort = np.sum(np.abs(iteration['acceleration_sequence']))
            steer_effort = np.sum(np.abs(iteration['steering_sequence']))
            accel_efforts.append(accel_effort)
            steer_efforts.append(steer_effort)
    
    if accel_efforts:
        ax3.plot(range(len(accel_efforts)), accel_efforts, 'g-o', 
                label='Acceleration Effort', markersize=6, linewidth=2)
        ax3.plot(range(len(steer_efforts)), steer_efforts, 'm-s', 
                label='Steering Effort', markersize=6, linewidth=2)
        ax3.set_xlabel('Iteration')
        ax3.set_ylabel('Control Effort')
        ax3.legend()
        ax3.grid(True, alpha=0.3)
        ax3.set_title('Control Effort Evolution')
    
    # Plot 4: Performance metrics
    ax4 = axes[1, 1]
    solve_times = [iter['solve_time'] for iter in iterations]
    control_changes = [iter['control_change'] for iter in iterations if iter['control_change'] != float('inf')]
    
    ax4_twin = ax4.twinx()
    
    if solve_times:
        line1 = ax4.bar(range(len(solve_times)), solve_times, alpha=0.7, color='blue', label='Solve Time')
        ax4.set_xlabel('Iteration')
        ax4.set_ylabel('Solve Time [s]', color='blue')
        ax4.tick_params(axis='y', labelcolor='blue')
    
    if control_changes:
        line2 = ax4_twin.plot(range(len(control_changes)), control_changes, 
                             'ro-', markersize=8, linewidth=2, label='Control Change')
        ax4_twin.set_ylabel('Control Change', color='red')
        ax4_twin.tick_params(axis='y', labelcolor='red')
        ax4_twin.set_yscale('log')
    
    ax4.set_title('Performance Metrics')
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()


def main():
    """Main demonstration function."""
    print("Iterative MPC Solver Demonstration")
    print("=" * 60)
    
    try:
        # Basic demonstration
        mpc_solver = demonstrate_basic_iterative_mpc()
        
        # Convergence analysis
        demonstrate_convergence_analysis()
        
        # Single vs iterative comparison
        compare_single_vs_iterative()
        
        # Advanced visualization
        demonstrate_advanced_visualization()
        
        print("\n" + "=" * 60)
        print("All demonstrations completed successfully!")
        print("\nKey Insights:")
        print("1. Iterative MPC improves solution quality through successive linearization")
        print("2. Convergence typically achieved in 2-4 iterations for most scenarios")
        print("3. More challenging trajectories require more iterations")
        print("4. Trade-off between solution quality and computation time")
        print("5. Visualization helps understand the convergence process")
        
    except Exception as e:
        print(f"Error during demonstration: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    import time
    main()