#!/usr/bin/env python3
"""
Example demonstrating the comparison between linearized and kinematic model predictions.

This script shows how to:
1. Solve MPC using linearized model
2. Predict using true kinematic model
3. Compare the differences between both predictions
4. Visualize the results

Usage:
    python kinematic_comparison_example.py                    # Auto-detect mode
    python kinematic_comparison_example.py --interactive     # Force interactive mode
    python kinematic_comparison_example.py --non-interactive # Force non-interactive mode

Interactive mode: Shows menu to select demonstration
Non-interactive mode: Runs default demonstration (Basic Kinematic Comparison) and exits
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
    """Create a test reference trajectory for comparison."""
    reference_trajectory = np.zeros((4, prediction_horizon + 1))
    reference_steering = np.zeros((1, prediction_horizon + 1))
    
    # Create a curved path that will show differences between models
    for i in range(prediction_horizon + 1):
        t = i * time_step
        reference_trajectory[0, i] = t * 3.0  # x position: move forward
        reference_trajectory[1, i] = 0.2 * t**2  # y position: curved path
        reference_trajectory[2, i] = 2.0 + 0.5 * t  # velocity: increasing
        reference_trajectory[3, i] = math.atan2(0.4 * t, 3.0)  # yaw: follow curve
        reference_steering[0, i] = 0.0  # no steering reference
    
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


def visualize_control_sequences(acceleration_sequence, steering_sequence, time_step, test_case_name):
    """Visualize acceleration and steering sequences from MPC solution."""
    if acceleration_sequence is None or steering_sequence is None:
        print("   No control sequences to visualize")
        return
    
    # Create time vector
    time_steps = np.arange(len(acceleration_sequence)) * time_step
    
    # Create figure with subplots
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    
    # Plot acceleration sequence
    ax1.plot(time_steps, acceleration_sequence, 'b-o', linewidth=2, markersize=6, alpha=0.8, label='Acceleration')
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('Acceleration [m/s²]')
    ax1.set_title(f'Control Sequences - {test_case_name}')
    ax1.grid(True, alpha=0.3)
    ax1.axhline(y=0, color='k', linestyle='--', alpha=0.5)
    ax1.legend()
    
    # Add acceleration statistics
    max_accel = np.max(acceleration_sequence)
    min_accel = np.min(acceleration_sequence)
    mean_accel = np.mean(acceleration_sequence)
    ax1.text(0.02, 0.98, f'Max: {max_accel:.3f} m/s²\nMin: {min_accel:.3f} m/s²\nMean: {mean_accel:.3f} m/s²', 
             transform=ax1.transAxes, verticalalignment='top', 
             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    # Plot steering sequence
    ax2.plot(time_steps, steering_sequence, 'r-s', linewidth=2, markersize=6, alpha=0.8, label='Steering')
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Steering Angle [rad]')
    ax2.grid(True, alpha=0.3)
    ax2.axhline(y=0, color='k', linestyle='--', alpha=0.5)
    ax2.legend()
    
    # Add steering statistics
    max_steer = np.max(steering_sequence)
    min_steer = np.min(steering_sequence)
    mean_steer = np.mean(steering_sequence)
    ax2.text(0.02, 0.98, f'Max: {max_steer:.3f} rad ({math.degrees(max_steer):.1f}°)\nMin: {min_steer:.3f} rad ({math.degrees(min_steer):.1f}°)\nMean: {mean_steer:.3f} rad ({math.degrees(mean_steer):.1f}°)', 
             transform=ax2.transAxes, verticalalignment='top',
             bbox=dict(boxstyle='round', facecolor='lightcoral', alpha=0.8))
    
    # Add overall title
    fig.suptitle(f'MPC Control Sequences - {test_case_name}', fontsize=14, fontweight='bold')
    
    plt.tight_layout()
    plt.subplots_adjust(top=0.93)  # Make room for suptitle
    plt.show()
    
    # Print control sequence summary
    print(f"   Control Sequence Summary:")
    print(f"     Acceleration: [{min_accel:.3f}, {max_accel:.3f}] m/s² (range: {max_accel - min_accel:.3f})")
    print(f"     Steering: [{min_steer:.3f}, {max_steer:.3f}] rad (range: {max_steer - min_steer:.3f})")
    print(f"     Steering: [{math.degrees(min_steer):.1f}°, {math.degrees(max_steer):.1f}°] (range: {math.degrees(max_steer - min_steer):.1f}°)")


def demonstrate_kinematic_comparison():
    """Demonstrate comparison between linearized and kinematic models."""
    print("=== Kinematic Model Comparison Demo ===")
    
    # 1. Create MPC solver
    print("1. Creating MPC solver...")
    mpc_solver = MPCSolver(
        prediction_horizon=8,  # Longer horizon to see more differences
        time_step=0.1,         # Smaller time step for better accuracy
        max_velocity=8.0,      # Moderate max velocity
        max_acceleration=1.0   # Moderate acceleration
    )
    print(f"   Prediction horizon: {mpc_solver.prediction_horizon}")
    print(f"   Time step: {mpc_solver.time_step}")
    
    # 2. Create test scenario
    print("\n2. Creating test scenario...")
    reference_trajectory, reference_steering = create_test_reference_trajectory(
        mpc_solver.prediction_horizon, mpc_solver.time_step
    )
    
    # Test different initial states to see varying differences
    test_cases = [
        {
            'name': 'Stationary Start',
            'initial_state': [0.0, 0.0, 0.0, 0.0],
            'description': 'Vehicle starts from rest'
        },
        {
            'name': 'Moving Start',
            'initial_state': [0.0, 0.0, 2.0, 0.0],
            'description': 'Vehicle starts with initial velocity'
        },
        {
            'name': 'Offset Start',
            'initial_state': [1.0, 0.5, 1.0, 0.2],
            'description': 'Vehicle starts with position and angle offset'
        },
        {
            'name': 'High Speed Start',
            'initial_state': [0.0, 0.0, 5.0, 0.0],
            'description': 'Vehicle starts with high velocity'
        }
    ]
    
    # Run comparison for each test case
    for i, test_case in enumerate(test_cases):
        print(f"\n--- Test Case {i+1}: {test_case['name']} ---")
        print(f"Description: {test_case['description']}")
        print(f"Initial state: {test_case['initial_state']}")
        
        # Create linearization trajectory
        linearization_trajectory = create_linearization_trajectory(
            test_case['initial_state'], reference_trajectory
        )
        
        # Solve MPC with kinematic comparison
        result = mpc_solver.solve_with_kinematic_comparison(
            reference_trajectory,
            linearization_trajectory,
            test_case['initial_state'],
            reference_steering
        )
        
        if result is None:
            print("   ✗ Failed to solve")
            continue
            
        acceleration_sequence, steering_sequence, predicted_x, predicted_y, predicted_yaw, predicted_velocity, kinematic_prediction = result
        
        if acceleration_sequence is not None and kinematic_prediction is not None:
            print("   ✓ Both models solved successfully!")
            
            # Prepare data for comparison
            linearized_prediction = {
                'x': predicted_x,
                'y': predicted_y,
                'velocity': predicted_velocity,
                'yaw': predicted_yaw
            }
            
            # Compare predictions
            comparison_stats = mpc_solver.compare_predictions(
                linearized_prediction,
                kinematic_prediction,
                f"Model Comparison - {test_case['name']}",
                acceleration_sequence,
                steering_sequence
            )
            
            # Print additional analysis
            if predicted_x is not None and predicted_y is not None and predicted_velocity is not None and kinematic_prediction is not None:
                print(f"   Final position difference: {np.sqrt((predicted_x[-1] - kinematic_prediction['x'][-1])**2 + (predicted_y[-1] - kinematic_prediction['y'][-1])**2):.4f} m")
                print(f"   Final velocity difference: {abs(predicted_velocity[-1] - kinematic_prediction['velocity'][-1]):.4f} m/s")
            
        else:
            print("   ✗ Failed to solve or compare models")
    
    return mpc_solver


def analyze_model_differences():
    """Analyze the differences between linearized and kinematic models."""
    print("\n=== Model Difference Analysis ===")
    
    # Create solver with different configurations
    configs = [
        {
            'name': 'Short Horizon',
            'solver': MPCSolver(prediction_horizon=3, time_step=0.2),
            'description': 'Short prediction horizon'
        },
        {
            'name': 'Long Horizon',
            'solver': MPCSolver(prediction_horizon=10, time_step=0.1),
            'description': 'Long prediction horizon with small time step'
        },
        {
            'name': 'High Speed',
            'solver': MPCSolver(prediction_horizon=5, time_step=0.2, max_velocity=15.0),
            'description': 'High speed scenario'
        },
        {
            'name': 'Low Speed',
            'solver': MPCSolver(prediction_horizon=5, time_step=0.2, max_velocity=3.0),
            'description': 'Low speed scenario'
        }
    ]
    
    # Test each configuration
    for config in configs:
        print(f"\n--- Configuration: {config['name']} ---")
        print(f"Description: {config['description']}")
        
        solver = config['solver']
        reference_trajectory, reference_steering = create_test_reference_trajectory(
            solver.prediction_horizon, solver.time_step
        )
        
        initial_state = [0.0, 0.0, 1.0, 0.0]  # Moderate initial conditions
        linearization_trajectory = create_linearization_trajectory(initial_state, reference_trajectory)
        
        result = solver.solve_with_kinematic_comparison(
            reference_trajectory,
            linearization_trajectory,
            initial_state,
            reference_steering
        )
        
        if result is None:
            print("   ✗ Failed to solve")
            continue
            
        acceleration_sequence, steering_sequence, predicted_x, predicted_y, predicted_yaw, predicted_velocity, kinematic_prediction = result
        
        if acceleration_sequence is not None and kinematic_prediction is not None:
            # Calculate differences
            position_errors = np.sqrt((predicted_x - kinematic_prediction['x'])**2 + (predicted_y - kinematic_prediction['y'])**2)
            velocity_errors = np.abs(predicted_velocity - kinematic_prediction['velocity'])
            yaw_errors = np.abs(predicted_yaw - kinematic_prediction['yaw'])
            
            print(f"   Max position error: {np.max(position_errors):.4f} m")
            print(f"   Mean position error: {np.mean(position_errors):.4f} m")
            print(f"   Max velocity error: {np.max(velocity_errors):.4f} m/s")
            print(f"   Max yaw error: {math.degrees(np.max(yaw_errors)):.4f} deg")
            print(f"   Final position error: {position_errors[-1]:.4f} m")
        else:
            print("   ✗ Failed to solve")


def demonstrate_iterative_linearization_effect():
    """Demonstrate how iterative linearization affects the prediction accuracy."""
    print("\n=== Iterative Linearization Effect ===")
    
    solver = MPCSolver(prediction_horizon=6, time_step=0.2)
    reference_trajectory, reference_steering = create_test_reference_trajectory(
        solver.prediction_horizon, solver.time_step
    )
    
    # Test with different initial linearization points
    initial_states = [
        [0.0, 0.0, 0.0, 0.0],      # Start at origin
        [0.0, 0.0, 2.0, 0.0],      # Start with velocity
        [1.0, 0.5, 1.0, 0.3],      # Start with offset
    ]
    
    for i, initial_state in enumerate(initial_states):
        print(f"\n--- Initial State {i+1}: {initial_state} ---")
        
        # Create different linearization trajectories
        linearization_trajectory = create_linearization_trajectory(initial_state, reference_trajectory)
        
        result = solver.solve_with_kinematic_comparison(
            reference_trajectory,
            linearization_trajectory,
            initial_state,
            reference_steering
        )
        
        if result is None:
            print("   ✗ Failed to solve")
            continue
            
        acceleration_sequence, steering_sequence, predicted_x, predicted_y, predicted_yaw, predicted_velocity, kinematic_prediction = result
        
        if acceleration_sequence is not None and kinematic_prediction is not None:
            # Calculate cumulative error
            position_errors = np.sqrt((predicted_x - kinematic_prediction['x'])**2 + (predicted_y - kinematic_prediction['y'])**2)
            cumulative_error = np.sum(position_errors)
            
            print(f"   Cumulative position error: {cumulative_error:.4f} m")
            print(f"   Max position error: {np.max(position_errors):.4f} m")
            print(f"   Final position error: {position_errors[-1]:.4f} m")
            
            # Show how error accumulates over time
            time_steps = np.arange(len(position_errors)) * solver.time_step
            plt.figure(figsize=(10, 6))
            plt.plot(time_steps, position_errors, 'b-o', linewidth=2, markersize=4, alpha=0.3)
            plt.xlabel('Time [s]')
            plt.ylabel('Position Error [m]')
            plt.title(f'Position Error Evolution - Initial State {i+1}')
            plt.grid(True)
            plt.show()
        else:
            print("   ✗ Failed to solve")


def main():
    """Main demonstration function with interactive menu."""
    print("Kinematic Model Comparison Demonstration")
    print("=" * 50)
    
    # Define available demonstrations
    demonstrations = {
        '1': {
            'name': 'Basic Kinematic Comparison',
            'function': demonstrate_kinematic_comparison,
            'description': 'Compare linearized vs kinematic models for different initial states'
        },
        '2': {
            'name': 'Model Difference Analysis',
            'function': analyze_model_differences,
            'description': 'Analyze differences between models with various configurations'
        },
        '3': {
            'name': 'Iterative Linearization Effect',
            'function': demonstrate_iterative_linearization_effect,
            'description': 'Show how iterative linearization affects prediction accuracy'
        },
        '4': {
            'name': 'Run All Demonstrations',
            'function': run_all_demonstrations,
            'description': 'Run all demonstrations in sequence'
        }
    }
    
    # Check if running in interactive mode
    is_interactive = True
    try:
        import sys
        # Check if stdin is a TTY (interactive terminal)
        if not sys.stdin.isatty():
            is_interactive = False
    except:
        is_interactive = False
    
    # Also check for command line arguments to force modes
    import sys
    if len(sys.argv) > 1:
        if sys.argv[1] in ['--non-interactive', '-n']:
            is_interactive = False
        elif sys.argv[1] in ['--interactive', '-i']:
            is_interactive = True
    
    if not is_interactive:
        # Non-interactive mode: run default demonstration once and exit
        print("\nNon-interactive environment detected. Running default demonstration...")
        print(f"\n{'='*60}")
        print(f"Running: {demonstrations['1']['name']}")
        print(f"{'='*60}")
        
        try:
            demonstrations['1']['function']()
            print(f"\n{'='*60}")
            print(f"Completed: {demonstrations['1']['name']}")
            print(f"{'='*60}")
            print("\nScript completed. Use interactive mode to select different demonstrations.")
        except Exception as e:
            print(f"Error running demonstration: {e}")
        return
    
    # Interactive mode: show menu
    while True:
        print("\nAvailable Demonstrations:")
        print("-" * 30)
        for key, demo in demonstrations.items():
            print(f"{key}. {demo['name']}")
            print(f"   {demo['description']}")
        
        print("\n0. Exit")
        
        # Get user choice with default
        try:
            choice = input(f"\nSelect demonstration (1-4, default=1): ").strip()
            if not choice:
                choice = '1'  # Default to first option
        except (EOFError, KeyboardInterrupt):
            print("\nExiting...")
            break
        
        if choice == '0':
            print("Exiting...")
            break
        elif choice in demonstrations:
            print(f"\n{'='*60}")
            print(f"Running: {demonstrations[choice]['name']}")
            print(f"{'='*60}")
            
            try:
                if choice == '4':
                    # Run all demonstrations
                    run_all_demonstrations()
                else:
                    # Run selected demonstration
                    demonstrations[choice]['function']()
                
                print(f"\n{'='*60}")
                print(f"Completed: {demonstrations[choice]['name']}")
                print(f"{'='*60}")
                
            except Exception as e:
                print(f"Error running demonstration: {e}")
                print("Please try again.")
        else:
            print("Invalid choice. Please select 1-4 or 0 to exit.")


def run_all_demonstrations():
    """Run all demonstrations in sequence."""
    print("Running all demonstrations...")
    
    # Basic comparison
    mpc_solver = demonstrate_kinematic_comparison()
    
    # Analyze differences
    analyze_model_differences()
    
    # Show iterative linearization effect
    demonstrate_iterative_linearization_effect()
    
    print("\n" + "=" * 50)
    print("All demonstrations completed!")
    print("\nKey Insights:")
    print("1. Linearized model is accurate for small deviations from linearization point")
    print("2. Differences increase with prediction horizon and time step")
    print("3. Higher speeds and larger steering angles show more differences")
    print("4. Iterative linearization helps reduce prediction errors")
    print("5. Kinematic model provides more accurate long-term predictions")


if __name__ == "__main__":
    main()


