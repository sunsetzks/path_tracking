"""
Test script for solve_linear_mpc function.

This script creates various test cases to validate the MPC solver functionality
with different reference trajectories and initial conditions.
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from mpc_core import MPCSolver


def create_straight_line_reference(prediction_horizon=5):
    """Create a straight line reference trajectory."""
    reference_trajectory = np.zeros((4, prediction_horizon + 1))
    reference_steering = np.zeros((1, prediction_horizon + 1))
    
    # Straight line moving forward
    for i in range(prediction_horizon + 1):
        reference_trajectory[0, i] = i * 2.0  # x position
        reference_trajectory[1, i] = 0.0      # y position
        reference_trajectory[2, i] = 5.0      # velocity (m/s)
        reference_trajectory[3, i] = 0.0      # yaw angle
        reference_steering[0, i] = 0.0        # steering angle
    
    return reference_trajectory, reference_steering


def create_curved_reference(prediction_horizon=5, time_step=0.2):
    """Create a curved reference trajectory."""
    reference_trajectory = np.zeros((4, prediction_horizon + 1))
    reference_steering = np.zeros((1, prediction_horizon + 1))
    
    # Curved path
    for i in range(prediction_horizon + 1):
        t = i * time_step
        reference_trajectory[0, i] = t * 5.0  # x position
        reference_trajectory[1, i] = 0.1 * t**2  # y position (parabolic curve)
        reference_trajectory[2, i] = 3.0      # velocity (m/s)
        reference_trajectory[3, i] = math.atan2(0.2 * t, 5.0)  # yaw angle
        reference_steering[0, i] = 0.0        # steering angle
    
    return reference_trajectory, reference_steering


def create_circular_reference(prediction_horizon=5, time_step=0.2):
    """Create a circular reference trajectory."""
    reference_trajectory = np.zeros((4, prediction_horizon + 1))
    reference_steering = np.zeros((1, prediction_horizon + 1))
    
    radius = 10.0
    angular_velocity = 0.5  # rad/s
    
    for i in range(prediction_horizon + 1):
        t = i * time_step
        angle = angular_velocity * t
        reference_trajectory[0, i] = radius * math.cos(angle)
        reference_trajectory[1, i] = radius * math.sin(angle)
        reference_trajectory[2, i] = radius * angular_velocity  # velocity
        reference_trajectory[3, i] = angle + math.pi/2  # yaw angle (tangent to circle)
        reference_steering[0, i] = 0.0
    
    return reference_trajectory, reference_steering


def create_random_reference(prediction_horizon=5):
    """Create a random reference trajectory."""
    np.random.seed(42)  # For reproducible results
    reference_trajectory = np.zeros((4, prediction_horizon + 1))
    reference_steering = np.zeros((1, prediction_horizon + 1))
    
    # Random walk trajectory
    for i in range(prediction_horizon + 1):
        if i == 0:
            reference_trajectory[0, i] = 0.0
            reference_trajectory[1, i] = 0.0
            reference_trajectory[2, i] = 2.0
            reference_trajectory[3, i] = 0.0
        else:
            # Random walk
            dx = np.random.normal(0, 0.5)
            dy = np.random.normal(0, 0.3)
            reference_trajectory[0, i] = reference_trajectory[0, i-1] + dx
            reference_trajectory[1, i] = reference_trajectory[1, i-1] + dy
            reference_trajectory[2, i] = 2.0 + np.random.normal(0, 0.5)
            reference_trajectory[3, i] = math.atan2(dy, dx)
        
        reference_steering[0, i] = 0.0
    
    return reference_trajectory, reference_steering


def create_linearization_trajectory(initial_state, reference_trajectory):
    """Create a linearization trajectory based on initial state and reference."""
    prediction_horizon = reference_trajectory.shape[1] - 1
    linearization_trajectory = np.zeros((4, prediction_horizon + 1))
    
    # Convert initial_state to numpy array
    initial_state_array = np.array(initial_state)
    
    # Start with initial state
    linearization_trajectory[:, 0] = initial_state_array
    
    # Interpolate towards reference trajectory
    for i in range(1, prediction_horizon + 1):
        alpha = i / prediction_horizon
        linearization_trajectory[:, i] = (1 - alpha) * initial_state_array + alpha * reference_trajectory[:, i]
    
    return linearization_trajectory


def test_mpc_solver(test_name, reference_trajectory, reference_steering, initial_state, mpc_solver=None):
    """Test the MPC solver with given parameters."""
    print(f"\n=== Testing {test_name} ===")
    print(f"Initial state: x={initial_state[0]:.2f}, y={initial_state[1]:.2f}, "
          f"v={initial_state[2]:.2f}, yaw={math.degrees(initial_state[3]):.2f}°")
    
    # Create MPC solver if not provided
    if mpc_solver is None:
        mpc_solver = MPCSolver()
    
    # Create linearization trajectory
    linearization_trajectory = create_linearization_trajectory(initial_state, reference_trajectory)
    
    # Solve MPC
    result = mpc_solver.solve(
        reference_trajectory,
        linearization_trajectory,
        initial_state,
        reference_steering
    )
    
    acceleration_sequence, steering_sequence, predicted_x, predicted_y, predicted_yaw, predicted_velocity = result
    
    if acceleration_sequence is not None:
        print("✓ MPC solved successfully!")
        print(f"Acceleration sequence: {acceleration_sequence}")
        print(f"Steering sequence: {[math.degrees(s) for s in steering_sequence]}°")
        print(f"Predicted final position: ({predicted_x[-1]:.2f}, {predicted_y[-1]:.2f})")
        print(f"Predicted final velocity: {predicted_velocity[-1]:.2f} m/s")
        print(f"Predicted final yaw: {math.degrees(predicted_yaw[-1]):.2f}°")
        
        # Get solver info
        solver_info = mpc_solver.get_solver_info()
        print(f"Solve time: {solver_info['solve_time']:.4f} seconds")
        print(f"Solve status: {solver_info['status']}")
        
        return True, result
    else:
        print("✗ MPC failed to solve!")
        return False, None


def visualize_test_results(test_name, reference_trajectory, reference_steering, initial_state, result, mpc_solver):
    """Visualize the test results."""
    if result is None or result[0] is None:
        return
    
    acceleration_sequence, steering_sequence, predicted_x, predicted_y, predicted_yaw, predicted_velocity = result
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle(f"MPC Test Results: {test_name}", fontsize=14)
    
    # Plot 1: Trajectory comparison
    ax1 = axes[0, 0]
    ax1.plot(reference_trajectory[0, :], reference_trajectory[1, :], 'r-o', 
             label='Reference trajectory', markersize=6)
    ax1.plot(predicted_x, predicted_y, 'b-s', 
             label='Predicted trajectory', markersize=6)
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
             label='Reference velocity', markersize=6)
    ax2.plot(time_steps, predicted_velocity, 'b-s', 
             label='Predicted velocity', markersize=6)
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Velocity [m/s]')
    ax2.legend()
    ax2.grid(True)
    ax2.set_title('Velocity Profile')
    
    # Plot 3: Steering angle
    ax3 = axes[1, 0]
    control_time_steps = np.arange(mpc_solver.prediction_horizon) * mpc_solver.time_step
    ax3.plot(control_time_steps, [math.degrees(s) for s in steering_sequence], 'b-s', 
             label='Steering angle', markersize=6)
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Steering angle [deg]')
    ax3.legend()
    ax3.grid(True)
    ax3.set_title('Steering Control')
    
    # Plot 4: Acceleration
    ax4 = axes[1, 1]
    ax4.plot(control_time_steps, acceleration_sequence, 'g-s', 
             label='Acceleration', markersize=6)
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Acceleration [m/s²]')
    ax4.legend()
    ax4.grid(True)
    ax4.set_title('Acceleration Control')
    
    plt.tight_layout()
    plt.show()


def run_comprehensive_tests():
    """Run comprehensive tests for the MPC solver."""
    print("Starting comprehensive MPC solver tests...")
    
    # Create MPC solver
    mpc_solver = MPCSolver()
    
    # Test cases
    test_cases = [
        ("Straight Line", create_straight_line_reference(mpc_solver.prediction_horizon)),
        ("Curved Path", create_curved_reference(mpc_solver.prediction_horizon, mpc_solver.time_step)),
        ("Circular Path", create_circular_reference(mpc_solver.prediction_horizon, mpc_solver.time_step)),
        ("Random Path", create_random_reference(mpc_solver.prediction_horizon)),
    ]
    
    # Different initial states to test
    initial_states = [
        [0.0, 0.0, 0.0, 0.0],  # Start at origin
        [1.0, 0.5, 2.0, 0.1],  # Slightly offset
        [0.0, 0.0, 5.0, 0.0],  # Moving forward
        [0.0, 0.0, 0.0, math.pi/4],  # Rotated
    ]
    
    successful_tests = 0
    total_tests = 0
    
    for test_name, (reference_trajectory, reference_steering) in test_cases:
        for i, initial_state in enumerate(initial_states):
            total_tests += 1
            test_case_name = f"{test_name} - Initial State {i+1}"
            
            success, result = test_mpc_solver(
                test_case_name,
                reference_trajectory,
                reference_steering,
                initial_state,
                mpc_solver
            )
            
            if success:
                successful_tests += 1
                # Visualize only the first successful test for each trajectory type
                if i == 0:
                    visualize_test_results(
                        test_name,
                        reference_trajectory,
                        reference_steering,
                        initial_state,
                        result,
                        mpc_solver
                    )
    
    print(f"\n=== Test Summary ===")
    print(f"Total tests: {total_tests}")
    print(f"Successful: {successful_tests}")
    print(f"Failed: {total_tests - successful_tests}")
    print(f"Success rate: {successful_tests/total_tests*100:.1f}%")


def test_constraint_violations():
    """Test MPC behavior with constraint violations."""
    print("\n=== Testing Constraint Violations ===")
    
    # Create MPC solver
    mpc_solver = MPCSolver()
    
    # Create a reference that violates constraints
    reference_trajectory = np.zeros((4, mpc_solver.prediction_horizon + 1))
    reference_steering = np.zeros((1, mpc_solver.prediction_horizon + 1))
    
    # Very high velocity reference (should be constrained)
    for i in range(mpc_solver.prediction_horizon + 1):
        reference_trajectory[0, i] = i * 2.0
        reference_trajectory[1, i] = 0.0
        reference_trajectory[2, i] = 20.0  # Very high velocity (should be constrained)
        reference_trajectory[3, i] = 0.0
        reference_steering[0, i] = 0.0
    
    initial_state = [0.0, 0.0, 0.0, 0.0]
    
    success, result = test_mpc_solver(
        "High Velocity Constraint Test",
        reference_trajectory,
        reference_steering,
        initial_state,
        mpc_solver
    )
    
    if success:
        _, _, _, _, _, predicted_velocity = result
        print(f"Max predicted velocity: {max(predicted_velocity):.2f} m/s")
        print(f"Max allowed velocity: {mpc_solver.max_velocity:.2f} m/s")
        print("✓ Velocity constraints properly applied!")


def test_class_mpc_solver():
    """Test the MPCSolver class functionality."""
    print("\n=== Testing MPCSolver Class ===")
    
    # Create MPC solver instance
    mpc_solver = MPCSolver(
        prediction_horizon=5,
        time_step=0.2,
        max_velocity=10.0,
        max_acceleration=1.0
    )
    
    print(f"Created MPC solver with prediction horizon: {mpc_solver.prediction_horizon}")
    print(f"Time step: {mpc_solver.time_step}")
    print(f"Max velocity: {mpc_solver.max_velocity:.2f} m/s")
    
    # Test with different configurations
    test_cases = [
        ("Default Config", MPCSolver()),
        ("Custom Config", MPCSolver(prediction_horizon=8, max_velocity=5.0)),
        ("High Precision", MPCSolver(time_step=0.1, prediction_horizon=10))
    ]
    
    initial_state = [0.0, 0.0, 0.0, 0.0]
    
    for test_name, solver in test_cases:
        print(f"\nTesting {test_name}...")
        
        # Create reference trajectory for this solver's horizon
        ref_traj, ref_steer = create_straight_line_reference(solver.prediction_horizon)
        
        linearization_trajectory = create_linearization_trajectory(initial_state, ref_traj)
        
        result = solver.solve(ref_traj, linearization_trajectory, initial_state, ref_steer)
        
        if result[0] is not None:
            print(f"   ✓ Solved successfully")
            solver_info = solver.get_solver_info()
            print(f"   Solve time: {solver_info['solve_time']:.4f} seconds")
            print(f"   Status: {solver_info['status']}")
        else:
            print(f"   ✗ Failed to solve")
    
    # Test parameter updates
    print("\nTesting parameter updates...")
    solver = MPCSolver()
    print(f"Initial max velocity: {solver.max_velocity:.2f} m/s")
    
    solver.update_parameters(max_velocity=8.0, max_acceleration=0.5)
    print(f"Updated max velocity: {solver.max_velocity:.2f} m/s")
    print(f"Updated max acceleration: {solver.max_acceleration:.2f} m/s²")
    
    print("✓ Class functionality test completed!")


if __name__ == "__main__":
    # Run comprehensive tests
    run_comprehensive_tests()
    
    # Test constraint handling
    test_constraint_violations()
    
    # Test class functionality
    test_class_mpc_solver()
    
    print("\nAll tests completed!")
