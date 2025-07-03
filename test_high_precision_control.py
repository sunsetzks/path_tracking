#!/usr/bin/env python3
"""
High-Precision Pure Pursuit Control Test Script

This script demonstrates and validates the ability to achieve sub-centimeter
positioning accuracy using enhanced Pure Pursuit control with precision modes.

Key Features Tested:
- 1cm positioning accuracy target
- Precision zone control mode
- Slow approach speeds near goal
- Detailed error analysis and reporting
- Performance metrics collection

Usage:
    python test_high_precision_control.py

Author: Path Tracking System
"""

import math
import sys
import os
from typing import Dict, List, Tuple

import numpy as np

# Add the PathTracking module to the path
sys.path.append(os.path.join(os.path.dirname(__file__), 'PathTracking'))

from PathTracking.pure_pursuit import PurePursuitController
from PathTracking.trajectory import Trajectory
from PathTracking.vehicle_model import VehicleModel, VehicleState


def create_precision_test_trajectory(test_type: str = "simple") -> Trajectory:
    """
    Create different test trajectories for precision validation.
    
    Args:
        test_type (str): Type of test trajectory
            - "simple": Straight line to a single goal point
            - "l_shape": L-shaped path requiring precise corner navigation
            - "parking": Parallel parking scenario with tight precision requirements
            
    Returns:
        Trajectory: Test trajectory for precision validation
    """
    trajectory = Trajectory()
    
    if test_type == "simple":
        # Simple straight line test - easiest case
        waypoints = [
            (0.0, 0.0, 0.0),
            (1.0, 0.0, 0.0),
            (2.0, 0.0, 0.0),
        ]
    elif test_type == "l_shape":
        # L-shaped path with precise corner
        waypoints = [
            (0.0, 0.0, 0.0),
            (2.0, 0.0, 0.0),
            (2.0, 2.0, np.pi/2),
        ]
    elif test_type == "parking":
        # Parking maneuver requiring high precision
        waypoints = [
            (0.0, 0.0, 0.0),
            (3.0, 0.0, 0.0),
            (4.0, 1.0, np.pi/2),
            (4.0, 3.0, np.pi/2),
            (3.0, 4.0, np.pi),
            (1.0, 4.0, np.pi),  # Final parking position
        ]
    else:
        raise ValueError(f"Unknown test type: {test_type}")
    
    for x, y, yaw in waypoints:
        trajectory.add_waypoint(x, y, yaw)
    
    return trajectory


def run_precision_test(
    test_name: str,
    trajectory: Trajectory,
    precision_target: float = 0.01,
    max_time: float = 120.0,
    time_step: float = 0.05,
    verbose: bool = True
) -> Dict:
    """
    Run a single precision test and return detailed results.
    
    Args:
        test_name (str): Name of the test
        trajectory (Trajectory): Test trajectory
        precision_target (float): Target precision in meters
        max_time (float): Maximum simulation time
        time_step (float): Simulation time step
        verbose (bool): Whether to print detailed progress
        
    Returns:
        Dict: Test results including final error, success status, and performance metrics
    """
    if verbose:
        print(f"\n{'='*60}")
        print(f"Running Precision Test: {test_name}")
        print(f"{'='*60}")
        print(f"Target precision: {precision_target*100:.1f}cm")
    
    # Create high-precision controller
    wheelbase = 2.5
    controller = PurePursuitController.create_high_precision_controller(
        wheelbase=wheelbase,
        trajectory=trajectory,
        precision_target=precision_target
    )
    
    # Create vehicle model and set initial state
    vehicle_model = VehicleModel(wheelbase=wheelbase)
    initial_state = VehicleState(
        position_x=trajectory.waypoints[0].x,
        position_y=trajectory.waypoints[0].y,
        yaw_angle=trajectory.waypoints[0].yaw,
        velocity=0.0
    )
    vehicle_model.set_state(initial_state)
    
    goal_waypoint = trajectory.waypoints[-1]
    if verbose:
        print(f"Start: ({initial_state.position_x:.3f}, {initial_state.position_y:.3f})")
        print(f"Goal: ({goal_waypoint.x:.3f}, {goal_waypoint.y:.3f})")
    
    # Simulation variables
    simulation_time = 0.0
    errors = []
    velocities = []
    times = []
    precision_mode_times = []
    max_velocity = 0.0
    
    # Main simulation loop
    while simulation_time < max_time:
        vehicle_state = vehicle_model.get_state()
        
        # Check if goal reached
        if controller.is_goal_reached(vehicle_state):
            # Calculate final errors
            longitudinal_error, lateral_error, angle_error = controller.calculate_goal_errors(
                vehicle_state, goal_waypoint
            )
            final_distance_error = math.sqrt(longitudinal_error**2 + lateral_error**2)
            
            success = final_distance_error <= precision_target
            
            if verbose:
                print(f"\n🎯 Goal reached at t={simulation_time:.2f}s!")
                print(f"Final positioning accuracy: {final_distance_error*100:.2f}cm")
                if success:
                    print("✅ TARGET PRECISION ACHIEVED!")
                else:
                    print("⚠️  Target precision not achieved")
            
            return {
                'test_name': test_name,
                'success': success,
                'final_error': final_distance_error,
                'target_precision': precision_target,
                'simulation_time': simulation_time,
                'min_error': min(errors) if errors else final_distance_error,
                'max_velocity': max_velocity,
                'precision_mode_duration': len(precision_mode_times) * time_step,
                'total_trajectory_points': len(trajectory.waypoints),
                'errors': errors,
                'times': times
            }
        
        # Get control input and update vehicle
        steering_angle, target_velocity = controller.compute_control(vehicle_state, time_step)
        vehicle_model.update_with_direct_control([steering_angle, target_velocity], time_step)
        
        # Record data
        dx = vehicle_state.position_x - goal_waypoint.x
        dy = vehicle_state.position_y - goal_waypoint.y
        current_error = math.sqrt(dx*dx + dy*dy)
        
        errors.append(current_error)
        velocities.append(abs(vehicle_state.velocity))
        times.append(simulation_time)
        max_velocity = max(max_velocity, abs(vehicle_state.velocity))
        
        if controller.is_in_precision_zone(vehicle_state):
            precision_mode_times.append(simulation_time)
        
        # Progress logging
        if verbose and simulation_time % 5.0 < time_step:
            mode = "🎯 PRECISION" if controller.is_in_precision_zone(vehicle_state) else "🚗 NORMAL"
            print(f"t={simulation_time:5.1f}s | Error: {current_error*100:5.1f}cm | "
                  f"Speed: {abs(vehicle_state.velocity)*100:4.1f}cm/s | {mode}")
        
        simulation_time += time_step
    
    # Timeout case
    vehicle_state = vehicle_model.get_state()
    dx = vehicle_state.position_x - goal_waypoint.x
    dy = vehicle_state.position_y - goal_waypoint.y
    final_error = math.sqrt(dx*dx + dy*dy)
    
    if verbose:
        print(f"\n⏰ Simulation timeout after {max_time}s")
        print(f"Final error: {final_error*100:.2f}cm")
    
    return {
        'test_name': test_name,
        'success': False,
        'final_error': final_error,
        'target_precision': precision_target,
        'simulation_time': max_time,
        'min_error': min(errors) if errors else final_error,
        'max_velocity': max_velocity,
        'precision_mode_duration': len(precision_mode_times) * time_step,
        'total_trajectory_points': len(trajectory.waypoints),
        'timeout': True,
        'errors': errors,
        'times': times
    }


def run_precision_test_suite() -> None:
    """
    Run a comprehensive suite of precision tests with different scenarios.
    """
    print("="*80)
    print("HIGH-PRECISION PURE PURSUIT CONTROL TEST SUITE")
    print("="*80)
    print("Testing sub-centimeter positioning accuracy capabilities")
    print()
    
    # Test configurations
    test_configs = [
        {
            'name': 'Simple Straight Line (1cm)',
            'trajectory': create_precision_test_trajectory('simple'),
            'precision_target': 0.01,
            'expected_time': 30.0
        },
        {
            'name': 'L-Shaped Path (1cm)',
            'trajectory': create_precision_test_trajectory('l_shape'),
            'precision_target': 0.01,
            'expected_time': 60.0
        },
        {
            'name': 'Parking Maneuver (1cm)',
            'trajectory': create_precision_test_trajectory('parking'),
            'precision_target': 0.01,
            'expected_time': 90.0
        },
        {
            'name': 'Ultra-High Precision (5mm)',
            'trajectory': create_precision_test_trajectory('simple'),
            'precision_target': 0.005,
            'expected_time': 45.0
        }
    ]
    
    results = []
    
    # Run all tests
    for config in test_configs:
        result = run_precision_test(
            test_name=config['name'],
            trajectory=config['trajectory'],
            precision_target=config['precision_target'],
            max_time=config['expected_time'] * 2,  # Allow extra time
            verbose=True
        )
        results.append(result)
        
        # Brief pause between tests
        print("\n" + "-"*40 + "\n")
    
    # Summary report
    print("="*80)
    print("TEST SUITE SUMMARY")
    print("="*80)
    
    successful_tests = sum(1 for r in results if r['success'])
    total_tests = len(results)
    
    print(f"Overall Success Rate: {successful_tests}/{total_tests} ({successful_tests/total_tests*100:.1f}%)")
    print()
    
    for result in results:
        status = "✅ PASS" if result['success'] else "❌ FAIL"
        timeout_note = " (TIMEOUT)" if result.get('timeout', False) else ""
        
        print(f"{status:8} | {result['test_name']:25} | "
              f"Error: {result['final_error']*100:5.2f}cm | "
              f"Target: {result['target_precision']*100:4.1f}cm | "
              f"Time: {result['simulation_time']:5.1f}s{timeout_note}")
    
    print()
    print("Performance Analysis:")
    print("-" * 40)
    
    avg_error = np.mean([r['final_error'] for r in results if r['success']])
    avg_time = np.mean([r['simulation_time'] for r in results if r['success']])
    avg_max_velocity = np.mean([r['max_velocity'] for r in results])
    
    if successful_tests > 0:
        print(f"Average final error (successful tests): {avg_error*100:.2f}cm")
        print(f"Average completion time: {avg_time:.1f}s")
    print(f"Average maximum velocity: {avg_max_velocity*100:.1f}cm/s")
    
    # Best performance
    best_result = min([r for r in results if r['success']], 
                     key=lambda x: x['final_error'], default=None)
    if best_result:
        print(f"\nBest performance: {best_result['test_name']}")
        print(f"  Final error: {best_result['final_error']*100:.2f}cm")
        print(f"  Completion time: {best_result['simulation_time']:.1f}s")


def main():
    """Main function to run precision tests."""
    print("High-Precision Pure Pursuit Control Validation")
    
    # Run the comprehensive test suite
    run_precision_test_suite()
    
    print("\n" + "="*80)
    print("Testing completed! See results above for detailed performance analysis.")
    print("="*80)


if __name__ == "__main__":
    main() 