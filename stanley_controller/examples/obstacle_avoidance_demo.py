"""
Obstacle Avoidance Demo

Demonstrates the Stanley controller's ability to navigate around obstacles.
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import os
from matplotlib.patches import Circle

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from stanley_controller import StanleyController, VehicleDynamics, SimulationEnvironment, Visualizer
from stanley_controller.utils.se2 import SE2
from stanley_controller.vehicle_dynamics import VehicleParameters
from stanley_controller.simulation import SimulationConfig, ScenarioGenerator
from stanley_controller.stanley_controller import ControlParams


def main():
    """Run obstacle avoidance demonstration."""
    print("Starting Obstacle Avoidance Demo...")
    
    # Create simulation configuration
    config = SimulationConfig(
        dt=0.1,
        max_time=30.0,
        collision_check=True,
        collision_safety_margin=0.5,
        goal_tolerance=1.0
    )
    
    # Create Stanley controller
    control_params = ControlParams(
        k_cross_track=0.8,  # Higher gain for better obstacle avoidance
        k_heading=1.0,
        max_steer_angle=np.radians(25.0),  # Slightly reduced for stability
        wheelbase=2.9,
        lookahead_distance=8.0  # Look further ahead for obstacles
    )
    controller = StanleyController(control_params)
    
    # Create vehicle dynamics
    vehicle_params = VehicleParameters()
    vehicle_dynamics = VehicleDynamics(vehicle_params)
    
    # Generate a straight path with obstacles
    path_points, path_yaw = ScenarioGenerator.straight_line_scenario(length=50.0)
    
    # Create simulation environment and add obstacles
    sim_env = SimulationEnvironment(config)
    
    # Add obstacles in a row
    obstacle_positions = [
        (15, -3, 1.5),
        (20, 2, 1.5),
        (25, -2, 1.5),
        (30, 3, 1.5),
        (35, -1, 1.5)
    ]
    
    for x, y, radius in obstacle_positions:
        sim_env.add_obstacle(x, y, radius)
    
    # Set initial state (start from the side)
    initial_state = SE2(x=-5.0, y=5.0, theta=0.0)
    
    # Run simulation
    print("Running simulation with obstacles...")
    result = sim_env.simulate(
        controller, vehicle_dynamics, initial_state, 
        path_points, path_yaw, target_speed=4.0
    )
    
    # Print results
    print(f"\nSimulation Results:")
    print(f"Success: {result['success']}")
    print(f"Collision: {result['collision']}")
    print(f"Time: {result['time']:.2f} s")
    print(f"Steps: {result['steps']}")
    
    if result['metrics']:
        metrics = result['metrics']
        print(f"Final distance to goal: {metrics['final_distance_to_goal']:.2f} m")
        print(f"Average tracking error: {metrics['average_distance_error']:.2f} m")
        print(f"Average speed: {metrics['average_speed']:.2f} m/s")
    
    # Create visualizer
    visualizer = Visualizer()
    
    # Plot trajectory with obstacles
    print("\nGenerating trajectory plot with obstacles...")
    plt.figure(figsize=(12, 8))
    
    # Plot reference path
    plt.plot(path_points[:, 0], path_points[:, 1], 'r--', linewidth=2, label='Reference Path')
    
    # Plot obstacles
    for obs in sim_env.obstacles:
        circle = Circle((obs.x, obs.y), obs.radius, color='red', alpha=0.5)
        plt.gca().add_patch(circle)
    
    # Plot vehicle trajectory
    if 'trajectory' in result:
        trajectory = result['trajectory']
        plt.plot(trajectory[:, 0], trajectory[:, 1], 'b-', linewidth=2, label='Vehicle Trajectory')
        
        # Mark start and end points
        plt.plot(trajectory[0, 0], trajectory[0, 1], 'go', markersize=10, label='Start')
        plt.plot(trajectory[-1, 0], trajectory[-1, 1], 'ro', markersize=10, label='End')
    
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Stanley Controller - Obstacle Avoidance')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.tight_layout()
    plt.show()
    
    # Plot control inputs
    print("Generating control inputs plot...")
    if 'controls' in result:
        controls = result['controls']
        times = [c['time'] for c in controls]
        steerings = [c['steering'] for c in controls]
        accelerations = [c['acceleration'] for c in controls]
        
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 8))
        
        # Steering angle
        ax1.plot(times, np.degrees(steerings), 'b-', linewidth=2)
        ax1.set_ylabel('Steering Angle (deg)')
        ax1.set_title('Control Inputs vs Time')
        ax1.grid(True, alpha=0.3)
        ax1.axhline(y=np.degrees(control_params.max_steer_angle), color='r', linestyle='--', alpha=0.5, label='Max Steering')
        ax1.axhline(y=-np.degrees(control_params.max_steer_angle), color='r', linestyle='--', alpha=0.5)
        ax1.legend()
        
        # Acceleration
        ax2.plot(times, accelerations, 'r-', linewidth=2)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Acceleration (m/s²)')
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        plt.show()
    
    # Plot error analysis
    print("Generating error analysis...")
    visualizer.plot_error_analysis(result)
    
    # Create animation if simulation was successful
    if result['success'] and not result['collision']:
        print("Creating animation...")
        visualizer.animate_simulation(result, reference_path=path_points)
    
    print("\nObstacle avoidance demo completed successfully!")


if __name__ == "__main__":
    main()