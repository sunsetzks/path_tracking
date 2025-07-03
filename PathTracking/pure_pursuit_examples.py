"""
Pure Pursuit Controller Examples and Simulations

This module contains example trajectories, simulation functions, and demonstrations
for the Pure Pursuit path tracking controller.

Examples included:
- Forward driving simulation with S-curves
- Reverse driving simulation with backing maneuvers
- Direction conflict test scenarios
- Acceleration planning demonstrations
"""

import argparse
import math
import os
import sys
from typing import List

import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np

# Add the parent directory to the path so we can import the modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PathTracking.pure_pursuit import PurePursuitController
from PathTracking.trajectory import Trajectory
from PathTracking.utils.vehicle_display import VehicleDisplay
from PathTracking.velocity_planning import VelocityController
from PathTracking.vehicle_model import VehicleModel, VehicleState


def create_test_trajectory() -> Trajectory:
    """
    Create a test trajectory with curves and direction changes.

    Returns:
        Trajectory: Test trajectory
    """
    trajectory = Trajectory()

    # First segment: Forward curve
    for i in range(50):
        angle = i * np.deg2rad(3)
        radius = 20.0
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        yaw = angle + np.deg2rad(90)  # Tangent to circle
        trajectory.add_waypoint(x, y, yaw, direction=1)  # Forward

    # Second segment: Straight line
    last_x, last_y = trajectory.waypoints[-1].x, trajectory.waypoints[-1].y
    last_yaw = trajectory.waypoints[-1].yaw

    for i in range(1, 30):
        distance = i * 1.0
        x = last_x + distance * math.cos(last_yaw)
        y = last_y + distance * math.sin(last_yaw)
        trajectory.add_waypoint(x, y, last_yaw, direction=1)  # Forward

    return trajectory


def create_forward_test_trajectory() -> Trajectory:
    """
    Create a forward driving test trajectory with curves and straight segments.

    Returns:
        Trajectory: Forward driving test trajectory
    """
    trajectory = Trajectory()

    # First segment: Forward S-curve
    for i in range(40):
        angle = i * np.deg2rad(4)  # More curved
        radius = 15.0
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        yaw = angle + np.deg2rad(90)  # Tangent to circle
        trajectory.add_waypoint(x, y, yaw, direction=1)  # Forward

    # Second segment: Reverse S-curve (still forward direction)
    last_x, last_y = trajectory.waypoints[-1].x, trajectory.waypoints[-1].y
    last_yaw = trajectory.waypoints[-1].yaw
    
    for i in range(40):
        angle = i * np.deg2rad(-3)  # Opposite curve
        radius = 12.0
        offset_x = last_x + 5.0  # Offset to connect smoothly
        offset_y = last_y
        x = offset_x + radius * math.cos(angle + last_yaw - np.deg2rad(90))
        y = offset_y + radius * math.sin(angle + last_yaw - np.deg2rad(90))
        yaw = angle + last_yaw
        trajectory.add_waypoint(x, y, yaw, direction=1)  # Forward

    # Third segment: Straight line to finish
    last_x, last_y = trajectory.waypoints[-1].x, trajectory.waypoints[-1].y
    last_yaw = trajectory.waypoints[-1].yaw

    for i in range(1, 25):
        distance = i * 1.0
        x = last_x + distance * math.cos(last_yaw)
        y = last_y + distance * math.sin(last_yaw)
        trajectory.add_waypoint(x, y, last_yaw, direction=1)  # Forward

    return trajectory


def create_reverse_test_trajectory() -> Trajectory:
    """
    Create a reverse driving test trajectory (backing up scenario).

    Returns:
        Trajectory: Reverse driving test trajectory
    """
    trajectory = Trajectory()

    # Start at origin and create a reverse trajectory
    # This simulates backing into a parking spot or reversing along a path
    
    # First segment: Straight reverse line
    for i in range(20):
        x = -i * 1.0  # Moving backward (negative x)
        y = 0.0
        yaw = np.deg2rad(180)  # Facing backward
        trajectory.add_waypoint(x, y, yaw, direction=-1)  # Reverse

    # Second segment: Curved reverse maneuver (like backing into parking)
    last_x, last_y = trajectory.waypoints[-1].x, trajectory.waypoints[-1].y
    
    for i in range(30):
        angle = i * np.deg2rad(3)  # Gradual curve
        radius = 10.0
        # Create curve for backing maneuver
        x = last_x - radius * math.sin(angle)
        y = last_y - radius * (1 - math.cos(angle))
        yaw = np.deg2rad(180) + angle  # Gradually turning while backing
        trajectory.add_waypoint(x, y, yaw, direction=-1)  # Reverse

    # Third segment: Final straight reverse segment
    last_x, last_y = trajectory.waypoints[-1].x, trajectory.waypoints[-1].y
    last_yaw = trajectory.waypoints[-1].yaw

    for i in range(1, 15):
        distance = i * 0.8
        x = last_x + distance * math.cos(last_yaw)
        y = last_y + distance * math.sin(last_yaw)
        trajectory.add_waypoint(x, y, last_yaw, direction=-1)  # Reverse

    return trajectory


def create_direction_conflict_test_trajectory() -> Trajectory:
    """
    Create a test trajectory that demonstrates direction conflicts.
    
    This trajectory has segments where the path direction might conflict
    with the robot's optimal driving direction based on its orientation.

    Returns:
        Trajectory: Test trajectory with potential direction conflicts
    """
    trajectory = Trajectory()

    # Segment 1: Normal forward path
    for i in range(10):
        x = i * 1.0
        y = 0.0
        yaw = 0.0  # Facing forward (east)
        trajectory.add_waypoint(x, y, yaw, direction=1)  # Forward

    # Segment 2: Sharp U-turn where robot might want to reverse instead
    # Path says "forward" but robot facing forward would need to reverse
    for i in range(15):
        angle = i * np.deg2rad(12)  # 180 degree turn in 15 steps
        radius = 3.0
        center_x = 9.0
        center_y = 3.0
        x = center_x + radius * math.cos(math.pi - angle)
        y = center_y + radius * math.sin(math.pi - angle)
        yaw = math.pi - angle + np.deg2rad(90)  # Tangent direction
        
        # Force path direction to be forward even though backing might be easier
        trajectory.add_waypoint(x, y, yaw, direction=1)  # Force forward

    # Segment 3: Return path where robot is now facing backward
    # but path says forward
    last_x, last_y = trajectory.waypoints[-1].x, trajectory.waypoints[-1].y
    last_yaw = trajectory.waypoints[-1].yaw
    
    for i in range(1, 12):
        distance = i * 1.0
        x = last_x + distance * math.cos(last_yaw)
        y = last_y + distance * math.sin(last_yaw)
        # Keep the orientation from the turn
        trajectory.add_waypoint(x, y, last_yaw, direction=1)  # Force forward

    return trajectory


def run_forward_simulation() -> None:
    """
    Run forward driving simulation.
    """
    print("\n" + "=" * 60)
    print("🚗 FORWARD DRIVING SIMULATION")
    print("=" * 60)
    print("This simulation demonstrates forward path tracking with:")
    print("- S-curve trajectories")
    print("- Dynamic lookahead distance")
    print("- Physics-based acceleration/deceleration")
    print("- Smooth velocity planning")
    print("\nControls: Space = Pause/Resume, Q/ESC = Quit")
    print("Starting forward simulation...")

    # Create forward trajectory
    trajectory = create_forward_test_trajectory()

    # Create vehicle model
    wheelbase = 2.9
    vehicle_model = VehicleModel(wheelbase=wheelbase)
    
    # Create velocity controller optimized for forward driving
    velocity_controller = VelocityController(
        max_forward_velocity=6.0,  # Higher max speed for forward driving
        max_backward_velocity=2.0,
        max_acceleration=2.0,  # Faster acceleration for forward
        max_deceleration=2.5,
        goal_tolerance=1.0,
        velocity_tolerance=0.2,
        conservative_braking_factor=1.2,
        min_velocity=0.5,
    )
    
    # Create pure pursuit controller
    controller = PurePursuitController(
        wheelbase=wheelbase,
        trajectory=trajectory,
        min_lookahead=2.5,  # Larger lookahead for forward driving
        k_gain=1.2,
        max_steering_angle=np.deg2rad(45.0),
        velocity_controller=velocity_controller,
    )

    # Run simulation
    run_simulation(vehicle_model, controller, max_time=120.0)


def run_reverse_simulation() -> None:
    """
    Run reverse driving simulation.
    """
    print("\n" + "=" * 60)
    print("🔄 REVERSE DRIVING SIMULATION")
    print("=" * 60)
    print("This simulation demonstrates reverse path tracking with:")
    print("- Backing maneuvers")
    print("- Reduced speed limits for safety")
    print("- Smaller lookahead distance")
    print("- Conservative acceleration limits")
    print("\nControls: Space = Pause/Resume, Q/ESC = Quit")
    print("Starting reverse simulation...")

    # Create reverse trajectory
    trajectory = create_reverse_test_trajectory()

    # Create vehicle model
    wheelbase = 2.9
    vehicle_model = VehicleModel(wheelbase=wheelbase)
    
    # Create velocity controller optimized for reverse driving
    velocity_controller = VelocityController(
        max_forward_velocity=3.0,
        max_backward_velocity=2.5,  # Conservative reverse speed
        max_acceleration=1.0,  # Slower acceleration for reverse
        max_deceleration=1.5,  # Gentler deceleration
        goal_tolerance=0.8,  # Tighter tolerance for reverse parking
        velocity_tolerance=0.1,
        conservative_braking_factor=1.5,  # More conservative for reverse
        min_velocity=0.3,  # Lower minimum velocity for precise maneuvering
    )
    
    # Create pure pursuit controller
    controller = PurePursuitController(
        wheelbase=wheelbase,
        trajectory=trajectory,
        min_lookahead=1.5,  # Smaller lookahead for reverse driving
        k_gain=0.8,  # Reduced gain for more careful control
        max_steering_angle=np.deg2rad(35.0),  # Slightly reduced max steering
        velocity_controller=velocity_controller,
    )

    # Run simulation
    run_simulation(vehicle_model, controller, max_time=120.0)


def run_direction_conflict_test() -> None:
    """
    Run a test simulation to demonstrate direction conflict detection.
    """
    print("\n" + "=" * 60)
    print("🔄 DIRECTION CONFLICT TEST")
    print("=" * 60)
    print("This simulation demonstrates direction conflict detection:")
    print("- Path with forced forward direction")
    print("- Scenarios where backing up would be more natural")
    print("- Warning messages when conflicts are detected")
    print("- Real-time direction analysis in status display")
    print("\nControls: Space = Pause/Resume, Q/ESC = Quit")
    print("Starting direction conflict test...")

    # Create conflict test trajectory
    trajectory = create_direction_conflict_test_trajectory()

    # Create vehicle model
    wheelbase = 2.9
    vehicle_model = VehicleModel(wheelbase=wheelbase)
    
    # Create velocity controller with moderate settings
    velocity_controller = VelocityController(
        max_forward_velocity=4.0,
        max_backward_velocity=3.0,
        max_acceleration=1.5,
        max_deceleration=2.0,
        goal_tolerance=1.0,
        velocity_tolerance=0.1,
        conservative_braking_factor=1.3,
        min_velocity=0.3,
    )
    
    # Create pure pursuit controller with smaller lookahead for tight maneuvers
    controller = PurePursuitController(
        wheelbase=wheelbase,
        trajectory=trajectory,
        min_lookahead=1.8,
        k_gain=0.9,
        max_steering_angle=np.deg2rad(40.0),
        velocity_controller=velocity_controller,
    )

    # Run simulation
    run_simulation(vehicle_model, controller, max_time=150.0)


def run_simulation(
    vehicle_model: VehicleModel,
    controller: PurePursuitController,
    time_step: float = 0.1,
    max_time: float = 60.0,
) -> None:
    """
    Run simulation of pure pursuit controller.

    Args:
        vehicle_model (VehicleModel): Vehicle model
        controller (PurePursuitController): Pure pursuit controller
        time_step (float): Simulation time step [s]
        max_time (float): Maximum simulation time [s]

    Controls:
        Space: Pause/Resume simulation
        Q/ESC: Quit simulation
    """
    # Get trajectory from controller
    trajectory = controller.get_trajectory()
    if trajectory is None:
        raise ValueError("Controller has no trajectory set")
    
    # Initialize vehicle state at trajectory start
    nearest_point = trajectory.find_nearest_point(0, 0)
    vehicle_model.set_state(
        VehicleState(
            position_x=nearest_point.x,
            position_y=nearest_point.y,
            yaw_angle=nearest_point.yaw,
            velocity=0.0,
            steering_angle=0.0,
        )
    )

    # Setup plot with subplots - left column for info, right for simulation
    fig, (ax_info, ax_sim) = plt.subplots(1, 2, figsize=(16, 8), gridspec_kw={'width_ratios': [1, 3]})
    
    # Remove axes for info panel
    ax_info.axis('off')

    # Initialize vehicle display and path history
    vehicle_display = VehicleDisplay(wheelbase=controller.wheelbase)
    x_history: List[float] = []
    y_history: List[float] = []

    # Initialize simulation time and state
    time = 0.0
    paused = False

    # Calculate trajectory bounds for fixed view
    x_coords = [wp.x for wp in trajectory.waypoints]
    y_coords = [wp.y for wp in trajectory.waypoints]
    x_min, x_max = min(x_coords), max(x_coords)
    y_min, y_max = min(y_coords), max(y_coords)
    margin = 5.0  # Reduced margin since we have separate info panel

    def on_key(event) -> None:
        nonlocal paused
        if event.key == " ":  # Space key
            paused = not paused
            if paused:
                print("Simulation paused. Press Space to continue.")
            else:
                print("Simulation resumed.")
        elif event.key in ["q", "escape"]:  # Q key or ESC key
            print("Quitting simulation...")
            plt.close(fig)

    # Connect the key event handler
    fig.canvas.mpl_connect("key_press_event", on_key)
    print("Controls: Space = Pause/Resume, Q/ESC = Quit")

    try:
        while time < max_time and plt.fignum_exists(fig.number):
            if not paused:
                # Clear previous plots
                ax_info.clear()
                ax_sim.clear()
                ax_info.axis('off')  # Keep info panel without axes

                # Setup simulation plot properties
                ax_sim.grid(True)
                ax_sim.set_aspect('equal', adjustable='box')

                # Plot trajectory
                ax_sim.plot(x_coords, y_coords, "b--", label="Reference Path")

                # Plot goal position with circle and arrow
                goal_waypoint = trajectory.waypoints[-1]
                goal_circle = patches.Circle(
                    (goal_waypoint.x, goal_waypoint.y), 
                    controller.velocity_controller.goal_tolerance, 
                    fill=False, 
                    edgecolor='red', 
                    linewidth=2
                )
                ax_sim.add_patch(goal_circle)
                
                # Add arrow showing goal orientation
                arrow_length = 2.0  # Length of the arrow
                arrow_dx = arrow_length * math.cos(goal_waypoint.yaw)
                arrow_dy = arrow_length * math.sin(goal_waypoint.yaw)
                ax_sim.arrow(
                    goal_waypoint.x, goal_waypoint.y,
                    arrow_dx, arrow_dy,
                    head_width=0.8, head_length=0.6,
                    fc='red', ec='red', alpha=0.5,
                    linewidth=2,
                    label="Goal"
                )

                # Update vehicle state and history
                vehicle_state = vehicle_model.get_state()
                x_history.append(vehicle_state.position_x)
                y_history.append(vehicle_state.position_y)

                # Check if goal is reached
                goal_reached = controller.is_goal_reached(vehicle_state)

                # Find and plot lookahead point
                target_point = controller.find_target_point(vehicle_state)
                if target_point is not None:
                    target_x, target_y, target_direction = target_point
                    ax_sim.plot(target_x, target_y, 'mx', markersize=12, alpha=0.7, markeredgewidth=3, label="Lookahead Point")
                    
                    # Draw line from vehicle to lookahead point
                    ax_sim.plot([vehicle_state.position_x, target_x], 
                            [vehicle_state.position_y, target_y], 
                            'm--', linewidth=1, alpha=0.7, label="Lookahead Line")

                # Calculate and apply control with time step for acceleration planning
                steering, target_velocity = controller.compute_control(vehicle_state, time_step)
                vehicle_model.update_with_direct_control(
                    [steering, target_velocity], time_step
                )

                # Plot vehicle path
                ax_sim.plot(x_history, y_history, "g-", label="Vehicle Path")

                # Calculate physics-based information for display
                goal_waypoint = trajectory.waypoints[-1]
                dx_goal = vehicle_state.position_x - goal_waypoint.x
                dy_goal = vehicle_state.position_y - goal_waypoint.y
                distance_to_goal = math.sqrt(dx_goal * dx_goal + dy_goal * dy_goal)
                
                stopping_distance = controller.velocity_controller.calculate_stopping_distance(vehicle_state.velocity)
                # Determine direction based on target point direction
                current_is_forward = True  # Default
                if target_point is not None:
                    current_is_forward = target_point[2] > 0  # target_direction > 0 means forward
                max_vel_for_distance = controller.velocity_controller.calculate_max_velocity_for_distance(distance_to_goal, is_forward=current_is_forward)
                
                # Calculate current acceleration
                current_acceleration = controller.velocity_controller.calculate_current_acceleration(
                    vehicle_state.velocity, target_velocity, time_step
                )
                
                # Calculate lookahead distance
                lookahead_distance = controller.calculate_lookahead_distance(vehicle_state.velocity)

                # Plot vehicle
                vehicle_display.plot_vehicle(
                    vehicle_state.position_x,
                    vehicle_state.position_y,
                    vehicle_state.yaw_angle,
                    vehicle_state.steering_angle,
                    ax=ax_sim,  # Pass the specific axes
                )
                
                # Plot stopping distance circle around vehicle
                if not goal_reached and stopping_distance > 0.1:
                    stopping_circle = patches.Circle(
                        (vehicle_state.position_x, vehicle_state.position_y), 
                        stopping_distance, 
                        fill=False, 
                        edgecolor='orange', 
                        linewidth=1,
                        linestyle='--',
                        alpha=0.7
                    )
                    ax_sim.add_patch(stopping_circle)
                
                # Calculate direction information for display
                velocity_dir = "Forward" if vehicle_state.velocity >= 0 else "Reverse"
                target_dir = "Forward" if target_velocity >= 0 else "Reverse"
                
                # Get direction information if target point exists
                path_direction_str = "N/A"
                robot_direction_str = "N/A"
                direction_match = True
                
                if target_point is not None:
                    target_x, target_y, path_direction = target_point
                    robot_direction = controller.determine_driving_direction(
                        vehicle_state, target_x, target_y, path_direction
                    )
                    path_direction_str = "Forward" if path_direction > 0 else "Reverse"
                    robot_direction_str = "Forward" if robot_direction > 0 else "Reverse"
                    direction_match = abs(path_direction - robot_direction) <= 0.1
                
                # Calculate position errors for display
                goal_waypoint = trajectory.waypoints[-1]
                longitudinal_error, lateral_error, angle_error = controller.calculate_goal_errors(vehicle_state, goal_waypoint)
                angle_error_deg = math.degrees(angle_error)
                
                # Add status text to the left info panel
                status_text = f"1. Vehicle Status\n"
                status_text += f"{'='*25}\n"
                status_text += f"Time: {time:.1f}s\n\n"
                
                status_text += f"2. Motion Info\n"
                status_text += f"Velocity: {vehicle_state.velocity:.2f} m/s\n"
                status_text += f"  Direction: {velocity_dir}\n"
                status_text += f"Target Vel: {target_velocity:.2f} m/s\n"
                status_text += f"  Direction: {target_dir}\n"
                status_text += f"Acceleration: {current_acceleration:.2f} m/s²\n\n"
                
                status_text += f"3. Control Info\n"
                status_text += f"Lookahead: {lookahead_distance:.2f} m\n"
                status_text += f"Distance to Goal: {distance_to_goal:.2f} m\n"
                status_text += f"Stopping Dist: {stopping_distance:.2f} m\n"
                status_text += f"Max Vel for Dist: {max_vel_for_distance:.2f} m/s\n\n"
                
                status_text += f"4. Direction Info\n"
                status_text += f"Path Direction: {path_direction_str}\n"
                status_text += f"Robot Direction: {robot_direction_str}\n"
                status_text += f"Direction Match: {'YES' if direction_match else 'NO'}\n\n"
                
                status_text += f"5. Goal Status\n"
                status_text += f"Goal Reached: {'YES' if goal_reached else 'NO'}\n"
                status_text += f"Errors:\n"
                status_text += f"  Longitudinal: {longitudinal_error:.3f}m\n"
                status_text += f"  Lateral: {lateral_error:.3f}m\n"
                status_text += f"  Angular: {angle_error_deg:.1f}°\n"
                
                if goal_reached:
                    status_text += f"\nVehicle STOPPED at goal!"
                
                # Display status text in left panel
                ax_info.text(
                    0.05, 0.95, 
                    status_text, 
                    fontsize=11, 
                    verticalalignment='top',
                    horizontalalignment='left',
                    transform=ax_info.transAxes,
                    bbox=dict(boxstyle="round,pad=0.5", facecolor="lightblue", alpha=0.8),
                    family='monospace'
                )

                # Add legend to simulation plot
                ax_sim.legend(loc='upper right')

                # Set view limits with margin, compatible with equal aspect
                x_range = x_max - x_min
                y_range = y_max - y_min
                max_range = max(x_range, y_range)
                
                # Calculate center points
                x_center = (x_min + x_max) / 2
                y_center = (y_min + y_max) / 2
                
                # Set symmetric limits for equal aspect ratio
                half_range = max_range / 2 + margin
                ax_sim.set_xlim(x_center - half_range, x_center + half_range)
                ax_sim.set_ylim(y_center - half_range, y_center + half_range)
                
                # Set titles
                ax_info.set_title("Status Information", fontsize=14, fontweight='bold')
                ax_sim.set_title("Path Tracking Simulation", fontsize=14, fontweight='bold')

                plt.pause(0.001)
                time += time_step
            else:
                plt.pause(0.1)  # Reduce CPU usage while paused

    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
    finally:
        plt.close(fig)  # Ensure figure is closed properly


def demo_acceleration_planning() -> None:
    """
    Demonstration of different acceleration planning settings.
    """
    print("\n=== Acceleration Planning Demo ===")
    print("Testing different acceleration limits...")
    
    # Create simple straight line trajectory for clear acceleration demonstration
    trajectory = Trajectory()
    for i in range(100):  # 100m straight line
        x = i * 1.0
        y = 0.0
        yaw = 0.0  # Straight line
        trajectory.add_waypoint(x, y, yaw, direction=1)
    
    wheelbase = 2.9
    
    # Test different acceleration settings
    test_configs = [
        {"name": "High Acceleration", "max_acc": 3.0, "max_dec": 4.0},
        {"name": "Moderate Acceleration", "max_acc": 1.5, "max_dec": 2.0},
        {"name": "Low Acceleration", "max_acc": 0.5, "max_dec": 1.0},
    ]
    
    for config in test_configs:
        print(f"\nTesting {config['name']}:")
        print(f"  Max Acceleration: {config['max_acc']} m/s²")
        print(f"  Max Deceleration: {config['max_dec']} m/s²")
        
        # Create velocity controller with specific settings
        velocity_controller = VelocityController(
            max_forward_velocity=5.0,
            max_acceleration=config['max_acc'],
            max_deceleration=config['max_dec'],
            goal_tolerance=1.0,
            min_velocity=0.5,
        )
        
        # Test acceleration calculation
        dt = 0.1
        current_vel = 0.0
        target_vel = 5.0
        
        # Simulate acceleration from 0 to target velocity
        time_steps = []
        velocities = []
        accelerations = []
        
        for step in range(100):  # 10 seconds max
            time_val = step * dt
            
            # Calculate target velocity with acceleration limits
            desired_vel = target_vel
            velocity_diff = desired_vel - current_vel
            
            if velocity_diff > 0:
                max_vel_change = config['max_acc'] * dt
            else:
                max_vel_change = config['max_dec'] * dt
                
            if abs(velocity_diff) > max_vel_change:
                if velocity_diff > 0:
                    new_vel = current_vel + max_vel_change
                else:
                    new_vel = current_vel - max_vel_change
            else:
                new_vel = desired_vel
            
            acceleration = (new_vel - current_vel) / dt
            
            time_steps.append(time_val)
            velocities.append(current_vel)
            accelerations.append(acceleration)
            
            current_vel = new_vel
            
            if abs(current_vel - target_vel) < 0.01:
                break
        
        print(f"  Time to reach target velocity: {time_steps[-1]:.1f}s")
        print(f"  Final velocity: {velocities[-1]:.2f} m/s")
        print(f"  Max acceleration achieved: {max(accelerations):.2f} m/s²")


def demo_high_precision_control() -> None:
    """
    Demonstrate high-precision Pure Pursuit control with sub-centimeter accuracy.
    
    This example shows how to achieve 1cm positioning accuracy using:
    - High-precision controller configuration
    - Slow approach speeds near the goal
    - Precision zone control mode
    """
    print("=" * 60)
    print("High-Precision Pure Pursuit Control Demo (1cm accuracy)")
    print("=" * 60)
    
    # Create a simple test trajectory with a challenging final approach
    waypoints = [
        (0.0, 0.0, 0.0),          # Start
        (2.0, 0.0, 0.0),          # Straight segment
        (4.0, 1.0, np.pi/4),      # Turn
        (5.0, 2.0, np.pi/2),      # Another turn
        (5.0, 3.0, np.pi/2),      # Final goal - requires precise positioning
    ]
    
    trajectory = Trajectory()
    for x, y, yaw in waypoints:
        trajectory.add_waypoint(x, y, yaw)
    
    # Create high-precision controller with 1cm target accuracy
    wheelbase = 2.5
    precision_target = 0.01  # 1cm precision
    
    controller = PurePursuitController.create_high_precision_controller(
        wheelbase=wheelbase,
        trajectory=trajectory,
        precision_target=precision_target
    )
    
    # Create vehicle model
    vehicle_model = VehicleModel(wheelbase=wheelbase)
    
    # Set initial state
    initial_state = VehicleState(
        position_x=0.0,
        position_y=0.0,
        yaw_angle=0.0,
        velocity=0.0
    )
    vehicle_model.set_state(initial_state)
    
    print(f"Target precision: {precision_target*100:.1f}cm")
    print(f"Initial position: ({initial_state.position_x:.3f}, {initial_state.position_y:.3f})")
    
    goal_waypoint = trajectory.waypoints[-1]
    print(f"Goal position: ({goal_waypoint.x:.3f}, {goal_waypoint.y:.3f})")
    print(f"Goal tolerance: {controller.velocity_controller.goal_tolerance*100:.1f}cm")
    print("\nStarting high-precision simulation...")
    
    # Run simulation with detailed logging
    time_step = 0.05  # Smaller time step for precision
    max_time = 120.0  # Allow more time for precise approach
    simulation_time = 0.0
    
    # Storage for analysis
    positions = []
    errors = []
    velocities = []
    times = []
    precision_mode_times = []
    
    while simulation_time < max_time:
        vehicle_state = vehicle_model.get_state()
        
        # Check if goal reached
        if controller.is_goal_reached(vehicle_state):
            # Calculate final errors
            longitudinal_error, lateral_error, angle_error = controller.calculate_goal_errors(
                vehicle_state, goal_waypoint
            )
            final_distance_error = math.sqrt(longitudinal_error**2 + lateral_error**2)
            
            print(f"\n🎯 High-precision goal reached at t={simulation_time:.2f}s!")
            print(f"Final positioning accuracy: {final_distance_error*100:.2f}cm")
            print(f"Target was: {precision_target*100:.1f}cm")
            
            if final_distance_error <= precision_target:
                print("✅ TARGET PRECISION ACHIEVED!")
            else:
                print("⚠️  Target precision not quite achieved, but within tolerance")
            
            break
        
        # Get control input
        steering_angle, target_velocity = controller.compute_control(vehicle_state, time_step)
        
        # Update vehicle model
        vehicle_model.update_with_direct_control([steering_angle, target_velocity], time_step)
        
        # Log data for analysis
        positions.append((vehicle_state.position_x, vehicle_state.position_y))
        dx = vehicle_state.position_x - goal_waypoint.x
        dy = vehicle_state.position_y - goal_waypoint.y
        current_error = math.sqrt(dx*dx + dy*dy)
        errors.append(current_error)
        velocities.append(abs(vehicle_state.velocity))
        times.append(simulation_time)
        
        # Track when precision mode is active
        if controller.is_in_precision_zone(vehicle_state):
            precision_mode_times.append(simulation_time)
        
        # Progress logging every 2 seconds
        if simulation_time % 2.0 < time_step:
            precision_mode = "🎯 PRECISION" if controller.is_in_precision_zone(vehicle_state) else "🚗 NORMAL"
            print(f"t={simulation_time:5.1f}s | Error: {current_error*100:5.1f}cm | "
                  f"Speed: {abs(vehicle_state.velocity)*100:4.1f}cm/s | Mode: {precision_mode}")
        
        simulation_time += time_step
    
    if simulation_time >= max_time:
        print(f"\n⏰ Simulation timeout after {max_time}s")
        vehicle_state = vehicle_model.get_state()
        dx = vehicle_state.position_x - goal_waypoint.x
        dy = vehicle_state.position_y - goal_waypoint.y
        final_error = math.sqrt(dx*dx + dy*dy)
        print(f"Final error: {final_error*100:.2f}cm")
    
    # Analysis
    print(f"\n📊 Performance Analysis:")
    print(f"   Total simulation time: {simulation_time:.2f}s")
    print(f"   Minimum error achieved: {min(errors)*100:.2f}cm")
    print(f"   Average speed: {np.mean(velocities)*100:.1f}cm/s")
    print(f"   Time in precision mode: {len(precision_mode_times)*time_step:.1f}s")
    
    if precision_mode_times:
        precision_start = precision_mode_times[0]
        print(f"   Precision mode activated at: t={precision_start:.1f}s")
    
    return {
        'final_error': min(errors),
        'simulation_time': simulation_time,
        'precision_achieved': min(errors) <= precision_target,
        'positions': positions,
        'errors': errors,
        'times': times
    }


def main() -> None:
    """
    Main function to run pure pursuit simulations with command line argument support.
    """
    # Set up command line argument parsing
    parser = argparse.ArgumentParser(
        description="Pure Pursuit Path Tracking Simulation with Forward and Reverse Driving",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Simulation Options:
  1 - Forward Driving Simulation (High speed S-curve trajectory)
  2 - Reverse Driving Simulation (Backing and parking maneuvers) [DEFAULT]
  3 - Run Both Simulations
  4 - Direction Conflict Test (Demonstrates robot-based direction detection)
        """
    )
    
    parser.add_argument(
        'simulation_choice',
        type=int,
        nargs='?',  # Make argument optional
        default=2,  # Default to option 2 (Reverse Driving)
        choices=[1, 2, 3, 4],
        help='Simulation choice: 1=Forward, 2=Reverse (default), 3=Both, 4=Direction Conflict Test'
    )
    
    args = parser.parse_args()
    choice = args.simulation_choice
    
    print("Pure Pursuit Path Tracking Simulation with Forward and Reverse Driving")
    print("=" * 70)
    
    # Run acceleration demo first
    demo_acceleration_planning()
    
    # Run high-precision control demonstration
    demo_high_precision_control()
    
    print("\n" + "=" * 70)
    print("🎯 RUNNING SIMULATION")
    print("=" * 70)
    
    # Execute the selected simulation
    if choice == 1:
        print("Running Forward Driving Simulation...")
        run_forward_simulation()
    elif choice == 2:
        print("Running Reverse Driving Simulation...")
        run_reverse_simulation()
    elif choice == 3:
        print("Running Both Simulations...")
        run_forward_simulation()
        run_reverse_simulation()
    elif choice == 4:
        print("Running Direction Conflict Test...")
        run_direction_conflict_test()
    
    print("\nSimulation completed successfully!")


if __name__ == "__main__":
    main() 