"""
Segmented Ramp Down Control Example

This example demonstrates the segmented ramp down control strategy for precise
end-point control. The controller uses a multi-phase approach:

1. Normal Phase: Standard velocity control with deceleration planning
2. Transition Phase: Smooth transition between normal and fine adjustment
3. Fine Adjustment Phase: Low-speed creep for precise positioning  
4. Final Braking Phase: Final deceleration to complete stop

The segmented approach provides better stopping precision compared to
traditional single-phase control.
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple

# Import PathTracking modules
from PathTracking.config import PathTrackingConfig, VelocityControllerConfig
from PathTracking.velocity_planning import VelocityController
from PathTracking.vehicle_model import VehicleModel, VehicleState
from PathTracking.trajectory import Trajectory, Waypoint


def create_straight_line_trajectory(start_x: float, start_y: float, end_x: float, end_y: float, num_points: int = 100) -> Trajectory:
    """Create a straight line trajectory from start to end point."""
    from PathTracking.config import TrajectoryConfig
    
    waypoints = []
    for i in range(num_points):
        t = i / (num_points - 1)
        x = start_x + t * (end_x - start_x)
        y = start_y + t * (end_y - start_y)
        # Calculate yaw angle (direction from current to next point)
        if i < num_points - 1:
            next_t = (i + 1) / (num_points - 1)
            next_x = start_x + next_t * (end_x - start_x)
            next_y = start_y + next_t * (end_y - start_y)
            yaw = math.atan2(next_y - y, next_x - x)
        else:
            # For the last point, use the same yaw as the previous point
            yaw = math.atan2(end_y - start_y, end_x - start_x)
        
        waypoints.append(Waypoint(x, y, yaw, direction=1))
    
    # Create trajectory with config
    config = TrajectoryConfig()
    return Trajectory(config, waypoints)


def simulate_segmented_control(
    trajectory: Trajectory,
    initial_velocity: float = 3.0,
    target_direction: float = 1.0,
    dt: float = 0.1,
    max_time: float = 30.0,
    enable_segmented: bool = True
) -> Tuple[List[float], List[float], List[float], List[str], List[float]]:
    """
    Simulate vehicle motion with segmented ramp down control.
    
    Returns:
        Tuple of (time_history, position_history, velocity_history, phase_history, distance_history)
    """
    # Create velocity controller configuration
    vel_config = VelocityControllerConfig(
        max_forward_velocity=5.0,
        max_backward_velocity=2.0,
        max_acceleration=1.0,
        max_deceleration=2.0,
        goal_tolerance=0.1,
        velocity_tolerance=0.05,
        conservative_braking_factor=1.2,
        min_velocity=0.1,
        # Segmented control parameters
        enable_segmented_ramp_down=enable_segmented,
        fine_adjustment_distance=0.5,
        transition_zone_distance=0.1,
        creep_speed_factor=0.8,
        final_braking_distance=0.1,
        smooth_transition_enabled=True
    )
    
    # Create velocity controller
    velocity_controller = VelocityController(vel_config)
    
    # Create vehicle model
    config = PathTrackingConfig()
    vehicle = VehicleModel(config.vehicle)
    
    # Initialize vehicle state at trajectory start
    start_waypoint = trajectory.waypoints[0]
    vehicle_state = VehicleState(
        position_x=start_waypoint.x,
        position_y=start_waypoint.y,
        yaw_angle=0.0,
        velocity=initial_velocity,
        steering_angle=0.0
    )
    
    # Simulation history
    time_history = []
    position_history = []
    velocity_history = []
    phase_history = []
    distance_history = []
    
    # Simulation loop
    time = 0.0
    while time < max_time:
        # Get diagnostics
        diagnostics = velocity_controller.get_control_diagnostics(vehicle_state, trajectory, target_direction)
        
        # Calculate target velocity
        target_velocity = velocity_controller.compute_target_velocity(
            vehicle_state, trajectory, target_direction, dt
        )
        
        # Update vehicle state (simplified - just update position and velocity)
        vehicle_state.velocity = target_velocity
        vehicle_state.position_x += vehicle_state.velocity * dt
        
        # Store history
        time_history.append(time)
        position_history.append(vehicle_state.position_x)
        velocity_history.append(vehicle_state.velocity)
        phase_history.append(diagnostics['control_phase'])
        distance_history.append(diagnostics['distance_to_goal'])
        
        # Check if goal reached
        if velocity_controller.is_goal_reached(vehicle_state, trajectory):
            print(f"Goal reached at time: {time:.2f}s")
            break
        
        time += dt
    
    return time_history, position_history, velocity_history, phase_history, distance_history


def plot_comparison():
    """Compare segmented vs traditional control strategies."""
    # Create a straight line trajectory (10 meters)
    trajectory = create_straight_line_trajectory(0, 0, 10, 0, 50)
    
    # Simulate with segmented control
    time_seg, pos_seg, vel_seg, phase_seg, dist_seg = simulate_segmented_control(
        trajectory, initial_velocity=3.0, enable_segmented=True
    )
    
    # Simulate with traditional control
    time_trad, pos_trad, vel_trad, phase_trad, dist_trad = simulate_segmented_control(
        trajectory, initial_velocity=3.0, enable_segmented=False
    )
    
    # Create plots
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('Segmented Ramp Down Control vs Traditional Control', fontsize=16)
    
    # Plot 1: Position vs Time
    axes[0, 0].plot(time_seg, pos_seg, 'b-', label='Segmented Control', linewidth=2)
    axes[0, 0].plot(time_trad, pos_trad, 'r--', label='Traditional Control', linewidth=2)
    axes[0, 0].axhline(y=10, color='g', linestyle=':', alpha=0.7, label='Goal Position')
    axes[0, 0].set_xlabel('Time [s]')
    axes[0, 0].set_ylabel('Position [m]')
    axes[0, 0].set_title('Position vs Time')
    axes[0, 0].legend()
    axes[0, 0].grid(True, alpha=0.3)
    
    # Plot 2: Velocity vs Time
    axes[0, 1].plot(time_seg, vel_seg, 'b-', label='Segmented Control', linewidth=2)
    axes[0, 1].plot(time_trad, vel_trad, 'r--', label='Traditional Control', linewidth=2)
    axes[0, 1].set_xlabel('Time [s]')
    axes[0, 1].set_ylabel('Velocity [m/s]')
    axes[0, 1].set_title('Velocity vs Time')
    axes[0, 1].legend()
    axes[0, 1].grid(True, alpha=0.3)
    
    # Plot 3: Velocity vs Distance to Goal
    axes[1, 0].plot(dist_seg, vel_seg, 'b-', label='Segmented Control', linewidth=2)
    axes[1, 0].plot(dist_trad, vel_trad, 'r--', label='Traditional Control', linewidth=2)
    axes[1, 0].axvline(x=0.5, color='orange', linestyle=':', alpha=0.7, label='Fine Adjustment Zone')
    axes[1, 0].axvline(x=0.1, color='red', linestyle=':', alpha=0.7, label='Final Braking Zone')
    axes[1, 0].set_xlabel('Distance to Goal [m]')
    axes[1, 0].set_ylabel('Velocity [m/s]')
    axes[1, 0].set_title('Velocity vs Distance to Goal')
    axes[1, 0].legend()
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].set_xlim(0, 2)  # Focus on the final approach
    
    # Plot 4: Control Phases (for segmented control only)
    if phase_seg:
        # Create phase color map
        phase_colors = {'normal': 'blue', 'transition': 'orange', 'fine_adjustment': 'green', 'final_braking': 'red'}
        colors = [phase_colors.get(phase, 'gray') for phase in phase_seg]
        
        axes[1, 1].scatter(time_seg, dist_seg, c=colors, s=10, alpha=0.7)
        axes[1, 1].set_xlabel('Time [s]')
        axes[1, 1].set_ylabel('Distance to Goal [m]')
        axes[1, 1].set_title('Control Phases (Segmented Control)')
        
        # Create legend
        from matplotlib.patches import Patch
        legend_elements = [Patch(facecolor=color, label=phase.replace('_', ' ').title()) 
                          for phase, color in phase_colors.items()]
        axes[1, 1].legend(handles=legend_elements, loc='upper right')
        axes[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()


def demonstrate_parameter_tuning():
    """Demonstrate the effect of different segmented control parameters."""
    trajectory = create_straight_line_trajectory(0, 0, 10, 0, 50)
    
    # Test different fine adjustment distances
    fine_adjustment_distances = [0.3, 0.5, 0.8, 1.0]
    
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('Effect of Fine Adjustment Distance on Control Performance', fontsize=16)
    
    for i, fine_adj_dist in enumerate(fine_adjustment_distances):
        # Create custom config
        vel_config = VelocityControllerConfig(
            max_forward_velocity=5.0,
            max_deceleration=2.0,
            enable_segmented_ramp_down=True,
            fine_adjustment_distance=fine_adj_dist,
            transition_zone_distance=0.1,
            creep_speed_factor=0.8,
            final_braking_distance=0.1
        )
        
        velocity_controller = VelocityController(vel_config)
        
        # Simulate
        time_hist, pos_hist, vel_hist, phase_hist, dist_hist = simulate_segmented_control(
            trajectory, initial_velocity=3.0, enable_segmented=True
        )
        
        # Plot velocity vs distance
        row = i // 2
        col = i % 2
        axes[row, col].plot(dist_hist, vel_hist, linewidth=2, label=f'Fine Adj: {fine_adj_dist}m')
        axes[row, col].axvline(x=fine_adj_dist, color='red', linestyle='--', alpha=0.7, label='Fine Adj Zone')
        axes[row, col].set_xlabel('Distance to Goal [m]')
        axes[row, col].set_ylabel('Velocity [m/s]')
        axes[row, col].set_title(f'Fine Adjustment Distance: {fine_adj_dist}m')
        axes[row, col].legend()
        axes[row, col].grid(True, alpha=0.3)
        axes[row, col].set_xlim(0, 2)
    
    plt.tight_layout()
    plt.show()


def main():
    """Main function to run the segmented ramp down control examples."""
    print("=== Segmented Ramp Down Control Example ===")
    print()
    
    # Demonstrate basic functionality
    print("1. Basic Segmented Control Demonstration")
    trajectory = create_straight_line_trajectory(0, 0, 10, 0, 50)
    
    # Create velocity controller with segmented control
    vel_config = VelocityControllerConfig(
        enable_segmented_ramp_down=True,
        fine_adjustment_distance=0.5,
        creep_speed_factor=0.8
    )
    velocity_controller = VelocityController(vel_config)
    
    # Print configuration
    print(f"Fine Adjustment Distance: {vel_config.fine_adjustment_distance}m")
    print(f"Transition Zone Distance: {vel_config.transition_zone_distance}m")
    print(f"Final Braking Distance: {vel_config.final_braking_distance}m")
    print(f"Creep Speed Factor: {vel_config.creep_speed_factor}")
    print()
    
    # Simulate a few steps to show phase transitions
    vehicle_state = VehicleState(position_x=0, position_y=0, yaw_angle=0, velocity=3.0, steering_angle=0)
    
    distances = [2.0, 0.8, 0.5, 0.3, 0.1, 0.05]
    for dist in distances:
        vehicle_state.position_x = 10 - dist  # Position vehicle at specific distance from goal
        diagnostics = velocity_controller.get_control_diagnostics(vehicle_state, trajectory, 1.0)
        target_vel = velocity_controller.compute_target_velocity(vehicle_state, trajectory, 1.0)
        
        print(f"Distance: {dist:.2f}m | Phase: {diagnostics['control_phase']:>15} | "
              f"Target Velocity: {target_vel:.3f} m/s | Creep Speed: {diagnostics['creep_speed']:.3f} m/s")
    
    print()
    print("2. Plotting Comparison...")
    plot_comparison()
    
    print()
    print("3. Parameter Tuning Demonstration...")
    demonstrate_parameter_tuning()
    
    print()
    print("=== Example Complete ===")


if __name__ == "__main__":
    main() 