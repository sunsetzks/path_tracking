"""
Pure Pursuit Path Tracking Controller

This module implements the Pure Pursuit path tracking algorithm with the following features:
- Dynamic lookahead distance based on vehicle speed
- Support for both forward and backward trajectory segments
- Nearest point search on trajectory
- Visualization using vehicle display

Author: Assistant
"""

import numpy as np
import math
import matplotlib.pyplot as plt
import matplotlib.patches as patches  # Add import for Circle
import sys
import os

# Add the parent directory to the path so we can import the modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PathTracking.trajectory import Trajectory
from PathTracking.vehicle_model import VehicleModel, VehicleState
from PathTracking.utils.vehicle_display import VehicleDisplay


class PurePursuitController:
    """
    Pure Pursuit path tracking controller with dynamic lookahead distance.
    
    This controller finds the nearest point on the trajectory and then looks ahead
    by a distance proportional to the vehicle's speed to find a target point.
    The steering angle is calculated to guide the vehicle toward this target point.
    """
    
    def __init__(self, wheelbase, min_lookahead=1.0, k_gain=1.0, max_steering_angle=np.deg2rad(45.0)):
        """
        Initialize the Pure Pursuit controller.
        
        Args:
            wheelbase (float): Vehicle wheelbase [m]
            min_lookahead (float): Minimum lookahead distance [m]
            k_gain (float): Lookahead distance gain (lookahead = k_gain * velocity + min_lookahead)
            max_steering_angle (float): Maximum steering angle [rad]
        """
        self.wheelbase = wheelbase
        self.min_lookahead = min_lookahead
        self.k_gain = k_gain
        self.max_steering_angle = max_steering_angle
    
    def calculate_lookahead_distance(self, velocity):
        """
        Calculate dynamic lookahead distance based on vehicle speed.
        
        Args:
            velocity (float): Current vehicle velocity [m/s] (absolute value)
            
        Returns:
            float: Lookahead distance [m]
        """
        # Use absolute velocity for lookahead calculation (works for both forward and reverse)
        abs_velocity = abs(velocity)
        return self.k_gain * abs_velocity + self.min_lookahead
    
    def find_target_point(self, trajectory, vehicle_state):
        """
        Find target point on trajectory using current position and lookahead distance.
        
        Args:
            trajectory (Trajectory): Path trajectory
            vehicle_state (VehicleState): Current vehicle state
            
        Returns:
            tuple: (target_x, target_y, target_direction) - target point coordinates and direction
                  Returns None if no target point is found
        """
        # Get current position and velocity
        current_x = vehicle_state.position_x
        current_y = vehicle_state.position_y
        current_velocity = vehicle_state.velocity
        
        # Calculate lookahead distance based on speed
        lookahead = self.calculate_lookahead_distance(current_velocity)
        
        # Find nearest point on trajectory
        nearest_x, nearest_y, nearest_yaw, nearest_direction, nearest_s = trajectory.find_nearest_point(
            current_x, current_y)
        
        # Get trajectory length
        trajectory_length = trajectory.get_trajectory_length()
        
        # Search for target point at lookahead distance
        target_s = nearest_s
        found_target = False
        
        # Search forward along the path for lookahead distance
        search_resolution = 0.1  # Search step [m]
        max_search_dist = min(lookahead * 3.0, trajectory_length)  # Limit search range
        
        for dist in np.arange(search_resolution, max_search_dist, search_resolution):
            # Check point ahead on trajectory
            target_s_candidate = nearest_s + dist
            if target_s_candidate > trajectory_length:
                target_s_candidate = target_s_candidate - trajectory_length  # Loop around
                
            # Get point at this distance
            target_x, target_y, _, target_direction = trajectory.interpolate_at_distance(target_s_candidate)
            
            # Calculate distance from current position to this point
            dx = target_x - current_x
            dy = target_y - current_y
            distance = math.sqrt(dx**2 + dy**2)
            
            # If distance is close to lookahead, we found our target
            if abs(distance - lookahead) < search_resolution:
                target_s = target_s_candidate
                found_target = True
                break
                
            # If we've gone past the lookahead distance, interpolate to get exact point
            if distance > lookahead:
                # Interpolate between this point and previous point
                prev_s = target_s_candidate - search_resolution
                if prev_s < 0:
                    prev_s += trajectory_length
                    
                prev_x, prev_y, _, prev_direction = trajectory.interpolate_at_distance(prev_s)
                
                # Linear interpolation to find point at exactly lookahead distance
                # This is a simplified approach - more accurate methods could be used
                target_s = prev_s + search_resolution * (lookahead - math.sqrt((prev_x - current_x)**2 + 
                                                                             (prev_y - current_y)**2)) / (distance - 
                                                                             math.sqrt((prev_x - current_x)**2 + 
                                                                                     (prev_y - current_y)**2))
                
                # Get interpolated point
                target_x, target_y, _, target_direction = trajectory.interpolate_at_distance(target_s)
                found_target = True
                break
        
        if not found_target:
            # If no target found, use the furthest point we checked
            target_x, target_y, _, target_direction = trajectory.interpolate_at_distance(target_s)
        
        return target_x, target_y, target_direction
    
    def compute_steering_angle(self, vehicle_state, target_x, target_y):
        """
        Compute steering angle using pure pursuit geometry.
        
        Args:
            vehicle_state (VehicleState): Current vehicle state
            target_x (float): Target point x-coordinate
            target_y (float): Target point y-coordinate
            
        Returns:
            float: Steering angle [rad]
        """
        # Transform target point to vehicle coordinates
        dx = target_x - vehicle_state.position_x
        dy = target_y - vehicle_state.position_y
        
        # Coordinate transformation to vehicle frame
        cos_yaw = math.cos(vehicle_state.yaw_angle)
        sin_yaw = math.sin(vehicle_state.yaw_angle)
        
        # Target in vehicle coordinates
        target_x_veh = dx * cos_yaw + dy * sin_yaw
        target_y_veh = -dx * sin_yaw + dy * cos_yaw
        
        # Pure pursuit control law
        # For reverse driving, we need to flip the sign of the steering angle
        direction_factor = 1.0 if vehicle_state.velocity >= 0 else -1.0
        
        # Check if target is directly ahead or behind
        if abs(target_x_veh) < 1e-6:
            steering_angle = 0.0
        else:
            # Pure pursuit formula: delta = arctan(2L*y / (x^2 + y^2))
            # Where L is wheelbase, (x,y) is target in vehicle coordinates
            steering_angle = direction_factor * math.atan2(2.0 * self.wheelbase * target_y_veh,
                                                         target_x_veh**2 + target_y_veh**2)
        
        # Limit steering angle
        steering_angle = np.clip(steering_angle, -self.max_steering_angle, self.max_steering_angle)
        
        return steering_angle
    
    def compute_control_input(self, trajectory, vehicle_state):
        """
        Compute control input (steering angle and velocity) for the vehicle.
        
        Args:
            trajectory (Trajectory): Path trajectory
            vehicle_state (VehicleState): Current vehicle state
            
        Returns:
            tuple: (steering_angle, target_velocity) - control inputs
                  Returns (0, 0) if no target point is found
        """
        # Find target point
        target_point = self.find_target_point(trajectory, vehicle_state)
        
        if target_point is None:
            return 0.0, 0.0
        
        target_x, target_y, target_direction = target_point
        
        # Compute steering angle
        steering_angle = self.compute_steering_angle(vehicle_state, target_x, target_y)
        
        # Set velocity based on trajectory direction
        # Use a fixed velocity magnitude, but direction depends on trajectory
        velocity_magnitude = 5.0  # Fixed velocity magnitude [m/s] - reduced for smoother simulation
        target_velocity = velocity_magnitude * target_direction
        
        return steering_angle, target_velocity


def create_test_trajectory():
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
    
    # Third segment: Backward curve
    last_x, last_y = trajectory.waypoints[-1].x, trajectory.waypoints[-1].y
    
    for i in range(40):
        angle = i * np.deg2rad(4.5)
        radius = 15.0
        x = last_x + radius * math.cos(angle)
        y = last_y + radius * math.sin(angle)
        yaw = angle  # Tangent to circle
        trajectory.add_waypoint(x, y, yaw, direction=-1)  # Backward
    
    return trajectory


def run_simulation(trajectory, vehicle_model, controller, time_step=0.1, max_time=60.0, 
                  show_animation=True, save_animation=False):
    """
    Run simulation of pure pursuit controller.
    
    Args:
        trajectory (Trajectory): Path trajectory
        vehicle_model (VehicleModel): Vehicle model
        controller (PurePursuitController): Pure pursuit controller
        time_step (float): Simulation time step [s]
        max_time (float): Maximum simulation time [s]
        show_animation (bool): Whether to show animation
        save_animation (bool): Whether to save animation frames
        
    Returns:
        tuple: (time_list, x_list, y_list, yaw_list, velocity_list, steering_list, target_x_list, target_y_list)
    """
    # Initialize vehicle state
    initial_x, initial_y, initial_yaw, initial_direction, _ = trajectory.find_nearest_point(0, 0)
    initial_state = VehicleState(
        position_x=initial_x,
        position_y=initial_y,
        yaw_angle=initial_yaw,
        velocity=0.0,
        steering_angle=0.0
    )
    vehicle_model.set_state(initial_state)
    
    # Initialize visualization
    if show_animation:
        plt.figure(figsize=(12, 9))
        
        # Plot trajectory
        x_coords = [wp.x for wp in trajectory.waypoints]
        y_coords = [wp.y for wp in trajectory.waypoints]
        directions = [wp.direction for wp in trajectory.waypoints]
        
        # Plot forward trajectory in blue, backward in red
        for i in range(1, len(x_coords)):
            if directions[i] > 0:
                plt.plot(x_coords[i-1:i+1], y_coords[i-1:i+1], 'b-')
            else:
                plt.plot(x_coords[i-1:i+1], y_coords[i-1:i+1], 'r-')
        
        plt.plot(initial_x, initial_y, 'go', label='Start')
        plt.axis('equal')
        plt.grid(True)
        plt.title('Pure Pursuit Path Tracking Simulation')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
    
    # Create vehicle display
    vehicle_display = VehicleDisplay(wheelbase=controller.wheelbase)
    
    # Simulation loop
    time = 0.0
    time_list = [0.0]
    x_list = [initial_x]
    y_list = [initial_y]
    yaw_list = [initial_yaw]
    velocity_list = [0.0]
    steering_list = [0.0]
    target_x_list = []
    target_y_list = []
    
    # Animation update interval (update every N steps)
    animation_interval = 1
    
    try:
        while time < max_time:
            # Get current state
            vehicle_state = vehicle_model.get_state()
            
            # Calculate control input
            steering_angle, target_velocity = controller.compute_control_input(trajectory, vehicle_state)
            
            # Find target point for visualization
            target_point = controller.find_target_point(trajectory, vehicle_state)
            if target_point is not None:
                target_x, target_y, _ = target_point
                target_x_list.append(target_x)
                target_y_list.append(target_y)
            
            # Apply control input using direct control mode
            vehicle_model.update_with_direct_control([steering_angle, target_velocity], time_step)
            
            # Update time
            time += time_step
            
            # Record state
            current_state = vehicle_model.get_state()
            time_list.append(time)
            x_list.append(current_state.position_x)
            y_list.append(current_state.position_y)
            yaw_list.append(current_state.yaw_angle)
            velocity_list.append(current_state.velocity)
            steering_list.append(current_state.steering_angle)
            
            # Visualization
            if show_animation and (len(time_list) % animation_interval == 0):  # Update every N steps for smoother animation
                plt.cla()
                
                # Plot trajectory
                for i in range(1, len(x_coords)):
                    if directions[i] > 0:
                        plt.plot(x_coords[i-1:i+1], y_coords[i-1:i+1], 'b-', alpha=0.5, label='Forward Path' if i==1 else "")
                    else:
                        plt.plot(x_coords[i-1:i+1], y_coords[i-1:i+1], 'r-', alpha=0.5, label='Backward Path' if i==len(x_coords)-20 else "")
                
                # Plot vehicle path
                plt.plot(x_list, y_list, 'g--', linewidth=1.5, label='Vehicle Path')
                
                # Plot target points
                if target_x_list and target_y_list:
                    plt.plot(target_x_list[-1], target_y_list[-1], 'rx', markersize=8, label='Target Point')
                
                # Plot vehicle
                vehicle_display.plot_vehicle(
                    current_state.position_x,
                    current_state.position_y,
                    current_state.yaw_angle,
                    current_state.steering_angle,
                    body_color='cyan' if current_state.velocity >= 0 else 'magenta'
                )
                
                # Plot lookahead distance circle
                lookahead = controller.calculate_lookahead_distance(abs(current_state.velocity))
                circle = patches.Circle(
                    (current_state.position_x, current_state.position_y),
                    lookahead,
                    color='b',
                    fill=False,
                    linestyle='--',
                    alpha=0.4
                )
                plt.gca().add_patch(circle)
                
                # Add information text
                info_text = (
                    f"Speed: {current_state.velocity:.2f} m/s\n"
                    f"Steering: {np.rad2deg(current_state.steering_angle):.1f} deg\n"
                    f"Lookahead: {lookahead:.2f} m\n"
                    f"Time: {time:.1f} s"
                )
                plt.text(
                    0.02, 0.95, info_text,
                    transform=plt.gca().transAxes,
                    fontsize=10,
                    verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='white', alpha=0.6)
                )
                
                plt.axis('equal')
                plt.grid(True)
                plt.title('Pure Pursuit Path Tracking Simulation')
                plt.xlabel('X [m]')
                plt.ylabel('Y [m]')
                
                # Create legend without duplicate entries
                handles, labels = plt.gca().get_legend_handles_labels()
                by_label = dict(zip(labels, handles))
                plt.legend(by_label.values(), by_label.keys(), loc='upper right')
                
                plt.pause(0.001)
                
                if save_animation:
                    plt.savefig(f"pure_pursuit_frame_{int(time*10):03d}.png")
                    
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
    
    return time_list, x_list, y_list, yaw_list, velocity_list, steering_list, target_x_list, target_y_list


def main():
    """
    Main function to run the pure pursuit simulation.
    """
    print("Pure Pursuit Path Tracking Simulation")
    print("Press Ctrl+C to stop the simulation")
    
    # Create test trajectory
    trajectory = create_test_trajectory()
    print(f"Created trajectory with {len(trajectory.waypoints)} waypoints")
    
    # Create vehicle model
    wheelbase = 2.9
    vehicle_model = VehicleModel(wheelbase=wheelbase)
    
    # Create pure pursuit controller
    controller = PurePursuitController(
        wheelbase=wheelbase,
        min_lookahead=2.0,
        k_gain=0.1,
        max_steering_angle=np.deg2rad(45.0)
    )
    
    # Run simulation
    run_simulation(
        trajectory=trajectory,
        vehicle_model=vehicle_model,
        controller=controller,
        time_step=0.1,
        max_time=60.0,  # Reduced simulation time
        show_animation=True
    )
    
    print("Simulation complete")
    plt.ioff()  # Turn off interactive mode
    plt.show()  # Show the final plot


if __name__ == "__main__":
    main()
