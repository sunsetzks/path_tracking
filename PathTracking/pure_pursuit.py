"""
Pure Pursuit Path Tracking Controller

This module implements the Pure Pursuit path tracking algorithm with the following features:
- Dynamic lookahead distance based on vehicle speed
- Support for both forward and backward trajectory segments
- Nearest point search on trajectory
- Visualization using vehicle display

Author: Assistant
"""

import matplotlib
matplotlib.use('Qt5Agg')  # Use Qt5Agg backend instead of TkAgg
import matplotlib.pyplot as plt
import matplotlib.patches as patches  # Add import for Circle
import numpy as np
import math
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
        nearest_point = trajectory.find_nearest_point(current_x, current_y)
        
        # Get trajectory length
        trajectory_length = trajectory.get_trajectory_length()
        
        # If we're near the end of the trajectory, use the last point
        if nearest_point.s + lookahead >= trajectory_length:
            point = trajectory.interpolate_at_distance(trajectory_length)
            return point.x, point.y, point.direction
        
        # Search for target point at lookahead distance
        target_s = nearest_point.s + lookahead
        
        # Get interpolated point
        point = trajectory.interpolate_at_distance(target_s)
        return point.x, point.y, point.direction
    
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
    nearest_point = trajectory.find_nearest_point(0, 0)
    initial_state = VehicleState(
        position_x=nearest_point.x,
        position_y=nearest_point.y,
        yaw_angle=nearest_point.yaw,
        velocity=0.0,
        steering_angle=0.0
    )
    vehicle_model.set_state(initial_state)
    
    # Initialize visualization
    if show_animation:
        plt.figure(figsize=(12, 9))
        
        # Plot trajectory once at the beginning
        x_coords = [wp.x for wp in trajectory.waypoints]
        y_coords = [wp.y for wp in trajectory.waypoints]
        directions = [wp.direction for wp in trajectory.waypoints]
        
        # Create static plot elements
        forward_path = None
        backward_path = None
        for i in range(1, len(x_coords)):
            if directions[i] > 0:
                if forward_path is None:
                    forward_path = plt.plot(x_coords[i-1:i+1], y_coords[i-1:i+1], 'b-', alpha=0.5, label='Forward Path')[0]
                else:
                    plt.plot(x_coords[i-1:i+1], y_coords[i-1:i+1], 'b-', alpha=0.5)
            else:
                if backward_path is None:
                    backward_path = plt.plot(x_coords[i-1:i+1], y_coords[i-1:i+1], 'r-', alpha=0.5, label='Backward Path')[0]
                else:
                    plt.plot(x_coords[i-1:i+1], y_coords[i-1:i+1], 'r-', alpha=0.5)
        
        # Initialize dynamic plot elements
        vehicle_path_line, = plt.plot([], [], 'g--', linewidth=1.5, label='Vehicle Path')
        target_point_line, = plt.plot([], [], 'rx', markersize=8, label='Target Point')
        lookahead_circle = patches.Circle((0, 0), 1.0, color='b', fill=False, linestyle='--', alpha=0.4)
        plt.gca().add_patch(lookahead_circle)
        
        # Create vehicle display
        vehicle_display = VehicleDisplay(wheelbase=controller.wheelbase)
        
        # Add static elements
        plt.grid(True)
        plt.title('Pure Pursuit Path Tracking Simulation')
        plt.xlabel('X [m]')
        plt.ylabel('Y [m]')
        plt.axis('equal')
        
        # Create info text box
        info_text = plt.text(
            0.02, 0.95, '',
            transform=plt.gca().transAxes,
            fontsize=10,
            verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='white', alpha=0.6)
        )
        
        # Create legend
        plt.legend(loc='upper right')
    else:
        vehicle_display = VehicleDisplay(wheelbase=controller.wheelbase)
    
    # Simulation loop
    time = 0.0
    time_list = [0.0]
    x_list = [initial_state.position_x]
    y_list = [initial_state.position_y]
    yaw_list = [initial_state.yaw_angle]
    velocity_list = [0.0]
    steering_list = [0.0]
    target_x_list = []
    target_y_list = []
    
    # Animation update interval (update every N steps)
    animation_interval = 5  # Reduced update frequency
    
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
            if show_animation and (len(time_list) % animation_interval == 0):
                # Update vehicle path
                vehicle_path_line.set_data(x_list, y_list)
                
                # Update target point
                if target_x_list and target_y_list:
                    target_point_line.set_data([target_x_list[-1]], [target_y_list[-1]])
                
                # Update vehicle
                vehicle_display.plot_vehicle(
                    current_state.position_x,
                    current_state.position_y,
                    current_state.yaw_angle,
                    current_state.steering_angle,
                    body_color='cyan' if current_state.velocity >= 0 else 'magenta'
                )
                
                # Update lookahead circle
                lookahead = controller.calculate_lookahead_distance(abs(current_state.velocity))
                lookahead_circle.center = (current_state.position_x, current_state.position_y)
                lookahead_circle.radius = lookahead
                
                # Update information text
                info_text.set_text(
                    f"Speed: {current_state.velocity:.2f} m/s\n"
                    f"Steering: {np.rad2deg(current_state.steering_angle):.1f} deg\n"
                    f"Lookahead: {lookahead:.2f} m\n"
                    f"Time: {time:.1f} s"
                )
                
                # Update plot limits to follow the vehicle
                plt.xlim(current_state.position_x - 30, current_state.position_x + 30)
                plt.ylim(current_state.position_y - 30, current_state.position_y + 30)
                
                plt.draw()
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
    
    # Enable interactive mode
    plt.ion()
    
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
    
    try:
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
        
        # Disable interactive mode and show the final plot
        plt.ioff()
        plt.show(block=True)  # This will block until the window is closed
        
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
        plt.ioff()
        plt.close('all')


if __name__ == "__main__":
    main()
