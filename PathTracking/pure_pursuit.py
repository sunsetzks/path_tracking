"""
Pure Pursuit Path Tracking Controller

This module implements the Pure Pursuit path tracking algorithm with the following features:
- Dynamic lookahead distance based on vehicle speed
- Support for both forward and backward trajectory segments
- Nearest point search on trajectory
- Visualization using vehicle display

Author: Assistant
"""

from typing import Optional, Tuple, List
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
    
    def __init__(self, wheelbase: float, min_lookahead: float = 1.0, k_gain: float = 1.0, max_steering_angle: float = np.deg2rad(45.0)) -> None:
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
    
    def calculate_lookahead_distance(self, velocity: float) -> float:
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
    
    def find_target_point(self, trajectory: Trajectory, vehicle_state: VehicleState) -> Optional[Tuple[float, float, float]]:
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
    
    def compute_steering_angle(self, vehicle_state: VehicleState, target_x: float, target_y: float) -> float:
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
    
    def compute_control_input(self, trajectory: Trajectory, vehicle_state: VehicleState) -> Tuple[float, float]:
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


def run_simulation(trajectory: Trajectory, vehicle_model: VehicleModel, controller: PurePursuitController, time_step: float = 0.1, max_time: float = 60.0) -> None:
    """
    Run simulation of pure pursuit controller.
    
    Args:
        trajectory (Trajectory): Path trajectory
        vehicle_model (VehicleModel): Vehicle model
        controller (PurePursuitController): Pure pursuit controller
        time_step (float): Simulation time step [s]
        max_time (float): Maximum simulation time [s]
    
    Controls:
        Space: Pause/Resume simulation
        Q/ESC: Quit simulation
    """
    # Initialize vehicle state at trajectory start
    nearest_point = trajectory.find_nearest_point(0, 0)
    vehicle_model.set_state(VehicleState(
        position_x=nearest_point.x,
        position_y=nearest_point.y,
        yaw_angle=nearest_point.yaw,
        velocity=0.0,
        steering_angle=0.0
    ))

    # Setup plot
    fig = plt.figure(figsize=(10, 8))
    
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
    margin = 5.0  # Add margin to view
    
    def on_key(event) -> None:
        nonlocal paused
        if event.key == ' ':  # Space key
            paused = not paused
            if paused:
                print("Simulation paused. Press Space to continue.")
            else:
                print("Simulation resumed.")
        elif event.key in ['q', 'escape']:  # Q key or ESC key
            print("Quitting simulation...")
            plt.close(fig)
    
    # Connect the key event handler
    fig.canvas.mpl_connect('key_press_event', on_key)
    print("Controls: Space = Pause/Resume, Q/ESC = Quit")
    
    try:
        while time < max_time and plt.fignum_exists(fig.number):
            if not paused:
                # Clear previous plot
                plt.cla()
                
                # Setup plot properties
                plt.grid(True)
                plt.axis('equal')
                
                # Plot trajectory
                plt.plot(x_coords, y_coords, 'b--', label='Reference Path')
                
                # Update vehicle state and history
                vehicle_state = vehicle_model.get_state()
                x_history.append(vehicle_state.position_x)
                y_history.append(vehicle_state.position_y)
                
                # Calculate and apply control
                steering, target_velocity = controller.compute_control_input(trajectory, vehicle_state)
                vehicle_model.update_with_direct_control([steering, target_velocity], time_step)
                
                # Plot vehicle path
                plt.plot(x_history, y_history, 'g-', label='Vehicle Path')
                
                # Plot vehicle
                vehicle_display.plot_vehicle(
                    vehicle_state.position_x,
                    vehicle_state.position_y,
                    vehicle_state.yaw_angle,
                    vehicle_state.steering_angle
                )
                
                # Add legend
                plt.legend()
                
                # Set fixed view limits with margin
                plt.xlim(x_min - margin, x_max + margin)
                plt.ylim(y_min - margin, y_max + margin)
                
                plt.pause(0.001)
                time += time_step
            else:
                plt.pause(0.1)  # Reduce CPU usage while paused
            
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
    finally:
        plt.close(fig)  # Ensure figure is closed properly


def main() -> None:
    """
    Main function to run the pure pursuit simulation.
    """
    print("Pure Pursuit Path Tracking Simulation")
    
    # Create test trajectory
    trajectory = create_test_trajectory()
    
    # Create vehicle model and controller
    wheelbase = 2.9
    vehicle_model = VehicleModel(wheelbase=wheelbase)
    controller = PurePursuitController(
        wheelbase=wheelbase,
        min_lookahead=2.0,
        k_gain=0.1,
        max_steering_angle=np.deg2rad(45.0)
    )
    
    # Run simulation
    run_simulation(trajectory, vehicle_model, controller)


if __name__ == "__main__":
    main()
