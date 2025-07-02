"""
Pure Pursuit Path Tracking Controller

This module implements the Pure Pursuit path tracking algorithm with the following features:
- Dynamic lookahead distance based on vehicle speed
- Support for both forward and backward trajectory segments 
- Nearest point search on trajectory
- Visualization using vehicle display

Author: Assistant
"""

import math
import os
import sys
from typing import List, Optional, Tuple

import matplotlib.patches as patches  # Add import for Circle
import matplotlib.pyplot as plt
import numpy as np

# Add the parent directory to the path so we can import the modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PathTracking.trajectory import Trajectory
from PathTracking.utils.vehicle_display import VehicleDisplay
from PathTracking.vehicle_model import VehicleModel, VehicleState


class PurePursuitController:
    """
    Pure Pursuit path tracking controller with dynamic lookahead distance.

    This controller finds the nearest point on the trajectory and then looks ahead
    by a distance proportional to the vehicle's speed to find a target point.
    The steering angle is calculated to guide the vehicle toward this target point.
    """

    def __init__(
        self,
        wheelbase: float,
        trajectory: Optional[Trajectory] = None,
        min_lookahead: float = 1.0,
        k_gain: float = 1.0,
        max_steering_angle: float = np.deg2rad(45.0),
        max_velocity: float = 5.0,
        goal_tolerance: float = 0.5,
        velocity_tolerance: float = 0.1,
    ) -> None:
        """
        Initialize the Pure Pursuit controller.

        Args:
            wheelbase (float): Vehicle wheelbase [m]
            trajectory (Optional[Trajectory]): Reference trajectory to follow
            min_lookahead (float): Minimum lookahead distance [m]
            k_gain (float): Lookahead distance gain (lookahead = k_gain * velocity + min_lookahead)
            max_steering_angle (float): Maximum steering angle [rad]
            max_velocity (float): Maximum velocity magnitude [m/s]
            goal_tolerance (float): Distance tolerance to consider goal reached [m]
            velocity_tolerance (float): Velocity tolerance to consider vehicle stopped [m/s]
        """
        self.wheelbase = wheelbase
        self.trajectory = trajectory
        self.min_lookahead = min_lookahead
        self.k_gain = k_gain
        self.max_steering_angle = max_steering_angle
        self.max_velocity = max_velocity
        self.goal_tolerance = goal_tolerance
        self.velocity_tolerance = velocity_tolerance
        self.goal_reached = False

    def set_trajectory(self, trajectory: Trajectory) -> None:
        """
        Set the reference trajectory for the controller.

        Args:
            trajectory (Trajectory): Reference trajectory to follow
        """
        self.trajectory = trajectory
        self.reset_goal_state()  # Reset goal state when setting new trajectory

    def get_trajectory(self) -> Optional[Trajectory]:
        """
        Get the current reference trajectory.

        Returns:
            Optional[Trajectory]: Current trajectory or None if not set
        """
        return self.trajectory

    def has_trajectory(self) -> bool:
        """
        Check if a trajectory is currently set.

        Returns:
            bool: True if trajectory is set, False otherwise
        """
        return self.trajectory is not None

    def compute_control(self, vehicle_state: VehicleState) -> Tuple[float, float]:
        """
        Compute control input using the internal trajectory.
        
        This is a convenience method that uses the internally stored trajectory.
        
        Args:
            vehicle_state (VehicleState): Current vehicle state

        Returns:
            tuple: (steering_angle, target_velocity) - control inputs
                  Returns (0, 0) if no trajectory is set, no target point is found, or goal is reached
        """
        if self.trajectory is None:
            return 0.0, 0.0
        
        return self.compute_control_input(vehicle_state, self.trajectory)

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

    def is_goal_reached(
        self, vehicle_state: VehicleState, trajectory: Optional[Trajectory] = None
    ) -> bool:
        """
        Check if the vehicle has reached the goal (end of trajectory).

        Args:
            vehicle_state (VehicleState): Current vehicle state
            trajectory (Optional[Trajectory]): Path trajectory. If None, uses internal trajectory

        Returns:
            bool: True if goal is reached, False otherwise
        """
        # Use internal trajectory if none provided
        traj = trajectory if trajectory is not None else self.trajectory
        
        if traj is None:
            raise ValueError("No trajectory provided and no internal trajectory set")
        
        if len(traj.waypoints) == 0:
            return True
        
        # Get the last waypoint (goal position)
        goal_waypoint = traj.waypoints[-1]
        
        # Calculate distance to goal
        dx = vehicle_state.position_x - goal_waypoint.x
        dy = vehicle_state.position_y - goal_waypoint.y
        distance_to_goal = math.sqrt(dx * dx + dy * dy)
        
        # Check if within tolerance and velocity is low enough
        position_reached = distance_to_goal <= self.goal_tolerance
        velocity_low = abs(vehicle_state.velocity) <= self.velocity_tolerance
        
        # Update goal reached state
        if position_reached and velocity_low:
            if not self.goal_reached:  # Only print once when goal is first reached
                print(f"🎯 Goal reached! Distance to goal: {distance_to_goal:.2f}m, Velocity: {abs(vehicle_state.velocity):.2f}m/s")
            self.goal_reached = True
            
        return self.goal_reached

    def reset_goal_state(self) -> None:
        """
        Reset the goal reached state.
        
        This method can be called when restarting trajectory tracking
        or when switching to a new trajectory.
        """
        self.goal_reached = False

    def find_target_point(
        self, vehicle_state: VehicleState, trajectory: Optional[Trajectory] = None
    ) -> Optional[Tuple[float, float, float]]:
        """
        Find target point on trajectory using current position and lookahead distance.

        Args:
            vehicle_state (VehicleState): Current vehicle state
            trajectory (Optional[Trajectory]): Path trajectory. If None, uses internal trajectory

        Returns:
            tuple: (target_x, target_y, target_direction) - target point coordinates and direction
                  Returns None if no target point is found
        """
        # Use internal trajectory if none provided
        traj = trajectory if trajectory is not None else self.trajectory
        
        if traj is None:
            raise ValueError("No trajectory provided and no internal trajectory set")
        
        # Get current position and velocity
        current_x = vehicle_state.position_x
        current_y = vehicle_state.position_y
        current_velocity = vehicle_state.velocity

        # Calculate lookahead distance based on speed
        lookahead = self.calculate_lookahead_distance(current_velocity)

        # Find nearest point on trajectory
        nearest_point = traj.find_nearest_point(current_x, current_y)

        # Get trajectory length
        trajectory_length = traj.get_trajectory_length()

        # If we're near the end of the trajectory, use the last point
        if nearest_point.s + lookahead >= trajectory_length:
            point = traj.interpolate_at_distance(trajectory_length)
            return point.x, point.y, point.direction

        # Search for target point at lookahead distance
        target_s = nearest_point.s + lookahead

        # Get interpolated point
        point = traj.interpolate_at_distance(target_s)
        return point.x, point.y, point.direction

    def compute_steering_angle(
        self, vehicle_state: VehicleState, target_x: float, target_y: float
    ) -> float:
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
            steering_angle = direction_factor * math.atan2(
                2.0 * self.wheelbase * target_y_veh, target_x_veh**2 + target_y_veh**2
            )

        # Limit steering angle
        steering_angle = np.clip(
            steering_angle, -self.max_steering_angle, self.max_steering_angle
        )

        return steering_angle

    def compute_control_input(
        self, vehicle_state: VehicleState, trajectory: Optional[Trajectory] = None
    ) -> Tuple[float, float]:
        """
        Compute control input (steering angle and velocity) for the vehicle.

        Args:
            vehicle_state (VehicleState): Current vehicle state
            trajectory (Optional[Trajectory]): Path trajectory. If None, uses internal trajectory

        Returns:
            tuple: (steering_angle, target_velocity) - control inputs
                  Returns (0, 0) if no target point is found or goal is reached
        """
        # Use internal trajectory if none provided
        traj = trajectory if trajectory is not None else self.trajectory
        
        if traj is None:
            raise ValueError("No trajectory provided and no internal trajectory set")
        
        # Check if goal is reached first
        if self.is_goal_reached(vehicle_state, traj):
            return 0.0, 0.0  # Stop the vehicle when goal is reached

        # Find target point
        target_point = self.find_target_point(vehicle_state, traj)

        if target_point is None:
            return 0.0, 0.0

        target_x, target_y, target_direction = target_point

        # Compute steering angle
        steering_angle = self.compute_steering_angle(vehicle_state, target_x, target_y)

        # Check if we're very close to the end of trajectory - reduce speed for smooth stopping
        trajectory_length = traj.get_trajectory_length()
        nearest_point = traj.find_nearest_point(vehicle_state.position_x, vehicle_state.position_y)
        distance_to_end = trajectory_length - nearest_point.s
        
        # Set velocity based on trajectory direction and distance to goal
        velocity_magnitude = self.max_velocity  # Use max velocity from controller settings
        
        # Reduce velocity when approaching the goal for smoother stopping
        if distance_to_end < 3.0 * self.goal_tolerance:  # Start slowing down early
            velocity_magnitude *= max(0.2, distance_to_end / (3.0 * self.goal_tolerance))
        
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


def run_simulation(
    trajectory: Trajectory,
    vehicle_model: VehicleModel,
    controller: PurePursuitController,
    time_step: float = 0.1,
    max_time: float = 60.0,
) -> None:
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
    vehicle_model.set_state(
        VehicleState(
            position_x=nearest_point.x,
            position_y=nearest_point.y,
            yaw_angle=nearest_point.yaw,
            velocity=0.0,
            steering_angle=0.0,
        )
    )

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
                # Clear previous plot
                plt.cla()

                # Setup plot properties
                plt.grid(True)
                plt.axis("equal")

                # Plot trajectory
                plt.plot(x_coords, y_coords, "b--", label="Reference Path")

                # Plot goal position with circle
                goal_waypoint = trajectory.waypoints[-1]
                goal_circle = patches.Circle(
                    (goal_waypoint.x, goal_waypoint.y), 
                    controller.goal_tolerance, 
                    fill=False, 
                    edgecolor='red', 
                    linewidth=2
                )
                plt.gca().add_patch(goal_circle)
                plt.plot(goal_waypoint.x, goal_waypoint.y, 'ro', markersize=8, label="Goal")

                # Update vehicle state and history
                vehicle_state = vehicle_model.get_state()
                x_history.append(vehicle_state.position_x)
                y_history.append(vehicle_state.position_y)

                # Check if goal is reached
                goal_reached = controller.is_goal_reached(vehicle_state)

                # Calculate and apply control
                steering, target_velocity = controller.compute_control(vehicle_state)
                vehicle_model.update_with_direct_control(
                    [steering, target_velocity], time_step
                )

                # Plot vehicle path
                plt.plot(x_history, y_history, "g-", label="Vehicle Path")

                # Plot vehicle
                vehicle_display.plot_vehicle(
                    vehicle_state.position_x,
                    vehicle_state.position_y,
                    vehicle_state.yaw_angle,
                    vehicle_state.steering_angle,
                )

                # Add status text
                status_text = f"Time: {time:.1f}s\n"
                status_text += f"Speed: {abs(vehicle_state.velocity):.2f} m/s\n"
                status_text += f"Goal Reached: {'YES' if goal_reached else 'NO'}"
                if goal_reached:
                    status_text += "\nVehicle STOPPED at goal!"
                
                plt.text(
                    x_min - margin + 1, 
                    y_max + margin - 1, 
                    status_text, 
                    fontsize=10, 
                    bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.7)
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
        trajectory=trajectory,  # Set the trajectory during initialization
        min_lookahead=2.0,
        k_gain=0.1,
        max_steering_angle=np.deg2rad(45.0),
        max_velocity=5.0,  # Maximum velocity: 5.0 m/s
        goal_tolerance=1.0,  # Goal tolerance: 1.0 meter
        velocity_tolerance=0.2,  # Stop when velocity < 0.2 m/s
    )

    # Run simulation
    run_simulation(trajectory, vehicle_model, controller)


if __name__ == "__main__":
    main()
