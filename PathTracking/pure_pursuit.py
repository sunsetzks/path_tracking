"""
Pure Pursuit Path Tracking Controller

This module implements the Pure Pursuit path tracking algorithm with the following features:
- Dynamic lookahead distance based on vehicle speed
- Support for both forward and backward trajectory segments 
- Nearest point search on trajectory
- Physics-based velocity planning with acceleration/deceleration constraints
- Real-time acceleration monitoring and display
- Visualization using vehicle display

The velocity controller now considers:
- Maximum acceleration limits for smooth speed changes
- Maximum deceleration limits for safe braking
- Physics-based stopping distance calculations
- Conservative braking factors for safety margins

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


class VelocityController:
    """
    Velocity controller for path tracking with physics-based velocity planning.
    
    This controller manages target velocity based on:
    - Maximum velocity constraints (forward/backward)
    - Maximum acceleration/deceleration physics
    - Distance to goal for precise stopping at target
    - Trajectory direction (forward/backward)
    - Minimum velocity constraint
    """

    def __init__(
        self,
        max_forward_velocity: float = 5.0,
        max_backward_velocity: float = 2.0,
        max_acceleration: float = 1.0,
        max_deceleration: float = 2.0,
        goal_tolerance: float = 0.5,
        velocity_tolerance: float = 0.1,
        conservative_braking_factor: float = 1.2,
        min_velocity: float = 0.1,
    ) -> None:
        """
        Initialize the velocity controller with physics-based acceleration/deceleration.

        Args:
            max_forward_velocity (float): Maximum forward velocity [m/s]
            max_backward_velocity (float): Maximum backward velocity [m/s]
            max_acceleration (float): Maximum acceleration magnitude [m/s²] (positive value)
            max_deceleration (float): Maximum deceleration magnitude [m/s²] (positive value)
            goal_tolerance (float): Distance tolerance to consider goal reached [m]
            velocity_tolerance (float): Velocity tolerance to consider vehicle stopped [m/s]
            conservative_braking_factor (float): Safety factor for deceleration distance (>1.0 for conservative approach)
            min_velocity (float): Minimum velocity magnitude [m/s] (absolute value)
        """
        self.max_forward_velocity = max_forward_velocity
        self.max_backward_velocity = max_backward_velocity
        self.max_acceleration = abs(max_acceleration)  # Ensure positive value
        self.max_deceleration = abs(max_deceleration)  # Ensure positive value
        self.goal_tolerance = goal_tolerance
        self.velocity_tolerance = velocity_tolerance
        self.conservative_braking_factor = conservative_braking_factor
        self.min_velocity = abs(min_velocity)  # Ensure positive value

    def calculate_stopping_distance(self, current_velocity: float) -> float:
        """
        Calculate the minimum distance required to stop from current velocity.
        
        Uses physics equation: d = v²/(2*a) where v is velocity, a is deceleration
        
        Args:
            current_velocity (float): Current velocity magnitude [m/s]
            
        Returns:
            float: Stopping distance [m]
        """
        velocity_magnitude = abs(current_velocity)
        if velocity_magnitude < self.velocity_tolerance:
            return 0.0
        
        # Physics-based stopping distance: d = v²/(2*a)
        stopping_distance = (velocity_magnitude ** 2) / (2 * self.max_deceleration)
        
        # Apply safety margin
        return stopping_distance * self.conservative_braking_factor

    def calculate_max_velocity_for_distance(self, distance_to_goal: float, is_forward: bool) -> float:
        """
        Calculate maximum velocity that allows stopping within the given distance.
        
        Uses physics equation: v = sqrt(2*a*d) where a is deceleration, d is distance
         
        Args:
            distance_to_goal (float): Distance to goal [m]
            is_forward (bool): Whether motion is forward direction
            
        Returns:
            float: Maximum velocity magnitude [m/s]
        """
        if distance_to_goal <= self.goal_tolerance:
            return 0.0
        
        # Account for goal tolerance in available stopping distance
        available_distance = max(0.0, distance_to_goal - self.goal_tolerance)
        
        # Remove safety margin to get actual usable distance
        usable_distance = available_distance / self.conservative_braking_factor
        
        if usable_distance <= 0.0:
            return 0.0
        
        # Physics-based maximum velocity: v = sqrt(2*a*d)
        max_velocity_for_distance = math.sqrt(2 * self.max_deceleration * usable_distance)
        
        # Apply velocity constraints based on direction
        max_velocity = self.max_forward_velocity if is_forward else self.max_backward_velocity
        return max(min(max_velocity_for_distance, max_velocity), self.min_velocity)

    def is_goal_reached(
        self, vehicle_state: VehicleState, trajectory: Trajectory
    ) -> bool:
        """
        Check if the vehicle has reached the goal (end of trajectory).

        Args:
            vehicle_state (VehicleState): Current vehicle state
            trajectory (Trajectory): Path trajectory

        Returns:
            bool: True if goal is reached, False otherwise
        """
        if len(trajectory.waypoints) == 0:
            return True
        
        # Get the last waypoint (goal position)
        goal_waypoint = trajectory.waypoints[-1]
        
        # Calculate distance to goal
        dx = vehicle_state.position_x - goal_waypoint.x
        dy = vehicle_state.position_y - goal_waypoint.y
        distance_to_goal = math.sqrt(dx * dx + dy * dy)
        
        # Check if within tolerance and velocity is low enough
        position_reached = distance_to_goal <= self.goal_tolerance
        
        return position_reached

    def calculate_distance_to_goal(self, vehicle_state: VehicleState, trajectory: Trajectory) -> float:
        """
        Calculate the distance from the vehicle to the goal (end of trajectory).

        Args:
            vehicle_state (VehicleState): Current vehicle state.
            trajectory (Trajectory): Path trajectory.

        Returns:
            float: Distance to the goal [m].
        """
        if len(trajectory.waypoints) == 0:
            return 0.0

        # Calculate distance to end of trajectory
        trajectory_length = trajectory.get_trajectory_length()
        nearest_point = trajectory.find_nearest_point(vehicle_state.position_x, vehicle_state.position_y)
        distance_to_end = trajectory_length - nearest_point.s

        # Calculate direct distance to goal
        goal_waypoint = trajectory.waypoints[-1]
        dx = vehicle_state.position_x - goal_waypoint.x
        dy = vehicle_state.position_y - goal_waypoint.y
        direct_distance_to_goal = math.sqrt(dx * dx + dy * dy)

        # Use the smaller of the two distances for conservative approach
        return min(distance_to_end, direct_distance_to_goal)

    def compute_target_velocity(
        self, vehicle_state: VehicleState, trajectory: Trajectory, target_direction: float, dt: float = 0.1
    ) -> float:
        """
        Compute target velocity using physics-based acceleration/deceleration planning.

        Args:
            vehicle_state (VehicleState): Current vehicle state.
            trajectory (Trajectory): Path trajectory.
            target_direction (float): Direction of motion (1.0 for forward, -1.0 for backward).
            dt (float): Time step for acceleration calculation [s].

        Returns:
            float: Target velocity [m/s] (positive for forward, negative for backward).
        """
        if self.is_goal_reached(vehicle_state, trajectory):
            return 0.0  # Stop the vehicle when goal is reached

        # Calculate distance to goal using the new method
        distance_to_goal = self.calculate_distance_to_goal(vehicle_state, trajectory)

        # Calculate maximum velocity that allows stopping at goal
        is_forward = target_direction > 0
        max_velocity_for_stopping = self.calculate_max_velocity_for_distance(distance_to_goal, is_forward)

        # Rest of the logic remains the same...
        max_velocity = self.max_forward_velocity if is_forward else self.max_backward_velocity
        desired_velocity_magnitude = max(min(max_velocity, max_velocity_for_stopping), self.min_velocity)
        desired_velocity = desired_velocity_magnitude * target_direction

        # Apply acceleration constraints
        current_velocity = vehicle_state.velocity
        velocity_difference = desired_velocity - current_velocity

        if velocity_difference > 0:
            max_velocity_change = self.max_acceleration * dt
        else:
            max_velocity_change = self.max_deceleration * dt

        if abs(velocity_difference) > max_velocity_change:
            if velocity_difference > 0:
                target_velocity = current_velocity + max_velocity_change
            else:
                target_velocity = current_velocity - max_velocity_change
        else:
            target_velocity = desired_velocity

        return target_velocity * target_direction
    
    def calculate_current_acceleration(
        self, current_velocity: float, target_velocity: float, dt: float = 0.1
    ) -> float:
        """
        Calculate the current acceleration based on velocity change.
        
        Args:
            current_velocity (float): Current velocity [m/s]
            target_velocity (float): Target velocity [m/s] 
            dt (float): Time step [s]
            
        Returns:
            float: Current acceleration [m/s²]
        """
        if dt <= 0:
            return 0.0
        
        acceleration = (target_velocity - current_velocity) / dt
        return acceleration
    
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
        k_gain: float = 10.0,
        max_steering_angle: float = np.deg2rad(45.0),
        velocity_controller: Optional[VelocityController] = None,
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
            velocity_controller (Optional[VelocityController]): Velocity controller instance. If None, creates default one
            goal_tolerance (float): Distance tolerance to consider goal reached [m] (used if no velocity_controller provided)
            velocity_tolerance (float): Velocity tolerance to consider vehicle stopped [m/s] (used if no velocity_controller provided)
        """
        self.wheelbase = wheelbase
        self.trajectory = trajectory
        self.min_lookahead = min_lookahead
        self.k_gain = k_gain
        self.max_steering_angle = max_steering_angle
        
        # Create velocity controller if not provided
        if velocity_controller is None:
            self.velocity_controller = VelocityController(
            )
        else:
            self.velocity_controller = velocity_controller
            
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

    def compute_control(self, vehicle_state: VehicleState, dt: float = 0.1) -> Tuple[float, float]:
        """
        Compute control input using the internal trajectory.
        
        This is a convenience method that uses the internally stored trajectory.
        
        Args:
            vehicle_state (VehicleState): Current vehicle state
            dt (float): Time step for acceleration calculation [s]

        Returns:
            tuple: (steering_angle, target_velocity) - control inputs
                  Returns (0, 0) if no trajectory is set, no target point is found, or goal is reached
        """
        if self.trajectory is None:
            return 0.0, 0.0
        
        return self.compute_control_input(vehicle_state, dt)

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

    def is_goal_reached(self, vehicle_state: VehicleState) -> bool:
        """
        Check if the vehicle has reached the goal (end of trajectory).

        Args:
            vehicle_state (VehicleState): Current vehicle state

        Returns:
            bool: True if goal is reached, False otherwise
        """
        if self.trajectory is None:
            raise ValueError("No trajectory set")
        
        # Use velocity controller's goal checking
        goal_reached = self.velocity_controller.is_goal_reached(vehicle_state, self.trajectory)
        
        # Update internal goal reached state and print message
        if goal_reached and not self.goal_reached:
            goal_waypoint = self.trajectory.waypoints[-1]
            dx = vehicle_state.position_x - goal_waypoint.x
            dy = vehicle_state.position_y - goal_waypoint.y
            distance_to_goal = math.sqrt(dx * dx + dy * dy)
            print(f"🎯 Goal reached! Distance to goal: {distance_to_goal:.2f}m, Velocity: {abs(vehicle_state.velocity):.2f}m/s")
            self.goal_reached = True
            
        return goal_reached

    def reset_goal_state(self) -> None:
        """
        Reset the goal reached state.
        
        This method can be called when restarting trajectory tracking
        or when switching to a new trajectory.
        """
        self.goal_reached = False

    def find_target_point(self, vehicle_state: VehicleState) -> Optional[Tuple[float, float, float]]:
        """
        Find target point on trajectory using current position and lookahead distance.

        Args:
            vehicle_state (VehicleState): Current vehicle state

        Returns:
            tuple: (target_x, target_y, target_direction) - target point coordinates and direction
                  Returns None if no target point is found
        """
        if self.trajectory is None:
            raise ValueError("No trajectory set")
        
        # Get current position and velocity
        current_x = vehicle_state.position_x
        current_y = vehicle_state.position_y
        current_velocity = vehicle_state.velocity

        # Calculate lookahead distance based on speed
        lookahead = self.calculate_lookahead_distance(current_velocity)

        # Find nearest point on trajectory
        nearest_point = self.trajectory.find_nearest_point(current_x, current_y)

        # Get trajectory length
        trajectory_length = self.trajectory.get_trajectory_length()

        # If we're near the end of the trajectory, use the last point
        if nearest_point.s + lookahead >= trajectory_length:
            point = self.trajectory.interpolate_at_distance(trajectory_length)
            return point.x, point.y, point.direction

        # Search for target point at lookahead distance
        target_s = nearest_point.s + lookahead

        # Get interpolated point
        point = self.trajectory.interpolate_at_distance(target_s)
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

    def compute_control_input(self, vehicle_state: VehicleState, dt: float = 0.1) -> Tuple[float, float]:
        """
        Compute control input (steering angle and velocity) for the vehicle.

        Args:
            vehicle_state (VehicleState): Current vehicle state
            dt (float): Time step for acceleration calculation [s]

        Returns:
            tuple: (steering_angle, target_velocity) - control inputs
                  Returns (0, 0) if no target point is found or goal is reached
        """
        if self.trajectory is None:
            raise ValueError("No trajectory set")
        
        # Check if goal is reached first
        if self.is_goal_reached(vehicle_state):
            return 0.0, 0.0  # Stop the vehicle when goal is reached

        # Find target point
        target_point = self.find_target_point(vehicle_state)

        if target_point is None:
            return 0.0, 0.0

        target_x, target_y, target_direction = target_point

        # Compute steering angle
        steering_angle = self.compute_steering_angle(vehicle_state, target_x, target_y)

        # Compute target velocity using velocity controller with time step
        target_velocity = self.velocity_controller.compute_target_velocity(
            vehicle_state, self.trajectory, target_direction, dt
        )

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
    input("Press Enter to start forward simulation...")

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
    input("Press Enter to start reverse simulation...")

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
    margin = 7.0  # Add margin to view (increased by 2m)

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
                plt.gca().set_aspect('equal', adjustable='box')

                # Plot trajectory
                plt.plot(x_coords, y_coords, "b--", label="Reference Path")

                # Plot goal position with circle
                goal_waypoint = trajectory.waypoints[-1]
                goal_circle = patches.Circle(
                    (goal_waypoint.x, goal_waypoint.y), 
                    controller.velocity_controller.goal_tolerance, 
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

                # Find and plot lookahead point
                target_point = controller.find_target_point(vehicle_state)
                if target_point is not None:
                    target_x, target_y, target_direction = target_point
                    plt.plot(target_x, target_y, 'mx', markersize=12, alpha=0.7, markeredgewidth=3, label="Lookahead Point")
                    
                    # Draw line from vehicle to lookahead point
                    plt.plot([vehicle_state.position_x, target_x], 
                            [vehicle_state.position_y, target_y], 
                            'm--', linewidth=1, alpha=0.7, label="Lookahead Line")

                # Calculate and apply control with time step for acceleration planning
                steering, target_velocity = controller.compute_control(vehicle_state, time_step)
                vehicle_model.update_with_direct_control(
                    [steering, target_velocity], time_step
                )

                # Plot vehicle path
                plt.plot(x_history, y_history, "g-", label="Vehicle Path")

                # Calculate physics-based information for display
                goal_waypoint = trajectory.waypoints[-1]
                dx_goal = vehicle_state.position_x - goal_waypoint.x
                dy_goal = vehicle_state.position_y - goal_waypoint.y
                distance_to_goal = math.sqrt(dx_goal * dx_goal + dy_goal * dy_goal)
                
                stopping_distance = controller.velocity_controller.calculate_stopping_distance(vehicle_state.velocity)
                max_vel_for_distance = controller.velocity_controller.calculate_max_velocity_for_distance(distance_to_goal, is_forward=True)
                
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
                    plt.gca().add_patch(stopping_circle)
                
                # Add status text with physics information
                status_text = f"Time: {time:.1f}s\n"
                status_text += f"Speed: {abs(vehicle_state.velocity):.2f} m/s\n"
                status_text += f"Target Speed: {abs(target_velocity):.2f} m/s\n"
                status_text += f"Acceleration: {current_acceleration:.2f} m/s²\n"
                status_text += f"Lookahead Distance: {lookahead_distance:.2f} m\n"
                status_text += f"Distance to Goal: {distance_to_goal:.2f} m\n"
                status_text += f"Stopping Distance: {stopping_distance:.2f} m\n"
                status_text += f"Max Vel for Distance: {max_vel_for_distance:.2f} m/s\n"
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

                # Set view limits with margin, compatible with equal aspect
                x_range = x_max - x_min
                y_range = y_max - y_min
                max_range = max(x_range, y_range)
                
                # Calculate center points
                x_center = (x_min + x_max) / 2
                y_center = (y_min + y_max) / 2
                
                # Set symmetric limits for equal aspect ratio
                half_range = max_range / 2 + margin
                plt.xlim(x_center - half_range, x_center + half_range)
                plt.ylim(y_center - half_range, y_center + half_range)

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


def main() -> None:
    """
    Main function to run both forward and reverse pure pursuit simulations.
    """
    print("Pure Pursuit Path Tracking Simulation with Forward and Reverse Driving")
    print("=" * 70)
    
    # Run acceleration demo first
    demo_acceleration_planning()
    
    print("\n" + "=" * 70)
    print("🎯 SIMULATION MENU")
    print("=" * 70)
    print("This demo includes two driving scenarios:")
    print("1. Forward Driving - High speed S-curve trajectory")
    print("2. Reverse Driving - Backing and parking maneuvers")
    print()
    
    while True:
        print("Select simulation to run:")
        print("1. Forward Driving Simulation")
        print("2. Reverse Driving Simulation")
        print("3. Run Both Simulations")
        print("4. Exit")
        
        choice = input("\nEnter your choice (1-4): ").strip()
        
        if choice == "1":
            run_forward_simulation()
        elif choice == "2":
            run_reverse_simulation()
        elif choice == "3":
            run_forward_simulation()
            run_reverse_simulation()
        elif choice == "4":
            print("Exiting simulation. Goodbye!")
            break
        else:
            print("Invalid choice. Please enter 1, 2, 3, or 4.")
        
        # Ask if user wants to run another simulation
        if choice in ["1", "2", "3"]:
            while True:
                continue_choice = input("\nRun another simulation? (y/n): ").strip().lower()
                if continue_choice in ['y', 'yes']:
                    break
                elif continue_choice in ['n', 'no']:
                    print("Exiting simulation. Goodbye!")
                    return
                else:
                    print("Please enter 'y' or 'n'")


if __name__ == "__main__":
    main()
