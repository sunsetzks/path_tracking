"""
"""

import argparse
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
from PathTracking.velocity_planning import VelocityController
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

    def calculate_goal_errors(self, vehicle_state: VehicleState, goal_waypoint) -> Tuple[float, float, float]:
        """
        Calculate errors in the goal's coordinate frame.

        Args:
            vehicle_state (VehicleState): Current vehicle state
            goal_waypoint: Goal waypoint

        Returns:
            Tuple[float, float, float]: (longitudinal_error, lateral_error, angle_error)
                longitudinal_error: Error along the goal direction (positive = ahead of goal)
                lateral_error: Error perpendicular to goal direction (positive = left of goal)
                angle_error: Angular error in radians
        """
        # Calculate position errors
        dx = vehicle_state.position_x - goal_waypoint.x
        dy = vehicle_state.position_y - goal_waypoint.y
        
        # Calculate longitudinal and lateral errors relative to goal orientation
        cos_goal_yaw = math.cos(goal_waypoint.yaw)
        sin_goal_yaw = math.sin(goal_waypoint.yaw)
        
        # Transform position error to goal frame
        # Longitudinal error: along the goal direction (positive = ahead of goal)
        longitudinal_error = dx * cos_goal_yaw + dy * sin_goal_yaw
        
        # Lateral error: perpendicular to goal direction (positive = left of goal)
        lateral_error = -dx * sin_goal_yaw + dy * cos_goal_yaw
        
        # Calculate angular error
        angle_error = vehicle_state.yaw_angle - goal_waypoint.yaw
        
        # Normalize angle error to [-pi, pi]
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
            
        return longitudinal_error, lateral_error, angle_error

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
        
        # Update internal goal reached state and print message with detailed errors
        if goal_reached and not self.goal_reached:
            goal_waypoint = self.trajectory.waypoints[-1]
            
            # Calculate position errors in goal frame
            longitudinal_error, lateral_error, angle_error = self.calculate_goal_errors(vehicle_state, goal_waypoint)
            
            # Calculate total distance error
            distance_to_goal = math.sqrt(longitudinal_error**2 + lateral_error**2)
            
            angle_error_deg = math.degrees(angle_error)
            
            print(f"🎯 Goal reached! Final position errors:")
            print(f"   Total distance error: {distance_to_goal:.3f}m")
            print(f"   Longitudinal error: {longitudinal_error:.3f}m ({'ahead' if longitudinal_error > 0 else 'behind'} goal)")
            print(f"   Lateral error: {lateral_error:.3f}m ({'left' if lateral_error > 0 else 'right'} of goal)")
            print(f"   Angular error: {angle_error_deg:.2f}° ({'counterclockwise' if angle_error > 0 else 'clockwise'} from goal)")
            print(f"   Final velocity: {abs(vehicle_state.velocity):.2f}m/s")
            
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

        # Determine actual driving direction based on robot orientation and target position
        actual_direction = self.determine_driving_direction(vehicle_state, target_x, target_y, target_direction)

        # Compute steering angle
        steering_angle = self.compute_steering_angle(vehicle_state, target_x, target_y)

        # Compute target velocity using velocity controller with time step
        target_velocity = self.velocity_controller.compute_target_velocity(
            vehicle_state, self.trajectory, actual_direction, dt
        )

        return steering_angle, target_velocity

    def determine_driving_direction(
        self, vehicle_state: VehicleState, target_x: float, target_y: float, path_direction: float
    ) -> float:
        """
        Determine the driving direction based on robot orientation and target point position.
        
        This method considers the vehicle's current heading and the relative position of the target point
        to determine whether the vehicle should move forward or backward. It issues warnings when
        the determined direction conflicts with the path's intended direction.

        Args:
            vehicle_state (VehicleState): Current vehicle state
            target_x (float): Target point x-coordinate
            target_y (float): Target point y-coordinate  
            path_direction (float): Intended direction from path (1.0 for forward, -1.0 for backward)

        Returns:
            float: Actual driving direction (1.0 for forward, -1.0 for backward)
        """
        # Calculate vector from vehicle to target point
        dx = target_x - vehicle_state.position_x
        dy = target_y - vehicle_state.position_y
        
        # Handle case where target is at current position
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            # Use path direction when target is at current position
            return path_direction
        
        # Calculate angle from vehicle to target point in global frame
        target_angle = math.atan2(dy, dx)
        
        # Calculate angle difference between vehicle heading and target direction
        angle_diff = target_angle - vehicle_state.yaw_angle
        
        # Normalize angle difference to [-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        # Determine driving direction based on angle difference
        # If target is within [-pi/2, pi/2] relative to vehicle heading: forward
        # If target is outside this range: backward
        if abs(angle_diff) <= math.pi / 2:
            robot_based_direction = 1.0  # Forward
        else:
            robot_based_direction = -1.0  # Backward
        
        # Check for conflict between path direction and robot-based direction
        if abs(path_direction - robot_based_direction) > 0.1:  # Threshold for floating point comparison
            path_dir_str = "forward" if path_direction > 0 else "backward"
            robot_dir_str = "forward" if robot_based_direction > 0 else "backward"
            
            print(f"⚠️  WARNING: Direction conflict detected!")
            print(f"   Path direction: {path_dir_str} ({path_direction:.1f})")
            print(f"   Robot-based direction: {robot_dir_str} ({robot_based_direction:.1f})")
            print(f"   Target point: ({target_x:.2f}, {target_y:.2f})")
            print(f"   Vehicle position: ({vehicle_state.position_x:.2f}, {vehicle_state.position_y:.2f})")
            print(f"   Vehicle heading: {math.degrees(vehicle_state.yaw_angle):.1f}°")
            print(f"   Angle to target: {math.degrees(target_angle):.1f}°")
            print(f"   Angle difference: {math.degrees(angle_diff):.1f}°")
            print(f"   Using robot-based direction: {robot_dir_str}")
        
        return robot_based_direction


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
                
                # Add status text with physics and direction information
                status_text = f"Time: {time:.1f}s\n"
                status_text += f"Velocity: {vehicle_state.velocity:.2f} m/s ({velocity_dir})\n"
                status_text += f"Target Velocity: {target_velocity:.2f} m/s ({target_dir})\n"
                status_text += f"Acceleration: {current_acceleration:.2f} m/s²\n"
                status_text += f"Lookahead Distance: {lookahead_distance:.2f} m\n"
                status_text += f"Distance to Goal: {distance_to_goal:.2f} m\n"
                status_text += f"Stopping Distance: {stopping_distance:.2f} m\n"
                status_text += f"Max Vel for Distance: {max_vel_for_distance:.2f} m/s\n"
                status_text += f"Path Direction: {path_direction_str}\n"
                status_text += f"Robot Direction: {robot_direction_str}\n"
                status_text += f"Direction Match: {'YES' if direction_match else 'NO ⚠️'}\n"
                status_text += f"Goal Reached: {'YES' if goal_reached else 'NO'}\n"
                status_text += f"--- Goal Errors ---\n"
                status_text += f"Longitudinal: {longitudinal_error:.3f}m\n"
                status_text += f"Lateral: {lateral_error:.3f}m\n"
                status_text += f"Angular: {angle_error_deg:.1f}°"
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
