"""
Pure Pursuit Path Tracking Controller

This module implements a Pure Pursuit controller for path tracking with dynamic lookahead distance.
The controller supports both forward and reverse driving with integrated velocity control.

Key Features:
- Dynamic lookahead distance based on vehicle speed
- Direction conflict detection and resolution
- Integrated velocity planning and acceleration control
- Goal reaching detection with detailed error reporting
- Support for both forward and reverse trajectories

Classes:
    PurePursuitController: Main controller class for pure pursuit path tracking

For examples and simulations, see pure_pursuit_examples.py
"""

import math
import os
import sys
from typing import Optional, Tuple

import numpy as np

# Add the parent directory to the path so we can import the modules
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PathTracking.trajectory import Trajectory
from PathTracking.velocity_planning import VelocityController
from PathTracking.vehicle_model import VehicleState



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

    @classmethod
    def create_high_precision_controller(
        cls,
        wheelbase: float,
        trajectory: Optional[Trajectory] = None,
        precision_target: float = 0.01,  # 1cm precision
    ) -> 'PurePursuitController':
        """
        Create a high-precision Pure Pursuit controller optimized for sub-centimeter accuracy.
        
        This factory method creates a controller with parameters specifically tuned for 
        high-precision positioning, including:
        - Ultra-low goal tolerance (1cm by default)
        - Small minimum lookahead distance for precise maneuvering
        - Conservative velocity limits and acceleration
        - High-resolution trajectory following
        
        Args:
            wheelbase (float): Vehicle wheelbase [m]
            trajectory (Optional[Trajectory]): Reference trajectory to follow
            precision_target (float): Target precision in meters (default: 0.01m = 1cm)
            
        Returns:
            PurePursuitController: High-precision controller instance
        """
        # High-precision velocity controller with very tight tolerances
        high_precision_velocity_controller = VelocityController(
            max_forward_velocity=1.0,  # Slower for precision
            max_backward_velocity=0.5,  # Even slower in reverse
            max_acceleration=0.5,  # Gentle acceleration
            max_deceleration=1.0,  # Controlled deceleration
            goal_tolerance=precision_target,  # 1cm tolerance
            velocity_tolerance=0.02,  # Very low velocity tolerance (2cm/s)
            conservative_braking_factor=2.0,  # Extra conservative braking
            min_velocity=0.05,  # Very slow minimum velocity (5cm/s)
        )
        
        # Create controller with precision-optimized parameters
        controller = cls(
            wheelbase=wheelbase,
            trajectory=trajectory,
            min_lookahead=0.2,  # Small lookahead for tight maneuvering
            k_gain=3.0,  # Reduced gain for more stable tracking
            max_steering_angle=np.deg2rad(30.0),  # Reduced max steering for smoothness
            velocity_controller=high_precision_velocity_controller,
        )
        
        return controller

    @classmethod
    def create_ultra_precision_controller(
        cls,
        wheelbase: float,
        trajectory: Optional[Trajectory] = None,
        precision_target: float = 0.005,  # 5mm precision
    ) -> 'PurePursuitController':
        """
        Create an ultra-precision Pure Pursuit controller for millimeter-level accuracy.
        
        This factory method creates a controller optimized for the highest possible
        positioning accuracy, suitable for applications requiring sub-centimeter precision.
        
        Args:
            wheelbase (float): Vehicle wheelbase [m]
            trajectory (Optional[Trajectory]): Reference trajectory to follow
            precision_target (float): Target precision in meters (default: 0.005m = 5mm)
            
        Returns:
            PurePursuitController: Ultra-precision controller instance
        """
        # Ultra-precision velocity controller with extreme conservative settings
        ultra_precision_velocity_controller = VelocityController(
            max_forward_velocity=0.3,  # Very slow for maximum precision
            max_backward_velocity=0.2,  # Ultra-slow in reverse
            max_acceleration=0.2,  # Very gentle acceleration
            max_deceleration=0.5,  # Gentle deceleration
            goal_tolerance=precision_target,  # 5mm tolerance
            velocity_tolerance=0.01,  # Ultra-low velocity tolerance (1cm/s)
            conservative_braking_factor=3.0,  # Maximum conservative braking
            min_velocity=0.02,  # Ultra-slow minimum velocity (2cm/s)
        )
        
        # Create controller with ultra-precision parameters
        controller = cls(
            wheelbase=wheelbase,
            trajectory=trajectory,
            min_lookahead=0.1,  # Very small lookahead
            k_gain=1.5,  # Very low gain for maximum stability
            max_steering_angle=np.deg2rad(15.0),  # Very limited steering for smoothness
            velocity_controller=ultra_precision_velocity_controller,
        )
        
        return controller

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

    def is_in_precision_zone(self, vehicle_state: VehicleState, precision_radius: float = 0.2) -> bool:
        """
        Check if vehicle is within precision zone near the goal.
        
        Args:
            vehicle_state (VehicleState): Current vehicle state
            precision_radius (float): Distance threshold for precision zone [m]
            
        Returns:
            bool: True if within precision zone, False otherwise
        """
        if self.trajectory is None:
            return False
            
        goal_waypoint = self.trajectory.waypoints[-1]
        dx = vehicle_state.position_x - goal_waypoint.x
        dy = vehicle_state.position_y - goal_waypoint.y
        distance_to_goal = math.sqrt(dx * dx + dy * dy)
        
        # Adaptive precision zone based on speed and approach quality
        # Slower vehicles can enter precision mode earlier
        speed_factor = max(0.5, min(1.5, abs(vehicle_state.velocity) / 0.5))
        
        # Better aligned vehicles can enter precision mode earlier  
        goal_direction = math.atan2(dy, dx)
        heading_error = abs(goal_direction - vehicle_state.yaw_angle)
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
        alignment_factor = max(0.7, 1.0 - abs(heading_error) / math.pi)
        
        # Adaptive radius: larger when moving fast or poorly aligned
        adaptive_radius = precision_radius * speed_factor / alignment_factor
        
        return distance_to_goal <= adaptive_radius

    def compute_precision_control(self, vehicle_state: VehicleState, dt: float = 0.1) -> Tuple[float, float]:
        """
        Compute high-precision control when near the goal.
        
        This method uses a different control strategy when the vehicle is very close to the goal,
        prioritizing accuracy over speed and using direct positioning control.
        
        Args:
            vehicle_state (VehicleState): Current vehicle state
            dt (float): Time step [s]
            
        Returns:
            tuple: (steering_angle, target_velocity) - precision control inputs
        """
        if self.trajectory is None:
            raise ValueError("No trajectory set")
            
        goal_waypoint = self.trajectory.waypoints[-1]
        
        # Calculate position errors in goal frame
        longitudinal_error, lateral_error, angle_error = self.calculate_goal_errors(vehicle_state, goal_waypoint)
        
        # Calculate total distance error
        distance_error = math.sqrt(longitudinal_error**2 + lateral_error**2)
        
        # If already at high precision, stop
        if distance_error <= self.velocity_controller.goal_tolerance:
            return 0.0, 0.0
        
        # Enhanced direct positioning control with adaptive parameters
        dx = goal_waypoint.x - vehicle_state.position_x
        dy = goal_waypoint.y - vehicle_state.position_y
        
        # Calculate desired heading to goal
        desired_heading = math.atan2(dy, dx)
        heading_error = desired_heading - vehicle_state.yaw_angle
        
        # Normalize heading error
        while heading_error > math.pi:
            heading_error -= 2 * math.pi
        while heading_error < -math.pi:
            heading_error += 2 * math.pi
        
        # Conservative precision steering control
        steering_gain = 2.0  # Moderate gain for stability
        steering_angle = np.clip(
            steering_gain * heading_error,
            -self.max_steering_angle,
            self.max_steering_angle
        )
        
        # Very slow approach velocity based on distance and alignment
        max_approach_velocity = 0.05  # Ultra-slow 5cm/s maximum
        distance_factor = min(distance_error / 0.05, 1.0)  # Scale with distance (5cm reference)
        angle_factor = max(0.3, 1.0 - abs(heading_error) / math.pi)  # Slower if not aligned
        
        target_velocity = max_approach_velocity * distance_factor * angle_factor
        
        # Ensure minimum movement if not at goal
        if distance_error > self.velocity_controller.goal_tolerance:
            target_velocity = max(target_velocity, 0.01)  # Minimum 1cm/s
        
        return steering_angle, target_velocity

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

        # Check if we should use precision control mode
        if self.is_in_precision_zone(vehicle_state):
            return self.compute_precision_control(vehicle_state, dt)

        # Standard Pure Pursuit control for normal operation
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








