"""
Velocity planning controller for path tracking with physics-based velocity planning.

This module contains the VelocityController class that manages target velocity based on:
- Maximum velocity constraints (forward/backward)
- Maximum acceleration/deceleration physics
- Distance to goal for precise stopping at target
- Trajectory direction (forward/backward)
- Minimum velocity constraint
"""

import math
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from PathTracking.trajectory import Trajectory
    from PathTracking.vehicle_model import VehicleState
    from ..config import VelocityControllerConfig

# Import configuration management
try:
    from ..config import get_velocity_controller_config
except ImportError:
    # Fallback for standalone usage
    get_velocity_controller_config = None


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
        max_forward_velocity: Optional[float] = None,
        max_backward_velocity: Optional[float] = None,
        max_acceleration: Optional[float] = None,
        max_deceleration: Optional[float] = None,
        goal_tolerance: Optional[float] = None,
        velocity_tolerance: Optional[float] = None,
        conservative_braking_factor: Optional[float] = None,
        min_velocity: Optional[float] = None,
        config: Optional['VelocityControllerConfig'] = None,
    ) -> None:
        """
        Initialize the velocity controller with physics-based acceleration/deceleration.

        Args:
            max_forward_velocity (float, optional): Maximum forward velocity [m/s]. If None, uses config default
            max_backward_velocity (float, optional): Maximum backward velocity [m/s]. If None, uses config default
            max_acceleration (float, optional): Maximum acceleration magnitude [m/s²] (positive value). If None, uses config default
            max_deceleration (float, optional): Maximum deceleration magnitude [m/s²] (positive value). If None, uses config default
            goal_tolerance (float, optional): Distance tolerance to consider goal reached [m]. If None, uses config default
            velocity_tolerance (float, optional): Velocity tolerance to consider vehicle stopped [m/s]. If None, uses config default
            conservative_braking_factor (float, optional): Safety factor for deceleration distance (>1.0 for conservative approach). If None, uses config default
            min_velocity (float, optional): Minimum velocity magnitude [m/s] (absolute value). If None, uses config default
            config (VelocityControllerConfig, optional): Configuration object. If None, uses global config
        """
        # Get configuration
        if config is None and get_velocity_controller_config is not None:
            config = get_velocity_controller_config()
        
        # Set parameters with fallback to defaults
        if config is not None:
            self.max_forward_velocity = max_forward_velocity if max_forward_velocity is not None else config.max_forward_velocity
            self.max_backward_velocity = max_backward_velocity if max_backward_velocity is not None else config.max_backward_velocity
            self.max_acceleration = abs(max_acceleration) if max_acceleration is not None else abs(config.max_acceleration)
            self.max_deceleration = abs(max_deceleration) if max_deceleration is not None else abs(config.max_deceleration)
            self.goal_tolerance = goal_tolerance if goal_tolerance is not None else config.goal_tolerance
            self.velocity_tolerance = velocity_tolerance if velocity_tolerance is not None else config.velocity_tolerance
            self.conservative_braking_factor = conservative_braking_factor if conservative_braking_factor is not None else config.conservative_braking_factor
            self.min_velocity = abs(min_velocity) if min_velocity is not None else abs(config.min_velocity)
        else:
            # Fallback defaults for standalone usage
            self.max_forward_velocity = max_forward_velocity if max_forward_velocity is not None else 5.0
            self.max_backward_velocity = max_backward_velocity if max_backward_velocity is not None else 2.0
            self.max_acceleration = abs(max_acceleration) if max_acceleration is not None else 1.0
            self.max_deceleration = abs(max_deceleration) if max_deceleration is not None else 2.0
            self.goal_tolerance = goal_tolerance if goal_tolerance is not None else 0.5
            self.velocity_tolerance = velocity_tolerance if velocity_tolerance is not None else 0.1
            self.conservative_braking_factor = conservative_braking_factor if conservative_braking_factor is not None else 1.2
            self.min_velocity = abs(min_velocity) if min_velocity is not None else 0.1

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
        self, vehicle_state: "VehicleState", trajectory: "Trajectory"
    ) -> bool:
        """
        Check if the vehicle has reached the goal (end of trajectory).
        The goal is considered reached if either:
        1. The vehicle is within goal_tolerance distance of the goal point, or
        2. The vehicle's longitudinal position s equals the goal's s on the trajectory

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
        
        # Check if within distance tolerance
        position_reached = distance_to_goal <= self.goal_tolerance
        
        # Check if current longitudinal position s equals goal's s
        nearest_point = trajectory.find_nearest_point(vehicle_state.position_x, vehicle_state.position_y)
        trajectory_length = trajectory.get_trajectory_length()
        s_reached = abs(nearest_point.s - trajectory_length) <= self.goal_tolerance
        
        return position_reached or s_reached

    def calculate_distance_to_goal(self, vehicle_state: "VehicleState", trajectory: "Trajectory") -> float:
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
        self, vehicle_state: "VehicleState", trajectory: "Trajectory", target_direction: float, dt: float = 0.1
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

        return target_velocity
    
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