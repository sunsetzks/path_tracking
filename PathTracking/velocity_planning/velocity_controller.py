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
from loguru import logger

if TYPE_CHECKING:
    from PathTracking.trajectory import Trajectory
    from PathTracking.vehicle_model import VehicleState
    from ..config import VelocityControllerConfig


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
        config: Optional["VelocityControllerConfig"] = None,
    ) -> None:
        """
        Initialize the velocity controller with physics-based acceleration/deceleration.

        Args:
            config (VelocityControllerConfig, optional): Configuration object. If None, creates a default config.
        """
        if config is None:
            # This allows creating a default controller when no config is passed.
            # In a real application, it's recommended to always pass a config.
            from ..config import VelocityControllerConfig

            config = VelocityControllerConfig()

        self.config = config

        self.max_forward_velocity = self.config.max_forward_velocity
        self.max_backward_velocity = self.config.max_backward_velocity
        self.max_acceleration = abs(self.config.max_acceleration)
        self.max_deceleration = abs(self.config.max_deceleration)
        self.goal_tolerance = self.config.goal_tolerance
        self.velocity_tolerance = self.config.velocity_tolerance
        self.conservative_braking_factor = self.config.conservative_braking_factor
        self.min_velocity = abs(self.config.min_velocity)
        
        # Segmented ramp down control parameters
        self.enable_segmented_ramp_down = self.config.enable_segmented_ramp_down
        self.fine_adjustment_distance = self.config.fine_adjustment_distance
        self.transition_zone_distance = self.config.transition_zone_distance
        self.creep_speed_factor = self.config.creep_speed_factor
        self.final_braking_distance = self.config.final_braking_distance
        self.smooth_transition_enabled = self.config.smooth_transition_enabled

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
        stopping_distance = (velocity_magnitude**2) / (2 * self.max_deceleration)

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

    def is_goal_reached(self, vehicle_state: "VehicleState", trajectory: "Trajectory") -> bool:
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

    def calculate_creep_speed(self, is_forward: bool) -> float:
        """
        Calculate the creep speed for fine adjustment phase.
        
        The creep speed is calculated as a safe speed that allows stopping within
        the fine adjustment distance, with a safety factor applied.
        
        Args:
            is_forward (bool): Whether motion is forward direction
            
        Returns:
            float: Creep speed magnitude [m/s]
        """
        # Calculate theoretical safe speed for fine adjustment distance
        # Using physics: v = sqrt(2*a*d)
        theoretical_speed = math.sqrt(2 * self.max_deceleration * self.fine_adjustment_distance)
        
        # Apply safety factor
        creep_speed = theoretical_speed * self.creep_speed_factor
        
        # Ensure creep speed is within velocity limits
        max_velocity = self.max_forward_velocity if is_forward else self.max_backward_velocity
        creep_speed = min(creep_speed, max_velocity)
        
        # Ensure creep speed is not below minimum velocity
        creep_speed = max(creep_speed, self.min_velocity)
        
        return creep_speed

    def get_control_phase(self, distance_to_goal: float) -> str:
        """
        Determine the current control phase based on distance to goal.
        
        Args:
            distance_to_goal (float): Distance to goal [m]
            
        Returns:
            str: Control phase ('normal', 'transition', 'fine_adjustment', 'final_braking')
        """
        if distance_to_goal <= self.final_braking_distance:
            return 'final_braking'
        elif distance_to_goal <= self.fine_adjustment_distance:
            return 'fine_adjustment'
        elif distance_to_goal <= (self.fine_adjustment_distance + self.transition_zone_distance):
            return 'transition'
        else:
            return 'normal'

    def calculate_transition_factor(self, distance_to_goal: float) -> float:
        """
        Calculate smooth transition factor between normal and fine adjustment phases.
        
        Args:
            distance_to_goal (float): Distance to goal [m]
            
        Returns:
            float: Transition factor (0.0 to 1.0)
                  0.0 = fully in fine adjustment phase
                  1.0 = fully in normal phase
        """
        if not self.smooth_transition_enabled:
            return 1.0 if distance_to_goal > self.fine_adjustment_distance else 0.0
        
        # Calculate transition factor using smooth S-curve
        transition_start = self.fine_adjustment_distance
        transition_end = self.fine_adjustment_distance + self.transition_zone_distance
        
        if distance_to_goal <= transition_start:
            return 0.0
        elif distance_to_goal >= transition_end:
            return 1.0
        else:
            # Smooth transition using cosine interpolation
            normalized_distance = (distance_to_goal - transition_start) / self.transition_zone_distance
            return 0.5 * (1 + math.cos(math.pi * (1 - normalized_distance)))

    def compute_segmented_target_velocity(
        self, 
        vehicle_state: "VehicleState", 
        trajectory: "Trajectory", 
        target_direction: float, 
        dt: float = 0.1
    ) -> float:
        """
        Compute target velocity using segmented ramp down control strategy.
        
        This method implements a three-phase control strategy:
        1. Normal phase: Standard velocity control with deceleration planning
        2. Transition phase: Smooth transition between normal and fine adjustment
        3. Fine adjustment phase: Low-speed creep for precise positioning
        4. Final braking phase: Final deceleration to complete stop
        
        Args:
            vehicle_state (VehicleState): Current vehicle state
            trajectory (Trajectory): Path trajectory
            target_direction (float): Direction of motion (1.0 for forward, -1.0 for backward)
            dt (float): Time step for acceleration calculation [s]
            
        Returns:
            float: Target velocity [m/s] (positive for forward, negative for backward)
        """
        # Check if goal is reached
        if self.is_goal_reached(vehicle_state, trajectory):
            return 0.0
        
        # Calculate distance to goal
        distance_to_goal = self.calculate_distance_to_goal(vehicle_state, trajectory)
        
        # Determine motion direction
        is_forward = target_direction > 0
        
        # Get current control phase
        control_phase = self.get_control_phase(distance_to_goal)
        current_velocity = vehicle_state.velocity
        
        if control_phase == 'final_braking':
            # Final braking phase: Aggressive deceleration to stop
            desired_velocity = 0.0
            
        elif control_phase == 'fine_adjustment':
            # Fine adjustment phase: Maintain creep speed for precise positioning
            creep_speed = self.calculate_creep_speed(is_forward)
            desired_velocity = creep_speed * target_direction
            
        elif control_phase == 'transition':
            # Transition phase: Smooth blend between normal and fine adjustment
            transition_factor = self.calculate_transition_factor(distance_to_goal)
            
            # Calculate normal phase velocity
            max_velocity_for_stopping = self.calculate_max_velocity_for_distance(distance_to_goal, is_forward)
            max_velocity = self.max_forward_velocity if is_forward else self.max_backward_velocity
            normal_velocity_magnitude = max(min(max_velocity, max_velocity_for_stopping), self.min_velocity)
            
            # Calculate fine adjustment phase velocity
            creep_speed = self.calculate_creep_speed(is_forward)
            
            # Blend velocities
            blended_velocity_magnitude = (
                transition_factor * normal_velocity_magnitude + 
                (1 - transition_factor) * creep_speed
            )
            desired_velocity = blended_velocity_magnitude * target_direction
            
        else:  # normal phase
            # Normal phase: Standard velocity control with deceleration planning
            max_velocity_for_stopping = self.calculate_max_velocity_for_distance(distance_to_goal, is_forward)
            max_velocity = self.max_forward_velocity if is_forward else self.max_backward_velocity
            desired_velocity_magnitude = max(min(max_velocity, max_velocity_for_stopping), self.min_velocity)
            desired_velocity = desired_velocity_magnitude * target_direction
        
        # Apply acceleration/deceleration constraints
        velocity_difference = desired_velocity - current_velocity
        
        # Determine maximum allowed velocity change
        if velocity_difference > 0:
            max_velocity_change = self.max_acceleration * dt
        else:
            max_velocity_change = self.max_deceleration * dt
        
        # Apply velocity change constraints
        if abs(velocity_difference) > max_velocity_change:
            if velocity_difference > 0:
                target_velocity = current_velocity + max_velocity_change
            else:
                target_velocity = current_velocity - max_velocity_change
        else:
            target_velocity = desired_velocity
        
        return target_velocity

    def calculate_current_acceleration(self, current_velocity: float, target_velocity: float, dt: float = 0.1) -> float:
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

    def compute_target_velocity(
        self, vehicle_state: "VehicleState", trajectory: "Trajectory", target_direction: float, dt: float = 0.1
    ) -> float:
        """
        Compute target velocity using physics-based acceleration/deceleration planning.
        
        This method will use segmented ramp down control if enabled, otherwise
        it will use the traditional single-phase control strategy.

        Args:
            vehicle_state (VehicleState): Current vehicle state.
            trajectory (Trajectory): Path trajectory.
            target_direction (float): Direction of motion (1.0 for forward, -1.0 for backward).
            dt (float): Time step for acceleration calculation [s].

        Returns:
            float: Target velocity [m/s] (positive for forward, negative for backward).
        """
        # Use segmented control if enabled
        if self.enable_segmented_ramp_down:
            return self.compute_segmented_target_velocity(vehicle_state, trajectory, target_direction, dt)
        
        # Traditional single-phase control
        return self._compute_traditional_target_velocity(vehicle_state, trajectory, target_direction, dt)

    def _compute_traditional_target_velocity(
        self, vehicle_state: "VehicleState", trajectory: "Trajectory", target_direction: float, dt: float = 0.1
    ) -> float:
        """
        Compute target velocity using traditional single-phase control strategy.
        
        This is the original implementation for backward compatibility.

        Args:
            vehicle_state (VehicleState): Current vehicle state.
            trajectory (Trajectory): Path trajectory.
            target_direction (float): Direction of motion (1.0 for forward, -1.0 for backward).
            dt (float): Time step for acceleration calculation [s].

        Returns:
            float: Target velocity [m/s] (positive for forward, negative for backward).
        """
        # Step 1: Check if goal is reached - return 0 velocity if true
        if self.is_goal_reached(vehicle_state, trajectory):
            return 0.0  # Stop the vehicle when goal is reached

        # Step 2: Calculate distance remaining to the goal position
        distance_to_goal = self.calculate_distance_to_goal(vehicle_state, trajectory)

        # Step 3: Determine if we're moving forward or backward
        is_forward = target_direction > 0

        # Step 4: Calculate maximum safe velocity that allows stopping at goal
        max_velocity_for_stopping = self.calculate_max_velocity_for_distance(distance_to_goal, is_forward)

        # Step 5: Get configured max velocity for current direction
        max_velocity = self.max_forward_velocity if is_forward else self.max_backward_velocity

        # Step 6: Calculate desired velocity within safe limits
        desired_velocity_magnitude = max(min(max_velocity, max_velocity_for_stopping), self.min_velocity)
        desired_velocity = desired_velocity_magnitude * target_direction

        # Step 7: Apply acceleration/deceleration constraints
        current_velocity = vehicle_state.velocity
        velocity_difference = desired_velocity - current_velocity

        # Step 8: Determine maximum allowed velocity change based on acceleration/deceleration limits
        if velocity_difference > 0:
            max_velocity_change = self.max_acceleration * dt  # Acceleration limit
        else:
            max_velocity_change = self.max_deceleration * dt  # Deceleration limit

        # Step 9: Apply velocity change with constraints
        if abs(velocity_difference) > max_velocity_change:
            if velocity_difference > 0:
                target_velocity = current_velocity + max_velocity_change  # Accelerate
            else:
                target_velocity = current_velocity - max_velocity_change  # Decelerate
        else:
            target_velocity = desired_velocity  # Within allowed change limits

        return target_velocity

    def get_control_diagnostics(
        self, vehicle_state: "VehicleState", trajectory: "Trajectory", target_direction: float
    ) -> dict:
        """
        Get diagnostic information about the current control state.
        
        This method provides detailed information about the segmented control
        strategy for debugging and monitoring purposes.
        
        Args:
            vehicle_state (VehicleState): Current vehicle state
            trajectory (Trajectory): Path trajectory
            target_direction (float): Direction of motion
            
        Returns:
            dict: Diagnostic information containing:
                - control_phase: Current control phase
                - distance_to_goal: Distance to goal [m]
                - creep_speed: Calculated creep speed [m/s]
                - transition_factor: Transition factor (0.0 to 1.0)
                - is_forward: Direction of motion
                - segmented_control_enabled: Whether segmented control is enabled
        """
        distance_to_goal = self.calculate_distance_to_goal(vehicle_state, trajectory)
        is_forward = target_direction > 0
        
        diagnostics = {
            'segmented_control_enabled': self.enable_segmented_ramp_down,
            'distance_to_goal': distance_to_goal,
            'is_forward': is_forward,
            'current_velocity': vehicle_state.velocity,
            'fine_adjustment_distance': self.fine_adjustment_distance,
            'final_braking_distance': self.final_braking_distance,
            'transition_zone_distance': self.transition_zone_distance,
        }
        
        if self.enable_segmented_ramp_down:
            control_phase = self.get_control_phase(distance_to_goal)
            creep_speed = self.calculate_creep_speed(is_forward)
            transition_factor = self.calculate_transition_factor(distance_to_goal)
            
            diagnostics.update({
                'control_phase': control_phase,
                'creep_speed': creep_speed,
                'transition_factor': transition_factor,
                'theoretical_creep_speed': math.sqrt(2 * self.max_deceleration * self.fine_adjustment_distance),
                'creep_speed_factor': self.creep_speed_factor,
            })
        else:
            diagnostics.update({
                'control_phase': 'traditional',
                'creep_speed': None,
                'transition_factor': None,
            })
        
        return diagnostics

    def print_control_status(
        self, vehicle_state: "VehicleState", trajectory: "Trajectory", target_direction: float
    ) -> None:
        """
        Print current control status for debugging purposes.
        
        Args:
            vehicle_state (VehicleState): Current vehicle state
            trajectory (Trajectory): Path trajectory
            target_direction (float): Direction of motion
        """
        diagnostics = self.get_control_diagnostics(vehicle_state, trajectory, target_direction)
        
        logger.info("=== Velocity Control Status ===")
        logger.info(f"Segmented Control: {'Enabled' if diagnostics['segmented_control_enabled'] else 'Disabled'}")
        logger.info(f"Control Phase: {diagnostics['control_phase']}")
        logger.info(f"Distance to Goal: {diagnostics['distance_to_goal']:.3f} m")
        logger.info(f"Current Velocity: {diagnostics['current_velocity']:.3f} m/s")
        logger.info(f"Direction: {'Forward' if diagnostics['is_forward'] else 'Backward'}")
        
        if diagnostics['segmented_control_enabled']:
            logger.info(f"Fine Adjustment Distance: {diagnostics['fine_adjustment_distance']:.3f} m")
            logger.info(f"Final Braking Distance: {diagnostics['final_braking_distance']:.3f} m")
            logger.info(f"Creep Speed: {diagnostics['creep_speed']:.3f} m/s")
            logger.info(f"Transition Factor: {diagnostics['transition_factor']:.3f}")
        
        logger.info("=" * 30)
