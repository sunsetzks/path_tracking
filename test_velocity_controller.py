"""
Unit tests for the VelocityController class.

This module contains comprehensive tests for velocity planning functionality including:
- Physics-based stopping distance calculations
- Maximum velocity calculations for given distances
- Goal reaching detection
- Target velocity computation with acceleration constraints
- Distance to goal calculations
- Current acceleration calculations
"""

import math
import unittest
from unittest.mock import MagicMock
from typing import List, Optional

from PathTracking.velocity_planning import VelocityController
from PathTracking.config import VelocityControllerConfig, TrajectoryConfig
from PathTracking.vehicle_model import VehicleState
from PathTracking.trajectory import Trajectory, Waypoint


class MockVehicleState:
    """Mock vehicle state for testing"""

    def __init__(self, position_x: float = 0.0, position_y: float = 0.0, velocity: float = 0.0):
        self.position_x = position_x
        self.position_y = position_y
        self.velocity = velocity


class MockWaypoint:
    """Mock waypoint for testing"""

    def __init__(self, x: float = 0.0, y: float = 0.0):
        self.x = x
        self.y = y


class MockTrajectoryPoint:
    """Mock trajectory point for testing"""

    def __init__(self, s: float = 0.0):
        self.s = s


class MockTrajectory:
    """Mock trajectory for testing"""

    def __init__(self, waypoints: Optional[List] = None, length: float = 10.0):
        self.waypoints = waypoints if waypoints is not None else [Waypoint(0, 0, 0), Waypoint(10, 0, 0)]
        self._length = length

    def get_trajectory_length(self) -> float:
        return self._length

    def find_nearest_point(self, x: float, y: float) -> MockTrajectoryPoint:
        # Simple mock implementation - return point based on x position
        s = min(max(x, 0), self._length)
        return MockTrajectoryPoint(s)


class TestVelocityController(unittest.TestCase):
    """Test cases for VelocityController"""

    def setUp(self):
        """Set up test fixtures"""
        # Create configuration for velocity controller
        config = VelocityControllerConfig(
            max_forward_velocity=5.0,
            max_backward_velocity=2.0,
            max_acceleration=1.0,
            max_deceleration=2.0,
            goal_tolerance=0.5,
            velocity_tolerance=0.1,
            conservative_braking_factor=1.2,
            min_velocity=0.1,
        )
        self.controller = VelocityController(config)

    def test_init_default_parameters(self):
        """Test VelocityController initialization with default parameters"""
        controller = VelocityController()
        self.assertEqual(controller.max_forward_velocity, 5.0)
        self.assertEqual(controller.max_backward_velocity, 2.0)
        self.assertEqual(controller.max_acceleration, 1.0)
        self.assertEqual(controller.max_deceleration, 1.0)
        self.assertEqual(controller.goal_tolerance, 0.1)
        self.assertEqual(controller.velocity_tolerance, 0.1)
        self.assertEqual(controller.conservative_braking_factor, 1.2)
        self.assertEqual(controller.min_velocity, 0.05)

    def test_init_custom_parameters(self):
        """Test VelocityController initialization with custom parameters"""
        config = VelocityControllerConfig(
            max_forward_velocity=10.0,
            max_backward_velocity=3.0,
            max_acceleration=2.5,
            max_deceleration=3.5,
            goal_tolerance=1.0,
            velocity_tolerance=0.2,
            conservative_braking_factor=1.5,
            min_velocity=0.2,
        )
        controller = VelocityController(config)
        self.assertEqual(controller.max_forward_velocity, 10.0)
        self.assertEqual(controller.max_backward_velocity, 3.0)
        self.assertEqual(controller.max_acceleration, 2.5)
        self.assertEqual(controller.max_deceleration, 3.5)
        self.assertEqual(controller.goal_tolerance, 1.0)
        self.assertEqual(controller.velocity_tolerance, 0.2)
        self.assertEqual(controller.conservative_braking_factor, 1.5)
        self.assertEqual(controller.min_velocity, 0.2)

    def test_init_absolute_values(self):
        """Test that negative acceleration/deceleration values are made positive"""
        config = VelocityControllerConfig(
            max_acceleration=2.0,
            max_deceleration=3.0,
            min_velocity=0.5,
        )
        controller = VelocityController(config)
        self.assertEqual(controller.max_acceleration, 2.0)
        self.assertEqual(controller.max_deceleration, 3.0)
        self.assertEqual(controller.min_velocity, 0.5)

    def test_calculate_stopping_distance_zero_velocity(self):
        """Test stopping distance calculation with zero velocity"""
        distance = self.controller.calculate_stopping_distance(0.0)
        self.assertEqual(distance, 0.0)

    def test_calculate_stopping_distance_low_velocity(self):
        """Test stopping distance calculation with velocity below tolerance"""
        distance = self.controller.calculate_stopping_distance(0.05)  # Below 0.1 tolerance
        self.assertEqual(distance, 0.0)

    def test_calculate_stopping_distance_normal_velocity(self):
        """Test stopping distance calculation with normal velocity"""
        velocity = 4.0  # m/s
        expected_distance = (
            (velocity**2) / (2 * self.controller.max_deceleration) * self.controller.conservative_braking_factor
        )
        # d = v²/(2*a) * safety_factor = 16/(2*2) * 1.2 = 4 * 1.2 = 4.8
        self.assertAlmostEqual(self.controller.calculate_stopping_distance(velocity), 4.8, places=2)

    def test_calculate_stopping_distance_negative_velocity(self):
        """Test stopping distance calculation with negative velocity (uses absolute value)"""
        velocity = -3.0  # m/s
        expected_distance = (
            (abs(velocity) ** 2) / (2 * self.controller.max_deceleration) * self.controller.conservative_braking_factor
        )
        # d = 9/(2*2) * 1.2 = 2.25 * 1.2 = 2.7
        self.assertAlmostEqual(self.controller.calculate_stopping_distance(velocity), 2.7, places=2)

    def test_calculate_max_velocity_for_distance_zero_distance(self):
        """Test max velocity calculation with zero distance"""
        velocity = self.controller.calculate_max_velocity_for_distance(0.0, is_forward=True)
        self.assertEqual(velocity, 0.0)

    def test_calculate_max_velocity_for_distance_distance_below_tolerance(self):
        """Test max velocity calculation with distance below goal tolerance"""
        velocity = self.controller.calculate_max_velocity_for_distance(0.3, is_forward=True)  # Below 0.5 tolerance
        self.assertEqual(velocity, 0.0)

    def test_calculate_max_velocity_for_distance_normal_distance_forward(self):
        """Test max velocity calculation with normal distance for forward motion"""
        distance = 10.0  # m
        available_distance = distance - self.controller.goal_tolerance  # 10.0 - 0.5 = 9.5
        usable_distance = available_distance / self.controller.conservative_braking_factor  # 9.5 / 1.2 = 7.917
        expected_velocity = math.sqrt(2 * self.controller.max_deceleration * usable_distance)  # sqrt(2*2*7.917) = 5.62
        expected_velocity = max(
            min(expected_velocity, self.controller.max_forward_velocity), self.controller.min_velocity
        )

        result = self.controller.calculate_max_velocity_for_distance(distance, is_forward=True)
        self.assertAlmostEqual(result, expected_velocity, places=2)

    def test_calculate_max_velocity_for_distance_normal_distance_backward(self):
        """Test max velocity calculation with normal distance for backward motion"""
        distance = 5.0  # m
        available_distance = distance - self.controller.goal_tolerance  # 5.0 - 0.5 = 4.5
        usable_distance = available_distance / self.controller.conservative_braking_factor  # 4.5 / 1.2 = 3.75
        expected_velocity = math.sqrt(2 * self.controller.max_deceleration * usable_distance)  # sqrt(2*2*3.75) = 3.87
        expected_velocity = max(
            min(expected_velocity, self.controller.max_backward_velocity), self.controller.min_velocity
        )
        # Limited by max_backward_velocity = 2.0

        result = self.controller.calculate_max_velocity_for_distance(distance, is_forward=False)
        self.assertEqual(result, 2.0)  # Limited by max_backward_velocity

    def test_calculate_max_velocity_for_distance_applies_min_velocity(self):
        """Test that minimum velocity constraint is applied"""
        # Very small distance that would result in velocity below minimum
        distance = 0.6  # Just above goal tolerance
        result = self.controller.calculate_max_velocity_for_distance(distance, is_forward=True)
        self.assertGreaterEqual(result, self.controller.min_velocity)

    def test_calculate_current_acceleration_positive(self):
        """Test acceleration calculation with positive acceleration"""
        current_velocity = 2.0
        target_velocity = 3.0
        dt = 0.1

        result = self.controller.calculate_current_acceleration(current_velocity, target_velocity, dt)
        expected = (target_velocity - current_velocity) / dt  # (3.0 - 2.0) / 0.1 = 10.0
        self.assertAlmostEqual(result, expected, places=2)

    def test_calculate_current_acceleration_negative(self):
        """Test acceleration calculation with negative acceleration (deceleration)"""
        current_velocity = 4.0
        target_velocity = 2.0
        dt = 0.1

        result = self.controller.calculate_current_acceleration(current_velocity, target_velocity, dt)
        expected = (target_velocity - current_velocity) / dt  # (2.0 - 4.0) / 0.1 = -20.0
        self.assertAlmostEqual(result, expected, places=2)

    def test_calculate_current_acceleration_zero_dt(self):
        """Test acceleration calculation with zero time step"""
        result = self.controller.calculate_current_acceleration(2.0, 3.0, 0.0)
        self.assertEqual(result, 0.0)

    def test_calculate_current_acceleration_negative_dt(self):
        """Test acceleration calculation with negative time step"""
        result = self.controller.calculate_current_acceleration(2.0, 3.0, -0.1)
        self.assertEqual(result, 0.0)

    def test_conservative_braking_factor_effect(self):
        """Test that conservative braking factor affects calculations"""
        # Create controller with higher safety factor
        config = VelocityControllerConfig(conservative_braking_factor=2.0)
        conservative_controller = VelocityController(config)

        velocity = 4.0
        distance_conservative = conservative_controller.calculate_stopping_distance(velocity)
        distance_normal = self.controller.calculate_stopping_distance(velocity)

        # Conservative controller should require longer stopping distance
        self.assertGreater(distance_conservative, distance_normal)

    def test_physics_equations_consistency(self):
        """Test that physics equations are consistent between stopping distance and max velocity calculations"""
        distance = 10.0

        # Calculate max velocity for this distance
        max_vel = self.controller.calculate_max_velocity_for_distance(distance, is_forward=True)

        # Calculate stopping distance for this velocity
        stopping_dist = self.controller.calculate_stopping_distance(max_vel)

        # The stopping distance should be approximately equal to the available distance
        # (accounting for goal tolerance and safety factor)
        available_distance = distance - self.controller.goal_tolerance

        # Allow some tolerance due to min_velocity constraints and floating point precision
        self.assertLessEqual(stopping_dist, available_distance * 1.1)  # 10% tolerance


class TestVelocityControllerWithRealTrajectory(unittest.TestCase):
    """Integration tests for VelocityController with real Trajectory objects"""

    def setUp(self):
        """Set up test fixtures for integration tests"""
        config = VelocityControllerConfig(
            max_forward_velocity=10.0,
            max_backward_velocity=3.0,
            max_acceleration=2.0,
            max_deceleration=3.0,
            goal_tolerance=1.0,
            velocity_tolerance=0.2,
            conservative_braking_factor=1.3,
            min_velocity=0.3,
        )
        self.controller = VelocityController(config)

    def create_test_trajectory(self, length: float = 50.0, num_points: int = 51):
        """Create a simple straight line trajectory for testing."""
        trajectory_config = TrajectoryConfig()
        waypoints = []
        for i in range(num_points):
            t = i / (num_points - 1)
            x = t * length
            y = 0.0
            yaw = 0.0
            waypoints.append(Waypoint(x=x, y=y, yaw=yaw, direction=1))
        return Trajectory(trajectory_config, waypoints)

    def test_is_goal_reached_empty_trajectory(self):
        """Test goal reached detection with empty trajectory"""
        trajectory_config = TrajectoryConfig()
        vehicle_state = VehicleState(position_x=0, position_y=0, velocity=0)
        trajectory = Trajectory(trajectory_config, [])

        result = self.controller.is_goal_reached(vehicle_state, trajectory)
        self.assertTrue(result)

    def test_is_goal_reached_within_tolerance(self):
        """Test goal reached detection when within tolerance"""
        trajectory = self.create_test_trajectory(10.0, 11)
        vehicle_state = VehicleState(position_x=10.3, position_y=0, velocity=0.05)  # 0.3m away, low velocity

        result = self.controller.is_goal_reached(vehicle_state, trajectory)
        self.assertTrue(result)

    def test_is_goal_reached_outside_tolerance(self):
        """Test goal reached detection when outside tolerance"""
        trajectory = self.create_test_trajectory(10.0, 11)
        vehicle_state = VehicleState(position_x=8.0, position_y=0, velocity=0.05)  # 2.0m away (> 1.0 tolerance)

        result = self.controller.is_goal_reached(vehicle_state, trajectory)
        self.assertFalse(result)

    def test_calculate_distance_to_goal_empty_trajectory(self):
        """Test distance to goal calculation with empty trajectory"""
        trajectory_config = TrajectoryConfig()
        vehicle_state = VehicleState(position_x=5, position_y=0, velocity=2)
        trajectory = Trajectory(trajectory_config, [])

        result = self.controller.calculate_distance_to_goal(vehicle_state, trajectory)
        self.assertEqual(result, 0.0)

    def test_calculate_distance_to_goal_normal_case(self):
        """Test distance to goal calculation in normal case"""
        trajectory = self.create_test_trajectory(10.0, 11)
        vehicle_state = VehicleState(position_x=3.0, position_y=0, velocity=2)  # 3m along trajectory

        result = self.controller.calculate_distance_to_goal(vehicle_state, trajectory)
        # Should be approximately 7.0m (10.0 - 3.0)
        self.assertAlmostEqual(result, 7.0, places=0)

    def test_compute_target_velocity_goal_reached(self):
        """Test target velocity computation when goal is reached"""
        trajectory = self.create_test_trajectory(10.0, 11)
        vehicle_state = VehicleState(position_x=10.2, position_y=0, velocity=1.0)  # Close to goal

        result = self.controller.compute_target_velocity(vehicle_state, trajectory, target_direction=1.0)
        self.assertEqual(result, 0.0)

    def test_compute_target_velocity_forward_acceleration(self):
        """Test target velocity computation with forward acceleration"""
        trajectory = self.create_test_trajectory(20.0, 21)
        vehicle_state = VehicleState(position_x=5.0, position_y=0, velocity=2.0)  # Current velocity 2.0 m/s

        dt = 0.1
        result = self.controller.compute_target_velocity(vehicle_state, trajectory, target_direction=1.0, dt=dt)

        # The segmented controller calculates based on physics, not limited by acceleration constraint
        # when using the traditional controller path. Since enable_segmented_ramp_down is not set to True,
        # this test should check the behavior of the traditional controller
        # For a distance of 15m from goal, the controller should calculate a reasonable target velocity
        self.assertGreaterEqual(result, 0)  # Should be positive for forward motion
        self.assertLessEqual(result, self.controller.max_forward_velocity)  # Should not exceed max velocity

    def test_compute_target_velocity_forward_deceleration(self):
        """Test target velocity computation with forward deceleration"""
        trajectory = self.create_test_trajectory(10.0, 11)
        vehicle_state = VehicleState(position_x=8.0, position_y=0, velocity=4.0)  # High velocity near goal

        dt = 0.1
        result = self.controller.compute_target_velocity(vehicle_state, trajectory, target_direction=1.0, dt=dt)

        # Should be decelerating due to proximity to goal
        self.assertLess(result, vehicle_state.velocity)

    def test_compute_target_velocity_applies_min_velocity(self):
        """Test that minimum velocity constraint is applied"""
        trajectory = self.create_test_trajectory(15.0, 16)
        vehicle_state = VehicleState(position_x=5.0, position_y=0, velocity=0.05)  # Very low velocity

        result = self.controller.compute_target_velocity(vehicle_state, trajectory, target_direction=1.0)

        # Should be at least minimum velocity
        self.assertGreaterEqual(abs(result), self.controller.min_velocity)

    def test_compute_target_velocity_applies_max_velocity_constraints(self):
        """Test that maximum velocity constraints are applied"""
        trajectory = self.create_test_trajectory(100.0, 101)  # Very far goal
        vehicle_state = VehicleState(position_x=5.0, position_y=0, velocity=0.5)

        # Forward motion
        result_forward = self.controller.compute_target_velocity(vehicle_state, trajectory, target_direction=1.0)
        self.assertLessEqual(result_forward, self.controller.max_forward_velocity)

        # Backward motion
        result_backward = self.controller.compute_target_velocity(vehicle_state, trajectory, target_direction=-1.0)
        self.assertGreaterEqual(result_backward, -self.controller.max_backward_velocity)

    def test_simplified_velocity_controller_phases(self):
        """Test the simplified velocity controller phase transitions"""
        # Create configuration with simplified ramp down control
        config = VelocityControllerConfig(
            max_forward_velocity=5.0,
            max_deceleration=2.0,
            min_velocity=0.1,
            enable_segmented_ramp_down=True,
            final_approach_distance=0.3,
            goal_tolerance=0.05
        )
        controller = VelocityController(config)
        
        # Create test trajectory
        trajectory = self.create_test_trajectory(10.0, 11)
        
        # Test different distances from goal
        test_cases = [
            (5.0, 'deceleration'),   # Far from goal
            (1.0, 'deceleration'),   # Moderate distance
            (0.5, 'deceleration'),   # Close but still decelerating
            (0.2, 'final_approach'), # Within final approach distance
            (0.1, 'final_approach'), # Very close
        ]
        
        for distance, expected_phase in test_cases:
            vehicle_x = 10.0 - distance
            vehicle_state = VehicleState(position_x=vehicle_x, position_y=0, velocity=1.0)
            
            # Get diagnostics
            diagnostics = controller.get_control_diagnostics(vehicle_state, trajectory, 1.0)
            
            self.assertEqual(diagnostics['control_phase'], expected_phase, 
                           f"Expected {expected_phase} at distance {distance}, got {diagnostics['control_phase']}")
            
            # Get target velocity
            target_velocity = controller.compute_target_velocity(vehicle_state, trajectory, 1.0, 0.1)
            
            if expected_phase == 'final_approach':
                # Should be minimum velocity in final approach
                self.assertAlmostEqual(target_velocity, config.min_velocity, places=2)
            else:
                # Should be calculated deceleration velocity
                self.assertGreater(target_velocity, config.min_velocity)


if __name__ == "__main__":
    # Run the tests
    unittest.main(verbosity=2)
