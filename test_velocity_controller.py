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

from PathTracking.velocity_planning import VelocityController


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

    def __init__(self, waypoints: list = None, length: float = 10.0):
        self.waypoints = waypoints if waypoints is not None else [MockWaypoint(0, 0), MockWaypoint(10, 0)]
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
        # Default velocity controller with standard parameters
        self.controller = VelocityController(
            max_forward_velocity=5.0,
            max_backward_velocity=2.0,
            max_acceleration=1.0,
            max_deceleration=2.0,
            goal_tolerance=0.5,
            velocity_tolerance=0.1,
            conservative_braking_factor=1.2,
            min_velocity=0.1,
        )

    def test_init_default_parameters(self):
        """Test VelocityController initialization with default parameters"""
        controller = VelocityController()
        self.assertEqual(controller.max_forward_velocity, 5.0)
        self.assertEqual(controller.max_backward_velocity, 2.0)
        self.assertEqual(controller.max_acceleration, 1.0)
        self.assertEqual(controller.max_deceleration, 2.0)
        self.assertEqual(controller.goal_tolerance, 0.5)
        self.assertEqual(controller.velocity_tolerance, 0.1)
        self.assertEqual(controller.conservative_braking_factor, 1.2)
        self.assertEqual(controller.min_velocity, 0.1)

    def test_init_custom_parameters(self):
        """Test VelocityController initialization with custom parameters"""
        controller = VelocityController(
            max_forward_velocity=10.0,
            max_backward_velocity=3.0,
            max_acceleration=2.5,
            max_deceleration=3.5,
            goal_tolerance=1.0,
            velocity_tolerance=0.2,
            conservative_braking_factor=1.5,
            min_velocity=0.2,
        )
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
        controller = VelocityController(
            max_acceleration=-2.0,  # Should become 2.0
            max_deceleration=-3.0,  # Should become 3.0
            min_velocity=-0.5,  # Should become 0.5
        )
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

    def test_is_goal_reached_empty_trajectory(self):
        """Test goal reached detection with empty trajectory"""
        vehicle_state = MockVehicleState(0, 0, 0)
        trajectory = MockTrajectory(waypoints=[])

        result = self.controller.is_goal_reached(vehicle_state, trajectory)
        self.assertTrue(result)

    def test_is_goal_reached_within_tolerance(self):
        """Test goal reached detection when within tolerance"""
        goal_waypoint = MockWaypoint(10, 0)
        vehicle_state = MockVehicleState(10.3, 0, 0.05)  # 0.3m away, low velocity
        trajectory = MockTrajectory(waypoints=[MockWaypoint(0, 0), goal_waypoint])

        result = self.controller.is_goal_reached(vehicle_state, trajectory)
        self.assertTrue(result)

    def test_is_goal_reached_outside_tolerance(self):
        """Test goal reached detection when outside tolerance"""
        goal_waypoint = MockWaypoint(10, 0)
        vehicle_state = MockVehicleState(9, 0, 0.05)  # 1.0m away (> 0.5 tolerance)
        trajectory = MockTrajectory(waypoints=[MockWaypoint(0, 0), goal_waypoint])

        result = self.controller.is_goal_reached(vehicle_state, trajectory)
        self.assertFalse(result)

    def test_calculate_distance_to_goal_empty_trajectory(self):
        """Test distance to goal calculation with empty trajectory"""
        vehicle_state = MockVehicleState(5, 0, 2)
        trajectory = MockTrajectory(waypoints=[])

        result = self.controller.calculate_distance_to_goal(vehicle_state, trajectory)
        self.assertEqual(result, 0.0)

    def test_calculate_distance_to_goal_normal_case(self):
        """Test distance to goal calculation in normal case"""
        goal_waypoint = MockWaypoint(10, 0)
        vehicle_state = MockVehicleState(3, 0, 2)  # 3m along trajectory
        trajectory = MockTrajectory(waypoints=[MockWaypoint(0, 0), goal_waypoint], length=10.0)

        # Distance along trajectory: 10.0 - 3.0 = 7.0
        # Direct distance: sqrt((10-3)^2 + (0-0)^2) = 7.0
        # Should return minimum: 7.0
        result = self.controller.calculate_distance_to_goal(vehicle_state, trajectory)
        self.assertAlmostEqual(result, 7.0, places=1)

    def test_compute_target_velocity_goal_reached(self):
        """Test target velocity computation when goal is reached"""
        goal_waypoint = MockWaypoint(10, 0)
        vehicle_state = MockVehicleState(10.2, 0, 1.0)  # Close to goal
        trajectory = MockTrajectory(waypoints=[MockWaypoint(0, 0), goal_waypoint])

        result = self.controller.compute_target_velocity(vehicle_state, trajectory, target_direction=1.0)
        self.assertEqual(result, 0.0)

    def test_compute_target_velocity_forward_acceleration(self):
        """Test target velocity computation with forward acceleration"""
        goal_waypoint = MockWaypoint(20, 0)
        vehicle_state = MockVehicleState(5, 0, 2.0)  # Current velocity 2.0 m/s
        trajectory = MockTrajectory(waypoints=[MockWaypoint(0, 0), goal_waypoint], length=20.0)

        dt = 0.1
        result = self.controller.compute_target_velocity(vehicle_state, trajectory, target_direction=1.0, dt=dt)

        # Should be limited by acceleration constraint
        max_velocity_change = self.controller.max_acceleration * dt  # 1.0 * 0.1 = 0.1
        expected_max = vehicle_state.velocity + max_velocity_change  # 2.0 + 0.1 = 2.1
        self.assertLessEqual(result, expected_max)

    def test_compute_target_velocity_forward_deceleration(self):
        """Test target velocity computation with forward deceleration"""
        goal_waypoint = MockWaypoint(10, 0)
        vehicle_state = MockVehicleState(8, 0, 4.0)  # High velocity near goal
        trajectory = MockTrajectory(waypoints=[MockWaypoint(0, 0), goal_waypoint], length=10.0)

        dt = 0.1
        result = self.controller.compute_target_velocity(vehicle_state, trajectory, target_direction=1.0, dt=dt)

        # Should be decelerating due to proximity to goal
        self.assertLess(result, vehicle_state.velocity)

    def test_compute_target_velocity_backward_motion(self):
        """Test target velocity computation for backward motion"""
        goal_waypoint = MockWaypoint(0, 0)
        vehicle_state = MockVehicleState(5, 0, -1.0)  # Moving backward
        trajectory = MockTrajectory(waypoints=[MockWaypoint(10, 0), goal_waypoint], length=10.0)

        dt = 0.1
        result = self.controller.compute_target_velocity(vehicle_state, trajectory, target_direction=-1.0, dt=dt)

        # Result should be negative (backward motion)
        self.assertLess(result, 0)

    def test_compute_target_velocity_applies_min_velocity(self):
        """Test that minimum velocity constraint is applied"""
        goal_waypoint = MockWaypoint(15, 0)
        vehicle_state = MockVehicleState(5, 0, 0.05)  # Very low velocity
        trajectory = MockTrajectory(waypoints=[MockWaypoint(0, 0), goal_waypoint], length=15.0)

        result = self.controller.compute_target_velocity(vehicle_state, trajectory, target_direction=1.0)

        # Should be at least minimum velocity
        self.assertGreaterEqual(abs(result), self.controller.min_velocity)

    def test_compute_target_velocity_applies_max_velocity_constraints(self):
        """Test that maximum velocity constraints are applied"""
        goal_waypoint = MockWaypoint(100, 0)  # Very far goal
        vehicle_state = MockVehicleState(5, 0, 0.5)
        trajectory = MockTrajectory(waypoints=[MockWaypoint(0, 0), goal_waypoint], length=100.0)

        # Forward motion
        result_forward = self.controller.compute_target_velocity(vehicle_state, trajectory, target_direction=1.0)
        self.assertLessEqual(result_forward, self.controller.max_forward_velocity)

        # Backward motion
        result_backward = self.controller.compute_target_velocity(vehicle_state, trajectory, target_direction=-1.0)
        self.assertGreaterEqual(result_backward, -self.controller.max_backward_velocity)

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

    def test_edge_case_very_small_distance_to_goal(self):
        """Test edge case with very small distance to goal"""
        goal_waypoint = MockWaypoint(1.0, 0)
        vehicle_state = MockVehicleState(0.9, 0, 2.0)  # Very close to goal
        trajectory = MockTrajectory(waypoints=[MockWaypoint(0, 0), goal_waypoint], length=1.0)

        result = self.controller.compute_target_velocity(vehicle_state, trajectory, target_direction=1.0)

        # Should be decelerating significantly
        self.assertLess(result, vehicle_state.velocity)

    def test_conservative_braking_factor_effect(self):
        """Test that conservative braking factor affects calculations"""
        # Create controller with higher safety factor
        conservative_controller = VelocityController(conservative_braking_factor=2.0)

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


class TestVelocityControllerIntegration(unittest.TestCase):
    """Integration tests for VelocityController with realistic scenarios"""

    def setUp(self):
        """Set up test fixtures for integration tests"""
        self.controller = VelocityController(
            max_forward_velocity=10.0,
            max_backward_velocity=3.0,
            max_acceleration=2.0,
            max_deceleration=3.0,
            goal_tolerance=1.0,
            velocity_tolerance=0.2,
            conservative_braking_factor=1.3,
            min_velocity=0.3,
        )

    def test_forward_driving_scenario(self):
        """Test realistic forward driving scenario"""
        # Vehicle starting at origin, goal at 50m
        goal_waypoint = MockWaypoint(50, 0)
        trajectory = MockTrajectory(waypoints=[MockWaypoint(0, 0), goal_waypoint], length=50.0)

        # Simulate vehicle progression
        positions = [0, 10, 20, 30, 40, 45, 48, 49.5]
        velocities = [0, 2, 5, 8, 10, 8, 5, 2]

        for pos, vel in zip(positions, velocities):
            vehicle_state = MockVehicleState(pos, 0, vel)
            target_vel = self.controller.compute_target_velocity(vehicle_state, trajectory, target_direction=1.0)

            # Ensure reasonable velocity planning
            self.assertGreaterEqual(target_vel, 0)  # Forward motion
            self.assertLessEqual(target_vel, self.controller.max_forward_velocity)

            # Near the end, velocity should be decreasing
            if pos > 45:
                self.assertLessEqual(target_vel, vel * 1.1)  # Allow small increase due to min_velocity

    def test_reverse_parking_scenario(self):
        """Test realistic reverse parking scenario"""
        # Vehicle backing into parking spot
        goal_waypoint = MockWaypoint(0, 0)
        trajectory = MockTrajectory(waypoints=[MockWaypoint(20, 0), goal_waypoint], length=20.0)

        # Simulate reverse motion
        positions = [20, 15, 10, 5, 2, 1, 0.5]
        velocities = [0, -1, -2, -2.5, -1.5, -0.8, -0.3]

        for pos, vel in zip(positions, velocities):
            vehicle_state = MockVehicleState(pos, 0, vel)
            target_vel = self.controller.compute_target_velocity(vehicle_state, trajectory, target_direction=-1.0)

            # Ensure reasonable velocity planning for reverse
            self.assertLessEqual(target_vel, 0)  # Backward motion
            self.assertGreaterEqual(target_vel, -self.controller.max_backward_velocity)

    def test_emergency_braking_scenario(self):
        """Test emergency braking scenario when approaching goal too fast"""
        goal_waypoint = MockWaypoint(10, 0)
        vehicle_state = MockVehicleState(8, 0, 15.0)  # Very high speed, close to goal
        trajectory = MockTrajectory(waypoints=[MockWaypoint(0, 0), goal_waypoint], length=10.0)

        dt = 0.1
        target_vel = self.controller.compute_target_velocity(vehicle_state, trajectory, target_direction=1.0, dt=dt)

        # Should be applying maximum deceleration
        max_deceleration_change = -self.controller.max_deceleration * dt
        expected_min_velocity = vehicle_state.velocity + max_deceleration_change

        self.assertAlmostEqual(target_vel, expected_min_velocity, places=2)


if __name__ == "__main__":
    # Run the tests
    unittest.main(verbosity=2)
