"""
Unit tests for trajectory planner module.
"""

import numpy as np
import pytest
from src.mpc_car_tracking.trajectory_planner import (
    TrajectoryPoint, TrajectoryPlanner
)
from src.mpc_car_tracking.vehicle_model import VehicleState, VehicleParameters


class TestTrajectoryPoint:
    """Test TrajectoryPoint class."""
    
    def test_initialization(self):
        """Test TrajectoryPoint initialization."""
        state = VehicleState(x=1.0, y=2.0, yaw=0.5, v=10.0)
        point = TrajectoryPoint(state=state, curvature=0.1, s=5.0, time=0.5)
        
        assert point.state == state
        assert point.curvature == 0.1
        assert point.s == 5.0
        assert point.time == 0.5
    
    def test_default_initialization(self):
        """Test default TrajectoryPoint initialization."""
        state = VehicleState()
        point = TrajectoryPoint(state=state)
        
        assert point.curvature == 0.0
        assert point.s == 0.0
        assert point.time == 0.0


class TestTrajectoryPlanner:
    """Test TrajectoryPlanner class."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.params = VehicleParameters()
        self.planner = TrajectoryPlanner(self.params)
    
    def test_initialization(self):
        """Test TrajectoryPlanner initialization."""
        planner = TrajectoryPlanner()
        assert isinstance(planner.vehicle_params, VehicleParameters)
        
        custom_params = VehicleParameters(wheelbase=3.0)
        planner_custom = TrajectoryPlanner(custom_params)
        assert planner_custom.vehicle_params.wheelbase == 3.0
    
    def test_straight_line(self):
        """Test straight line trajectory generation."""
        self.setUp()
        
        start_state = VehicleState(x=0, y=0, yaw=0, v=10)
        end_state = VehicleState(x=10, y=0, yaw=0, v=10)
        
        trajectory = self.planner.straight_line(start_state, end_state, num_points=11, target_speed=10)
        
        assert len(trajectory) == 11
        
        # Check first and last points
        assert trajectory[0].state.x == pytest.approx(0.0, abs=1e-6)
        assert trajectory[0].state.y == pytest.approx(0.0, abs=1e-6)
        assert trajectory[-1].state.x == pytest.approx(10.0, abs=1e-6)
        assert trajectory[-1].state.y == pytest.approx(0.0, abs=1e-6)
        
        # Check that all points have the same heading and speed
        for point in trajectory:
            assert point.state.yaw == pytest.approx(0.0, abs=1e-6)
            assert point.state.v == pytest.approx(10.0, abs=1e-6)
            assert point.curvature == pytest.approx(0.0, abs=1e-6)
    
    def test_straight_line_angled(self):
        """Test angled straight line trajectory."""
        self.setUp()
        
        start_state = VehicleState(x=0, y=0, yaw=0, v=10)
        end_state = VehicleState(x=10, y=10, yaw=0, v=10)
        
        trajectory = self.planner.straight_line(start_state, end_state, num_points=11, target_speed=10)
        
        # Check heading is 45 degrees
        expected_heading = np.arctan2(10, 10)  # 45 degrees
        for point in trajectory:
            assert point.state.yaw == pytest.approx(expected_heading, abs=1e-6)
    
    def test_circular_path(self):
        """Test circular trajectory generation."""
        self.setUp()
        
        center = (0, 0)
        radius = 5.0
        
        trajectory = self.planner.circular_path(
            center=center, radius=radius, start_angle=0, end_angle=np.pi/2,
            num_points=11, target_speed=10, clockwise=False
        )
        
        assert len(trajectory) == 11
        
        # Check first and last points
        assert trajectory[0].state.x == pytest.approx(radius, abs=1e-6)
        assert trajectory[0].state.y == pytest.approx(0.0, abs=1e-6)
        assert trajectory[-1].state.x == pytest.approx(0.0, abs=1e-6)
        assert trajectory[-1].state.y == pytest.approx(radius, abs=1e-6)
        
        # Check curvature is constant
        expected_curvature = 1.0 / radius
        for point in trajectory:
            assert point.curvature == pytest.approx(expected_curvature, abs=1e-6)
            assert point.state.v == pytest.approx(10.0, abs=1e-6)
    
    def test_circular_path_clockwise(self):
        """Test clockwise circular trajectory."""
        self.setUp()
        
        center = (0, 0)
        radius = 5.0
        
        trajectory = self.planner.circular_path(
            center=center, radius=radius, start_angle=0, end_angle=np.pi/2,
            num_points=11, target_speed=10, clockwise=True
        )
        
        # Check curvature is negative for clockwise
        expected_curvature = -1.0 / radius
        for point in trajectory:
            assert point.curvature == pytest.approx(expected_curvature, abs=1e-6)
    
    def test_figure_eight(self):
        """Test figure-8 trajectory generation."""
        self.setUp()
        
        center = (0, 0)
        width = 10.0
        height = 6.0
        
        trajectory = self.planner.figure_eight(
            center=center, width=width, height=height,
            num_points=100, target_speed=8
        )
        
        assert len(trajectory) == 100
        
        # Check that trajectory forms a closed loop (approximately)
        start_x, start_y = trajectory[0].state.x, trajectory[0].state.y
        end_x, end_y = trajectory[-1].state.x, trajectory[-1].state.y
        
        # Should be close to starting point for figure-8
        distance = np.sqrt((end_x - start_x)**2 + (end_y - start_y)**2)
        assert distance < 1.0  # Allow some tolerance
        
        # Check speed consistency
        for point in trajectory:
            assert point.state.v == pytest.approx(8.0, abs=1e-6)
    
    def test_spline_path_two_points(self):
        """Test spline path with two points (should be linear)."""
        self.setUp()
        
        waypoints = [(0, 0), (10, 5)]
        
        trajectory = self.planner.spline_path(
            waypoints=waypoints, target_speed=10, num_points=11
        )
        
        assert len(trajectory) == 11
        
        # Check that it's a straight line
        expected_slope = 5.0 / 10.0  # dy/dx
        expected_heading = np.arctan(expected_slope)
        
        for point in trajectory:
            assert point.state.yaw == pytest.approx(expected_heading, abs=1e-6)
            assert point.curvature == pytest.approx(0.0, abs=1e-6)
    
    def test_spline_path_multiple_points(self):
        """Test spline path with multiple waypoints."""
        self.setUp()
        
        waypoints = [(0, 0), (5, 5), (10, 0), (15, -5)]
        
        trajectory = self.planner.spline_path(
            waypoints=waypoints, target_speed=10, num_points=50
        )
        
        assert len(trajectory) == 50
        
        # Check that trajectory passes through (approximately) the waypoints
        # First point
        assert trajectory[0].state.x == pytest.approx(0.0, abs=1e-1)
        assert trajectory[0].state.y == pytest.approx(0.0, abs=1e-1)
        
        # Last point
        assert trajectory[-1].state.x == pytest.approx(15.0, abs=1e-1)
        assert trajectory[-1].state.y == pytest.approx(-5.0, abs=1e-1)
    
    def test_spline_path_insufficient_points(self):
        """Test spline path with insufficient waypoints."""
        self.setUp()
        
        waypoints = [(0, 0)]  # Only one point
        
        with pytest.raises(ValueError):
            self.planner.spline_path(waypoints=waypoints)
    
    def test_lane_change(self):
        """Test lane change trajectory generation."""
        self.setUp()
        
        start_state = VehicleState(x=0, y=0, yaw=0, v=15)
        lane_offset = 3.5
        lane_change_distance = 50.0
        
        trajectory = self.planner.lane_change(
            start_state=start_state,
            lane_offset=lane_offset,
            lane_change_distance=lane_change_distance,
            target_speed=15,
            num_points=50
        )

        assert len(trajectory) == 50

        # Check boundary conditions
        # Start: should be at origin with zero lateral offset
        assert trajectory[0].state.x == pytest.approx(0.0, abs=1e-6)
        assert trajectory[0].state.y == pytest.approx(0.0, abs=1e-6)

        # End: should have completed lane change
        assert trajectory[-1].state.x == pytest.approx(lane_change_distance, abs=1e-1)
        assert trajectory[-1].state.y == pytest.approx(lane_offset, abs=1e-1)

        # Check speed consistency
        for point in trajectory:
            assert point.state.v == pytest.approx(15.0, abs=1e-6)

    def test_resample_trajectory(self):
        """Test trajectory resampling."""
        self.setUp()

        # Create a simple trajectory
        start_state = VehicleState(x=0, y=0, yaw=0, v=10)
        end_state = VehicleState(x=10, y=0, yaw=0, v=10)

        original_trajectory = self.planner.straight_line(
            start_state, end_state, num_points=11, target_speed=10
        )

        # Resample with different time step
        dt = 0.2
        resampled = self.planner.resample_trajectory(original_trajectory, dt)

        # Check that resampling produces reasonable number of points
        total_time = original_trajectory[-1].time
        expected_points = int(total_time / dt) + 1
        assert len(resampled) == expected_points

        # Check first and last points
        assert resampled[0].x == pytest.approx(0.0, abs=1e-6)
        assert resampled[0].y == pytest.approx(0.0, abs=1e-6)
        assert resampled[-1].x == pytest.approx(10.0, abs=1e-1)
        assert resampled[-1].y == pytest.approx(0.0, abs=1e-1)

    def test_resample_trajectory_empty(self):
        """Test resampling empty trajectory."""
        self.setUp()

        resampled = self.planner.resample_trajectory([], dt=0.1)
        assert len(resampled) == 0