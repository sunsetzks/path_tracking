"""
Unit tests for vehicle model module.
"""

import numpy as np
import pytest
from src.mpc_car_tracking.vehicle_model import (
    VehicleState, VehicleControl, VehicleParameters, BicycleModel
)


class TestVehicleState:
    """Test VehicleState class."""
    
    def test_initialization(self):
        """Test VehicleState initialization."""
        state = VehicleState(x=1.0, y=2.0, yaw=0.5, v=10.0)
        assert state.x == 1.0
        assert state.y == 2.0
        assert state.yaw == 0.5
        assert state.v == 10.0
    
    def test_default_initialization(self):
        """Test default VehicleState initialization."""
        state = VehicleState()
        assert state.x == 0.0
        assert state.y == 0.0
        assert state.yaw == 0.0
        assert state.v == 0.0
    
    def test_to_array(self):
        """Test conversion to numpy array."""
        state = VehicleState(x=1.0, y=2.0, yaw=0.5, v=10.0)
        array = state.to_array()
        expected = np.array([1.0, 2.0, 0.5, 10.0])
        np.testing.assert_array_equal(array, expected)
    
    def test_from_array(self):
        """Test creation from numpy array."""
        array = np.array([1.0, 2.0, 0.5, 10.0])
        state = VehicleState.from_array(array)
        assert state.x == 1.0
        assert state.y == 2.0
        assert state.yaw == 0.5
        assert state.v == 10.0


class TestVehicleControl:
    """Test VehicleControl class."""
    
    def test_initialization(self):
        """Test VehicleControl initialization."""
        control = VehicleControl(acceleration=2.0, steering_angle=0.1)
        assert control.acceleration == 2.0
        assert control.steering_angle == 0.1
    
    def test_default_initialization(self):
        """Test default VehicleControl initialization."""
        control = VehicleControl()
        assert control.acceleration == 0.0
        assert control.steering_angle == 0.0
    
    def test_to_array(self):
        """Test conversion to numpy array."""
        control = VehicleControl(acceleration=2.0, steering_angle=0.1)
        array = control.to_array()
        expected = np.array([2.0, 0.1])
        np.testing.assert_array_equal(array, expected)


class TestVehicleParameters:
    """Test VehicleParameters class."""
    
    def test_default_parameters(self):
        """Test default vehicle parameters."""
        params = VehicleParameters()
        assert params.wheelbase == 2.7
        assert params.max_speed == 30.0
        assert params.max_acceleration == 3.0
        assert params.max_deceleration == -5.0
        assert params.max_steering_angle == np.pi / 3
        assert params.max_steering_rate == np.pi / 2
    
    def test_custom_parameters(self):
        """Test custom vehicle parameters."""
        params = VehicleParameters(
            wheelbase=3.0,
            max_speed=25.0,
            max_acceleration=2.5
        )
        assert params.wheelbase == 3.0
        assert params.max_speed == 25.0
        assert params.max_acceleration == 2.5


class TestBicycleModel:
    """Test BicycleModel class."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.params = VehicleParameters()
        self.model = BicycleModel(self.params)
        self.dt = 0.1
    
    def test_initialization(self):
        """Test BicycleModel initialization."""
        model = BicycleModel()
        assert isinstance(model.params, VehicleParameters)
        
        custom_params = VehicleParameters(wheelbase=3.0)
        model_custom = BicycleModel(custom_params)
        assert model_custom.params.wheelbase == 3.0
    
    def test_step_stationary(self):
        """Test vehicle step with zero control."""
        self.setUp()
        
        initial_state = VehicleState(x=0, y=0, yaw=0, v=0)
        control = VehicleControl(acceleration=0, steering_angle=0)
        
        next_state = self.model.step(initial_state, control, self.dt)
        
        # Vehicle should remain stationary
        assert next_state.x == 0.0
        assert next_state.y == 0.0
        assert next_state.yaw == 0.0
        assert next_state.v == 0.0
    
    def test_step_straight_motion(self):
        """Test straight line motion."""
        self.setUp()
        
        initial_state = VehicleState(x=0, y=0, yaw=0, v=10)
        control = VehicleControl(acceleration=0, steering_angle=0)
        
        next_state = self.model.step(initial_state, control, self.dt)
        
        # Vehicle should move forward in x direction
        assert next_state.x == pytest.approx(1.0, abs=1e-6)  # 10 * 0.1
        assert next_state.y == pytest.approx(0.0, abs=1e-6)
        assert next_state.yaw == pytest.approx(0.0, abs=1e-6)
        assert next_state.v == pytest.approx(10.0, abs=1e-6)
    
    def test_step_acceleration(self):
        """Test acceleration."""
        self.setUp()
        
        initial_state = VehicleState(x=0, y=0, yaw=0, v=10)
        control = VehicleControl(acceleration=2.0, steering_angle=0)
        
        next_state = self.model.step(initial_state, control, self.dt)
        
        # Check velocity increase
        expected_v = 10.0 + 2.0 * self.dt
        assert next_state.v == pytest.approx(expected_v, abs=1e-6)
    
    def test_step_turning(self):
        """Test turning motion."""
        self.setUp()
        
        initial_state = VehicleState(x=0, y=0, yaw=0, v=10)
        control = VehicleControl(acceleration=0, steering_angle=0.1)
        
        next_state = self.model.step(initial_state, control, self.dt)
        
        # Yaw should change
        expected_yaw_dot = 10.0 / self.params.wheelbase * np.tan(0.1)
        expected_yaw = expected_yaw_dot * self.dt
        assert next_state.yaw == pytest.approx(expected_yaw, abs=1e-6)
    
    def test_step_velocity_constraints(self):
        """Test velocity constraints."""
        self.setUp()
        
        # Test maximum velocity constraint
        initial_state = VehicleState(x=0, y=0, yaw=0, v=self.params.max_speed)
        control = VehicleControl(acceleration=5.0, steering_angle=0)
        
        next_state = self.model.step(initial_state, control, self.dt)
        assert next_state.v <= self.params.max_speed
        
        # Test minimum velocity constraint (should not go negative)
        initial_state = VehicleState(x=0, y=0, yaw=0, v=0.1)
        control = VehicleControl(acceleration=-10.0, steering_angle=0)
        
        next_state = self.model.step(initial_state, control, self.dt)
        assert next_state.v >= 0.0
    
    def test_normalize_angle(self):
        """Test angle normalization."""
        self.setUp()
        
        # Test angle wrapping
        assert self.model._normalize_angle(2 * np.pi) == pytest.approx(0.0, abs=1e-6)
        assert self.model._normalize_angle(-2 * np.pi) == pytest.approx(0.0, abs=1e-6)
        assert self.model._normalize_angle(3 * np.pi) == pytest.approx(-np.pi, abs=1e-6)
        assert self.model._normalize_angle(-3 * np.pi) == pytest.approx(np.pi, abs=1e-6)
    
    def test_linearize(self):
        """Test model linearization."""
        self.setUp()
        
        state = VehicleState(x=0, y=0, yaw=0, v=10)
        control = VehicleControl(acceleration=0, steering_angle=0.1)
        
        A, B = self.model.linearize(state, control, self.dt)
        
        # Check dimensions
        assert A.shape == (4, 4)
        assert B.shape == (4, 2)
        
        # Check that A is approximately identity + dt * A_continuous
        assert A[0, 0] == pytest.approx(1.0, abs=1e-6)
        assert A[1, 1] == pytest.approx(1.0, abs=1e-6)
        assert A[2, 2] == pytest.approx(1.0, abs=1e-6)
        assert A[3, 3] == pytest.approx(1.0, abs=1e-6)
    
    def test_apply_constraints(self):
        """Test control constraint application."""
        self.setUp()
        
        state = VehicleState(x=0, y=0, yaw=0, v=10)
        
        # Test steering angle constraints
        control = VehicleControl(acceleration=0, steering_angle=2.0)  # Exceed max
        constrained = self.model._apply_constraints(control, state)
        assert abs(constrained.steering_angle) <= self.params.max_steering_angle
        
        # Test acceleration constraints
        control = VehicleControl(acceleration=10.0, steering_angle=0)  # Exceed max
        constrained = self.model._apply_constraints(control, state)
        assert constrained.acceleration <= self.params.max_acceleration
        
        control = VehicleControl(acceleration=-10.0, steering_angle=0)  # Exceed min
        constrained = self.model._apply_constraints(control, state)
        assert constrained.acceleration >= self.params.max_deceleration
    
    def test_get_bounds(self):
        """Test state and control bounds."""
        self.setUp()
        
        # Test state bounds
        state_lb, state_ub = self.model.get_state_bounds()
        assert len(state_lb) == 4
        assert len(state_ub) == 4
        assert state_lb[3] == 0.0  # Minimum velocity
        assert state_ub[3] == self.params.max_speed
        
        # Test control bounds
        control_lb, control_ub = self.model.get_control_bounds()
        assert len(control_lb) == 2
        assert len(control_ub) == 2
        assert control_lb[0] == self.params.max_deceleration
        assert control_ub[0] == self.params.max_acceleration
        assert control_lb[1] == -self.params.max_steering_angle
        assert control_ub[1] == self.params.max_steering_angle