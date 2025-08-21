"""
Unit tests for SMC controller package.

Tests the core functionality of the sliding mode controller.
"""

import numpy as np
import pytest
from smc_controller import (
    KinSlidingController,
    VehicleModel,
    CARParameters,
    Trajectory,
    get_local_error,
    sgn,
    saturate,
    simulate,
    default_options
)


class TestUtils:
    """Test utility functions."""

    def test_sgn(self):
        """Test sign approximation function."""
        assert sgn(1.0) > 0
        assert sgn(-1.0) < 0
        assert abs(sgn(0.0)) < 1e-6

    def test_saturate(self):
        """Test saturation function."""
        assert saturate(5.0, -2.0, 2.0) == 2.0
        assert saturate(-5.0, -2.0, 2.0) == -2.0
        assert saturate(1.0, -2.0, 2.0) == 1.0

    def test_get_local_error(self):
        """Test local error calculation."""
        state = np.array([0.0, 0.0, 0.0, 10.0, 0.0, 0.0])
        pd = np.array([1.0, 0.0])
        vd = np.array([10.0, 0.0])
        thetad = 0.0
        wd = 0.0
        DL = 0.0

        e, de = get_local_error(state, pd, vd, thetad, wd, DL, ref=1)

        assert len(e) == 4  # [ex, ey, e_theta, e_velocity?]
        assert len(de) == 4


class TestParameters:
    """Test parameter classes."""

    def test_car_parameters(self):
        """Test CAR parameters initialization."""
        params = CARParameters.default()
        assert params.m > 0
        assert params.L > 0
        assert params.MAX_delta > 0

    def test_simulation_options(self):
        """Test simulation options."""
        options = default_options()
        assert options.stepSize > 0
        assert options.p is not None


class TestVehicleModel:
    """Test vehicle model."""

    def test_initialization(self):
        """Test vehicle model initialization."""
        params = CARParameters.default()
        model = VehicleModel(params)
        assert model.params == params

    def test_dynamics(self):
        """Test vehicle dynamics calculation."""
        params = CARParameters.default()
        model = VehicleModel(params)

        # Test state
        x = np.array([0.0, 0.0, 0.0, 10.0, 0.0, 0.0])
        u = np.array([0.0, 100.0, 100.0, 0.0])  # [delta, omega_f, omega_r, ax]

        dx, y = model.dynamics(0.0, x, u, params)

        assert len(dx) == 7  # State derivatives
        assert len(y) >= 4   # Output measurements


class TestController:
    """Test SMC controller."""

    def test_kinematic_controller(self):
        """Test kinematic sliding mode controller."""
        controller = KinSlidingController()
        assert controller.name == 'KINSM'
        assert controller.control_point == 'REAR'

    def test_control_input(self):
        """Test control input computation."""
        controller = KinSlidingController()
        params = CARParameters.default()

        # Create simple trajectory
        traj = Trajectory()
        traj.T = 10.0
        traj.X = lambda t: 10 * t
        traj.Y = lambda t: 0.0
        traj.theta = lambda t: 0.0
        traj.dX = lambda t: 10.0
        traj.dY = lambda t: 0.0
        traj.dtheta = lambda t: 0.0
        traj.ddtheta = lambda t: 0.0
        traj.a = lambda t: 0.0

        # Test state
        state = np.array([0.0, 0.0, 0.0, 10.0, 0.0, 0.0])

        u = controller.compute_input(state, traj, 0.0, params)

        assert len(u) >= 2  # At least [delta, acceleration]


class TestTrajectory:
    """Test trajectory class."""

    def test_trajectory_creation(self):
        """Test trajectory creation."""
        traj = Trajectory()
        assert traj.T == 0.0
        assert traj.lambda_ == 0.0

    def test_transform(self):
        """Test trajectory transformation."""
        traj = Trajectory()
        traj.T = 10.0
        traj.X = lambda t: 10 * t
        traj.Y = lambda t: 0.0
        traj.theta = lambda t: 0.0
        traj.dX = lambda t: 10.0
        traj.dY = lambda t: 0.0
        traj.dtheta = lambda t: 0.0
        traj.ddtheta = lambda t: 0.0
        traj.dddtheta = lambda t: 0.0

        transformed = traj.transform(1.0)
        assert transformed.lambda_ == 1.0


class TestSimulation:
    """Test simulation functionality."""

    def test_basic_simulation(self):
        """Test basic simulation execution."""
        # Create simple trajectory
        traj = Trajectory()
        traj.id = "test"
        traj.T = 2.0
        traj.X = lambda t: 10 * t
        traj.Y = lambda t: 0.0
        traj.theta = lambda t: 0.0
        traj.dX = lambda t: 10.0
        traj.dY = lambda t: 0.0
        traj.dtheta = lambda t: 0.0
        traj.ddtheta = lambda t: 0.0
        traj.a = lambda t: 0.0

        # Setup
        controller = KinSlidingController()
        model = VehicleModel()
        x0 = np.array([0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 0.0])
        options = default_options()

        # Run simulation
        result = simulate(traj, traj, controller, model, x0, options)

        # Check results
        assert 'controller' in result
        assert 'max_error' in result
        assert 'avg_error' in result
        assert len(result['data']['X']) > 0
        assert len(result['data']['T']) > 0


def test_integration():
    """Integration test for the complete system."""
    # This test ensures all components work together
    traj = Trajectory()
    traj.id = "integration_test"
    traj.T = 1.0
    traj.X = lambda t: 5 * t
    traj.Y = lambda t: 0.0
    traj.theta = lambda t: 0.0
    traj.dX = lambda t: 5.0
    traj.dY = lambda t: 0.0
    traj.dtheta = lambda t: 0.0
    traj.ddtheta = lambda t: 0.0
    traj.a = lambda t: 0.0

    controller = KinSlidingController()
    model = VehicleModel()
    x0 = np.array([0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0])
    options = default_options()

    result = simulate(traj, traj, controller, model, x0, options)

    # Check that simulation completed successfully
    assert result['total_time'] > 0
    assert result['max_error'][0] >= 0
    assert result['max_error'][1] >= 0


if __name__ == "__main__":
    # Run basic tests
    test_utils = TestUtils()
    test_utils.test_sgn()
    test_utils.test_saturate()

    test_params = TestParameters()
    test_params.test_car_parameters()

    test_model = TestVehicleModel()
    test_model.test_initialization()

    test_cont = TestController()
    test_cont.test_kinematic_controller()

    test_traj = TestTrajectory()
    test_traj.test_trajectory_creation()

    print("All basic tests passed!")

    # Run integration test
    test_integration()
    print("Integration test passed!")
