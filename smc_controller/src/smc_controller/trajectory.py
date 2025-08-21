"""
Trajectory class for path representation.

Converted from MATLAB @Trajectory class.
"""

import numpy as np
from typing import Union, Optional, Any, Tuple
from .parameters import CARParameters


class Trajectory:
    """
    Trajectory representation for path tracking.

    Provides trajectory information including position, velocity, acceleration,
    and orientation along the path.
    """

    def __init__(self):
        """Initialize trajectory object."""
        self.id = ""  # Identifier
        self.T = 0.0  # End time
        self.s = None  # Arc length
        self.v = None  # Path-tangential velocity
        self.a = None  # Derivative of path-tangential velocity
        self.j = None  # Second derivative of path-tangential velocity
        self.theta = None  # Velocity orientation
        self.kappa = None  # Path curvature
        self.dtheta = None  # Derivative of velocity orientation
        self.ddtheta = None  # Second derivative of velocity orientation
        self.dddtheta = None  # Third derivative of velocity orientation

        # Position and derivatives
        self.X = None  # Earth-fixed x-coordinates
        self.Y = None  # Earth-fixed y-coordinates
        self.dX = None  # First derivatives
        self.dY = None
        self.ddX = None  # Second derivatives
        self.ddY = None

        # Internal dynamics solution
        self.psi0 = None  # Orientation of vehicle
        self.dpsi0 = None
        self.ddpsi0 = None

        self.lambda_ = 0.0  # Transformation distance

    def load(self, trajectory_data: dict):
        """
        Load trajectory from data dictionary.

        Args:
            trajectory_data: Dictionary containing trajectory data
        """
        # Load trajectory parameters
        if 'id' in trajectory_data:
            self.id = trajectory_data['id']
        if 'T' in trajectory_data:
            self.T = trajectory_data['T']

        # Load trajectory functions/data
        if 'X' in trajectory_data:
            self.X = trajectory_data['X']
        if 'Y' in trajectory_data:
            self.Y = trajectory_data['Y']
        if 'theta' in trajectory_data:
            self.theta = trajectory_data['theta']

        # Load derivatives if available
        if 'dX' in trajectory_data:
            self.dX = trajectory_data['dX']
        if 'dY' in trajectory_data:
            self.dY = trajectory_data['dY']
        if 'ddX' in trajectory_data:
            self.ddX = trajectory_data['ddX']
        if 'ddY' in trajectory_data:
            self.ddY = trajectory_data['ddY']
        if 'dtheta' in trajectory_data:
            self.dtheta = trajectory_data['dtheta']
        if 'ddtheta' in trajectory_data:
            self.ddtheta = trajectory_data['ddtheta']
        if 'dddtheta' in trajectory_data:
            self.dddtheta = trajectory_data['dddtheta']

        # Load other parameters
        if 'v' in trajectory_data:
            self.v = trajectory_data['v']
        if 'a' in trajectory_data:
            self.a = trajectory_data['a']
        if 'kappa' in trajectory_data:
            self.kappa = trajectory_data['kappa']

    def make_static(self, res: float = 0.01):
        """
        Create static trajectory representation.

        Args:
            res: Resolution for static representation

        Returns:
            Static trajectory object
        """
        # This would create a time-based interpolation of the trajectory
        # For now, return self
        return self

    def transform(self, lambda_: float):
        """
        Transform trajectory for reference position.

        Args:
            lambda_: Transformation distance

        Returns:
            Transformed trajectory
        """
        # Create new trajectory with transformation
        new_traj = Trajectory()
        new_traj.lambda_ = lambda_

        # Copy original data
        new_traj.id = self.id + "_transformed"
        new_traj.T = self.T

        # Apply transformation
        if self.X is not None and self.Y is not None and self.theta is not None:
            # Transform position by lambda along the path
            def transform_position(t):
                x = self.X(t) + lambda_ * np.cos(self.theta(t))
                y = self.Y(t) + lambda_ * np.sin(self.theta(t))
                return np.array([x, y])

            def transform_velocity(t):
                vx = self.dX(t) - lambda_ * np.sin(self.theta(t)) * self.dtheta(t)
                vy = self.dY(t) + lambda_ * np.cos(self.theta(t)) * self.dtheta(t)
                return np.array([vx, vy])

            new_traj.X = lambda t: transform_position(t)[0]
            new_traj.Y = lambda t: transform_position(t)[1]
            new_traj.dX = lambda t: transform_velocity(t)[0]
            new_traj.dY = lambda t: transform_velocity(t)[1]
            new_traj.theta = self.theta
            new_traj.dtheta = self.dtheta
            new_traj.ddtheta = self.ddtheta
            new_traj.dddtheta = self.dddtheta
            new_traj.v = self.v
            new_traj.a = self.a
            new_traj.kappa = self.kappa

        return new_traj

    def solve_internal_dynamics(self, p: CARParameters):
        """
        Solve internal dynamics for the trajectory.

        Args:
            p: Vehicle parameters
        """
        # This would solve for the vehicle's orientation dynamics
        # For now, we'll use a simplified approach
        if self.theta is not None:
            self.psi0 = self.theta  # Assume perfect tracking
            self.dpsi0 = self.dtheta if self.dtheta is not None else lambda t: 0
            self.ddpsi0 = self.ddtheta if self.ddtheta is not None else lambda t: 0

    @staticmethod
    def arc_length(X: np.ndarray, Y: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Calculate arc length and cumulative distance.

        Args:
            X: X coordinates
            Y: Y coordinates

        Returns:
            Tuple of (arc length array, cumulative distance)
        """
        dx = np.diff(X)
        dy = np.diff(Y)
        ds = np.sqrt(dx**2 + dy**2)
        s = np.concatenate([[0], np.cumsum(ds)])

        return s, ds

    @staticmethod
    def internal_dynamics_vehicle_A(xi: np.ndarray, p: CARParameters,
                                   dX: float, ddX: float, dY: float, ddY: float) -> np.ndarray:
        """
        Internal dynamics for vehicle model A.

        Args:
            xi: Internal state
            p: Vehicle parameters
            dX, ddX, dY, ddY: Trajectory derivatives

        Returns:
            Internal dynamics derivatives
        """
        # Simplified internal dynamics (would need full implementation)
        dxi = np.zeros_like(xi)
        return dxi


# Create a simple test trajectory for demonstration
def create_test_trajectory() -> Trajectory:
    """Create a simple circular trajectory for testing."""
    traj = Trajectory()
    traj.id = "test_circle"
    traj.T = 10.0

    # Circular trajectory
    radius = 50.0
    speed = 10.0

    # Parametric equations for circle
    traj.X = lambda t: radius * np.cos(2 * np.pi * t / traj.T)
    traj.Y = lambda t: radius * np.sin(2 * np.pi * t / traj.T)
    traj.theta = lambda t: np.pi/2 + 2 * np.pi * t / traj.T
    traj.dX = lambda t: -radius * 2 * np.pi / traj.T * np.sin(2 * np.pi * t / traj.T)
    traj.dY = lambda t: radius * 2 * np.pi / traj.T * np.cos(2 * np.pi * t / traj.T)
    traj.dtheta = lambda t: 2 * np.pi / traj.T
    traj.ddX = lambda t: -radius * (2 * np.pi / traj.T)**2 * np.cos(2 * np.pi * t / traj.T)
    traj.ddY = lambda t: -radius * (2 * np.pi / traj.T)**2 * np.sin(2 * np.pi * t / traj.T)
    traj.ddtheta = lambda t: 0.0
    traj.dddtheta = lambda t: 0.0
    traj.v = lambda t: speed
    traj.a = lambda t: 0.0
    traj.kappa = lambda t: 1.0 / radius

    return traj
