"""
Trajectory class for path tracking
Simplified version translated from MATLAB Trajectory class
"""

import numpy as np
import scipy.interpolate as interp


class Trajectory:
    """
    Trajectory class for storing and accessing reference trajectories
    """

    def __init__(self):
        self.id = ""
        self.T = 0.0  # end time
        self.lambda_offset = 0.0  # transformation distance

        # Initialize trajectory functions (will be set by load or make_static)
        self.X = None
        self.Y = None
        self.dX = None
        self.dY = None
        self.ddX = None
        self.ddY = None
        self.theta = None
        self.dtheta = None
        self.ddtheta = None
        self.dddtheta = None
        self.s = None  # arc length
        self.v = None  # tangential velocity
        self.a = None  # acceleration
        self.j = None  # jerk
        self.kappa = None  # curvature

    def load(self, scenario_file):
        """
        Load trajectory from scenario file
        For now, creates a simple lane change trajectory
        """
        self.id = scenario_file

        if 'single_lane_change' in scenario_file:
            self._make_single_lane_change()
        elif 'double_lane_change' in scenario_file:
            self._make_double_lane_change()
        else:
            self._make_default_trajectory()

    def _make_single_lane_change(self):
        """Create a single lane change trajectory"""
        # Time parameters
        self.T = 10.0

        # Simple clothoid-based trajectory
        t = np.linspace(0, self.T, 1000)

        # Path definition (smooth lane change)
        x_path = 20 * t / self.T  # move forward
        y_path = 3 * np.sin(np.pi * t / self.T)  # lane change

        # Heading angle
        theta_path = np.arctan2(np.gradient(y_path, t), np.gradient(x_path, t))

        # Velocity profile (constant speed with smooth start/stop)
        v_profile = 15 * np.ones_like(t)  # 15 m/s constant speed
        v_profile[:50] = v_profile[:50] * np.linspace(0, 1, 50)  # smooth start
        v_profile[-50:] = v_profile[-50:] * np.linspace(1, 0, 50)  # smooth stop

        # Create interpolation functions
        self.X = interp.interp1d(t, x_path, bounds_error=False, fill_value='extrapolate')
        self.Y = interp.interp1d(t, y_path, bounds_error=False, fill_value='extrapolate')
        self.theta = interp.interp1d(t, theta_path, bounds_error=False, fill_value='extrapolate')
        self.v = interp.interp1d(t, v_profile, bounds_error=False, fill_value='extrapolate')

        # Calculate derivatives
        dt = t[1] - t[0]
        dX_vals = np.gradient(x_path, dt)
        dY_vals = np.gradient(y_path, dt)
        ddX_vals = np.gradient(dX_vals, dt)
        ddY_vals = np.gradient(dY_vals, dt)
        dtheta_vals = np.gradient(theta_path, dt)
        ddtheta_vals = np.gradient(dtheta_vals, dt)
        dddtheta_vals = np.gradient(ddtheta_vals, dt)

        self.dX = interp.interp1d(t, dX_vals, bounds_error=False, fill_value='extrapolate')
        self.dY = interp.interp1d(t, dY_vals, bounds_error=False, fill_value='extrapolate')
        self.ddX = interp.interp1d(t, ddX_vals, bounds_error=False, fill_value='extrapolate')
        self.ddY = interp.interp1d(t, ddY_vals, bounds_error=False, fill_value='extrapolate')
        self.dtheta = interp.interp1d(t, dtheta_vals, bounds_error=False, fill_value='extrapolate')
        self.ddtheta = interp.interp1d(t, ddtheta_vals, bounds_error=False, fill_value='extrapolate')
        self.dddtheta = interp.interp1d(t, dddtheta_vals, bounds_error=False, fill_value='extrapolate')

        # Simple acceleration profile
        a_vals = np.gradient(v_profile, dt)
        j_vals = np.gradient(a_vals, dt)
        self.a = interp.interp1d(t, a_vals, bounds_error=False, fill_value='extrapolate')
        self.j = interp.interp1d(t, j_vals, bounds_error=False, fill_value='extrapolate')

        # Curvature (simplified)
        kappa_vals = dtheta_vals / v_profile
        self.kappa = interp.interp1d(t, kappa_vals, bounds_error=False, fill_value='extrapolate')

    def _make_double_lane_change(self):
        """Create a double lane change trajectory"""
        # Time parameters
        self.T = 15.0

        # More complex trajectory with double lane change
        t = np.linspace(0, self.T, 1000)

        # Path definition
        x_path = 25 * t / self.T
        # Double sine wave for double lane change
        y_path = 2 * np.sin(2 * np.pi * t / self.T) * np.exp(-0.5 * (t - self.T/2)**2 / (self.T/4)**2)

        # Heading angle
        theta_path = np.arctan2(np.gradient(y_path, t), np.gradient(x_path, t))

        # Velocity profile
        v_profile = 12 * np.ones_like(t)
        v_profile[:75] = v_profile[:75] * np.linspace(0, 1, 75)
        v_profile[-75:] = v_profile[-75:] * np.linspace(1, 0, 75)

        # Create interpolation functions (same as single lane change)
        self.X = interp.interp1d(t, x_path, bounds_error=False, fill_value='extrapolate')
        self.Y = interp.interp1d(t, y_path, bounds_error=False, fill_value='extrapolate')
        self.theta = interp.interp1d(t, theta_path, bounds_error=False, fill_value='extrapolate')
        self.v = interp.interp1d(t, v_profile, bounds_error=False, fill_value='extrapolate')

        # Calculate derivatives (same as single lane change)
        dt = t[1] - t[0]
        dX_vals = np.gradient(x_path, dt)
        dY_vals = np.gradient(y_path, dt)
        ddX_vals = np.gradient(dX_vals, dt)
        ddY_vals = np.gradient(dY_vals, dt)
        dtheta_vals = np.gradient(theta_path, dt)
        ddtheta_vals = np.gradient(dtheta_vals, dt)
        dddtheta_vals = np.gradient(ddtheta_vals, dt)

        self.dX = interp.interp1d(t, dX_vals, bounds_error=False, fill_value='extrapolate')
        self.dY = interp.interp1d(t, dY_vals, bounds_error=False, fill_value='extrapolate')
        self.ddX = interp.interp1d(t, ddX_vals, bounds_error=False, fill_value='extrapolate')
        self.ddY = interp.interp1d(t, ddY_vals, bounds_error=False, fill_value='extrapolate')
        self.dtheta = interp.interp1d(t, dtheta_vals, bounds_error=False, fill_value='extrapolate')
        self.ddtheta = interp.interp1d(t, ddtheta_vals, bounds_error=False, fill_value='extrapolate')
        self.dddtheta = interp.interp1d(t, dddtheta_vals, bounds_error=False, fill_value='extrapolate')

        a_vals = np.gradient(v_profile, dt)
        j_vals = np.gradient(a_vals, dt)
        self.a = interp.interp1d(t, a_vals, bounds_error=False, fill_value='extrapolate')
        self.j = interp.interp1d(t, j_vals, bounds_error=False, fill_value='extrapolate')

        kappa_vals = dtheta_vals / v_profile
        self.kappa = interp.interp1d(t, kappa_vals, bounds_error=False, fill_value='extrapolate')

    def _make_default_trajectory(self):
        """Create a default straight line trajectory"""
        self.T = 8.0
        t = np.linspace(0, self.T, 1000)

        # Straight line path
        x_path = 10 * t / self.T
        y_path = np.zeros_like(t)
        theta_path = np.zeros_like(t)
        v_profile = 8 * np.ones_like(t)
        v_profile[:50] = v_profile[:50] * np.linspace(0, 1, 50)
        v_profile[-50:] = v_profile[-50:] * np.linspace(1, 0, 50)

        # Create interpolation functions
        self.X = interp.interp1d(t, x_path, bounds_error=False, fill_value='extrapolate')
        self.Y = interp.interp1d(t, y_path, bounds_error=False, fill_value='extrapolate')
        self.theta = interp.interp1d(t, theta_path, bounds_error=False, fill_value='extrapolate')
        self.v = interp.interp1d(t, v_profile, bounds_error=False, fill_value='extrapolate')

        # Calculate derivatives
        dt = t[1] - t[0]
        dX_vals = np.gradient(x_path, dt)
        dY_vals = np.gradient(y_path, dt)
        ddX_vals = np.gradient(dX_vals, dt)
        ddY_vals = np.gradient(dY_vals, dt)
        dtheta_vals = np.gradient(theta_path, dt)
        ddtheta_vals = np.gradient(dtheta_vals, dt)
        dddtheta_vals = np.gradient(ddtheta_vals, dt)

        self.dX = interp.interp1d(t, dX_vals, bounds_error=False, fill_value='extrapolate')
        self.dY = interp.interp1d(t, dY_vals, bounds_error=False, fill_value='extrapolate')
        self.ddX = interp.interp1d(t, ddX_vals, bounds_error=False, fill_value='extrapolate')
        self.ddY = interp.interp1d(t, ddY_vals, bounds_error=False, fill_value='extrapolate')
        self.dtheta = interp.interp1d(t, dtheta_vals, bounds_error=False, fill_value='extrapolate')
        self.ddtheta = interp.interp1d(t, ddtheta_vals, bounds_error=False, fill_value='extrapolate')
        self.dddtheta = interp.interp1d(t, dddtheta_vals, bounds_error=False, fill_value='extrapolate')

        a_vals = np.gradient(v_profile, dt)
        j_vals = np.gradient(a_vals, dt)
        self.a = interp.interp1d(t, a_vals, bounds_error=False, fill_value='extrapolate')
        self.j = interp.interp1d(t, j_vals, bounds_error=False, fill_value='extrapolate')

        kappa_vals = dtheta_vals / v_profile
        self.kappa = interp.interp1d(t, kappa_vals, bounds_error=False, fill_value='extrapolate')

    def transform(self, lambda_offset):
        """
        Transform trajectory for a reference position offset
        """
        new_traj = Trajectory()
        new_traj.id = self.id + "_transformed"
        new_traj.T = self.T
        new_traj.lambda_offset = lambda_offset

        # For now, return self (simplified implementation)
        # In full implementation, this would adjust all trajectory functions
        # by the offset distance lambda_offset
        return new_traj

    def make_static(self, res):
        """
        Create a static, linear interpolated representation
        """
        # For now, return self (simplified implementation)
        return self

    @staticmethod
    def arcLength(X, Y):
        """
        Calculate arc length and inverse mapping
        """
        # Simplified implementation
        return np.array([0]), lambda s: 0
