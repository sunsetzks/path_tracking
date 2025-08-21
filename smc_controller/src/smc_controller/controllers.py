"""
Sliding Mode Controllers for trajectory tracking.

Converted from MATLAB KinSliding.m and related controller functions.
"""

import numpy as np
from typing import Callable, Any, Dict, Tuple
from .parameters import CARParameters
from .utils import get_local_error, sgn, saturate, parse_input
from .vehicle_model import VehicleModel


class Controller:
    """
    Base controller class.
    """

    def __init__(self, name: str, control_point: str = 'REAR'):
        self.name = name
        self.control_point = control_point
        self.int_state0 = []  # For discrete controllers


class KinSlidingController(Controller):
    """
    Kinematic Sliding Mode Controller for trajectory tracking.

    Converted from MATLAB KinSliding.m
    """

    def __init__(self):
        super().__init__('KINSM', 'REAR')

        # Controller gains
        self.k0 = 0.05
        self.k1 = 0.25
        self.k2 = 0.5
        self.p1 = 1.0
        self.p2 = 3.0
        self.q1 = 1.0
        self.q2 = 3.0

    def init(self, model: VehicleModel, options: Any) -> 'KinSlidingController':
        """Initialize controller with model and options."""
        return self

    def compute_input(self, *args: Any) -> np.ndarray:
        """
        Compute control input.

        Args:
            Variable arguments: state, tau, t, p, [newTick, intState]

        Returns:
            Control input vector
        """
        # Parse inputs
        state, tau, t, p = parse_input(*args)

        # Trajectory values
        pd = np.array([tau.X(t), tau.Y(t)])
        vd = np.array([tau.dX(t), tau.dY(t)])
        thetad = tau.theta(t)
        omegad = tau.dtheta(t)
        domegad = tau.ddtheta(t)
        lad = tau.a(t)

        # Get local error (rear axle tracking)
        e, de = get_local_error(state, pd, vd, thetad, omegad, -p.l_R, 2)

        # Get rear wheel velocity
        vr = state[3]  # vx (assuming vx ≈ vr for kinematic controller)

        # Sliding surfaces
        s1 = de[0] + self.k1 * e[0]
        s2 = de[1] + self.k2 * e[2] + self.k0 * sgn(e[2]) * e[2]  # Use e[2] instead of e[3]

        # Control law
        dv_c = 1 / np.cos(e[2]) * (
            -self.q1 * s1 - self.p1 * sgn(s1) - self.k1 * de[0] - domegad * e[2] -
            omegad * de[2] + vr * de[2] * np.sin(e[2]) + lad  # Use e[2] instead of e[3]
        )

        # Steering angle control
        p_L = p.L
        vr_cos_e3 = vr * np.cos(e[2]) + self.k0 * sgn(e[2])  # Use e[2] instead of e[3]
        if abs(vr_cos_e3) > 1e-6:
            delta = np.arctan(p_L / vr * omegad + p_L / vr_cos_e3 * (
                -self.q2 * s2 - self.p2 * sgn(s2) - self.k2 * de[2] - domegad * e[0] +
                omegad * de[0]
            ))
        else:
            delta = 0.0

        # Saturate steering angle
        delta = saturate(delta, -p.MAX_delta, p.MAX_delta)

        # Return control input [delta, omega_f, omega_r, ax_command]
        u = np.array([delta, 0.0, 0.0, dv_c])  # [steering, front_wheel_omega, rear_wheel_omega, acceleration]

        return u


class AXController:
    """
    Acceleration/Steering controller mapping.

    Converts controller outputs to vehicle inputs.
    """

    def __init__(self, dv_c: float, state: np.ndarray, delta: float, p: CARParameters):
        """
        Map controller outputs to vehicle inputs.

        Args:
            dv_c: Desired velocity change
            state: Vehicle state
            delta: Steering angle
            p: Vehicle parameters

        Returns:
            Mapped control inputs
        """
        # This would implement the mapping from controller outputs to
        # actual vehicle inputs (wheel speeds, etc.)
        # For now, return basic mapping
        return np.array([delta, dv_c, 0.0, dv_c])


def create_kinematic_sliding_controller() -> KinSlidingController:
    """Factory function to create kinematic sliding mode controller."""
    controller = KinSlidingController()
    return controller
