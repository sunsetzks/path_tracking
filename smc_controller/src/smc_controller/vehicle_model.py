"""
Vehicle dynamics model.

Converted from MATLAB vmodel_A.m (bicycle model with nonlinear tire model).
"""

import numpy as np
from typing import Tuple, Callable, Any
from .parameters import CARParameters


class VehicleModel:
    """
    Bicycle vehicle model with nonlinear tire characteristics.

    State: [X, Y, psi, vx, vy, omega]
    Input: [delta, omega_f, omega_r, ax_command]

    Where:
        X, Y: Position
        psi: Yaw angle
        vx, vy: Longitudinal and lateral velocities
        omega: Yaw rate
        delta: Steering angle
        omega_f, omega_r: Front and rear wheel angular velocities
        ax_command: Longitudinal acceleration command
    """

    def __init__(self, params: CARParameters = None):
        """
        Initialize vehicle model.

        Args:
            params: Vehicle parameters
        """
        self.params = params if params is not None else CARParameters.default()

    def dynamics(self, t: float, x: np.ndarray, u: np.ndarray, p: CARParameters) -> Tuple[np.ndarray, np.ndarray]:
        """
        Vehicle dynamics equations.

        Args:
            t: Time
            x: State vector [X, Y, psi, vx, vy, omega]
            u: Input vector [delta, omega_f, omega_r, ax_command]
            p: Vehicle parameters

        Returns:
            Tuple of (state derivatives, output measurements)
        """
        # Extract parameters
        m, J, l_F, l_R, L, h, mu0 = p.m, p.J, p.l_F, p.l_R, p.L, p.h, p.mu0
        B_F, C_F, B_R, C_R, R = p.B_F, p.C_F, p.B_R, p.C_R, p.R
        g = 9.81

        # Extract state
        X, Y, psi, vx, vy, omega = x[:6]

        # Extract inputs with limitations
        delta = np.clip(u[0], -np.pi/4, np.pi/4)
        omega_F_cmd = u[1] if len(u) > 1 else 0
        omega_R_cmd = u[2] if len(u) > 2 else 0
        ax_cmd = u[3] if len(u) > 3 else 0

        # Front wheel velocities and constraints
        vf = np.array([vx, vy + l_F * omega])  # Velocity at front wheel center
        vfW = np.array([
            [np.cos(delta), np.sin(delta)],
            [-np.sin(delta), np.cos(delta)]
        ]) @ vf  # Velocity in wheel coordinates

        vf_abs = np.linalg.norm(vf)
        vfL = np.array([-np.cos(delta), -np.sin(delta)]) * omega_F_cmd * R  # Velocity of contact patch
        vfL_abs = np.linalg.norm(vfL)

        # Front slip calculation
        if max(vf_abs, vfL_abs) > 1e-6:
            sf = (vf + vfL) / max(vf_abs, vfL_abs)
            sf_abs = np.linalg.norm(sf)
            ff_abs = np.sin(C_F * np.arctan(B_F * sf_abs / mu0))
            ff = ff_abs * (-sf / sf_abs) if sf_abs > 0 else np.zeros(2)
        else:
            ff = np.zeros(2)

        # Rear wheel velocities and constraints
        vr = np.array([vx, vy - l_R * omega])  # Velocity at rear wheel center
        vr_abs = np.linalg.norm(vr)
        vrL = np.array([-1, 0]) * omega_R_cmd * R  # Velocity of contact patch
        vrL_abs = np.linalg.norm(vrL)

        # Rear slip calculation
        if max(vr_abs, vrL_abs) > 1e-6:
            sr = (vr + vrL) / max(vr_abs, vrL_abs)
            sr_abs = np.linalg.norm(sr)
            fr_abs = np.sin(C_R * np.arctan(B_R * sr_abs / mu0))
            fr = fr_abs * (-sr / sr_abs) if sr_abs > 0 else np.zeros(2)
        else:
            fr = np.zeros(2)

        # Normal force distribution
        Fzf = l_R * m * g / (L + ff[0] * mu0 * h)
        Fzr = m * g - Fzf

        # Tire forces
        Ff = ff * mu0 * Fzf  # Front forces
        Fr = fr * mu0 * Fzr  # Rear forces

        # Equations of motion
        # Longitudinal and lateral accelerations
        ax = (Ff[0] + Fr[0]) / m + vy * omega
        ay = (Ff[1] + Fr[1]) / m - vx * omega

        # Yaw acceleration
        alpha = (l_F * Ff[1] - l_R * Fr[1]) / J

        # State derivatives
        dx = np.zeros(6)  # 6 states: X, Y, psi, vx, vy, omega

        # Position derivatives
        dx[0] = np.cos(psi) * vx - np.sin(psi) * vy  # dX/dt
        dx[1] = np.sin(psi) * vx + np.cos(psi) * vy  # dY/dt
        dx[2] = omega  # dpsi/dt

        # Velocity derivatives
        dx[3] = ax  # dvx/dt
        dx[4] = ay  # dvy/dt
        dx[5] = alpha  # domega/dt

        # Output measurements: [front_slip, rear_slip, steering_angle, front_forces, rear_forces]
        y = np.concatenate([ff, fr, [delta], Ff, Fr])

        return dx, y

    def __call__(self, t: float, x: np.ndarray, u: np.ndarray, p: CARParameters) -> Tuple[np.ndarray, np.ndarray]:
        """Make the model callable."""
        return self.dynamics(t, x, u, p)


def vmodel_A(t: float, x: np.ndarray, u: np.ndarray, p: CARParameters) -> Tuple[np.ndarray, np.ndarray]:
    """
    Wrapper function for backward compatibility with MATLAB naming.
    """
    model = VehicleModel(p)
    return model.dynamics(t, x, u, p)
