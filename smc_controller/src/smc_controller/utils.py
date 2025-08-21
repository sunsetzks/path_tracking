"""
Utility functions for SMC controller.

Converted from MATLAB fncs/ directory functions.
"""

import numpy as np
from typing import Tuple, Union, Any
from .parameters import CARParameters


def sgn(x: Union[float, np.ndarray]) -> Union[float, np.ndarray]:
    """
    Analytical sign approximation function.

    Args:
        x: Input value(s)

    Returns:
        Sign approximation using tanh function
    """
    return np.tanh(10 * x)


def saturate(x: Union[float, np.ndarray],
            lb: float,
            ub: float) -> Union[float, np.ndarray]:
    """
    Saturate value between lower and upper bounds.

    Args:
        x: Input value(s)
        lb: Lower bound
        ub: Upper bound

    Returns:
        Saturated value(s)
    """
    return np.clip(x, lb, ub)


def skew(omega: np.ndarray) -> np.ndarray:
    """
    Create skew-symmetric matrix from 3D vector.

    Args:
        omega: 3D vector

    Returns:
        3x3 skew-symmetric matrix
    """
    omega = omega.flatten()
    return np.array([
        [0, -omega[2], omega[1]],
        [omega[2], 0, -omega[0]],
        [-omega[1], omega[0], 0]
    ])


def get_local_error(state: np.ndarray,
                   pd: np.ndarray,
                   vd: np.ndarray,
                   thetad: float,
                   wd: float,
                   DL: float,
                   ref: int = 1) -> Tuple[np.ndarray, np.ndarray]:
    """
    Calculate local tracking error in specified reference frame.

    Args:
        state: Vehicle state [x, y, psi, vx, vy, omega]
        pd: Desired position [xd, yd]
        vd: Desired velocity [vdx, vdy]
        thetad: Desired orientation
        wd: Desired angular velocity
        DL: Distance from CG to reference point
        ref: Reference frame (1: local vehicle frame, 2: local desired vehicle frame)

    Returns:
        Tuple of (position error, velocity error)
    """
    x, y, psi, vx, vy, omega = state[:6]

    if ref == 1:
        # Local vehicle frame
        rot_z = np.array([
            [np.cos(psi), -np.sin(psi), 0],
            [np.sin(psi), np.cos(psi), 0],
            [0, 0, 1]
        ])
        drot_z = skew(np.array([0, 0, omega])) @ rot_z
    elif ref == 2:
        # Local desired vehicle frame
        rot_z = np.array([
            [np.cos(thetad), -np.sin(thetad), 0],
            [np.sin(thetad), np.cos(thetad), 0],
            [0, 0, 1]
        ])
        drot_z = skew(np.array([0, 0, wd])) @ rot_z
    else:
        raise ValueError("Not a valid frame")

    # Position error in Cartesian coordinates
    e_cart = np.array([
        x + DL * np.cos(psi) - pd[0],
        y + DL * np.sin(psi) - pd[1],
        psi - thetad
    ])

    # Position error in local frame
    e = rot_z.T @ e_cart

    # Velocity in body frame
    Rot = np.array([
        [np.cos(psi), -np.sin(psi)],
        [np.sin(psi), np.cos(psi)]
    ])
    v_xy_cart = Rot @ np.array([vx, vy]) + np.array([-np.sin(psi), np.cos(psi)]) * DL * omega

    # Velocity error
    v_error_cart = np.array([v_xy_cart[0] - vd[0], v_xy_cart[1] - vd[1], omega - wd])
    de = rot_z.T @ v_error_cart + drot_z.T @ e_cart

    return e, de


def numerical_derivative(f: callable, x: float, h: float = 1e-6) -> float:
    """
    Calculate numerical derivative using central difference.

    Args:
        f: Function to differentiate
        x: Point at which to evaluate derivative
        h: Step size

    Returns:
        Numerical derivative
    """
    return (f(x + h) - f(x - h)) / (2 * h)


def generate_noise(V: float, T: float, step_size: float) -> callable:
    """
    Generate measurement noise function.

    Args:
        V: Noise level
        T: Total time
        step_size: Simulation step size

    Returns:
        Noise function of time
    """
    np.random.seed(42)  # For reproducibility

    def noise_func(t: float) -> np.ndarray:
        if V > 0:
            return V * np.random.randn(6)  # 6 states
        else:
            return np.zeros(6)

    return noise_func


def parse_input(*args: Any) -> Tuple[np.ndarray, Any, float, CARParameters]:
    """
    Parse variable input arguments for controller.

    Args:
        *args: Variable arguments (state, tau, t, p, ...)

    Returns:
        Tuple of (state, tau, t, p)
    """
    state = args[0]  # Vehicle state
    tau = args[1]    # Trajectory
    t = args[2]      # Time
    p = args[3]      # Parameters

    return state, tau, t, p
