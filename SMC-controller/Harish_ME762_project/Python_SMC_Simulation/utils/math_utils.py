"""
Mathematical utility functions for SMC simulation
"""

import numpy as np


def sgn(x):
    """
    Sign function

    Args:
        x: input value or array

    Returns:
        sign of x (-1, 0, or 1)
    """
    return np.sign(x)


def saturate(x, min_val, max_val):
    """
    Saturation function - clips value to specified range

    Args:
        x: input value
        min_val: minimum allowed value
        max_val: maximum allowed value

    Returns:
        clipped value
    """
    return np.clip(x, min_val, max_val)


def get_local_error(state, pd, vd, thetad, omegad, lambda_offset, control_type=2):
    """
    Calculate local tracking errors
    Translated from MATLAB getLocalError function

    Args:
        state: vehicle state [X, Y, psi, vx, vy, omega]
        pd: desired position [Xd, Yd]
        vd: desired velocity [vxd, vyd]
        thetad: desired heading
        omegad: desired yaw rate
        lambda_offset: offset distance from reference point
        control_type: type of control (1 or 2)

    Returns:
        e: position errors in vehicle frame
        de: velocity errors in vehicle frame
    """

    # Current state
    p = np.array([state[0], state[1]])  # current position
    psi = state[2]                     # current heading
    v = np.array([state[3], state[4]]) # current velocity
    omega = state[5]                   # current yaw rate

    # Position error in global frame
    ep = p - pd

    # Rotation matrix from global to vehicle frame
    R = np.array([[np.cos(psi), np.sin(psi)],
                  [-np.sin(psi), np.cos(psi)]])

    # Position error in vehicle frame
    e = R @ ep

    # Apply offset if specified
    if lambda_offset != 0:
        e[0] += lambda_offset

    # Velocity error in global frame
    ev = v - vd

    # Velocity error in vehicle frame
    de = R @ ev

    # Heading error
    e_psi = psi - thetad

    # For control type 2, include heading error as third component
    if control_type == 2:
        e = np.array([e[0], e[1], e_psi])
        de = np.array([de[0], de[1], -omega + omegad])

    return e, de
