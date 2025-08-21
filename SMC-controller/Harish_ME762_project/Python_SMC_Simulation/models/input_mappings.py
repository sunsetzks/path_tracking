"""
Input mapping functions for vehicle control
Translated from MATLAB mapping functions
"""

import numpy as np


def map0(ax_ref, y, steer, p):
    """
    Simple input mapping - direct mapping to acceleration and steering
    Translated from MATLAB map0.m

    Args:
        ax_ref: desired longitudinal acceleration
        y: outputs (not used)
        steer: steering angle
        p: vehicle parameters

    Returns:
        u: control input [steering_angle, acceleration]
    """
    return np.array([steer, ax_ref])


def AXController_pacejka(AX_REF, x, delta, p):
    """
    Advanced input mapping using Pacejka tire model
    Translated from MATLAB AXController_pacejka.m

    Args:
        AX_REF: desired longitudinal acceleration
        x: vehicle state
        delta: steering angle
        p: vehicle parameters

    Returns:
        u: control input [delta, omega_f, omega_r]
    """
    mu0 = p['mu0']
    R = p['R']
    m = p['m']
    vx = x[3]
    vy = x[4]
    omega = x[5]
    l_F = p['l_F']
    l_R = p['l_R']
    C_R = p['C_R']
    B_R = p['B_R']
    L = p['L']
    h = p['h']
    J = p['J']
    g = 9.81

    # Calculate required braking force
    Fb = (AX_REF - vy * omega) * m
    Fxf = Fb
    Fxr = 0

    # Normal forces
    fzf_max = (m * g * l_R) / (-mu0 * h + L)
    Fzf = np.maximum(0, np.minimum(fzf_max, l_R/L * m * g - h/L * Fb))
    Fzr = m * g - Fzf

    # Maximal absolute forces
    Fmaxf = mu0 * Fzf
    Fmaxr = mu0 * Fzr

    # Velocity at rear axle
    vr = np.array([vx, vy - l_R * omega])

    # Rear lateral tire force function
    omega_R = vx / R

    # Calculate rear tire force
    vr_slip = vr - np.array([R * omega_R, 0])
    vr_norm = np.linalg.norm(vr)

    if vr_norm > 0:
        slip_ratio = np.linalg.norm(vr_slip) / vr_norm / mu0
        fr_abs = mu0 * Fmaxr * np.sin(C_R * np.arctan(B_R * slip_ratio))
        if np.linalg.norm(vr_slip) == 0:
            Fyr = 0
        else:
            fr_unit = vr_slip / np.linalg.norm(vr_slip)
            fr = -fr_abs * fr_unit
            Fyr = fr[1]
    else:
        Fyr = 0

    # Front lateral tire force (simplified)
    Fyf = 0  # (m*(0+vx*omega) - 0*Fyr)

    # Absolute front tire force
    Ff_abs = np.sqrt(Fxf**2 + Fyf**2)

    # Check if maximal tire force exceeded
    if Ff_abs > Fmaxf:
        scale = Fmaxf / Ff_abs
        Fxf = Fxf * scale
        Fyf = Fyf * scale
        Ff_abs = Fmaxf

    # Calculate required slip
    B_F = p['B_F']
    C_F = p['C_F']

    if Ff_abs == 0:
        sf = np.zeros(2)
    else:
        sf_abs = mu0 / B_F * np.tan(1/C_F * np.arcsin(Ff_abs / Fmaxf))
        sf = np.array([-Fxf/Ff_abs * sf_abs, -Fyf/Ff_abs * sf_abs])

    # Velocity at front axle
    vf = np.array([vx, vy + l_F * omega])
    vf_abs = np.linalg.norm(vf)

    # Latsch velocity
    Rot_delta = np.array([[np.cos(delta), -np.sin(delta)],
                          [np.sin(delta), np.cos(delta)]])
    vLf = sf * vf_abs - vf

    # Required front wheel speed
    omega_F = -vLf[0] / R / Rot_delta[0, 0]

    return np.array([delta, omega_F, omega_R])
