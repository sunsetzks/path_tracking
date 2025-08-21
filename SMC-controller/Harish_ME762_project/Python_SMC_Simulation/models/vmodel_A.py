"""
Vehicle Model A - Bicycle model with nonlinear tire dynamics
Translated from MATLAB vmodel_A.m

State: x = [X, Y, psi, vx, vy, omega]
Input: u = [delta, omega_f, omega_r]
"""

import numpy as np


def vmodel_A(t, x, u, p):
    """
    Bicycle model with nonlinear tire model
    State x = [X, Y, psi, vx, vy, omega]
    Input u = [delta, omega_f, omega_r]

    Args:
        t: time (not used, for compatibility with ODE solvers)
        x: state vector [X, Y, psi, vx, vy, omega]
        u: input vector [delta, omega_f, omega_r]
        p: vehicle parameters

    Returns:
        dx: state derivatives
        y: outputs [ff, fr, delta, Ff, Fr] where ff, fr are slip vectors
    """

    # Extract parameters
    m = p['m']
    J = p['J']
    l_F = p['l_F']
    l_R = p['l_R']
    L = p['L']
    h = p['h']
    mu0 = p['mu0']
    B_F = p['B_F']
    C_F = p['C_F']
    B_R = p['B_R']
    C_R = p['C_R']
    R = p['R']
    g = 9.81

    # Extract state
    X = x[0]      # X position
    Y = x[1]      # Y position
    psi = x[2]    # heading angle
    vx = x[3]     # longitudinal velocity
    vy = x[4]     # lateral velocity
    omega = x[5]  # yaw rate

    # Input with limitations
    delta = np.clip(u[0], -np.pi/4, np.pi/4)  # steering angle

    # Front wheel velocity
    vf = np.array([vx, vy + l_F * omega])  # velocity at front wheel center
    # Rotation matrix for front wheel
    vfW = np.array([[np.cos(delta), np.sin(delta)],
                    [-np.sin(delta), np.cos(delta)]]) @ vf
    # Front wheel angular velocity with limits
    omega_F = np.clip(u[1], np.maximum(vfW[0]/R, 0), np.minimum(vfW[0]/R, 0))

    # Rear wheel velocity
    vr = np.array([vx, vy - l_R * omega])  # velocity at rear wheel center
    # Rear wheel angular velocity with limits
    omega_R = np.clip(u[2], np.maximum(vr[0]/R, 0), np.minimum(vr[0]/R, 0))

    # FRONT TIRE DYNAMICS
    vf_abs = np.linalg.norm(vf)
    vfL = np.array([-np.cos(delta), -np.sin(delta)]) * omega_F * R  # velocity of contact point
    vfL_abs = np.linalg.norm(vfL)

    if vf_abs > 0 or vfL_abs > 0:
        sf = (vf + vfL) / np.maximum(vf_abs, vfL_abs)  # slip vector
        sf_abs = np.linalg.norm(sf)
        ff_abs = np.sin(C_F * np.arctan(B_F * sf_abs / mu0))
        if sf_abs == 0:
            ff = np.zeros(2)
        else:
            ff = ff_abs * (-sf / sf_abs)
    else:
        ff = np.zeros(2)

    # REAR TIRE DYNAMICS
    vr_abs = np.linalg.norm(vr)
    vrL = np.array([-1, 0]) * omega_R * R  # velocity of contact point
    vrL_abs = np.linalg.norm(vrL)

    if vr_abs > 0 or vrL_abs > 0:
        sr = (vr + vrL) / np.maximum(vr_abs, vrL_abs)  # slip vector
        sr_abs = np.linalg.norm(sr)
        fr_abs = np.sin(C_R * np.arctan(B_R * sr_abs / mu0))
        if sr_abs == 0:
            fr = np.zeros(2)
        else:
            fr = fr_abs * (-sr / sr_abs)
    else:
        fr = np.zeros(2)

    # Calculate normal forces
    Fzf = l_R * m * g / (L + ff[0] * mu0 * h)
    Fzr = m * g - Fzf

    # FRONT FORCES
    Ff = ff * mu0 * Fzf

    # REAR FORCES
    Fr = fr * mu0 * Fzr

    # ACCELERATIONS
    dv = np.zeros(2)
    dv[0] = (Ff[0] + Fr[0]) / m + vy * omega  # longitudinal acceleration
    dv[1] = (Ff[1] + Fr[1]) / m - vx * omega  # lateral acceleration
    domega = (l_F * Ff[1] - l_R * Fr[1]) / J   # yaw acceleration

    # State derivatives
    dx = np.zeros(7)  # 7 states (6 vehicle + 1 for time integration)

    # Position derivatives
    cp = np.cos(psi)
    sp = np.sin(psi)
    dx[0] = cp * vx - sp * vy  # dX/dt
    dx[1] = sp * vx + cp * vy  # dY/dt
    dx[2] = omega              # dpsi/dt

    # Velocity derivatives
    dx[3] = dv[0]              # dvx/dt
    dx[4] = dv[1]              # dvy/dt
    dx[5] = domega             # domega/dt
    dx[6] = u[3] if len(u) > 3 else 0  # time derivative (for compatibility)

    # Outputs: [front_slip_vector, rear_slip_vector, steering_angle, front_forces, rear_forces]
    y = np.concatenate([ff, fr, [delta], Ff, Fr])

    return dx, y
