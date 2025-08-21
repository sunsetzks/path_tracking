"""
Kinematic Sliding Mode Controller
Translated from MATLAB KinSliding.m
"""

import numpy as np
from ..utils.math_utils import sgn, saturate, get_local_error
from ..models.input_mappings import map0, AXController_pacejka


class KinSliding:
    """
    Kinematic Sliding Mode Controller for path tracking
    """

    def __init__(self):
        self.name = 'KINSM'
        self.control_point = 'REAR'
        self.compute_input = None
        self.init = self._init_controller

    def _init_controller(self, model, options):
        """
        Initialize the controller
        """
        # Determine which input mapping to use based on model type
        if hasattr(model, '__name__') and model.__name__ == 'vmodel_A':
            mapInput = AXController_pacejka
        else:
            mapInput = map0

        # Set the compute_input function
        self.compute_input = lambda *args: self._compute_input(mapInput, *args)
        return self

    def _compute_input(self, mapInput, *args):
        """
        Compute control input
        Args can be:
            - (state, tau, t, p) for continuous controller
            - (state, tau, t, p, newTick, intState) for discrete controller
        """
        if len(args) >= 4:
            state, tau, t, p = args[0], args[1], args[2], args[3]
        else:
            raise ValueError("Insufficient arguments for compute_input")

        # Reference trajectory values
        pd = np.array([tau.X(t), tau.Y(t)])          # desired position
        vd = np.array([tau.dX(t), tau.dY(t)])        # desired velocity
        thetad = tau.theta(t)                        # desired heading
        omegad = tau.dtheta(t)                       # desired yaw rate
        domegad = tau.ddtheta(t)                     # desired yaw acceleration
        lad = tau.a(t)                               # desired acceleration magnitude

        # Control parameters
        k0 = 0.05
        k1 = 0.25
        k2 = 0.5

        # Get local errors
        lambda_offset = -p['l_R']  # rear axle offset
        e, de = get_local_error(state, pd, vd, thetad, omegad, lambda_offset, 2)

        # Current velocity
        vr = state[3]  # vx == rear wheel velocity

        # Sliding surfaces
        s1 = de[0] + k1 * e[0]
        s2 = de[1] + k2 * e[1] + k0 * sgn(e[1]) * e[2]

        # Controller gains
        p1 = 1
        p2 = 3
        q1 = 1
        q2 = 3

        # Control law
        dv_c = 1 / np.cos(e[2]) * (-q1 * s1 - p1 * sgn(s1) - k1 * de[0] - domegad * e[1]
                                  - omegad * de[1] + vr * de[2] * np.sin(e[2]) + lad)

        # Steering angle calculation
        L = p['L']
        delta = np.arctan(L / vr * omegad + L / (vr * (vr * np.cos(e[2]) + k0 * sgn(e[1]))) *
                         (-q2 * s2 - p2 * sgn(s2) - k2 * de[1] - dv_c * np.sin(e[2]) +
                          domegad * e[0] + omegad * de[0]))

        # Apply steering limits
        delta = saturate(delta, -p['MAX_delta'], p['MAX_delta'])

        # Map to control inputs
        u = mapInput(dv_c, state, delta, p)

        # Add extra dimension for compatibility
        if len(u) < 4:
            u = np.append(u, 0)

        return u
