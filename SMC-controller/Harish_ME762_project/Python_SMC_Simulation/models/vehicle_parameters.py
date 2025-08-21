"""
Vehicle parameters for SMC simulation
Translated from MATLAB CARparameters.m
"""

import numpy as np


def get_vehicle_parameters():
    """
    Returns vehicle parameters dictionary equivalent to MATLAB CARparameters()

    Returns:
        dict: Vehicle parameters including mass, inertia, dimensions, tire parameters
    """
    p = {}

    # Vehicle physical parameters
    p['m'] = 1750.0          # mass [kg]
    p['J'] = 2500.0          # moment of inertia [kg*m^2]
    p['l_F'] = 1.43          # distance from CG to front axle [m]
    p['l_R'] = 1.27          # distance from CG to rear axle [m]
    p['L'] = p['l_F'] + p['l_R']  # wheelbase [m]
    p['h'] = 0.5             # height of CG [m]

    # Tire parameters
    p['mu0'] = 1.0           # friction coefficient
    p['B_F'] = 10.4          # front tire stiffness factor
    p['C_F'] = 1.3           # front tire shape factor
    p['Kfz_F'] = 0.1         # front vertical stiffness
    p['B_R'] = 21.4          # rear tire stiffness factor
    p['C_R'] = 1.1           # rear tire shape factor
    p['Kfz_R'] = 0.1         # rear vertical stiffness
    p['R'] = 0.32            # wheel radius [m]
    p['Jwheel'] = 1.2        # wheel inertia [kg*m^2]

    # Linear approximation values for control
    p['cf'] = 1.0299e5       # front cornering stiffness [N/rad]
    p['cr'] = 1.8917e5       # rear cornering stiffness [N/rad]

    # Maximum values
    p['MAX_delta'] = np.pi/4  # maximum steering angle [rad]

    # Controller gains
    p['kA0'] = 5.0
    p['kA1'] = 3.3541

    return p


def get_default_options():
    """
    Returns default simulation options equivalent to MATLAB default_options()

    Returns:
        dict: Default simulation options
    """
    options = {}

    options['DISPLAY_OUTPUT'] = 1
    options['V'] = 0.0  # noise level
    options['p'] = get_vehicle_parameters()  # real parameters
    options['pc'] = get_vehicle_parameters()  # parameters known to controller
    options['stepSize'] = 0.02  # time step [s]
    options['types'] = ['vmodel_A', 'vmodel_K']  # model types

    return options
