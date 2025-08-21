"""
Vehicle parameters and configuration.

Converted from MATLAB CARparameters.m
"""

import numpy as np
from dataclasses import dataclass
from typing import Dict, Any


@dataclass
class CARParameters:
    """
    Vehicle parameters for the bicycle model.

    Attributes:
        m: Vehicle mass [kg]
        J: Vehicle moment of inertia [kg*m^2]
        l_F: Distance from CG to front axle [m]
        l_R: Distance from CG to rear axle [m]
        L: Wheelbase (l_F + l_R) [m]
        h: Height of CG [m]
        mu0: Road friction coefficient
        B_F: Front tire stiffness factor
        C_F: Front tire shape factor
        Kfz_F: Front vertical stiffness
        B_R: Rear tire stiffness factor
        C_R: Rear tire shape factor
        Kfz_R: Rear vertical stiffness
        R: Wheel radius [m]
        Jwheel: Wheel inertia [kg*m^2]
        cf: Front cornering stiffness [N/rad]
        cr: Rear cornering stiffness [N/rad]
        MAX_delta: Maximum steering angle [rad]
        kA0: Controller gain A0
        kA1: Controller gain A1
    """

    # Vehicle physical parameters
    m: float = 1750.0
    J: float = 2500.0
    l_F: float = 1.43
    l_R: float = 1.27
    h: float = 0.5
    mu0: float = 1.0

    # Tire parameters
    B_F: float = 10.4
    C_F: float = 1.3
    Kfz_F: float = 0.1
    B_R: float = 21.4
    C_R: float = 1.1
    Kfz_R: float = 0.1
    R: float = 0.32
    Jwheel: float = 1.2

    # Control parameters
    cf: float = 1.0299e5
    cr: float = 1.8917e5
    MAX_delta: float = np.pi / 4

    # Controller gains
    kA0: float = 5.0
    kA1: float = 3.3541

    def __post_init__(self):
        """Calculate derived parameters after initialization."""
        self.L = self.l_F + self.l_R

    @classmethod
    def default(cls) -> 'CARParameters':
        """Create default CAR parameters."""
        return cls()

    def to_dict(self) -> Dict[str, Any]:
        """Convert parameters to dictionary."""
        return {
            'm': self.m, 'J': self.J, 'l_F': self.l_F, 'l_R': self.l_R,
            'L': self.L, 'h': self.h, 'mu0': self.mu0,
            'B_F': self.B_F, 'C_F': self.C_F, 'Kfz_F': self.Kfz_F,
            'B_R': self.B_R, 'C_R': self.C_R, 'Kfz_R': self.Kfz_R,
            'R': self.R, 'Jwheel': self.Jwheel,
            'cf': self.cf, 'cr': self.cr, 'MAX_delta': self.MAX_delta,
            'kA0': self.kA0, 'kA1': self.kA1
        }


class SimulationOptions:
    """
    Simulation options and configuration.

    Attributes:
        DISPLAY_OUTPUT: Whether to display simulation output
        V: Noise level
        stepSize: Simulation time step
        p: Real vehicle parameters
        pc: Parameters known to controller
        types: Available model types
    """

    def __init__(self):
        self.DISPLAY_OUTPUT = 1
        self.V = 0  # noise
        self.stepSize = 0.02

        # Parameters
        self.p = CARParameters.default()
        self.pc = CARParameters.default()

        # Model types
        self.types = ['vmodel_A', 'vmodel_K']


def default_options() -> SimulationOptions:
    """Create default simulation options."""
    return SimulationOptions()
