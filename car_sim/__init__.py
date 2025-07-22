"""
Vehicle Dynamics Simulation Package

This package provides a comprehensive vehicle dynamics simulation using the bicycle model.
"""

from .vehicle_dynamics import VehicleSimulation, VehicleParameters, VehicleState
from .visualization import plot_vehicle_trajectory, plot_state_variables, animate_vehicle_simulation

__version__ = "0.1.0"
__author__ = "Roo"
__description__ = "Vehicle dynamics simulation using bicycle model"

# Package level imports
__all__ = [
    'VehicleSimulation',
    'VehicleParameters',
    'VehicleState',
    'plot_vehicle_trajectory',
    'plot_state_variables',
    'animate_vehicle_simulation'
]