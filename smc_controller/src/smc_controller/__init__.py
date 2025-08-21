"""
Sliding Mode Control (SMC) for Vehicle Trajectory Tracking

Converted from MATLAB SMC simulation code to Python.
Provides kinematic and dynamic sliding mode controllers for path tracking.
"""

from .vehicle_model import VehicleModel
from .trajectory import Trajectory, create_test_trajectory
from .controllers import KinSlidingController
from .parameters import CARParameters, default_options
from .simulation import simulate
from .utils import *

__version__ = "1.0.0"

def main() -> None:
    print("Sliding Mode Control for Vehicle Trajectory Tracking")
