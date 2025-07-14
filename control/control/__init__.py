"""
Control algorithms package for path tracking.

This package contains various control algorithms including:
- Sliding Mode Control (SMC) for robust position and velocity control
"""

from .sliding_mode_control import SlidingModeController, VehicleSystem, SMCSimulator

__all__ = ['SlidingModeController', 'VehicleSystem', 'SMCSimulator'] 