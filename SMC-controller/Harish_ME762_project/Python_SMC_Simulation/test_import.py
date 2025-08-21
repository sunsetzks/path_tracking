#!/usr/bin/env python3
"""Test script to verify package imports work correctly."""

import sys
import os

# Add the current directory to Python path to enable imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import controllers.kinematic_sliding_controller
    import models.vehicle_parameters
    import trajectory.trajectory
    import utils.math_utils
    print("✅ All package imports successful!")
    print("✅ SMC Simulation package is working correctly!")
except ImportError as e:
    print(f"❌ Import error: {e}")
    exit(1)
