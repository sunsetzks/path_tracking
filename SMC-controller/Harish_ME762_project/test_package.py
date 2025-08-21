#!/usr/bin/env python3
"""Test script to verify the SMC simulation package works correctly."""

import sys
import os

# Add the current directory to Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    # Test the package imports
    import Python_SMC_Simulation.controllers.kinematic_sliding_controller
    import Python_SMC_Simulation.models.vehicle_parameters
    import Python_SMC_Simulation.trajectory.trajectory
    import Python_SMC_Simulation.utils.math_utils

    print("✅ All package imports successful!")
    print("✅ SMC Simulation package is working correctly!")

    # Test that we can access the main test function
    from Python_SMC_Simulation.test_smc_simulation import test_smc_simulation
    print("✅ Main test function accessible!")

except ImportError as e:
    print(f"❌ Import error: {e}")
    exit(1)
except Exception as e:
    print(f"❌ Other error: {e}")
    exit(1)
