#!/usr/bin/env python3

import sys
import os

# Add the current directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import smc_controller
    print("✓ Import successful!")

    # Test basic components
    from smc_controller import KinSlidingController, VehicleModel, CARParameters
    print("✓ Core components imported successfully!")

    # Test instantiation
    controller = KinSlidingController()
    params = CARParameters.default()
    model = VehicleModel(params)
    print("✓ Components instantiated successfully!")

    print("\n🎉 SMC Controller package is working correctly!")

except ImportError as e:
    print(f"✗ Import failed: {e}")
    sys.exit(1)
except Exception as e:
    print(f"✗ Error: {e}")
    sys.exit(1)
