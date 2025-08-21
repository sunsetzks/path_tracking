#!/usr/bin/env python3

import sys
import os

# Add the smc_controller directory to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'smc_controller'))

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
    import traceback
    traceback.print_exc()
    sys.exit(1)
except Exception as e:
    print(f"✗ Error: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
