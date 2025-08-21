#!/usr/bin/env python3
"""
Simple test script to run the SMC simulation
"""

import sys
import os

# Add the current directory to Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from .test_smc_simulation import test_smc_simulation


def main():
    """Main function to run test"""
    print("Testing SMC Simulation...")

    try:
        # Run single lane change test
        print("\n=== Single Lane Change Test ===")
        benchmark1 = test_smc_simulation('single_lane_change')

        # Run double lane change test
        print("\n=== Double Lane Change Test ===")
        benchmark2 = test_smc_simulation('double_lane_change')

        print("\n✅ All tests completed successfully!")

        return 0

    except Exception as e:
        print(f"\n❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    exit(main())
