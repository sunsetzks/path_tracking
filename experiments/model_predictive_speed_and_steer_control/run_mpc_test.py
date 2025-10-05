#!/usr/bin/env python3
"""
Simple script to run MPC solver tests.

Usage:
    python run_mpc_test.py
"""

import sys
import pathlib

# Add parent directory to path for imports
sys.path.append(str(pathlib.Path(__file__).parent.parent))

from test_mpc_solver import run_comprehensive_tests, test_constraint_violations

if __name__ == "__main__":
    print("MPC Solver Test Suite")
    print("=" * 50)
    
    try:
        # Run comprehensive tests
        run_comprehensive_tests()
        
        # Test constraint handling
        test_constraint_violations()
        
        print("\n" + "=" * 50)
        print("All tests completed successfully!")
        
    except Exception as e:
        print(f"Error during testing: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
