#!/usr/bin/env python3
"""
Basic test script for pure Python Dubins implementation
"""

import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

import dubins

def test_basic_functionality():
    """Test basic Dubins path functionality"""
    print("Testing basic Dubins path functionality...")
    
    # Test case 1: Simple straight line
    q0 = (0.0, 0.0, 0.0)
    q1 = (4.0, 0.0, 0.0)
    rho = 1.0
    
    path = dubins.shortest_path(q0, q1, rho)
    print(f"Path length: {path.path_length()}")
    print(f"Path type: {path.path_type()}")
    
    # Test path sampling
    points, distances = dubins.path_sample(q0, q1, rho, step_size=0.5)
    print(f"Sampled {len(points)} points")
    
    # Test backward motion
    print("Testing backward motion...")
    backward_points, backward_distances = dubins.backward_path_sample(q0, q1, rho, step_size=0.5)
    print(f"Backward sampled {len(backward_points)} points")
    
    # Test case 2: More complex path
    q0 = (0.0, 0.0, 0.0)
    q1 = (3.0, 4.0, numpy.pi/4)
    rho = 1.0
    
    path2 = dubins.shortest_path(q0, q1, rho)
    print(f"Complex path length: {path2.path_length()}")
    print(f"Complex path type: {path2.path_type()}")
    
    # Test specific path types
    lsl_path = dubins.path(q0, q1, rho, dubins.LSL)
    rsr_path = dubins.path(q0, q1, rho, dubins.RSR)
    
    print(f"LSL path exists: {lsl_path is not None}")
    print(f"RSR path exists: {rsr_path is not None}")
    
    print("All tests passed!")

if __name__ == "__main__":
    try:
        import numpy
        test_basic_functionality()
    except ImportError as e:
        print(f"Import error: {e}")
        print("Please install numpy: pip install numpy")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()