#!/usr/bin/env python3
"""
Test script for PlotUtils functionality

This script tests the new matplotlib-like plot and scatter utilities
to ensure they work correctly.

Author: Generated for path_tracking project
Date: 2025-01-27
"""

import sys
import os
import numpy as np

# Add the package to the path
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from forglove_helper.channel_utils import PlotUtils

def test_plot_2d():
    """Test 2D plotting functionality"""
    print("Testing 2D plot...")

    x = np.linspace(0, 10, 50)
    y = np.sin(x)

    try:
        primitives = PlotUtils.plot(x, y, color=(1.0, 0.0, 0.0, 1.0))
        print(f"✓ 2D plot created successfully: {len(primitives)} primitives")
        return True
    except Exception as e:
        print(f"✗ 2D plot failed: {e}")
        return False

def test_plot_3d():
    """Test 3D plotting functionality"""
    print("Testing 3D plot...")

    t = np.linspace(0, 4*np.pi, 100)
    x = np.cos(t)
    y = np.sin(t)
    z = t / 2

    try:
        primitives = PlotUtils.plot(x, y, z, color=(0.0, 1.0, 0.0, 1.0))
        print(f"✓ 3D plot created successfully: {len(primitives)} primitives")
        return True
    except Exception as e:
        print(f"✗ 3D plot failed: {e}")
        return False

def test_scatter_2d():
    """Test 2D scatter plot functionality"""
    print("Testing 2D scatter plot...")

    x = np.random.normal(0, 2, 20)
    y = np.random.normal(0, 2, 20)

    try:
        primitives = PlotUtils.scatter(x, y, marker='circle', s=50)
        print(f"✓ 2D scatter plot created successfully: {len(primitives)} primitives")
        return True
    except Exception as e:
        print(f"✗ 2D scatter plot failed: {e}")
        return False

def test_scatter_3d():
    """Test 3D scatter plot functionality"""
    print("Testing 3D scatter plot...")

    x = np.random.normal(0, 1, 15)
    y = np.random.normal(0, 1, 15)
    z = np.random.normal(0, 1, 15)

    try:
        primitives = PlotUtils.scatter(x, y, z, marker='square', s=30)
        print(f"✓ 3D scatter plot created successfully: {len(primitives)} primitives")
        return True
    except Exception as e:
        print(f"✗ 3D scatter plot failed: {e}")
        return False

def test_color_parsing():
    """Test color parsing functionality"""
    print("Testing color parsing...")

    x = [1.0, 2.0, 3.0]
    y = [1.0, 4.0, 2.0]

    try:
        # Test string color
        primitives1 = PlotUtils.scatter(x, y, c='red')
        print(f"✓ String color parsing works: {len(primitives1)} primitives")

        # Test RGB tuple
        primitives2 = PlotUtils.scatter(x, y, c=(0.5, 0.5, 0.5, 1.0))
        print(f"✓ RGB tuple color parsing works: {len(primitives2)} primitives")

        # Test RGBA tuple
        primitives3 = PlotUtils.scatter(x, y, c=(1.0, 0.0, 0.0, 0.8))
        print(f"✓ RGBA tuple color parsing works: {len(primitives3)} primitives")

        # Test list of colors
        colors = [(1.0, 0.0, 0.0, 1.0), (0.0, 1.0, 0.0, 1.0), (0.0, 0.0, 1.0, 1.0)]
        primitives4 = PlotUtils.scatter(x, y, c=colors)
        print(f"✓ List of colors parsing works: {len(primitives4)} primitives")

        return True
    except Exception as e:
        print(f"✗ Color parsing failed: {e}")
        return False

def test_axes_creation():
    """Test coordinate axes creation"""
    print("Testing axes creation...")

    try:
        primitives = PlotUtils.create_axes(xlim=(-5, 5), ylim=(-3, 3), zlim=(-2, 2))
        print(f"✓ 3D axes created successfully: {len(primitives)} primitives")

        primitives_2d = PlotUtils.create_axes(xlim=(-5, 5), ylim=(-3, 3))
        print(f"✓ 2D axes created successfully: {len(primitives_2d)} primitives")

        return True
    except Exception as e:
        print(f"✗ Axes creation failed: {e}")
        return False

def test_marker_styles():
    """Test different marker styles"""
    print("Testing marker styles...")

    x = [1.0, 2.0, 3.0, 4.0]
    y = [1.0, 4.0, 2.0, 3.0]

    try:
        # Test different marker styles
        circle_primitives = PlotUtils.scatter(x, y, marker='circle')
        print(f"✓ Circle markers: {len(circle_primitives)} primitives")

        square_primitives = PlotUtils.scatter(x, y, marker='square')
        print(f"✓ Square markers: {len(square_primitives)} primitives")

        diamond_primitives = PlotUtils.scatter(x, y, marker='diamond')
        print(f"✓ Diamond markers: {len(diamond_primitives)} primitives")

        # Test plot with markers
        line_primitives = PlotUtils.plot(x, y, marker='line')
        print(f"✓ Line plot: {len(line_primitives)} primitives")

        return True
    except Exception as e:
        print(f"✗ Marker styles test failed: {e}")
        return False

def main():
    """Run all tests"""
    print("Running PlotUtils Tests")
    print("=" * 50)

    tests = [
        test_plot_2d,
        test_plot_3d,
        test_scatter_2d,
        test_scatter_3d,
        test_color_parsing,
        test_axes_creation,
        test_marker_styles
    ]

    passed = 0
    failed = 0

    for test in tests:
        if test():
            passed += 1
        else:
            failed += 1
        print()

    print("=" * 50)
    print(f"Tests completed: {passed} passed, {failed} failed")

    if failed == 0:
        print("🎉 All tests passed!")
        return 0
    else:
        print("❌ Some tests failed")
        return 1

if __name__ == "__main__":
    exit(main())
