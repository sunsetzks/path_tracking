"""
Example script demonstrating the usage of CubicSplineSmoother for path smoothing.

This script shows different examples of path smoothing:
1. Simple path with sharp corners
2. Figure-8 shape path
3. Spiral path
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

from path_smoothing import CubicSplineSmoother


def example_sharp_corners():
    """Example of smoothing a path with sharp corners."""
    # Create a path with sharp corners
    x = [0.0, 5.0, 5.0, 10.0, 10.0, 15.0]
    y = [0.0, 0.0, 5.0, 5.0, 0.0, 0.0]

    # Create and use the smoother
    smoother = CubicSplineSmoother(path_resolution=0.1)
    smoothed_path = smoother.smooth_path(x, y)

    print("\nSharp Corners Path Statistics:")
    max_curv, mean_curv, std_curv = smoother.get_curvature_stats()
    print(f"Total Length: {smoother.total_length:.2f} m")
    print(f"Maximum Curvature: {max_curv:.2f} 1/m")
    print(f"Mean Curvature: {mean_curv:.2f} 1/m")
    print(f"Curvature Std Dev: {std_curv:.2f} 1/m")

    smoother.plot_path(show_curvature=True)


def example_figure_eight():
    """Example of smoothing a figure-8 shaped path."""
    # Create a figure-8 shape
    t = np.linspace(0, 2*np.pi, 20)
    x = 10 * np.sin(t)
    y = 5 * np.sin(2*t)

    # Create and use the smoother
    smoother = CubicSplineSmoother(path_resolution=0.1)
    smoothed_path = smoother.smooth_path(x.tolist(), y.tolist())

    print("\nFigure-8 Path Statistics:")
    max_curv, mean_curv, std_curv = smoother.get_curvature_stats()
    print(f"Total Length: {smoother.total_length:.2f} m")
    print(f"Maximum Curvature: {max_curv:.2f} 1/m")
    print(f"Mean Curvature: {mean_curv:.2f} 1/m")
    print(f"Curvature Std Dev: {std_curv:.2f} 1/m")

    smoother.plot_path(show_curvature=True)


def example_spiral():
    """Example of smoothing a spiral path."""
    # Create a spiral
    t = np.linspace(0, 6*np.pi, 30)
    r = t
    x = r * np.cos(t)
    y = r * np.sin(t)

    # Create and use the smoother
    smoother = CubicSplineSmoother(path_resolution=0.1)
    smoothed_path = smoother.smooth_path(x.tolist(), y.tolist())

    print("\nSpiral Path Statistics:")
    max_curv, mean_curv, std_curv = smoother.get_curvature_stats()
    print(f"Total Length: {smoother.total_length:.2f} m")
    print(f"Maximum Curvature: {max_curv:.2f} 1/m")
    print(f"Mean Curvature: {mean_curv:.2f} 1/m")
    print(f"Curvature Std Dev: {std_curv:.2f} 1/m")

    smoother.plot_path(show_curvature=True)


def main():
    """Run all examples."""
    print("Cubic Spline Path Smoothing Examples")
    print("====================================")

    # Example 1: Path with sharp corners
    print("\nExample 1: Path with Sharp Corners")
    example_sharp_corners()

    # Example 2: Figure-8 shape
    print("\nExample 2: Figure-8 Shape")
    example_figure_eight()

    # Example 3: Spiral path
    print("\nExample 3: Spiral Path")
    example_spiral()


if __name__ == "__main__":
    main() 