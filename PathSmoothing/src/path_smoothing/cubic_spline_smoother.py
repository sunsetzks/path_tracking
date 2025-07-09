"""
Path smoothing using cubic spline interpolation.

This module provides functionality to smooth paths using cubic spline interpolation,
ensuring continuous first and second derivatives for smooth motion planning.

Author: Assistant
"""

import numpy as np
from numpy.typing import NDArray
from typing import List, Tuple, Optional
from dataclasses import dataclass
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline


@dataclass
class PathPoint:
    """A point on the path with optional derivatives."""
    x: float
    y: float
    yaw: Optional[float] = None  # heading angle
    curvature: Optional[float] = None  # path curvature
    s: Optional[float] = None  # distance along path


class CubicSplineSmoother:
    """
    Path smoothing using cubic spline interpolation.
    
    This class provides methods to:
    1. Smooth a given path using cubic splines
    2. Ensure continuous derivatives up to 2nd order
    3. Visualize the smoothing results
    4. Calculate path properties (curvature, arc length)
    """

    def __init__(self, path_resolution: float = 0.1):
        """
        Initialize the path smoother.

        Args:
            path_resolution: Distance between interpolated points (meters)
        """
        self.path_resolution = path_resolution
        self._reset()

    def _reset(self):
        """Reset internal state."""
        self.original_points: List[PathPoint] = []
        self.smoothed_points: List[PathPoint] = []
        self.total_length: float = 0.0

    def smooth_path(self, x: List[float], y: List[float]) -> List[PathPoint]:
        """
        Smooth a path defined by waypoints using cubic spline interpolation.

        Args:
            x: List of x coordinates
            y: List of y coordinates

        Returns:
            List of PathPoint objects containing the smoothed path
        """
        if len(x) != len(y):
            raise ValueError("x and y coordinate lists must have same length")
        if len(x) < 2:
            raise ValueError("Need at least 2 points to create a path")

        self._reset()
        
        # Store original points
        for xi, yi in zip(x, y):
            self.original_points.append(PathPoint(x=xi, y=yi))

        # Convert to numpy arrays
        x_arr = np.array(x, dtype=np.float64)
        y_arr = np.array(y, dtype=np.float64)

        # Calculate path length for parameterization
        dx = np.diff(x_arr)
        dy = np.diff(y_arr)
        ds = np.sqrt(dx**2 + dy**2)
        s = np.zeros(len(x))
        s[1:] = np.cumsum(ds)
        self.total_length = float(s[-1])

        # Create cubic splines for x and y coordinates
        spline_x = self._create_spline(s, x_arr)
        spline_y = self._create_spline(s, y_arr)

        # Generate smoothed path points
        s_interp = np.arange(0, self.total_length, self.path_resolution)
        if s_interp[-1] < self.total_length:
            s_interp = np.append(s_interp, self.total_length)

        for si in s_interp:
            # Position
            xi = float(spline_x(si))
            yi = float(spline_y(si))
            
            # First derivatives
            dxi = float(spline_x.derivative()(si))
            dyi = float(spline_y.derivative()(si))
            
            # Second derivatives
            ddxi = float(spline_x.derivative(2)(si))
            ddyi = float(spline_y.derivative(2)(si))
            
            # Calculate yaw (heading angle)
            yaw = float(np.arctan2(dyi, dxi))
            
            # Calculate curvature
            curvature = float((dyi * ddxi - dxi * ddyi) / ((dxi**2 + dyi**2)**(3/2)))
            
            point = PathPoint(x=xi, y=yi, yaw=yaw, curvature=curvature, s=float(si))
            self.smoothed_points.append(point)

        return self.smoothed_points

    def _create_spline(self, x: NDArray[np.float64], y: NDArray[np.float64]) -> CubicSpline:
        """Create a cubic spline with natural boundary conditions."""
        return CubicSpline(x, y, bc_type='natural')

    def get_curvature_stats(self) -> Tuple[float, float, float]:
        """
        Calculate curvature statistics of the smoothed path.

        Returns:
            Tuple of (max_curvature, mean_curvature, curvature_std)
        """
        if not self.smoothed_points:
            raise ValueError("No smoothed path available. Call smooth_path first.")

        curvatures = np.array([p.curvature for p in self.smoothed_points if p.curvature is not None])
        if len(curvatures) == 0:
            return 0.0, 0.0, 0.0

        return float(np.max(np.abs(curvatures))), float(np.mean(np.abs(curvatures))), float(np.std(curvatures))

    def plot_path(self, show_curvature: bool = True):
        """
        Visualize the original and smoothed paths.

        Args:
            show_curvature: Whether to show the curvature plot
        """
        if not self.smoothed_points:
            raise ValueError("No smoothed path available. Call smooth_path first.")

        fig = plt.figure(figsize=(15, 5 if show_curvature else 8))
        
        # Path plot
        ax_path = plt.subplot(1 + int(show_curvature), 1, 1)
        
        # Plot original points
        orig_x = [p.x for p in self.original_points]
        orig_y = [p.y for p in self.original_points]
        ax_path.plot(orig_x, orig_y, 'o', label='Original points', color='red', markersize=8)
        
        # Plot smoothed path
        smooth_x = [p.x for p in self.smoothed_points]
        smooth_y = [p.y for p in self.smoothed_points]
        ax_path.plot(smooth_x, smooth_y, '-', label='Smoothed path', color='blue', linewidth=2)
        
        # Plot arrows to show heading
        arrow_indices = np.linspace(0, len(self.smoothed_points)-1, 20, dtype=int)
        for idx in arrow_indices:
            p = self.smoothed_points[idx]
            if p.yaw is not None:
                dx = np.cos(p.yaw)
                dy = np.sin(p.yaw)
                ax_path.arrow(p.x, p.y, dx, dy, head_width=0.1, head_length=0.2, fc='g', ec='g', alpha=0.5)
        
        ax_path.set_aspect('equal')
        ax_path.grid(True)
        ax_path.legend()
        ax_path.set_title('Path Comparison')
        ax_path.set_xlabel('X [m]')
        ax_path.set_ylabel('Y [m]')

        if show_curvature:
            # Curvature plot
            ax_curv = plt.subplot(2, 1, 2)
            s_values = np.array([p.s for p in self.smoothed_points if p.s is not None])
            curvatures = np.array([p.curvature for p in self.smoothed_points if p.curvature is not None])
            ax_curv.plot(s_values, curvatures, '-', color='blue', linewidth=2)
            ax_curv.grid(True)
            ax_curv.set_title('Path Curvature')
            ax_curv.set_xlabel('Distance along path [m]')
            ax_curv.set_ylabel('Curvature [1/m]')

        plt.tight_layout()
        plt.show()


def main():
    """Example usage of the CubicSplineSmoother."""
    # Create example path (figure-8 shape)
    t = np.linspace(0, 2*np.pi, 20)
    x = 10 * np.sin(t)
    y = 5 * np.sin(2*t)

    # Create and use the smoother
    smoother = CubicSplineSmoother(path_resolution=0.1)
    smoothed_path = smoother.smooth_path(x.tolist(), y.tolist())

    # Print path statistics
    max_curv, mean_curv, std_curv = smoother.get_curvature_stats()
    print(f"Path Statistics:")
    print(f"Total Length: {smoother.total_length:.2f} m")
    print(f"Maximum Curvature: {max_curv:.2f} 1/m")
    print(f"Mean Curvature: {mean_curv:.2f} 1/m")
    print(f"Curvature Std Dev: {std_curv:.2f} 1/m")

    # Visualize the results
    smoother.plot_path(show_curvature=True)


if __name__ == "__main__":
    main() 