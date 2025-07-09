"""
B-spline path smoothing implementation.

This module provides functionality to smooth paths using B-spline curves,
offering local control and high smoothness for trajectory planning.

B-splines (Basis splines) are piecewise polynomial functions that provide:
- Local control: moving one control point affects only a local region
- High smoothness: continuous derivatives up to order n-1 for degree n
- Flexibility: support for different degrees and knot configurations

Author: Assistant
"""

import numpy as np
from numpy.typing import NDArray
from typing import List, Tuple, Optional, Union
from dataclasses import dataclass
import matplotlib.pyplot as plt
from scipy.interpolate import BSpline, splprep, splev
from .cubic_spline_smoother import PathPoint


class BSplineSmoother:
    """
    Path smoothing using B-spline curves.
    
    This class provides methods to:
    1. Smooth a given path using B-spline curves with configurable degree
    2. Support both open and closed (periodic) paths
    3. Provide local control through control points
    4. Calculate path properties (curvature, arc length, derivatives)
    5. Visualize the smoothing results
    """

    def __init__(self, 
                 degree: int = 3,
                 path_resolution: float = 0.1,
                 smoothing_factor: float = 0.0,
                 periodic: bool = False):
        """
        Initialize the B-spline path smoother.

        Args:
            degree: Degree of the B-spline (1=linear, 2=quadratic, 3=cubic, etc.)
            path_resolution: Distance between interpolated points (meters)
            smoothing_factor: Smoothing factor for spline fitting (0=interpolation, >0=approximation)
            periodic: Whether the path should be treated as closed/periodic
        """
        if degree < 1:
            raise ValueError("B-spline degree must be at least 1")
        
        self.degree = degree
        self.path_resolution = path_resolution
        self.smoothing_factor = smoothing_factor
        self.periodic = periodic
        self._reset()

    def _reset(self):
        """Reset internal state."""
        self.original_points: List[PathPoint] = []
        self.smoothed_points: List[PathPoint] = []
        self.control_points: Optional[NDArray[np.float64]] = None
        self.knots: Optional[NDArray[np.float64]] = None
        self.total_length: float = 0.0
        self.bspline_x: Optional[BSpline] = None
        self.bspline_y: Optional[BSpline] = None

    def smooth_path(self, x: List[float], y: List[float]) -> List[PathPoint]:
        """
        Smooth a path defined by waypoints using B-spline curves.

        Args:
            x: List of x coordinates
            y: List of y coordinates

        Returns:
            List of PathPoint objects containing the smoothed path
        """
        if len(x) != len(y):
            raise ValueError("x and y coordinate lists must have same length")
        if len(x) < self.degree + 1:
            raise ValueError(f"Need at least {self.degree + 1} points for degree {self.degree} B-spline")

        self._reset()
        
        # Store original points
        for xi, yi in zip(x, y):
            self.original_points.append(PathPoint(x=xi, y=yi))

        # Convert to numpy arrays
        points = np.array(list(zip(x, y)), dtype=np.float64)
        
        # Fit B-spline using scipy's splprep
        if self.periodic:
            # For periodic splines, ensure the path is closed
            if not (np.isclose(points[0, 0], points[-1, 0]) and 
                   np.isclose(points[0, 1], points[-1, 1])):
                # Add the first point at the end to close the loop
                points = np.vstack([points, points[0:1]])
        
        # Fit the B-spline
        tck, u = splprep([points[:, 0], points[:, 1]], 
                        k=min(self.degree, len(points) - 1),
                        s=self.smoothing_factor,
                        per=self.periodic)
        
        # Store spline information
        self.knots = tck[0]
        self.control_points = np.array([tck[1][0], tck[1][1]]).T
        
        # Create BSpline objects for easier derivative computation
        self.bspline_x = BSpline(tck[0], tck[1][0], tck[2])
        self.bspline_y = BSpline(tck[0], tck[1][1], tck[2])
        
        # Calculate total path length (approximate)
        u_fine = np.linspace(0, 1, 1000)
        x_fine, y_fine = splev(u_fine, tck)
        dx_fine = np.diff(x_fine)
        dy_fine = np.diff(y_fine)
        ds_fine = np.sqrt(dx_fine**2 + dy_fine**2)
        self.total_length = float(np.sum(ds_fine))
        
        # Generate smoothed path points with desired resolution
        num_points = max(int(self.total_length / self.path_resolution), 10)
        u_interp = np.linspace(0, 1, num_points)
        
        # Calculate positions and derivatives
        x_smooth, y_smooth = splev(u_interp, tck)
        dx_du, dy_du = splev(u_interp, tck, der=1)
        ddx_du2, ddy_du2 = splev(u_interp, tck, der=2)
        
        # Calculate arc length parameter for each point
        s_values = self._calculate_arc_lengths(u_interp, tck)
        
        for i, (xi, yi, dxi, dyi, ddxi, ddyi, si) in enumerate(
            zip(x_smooth, y_smooth, dx_du, dy_du, ddx_du2, ddy_du2, s_values)):
            
            # Calculate yaw (heading angle)
            yaw = float(np.arctan2(dyi, dxi))
            
            # Calculate curvature using the formula for parametric curves
            # κ = |x'y'' - y'x''| / (x'² + y'²)^(3/2)
            speed_squared = dxi**2 + dyi**2
            if speed_squared > 1e-10:  # Avoid division by zero
                curvature = float(abs(dxi * ddyi - dyi * ddxi) / (speed_squared**(3/2)))
            else:
                curvature = 0.0
            
            point = PathPoint(x=float(xi), y=float(yi), yaw=yaw, 
                            curvature=curvature, s=float(si))
            self.smoothed_points.append(point)

        return self.smoothed_points

    def _calculate_arc_lengths(self, u_params: NDArray[np.float64], tck) -> NDArray[np.float64]:
        """Calculate arc length values for given parameter values."""
        s_values = np.zeros_like(u_params)
        
        for i in range(1, len(u_params)):
            # Integrate from 0 to current parameter value
            u_seg = np.linspace(u_params[i-1], u_params[i], 10)
            x_seg, y_seg = splev(u_seg, tck)
            dx_seg = np.diff(x_seg)
            dy_seg = np.diff(y_seg)
            ds_seg = np.sqrt(dx_seg**2 + dy_seg**2)
            s_values[i] = s_values[i-1] + np.sum(ds_seg)
        
        return s_values

    def get_control_points(self) -> Optional[List[Tuple[float, float]]]:
        """
        Get the control points of the fitted B-spline.
        
        Returns:
            List of (x, y) control point coordinates, or None if no spline fitted
        """
        if self.control_points is None:
            return None
        return [(float(x), float(y)) for x, y in self.control_points]

    def evaluate_at_parameter(self, u: float) -> Optional[PathPoint]:
        """
        Evaluate the B-spline at a given parameter value.
        
        Args:
            u: Parameter value between 0 and 1
            
        Returns:
            PathPoint at the given parameter, or None if no spline fitted
        """
        if self.bspline_x is None or self.bspline_y is None:
            return None
        
        if not 0 <= u <= 1:
            raise ValueError("Parameter u must be between 0 and 1")
        
        # Position
        x = float(self.bspline_x(u))
        y = float(self.bspline_y(u))
        
        # First derivatives
        dx_du = float(self.bspline_x.derivative()(u))
        dy_du = float(self.bspline_y.derivative()(u))
        
        # Second derivatives
        ddx_du2 = float(self.bspline_x.derivative(2)(u))
        ddy_du2 = float(self.bspline_y.derivative(2)(u))
        
        # Calculate yaw and curvature
        yaw = float(np.arctan2(dy_du, dx_du))
        
        speed_squared = dx_du**2 + dy_du**2
        if speed_squared > 1e-10:
            curvature = float(abs(dx_du * ddy_du2 - dy_du * ddx_du2) / (speed_squared**(3/2)))
        else:
            curvature = 0.0
        
        return PathPoint(x=x, y=y, yaw=yaw, curvature=curvature)

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

        return float(np.max(curvatures)), float(np.mean(curvatures)), float(np.std(curvatures))

    def plot_path(self, show_curvature: bool = True, show_control_points: bool = True):
        """
        Visualize the original and smoothed paths.

        Args:
            show_curvature: Whether to show the curvature plot
            show_control_points: Whether to show B-spline control points
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
        ax_path.plot(smooth_x, smooth_y, '-', label=f'B-spline (degree {self.degree})', 
                    color='blue', linewidth=2)
        
        # Plot control points if requested
        if show_control_points and self.control_points is not None:
            ctrl_points = self.get_control_points()
            if ctrl_points:
                ctrl_x, ctrl_y = zip(*ctrl_points)
                ax_path.plot(ctrl_x, ctrl_y, 's', label='Control points', 
                           color='green', markersize=6, alpha=0.7)
                ax_path.plot(ctrl_x, ctrl_y, '--', color='green', alpha=0.5)
        
        # Plot arrows to show heading
        arrow_indices = np.linspace(0, len(self.smoothed_points)-1, 15, dtype=int)
        for idx in arrow_indices:
            p = self.smoothed_points[idx]
            if p.yaw is not None:
                dx = 0.5 * np.cos(p.yaw)
                dy = 0.5 * np.sin(p.yaw)
                ax_path.arrow(p.x, p.y, dx, dy, head_width=0.1, head_length=0.15, 
                            fc='orange', ec='orange', alpha=0.6)
        
        ax_path.set_aspect('equal')
        ax_path.grid(True)
        ax_path.legend()
        ax_path.set_title(f'B-spline Path Smoothing (degree={self.degree}, periodic={self.periodic})')
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

    def get_smoothness_metrics(self) -> Tuple[float, float]:
        """
        Calculate smoothness metrics for the path.
        
        Returns:
            Tuple of (curvature_variation, max_curvature_change_rate)
        """
        if not self.smoothed_points:
            raise ValueError("No smoothed path available. Call smooth_path first.")
        
        curvatures = np.array([p.curvature for p in self.smoothed_points if p.curvature is not None])
        
        if len(curvatures) < 2:
            return 0.0, 0.0
        
        # Curvature variation (standard deviation)
        curvature_variation = float(np.std(curvatures))
        
        # Maximum curvature change rate
        curvature_changes = np.abs(np.diff(curvatures))
        max_curvature_change_rate = float(np.max(curvature_changes)) if len(curvature_changes) > 0 else 0.0
        
        return curvature_variation, max_curvature_change_rate


def main():
    """Example usage of the BSplineSmoother."""
    # Create example path with sharp corners
    x = [0.0, 3.0, 6.0, 6.0, 3.0, 0.0, 0.0]
    y = [0.0, 0.0, 0.0, 3.0, 3.0, 3.0, 0.0]
    
    print("B-spline Path Smoothing Example")
    print("===============================")
    
    # Test different degrees
    for degree in [2, 3, 4]:
        print(f"\nTesting B-spline with degree {degree}:")
        
        smoother = BSplineSmoother(degree=degree, path_resolution=0.1)
        smoothed_path = smoother.smooth_path(x, y)
        
        max_curv, mean_curv, std_curv = smoother.get_curvature_stats()
        curv_var, max_curv_change = smoother.get_smoothness_metrics()
        
        print(f"  Total Length: {smoother.total_length:.2f} m")
        print(f"  Maximum Curvature: {max_curv:.3f} 1/m")
        print(f"  Mean Curvature: {mean_curv:.3f} 1/m")
        print(f"  Curvature Std Dev: {std_curv:.3f} 1/m")
        print(f"  Curvature Variation: {curv_var:.3f}")
        print(f"  Max Curvature Change Rate: {max_curv_change:.3f}")
        
        smoother.plot_path(show_curvature=True, show_control_points=True)


if __name__ == "__main__":
    main() 