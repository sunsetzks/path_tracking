# PathSmoothing/examples/method_comparison.py
"""
Comparison of path smoothing methods (B-spline, cubic spline, gradient-based).
"""

import numpy as np
import matplotlib.pyplot as plt
from path_smoothing import (
    BSplineSmoother,
    CubicSplineSmoother,
    GradientPathSmoother,
    PathPoint
)

def create_test_paths():
    """Generate different test paths for comparison."""
    paths = {}
    
    # 1. Sharp rectangular path
    x_rect = [0, 5, 5, 0, 0]
    y_rect = [0, 0, 3, 3, 0]
    paths['rectangular'] = (x_rect, y_rect)
    
    # 2. Smooth figure-8 path
    t = np.linspace(0, 2*np.pi, 20)
    x_fig8 = 5 * np.sin(t)
    y_fig8 = 3 * np.sin(2*t)
    paths['figure8'] = (x_fig8.tolist(), y_fig8.tolist())
    
    # 3. Random noisy path
    x_rand = np.linspace(0, 10, 15)
    y_rand = np.sin(x_rand) + np.random.normal(0, 0.2, len(x_rand))
    paths['random'] = (x_rand.tolist(), y_rand.tolist())
    
    # 4. Spiral path
    theta = np.linspace(0, 4*np.pi, 30)
    r = theta
    x_spiral = r * np.cos(theta)
    y_spiral = r * np.sin(theta)
    paths['spiral'] = (x_spiral.tolist(), y_spiral.tolist())
    
    return paths

def compare_methods(x, y, path_name):
    """Compare smoothing methods on a single path."""
    # Initialize smoothers
    bspline = BSplineSmoother(degree=3, path_resolution=0.1)
    cubic = CubicSplineSmoother(path_resolution=0.1)
    gradient = GradientPathSmoother(alpha=0.1, beta=0.2, learning_rate=0.01)
    
    # Apply smoothing
    bspline_path = bspline.smooth_path(x, y)
    cubic_path = cubic.smooth_path(x, y)
    
    # Gradient smoother needs different input format
    xy_pairs = list(zip(x, y))
    gradient_path = gradient.smooth_path(xy_pairs)
    gradient_x, gradient_y = zip(*gradient_path)
    
    # Get curvature statistics
    bspline_stats = bspline.get_curvature_stats()
    cubic_stats = cubic.get_curvature_stats()
    gradient_curv = gradient.get_path_curvature(gradient_path)
    gradient_stats = (
        max(abs(c) for c in gradient_curv),
        np.mean([abs(c) for c in gradient_curv]),
        np.std([abs(c) for c in gradient_curv])
    )
    
    # Plot results
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))
    fig.suptitle(f'Path Smoothing Comparison - {path_name}', fontsize=16)
    
    # Path plot
    ax1.plot(x, y, 'ko-', label='Original', markersize=6)
    ax1.plot([p.x for p in bspline_path], [p.y for p in bspline_path], 
             'b-', label='B-spline', linewidth=2)
    ax1.plot([p.x for p in cubic_path], [p.y for p in cubic_path], 
             'g-', label='Cubic spline', linewidth=2)
    ax1.plot(gradient_x, gradient_y, 'r-', label='Gradient-based', linewidth=2)
    
    ax1.set_aspect('equal')
    ax1.grid(True)
    ax1.legend()
    ax1.set_title('Path Comparison')
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    
    # Curvature plot
    ax2.plot([p.s for p in bspline_path], [p.curvature for p in bspline_path], 
             'b-', label='B-spline', linewidth=2)
    ax2.plot([p.s for p in cubic_path], [p.curvature for p in cubic_path], 
             'g-', label='Cubic spline', linewidth=2)
    ax2.plot(np.linspace(0, 1, len(gradient_curv)), gradient_curv, 
             'r-', label='Gradient-based', linewidth=2)
    
    ax2.grid(True)
    ax2.legend()
    ax2.set_title('Curvature Comparison')
    ax2.set_xlabel('Normalized path length')
    ax2.set_ylabel('Curvature [1/m]')
    
    plt.tight_layout()
    
    # Print statistics
    print(f"\n{path_name} Path Statistics:")
    print("Method          | Max Curvature | Mean Curvature | Curvature Std Dev")
    print("------------------------------------------------------------------")
    print(f"B-spline       | {bspline_stats[0]:.4f}       | {bspline_stats[1]:.4f}        | {bspline_stats[2]:.4f}")
    print(f"Cubic spline   | {cubic_stats[0]:.4f}       | {cubic_stats[1]:.4f}        | {cubic_stats[2]:.4f}")
    print(f"Gradient-based | {gradient_stats[0]:.4f}       | {gradient_stats[1]:.4f}        | {gradient_stats[2]:.4f}")
    
    return fig

def main():
    """Main comparison function."""
    test_paths = create_test_paths()
    
    for name, (x, y) in test_paths.items():
        fig = compare_methods(x, y, name)
        plt.show()

if __name__ == "__main__":
    main()
