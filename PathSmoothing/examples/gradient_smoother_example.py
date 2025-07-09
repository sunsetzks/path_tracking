"""
Example demonstrating gradient-based path smoothing with fixed endpoints.

This example shows how the gradient-based smoother preserves start and end points
while smoothing the intermediate path points.
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
from pathlib import Path

# Add the source directory to path
sys.path.append(str(Path(__file__).parent.parent / "src"))

from path_smoothing.gradient_based_smoother import GradientPathSmoother


def create_zigzag_path():
    """Create a zigzag path for testing."""
    x = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]
    y = [0.0, 2.0, -1.0, 3.0, -2.0, 4.0, -1.0, 3.0, -2.0, 2.0, 0.0]
    return list(zip(x, y))


def main():
    """Demonstrate gradient-based path smoothing with endpoint preservation."""
    
    # Create test path
    original_path = create_zigzag_path()
    print(f"Original path start: {original_path[0]}")
    print(f"Original path end: {original_path[-1]}")
    
    # Initialize smoother with endpoint fixing enabled
    smoother_fixed = GradientPathSmoother(
        alpha=0.5,          # Higher weight for smoothness
        beta=0.3,           # Lower weight for similarity to allow more smoothing
        learning_rate=0.01,
        max_iterations=2000,
        fix_endpoints=True  # Keep start and end points fixed
    )
    
    # Initialize smoother without endpoint fixing for comparison
    smoother_free = GradientPathSmoother(
        alpha=0.5,
        beta=0.3,
        learning_rate=0.01,
        max_iterations=2000,
        fix_endpoints=False  # Allow start and end points to move
    )
    
    # Smooth the path with both methods
    smoothed_path_fixed = smoother_fixed.smooth_path(original_path)
    smoothed_path_free = smoother_free.smooth_path(original_path)
    
    print(f"Smoothed path (fixed) start: {smoothed_path_fixed[0]}")
    print(f"Smoothed path (fixed) end: {smoothed_path_fixed[-1]}")
    print(f"Smoothed path (free) start: {smoothed_path_free[0]}")
    print(f"Smoothed path (free) end: {smoothed_path_free[-1]}")
    
    # Verify endpoints are preserved when fix_endpoints=True
    start_preserved = np.allclose(original_path[0], smoothed_path_fixed[0], atol=1e-10)
    end_preserved = np.allclose(original_path[-1], smoothed_path_fixed[-1], atol=1e-10)
    
    print(f"\nEndpoint preservation check:")
    print(f"Start point preserved: {start_preserved}")
    print(f"End point preserved: {end_preserved}")
    
    # Calculate curvature for both paths
    curvature_fixed = smoother_fixed.get_path_curvature(smoothed_path_fixed)
    curvature_free = smoother_free.get_path_curvature(smoothed_path_free)
    
    # Plot results
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    
    # Plot paths
    original_x, original_y = zip(*original_path)
    fixed_x, fixed_y = zip(*smoothed_path_fixed)
    free_x, free_y = zip(*smoothed_path_free)
    
    ax1.plot(original_x, original_y, 'ro-', label='Original path', linewidth=2, markersize=8)
    ax1.plot(fixed_x, fixed_y, 'b-', label='Smoothed (fixed endpoints)', linewidth=2)
    ax1.plot(free_x, free_y, 'g--', label='Smoothed (free endpoints)', linewidth=2)
    
    # Highlight start and end points
    ax1.plot(original_x[0], original_y[0], 'ks', markersize=12, label='Start point')
    ax1.plot(original_x[-1], original_y[-1], 'ks', markersize=12, label='End point')
    
    ax1.set_xlabel('X [m]')
    ax1.set_ylabel('Y [m]')
    ax1.set_title('Path Smoothing Comparison')
    ax1.legend()
    ax1.grid(True)
    ax1.set_aspect('equal')
    
    # Plot curvature comparison
    path_length_fixed = np.arange(len(curvature_fixed))
    path_length_free = np.arange(len(curvature_free))
    
    ax2.plot(path_length_fixed, curvature_fixed, 'b-', label='Fixed endpoints', linewidth=2)
    ax2.plot(path_length_free, curvature_free, 'g--', label='Free endpoints', linewidth=2)
    ax2.set_xlabel('Path point index')
    ax2.set_ylabel('Curvature [1/m]')
    ax2.set_title('Path Curvature Comparison')
    ax2.legend()
    ax2.grid(True)
    
    plt.tight_layout()
    plt.show()
    
    # Print statistics
    print(f"\nCurvature statistics:")
    print(f"Fixed endpoints - Max: {max(curvature_fixed):.4f}, Mean: {np.mean(curvature_fixed):.4f}")
    print(f"Free endpoints  - Max: {max(curvature_free):.4f}, Mean: {np.mean(curvature_free):.4f}")


if __name__ == "__main__":
    main() 