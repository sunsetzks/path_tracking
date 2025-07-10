#!/usr/bin/env python3
"""
Comparison of different path smoothing methods including the improved gradient-based smoother.
This example demonstrates the effectiveness of interpolation before gradient descent.
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import os

# Add the src directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from path_smoothing.cubic_spline_smoother import CubicSplineSmoother
from path_smoothing.gradient_based_smoother import GradientPathSmoother
from path_smoothing.bspline_smoother import BSplineSmoother

def create_test_paths():
    """Create different test paths for comparison."""
    
    # Sharp turn path
    sharp_turn_path = [
        (0.0, 0.0),
        (2.0, 0.0),
        (4.0, 0.0),
        (4.1, 2.0),  # Very sharp turn
        (4.2, 4.0),
        (6.0, 4.0),
        (8.0, 4.0)
    ]
    
    # Zigzag path
    zigzag_path = [
        (0.0, 0.0),
        (1.0, 2.0),
        (2.0, -1.0),
        (3.0, 3.0),
        (4.0, -2.0),
        (5.0, 4.0),
        (6.0, -1.0),
        (7.0, 3.0),
        (8.0, 0.0)
    ]
    
    # Noisy path
    np.random.seed(42)  # For reproducible results
    smooth_x = np.linspace(0, 8, 10)
    smooth_y = 2 * np.sin(smooth_x / 2)
    noise = np.random.normal(0, 0.3, len(smooth_y))
    noisy_path = list(zip(smooth_x, smooth_y + noise))
    
    return {
        'Sharp Turn': sharp_turn_path,
        'Zigzag': zigzag_path,
        'Noisy Sine': noisy_path
    }

def compare_smoothing_methods():
    """Compare all smoothing methods on different test paths."""
    
    test_paths = create_test_paths()
    
    # Initialize smoothers
    cubic_smoother = CubicSplineSmoother(path_resolution=0.1)
    
    gradient_smoother_basic = GradientPathSmoother(
        alpha=0.5,
        beta=0.3,
        target_distance=0.2,  # Fixed distance interval for interpolation (was learning_rate)
        max_iterations=500
        # Removed interpolation_factor and interpolation_method as they don't exist
    )
    
    gradient_smoother_interp = GradientPathSmoother(
        alpha=0.5,
        beta=0.3,
        target_distance=0.1,  # Smaller distance for more points (was learning_rate)
        max_iterations=500
        # Removed interpolation_factor and interpolation_method as they don't exist
    )
    
    bspline_smoother = BSplineSmoother(degree=3, smoothing_factor=0.1)
    
    # Create figure for comparison
    fig, axes = plt.subplots(2, 3, figsize=(18, 12))
    
    for col, (path_name, original_path) in enumerate(test_paths.items()):
        # Apply all smoothing methods
        # Convert path to x, y lists for cubic and bspline smoothers
        original_x, original_y = zip(*original_path)
        
        try:
            cubic_smoothed_points = cubic_smoother.smooth_path(list(original_x), list(original_y))
            cubic_smoothed = [(p.x, p.y) for p in cubic_smoothed_points]
        except Exception as e:
            print(f"Cubic spline failed for {path_name}: {e}")
            cubic_smoothed = original_path
            
        gradient_basic_smoothed = gradient_smoother_basic.smooth_path(original_path)
        gradient_interp_smoothed = gradient_smoother_interp.smooth_path(original_path)
        
        try:
            bspline_smoothed_points = bspline_smoother.smooth_path(list(original_x), list(original_y))
            bspline_smoothed = [(p.x, p.y) for p in bspline_smoothed_points]
        except Exception as e:
            print(f"B-spline failed for {path_name}: {e}")
            bspline_smoothed = original_path
        
        # Convert to numpy arrays for plotting
        original_array = np.array(original_path)
        cubic_array = np.array(cubic_smoothed)
        gradient_basic_array = np.array(gradient_basic_smoothed)
        gradient_interp_array = np.array(gradient_interp_smoothed)
        bspline_array = np.array(bspline_smoothed)
        
        # Plot paths
        ax = axes[0, col]
        ax.plot(original_array[:, 0], original_array[:, 1], 'ro-', 
                label='Original', markersize=6, linewidth=2, alpha=0.8)
        ax.plot(cubic_array[:, 0], cubic_array[:, 1], 'g-', 
                label='Cubic Spline', linewidth=2)
        ax.plot(gradient_basic_array[:, 0], gradient_basic_array[:, 1], 'b--', 
                label='Gradient (Basic)', linewidth=2)
        ax.plot(gradient_interp_array[:, 0], gradient_interp_array[:, 1], 'm-', 
                label='Gradient (Interpolated)', linewidth=2)
        ax.plot(bspline_array[:, 0], bspline_array[:, 1], 'c-.', 
                label='B-Spline', linewidth=2)
        
        ax.set_title(f'{path_name} Path')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        
        # Calculate and plot curvatures
        ax_curv = axes[1, col]
        
        try:
            original_curvature = gradient_smoother_basic.get_path_curvature(original_path)
            cubic_curvature = gradient_smoother_basic.get_path_curvature(cubic_smoothed)
            gradient_basic_curvature = gradient_smoother_basic.get_path_curvature(gradient_basic_smoothed)
            gradient_interp_curvature = gradient_smoother_interp.get_path_curvature(gradient_interp_smoothed)
            bspline_curvature = gradient_smoother_basic.get_path_curvature(bspline_smoothed)
            
            x_orig = range(len(original_curvature))
            x_cubic = range(len(cubic_curvature))
            x_grad_basic = range(len(gradient_basic_curvature))
            x_grad_interp = range(len(gradient_interp_curvature))
            x_bspline = range(len(bspline_curvature))
            
            ax_curv.plot(x_orig, original_curvature, 'ro-', label='Original', markersize=4)
            ax_curv.plot(x_cubic, cubic_curvature, 'g-', label='Cubic Spline')
            ax_curv.plot(x_grad_basic, gradient_basic_curvature, 'b--', label='Gradient (Basic)')
            ax_curv.plot(x_grad_interp, gradient_interp_curvature, 'm-', label='Gradient (Interpolated)')
            ax_curv.plot(x_bspline, bspline_curvature, 'c-.', label='B-Spline')
            
        except Exception as e:
            print(f"Curvature calculation failed for {path_name}: {e}")
            ax_curv.text(0.5, 0.5, 'Curvature calculation failed', 
                        transform=ax_curv.transAxes, ha='center', va='center')
        
        ax_curv.set_title(f'{path_name} Curvature')
        ax_curv.set_xlabel('Point Index')
        ax_curv.set_ylabel('Curvature')
        ax_curv.legend()
        ax_curv.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()

def analyze_interpolation_benefits():
    """Analyze the specific benefits of different target distances in gradient-based smoothing."""
    
    # Create a path with sparse points but smooth underlying curve
    t = np.linspace(0, 4*np.pi, 8)  # Only 8 points for a full sine curve
    sparse_path = list(zip(t, 2 * np.sin(t)))
    
    # Create smoothers with different target distances (effectively different interpolation densities)
    smoothers = {
        'Sparse (0.5)': GradientPathSmoother(
            alpha=0.6, beta=0.4, target_distance=0.5
        ),
        'Medium (0.2)': GradientPathSmoother(
            alpha=0.6, beta=0.4, target_distance=0.2
        ),
        'Dense (0.1)': GradientPathSmoother(
            alpha=0.6, beta=0.4, target_distance=0.1
        ),
        'Very Dense (0.05)': GradientPathSmoother(
            alpha=0.6, beta=0.4, target_distance=0.05
        )
    }
    
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 12))
    
    original_array = np.array(sparse_path)
    
    # Generate true smooth curve for comparison
    t_fine = np.linspace(0, 4*np.pi, 100)
    true_curve = np.column_stack([t_fine, 2 * np.sin(t_fine)])
    
    ax1.plot(true_curve[:, 0], true_curve[:, 1], 'k-', 
             label='True Smooth Curve', linewidth=3, alpha=0.7)
    ax1.plot(original_array[:, 0], original_array[:, 1], 'ro-', 
             label='Sparse Original', markersize=8, linewidth=2)
    
    colors = ['blue', 'green', 'purple', 'orange']
    styles = ['-', '--', '-.', ':']
    
    results = {}
    
    for i, (method_name, smoother) in enumerate(smoothers.items()):
        # Get smoothed path
        smoothed = smoother.smooth_path(sparse_path)
        smoothed_array = np.array(smoothed)
        
        results[method_name] = {
            'smoothed': smoothed_array,
            'num_points': len(smoothed_array)
        }
        
        # Plot on main comparison
        ax1.plot(smoothed_array[:, 0], smoothed_array[:, 1], 
                color=colors[i], linestyle=styles[i], 
                label=f'{method_name} ({len(smoothed_array)} pts)', linewidth=2)
    
    ax1.set_title('Target Distance Effect on Sparse Path Smoothing')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    
    # Plot point density comparison
    methods = list(smoothers.keys())
    point_counts = [results[method]['num_points'] for method in methods]
    
    ax2.bar(methods, point_counts, color=colors[:len(methods)], alpha=0.7)
    ax2.set_title('Number of Points After Processing')
    ax2.set_ylabel('Number of Points')
    plt.setp(ax2.get_xticklabels(), rotation=45, ha='right')
    ax2.grid(True, alpha=0.3)
    
    # Analyze smoothness (curvature variance)
    smoothness_scores = []
    for method in methods:
        curvature = smoothers[method].get_path_curvature(
            [(float(p[0]), float(p[1])) for p in results[method]['smoothed']]
        )
        smoothness_score = np.var(curvature)  # Lower variance = smoother
        smoothness_scores.append(smoothness_score)
    
    ax3.bar(methods, smoothness_scores, color=colors[:len(methods)], alpha=0.7)
    ax3.set_title('Smoothness Quality (Lower = Better)')
    ax3.set_ylabel('Curvature Variance')
    plt.setp(ax3.get_xticklabels(), rotation=45, ha='right')
    ax3.grid(True, alpha=0.3)
    
    # Calculate deviation from true curve (approximation)
    deviations = []
    for method in methods:
        smoothed_array = results[method]['smoothed']
        # Sample true curve at similar points
        t_sample = np.linspace(0, 4*np.pi, len(smoothed_array))
        true_sample = np.column_stack([t_sample, 2 * np.sin(t_sample)])
        
        # Calculate average distance
        distances = np.sqrt(np.sum((smoothed_array - true_sample)**2, axis=1))
        avg_deviation = np.mean(distances)
        deviations.append(avg_deviation)
    
    ax4.bar(methods, deviations, color=colors[:len(methods)], alpha=0.7)
    ax4.set_title('Deviation from True Curve (Lower = Better)')
    ax4.set_ylabel('Average Distance')
    plt.setp(ax4.get_xticklabels(), rotation=45, ha='right')
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    # Print numerical results
    print("\n=== Target Distance Analysis Results ===")
    for method in methods:
        print(f"\n{method}:")
        print(f"  Points: {results[method]['num_points']}")
        print(f"  Smoothness (curvature variance): {smoothness_scores[methods.index(method)]:.4f}")
        print(f"  Deviation from true curve: {deviations[methods.index(method)]:.4f}")

if __name__ == "__main__":
    print("Path Smoothing Methods Comparison with Interpolation")
    print("=" * 60)
    
    print("\n1. Comparing all smoothing methods...")
    compare_smoothing_methods()
    
    print("\n2. Analyzing interpolation benefits...")
    analyze_interpolation_benefits()
    
    print("\nComparison completed!")
