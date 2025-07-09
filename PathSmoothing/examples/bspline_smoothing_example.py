"""
Example script demonstrating the usage of BSplineSmoother for path smoothing.

This script shows different examples of B-spline path smoothing:
1. Different B-spline degrees (linear, quadratic, cubic, quartic)
2. Comparison between open and closed (periodic) paths
3. Effect of smoothing factor on approximation vs interpolation
4. Complex path scenarios (figure-8, spiral, zigzag)

B-splines offer several advantages:
- Local control: modifying one control point affects only nearby curve segments
- High smoothness: continuous derivatives up to degree-1
- Flexibility: adjustable degree and smoothing parameters
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

from path_smoothing import BSplineSmoother


def example_different_degrees():
    """Example comparing different B-spline degrees."""
    print("\n" + "="*50)
    print("Example 1: Comparing Different B-spline Degrees")
    print("="*50)
    
    # Create a path with sharp corners
    x = [0.0, 4.0, 8.0, 8.0, 4.0, 0.0]
    y = [0.0, 0.0, 2.0, 6.0, 6.0, 2.0]

    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    fig.suptitle('B-spline Smoothing with Different Degrees', fontsize=16)
    
    degrees = [1, 2, 3, 4]
    
    for i, degree in enumerate(degrees):
        ax = axes[i//2, i%2]
        
        try:
            smoother = BSplineSmoother(degree=degree, path_resolution=0.05)
            smoothed_path = smoother.smooth_path(x, y)

            # Plot original points
            ax.plot(x, y, 'o-', label='Original path', color='red', markersize=8, alpha=0.7)
            
            # Plot smoothed path
            smooth_x = [p.x for p in smoothed_path]
            smooth_y = [p.y for p in smoothed_path]
            ax.plot(smooth_x, smooth_y, '-', label=f'B-spline (degree {degree})', 
                   color='blue', linewidth=2)
            
            # Plot control points
            ctrl_points = smoother.get_control_points()
            if ctrl_points:
                ctrl_x, ctrl_y = zip(*ctrl_points)
                ax.plot(ctrl_x, ctrl_y, 's', label='Control points', 
                       color='green', markersize=5, alpha=0.7)
                ax.plot(ctrl_x, ctrl_y, '--', color='green', alpha=0.4)

            max_curv, mean_curv, std_curv = smoother.get_curvature_stats()
            curv_var, max_curv_change = smoother.get_smoothness_metrics()
            
            ax.set_title(f'Degree {degree} B-spline\nMax κ: {max_curv:.3f}, Mean κ: {mean_curv:.3f}')
            ax.set_aspect('equal')
            ax.grid(True, alpha=0.3)
            ax.legend()
            
            print(f"Degree {degree} B-spline:")
            print(f"  Total Length: {smoother.total_length:.2f} m")
            print(f"  Maximum Curvature: {max_curv:.3f} 1/m")
            print(f"  Mean Curvature: {mean_curv:.3f} 1/m")
            print(f"  Curvature Variation: {curv_var:.3f}")
            
        except Exception as e:
            ax.text(0.5, 0.5, f'Error with degree {degree}:\n{str(e)}', 
                   transform=ax.transAxes, ha='center', va='center')
            ax.set_title(f'Degree {degree} - Error')
    
    plt.tight_layout()
    plt.show()


def example_periodic_path():
    """Example demonstrating periodic (closed) B-spline paths."""
    print("\n" + "="*50)
    print("Example 2: Periodic vs Open B-spline Paths")
    print("="*50)
    
    # Create a closed path (rectangle with one corner cut)
    x = [0.0, 5.0, 10.0, 10.0, 5.0, 0.0, 0.0]
    y = [0.0, 0.0, 2.0, 8.0, 8.0, 4.0, 0.0]
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    fig.suptitle('Open vs Periodic B-spline Paths', fontsize=16)
    
    # Open path
    smoother_open = BSplineSmoother(degree=3, path_resolution=0.1, periodic=False)
    smoothed_open = smoother_open.smooth_path(x, y)
    
    ax1.plot(x, y, 'o-', label='Original path', color='red', markersize=8, alpha=0.7)
    smooth_x = [p.x for p in smoothed_open]
    smooth_y = [p.y for p in smoothed_open]
    ax1.plot(smooth_x, smooth_y, '-', label='Open B-spline', color='blue', linewidth=2)
    
    ctrl_points = smoother_open.get_control_points()
    if ctrl_points:
        ctrl_x, ctrl_y = zip(*ctrl_points)
        ax1.plot(ctrl_x, ctrl_y, 's', color='green', markersize=5, alpha=0.7)
        ax1.plot(ctrl_x, ctrl_y, '--', color='green', alpha=0.4)
    
    ax1.set_title('Open B-spline')
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    
    # Periodic path
    smoother_periodic = BSplineSmoother(degree=3, path_resolution=0.1, periodic=True)
    smoothed_periodic = smoother_periodic.smooth_path(x, y)
    
    ax2.plot(x, y, 'o-', label='Original path', color='red', markersize=8, alpha=0.7)
    smooth_x = [p.x for p in smoothed_periodic]
    smooth_y = [p.y for p in smoothed_periodic]
    ax2.plot(smooth_x, smooth_y, '-', label='Periodic B-spline', color='blue', linewidth=2)
    
    ctrl_points = smoother_periodic.get_control_points()
    if ctrl_points:
        ctrl_x, ctrl_y = zip(*ctrl_points)
        ax2.plot(ctrl_x, ctrl_y, 's', color='green', markersize=5, alpha=0.7)
        ax2.plot(ctrl_x, ctrl_y, '--', color='green', alpha=0.4)
    
    ax2.set_title('Periodic B-spline')
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    
    plt.tight_layout()
    plt.show()
    
    # Print statistics
    max_curv_open, mean_curv_open, _ = smoother_open.get_curvature_stats()
    max_curv_periodic, mean_curv_periodic, _ = smoother_periodic.get_curvature_stats()
    
    print(f"Open B-spline - Max curvature: {max_curv_open:.3f}, Mean: {mean_curv_open:.3f}")
    print(f"Periodic B-spline - Max curvature: {max_curv_periodic:.3f}, Mean: {mean_curv_periodic:.3f}")


def example_smoothing_factor():
    """Example showing the effect of smoothing factor."""
    print("\n" + "="*50)
    print("Example 3: Effect of Smoothing Factor")
    print("="*50)
    
    # Create a noisy path
    t = np.linspace(0, 4*np.pi, 20)
    x = t * np.cos(t) + 0.5 * np.random.randn(20)
    y = t * np.sin(t) + 0.5 * np.random.randn(20)
    
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    fig.suptitle('Effect of Smoothing Factor on B-spline Fitting', fontsize=16)
    
    smoothing_factors = [0.0, 0.1, 1.0, 5.0]
    
    for i, s_factor in enumerate(smoothing_factors):
        ax = axes[i//2, i%2]
        
        smoother = BSplineSmoother(degree=3, path_resolution=0.1, smoothing_factor=s_factor)
        smoothed_path = smoother.smooth_path(x.tolist(), y.tolist())
        
        # Plot original points
        ax.plot(x, y, 'o', label='Original noisy path', color='red', markersize=6, alpha=0.7)
        
        # Plot smoothed path
        smooth_x = [p.x for p in smoothed_path]
        smooth_y = [p.y for p in smoothed_path]
        ax.plot(smooth_x, smooth_y, '-', label=f'Smoothing factor: {s_factor}', 
               color='blue', linewidth=2)
        
        max_curv, mean_curv, std_curv = smoother.get_curvature_stats()
        
        ax.set_title(f'Smoothing Factor: {s_factor}\nMax κ: {max_curv:.3f}')
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.legend()
        
        print(f"Smoothing factor {s_factor}:")
        print(f"  Maximum Curvature: {max_curv:.3f} 1/m")
        print(f"  Mean Curvature: {mean_curv:.3f} 1/m")
        print(f"  Curvature Std Dev: {std_curv:.3f} 1/m")
    
    plt.tight_layout()
    plt.show()


def example_complex_paths():
    """Example with complex path scenarios."""
    print("\n" + "="*50)
    print("Example 4: Complex Path Scenarios")
    print("="*50)
    
    fig, axes = plt.subplots(2, 2, figsize=(15, 12))
    fig.suptitle('B-spline Smoothing for Complex Paths', fontsize=16)
    
    # Figure-8 path
    t = np.linspace(0, 2*np.pi, 16)
    x_fig8 = 5 * np.sin(t)
    y_fig8 = 3 * np.sin(2*t)
    
    smoother_fig8 = BSplineSmoother(degree=3, path_resolution=0.1)
    smoothed_fig8 = smoother_fig8.smooth_path(x_fig8.tolist(), y_fig8.tolist())
    
    axes[0, 0].plot(x_fig8, y_fig8, 'o', color='red', markersize=6, alpha=0.7)
    smooth_x = [p.x for p in smoothed_fig8]
    smooth_y = [p.y for p in smoothed_fig8]
    axes[0, 0].plot(smooth_x, smooth_y, '-', color='blue', linewidth=2)
    axes[0, 0].set_title('Figure-8 Path')
    axes[0, 0].set_aspect('equal')
    axes[0, 0].grid(True, alpha=0.3)
    
    # Spiral path
    t = np.linspace(0, 4*np.pi, 25)
    r = t * 0.5
    x_spiral = r * np.cos(t)
    y_spiral = r * np.sin(t)
    
    smoother_spiral = BSplineSmoother(degree=3, path_resolution=0.1)
    smoothed_spiral = smoother_spiral.smooth_path(x_spiral.tolist(), y_spiral.tolist())
    
    axes[0, 1].plot(x_spiral, y_spiral, 'o', color='red', markersize=5, alpha=0.7)
    smooth_x = [p.x for p in smoothed_spiral]
    smooth_y = [p.y for p in smoothed_spiral]
    axes[0, 1].plot(smooth_x, smooth_y, '-', color='blue', linewidth=2)
    axes[0, 1].set_title('Spiral Path')
    axes[0, 1].set_aspect('equal')
    axes[0, 1].grid(True, alpha=0.3)
    
    # Zigzag path
    x_zigzag = [0.0, 1.0, 0.0, 2.0, 0.0, 3.0, 0.0, 4.0, 0.0, 5.0]
    y_zigzag = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]
    
    smoother_zigzag = BSplineSmoother(degree=3, path_resolution=0.1)
    smoothed_zigzag = smoother_zigzag.smooth_path(x_zigzag, y_zigzag)
    
    axes[1, 0].plot(x_zigzag, y_zigzag, 'o-', color='red', markersize=6, alpha=0.7)
    smooth_x = [p.x for p in smoothed_zigzag]
    smooth_y = [p.y for p in smoothed_zigzag]
    axes[1, 0].plot(smooth_x, smooth_y, '-', color='blue', linewidth=2)
    axes[1, 0].set_title('Zigzag Path')
    axes[1, 0].set_aspect('equal')
    axes[1, 0].grid(True, alpha=0.3)
    
    # Racing track-like path
    theta = np.linspace(0, 2*np.pi, 12)
    r = 5 + 2 * np.cos(3*theta)
    x_track = r * np.cos(theta)
    y_track = r * np.sin(theta)
    
    smoother_track = BSplineSmoother(degree=3, path_resolution=0.1, periodic=True)
    smoothed_track = smoother_track.smooth_path(x_track.tolist(), y_track.tolist())
    
    axes[1, 1].plot(x_track, y_track, 'o', color='red', markersize=6, alpha=0.7)
    smooth_x = [p.x for p in smoothed_track]
    smooth_y = [p.y for p in smoothed_track]
    axes[1, 1].plot(smooth_x, smooth_y, '-', color='blue', linewidth=2)
    axes[1, 1].set_title('Racing Track Path (Periodic)')
    axes[1, 1].set_aspect('equal')
    axes[1, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    # Print statistics for all paths
    paths_data = [
        ("Figure-8", smoother_fig8),
        ("Spiral", smoother_spiral),
        ("Zigzag", smoother_zigzag),
        ("Racing Track", smoother_track)
    ]
    
    for name, smoother in paths_data:
        max_curv, mean_curv, std_curv = smoother.get_curvature_stats()
        curv_var, max_curv_change = smoother.get_smoothness_metrics()
        print(f"{name} path:")
        print(f"  Length: {smoother.total_length:.2f} m")
        print(f"  Max curvature: {max_curv:.3f} 1/m")
        print(f"  Mean curvature: {mean_curv:.3f} 1/m")
        print(f"  Curvature variation: {curv_var:.3f}")


def example_parameter_evaluation():
    """Example showing parameter-based path evaluation."""
    print("\n" + "="*50)
    print("Example 5: Parameter-based Path Evaluation")
    print("="*50)
    
    # Create a simple curved path
    x = [0.0, 2.0, 4.0, 6.0, 8.0, 10.0]
    y = [0.0, 3.0, 2.0, 4.0, 1.0, 3.0]
    
    smoother = BSplineSmoother(degree=3, path_resolution=0.1)
    smoothed_path = smoother.smooth_path(x, y)
    
    # Evaluate at specific parameter values
    param_values = [0.0, 0.25, 0.5, 0.75, 1.0]
    eval_points = []
    
    for u in param_values:
        point = smoother.evaluate_at_parameter(u)
        if point:
            eval_points.append(point)
    
    # Plot the results
    plt.figure(figsize=(12, 8))
    
    # Plot original and smoothed path
    plt.plot(x, y, 'o-', label='Original path', color='red', markersize=8, alpha=0.7)
    smooth_x = [p.x for p in smoothed_path]
    smooth_y = [p.y for p in smoothed_path]
    plt.plot(smooth_x, smooth_y, '-', label='B-spline path', color='blue', linewidth=2)
    
    # Plot evaluation points
    eval_x = [p.x for p in eval_points]
    eval_y = [p.y for p in eval_points]
    plt.plot(eval_x, eval_y, 's', label='Parameter evaluation points', 
             color='purple', markersize=10, alpha=0.8)
    
    # Add arrows showing heading at evaluation points
    for i, (point, u) in enumerate(zip(eval_points, param_values)):
        if point.yaw is not None:
            dx = 0.5 * np.cos(point.yaw)
            dy = 0.5 * np.sin(point.yaw)
            plt.arrow(point.x, point.y, dx, dy, head_width=0.15, head_length=0.2, 
                     fc='purple', ec='purple', alpha=0.7)
            plt.text(point.x + 0.3, point.y + 0.3, f'u={u}', fontsize=10)
    
    plt.title('B-spline Parameter-based Evaluation')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.tight_layout()
    plt.show()
    
    # Print evaluation results
    print("Parameter-based evaluation results:")
    for u, point in zip(param_values, eval_points):
        print(f"  u={u:.2f}: pos=({point.x:.2f}, {point.y:.2f}), "
              f"yaw={point.yaw:.3f} rad, κ={point.curvature:.3f} 1/m")


def main():
    """Run all B-spline smoothing examples."""
    print("B-spline Path Smoothing Examples")
    print("================================")
    print("This script demonstrates various features of B-spline path smoothing:")
    print("- Different polynomial degrees")
    print("- Open vs periodic (closed) paths")
    print("- Smoothing factor effects")
    print("- Complex path scenarios")
    print("- Parameter-based evaluation")
    
    example_different_degrees()
    example_periodic_path()
    example_smoothing_factor()
    example_complex_paths()
    example_parameter_evaluation()
    
    print("\n" + "="*50)
    print("All examples completed!")
    print("B-splines provide excellent local control and smoothness")
    print("for robotic path planning and trajectory generation.")


if __name__ == "__main__":
    main() 