#!/usr/bin/env python3
"""
Example demonstrating the gradient-based path smoother with interpolation.
This example shows how interpolation before gradient descent improves smoothing results.
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import os

# Add the parent directory to the path so we can import from src
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(parent_dir, 'src'))

from path_smoothing.gradient_based_smoother import GradientPathSmoother

def create_zigzag_path():
    """Create a simple zigzag path for testing."""
    x_points = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    y_points = [0.0, 1.0, 0.0, 1.0, 0.0, 1.0]
    return list(zip(x_points, y_points))

def create_curved_path():
    """Create a curved path with varying point density."""
    # Create a path with non-uniform spacing
    angles = np.array([0, 0.5, 1.2, 2.0, 3.0, 4.5, 6.0])
    x_points = 5 * np.cos(angles)
    y_points = 5 * np.sin(angles)
    return list(zip(x_points, y_points))

def plot_comparison(original_path, interpolated_path, smoothed_path, title):
    """Plot original, interpolated, and smoothed paths for comparison."""
    plt.figure(figsize=(12, 8))
    
    # Convert paths to arrays for plotting
    orig_arr = np.array(original_path)
    interp_arr = np.array(interpolated_path)
    smooth_arr = np.array(smoothed_path)
    
    plt.plot(orig_arr[:, 0], orig_arr[:, 1], 'ro-', label='Original Path', linewidth=2, markersize=8)
    plt.plot(interp_arr[:, 0], interp_arr[:, 1], 'b.-', label='Interpolated Path', linewidth=1, markersize=4, alpha=0.7)
    plt.plot(smooth_arr[:, 0], smooth_arr[:, 1], 'g-', label='Smoothed Path', linewidth=2)
    
    plt.title(title)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')

def main():
    """Demonstrate different interpolation modes."""
    
    # Create test paths
    zigzag_path = create_zigzag_path()
    curved_path = create_curved_path()
    
    print("=== 梯度路径平滑器示例 ===")
    print("演示基于距离的插值功能\n")
    
    # Test 1: Factor-based interpolation (original method)
    print("1. 使用插值因子的方法 (interpolation_factor=3)")
    smoother1 = GradientPathSmoother(
        alpha=0.3,
        beta=0.5,
        interpolation_factor=10,
        distance_based=False,
        max_iterations=500
    )
    
    interpolated1 = smoother1.get_interpolated_path(zigzag_path)
    smoothed1 = smoother1.smooth_path(zigzag_path)
    
    print(f"原始路径点数: {len(zigzag_path)}")
    print(f"插值后点数: {len(interpolated1)}")
    print(f"平滑后点数: {len(smoothed1)}")
    
    # Test 2: Distance-based interpolation with fixed distance
    print("\n2. 使用固定距离间隔的插值 (target_distance=0.3)")
    smoother2 = GradientPathSmoother(
        alpha=1.3,
        beta=0.05,
        distance_based=True,
        target_distance=0.05,
        max_iterations=5000
    )
    
    interpolated2 = smoother2.get_interpolated_path(zigzag_path)
    smoothed2 = smoother2.smooth_path(zigzag_path)
    
    print(f"原始路径点数: {len(zigzag_path)}")
    print(f"插值后点数: {len(interpolated2)}")
    print(f"平滑后点数: {len(smoothed2)}")
    
    # Test 3: Distance-based interpolation with auto distance
    print("\n3. 使用自动计算距离间隔的插值 (distance_based=True, auto)")
    smoother3 = GradientPathSmoother(
        alpha=0.3,
        beta=0.5,
        distance_based=True,
        interpolation_factor=4,  # Used for auto distance calculation
        max_iterations=500
    )
    
    interpolated3 = smoother3.get_interpolated_path(curved_path)
    smoothed3 = smoother3.smooth_path(curved_path)
    
    # Calculate average distance for reference
    curved_arr = np.array(curved_path)
    distances = np.sqrt(np.sum(np.diff(curved_arr, axis=0)**2, axis=1))
    avg_distance = np.mean(distances)
    auto_distance = avg_distance / 4
    
    print(f"原始路径点数: {len(curved_path)}")
    print(f"平均距离: {avg_distance:.3f}")
    print(f"自动计算的目标距离: {auto_distance:.3f}")
    print(f"插值后点数: {len(interpolated3)}")
    print(f"平滑后点数: {len(smoothed3)}")
    
    # Plotting
    plot_comparison(zigzag_path, interpolated1, smoothed1, 
                   "Factor-based Interpolation (factor=3)")
    
    plot_comparison(zigzag_path, interpolated2, smoothed2, 
                   "Distance-based Interpolation (distance=0.3)")
    
    plot_comparison(curved_path, interpolated3, smoothed3, 
                   "Auto Distance-based Interpolation")
    
    plt.show()
    
    # Distance analysis
    print("\n=== 距离分析 ===")
    for i, (name, path) in enumerate([
        ("Factor-based", interpolated1),
        ("Fixed distance", interpolated2), 
        ("Auto distance", interpolated3)
    ]):
        path_arr = np.array(path)
        distances = np.sqrt(np.sum(np.diff(path_arr, axis=0)**2, axis=1))
        print(f"{name}: 平均距离={np.mean(distances):.3f}, "
              f"标准差={np.std(distances):.3f}, "
              f"最小距离={np.min(distances):.3f}, "
              f"最大距离={np.max(distances):.3f}")

if __name__ == "__main__":
    main() 