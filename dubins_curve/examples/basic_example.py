"""
Basic example of using Dubins curves

This example demonstrates how to use the Dubins path implementation
to calculate and visualize paths between poses.
"""

import sys
import os
import math
import matplotlib.pyplot as plt

# Add the parent directory to the path so we can import our package
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from dubins_curve.dubins import DubinsPath
from dubins_curve.types import Pose, DubinsPathResult


def plot_path(start_pose, end_pose, path_result, title="Dubins Path"):
    """Plot a Dubins path with start and end poses"""
    fig, ax = plt.subplots(1, 1, figsize=(10, 8))
    
    # Plot path points
    if path_result.success:
        path_points = DubinsPath(turning_radius=1.0).get_path_points(path_result, num_points=100)
        if path_points:
            x_coords, y_coords = zip(*path_points)
            ax.plot(x_coords, y_coords, 'b-', linewidth=2, label=f'Path ({path_result.path_type})')
    
    # Plot start pose
    ax.arrow(start_pose.x, start_pose.y, 
             0.5 * math.cos(start_pose.theta), 
             0.5 * math.sin(start_pose.theta),
             head_width=0.2, head_length=0.1, fc='green', ec='green')
    ax.plot(start_pose.x, start_pose.y, 'go', markersize=10, label='Start')
    
    # Plot end pose
    ax.arrow(end_pose.x, end_pose.y, 
             0.5 * math.cos(end_pose.theta), 
             0.5 * math.sin(end_pose.theta),
             head_width=0.2, head_length=0.1, fc='red', ec='red')
    ax.plot(end_pose.x, end_pose.y, 'ro', markersize=10, label='End')
    
    # Plot segments
    if path_result.success:
        colors = {'L': 'blue', 'R': 'red', 'S': 'green'}
        for i, segment in enumerate(path_result.segments):
            segment_points = DubinsPath(turning_radius=1.0).get_path_points(
                DubinsPathResult(path_result.path_type, segment.length, [segment], True), 
                num_points=20
            )
            if segment_points:
                x_coords, y_coords = zip(*segment_points)
                ax.plot(x_coords, y_coords, '--', color=colors.get(segment.segment_type, 'black'),
                       alpha=0.7, label=f'{segment.segment_type}-segment {i+1}')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title(title)
    ax.grid(True)
    ax.axis('equal')
    ax.legend()
    
    return fig, ax


def example_basic_path():
    """Example of basic path calculation"""
    print("=== Basic Path Example ===")
    
    # Create start and end poses
    start = Pose(0, 0, 0)  # Start at origin, facing positive X
    end = Pose(4, 0, 0)    # End 4 units away, still facing positive X
    
    # Calculate Dubins path
    dubins = DubinsPath(turning_radius=1.0)
    result = dubins.path(start, end)
    
    print(f"Start pose: ({start.x}, {start.y}, {start.theta:.2f})")
    print(f"End pose: ({end.x}, {end.y}, {end.theta:.2f})")
    print(f"Path type: {result.path_type}")
    print(f"Total length: {result.total_length:.3f}")
    print(f"Number of segments: {len(result.segments)}")
    
    if result.success:
        for i, segment in enumerate(result.segments):
            print(f"  Segment {i+1}: {segment.segment_type}, length: {segment.length:.3f}")
    
    # Plot the path
    fig, ax = plot_path(start, end, result, "Basic Dubins Path")
    plt.show()


def example_different_orientations():
    """Example with different start and end orientations"""
    print("\n=== Different Orientations Example ===")
    
    # Create poses with different orientations
    start = Pose(0, 0, math.pi/4)    # Start at 45 degrees
    end = Pose(4, 3, -math.pi/4)    # End at -45 degrees
    
    # Calculate Dubins path
    dubins = DubinsPath(turning_radius=1.0)
    result = dubins.path(start, end)
    
    print(f"Start pose: ({start.x}, {start.y}, {start.theta:.2f})")
    print(f"End pose: ({end.x}, {end.y}, {end.theta:.2f})")
    print(f"Path type: {result.path_type}")
    print(f"Total length: {result.total_length:.3f}")
    
    # Plot the path
    fig, ax = plot_path(start, end, result, "Dubins Path - Different Orientations")
    plt.show()


def example_multiple_paths():
    """Example showing multiple possible paths"""
    print("\n=== Multiple Paths Example ===")
    
    start = Pose(0, 0, 0)
    end = Pose(6, 0, math.pi)
    
    # Calculate path with different turning radii
    turning_radii = [0.5, 1.0, 2.0]
    
    fig, ax = plt.subplots(1, 1, figsize=(12, 8))
    
    for radius in turning_radii:
        dubins = DubinsPath(turning_radius=radius)
        result = dubins.path(start, end)
        
        if result.success:
            path_points = dubins.get_path_points(result, num_points=100)
            if path_points:
                x_coords, y_coords = zip(*path_points)
                ax.plot(x_coords, y_coords, linewidth=2, 
                       label=f'Radius={radius}, Type={result.path_type}, Length={result.total_length:.2f}')
    
    # Plot start and end poses
    ax.arrow(start.x, start.y, 0.5 * math.cos(start.theta), 0.5 * math.sin(start.theta),
             head_width=0.2, head_length=0.1, fc='green', ec='green')
    ax.plot(start.x, start.y, 'go', markersize=10, label='Start')
    
    ax.arrow(end.x, end.y, 0.5 * math.cos(end.theta), 0.5 * math.sin(end.theta),
             head_width=0.2, head_length=0.1, fc='red', ec='red')
    ax.plot(end.x, end.y, 'ro', markersize=10, label='End')
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title("Dubins Paths with Different Turning Radii")
    ax.grid(True)
    ax.axis('equal')
    ax.legend()
    plt.show()


def example_path_continuity():
    """Example showing path continuity"""
    print("\n=== Path Continuity Example ===")
    
    # Create a sequence of poses
    poses = [
        Pose(0, 0, 0),
        Pose(3, 2, math.pi/4),
        Pose(6, 1, -math.pi/6),
        Pose(8, 4, math.pi/2)
    ]
    
    fig, ax = plt.subplots(1, 1, figsize=(12, 8))
    
    # Plot each segment
    colors = ['blue', 'red', 'green', 'orange']
    for i in range(len(poses) - 1):
        start = poses[i]
        end = poses[i + 1]
        
        dubins = DubinsPath(turning_radius=1.0)
        result = dubins.path(start, end)
        
        if result.success:
            path_points = dubins.get_path_points(result, num_points=50)
            if path_points:
                x_coords, y_coords = zip(*path_points)
                ax.plot(x_coords, y_coords, color=colors[i], linewidth=2, 
                       label=f'Segment {i+1}: {result.path_type}')
    
    # Plot all poses
    for i, pose in enumerate(poses):
        ax.arrow(pose.x, pose.y, 0.3 * math.cos(pose.theta), 0.3 * math.sin(pose.theta),
                 head_width=0.15, head_length=0.08, fc='black', ec='black')
        ax.plot(pose.x, pose.y, 'ko', markersize=8)
        ax.text(pose.x + 0.2, pose.y + 0.2, f'P{i+1}', fontsize=10)
    
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title("Continuous Dubins Path")
    ax.grid(True)
    ax.axis('equal')
    ax.legend()
    plt.show()


def main():
    """Run all examples"""
    print("Dubins Curve Examples")
    print("=" * 50)
    
    try:
        example_basic_path()
        example_different_orientations()
        example_multiple_paths()
        example_path_continuity()
        
    except Exception as e:
        print(f"Error running examples: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()