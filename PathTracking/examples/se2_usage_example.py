#!/usr/bin/env python3
"""
Example demonstrating SE2 coordinate transformations in path tracking applications.

This example shows how to use the SE2 utility class for coordinate transformations
in robotics and path tracking scenarios.
"""

import numpy as np
import matplotlib.pyplot as plt
from PathTracking.utils import SE2, interpolate_se2, create_se2_from_points


def example_robot_pose_transformation():
    """
    Example: Transform robot sensor data from local to global coordinates.
    """
    print("Example: Robot Pose Transformation")
    print("-" * 40)
    
    # Robot pose in global frame
    robot_pose = SE2(x=5.0, y=3.0, theta=np.pi/4)  # 45 degrees
    print(f"Robot pose: {robot_pose}")
    
    # Sensor data in robot's local frame (e.g., LIDAR points)
    local_points = np.array([
        [2.0, 0.0],    # Point directly in front
        [1.0, 1.0],    # Point to front-left
        [1.0, -1.0],   # Point to front-right
        [0.0, 1.5],    # Point to left
        [0.0, -1.5],   # Point to right
    ])
    
    # Transform to global coordinates
    global_points = robot_pose.transform_points(local_points)
    
    print(f"Local sensor points:\n{local_points}")
    print(f"Global sensor points:\n{global_points}")
    
    return robot_pose, local_points, global_points


def example_path_interpolation():
    """
    Example: Generate smooth path between waypoints using SE2 interpolation.
    """
    print("\nExample: Path Interpolation")
    print("-" * 40)
    
    # Define waypoints as SE2 poses
    waypoints = [
        SE2(0.0, 0.0, 0.0),           # Start facing east
        SE2(5.0, 2.0, np.pi/2),       # Turn north
        SE2(7.0, 8.0, np.pi),         # Turn west
        SE2(2.0, 10.0, -np.pi/2),     # Turn south
        SE2(0.0, 12.0, 0.0),          # Turn east again
    ]
    
    print("Waypoints:")
    for i, wp in enumerate(waypoints):
        print(f"  {i}: {wp}")
    
    # Interpolate between consecutive waypoints
    interpolated_path = []
    num_steps = 10
    
    for i in range(len(waypoints) - 1):
        start = waypoints[i]
        end = waypoints[i + 1]
        
        for t in np.linspace(0, 1, num_steps):
            interpolated_pose = interpolate_se2(start, end, t)
            interpolated_path.append(interpolated_pose)
    
    print(f"\nGenerated {len(interpolated_path)} interpolated poses")
    
    return waypoints, interpolated_path


def example_relative_transformations():
    """
    Example: Compute relative transformations between robot poses.
    """
    print("\nExample: Relative Transformations")
    print("-" * 40)
    
    # Two robot poses
    pose_a = SE2(2.0, 1.0, np.pi/6)
    pose_b = SE2(5.0, 4.0, np.pi/3)
    
    print(f"Pose A: {pose_a}")
    print(f"Pose B: {pose_b}")
    
    # Compute relative transformation from A to B
    relative_transform = pose_b.relative_to(pose_a)
    print(f"Relative transform (A -> B): {relative_transform}")
    
    # Verify: A * relative_transform should equal B
    computed_b = pose_a.compose(relative_transform)
    print(f"Verification (A * rel): {computed_b}")
    
    # Compute distance and angle difference
    distance = pose_a.distance_to(pose_b)
    angle_diff = pose_a.angle_difference_to(pose_b)
    print(f"Distance: {distance:.3f} m")
    print(f"Angle difference: {angle_diff:.3f} rad ({np.degrees(angle_diff):.1f}°)")


def example_coordinate_frame_transformation():
    """
    Example: Transform between different coordinate frames.
    """
    print("\nExample: Coordinate Frame Transformation")
    print("-" * 40)
    
    # Define a local coordinate frame relative to global frame
    local_frame = SE2(3.0, 2.0, np.pi/4)  # Local frame origin and orientation
    
    # Point coordinates in local frame
    local_point = np.array([1.0, 1.0])
    
    # Transform to global frame
    global_point = local_frame.transform_point(local_point)
    
    # Transform back to local frame using inverse
    local_frame_inv = local_frame.inverse()
    recovered_local_point = local_frame_inv.transform_point(global_point)
    
    print(f"Local frame: {local_frame}")
    print(f"Point in local frame: {local_point}")
    print(f"Point in global frame: {global_point}")
    print(f"Recovered local point: {recovered_local_point}")
    print(f"Transformation error: {np.linalg.norm(local_point - recovered_local_point):.6f}")


def visualize_se2_examples():
    """
    Visualize the SE2 transformation examples.
    """
    print("\nCreating visualization...")
    
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 12))
    
    # Example 1: Robot pose transformation
    robot_pose, local_points, global_points = example_robot_pose_transformation()
    
    ax1.scatter(local_points[:, 0], local_points[:, 1], c='blue', label='Local points', s=50)
    ax1.scatter(global_points[:, 0], global_points[:, 1], c='red', label='Global points', s=50)
    
    # Draw robot pose
    robot_x, robot_y, robot_theta = robot_pose.x, robot_pose.y, robot_pose.theta
    ax1.arrow(robot_x, robot_y, np.cos(robot_theta), np.sin(robot_theta), 
             head_width=0.2, head_length=0.2, fc='green', ec='green', linewidth=2)
    
    ax1.set_title('Robot Sensor Data Transformation')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.grid(True)
    ax1.legend()
    ax1.axis('equal')
    
    # Example 2: Path interpolation
    waypoints, interpolated_path = example_path_interpolation()
    
    # Plot waypoints
    wp_x = [wp.x for wp in waypoints]
    wp_y = [wp.y for wp in waypoints]
    ax2.plot(wp_x, wp_y, 'ro-', markersize=8, linewidth=2, label='Waypoints')
    
    # Plot interpolated path
    interp_x = [pose.x for pose in interpolated_path]
    interp_y = [pose.y for pose in interpolated_path]
    ax2.plot(interp_x, interp_y, 'b.', markersize=4, label='Interpolated path')
    
    # Draw orientation arrows for waypoints
    for wp in waypoints:
        ax2.arrow(wp.x, wp.y, 0.5*np.cos(wp.theta), 0.5*np.sin(wp.theta),
                 head_width=0.2, head_length=0.2, fc='red', ec='red', alpha=0.7)
    
    ax2.set_title('Path Interpolation with SE2')
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    ax2.grid(True)
    ax2.legend()
    ax2.axis('equal')
    
    # Example 3: Coordinate frame transformation
    local_frame = SE2(3.0, 2.0, np.pi/4)
    
    # Create a shape in local frame
    local_shape = np.array([
        [0, 0], [2, 0], [2, 1], [0, 1], [0, 0]  # Rectangle
    ])
    global_shape = local_frame.transform_points(local_shape)
    
    ax3.plot(local_shape[:, 0], local_shape[:, 1], 'b-', linewidth=2, label='Local frame')
    ax3.plot(global_shape[:, 0], global_shape[:, 1], 'r-', linewidth=2, label='Global frame')
    
    # Draw coordinate frames
    # Local frame at origin
    ax3.arrow(0, 0, 1, 0, head_width=0.1, head_length=0.1, fc='blue', ec='blue')
    ax3.arrow(0, 0, 0, 1, head_width=0.1, head_length=0.1, fc='blue', ec='blue')
    
    # Global frame
    frame_origin = local_frame.translation_vector()
    frame_x = local_frame.transform_point([1, 0]) - frame_origin
    frame_y = local_frame.transform_point([0, 1]) - frame_origin
    
    ax3.arrow(frame_origin[0], frame_origin[1], frame_x[0], frame_x[1],
             head_width=0.1, head_length=0.1, fc='red', ec='red')
    ax3.arrow(frame_origin[0], frame_origin[1], frame_y[0], frame_y[1],
             head_width=0.1, head_length=0.1, fc='red', ec='red')
    
    ax3.set_title('Coordinate Frame Transformation')
    ax3.set_xlabel('X (m)')
    ax3.set_ylabel('Y (m)')
    ax3.grid(True)
    ax3.legend()
    ax3.axis('equal')
    
    # Example 4: SE2 composition
    se2_1 = SE2(1, 1, np.pi/6)
    se2_2 = SE2(2, 1, np.pi/4)
    se2_composed = se2_1.compose(se2_2)
    
    # Show how transformations compose
    test_point = np.array([1, 0])
    point_1 = se2_1.transform_point(test_point)
    point_2 = se2_2.transform_point(test_point)
    point_composed = se2_composed.transform_point(test_point)
    
    ax4.scatter(*test_point, c='black', s=100, label='Original point')
    ax4.scatter(*point_1, c='blue', s=100, label='After SE2_1')
    ax4.scatter(*point_2, c='green', s=100, label='After SE2_2')
    ax4.scatter(*point_composed, c='red', s=100, label='After composition')
    
    # Draw SE2 frames
    for se2, color, label in [(se2_1, 'blue', 'SE2_1'), 
                              (se2_2, 'green', 'SE2_2'),
                              (se2_composed, 'red', 'Composed')]:
        x, y, theta = se2.x, se2.y, se2.theta
        ax4.arrow(x, y, 0.5*np.cos(theta), 0.5*np.sin(theta),
                 head_width=0.1, head_length=0.1, fc=color, ec=color, alpha=0.7)
        ax4.text(x+0.2, y+0.2, label, color=color, fontsize=10)
    
    ax4.set_title('SE2 Composition')
    ax4.set_xlabel('X (m)')
    ax4.set_ylabel('Y (m)')
    ax4.grid(True)
    ax4.legend()
    ax4.axis('equal')
    
    plt.tight_layout()
    plt.show()


def main():
    """
    Run all SE2 examples.
    """
    print("SE2 Coordinate Transformation Examples")
    print("=" * 50)
    
    example_robot_pose_transformation()
    example_path_interpolation()
    example_relative_transformations()
    example_coordinate_frame_transformation()
    
    # Uncomment to show visualization
    # visualize_se2_examples()


if __name__ == "__main__":
    main() 