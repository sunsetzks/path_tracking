"""
Path Data Visualization Demo
Demonstration of the new path data visualization window showing x, y, yaw, steer data.
"""

from typing import List, Optional, Union, Sequence
import numpy as np
import matplotlib.pyplot as plt
from astar_project.hybrid_astar import HybridAStar, VehicleModel, State, DirectionMode, Node
from astar_project.visualizer import HybridAStarVisualizer


def visualize_path_data(path: Optional[List[Node]]) -> None:
    """Display path data (x, y, yaw, steer) in a separate window"""
    if not path:
        print("No path data to display")
        return
    
    # Extract path data
    x_coords = [node.state.x for node in path]
    y_coords = [node.state.y for node in path]
    yaw_angles = [node.state.yaw for node in path]
    steer_angles = [node.state.steer for node in path]
    directions = [node.state.direction.name for node in path]
    
    # Create path data window with additional subplot for direction
    fig, axes = plt.subplots(2, 3, figsize=(18, 10))
    fig.suptitle('Path Data Analysis - Vehicle Trajectory Details', fontsize=16, fontweight='bold')
    
    # Plot X coordinates
    axes[0, 0].plot(range(len(x_coords)), x_coords, 'b-', linewidth=2, marker='o', markersize=3)
    axes[0, 0].set_title('X Coordinates vs Waypoint Index', fontsize=12)
    axes[0, 0].set_xlabel('Waypoint Index')
    axes[0, 0].set_ylabel('X (m)')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].text(0.02, 0.98, f'Range: {min(x_coords):.2f} to {max(x_coords):.2f} m', 
                    transform=axes[0, 0].transAxes, verticalalignment='top',
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='lightblue', alpha=0.8))
    
    # Plot Y coordinates
    axes[0, 1].plot(range(len(y_coords)), y_coords, 'g-', linewidth=2, marker='o', markersize=3)
    axes[0, 1].set_title('Y Coordinates vs Waypoint Index', fontsize=12)
    axes[0, 1].set_xlabel('Waypoint Index')
    axes[0, 1].set_ylabel('Y (m)')
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].text(0.02, 0.98, f'Range: {min(y_coords):.2f} to {max(y_coords):.2f} m', 
                    transform=axes[0, 1].transAxes, verticalalignment='top',
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='lightgreen', alpha=0.8))
    
    # Plot XY trajectory
    axes[0, 2].plot(x_coords, y_coords, 'purple', linewidth=2, marker='o', markersize=3, alpha=0.7)
    axes[0, 2].set_title('XY Trajectory (Bird\'s Eye View)', fontsize=12)
    axes[0, 2].set_xlabel('X (m)')
    axes[0, 2].set_ylabel('Y (m)')
    axes[0, 2].grid(True, alpha=0.3)
    axes[0, 2].set_aspect('equal', adjustable='box')
    # Mark start and end points
    axes[0, 2].scatter(x_coords[0], y_coords[0], color='green', s=100, marker='s', label='Start', zorder=5)
    axes[0, 2].scatter(x_coords[-1], y_coords[-1], color='red', s=100, marker='*', label='Goal', zorder=5)
    axes[0, 2].legend()
    
    # Plot Yaw angles (converted to degrees)
    yaw_degrees = [np.degrees(yaw) for yaw in yaw_angles]
    axes[1, 0].plot(range(len(yaw_degrees)), yaw_degrees, 'r-', linewidth=2, marker='o', markersize=3)
    axes[1, 0].set_title('Vehicle Heading (Yaw) vs Waypoint Index', fontsize=12)
    axes[1, 0].set_xlabel('Waypoint Index')
    axes[1, 0].set_ylabel('Yaw (degrees)')
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].text(0.02, 0.98, f'Range: {min(yaw_degrees):.1f}° to {max(yaw_degrees):.1f}°', 
                    transform=axes[1, 0].transAxes, verticalalignment='top',
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='lightcoral', alpha=0.8))
    
    # Plot Steering angles (converted to degrees)
    steer_degrees = [np.degrees(steer) for steer in steer_angles]
    axes[1, 1].plot(range(len(steer_degrees)), steer_degrees, 'm-', linewidth=2, marker='o', markersize=3)
    axes[1, 1].axhline(y=0, color='black', linestyle='--', alpha=0.5)
    axes[1, 1].set_title('Steering Angle vs Waypoint Index', fontsize=12)
    axes[1, 1].set_xlabel('Waypoint Index')
    axes[1, 1].set_ylabel('Steering Angle (degrees)')
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].text(0.02, 0.98, f'Range: {min(steer_degrees):.1f}° to {max(steer_degrees):.1f}°', 
                    transform=axes[1, 1].transAxes, verticalalignment='top',
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='plum', alpha=0.8))
    
    # Plot Direction changes (Forward/Backward)
    direction_indices = []
    forward_indices = []
    backward_indices = []
    
    for i, direction in enumerate(directions):
        if direction == 'FORWARD':
            forward_indices.append(i)
        else:
            backward_indices.append(i)
    
    if forward_indices:
        axes[1, 2].scatter(forward_indices, [1]*len(forward_indices), 
                          color='green', s=30, alpha=0.7, label='Forward', marker='>')
    if backward_indices:
        axes[1, 2].scatter(backward_indices, [0]*len(backward_indices), 
                          color='red', s=30, alpha=0.7, label='Backward', marker='<')
    
    axes[1, 2].set_title('Driving Direction vs Waypoint Index', fontsize=12)
    axes[1, 2].set_xlabel('Waypoint Index')
    axes[1, 2].set_ylabel('Direction')
    axes[1, 2].set_yticks([0, 1])
    axes[1, 2].set_yticklabels(['Backward', 'Forward'])
    axes[1, 2].grid(True, alpha=0.3)
    axes[1, 2].legend()
    
    # Calculate additional statistics
    total_distance = sum(np.sqrt((x_coords[i+1] - x_coords[i])**2 + (y_coords[i+1] - y_coords[i])**2) 
                        for i in range(len(x_coords)-1))
    
    # Count direction changes
    direction_changes = 0
    for i in range(1, len(directions)):
        if directions[i] != directions[i-1]:
            direction_changes += 1
    
    # Calculate curvature changes (smoothness indicator)
    yaw_changes = [abs(yaw_angles[i+1] - yaw_angles[i]) for i in range(len(yaw_angles)-1)]
    max_yaw_change = max(yaw_changes) if yaw_changes else 0
    avg_yaw_change = np.mean(yaw_changes) if yaw_changes else 0
    
    # Add a text box with comprehensive statistics
    stats_text = f"""Path Statistics Summary:
• Total waypoints: {len(path)}
• Total distance: {total_distance:.2f} m
• Avg waypoint spacing: {total_distance/(len(path)-1):.3f} m

Steering Analysis:
• Max steering angle: {max([abs(s) for s in steer_degrees]):.1f}°
• Avg |steering|: {np.mean([abs(s) for s in steer_degrees]):.1f}°
• Steering reversals: {sum(1 for i in range(1, len(steer_degrees)) if steer_degrees[i]*steer_degrees[i-1] < 0)}

Heading & Smoothness:
• Total heading change: {abs(max(yaw_degrees) - min(yaw_degrees)):.1f}°
• Max turn rate: {np.degrees(max_yaw_change):.1f}°/step
• Avg turn rate: {np.degrees(avg_yaw_change):.1f}°/step

Motion Direction:
• Forward segments: {len(forward_indices)}
• Backward segments: {len(backward_indices)}
• Direction changes: {direction_changes}"""
    
    fig.text(0.02, 0.02, stats_text, fontsize=9, verticalalignment='bottom',
             bbox=dict(boxstyle='round,pad=0.5', facecolor='lightyellow', alpha=0.9))
    
    plt.tight_layout()
    plt.subplots_adjust(bottom=0.25)  # Make room for statistics
    plt.show()


def visualize_planning_result(planner: HybridAStar, path: Optional[List[Node]], start: State, goal: State) -> None:
    """Helper function to visualize planning results"""
    if not path:
        print("No path to visualize")
        return
    
    # Create visualizer
    visualizer: HybridAStarVisualizer = HybridAStarVisualizer()
    
    # Get visualization data from planner
    viz_data = planner.get_visualization_data()
    
    # Visualize the node path with all parameters explicitly
    visualizer.visualize_node_path(
        path_nodes=path,
        start=start,
        goal=goal,
        planner_instance=planner,
        explored_nodes=viz_data['explored_nodes'],
        simulation_trajectories=viz_data['simulation_trajectories'],
        obstacle_map=viz_data['obstacle_map'],
        map_origin_x=viz_data['map_origin_x'],
        map_origin_y=viz_data['map_origin_y'],
        grid_resolution=viz_data['grid_resolution'],
        vehicle_model=viz_data['vehicle_model'],
        show_exploration=True,
        show_trajectories=True,
        show_costs=False  # 默认隐藏成本面板
    )
    
    # Also display the path data in a separate window
    visualize_path_data(path)


def create_obstacle_map_scenario1() -> np.ndarray:
    """Create a simple obstacle map with corridors"""
    map_size: int = 60
    obstacle_map: np.ndarray = np.zeros((map_size, map_size))
    
    # Create walls
    obstacle_map[0:5, :] = 1     # Top wall
    obstacle_map[-5:, :] = 1     # Bottom wall
    obstacle_map[:, 0:5] = 1     # Left wall
    obstacle_map[:, -5:] = 1     # Right wall
    
    # Create some obstacles
    obstacle_map[20:40, 15:20] = 1   # Vertical obstacle
    obstacle_map[25:30, 30:50] = 1   # Horizontal obstacle
    obstacle_map[10:15, 35:45] = 1   # Small rectangle
    
    return obstacle_map


def demo_basic_navigation() -> bool:
    """Demo basic navigation with obstacles"""
    print("=== Path Data Visualization Demo ===")
    
    # Create vehicle model
    vehicle: VehicleModel = VehicleModel(wheelbase=2.5, max_steer=np.pi/3)
    
    # Create hybrid A* planner with specific parameters
    planner: HybridAStar = HybridAStar(
        vehicle_model=vehicle,
        grid_resolution=0.5,
        angle_resolution=np.pi/8,
        steer_resolution=np.pi/16,
        velocity=3.0,
        simulation_time=0.8,
        dt=0.1
    )
    
    # Set cost weights for smooth driving
    planner.w_steer = 8.0      # Moderate steering penalty
    planner.w_turn = 12.0      # Turn penalty
    planner.w_cusp = 100.0     # High cusp penalty
    planner.w_path = 3.0       # Path smoothness
    
    # Create obstacle map
    obstacle_map: np.ndarray = create_obstacle_map_scenario1()
    planner.set_obstacle_map(obstacle_map, origin_x=-5, origin_y=-5)
    
    # Define start and goal
    start: State = State(x=2.0, y=2.0, yaw=0, direction=DirectionMode.FORWARD)
    goal: State = State(x=22.0, y=20.0, yaw=np.pi/2, direction=DirectionMode.FORWARD)
    
    # Plan path
    print("Planning path...")
    path: Optional[List[Node]] = planner.plan_path(start, goal, max_iterations=3000)
    
    if path:
        print(f"✓ Path found with {len(path)} waypoints")
        visualize_planning_result(planner, path, start, goal)
        return True
    else:
        print("✗ No path found!")
        return False


def main() -> None:
    """Run path data visualization demo"""
    print("Path Data Visualization Demo")
    print("=============================")
    print("This demo shows the new path data visualization window with:")
    print("- X, Y coordinates over waypoint index")
    print("- Vehicle heading (yaw) angle")
    print("- Steering angles")
    print("- Driving direction (forward/backward)")
    print("- XY trajectory bird's eye view")
    print("- Comprehensive path statistics")
    print("")
    
    result = demo_basic_navigation()
    
    if result:
        print("\n🎉 Demo completed successfully!")
        print("You should see two windows:")
        print("1. Main path visualization with obstacles and exploration")
        print("2. Path data analysis window with detailed trajectory information")
    else:
        print("\n⚠️ Demo failed - no path found")


if __name__ == "__main__":
    main()
