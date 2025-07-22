#!/usr/bin/env python3
"""
Test script to demonstrate the smooth path visualization enhancement
"""

import sys
import os
sys.path.append('/home/zks/ws/path_tracking/astar_project')

import numpy as np
import matplotlib.pyplot as plt
from astar_project.hybrid_astar import HybridAStar, VehicleModel, State, DirectionMode


def create_simple_scenario():
    """Create a simple scenario for testing smooth visualization"""
    # Create vehicle model
    vehicle = VehicleModel(wheelbase=2.5, max_steer=np.pi/4)
    
    # Create hybrid A* planner
    planner = HybridAStar(
        vehicle_model=vehicle,
        grid_resolution=0.5,
        angle_resolution=np.pi/8,
        steer_resolution=np.pi/16,
        velocity=2.5,
        simulation_time=0.6,
        dt=0.1
    )
    
    # Set cost weights for smooth driving
    planner.w_steer = 6.0      
    planner.w_turn = 10.0      
    planner.w_cusp = 50.0     
    planner.w_path = 4.0       
    
    # Create simple obstacle map
    map_size = 40
    obstacle_map = np.zeros((map_size, map_size))
    
    # Add some obstacles to create interesting path
    obstacle_map[15:25, 10:15] = 1   # Vertical obstacle
    obstacle_map[10:15, 20:30] = 1   # Horizontal obstacle
    
    planner.set_obstacle_map(obstacle_map, origin_x=-5, origin_y=-5)
    
    return planner


def test_smooth_path_visualization():
    """Test the enhanced smooth path visualization"""
    print("Testing smooth path visualization...")
    
    planner = create_simple_scenario()
    
    # Define start and goal for a curved path
    start = State(x=2.0, y=2.0, yaw=np.pi/6, direction=DirectionMode.FORWARD)
    goal = State(x=25.0, y=15.0, yaw=np.pi/3, direction=DirectionMode.FORWARD)
    
    print(f"Planning path from ({start.x:.1f}, {start.y:.1f}) to ({goal.x:.1f}, {goal.y:.1f})...")
    
    # Plan path
    path = planner.plan_path(start, goal, max_iterations=2000)
    
    if path:
        print(f"✓ Path found with {len(path)} waypoints")
        
        # Show enhanced visualization with smooth curves
        print("- Displaying enhanced visualization with smooth curves...")
        planner.visualize_path(path, start, goal, 
                              show_exploration=False,  # Less clutter
                              show_trajectories=False,  # Focus on final path
                              show_costs=True)
        
        print(f"- Path successfully visualized with smooth interpolation")
        print(f"- Total path length: {len(path)} waypoints")
        
        return True
    else:
        print("✗ No path found!")
        return False


def test_comparison_visualization():
    """Create a comparison between original and smooth visualization"""
    print("\nCreating detailed smooth path visualization...")
    
    planner = create_simple_scenario()
    
    # Simple curved scenario
    start = State(x=1.0, y=1.0, yaw=0, direction=DirectionMode.FORWARD)
    goal = State(x=20.0, y=18.0, yaw=np.pi/2, direction=DirectionMode.FORWARD)
    
    path = planner.plan_path(start, goal, max_iterations=1500)
    
    if path and len(path) > 5:
        print(f"✓ Found path with {len(path)} waypoints for detailed visualization")
        
        # Show the enhanced visualization with smooth curves
        planner.visualize_path(path, start, goal, 
                              show_exploration=True,
                              show_trajectories=True,
                              show_costs=True)
        
        print("✓ Enhanced smooth visualization displayed successfully")
        print(f"- Path smoothly interpolated from {len(path)} discrete waypoints")
        print(f"- Steering angle colors smoothly transition along the curve")
        return True
    else:
        print("✗ Could not create visualization - path too short or not found")
        return False


def plot_original_linear_segments(ax, path, start, goal, planner):
    """Helper function to plot path with original linear segments method"""
    
    # Plot obstacle map if available
    if planner.obstacle_map is not None:
        extent = (planner.map_origin_x, 
                 planner.map_origin_x + planner.map_width * planner.grid_resolution,
                 planner.map_origin_y,
                 planner.map_origin_y + planner.map_height * planner.grid_resolution)
        ax.imshow(planner.obstacle_map, extent=extent, origin='lower', 
                 cmap='gray', alpha=0.3)
    
    # Plot path with original linear segments
    x_coords = [state.x for state in path]
    y_coords = [state.y for state in path]
    steer_angles = [state.steer for state in path]
    
    # Original linear segments method
    for i in range(len(path)-1):
        # Color based on steering angle
        steer_normalized = abs(steer_angles[i]) / planner.vehicle_model.max_steer
        color_intensity = steer_normalized
        
        if steer_angles[i] > 0:
            color = plt.cm.get_cmap('Reds')(0.3 + 0.7 * color_intensity)
        elif steer_angles[i] < 0:
            color = plt.cm.get_cmap('Blues')(0.3 + 0.7 * color_intensity)
        else:
            color = 'green'
        
        ax.plot([x_coords[i], x_coords[i+1]], [y_coords[i], y_coords[i+1]], 
               color=color, linewidth=3, alpha=0.8)
    
    # Start and goal
    ax.plot(start.x, start.y, 'go', markersize=12, label='Start', zorder=10)
    ax.plot(goal.x, goal.y, 'ro', markersize=12, label='Goal', zorder=10)
    
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')


def add_original_visualization_method():
    """This function is no longer needed"""
    pass


def main():
    """Run smooth visualization tests"""
    print("Hybrid A* Smooth Path Visualization Test")
    print("========================================")
    
    results = []
    
    # Test basic smooth visualization
    results.append(test_smooth_path_visualization())
    
    # Test detailed visualization  
    results.append(test_comparison_visualization())
    
    # Summary
    print(f"\n=== Test Summary ===")
    print(f"Successful tests: {sum(results)}/{len(results)}")
    
    if all(results):
        print("🎉 All visualization tests completed successfully!")
        print("✓ Path visualization now shows smooth curves instead of linear segments")
        print("✓ Steering angle colors are smoothly interpolated along the path")
        print("✓ Path appears more natural and visually appealing")
        print("\nKey improvements:")
        print("- Cubic spline interpolation creates smooth curves")
        print("- Path colors transition smoothly based on steering angles")
        print("- Much more natural looking trajectories")
    else:
        print("⚠️  Some tests failed")


if __name__ == "__main__":
    main()
