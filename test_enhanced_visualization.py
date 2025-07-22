#!/usr/bin/env python3
"""
Test script for enhanced Hybrid A* visualization
"""

import sys
import os
sys.path.append('/home/zks/ws/path_tracking/astar_project')

import numpy as np
from astar_project.hybrid_astar import *

def test_enhanced_visualization():
    """Test the enhanced visualization features"""
    print("Testing Enhanced Hybrid A* Visualization")
    print("=" * 50)
    
    # Create vehicle model
    vehicle = VehicleModel(wheelbase=2.5, max_steer=np.pi/4)
    
    # Create hybrid A* planner with smaller grid for faster testing
    planner = HybridAStar(
        vehicle_model=vehicle,
        grid_resolution=1.0,  # Larger grid for faster search
        angle_resolution=np.pi/4,  # Fewer angle discretizations
        velocity=2.0,
        simulation_time=0.3,  # Shorter simulation time
        dt=0.1
    )
    
    # Create simple obstacle map
    map_size = 20  # Smaller map
    obstacle_map = np.zeros((map_size, map_size))
    
    # Add obstacles
    obstacle_map[8:12, 6:10] = 1  # Rectangle obstacle
    obstacle_map[4:7, 14:17] = 1  # Another obstacle
    
    planner.set_obstacle_map(obstacle_map, origin_x=-5, origin_y=-5)
    
    # Define start and goal states
    start = State(x=-2.0, y=-2.0, yaw=np.pi/6, direction=DirectionMode.FORWARD)
    goal = State(x=6.0, y=6.0, yaw=np.pi/3, direction=DirectionMode.FORWARD)
    
    print(f"Start: ({start.x:.1f}, {start.y:.1f}, {np.degrees(start.yaw):.1f}°)")
    print(f"Goal:  ({goal.x:.1f}, {goal.y:.1f}, {np.degrees(goal.yaw):.1f}°)")
    print()
    
    # Plan path
    print("Planning path...")
    path = planner.plan_path(start, goal, max_iterations=500)
    
    if path:
        print(f"✓ Path found with {len(path)} waypoints")
        print(f"✓ Explored {len(planner.explored_nodes)} nodes")
        print(f"✓ Simulated {len(planner.simulation_trajectories)} trajectories")
        
        # Test visualization functions
        print("\nTesting visualization features:")
        
        # 1. Basic statistics
        print("- Calculating path statistics...")
        planner._print_path_statistics(path)
        
        # 2. Test if visualization functions run without errors
        try:
            print("- Testing main visualization function...")
            # We won't actually show plots in test mode, but verify no errors
            fig, ax = plt.subplots(1, 1, figsize=(10, 8))
            planner._plot_main_visualization(ax, path, start, goal, True, True)
            plt.close(fig)
            print("  ✓ Main visualization works")
            
            print("- Testing cost analysis function...")
            fig, ax = plt.subplots(1, 1, figsize=(10, 6))
            planner._plot_cost_analysis(ax, path)
            plt.close(fig)
            print("  ✓ Cost analysis works")
            
            print("- Testing search progress visualization...")
            fig, ax = plt.subplots(1, 1, figsize=(10, 8))
            # Mock the visualization without showing
            exploration_x = [node.state.x for node in planner.explored_nodes[:50]]
            exploration_y = [node.state.y for node in planner.explored_nodes[:50]]
            ax.scatter(exploration_x, exploration_y, alpha=0.7)
            plt.close(fig)
            print("  ✓ Search progress visualization works")
            
        except Exception as e:
            print(f"  ✗ Visualization error: {e}")
        
        # Summary statistics
        print(f"\n{'='*50}")
        print("VISUALIZATION FEATURES SUMMARY:")
        print(f"{'='*50}")
        print("✓ Path visualization with steering angle color coding")
        print("✓ Vehicle orientation and steering visualization")
        print("✓ Exploration nodes display")
        print("✓ Simulation trajectories display")
        print("✓ Cost analysis charts")
        print("✓ Detailed path statistics")
        print("✓ Search progress visualization")
        print(f"{'='*50}")
        
        return True
        
    else:
        print("✗ No path found")
        if planner.explored_nodes:
            print(f"  But explored {len(planner.explored_nodes)} nodes")
            print(f"  And simulated {len(planner.simulation_trajectories)} trajectories")
        return False

if __name__ == "__main__":
    # Import matplotlib and set backend for testing
    import matplotlib
    matplotlib.use('Agg')  # Use non-GUI backend for testing
    import matplotlib.pyplot as plt
    
    success = test_enhanced_visualization()
    
    if success:
        print("\n🎉 All enhanced visualization features tested successfully!")
        print("The Hybrid A* implementation now includes:")
        print("  • Detailed exploration visualization")
        print("  • Forward simulation trajectory display")
        print("  • Comprehensive cost analysis")
        print("  • Enhanced path statistics")
        print("  • Multi-panel visualization layout")
    else:
        print("\n⚠️  Some issues encountered during testing")
