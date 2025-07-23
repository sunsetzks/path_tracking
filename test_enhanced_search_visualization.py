#!/usr/bin/env python3
"""
Test enhanced search tree visualization for Hybrid A*
Demonstrates clear visibility of search nodes and connections
"""

import numpy as np
import sys
import os

# Add the astar_project module to path
sys.path.append('/home/zks/ws/path_tracking/astar_project')

from astar_project.hybrid_astar import HybridAStar, VehicleModel, State, DirectionMode

def create_test_scenario():
    """Create a challenging test scenario with obstacles"""
    
    # Create vehicle model
    vehicle = VehicleModel(wheelbase=2.5, max_steer=np.pi/4)
    
    # Create hybrid A* planner with moderate resolution for good visualization
    planner = HybridAStar(
        vehicle_model=vehicle,
        grid_resolution=0.8,  # Slightly coarser for cleaner visualization
        angle_resolution=np.pi/8,
        velocity=2.0,
        simulation_time=0.8,  # Longer simulation time for more nodes
        dt=0.1
    )
    
    # Create complex obstacle map
    map_size = 40
    obstacle_map = np.zeros((map_size, map_size))
    
    # Add various obstacles to create interesting search patterns
    # Vertical wall with gap
    obstacle_map[15:30, 18:20] = 1
    obstacle_map[22:25, 18:20] = 0  # Gap in wall
    
    # L-shaped obstacle
    obstacle_map[8:15, 8:12] = 1
    obstacle_map[8:12, 8:18] = 1
    
    # Diagonal obstacle
    for i in range(15):
        for j in range(3):
            if 25+i < map_size and 5+i+j < map_size:
                obstacle_map[25+i, 5+i+j] = 1
    
    # Circular-like obstacle
    center_x, center_y = 30, 25
    for i in range(map_size):
        for j in range(map_size):
            if (i - center_x)**2 + (j - center_y)**2 < 25:
                obstacle_map[i, j] = 1
    
    planner.set_obstacle_map(obstacle_map, origin_x=-5, origin_y=-5)
    
    return planner, obstacle_map

def run_enhanced_visualization_test():
    """Run test with enhanced search tree visualization"""
    
    print("="*70)
    print("ENHANCED HYBRID A* SEARCH TREE VISUALIZATION TEST")
    print("="*70)
    print("This test demonstrates:")
    print("1. Clear parent-child connections in search tree")
    print("2. Cost-based node coloring and sizing")
    print("3. Enhanced edge visibility with directional arrows")
    print("4. Filtered node display to reduce visual clutter")
    print("="*70)
    
    # Create test scenario
    planner, obstacle_map = create_test_scenario()
    
    # Define challenging start and goal states
    start = State(x=-3.0, y=-3.0, yaw=np.pi/6, direction=DirectionMode.FORWARD)
    goal = State(x=25.0, y=20.0, yaw=np.pi/3, direction=DirectionMode.FORWARD)
    
    print(f"\nStart: ({start.x:.1f}, {start.y:.1f}) at {np.degrees(start.yaw):.1f}°")
    print(f"Goal:  ({goal.x:.1f}, {goal.y:.1f}) at {np.degrees(goal.yaw):.1f}°")
    
    # Plan path
    print("\nRunning Hybrid A* planning...")
    path = planner.plan_path(start, goal, max_iterations=3000)
    
    if path:
        print(f"\n✓ Path found with {len(path)} waypoints")
        
        # Show original visualization with enhanced exploration details
        print("\n1. Enhanced main visualization with clear connections...")
        planner.visualize_path(path, start, goal, 
                              show_exploration=True, 
                              show_trajectories=False,  # Reduce clutter
                              show_costs=True)
        
        # Show detailed search tree structure
        print("\n2. Detailed search tree with parent-child connections...")
        planner.visualize_detailed_search_tree(path, start, goal, 
                                              max_connections=1200, 
                                              node_spacing_filter=0.5)
        
        # Show search progression over time
        print("\n3. Search progression visualization...")
        planner.visualize_search_progress(path, start, goal, max_nodes_to_show=400)
        
        # Print comprehensive statistics
        print("\n" + "="*50)
        print("SEARCH PERFORMANCE ANALYSIS")
        print("="*50)
        
        total_nodes = len(planner.explored_nodes)
        total_trajectories = len(planner.simulation_trajectories)
        
        print(f"Total nodes explored: {total_nodes}")
        print(f"Total trajectories simulated: {total_trajectories}")
        print(f"Success rate: {len(path)}/{total_nodes} = {len(path)/total_nodes*100:.2f}%")
        
        # Calculate search efficiency metrics
        if path:
            path_distance = sum(np.sqrt((path[i+1].x - path[i].x)**2 + 
                                       (path[i+1].y - path[i].y)**2) 
                               for i in range(len(path)-1))
            straight_line_distance = np.sqrt((goal.x - start.x)**2 + (goal.y - start.y)**2)
            
            print(f"Path length: {path_distance:.2f} m")
            print(f"Straight-line distance: {straight_line_distance:.2f} m")
            print(f"Path optimality ratio: {straight_line_distance/path_distance:.2f}")
        
        # Analyze search pattern
        if planner.explored_nodes:
            node_positions = [(n.state.x, n.state.y) for n in planner.explored_nodes]
            x_coords = [pos[0] for pos in node_positions]
            y_coords = [pos[1] for pos in node_positions]
            
            search_area = (max(x_coords) - min(x_coords)) * (max(y_coords) - min(y_coords))
            node_density = total_nodes / search_area if search_area > 0 else 0
            
            print(f"Search area covered: {search_area:.1f} m²")
            print(f"Node density: {node_density:.2f} nodes/m²")
        
    else:
        print("\n✗ No path found!")
        print("Showing exploration data to analyze search behavior...")
        
        if planner.explored_nodes:
            # Even if no path found, show the search tree
            planner.visualize_detailed_search_tree([], start, goal, 
                                                  max_connections=800,
                                                  node_spacing_filter=0.3)
    
    print("\n" + "="*70)
    print("VISUALIZATION FEATURES DEMONSTRATED:")
    print("• Parent-child connections with color gradients")
    print("• Node sizing based on cost and importance")  
    print("• Directional arrows showing search progression")
    print("• Cost-based color mapping for nodes and edges")
    print("• Smart filtering to reduce visual clutter")
    print("• Enhanced obstacle and path rendering")
    print("="*70)

def run_comparison_test():
    """Run a simpler scenario for clear comparison"""
    
    print("\n" + "="*50)
    print("SIMPLE SCENARIO FOR CLEAR VISUALIZATION")
    print("="*50)
    
    # Create simple scenario
    vehicle = VehicleModel(wheelbase=2.5, max_steer=np.pi/4)
    planner = HybridAStar(
        vehicle_model=vehicle,
        grid_resolution=1.0,
        angle_resolution=np.pi/6,
        velocity=1.5,
        simulation_time=1.0,
        dt=0.1
    )
    
    # Simple obstacle map
    map_size = 20
    obstacle_map = np.zeros((map_size, map_size))
    obstacle_map[8:12, 8:12] = 1  # Single square obstacle
    
    planner.set_obstacle_map(obstacle_map, origin_x=0, origin_y=0)
    
    # Simple start and goal
    start = State(x=2.0, y=2.0, yaw=0, direction=DirectionMode.FORWARD)
    goal = State(x=15.0, y=15.0, yaw=np.pi/2, direction=DirectionMode.FORWARD)
    
    print(f"Simple scenario: Start ({start.x}, {start.y}) → Goal ({goal.x}, {goal.y})")
    
    # Plan path
    path = planner.plan_path(start, goal, max_iterations=1000)
    
    if path:
        print(f"Path found with {len(path)} waypoints")
        
        # Show detailed tree with all connections visible
        planner.visualize_detailed_search_tree(path, start, goal, 
                                              max_connections=500,
                                              node_spacing_filter=0.2)

if __name__ == "__main__":
    # Run main enhanced visualization test
    run_enhanced_visualization_test()
    
    # Run simple comparison test
    run_comparison_test()
    
    print("\n🎯 Enhanced search tree visualization test completed!")
    print("The visualizations now clearly show:")
    print("  • All parent-child connections in the search tree")
    print("  • Node importance through size and color")
    print("  • Search progression with directional information")
    print("  • Cost analysis and path quality metrics")
