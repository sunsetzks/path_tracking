"""
Demo showing the separated Hybrid A* algorithm and visualization

This demonstrates how to use the algorithm and visualizer separately.
"""

import numpy as np
from .hybrid_astar import HybridAStar, VehicleModel, State, DirectionMode
from .visualizer import HybridAStarVisualizer


def main():
    """Main demo function"""
    print("Hybrid A* Path Planning Demo - Separated Algorithm & Visualization")
    print("=" * 70)
    
    # Create vehicle model
    vehicle = VehicleModel(wheelbase=2.5, max_steer=np.pi/4)
    
    # Create hybrid A* planner
    planner = HybridAStar(
        vehicle_model=vehicle,
        grid_resolution=0.5,
        angle_resolution=np.pi/8,
        velocity=2.0,
        simulation_time=0.5,
        dt=0.1
    )
    
    # Create simple obstacle map (optional)
    map_size = 50
    obstacle_map = np.zeros((map_size, map_size))
    
    # Add some obstacles
    obstacle_map[20:30, 15:25] = 1  # Rectangle obstacle
    obstacle_map[10:15, 35:45] = 1  # Another obstacle
    obstacle_map[5:8, 5:15] = 1     # Small obstacle
    
    planner.set_obstacle_map(obstacle_map, origin_x=-10, origin_y=-10)
    
    # Define start and goal states
    start = State(x=-5.0, y=-5.0, yaw=np.pi/4, direction=DirectionMode.FORWARD)
    goal = State(x=15.0, y=15.0, yaw=np.pi/2, direction=DirectionMode.FORWARD)
    
    print(f"Planning path from ({start.x:.1f}, {start.y:.1f}) to ({goal.x:.1f}, {goal.y:.1f})")
    
    # Plan path (pure algorithm, no visualization dependencies)
    path = planner.plan_path(start, goal, max_iterations=5000)
    
    if path:
        print(f"✓ Path found with {len(path)} waypoints")
        
        # Get statistics from the algorithm
        stats = planner.get_statistics(path)
        print(f"\nPath Statistics:")
        print(f"  Total distance: {stats['total_distance']:.2f} m")
        print(f"  Max steering angle: {np.degrees(stats['max_steering_angle']):.1f}°")
        print(f"  Direction changes: {stats['direction_changes']}")
        print(f"  Nodes explored: {stats['nodes_explored']}")
        print(f"  Trajectories simulated: {stats['trajectories_simulated']}")
        
        # Now use the separate visualizer
        print("\nCreating visualizations...")
        visualizer = HybridAStarVisualizer()
        
        # Get visualization data from the algorithm
        viz_data = planner.get_visualization_data()
        
        # Show enhanced path visualization
        print("1. Enhanced path visualization with exploration details...")
        visualizer.visualize_path(
            path=path,
            start=start,
            goal=goal,
            explored_nodes=viz_data['explored_nodes'],
            simulation_trajectories=viz_data['simulation_trajectories'],
            obstacle_map=viz_data['obstacle_map'],
            map_origin_x=viz_data['map_origin_x'],
            map_origin_y=viz_data['map_origin_y'],
            grid_resolution=viz_data['grid_resolution'],
            vehicle_model=viz_data['vehicle_model'],
            show_exploration=True,
            show_trajectories=True,
            show_costs=True
        )
        
        # Show search progress visualization
        print("2. Search progress visualization...")
        visualizer.visualize_search_progress(
            path=path,
            start=start,
            goal=goal,
            explored_nodes=viz_data['explored_nodes'],
            obstacle_map=viz_data['obstacle_map'],
            map_origin_x=viz_data['map_origin_x'],
            map_origin_y=viz_data['map_origin_y'],
            grid_resolution=viz_data['grid_resolution'],
            max_nodes_to_show=300
        )
        
        # Show detailed search tree
        print("3. Detailed search tree visualization...")
        visualizer.visualize_detailed_search_tree(
            path=path,
            start=start,
            goal=goal,
            explored_nodes=viz_data['explored_nodes'],
            obstacle_map=viz_data['obstacle_map'],
            map_origin_x=viz_data['map_origin_x'],
            map_origin_y=viz_data['map_origin_y'],
            grid_resolution=viz_data['grid_resolution'],
            max_connections=800,
            node_spacing_filter=0.4
        )
        
    else:
        print("✗ No path found!")
        stats = planner.get_statistics(None)
        print(f"Search Statistics:")
        print(f"  Nodes explored: {stats['nodes_explored']}")
        print(f"  Trajectories simulated: {stats['trajectories_simulated']}")
        
        # Even if no path found, can show exploration
        if stats['nodes_explored'] > 0:
            print("Showing exploration data...")
            visualizer = HybridAStarVisualizer()
            viz_data = planner.get_visualization_data()
            
            visualizer.visualize_search_progress(
                path=[],
                start=start,
                goal=goal,
                explored_nodes=viz_data['explored_nodes'],
                obstacle_map=viz_data['obstacle_map'],
                map_origin_x=viz_data['map_origin_x'],
                map_origin_y=viz_data['map_origin_y'],
                grid_resolution=viz_data['grid_resolution']
            )


def minimal_example():
    """Minimal example showing algorithm without any visualization"""
    print("\nMinimal Example (Algorithm Only):")
    print("-" * 40)
    
    # Simple setup
    vehicle = VehicleModel()
    planner = HybridAStar(vehicle_model=vehicle)
    
    start = State(x=0.0, y=0.0, yaw=0.0)
    goal = State(x=10.0, y=10.0, yaw=np.pi/2)
    
    # Plan path (no visualization dependencies)
    path = planner.plan_path(start, goal, max_iterations=1000)
    
    if path:
        stats = planner.get_statistics(path)
        print(f"✓ Found path: {stats['path_length_waypoints']} waypoints, "
              f"{stats['total_distance']:.1f}m")
    else:
        print("✗ No path found")


if __name__ == "__main__":
    # Run main demo
    main()
    
    # Run minimal example  
    minimal_example()
    
    print("\nDemo completed!")
    print("Benefits of separation:")
    print("  • Core algorithm has no matplotlib dependency")
    print("  • Visualization is optional and modular") 
    print("  • Algorithm can be used in production without visualization overhead")
    print("  • Different visualization backends can be implemented")
    print("  • Easier to test and maintain")
