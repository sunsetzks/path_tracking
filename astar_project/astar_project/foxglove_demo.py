"""
Demo script for Foxglove Hybrid A* Visualizer

This script demonstrates how to use the Foxglove-based visualization system
for path planning results. It includes fallback visualization using matplotlib
when Foxglove SDK is not available.
"""

import asyncio
import numpy as np
import matplotlib.pyplot as plt
from typing import List, Optional, Any

# Import our modules
try:
    from .hybrid_astar import VehicleModel, HybridAStar, State, DirectionMode
    from .foxglove_visualizer import FoxgloveHybridAStarVisualizer, FOXGLOVE_AVAILABLE
except ImportError:
    from astar_project.hybrid_astar import VehicleModel, HybridAStar, State, DirectionMode
    try:
        from astar_project.foxglove_visualizer import FoxgloveHybridAStarVisualizer, FOXGLOVE_AVAILABLE
    except ImportError:
        FOXGLOVE_AVAILABLE = False
        print("Foxglove visualizer not available, using matplotlib fallback")


def create_test_scenario():
    """Create a test scenario for path planning"""
    
    # Create vehicle model
    vehicle = VehicleModel(wheelbase=2.5, max_steer=np.pi/4)
    
    # Create planner
    planner = HybridAStar(
        vehicle_model=vehicle,
        grid_resolution=0.5,
        angle_resolution=np.pi/8,
        velocity=2.0,
        simulation_time=0.5,
        dt=0.1
    )
    
    # Create simple obstacle map (same as working example)
    map_size = 50
    obstacle_map = np.zeros((map_size, map_size))
    
    # Add some obstacles
    obstacle_map[20:30, 15:25] = 1  # Rectangle obstacle
    obstacle_map[10:15, 35:45] = 1  # Another obstacle
    
    planner.set_obstacle_map(obstacle_map, origin_x=-10, origin_y=-10)
    
    # Define start and goal (same as working example)
    start = State(x=-5.0, y=-5.0, yaw=np.pi/4, direction=DirectionMode.NONE)
    goal = State(x=15.0, y=15.0, yaw=np.pi/2, direction=DirectionMode.FORWARD)
    
    return planner, start, goal


def matplotlib_fallback_visualization(path: List[State], start: State, goal: State,
                                    explored_nodes: Optional[List] = None,
                                    simulation_trajectories: Optional[List[Any]] = None,
                                    obstacle_map: Optional[np.ndarray] = None,
                                    map_origin_x: float = 0, map_origin_y: float = 0,
                                    grid_resolution: float = 1.0,
                                    vehicle_model: Optional[VehicleModel] = None):
    """Fallback visualization using matplotlib when Foxglove is not available"""
    
    plt.figure(figsize=(12, 10))
    
    # Plot obstacle map
    if obstacle_map is not None:
        height, width = obstacle_map.shape
        extent = (map_origin_x, map_origin_x + width * grid_resolution,
                 map_origin_y, map_origin_y + height * grid_resolution)
        plt.imshow(obstacle_map, extent=extent, cmap='gray', alpha=0.8, origin='lower')
    
    # Plot exploration nodes
    if explored_nodes:
        exploration_x = []
        exploration_y = []
        for node in explored_nodes:
            exploration_x.append(node.state.x)
            exploration_y.append(node.state.y)
            
            # Draw connection to parent
            if node.parent:
                plt.plot([node.parent.state.x, node.state.x],
                        [node.parent.state.y, node.state.y],
                        'c-', alpha=0.3, linewidth=0.5)
        
        plt.scatter(exploration_x, exploration_y, c='cyan', s=1, alpha=0.6, label='Explored nodes')
    
    # Plot final path
    if path:
        path_x = [state.x for state in path]
        path_y = [state.y for state in path]
        plt.plot(path_x, path_y, 'r-', linewidth=3, label='Final path')
        
        # Plot vehicle orientations
        step = max(1, len(path) // 20)
        for i in range(0, len(path), step):
            state = path[i]
            dx = 1.0 * np.cos(state.yaw)
            dy = 1.0 * np.sin(state.yaw)
            plt.arrow(state.x, state.y, dx, dy, head_width=0.3, head_length=0.3,
                     fc='blue', ec='blue', alpha=0.7)
    
    # Plot start and goal
    plt.scatter(start.x, start.y, c='green', s=100, marker='s', label='Start', zorder=10)
    plt.scatter(goal.x, goal.y, c='red', s=100, marker='*', label='Goal', zorder=10)
    
    # Start and goal arrows
    start_dx = 1.5 * np.cos(start.yaw)
    start_dy = 1.5 * np.sin(start.yaw)
    plt.arrow(start.x, start.y, start_dx, start_dy, head_width=0.4, head_length=0.4,
             fc='green', ec='darkgreen', linewidth=2, zorder=10)
    
    goal_dx = 1.5 * np.cos(goal.yaw)
    goal_dy = 1.5 * np.sin(goal.yaw)
    plt.arrow(goal.x, goal.y, goal_dx, goal_dy, head_width=0.4, head_length=0.4,
             fc='red', ec='darkred', linewidth=2, zorder=10)
    
    plt.xlabel('X (m)')
    plt.ylabel('Y (m)')
    plt.title('Hybrid A* Path Planning - Matplotlib Visualization')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.tight_layout()
    plt.show()


async def foxglove_demo():
    """Demo using Foxglove visualization"""
    print("Running Foxglove visualization demo...")
    
    # Create test scenario
    planner, start, goal = create_test_scenario()
    
    print("Planning path...")
    path_nodes = planner.plan_path(start, goal, max_iterations=5000)
    
    if not path_nodes:
        print("No path found!")
        return
    
    # Extract states from nodes
    path_states = [node.state for node in path_nodes]
    print(f"Path found with {len(path_states)} waypoints")
    
    # Create Foxglove visualizer
    visualizer = FoxgloveHybridAStarVisualizer(port=8765)
    
    try:
        # Get visualization data from planner
        viz_data = planner.get_visualization_data()
        
        # Start visualization
        print("Starting Foxglove server...")
        await visualizer.start_server()
        
        print(f"Foxglove server running on ws://localhost:8765")
        print("Connect Foxglove Studio to view the visualization")
        print("Available channels:")
        for channel_name in visualizer.channels.values():
            print(f"  - {channel_name}")
        
        # Send visualization data
        visualizer.visualize_path_planning(
            path=path_states,
            start=start,
            goal=goal,
            **viz_data
        )
        
        # Print statistics
        stats = planner.get_statistics(path_nodes)
        print(f"\nPath Statistics:")
        for key, value in stats.items():
            if isinstance(value, float):
                print(f"  {key}: {value:.3f}")
            else:
                print(f"  {key}: {value}")
        
        # Keep server running
        print("\nVisualization active. Press Ctrl+C to stop...")
        while True:
            await asyncio.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\nStopping visualization...")
    finally:
        await visualizer.stop_server()


def matplotlib_demo():
    """Demo using matplotlib fallback"""
    print("Running matplotlib visualization demo...")
    
    # Create test scenario
    planner, start, goal = create_test_scenario()
    
    print("Planning path...")
    path_nodes = planner.plan_path(start, goal, max_iterations=5000)
    
    if not path_nodes:
        print("No path found!")
        return
    
    # Extract states from nodes
    path_states = [node.state for node in path_nodes]
    print(f"Path found with {len(path_states)} waypoints")
    
    # Get visualization data
    viz_data = planner.get_visualization_data()
    
    # Show matplotlib visualization
    matplotlib_fallback_visualization(
        path=path_states,
        start=start,
        goal=goal,
        **viz_data
    )
    
    # Print statistics
    stats = planner.get_statistics(path_nodes)
    print(f"\nPath Statistics:")
    for key, value in stats.items():
        if isinstance(value, float):
            print(f"  {key}: {value:.3f}")
        else:
            print(f"  {key}: {value}")


async def live_planning_demo():
    """Demo of live planning visualization"""
    if not FOXGLOVE_AVAILABLE:
        print("Foxglove not available for live demo")
        return
    
    print("Running live planning visualization demo...")
    
    # Create test scenario
    planner, start, goal = create_test_scenario()
    
    # Create visualizer
    visualizer = FoxgloveHybridAStarVisualizer(port=8765)
    
    try:
        print("Starting live planning with visualization...")
        path_nodes = await visualizer.visualize_live_planning(planner, start, goal)
        
        if path_nodes:
            print(f"Live planning completed! Path found with {len(path_nodes)} waypoints")
        else:
            print("Live planning completed, but no path was found")
        
        # Keep server running for viewing
        print("Visualization server still running. Press Ctrl+C to stop...")
        while True:
            await asyncio.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\nStopping live visualization...")
    finally:
        await visualizer.stop_server()


def main():
    """Main demo function"""
    print("Hybrid A* Foxglove Visualization Demo")
    print("=" * 50)
    
    if FOXGLOVE_AVAILABLE:
        print("Foxglove SDK is available!")
        print("\nChoose demo mode:")
        print("1. Static visualization (Foxglove)")
        print("2. Live planning visualization (Foxglove)")
        print("3. Matplotlib fallback")
        
        try:
            choice = input("\nEnter choice (1-3): ").strip()
        except (KeyboardInterrupt, EOFError):
            print("\nExiting...")
            return
        
        if choice == "1":
            asyncio.run(foxglove_demo())
        elif choice == "2":
            asyncio.run(live_planning_demo())
        elif choice == "3":
            matplotlib_demo()
        else:
            print("Invalid choice, using matplotlib fallback")
            matplotlib_demo()
    else:
        print("Foxglove SDK not available, using matplotlib fallback")
        print("To use Foxglove visualization, install with:")
        print("  pip install foxglove-sdk foxglove-schemas-protobuf")
        print("\nRunning matplotlib demo...")
        matplotlib_demo()


if __name__ == "__main__":
    main()
