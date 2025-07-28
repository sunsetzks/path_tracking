#!/usr/bin/env python3
"""
Test script for the updated Foxglove visualizer that shows:
- Exploration nodes as spheres (colored by cost)
- Node simulation trajectories as lines
"""

import asyncio
from typing import List, Optional
import numpy as np
import matplotlib.pyplot as plt
from astar_project.hybrid_astar import State, DirectionMode, VehicleModel, HybridAStar, Node
from astar_project.foxglove_visualizer import FoxgloveHybridAStarVisualizer, FOXGLOVE_AVAILABLE


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


def visualize_planning_result(planner: HybridAStar, path: Optional[List[Node]], start: State, goal: State) -> None:
    """Helper function to visualize planning results using Foxglove"""
    if not path:
        print("No path to visualize")
        return
    
    if not FOXGLOVE_AVAILABLE:
        print("Foxglove SDK not available. Using matplotlib fallback.")
        from astar_project.foxglove_visualizer import matplotlib_fallback_visualization
        
        path_states = [node.state for node in path]
        viz_data = planner.get_visualization_data()
        
        matplotlib_fallback_visualization(
            path=path_states,
            start=start,
            goal=goal,
            explored_nodes=viz_data['explored_nodes'],
            obstacle_map=viz_data['obstacle_map'],
            map_origin_x=viz_data['map_origin_x'],
            map_origin_y=viz_data['map_origin_y'],
            grid_resolution=viz_data['grid_resolution']
        )
        return
    
    # Use Foxglove visualizer
    visualizer = FoxgloveHybridAStarVisualizer(port=8760)
    
    try:
        print("Starting Foxglove visualization server...")
        
        # Get visualization data and convert path
        viz_data = planner.get_visualization_data()
        path_states = [node.state for node in path]
        
        # Use the visualize_path_planning method
        visualizer.visualize_path_planning(
            path=path_states,
            start=start,
            goal=goal,
            explored_nodes=viz_data['explored_nodes'],
            simulation_trajectories=viz_data['simulation_trajectories'],
            obstacle_map=viz_data['obstacle_map'],
            map_origin_x=viz_data['map_origin_x'],
            map_origin_y=viz_data['map_origin_y'],
            grid_resolution=viz_data['grid_resolution'],
            vehicle_model=viz_data['vehicle_model']
        )
        
        print("\n" + "="*50)
        print("📡 Foxglove visualization server is running!")
        print(f"🌐 Connect Foxglove Studio to: ws://localhost:8765")
        print("📊 Subscribe to '/hybrid_astar/scene' topic")
        print("⌨️  Press Ctrl+C to continue...")
        print("="*50)
        
        # Keep server running for a reasonable time
        import time
        time.sleep(10)  # Show for 10 seconds
        
    except KeyboardInterrupt:
        print("\n🛑 Stopping visualizer...")
    finally:
        visualizer.stop_server()


def demo_basic_navigation() -> bool:
    """Demo basic navigation with obstacles"""
    print("=== Demo 1: Basic Navigation ===")
    
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


async def test_updated_visualizer():
    """Test the updated visualizer with exploration nodes as spheres and trajectories as lines"""
    
    if not FOXGLOVE_AVAILABLE:
        print("Foxglove SDK not available. Using matplotlib fallback.")
        test_matplotlib_fallback()
        return
    
    print("Testing updated Foxglove Hybrid A* Visualizer...")
    print("New features:")
    print("- Exploration nodes displayed as spheres (colored by f_cost)")
    print("- Node simulation trajectories displayed as lines")
    
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
    
    # Create simple obstacle map
    map_size = 30
    obstacle_map = np.zeros((map_size, map_size))
    obstacle_map[12:18, 8:15] = 1  # Rectangle obstacle
    obstacle_map[5:10, 20:25] = 1  # Another obstacle
    
    planner.set_obstacle_map(obstacle_map, origin_x=-5, origin_y=-5)
    
    # Define start and goal
    start = State(x=-3.0, y=-3.0, yaw=np.pi/4, direction=DirectionMode.NONE)
    goal = State(x=8.0, y=8.0, yaw=np.pi/2, direction=DirectionMode.FORWARD)
    
    # Create visualizer
    visualizer = FoxgloveHybridAStarVisualizer(port=8765)
    
    try:
        # Plan path with visualization
        print("\nRunning path planning with updated visualization...")
        path = visualizer.visualize_live_planning(planner, start, goal)
        
        if path:
            print(f"✓ Path found with {len(path)} waypoints")
            print(f"✓ Explored {len(planner.explored_nodes)} nodes")
            print(f"✓ Generated {len(planner.simulation_trajectories)} simulation trajectories")
        else:
            print("✗ No path found")
        
        # Keep server running for viewing
        print("\n" + "="*60)
        print("📡 Foxglove visualization server is running!")
        print(f"🌐 Connect Foxglove Studio to: ws://localhost:8765")
        print("📊 In Foxglove Studio:")
        print("   1. Add a 3D panel")
        print("   2. Subscribe to '/hybrid_astar/scene' topic")
        print("   3. Add a Plot panel for '/hybrid_astar/statistics' topic")
        print("\n🔍 What you should see:")
        print("   • Blue to red spheres = exploration nodes (cost gradient)")
        print("   • Green/red lines = simulation trajectories (forward/backward)")
        print("   • Magenta line = final path")
        print("   • Green sphere = start, Red sphere = goal")
        print("   • Gray cubes = obstacles")
        print("\n⌨️  Press Ctrl+C to stop...")
        print("="*60)
        
        while True:
            await asyncio.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\n🛑 Stopping visualizer...")
    finally:
        visualizer.stop_server()


def test_matplotlib_fallback():
    """Test matplotlib fallback visualization"""
    from astar_project.foxglove_visualizer import matplotlib_fallback_visualization
    
    print("Testing matplotlib fallback with updated visualization style...")
    
    # Create vehicle model and planner
    vehicle = VehicleModel(wheelbase=2.5, max_steer=np.pi/4)
    planner = HybridAStar(
        vehicle_model=vehicle,
        grid_resolution=0.5,
        angle_resolution=np.pi/8,
        velocity=2.0,
        simulation_time=0.5,
        dt=0.1
    )
    
    # Simple obstacle map
    map_size = 20
    obstacle_map = np.zeros((map_size, map_size))
    obstacle_map[8:12, 6:10] = 1
    planner.set_obstacle_map(obstacle_map, origin_x=-2, origin_y=-2)
    
    # Plan path
    start = State(x=-1.0, y=-1.0, yaw=0, direction=DirectionMode.NONE)
    goal = State(x=6.0, y=6.0, yaw=np.pi/2, direction=DirectionMode.FORWARD)
    
    path_nodes = planner.plan_path(start, goal, max_iterations=1000)
    
    if path_nodes:
        path_states = [node.state for node in path_nodes]
        
        # Call matplotlib fallback
        matplotlib_fallback_visualization(
            path=path_states,
            start=start,
            goal=goal,
            explored_nodes=planner.explored_nodes,
            obstacle_map=obstacle_map,
            map_origin_x=-2,
            map_origin_y=-2,
            grid_resolution=0.5
        )
        print("✓ Matplotlib visualization displayed")
    else:
        print("✗ No path found for matplotlib test")


def test_foxglove_fallback():
    """Test foxglove visualization with same environment as matplotlib fallback"""
    
    if not FOXGLOVE_AVAILABLE:
        print("Foxglove SDK not available. Cannot run foxglove test.")
        return
    
    print("Testing Foxglove visualization with same environment as matplotlib fallback...")
    
    # Create vehicle model and planner (same as matplotlib test)
    vehicle = VehicleModel(wheelbase=2.5, max_steer=np.pi/4)
    planner = HybridAStar(
        vehicle_model=vehicle,
        grid_resolution=0.5,
        angle_resolution=np.pi/8,
        velocity=2.0,
        simulation_time=0.5,
        dt=0.1
    )
    
    # Simple obstacle map (same as matplotlib test)
    map_size = 20
    obstacle_map = np.zeros((map_size, map_size))
    obstacle_map[8:12, 6:10] = 1
    planner.set_obstacle_map(obstacle_map, origin_x=-2, origin_y=-2)
    
    # Plan path (same as matplotlib test)
    start = State(x=-1.0, y=-1.0, yaw=0, direction=DirectionMode.NONE)
    goal = State(x=6.0, y=6.0, yaw=np.pi/2, direction=DirectionMode.FORWARD)
    
    path_nodes = planner.plan_path(start, goal, max_iterations=1000)
    
    if path_nodes:
        path_states = [node.state for node in path_nodes]
        
        # Use Foxglove visualizer
        visualizer = FoxgloveHybridAStarVisualizer(port=8766)
        
        try:
            print("Starting Foxglove visualization server...")
            
            # Get visualization data
            viz_data = planner.get_visualization_data()
            
            # Use the visualize_path_planning method
            visualizer.visualize_path_planning(
                path=path_states,
                start=start,
                goal=goal,
                explored_nodes=viz_data['explored_nodes'],
                simulation_trajectories=viz_data['simulation_trajectories'],
                obstacle_map=viz_data['obstacle_map'],
                map_origin_x=viz_data['map_origin_x'],
                map_origin_y=viz_data['map_origin_y'],
                grid_resolution=viz_data['grid_resolution'],
                vehicle_model=viz_data['vehicle_model']
            )
            
            print("\n" + "="*50)
            print("📡 Foxglove visualization server is running!")
            print(f"🌐 Connect Foxglove Studio to: ws://localhost:8766")
            print("📊 Subscribe to '/hybrid_astar/scene' topic")
            print("🔍 Same environment as matplotlib test but in 3D!")
            print("⌨️  Press Ctrl+C to continue...")
            print("="*50)
            
            # Keep server running for a reasonable time
            import time
            time.sleep(10)  # Show for 10 seconds
            
            print("✓ Foxglove visualization displayed")
            
        except KeyboardInterrupt:
            print("\n🛑 Stopping visualizer...")
        finally:
            visualizer.stop_server()
    else:
        print("✗ No path found for foxglove test")


if __name__ == "__main__":
    # demo_success = demo_basic_navigation()
    # test_matplotlib_fallback()
    test_foxglove_fallback()


