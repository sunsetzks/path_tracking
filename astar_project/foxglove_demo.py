#!/usr/bin/env python3
"""
Foxglove Visualization Demo for Hybrid A* Path Planning

This script demonstrates the updated Foxglove visualization capabilities
using the modern Foxglove SDK API. It provides both live Foxglove visualization
and matplotlib fallback.

Usage:
    python foxglove_demo.py

Requirements:
    - foxglove-sdk (pip install foxglove-sdk)
    - matplotlib (for fallback visualization)
    
Author: AI Assistant
Date: 2025-07-24
"""

import asyncio
import sys
import os
import numpy as np

# Add project path
sys.path.append('/home/zks/ws/path_tracking/astar_project')

try:
    from astar_project.hybrid_astar import VehicleModel, HybridAStar, State, DirectionMode
    print("✓ Successfully imported Hybrid A* modules")
except ImportError as e:
    print(f"✗ Failed to import Hybrid A* modules: {e}")
    sys.exit(1)

try:
    from astar_project.foxglove_visualizer import (
        FoxgloveHybridAStarVisualizer, 
        FOXGLOVE_AVAILABLE,
        matplotlib_fallback_visualization
    )
    if FOXGLOVE_AVAILABLE:
        print("✓ Foxglove SDK is available - using live visualization")
    else:
        print("⚠ Foxglove SDK not available - will use matplotlib fallback")
except ImportError as e:
    print(f"⚠ Foxglove visualizer import failed: {e}")
    FOXGLOVE_AVAILABLE = False


def create_test_scenario():
    """Create a test scenario for path planning"""
    print("\n=== Creating Test Scenario ===")
    
    # Create vehicle model
    vehicle = VehicleModel(wheelbase=2.5, max_steer=np.pi/4)
    print(f"✓ Vehicle model: wheelbase={vehicle.wheelbase}m, max_steer={vehicle.max_steer:.2f}rad")
    
    # Create planner
    planner = HybridAStar(
        vehicle_model=vehicle,
        grid_resolution=0.5,
        angle_resolution=np.pi/8,
        velocity=2.0,
        simulation_time=0.5,
        dt=0.1
    )
    print("✓ Hybrid A* planner created")
    
    # Create obstacle map with interesting features
    map_size = 60
    obstacle_map = np.zeros((map_size, map_size))
    
    # Add various obstacles
    obstacle_map[15:25, 10:20] = 1    # Rectangle obstacle
    obstacle_map[35:45, 25:35] = 1    # Another rectangle
    obstacle_map[25:35, 45:55] = 1    # Third rectangle
    
    # Add some scattered obstacles
    obstacle_map[10:12, 30:32] = 1
    obstacle_map[20:22, 15:17] = 1
    obstacle_map[40:42, 10:12] = 1
    
    planner.set_obstacle_map(obstacle_map, origin_x=-15, origin_y=-15)
    print(f"✓ Obstacle map set: {map_size}x{map_size} grid")
    
    # Define challenging start and goal
    start = State(x=-10.0, y=-10.0, yaw=np.pi/4, direction=DirectionMode.NONE)
    goal = State(x=20.0, y=20.0, yaw=np.pi/2, direction=DirectionMode.FORWARD)
    print(f"✓ Start: ({start.x}, {start.y}, {start.yaw:.2f})")
    print(f"✓ Goal:  ({goal.x}, {goal.y}, {goal.yaw:.2f})")
    
    return planner, start, goal, obstacle_map


def test_basic_planning():
    """Test basic path planning functionality"""
    print("\n=== Testing Basic Path Planning ===")
    
    planner, start, goal, obstacle_map = create_test_scenario()
    
    print("Planning path...")
    path = planner.plan_path(start, goal, max_iterations=20000)
    
    if path:
        print(f"✓ Path found with {len(path)} waypoints")
        # Extract states from path nodes
        path_states = [node.state for node in path]
        
        # Get visualization data
        viz_data = planner.get_visualization_data()
        print(f"✓ Visualization data extracted: {len(viz_data.get('explored_nodes', []))} explored nodes")
        
        return path_states, start, goal, viz_data, obstacle_map
    else:
        print("✗ No path found")
        return None, start, goal, {}, obstacle_map


async def test_foxglove_visualization():
    """Test Foxglove live visualization"""
    print("\n=== Testing Foxglove Live Visualization ===")
    
    if not FOXGLOVE_AVAILABLE:
        print("⚠ Skipping Foxglove test - SDK not available")
        return False
    
    # Get planning results
    path_states, start, goal, viz_data, obstacle_map = test_basic_planning()
    
    if not path_states:
        print("✗ Cannot test visualization without valid path")
        return False
    
    try:
        # Create visualizer
        visualizer = FoxgloveHybridAStarVisualizer(port=8765)
        print("✓ Foxglove visualizer created")
        
        # Visualize the path planning results
        visualizer.visualize_path_planning(
            path=path_states,
            start=start,
            goal=goal,
            explored_nodes=viz_data.get('explored_nodes', []),
            obstacle_map=obstacle_map,
            map_origin_x=-15,
            map_origin_y=-15,
            grid_resolution=0.5
        )
        
        print("✓ Visualization data sent to Foxglove")
        print(f"🌐 Connect Foxglove Studio to: ws://localhost:8765")
        print("📊 Topics to visualize:")
        print("   - /hybrid_astar/visualization (path, exploration tree, obstacles)")
        print("   - /hybrid_astar/statistics (planning metrics)")
        print("\n⏱  Keeping server running for 30 seconds...")
        print("   Press Ctrl+C to stop early")
        
        # Keep server running for demonstration
        for i in range(30):
            await asyncio.sleep(1)
            if i % 5 == 0:
                print(f"   {30-i} seconds remaining...")
        
        visualizer.stop_server()
        print("✓ Foxglove server stopped")
        return True
        
    except Exception as e:
        print(f"✗ Foxglove visualization failed: {e}")
        return False


def test_matplotlib_fallback():
    """Test matplotlib fallback visualization"""
    print("\n=== Testing Matplotlib Fallback Visualization ===")
    
    # Get planning results
    path_states, start, goal, viz_data, obstacle_map = test_basic_planning()
    
    if not path_states:
        print("✗ Cannot test visualization without valid path")
        return False
    
    try:
        matplotlib_fallback_visualization(
            path=path_states,
            start=start,
            goal=goal,
            explored_nodes=viz_data.get('explored_nodes', []),
            obstacle_map=obstacle_map,
            map_origin_x=-15,
            map_origin_y=-15,
            grid_resolution=0.5
        )
        print("✓ Matplotlib visualization displayed")
        return True
        
    except ImportError:
        print("⚠ Matplotlib not available for fallback visualization")
        return False
    except Exception as e:
        print(f"✗ Matplotlib visualization failed: {e}")
        return False


async def run_demo():
    """Run the complete demonstration"""
    print("Foxglove Hybrid A* Visualization Demo")
    print("=" * 50)
    
    # Test basic planning
    test_basic_planning()
    
    # Choose visualization method
    if FOXGLOVE_AVAILABLE:
        print("\n🚀 Running Foxglove live visualization demo...")
        success = await test_foxglove_visualization()
        
        if not success:
            print("\n📊 Falling back to matplotlib visualization...")
            test_matplotlib_fallback()
    else:
        print("\n📊 Running matplotlib fallback visualization...")
        test_matplotlib_fallback()
    
    print("\n" + "=" * 50)
    print("Demo completed!")
    
    if FOXGLOVE_AVAILABLE:
        print("\n💡 Tips for using Foxglove Studio:")
        print("1. Install Foxglove Studio from: https://foxglove.dev/download")
        print("2. Connect to WebSocket data source: ws://localhost:8765")
        print("3. Add visualization panels for the topics:")
        print("   - JSON panel for /hybrid_astar/visualization")
        print("   - Raw Messages panel for /hybrid_astar/statistics")
        print("4. Create custom layouts for different views")
    else:
        print("\n💡 To enable Foxglove visualization:")
        print("pip install foxglove-sdk")


def main():
    """Main entry point"""
    try:
        asyncio.run(run_demo())
    except KeyboardInterrupt:
        print("\n🛑 Demo stopped by user")
    except Exception as e:
        print(f"\n❌ Demo failed: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()
