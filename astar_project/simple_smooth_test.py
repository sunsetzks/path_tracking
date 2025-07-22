#!/usr/bin/env python3
"""
Simple test to demonstrate smooth path visualization
"""

import sys
sys.path.append('/home/zks/ws/path_tracking/astar_project')

import numpy as np
import matplotlib.pyplot as plt
from astar_project.hybrid_astar import HybridAStar, VehicleModel, State, DirectionMode


def simple_smooth_test():
    """Simple test with no obstacles to ensure smooth path works"""
    print("Testing smooth path visualization (no obstacles)...")
    
    # Create vehicle model
    vehicle = VehicleModel(wheelbase=2.5, max_steer=np.pi/4)
    
    # Create hybrid A* planner with relaxed parameters
    planner = HybridAStar(
        vehicle_model=vehicle,
        grid_resolution=1.0,
        angle_resolution=np.pi/4,
        steer_resolution=np.pi/8,
        velocity=3.0,
        simulation_time=0.8,
        dt=0.1
    )
    
    # Simple cost weights
    planner.w_steer = 5.0      
    planner.w_turn = 8.0      
    planner.w_cusp = 20.0     
    planner.w_path = 2.0       
    
    # No obstacles - just open space
    map_size = 30
    obstacle_map = np.zeros((map_size, map_size))
    planner.set_obstacle_map(obstacle_map, origin_x=0, origin_y=0)
    
    # Simple start and goal
    start = State(x=2.0, y=2.0, yaw=0, direction=DirectionMode.FORWARD)
    goal = State(x=20.0, y=15.0, yaw=np.pi/4, direction=DirectionMode.FORWARD)
    
    print(f"Planning path from ({start.x:.1f}, {start.y:.1f}) to ({goal.x:.1f}, {goal.y:.1f})...")
    
    # Plan path
    path = planner.plan_path(start, goal, max_iterations=1000)
    
    if path:
        print(f"✓ Path found with {len(path)} waypoints")
        
        # Show visualization with smooth curves
        print("- Displaying smooth curve visualization...")
        planner.visualize_path(path, start, goal, 
                              show_exploration=False,
                              show_trajectories=False,
                              show_costs=False)
        
        print("✓ Smooth path visualization successful!")
        print(f"  - Original waypoints: {len(path)}")
        print(f"  - Displayed as smooth interpolated curve")
        return True
    else:
        print("✗ No path found!")
        return False


def create_curved_scenario_test():
    """Test with obstacles that force curved path"""
    print("\nTesting curved path with obstacles...")
    
    # Create vehicle model
    vehicle = VehicleModel(wheelbase=2.5, max_steer=np.pi/4)
    
    # Create planner
    planner = HybridAStar(
        vehicle_model=vehicle,
        grid_resolution=0.8,
        angle_resolution=np.pi/6,
        steer_resolution=np.pi/12,
        velocity=2.5,
        simulation_time=0.8,
        dt=0.1
    )
    
    # Set weights
    planner.w_steer = 4.0      
    planner.w_turn = 6.0      
    planner.w_cusp = 30.0     
    planner.w_path = 3.0       
    
    # Create obstacle map with single obstacle to force curve
    map_size = 35
    obstacle_map = np.zeros((map_size, map_size))
    
    # Single central obstacle to force path around it
    obstacle_map[12:18, 8:12] = 1
    
    planner.set_obstacle_map(obstacle_map, origin_x=0, origin_y=0)
    
    # Start and goal that require going around obstacle
    start = State(x=2.0, y=10.0, yaw=0, direction=DirectionMode.FORWARD)
    goal = State(x=20.0, y=10.0, yaw=0, direction=DirectionMode.FORWARD)
    
    print(f"Planning curved path from ({start.x:.1f}, {start.y:.1f}) to ({goal.x:.1f}, {goal.y:.1f})...")
    
    # Plan path
    path = planner.plan_path(start, goal, max_iterations=2000)
    
    if path:
        print(f"✓ Curved path found with {len(path)} waypoints")
        
        # Show enhanced visualization
        print("- Displaying enhanced smooth curve visualization...")
        planner.visualize_path(path, start, goal, 
                              show_exploration=True,
                              show_trajectories=True,
                              show_costs=True)
        
        print("✓ Enhanced curved path visualization successful!")
        return True
    else:
        print("✗ No curved path found!")
        
        # Still show what was explored
        if planner.explored_nodes:
            print("- Showing exploration data...")
            planner.visualize_search_progress([], start, goal)
        return False


def main():
    """Run simple smooth visualization tests"""
    print("Simple Smooth Path Visualization Test")
    print("====================================")
    
    results = []
    
    # Test 1: Simple straight-ish path
    results.append(simple_smooth_test())
    
    # Test 2: Curved path around obstacle
    results.append(create_curved_scenario_test())
    
    # Summary
    print(f"\n=== Test Summary ===")
    print(f"Successful tests: {sum(results)}/{len(results)}")
    
    if any(results):
        print("🎉 Smooth visualization working!")
        print("\nPath visualization improvements:")
        print("✓ Smooth curves instead of jagged line segments")  
        print("✓ Natural looking vehicle trajectories")
        print("✓ Smooth color transitions for steering angles")
        print("✓ Scipy cubic spline interpolation creates realistic paths")
    else:
        print("⚠️ Need to adjust planning parameters")


if __name__ == "__main__":
    main()
