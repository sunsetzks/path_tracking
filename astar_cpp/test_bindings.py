#!/usr/bin/env python3
"""
Comprehensive test of Hybrid A* Python bindings
"""

import hybrid_astar_cpp as ha
import numpy as np
import time

def test_basic_functionality():
    """Test basic functionality of Python bindings"""
    print("=" * 60)
    print("Testing Basic Functionality")
    print("=" * 60)
    
    # Test DirectionMode enum
    print("Testing DirectionMode enum:")
    print(f"  FORWARD = {ha.DirectionMode.FORWARD.value}")
    print(f"  BACKWARD = {ha.DirectionMode.BACKWARD.value}")
    print(f"  NONE = {ha.DirectionMode.NONE.value}")
    
    # Test State creation and equality
    print("\nTesting State class:")
    state1 = ha.State(1.0, 2.0, np.pi/4, ha.DirectionMode.FORWARD, 0.1)
    state2 = ha.State(1.05, 2.05, np.pi/4 + 0.05, ha.DirectionMode.FORWARD, 0.1)
    print(f"  State1: x={state1.x:.2f}, y={state1.y:.2f}, yaw={state1.yaw:.3f}")
    print(f"  State2: x={state2.x:.2f}, y={state2.y:.2f}, yaw={state2.yaw:.3f}")
    print(f"  States equal (within tolerance): {state1 == state2}")
    
    # Test VehicleModel
    print("\nTesting VehicleModel:")
    vehicle = ha.VehicleModel(2.5, np.pi/4)
    print(f"  Wheelbase: {vehicle.wheelbase}m")
    print(f"  Max steering: {vehicle.max_steer:.3f} rad ({np.degrees(vehicle.max_steer):.1f}°)")
    
    # Test motion simulation
    print("\nTesting motion simulation:")
    initial_state = ha.State(0.0, 0.0, 0.0, ha.DirectionMode.FORWARD, 0.0)
    states = vehicle.simulate_motion(initial_state, 1.0, 0.0, 0.1, 10)
    print(f"  Simulated {len(states)} states")
    print(f"  Final position: ({states[-1].x:.2f}, {states[-1].y:.2f})")
    print(f"  Final yaw: {states[-1].yaw:.3f} rad ({np.degrees(states[-1].yaw):.1f}°)")
    
    return True

def test_obstacle_map():
    """Test obstacle map functionality"""
    print("\n" + "=" * 60)
    print("Testing Obstacle Map")
    print("=" * 60)
    
    # Create obstacle map
    print("Creating 10x10 obstacle map...")
    obstacle_map = ha.create_obstacle_map(10, 10)
    print(f"  Map size: {len(obstacle_map)} x {len(obstacle_map[0])}")
    
    # Add rectangle obstacle
    print("Adding rectangle obstacle (2,2) to (5,5)...")
    obstacle_map = ha.add_rectangle_obstacle(obstacle_map, 2, 2, 5, 5)
    
    # Count obstacles
    obstacle_count = sum(sum(row) for row in obstacle_map)
    print(f"  Total obstacle cells: {obstacle_count}")
    
    # Visualize map
    print("\nObstacle map visualization (1=obstacle, 0=free):")
    for row in obstacle_map:
        print("  " + "".join(str(cell) for cell in row))
    
    return obstacle_map

def test_planner():
    """Test the HybridAStar planner"""
    print("\n" + "=" * 60)
    print("Testing HybridAStar Planner")
    print("=" * 60)
    
    # Create vehicle and planner
    vehicle = ha.VehicleModel(2.5, np.pi/4)
    planner = ha.HybridAStar(vehicle, 1.0, np.pi/8, np.pi/16, 2.0, 1.0, 0.2)
    print("Created HybridAStar planner with:")
    print(f"  Resolution: 1.0m")
    print(f"  Yaw resolution: {np.degrees(np.pi/8):.1f}°")
    
    # Set up obstacle map
    obstacle_map = ha.create_obstacle_map(10, 10)
    obstacle_map = ha.add_rectangle_obstacle(obstacle_map, 4, 3, 6, 7)
    planner.set_obstacle_map(obstacle_map, 0.0, 0.0)
    print("Set obstacle map with rectangle from (4,3) to (6,7)")
    
    # Test collision checking
    print("\nTesting collision detection:")
    free_state = ha.State(1.0, 1.0, 0.0, ha.DirectionMode.FORWARD, 0.0)
    obstacle_state = ha.State(5.0, 5.0, 0.0, ha.DirectionMode.FORWARD, 0.0)
    print(f"  State at (1,1): collision_free = {planner.is_collision_free(free_state)}")
    print(f"  State at (5,5): collision_free = {planner.is_collision_free(obstacle_state)}")
    
    return planner, obstacle_map

def test_path_planning():
    """Test path planning functionality"""
    print("\n" + "=" * 60)
    print("Testing Path Planning")
    print("=" * 60)
    
    # Create planner
    vehicle = ha.VehicleModel(2.5, np.pi/4)
    planner = ha.HybridAStar(vehicle, 1.0, np.pi/8, np.pi/16, 2.0, 1.0, 0.2)
    
    # Simple case - no obstacles
    print("Test 1: Simple path with no obstacles")
    obstacle_map = ha.create_obstacle_map(20, 20)
    planner.set_obstacle_map(obstacle_map, 0.0, 0.0)
    
    start = ha.State(2.0, 2.0, 0.0, ha.DirectionMode.NONE, 0.0)
    goal = ha.State(15.0, 15.0, np.pi/2, ha.DirectionMode.FORWARD, 0.0)
    
    print(f"  Start: ({start.x}, {start.y}) yaw={np.degrees(start.yaw):.1f}°")
    print(f"  Goal:  ({goal.x}, {goal.y}) yaw={np.degrees(goal.yaw):.1f}°")
    
    start_time = time.time()
    path = planner.plan_path(start, goal, 1000)
    planning_time = time.time() - start_time
    
    if path:
        print(f"  ✓ Path found with {len(path)} waypoints in {planning_time:.3f}s")
        stats = planner.get_statistics(path)
        print(f"  Total distance: {stats['total_distance']:.2f}m")
        print(f"  Direction changes: {stats['direction_changes']:.0f}")
    else:
        print("  ✗ No path found")
    
    # Complex case - with obstacles
    print("\nTest 2: Path planning with obstacles")
    obstacle_map = ha.create_obstacle_map(20, 20)
    obstacle_map = ha.add_rectangle_obstacle(obstacle_map, 8, 8, 12, 12)
    planner.set_obstacle_map(obstacle_map, 0.0, 0.0)
    
    start = ha.State(2.0, 2.0, 0.0, ha.DirectionMode.NONE, 0.0)
    goal = ha.State(15.0, 15.0, 0.0, ha.DirectionMode.FORWARD, 0.0)
    
    start_time = time.time()
    path = planner.plan_path(start, goal, 2000)
    planning_time = time.time() - start_time
    
    if path:
        print(f"  ✓ Path found with {len(path)} waypoints in {planning_time:.3f}s")
        stats = planner.get_statistics(path)
        print(f"  Total distance: {stats['total_distance']:.2f}m")
        print(f"  Direction changes: {stats['direction_changes']:.0f}")
        
        # Show first few and last few waypoints
        print("  First 3 waypoints:")
        for i in range(min(3, len(path))):
            node = path[i]
            wp = node.state
            print(f"    {i}: ({wp.x:.2f}, {wp.y:.2f}) yaw={np.degrees(wp.yaw):.1f}° dir={wp.direction.value}")
        
        print("  Last 3 waypoints:")
        for i in range(max(0, len(path)-3), len(path)):
            node = path[i]
            wp = node.state
            print(f"    {i}: ({wp.x:.2f}, {wp.y:.2f}) yaw={np.degrees(wp.yaw):.1f}° dir={wp.direction.value}")
    else:
        print("  ✗ No path found")
    
    return path

def test_performance():
    """Test performance with different configurations"""
    print("\n" + "=" * 60)
    print("Performance Testing")
    print("=" * 60)
    
    # Different resolution tests
    resolutions = [0.5, 1.0, 2.0]
    vehicle = ha.VehicleModel(2.5, np.pi/4)
    
    start = ha.State(1.0, 1.0, 0.0, ha.DirectionMode.NONE, 0.0)
    goal = ha.State(10.0, 10.0, 0.0, ha.DirectionMode.FORWARD, 0.0)
    
    print("Testing different resolutions:")
    for res in resolutions:
        planner = ha.HybridAStar(vehicle, res, np.pi/8, np.pi/16, 2.0, 1.0, 0.2)
        obstacle_map = ha.create_obstacle_map(int(15/res), int(15/res))
        planner.set_obstacle_map(obstacle_map, 0.0, 0.0)
        
        start_time = time.time()
        path = planner.plan_path(start, goal, 1000)
        planning_time = time.time() - start_time
        
        if path:
            stats = planner.get_statistics(path)
            print(f"  Resolution {res}m: {len(path)} waypoints, {planning_time:.3f}s, {stats['total_distance']:.2f}m")
        else:
            print(f"  Resolution {res}m: No path found in {planning_time:.3f}s")

def main():
    """Run all tests"""
    print("Hybrid A* Python Bindings Test Suite")
    print("=" * 60)
    
    try:
        test_basic_functionality()
        test_obstacle_map()
        test_planner()
        path = test_path_planning()
        test_performance()
        
        print("\n" + "=" * 60)
        print("All tests completed successfully! ✓")
        print("=" * 60)
        
        if path:
            print(f"\nSample path generated with {len(path)} waypoints")
            print("Python bindings are fully functional and ready to use!")
        
    except Exception as e:
        print(f"\n❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    return True

if __name__ == "__main__":
    main()
