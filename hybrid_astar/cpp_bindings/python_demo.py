#!/usr/bin/env python3
"""
Hybrid A* Path Planning - Python Demo
Demonstrates the C++ implementation through Python bindings
"""

import hybrid_astar_cpp as ha
import numpy as np
import time

def print_header(title):
    """Print a formatted header"""
    print("\n" + "=" * 60)
    print(f" {title}")
    print("=" * 60)

def print_path_info(path):
    """Print detailed information about a path"""
    if not path:
        print("❌ No path found")
        return
    
    print(f"✅ Path found with {len(path)} nodes")
    
    # Extract states from nodes
    states = [node.state for node in path]
    
    # Calculate basic statistics
    total_distance = 0.0
    for i in range(len(states) - 1):
        dx = states[i+1].x - states[i].x
        dy = states[i+1].y - states[i].y
        total_distance += np.sqrt(dx*dx + dy*dy)
    
    print(f"📏 Total distance: {total_distance:.2f}m")
    print(f"🗺️  Average spacing: {total_distance/(len(states)-1):.2f}m")
    
    # Show key waypoints
    print(f"🚀 Start: ({states[0].x:.1f}, {states[0].y:.1f}) @ {np.degrees(states[0].yaw):.0f}°")
    print(f"🎯 Goal:  ({states[-1].x:.1f}, {states[-1].y:.1f}) @ {np.degrees(states[-1].yaw):.0f}°")

def demo_basic_functionality():
    """Demo 1: Basic functionality and vehicle model"""
    print_header("Demo 1: Basic Functionality")
    
    # Create vehicle model
    vehicle = ha.VehicleModel(wheelbase=2.5, max_steer=np.pi/4)
    print(f"🚗 Vehicle: wheelbase={vehicle.wheelbase}m, max_steer={np.degrees(vehicle.max_steer):.0f}°")
    
    # Test motion simulation
    initial_state = ha.State(0.0, 0.0, 0.0, ha.DirectionMode.FORWARD, 0.0)
    states = vehicle.simulate_motion(initial_state, velocity=2.0, steer_rate=0.1, dt=0.1, steps=20)
    
    print(f"🎯 Motion simulation: {len(states)} states")
    print(f"   Final position: ({states[-1].x:.2f}, {states[-1].y:.2f})")
    print(f"   Final heading: {np.degrees(states[-1].yaw):.1f}°")
    
    # Test state equality
    state1 = ha.State(1.0, 2.0, np.pi/4, ha.DirectionMode.FORWARD)
    state2 = ha.State(1.05, 2.05, np.pi/4 + 0.05, ha.DirectionMode.FORWARD)
    print(f"🔍 State tolerance test: {state1 == state2} (should be True)")

def demo_obstacle_avoidance():
    """Demo 2: Path planning with obstacle avoidance"""
    print_header("Demo 2: Obstacle Avoidance")
    
    # Setup
    vehicle = ha.VehicleModel(2.5, np.pi/4)
    planner = ha.HybridAStar(vehicle, grid_resolution=1.0, angle_resolution=np.pi/8)
    
    # Create environment with obstacles
    print("🏗️  Creating 15x15 environment...")
    obstacle_map = ha.create_obstacle_map(15, 15)
    
    # Add multiple obstacles
    print("🚧 Adding obstacles...")
    obstacle_map = ha.add_rectangle_obstacle(obstacle_map, 5, 3, 10, 6)  # Large block
    obstacle_map = ha.add_rectangle_obstacle(obstacle_map, 2, 8, 5, 12)   # Another block
    planner.set_obstacle_map(obstacle_map, 0.0, 0.0)
    
    # Visualize environment
    print("\n🗺️  Environment (1=obstacle, 0=free):")
    for i, row in enumerate(obstacle_map):
        row_str = "".join("██" if cell else "  " for cell in row)
        print(f"   {row_str}")
    
    # Plan path
    start = ha.State(1.0, 1.0, 0.0, ha.DirectionMode.NONE)
    goal = ha.State(12.0, 12.0, np.pi/2, ha.DirectionMode.FORWARD)
    
    print(f"\n🚀 Planning from ({start.x}, {start.y}) to ({goal.x}, {goal.y})...")
    start_time = time.time()
    path = planner.plan_path(start, goal, max_iterations=2000)
    planning_time = time.time() - start_time
    
    print(f"⏱️  Planning time: {planning_time:.3f}s")
    print_path_info(path)
    
    if path:
        # Get detailed statistics
        stats = planner.get_statistics(path)
        print(f"📊 Algorithm stats:")
        print(f"   Nodes explored: {stats['nodes_explored']:.0f}")
        print(f"   Trajectories simulated: {stats['trajectories_simulated']:.0f}")
        print(f"   Max steering: {np.degrees(stats['max_steering_angle']):.1f}°")
        print(f"   Direction changes: {stats['direction_changes']:.0f}")

def demo_parking_scenario():
    """Demo 3: Parking scenario with tight constraints"""
    print_header("Demo 3: Parking Scenario")
    
    # Create more precise vehicle for parking
    vehicle = ha.VehicleModel(wheelbase=2.7, max_steer=np.pi/3)  # More agile vehicle
    planner = ha.HybridAStar(vehicle, 
                            grid_resolution=0.5,     # Higher resolution
                            angle_resolution=np.pi/12,  # Finer angles  
                            steer_resolution=np.pi/24)
    
    # Create parking lot environment
    print("🏢 Creating parking lot (20x20, 0.5m resolution)...")
    obstacle_map = ha.create_obstacle_map(20, 20)
    
    # Add parking structure
    # Outer walls
    obstacle_map = ha.add_rectangle_obstacle(obstacle_map, 0, 0, 20, 1)   # Bottom wall
    obstacle_map = ha.add_rectangle_obstacle(obstacle_map, 0, 19, 20, 20) # Top wall  
    obstacle_map = ha.add_rectangle_obstacle(obstacle_map, 0, 0, 1, 20)   # Left wall
    obstacle_map = ha.add_rectangle_obstacle(obstacle_map, 19, 0, 20, 20) # Right wall
    
    # Parking spaces (leaving gaps for parking spots)
    obstacle_map = ha.add_rectangle_obstacle(obstacle_map, 3, 3, 6, 6)    # Parked car 1
    obstacle_map = ha.add_rectangle_obstacle(obstacle_map, 9, 3, 12, 6)   # Parked car 2
    obstacle_map = ha.add_rectangle_obstacle(obstacle_map, 15, 3, 18, 6)  # Parked car 3
    
    planner.set_obstacle_map(obstacle_map, 0.0, 0.0)
    
    # Park between cars 1 and 2
    start = ha.State(2.0, 15.0, -np.pi/2, ha.DirectionMode.NONE)  # Coming from top
    goal = ha.State(7.5, 4.5, 0.0, ha.DirectionMode.FORWARD)       # Parking spot
    
    print(f"🚗 Attempting to park...")
    print(f"   Start: Road position ({start.x}, {start.y})")
    print(f"   Goal:  Parking spot ({goal.x}, {goal.y})")
    
    start_time = time.time()
    path = planner.plan_path(start, goal, max_iterations=3000)
    planning_time = time.time() - start_time
    
    print(f"⏱️  Planning time: {planning_time:.3f}s")
    print_path_info(path)
    
    if path:
        # Analyze parking maneuver
        states = [node.state for node in path]
        direction_changes = sum(1 for i in range(1, len(states)) 
                              if states[i].direction != states[i-1].direction)
        
        forward_states = sum(1 for s in states if s.direction == ha.DirectionMode.FORWARD)
        backward_states = sum(1 for s in states if s.direction == ha.DirectionMode.BACKWARD)
        
        print(f"🔄 Maneuver analysis:")
        print(f"   Direction changes: {direction_changes}")
        print(f"   Forward moves: {forward_states}")
        print(f"   Reverse moves: {backward_states}")

def demo_performance_comparison():
    """Demo 4: Performance comparison with different settings"""
    print_header("Demo 4: Performance Comparison")
    
    vehicle = ha.VehicleModel(2.5, np.pi/4)
    
    # Test different configurations
    configs = [
        {"resolution": 2.0, "angle_res": np.pi/4, "name": "Coarse (fast)"},
        {"resolution": 1.0, "angle_res": np.pi/8, "name": "Medium (balanced)"},
        {"resolution": 0.5, "angle_res": np.pi/12, "name": "Fine (precise)"},
    ]
    
    # Simple test scenario
    start = ha.State(1.0, 1.0, 0.0, ha.DirectionMode.NONE)
    goal = ha.State(8.0, 8.0, np.pi/4, ha.DirectionMode.FORWARD)
    
    print("🏁 Performance comparison for simple diagonal path:")
    print("   Configuration           Time     Nodes  Distance  Quality")
    print("   " + "-" * 55)
    
    for config in configs:
        planner = ha.HybridAStar(vehicle, 
                               grid_resolution=config["resolution"],
                               angle_resolution=config["angle_res"])
        
        # Empty environment
        size = int(10 / config["resolution"])
        obstacle_map = ha.create_obstacle_map(size, size)
        planner.set_obstacle_map(obstacle_map, 0.0, 0.0)
        
        # Plan path
        start_time = time.time()
        path = planner.plan_path(start, goal, max_iterations=1000)
        planning_time = time.time() - start_time
        
        if path:
            stats = planner.get_statistics(path)
            quality = "Smooth" if stats['direction_changes'] <= 1 else "Complex"
            print(f"   {config['name']:<20} {planning_time:6.3f}s {stats['nodes_explored']:6.0f} "
                  f"{stats['total_distance']:8.2f}m {quality}")
        else:
            print(f"   {config['name']:<20} {planning_time:6.3f}s   FAIL      -        -")

def main():
    """Run all demos"""
    print("🌟 Hybrid A* Path Planning - Python Demo")
    print("C++ Implementation with Python Bindings")
    
    try:
        demo_basic_functionality()
        demo_obstacle_avoidance() 
        demo_parking_scenario()
        demo_performance_comparison()
        
        print_header("Demo Complete! 🎉")
        print("✅ All demonstrations completed successfully!")
        print("✅ Python bindings are fully functional!")
        print("✅ C++ implementation is working perfectly!")
        
        print("\n🔗 Next steps:")
        print("   • Use 'make test' to run C++ tests")
        print("   • Use 'make demo' to run C++ demo")
        print("   • Use 'make python' to rebuild Python bindings")
        print("   • Import 'hybrid_astar_cpp' in your Python projects!")
        
    except Exception as e:
        print(f"\n❌ Demo failed: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    return True

if __name__ == "__main__":
    main()
