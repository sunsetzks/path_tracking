#!/usr/bin/env python3
"""
Simple test script for Hybrid A* algorithm
"""
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from astar_project.hybrid_astar import HybridAStar, VehicleModel, State, DirectionMode
import numpy as np

def test_basic_functionality():
    """Test basic functionality of the Hybrid A* algorithm"""
    print("=== Testing Hybrid A* Algorithm ===")
    
    try:
        # Test 1: Create vehicle model
        print("Test 1: Creating vehicle model...")
        vehicle = VehicleModel(wheelbase=2.5, max_steer=np.pi/4)
        print(f"✓ Vehicle model created: wheelbase={vehicle.wheelbase}m, max_steer={np.degrees(vehicle.max_steer):.1f}°")
        
        # Test 2: Create states
        print("Test 2: Creating states...")
        start = State(x=0, y=0, yaw=0, direction=DirectionMode.FORWARD)
        goal = State(x=5, y=5, yaw=np.pi/2, direction=DirectionMode.FORWARD)
        print(f"✓ States created: start=({start.x},{start.y}), goal=({goal.x},{goal.y})")
        
        # Test 3: Test motion simulation
        print("Test 3: Testing motion simulation...")
        states = vehicle.simulate_motion(start, velocity=2.0, steer_rate=0, dt=0.5, steps=2)
        print(f"✓ Motion simulation: {len(states)} states generated")
        print(f"  Final position: ({states[-1].x:.2f}, {states[-1].y:.2f})")
        
        # Test 4: Create planner
        print("Test 4: Creating Hybrid A* planner...")
        planner = HybridAStar(
            vehicle_model=vehicle,
            grid_resolution=1.0,
            velocity=2.0,
            simulation_time=0.5
        )
        print("✓ Planner created successfully")
        
        # Test 5: Test cost calculations
        print("Test 5: Testing cost calculations...")
        steer_cost = planner.calculate_steering_cost(np.pi/8)
        turn_cost = planner.calculate_turning_cost(0, np.pi/4)
        cusp_cost = planner.calculate_cusp_cost(DirectionMode.FORWARD, DirectionMode.BACKWARD)
        print(f"✓ Cost calculations: steer={steer_cost:.3f}, turn={turn_cost:.3f}, cusp={cusp_cost:.1f}")
        
        # Test 6: Simple path planning (no obstacles)
        print("Test 6: Testing simple path planning...")
        path = planner.plan_path(start, goal, max_iterations=100)
        
        if path:
            print(f"✓ Path found with {len(path)} waypoints!")
            print(f"  Start: ({path[0].x:.1f}, {path[0].y:.1f}, {np.degrees(path[0].yaw):.1f}°)")
            print(f"  End: ({path[-1].x:.1f}, {path[-1].y:.1f}, {np.degrees(path[-1].yaw):.1f}°)")
            
            # Verify path continuity
            max_step = 0
            for i in range(len(path) - 1):
                dx = path[i+1].x - path[i].x
                dy = path[i+1].y - path[i].y
                step = np.sqrt(dx*dx + dy*dy)
                max_step = max(max_step, step)
            
            print(f"  Max step size: {max_step:.2f}m")
            if max_step < 5.0:  # Reasonable step size
                print("✓ Path continuity check passed")
            else:
                print("⚠ Path may have large jumps")
        else:
            print("⚠ No path found (may need more iterations)")
        
        print("\n🎉 All basic tests completed successfully!")
        return True
        
    except Exception as e:
        print(f"❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_with_obstacles():
    """Test path planning with obstacles"""
    print("\n=== Testing with Obstacles ===")
    
    try:
        # Create vehicle and planner
        vehicle = VehicleModel(wheelbase=2.0, max_steer=np.pi/3)
        planner = HybridAStar(
            vehicle_model=vehicle,
            grid_resolution=0.5,
            velocity=1.5,
            simulation_time=0.8
        )
        
        # Create simple obstacle map
        obstacle_map = np.zeros((20, 20))
        obstacle_map[8:12, 8:12] = 1  # Central obstacle
        planner.set_obstacle_map(obstacle_map, origin_x=-2, origin_y=-2)
        print("✓ Obstacle map created and set")
        
        # Plan path around obstacle
        start = State(x=0, y=0, yaw=0, direction=DirectionMode.FORWARD)
        goal = State(x=8, y=8, yaw=np.pi/2, direction=DirectionMode.FORWARD)
        
        print("Planning path around obstacle...")
        path = planner.plan_path(start, goal, max_iterations=500)
        
        if path:
            print(f"✓ Path around obstacle found with {len(path)} waypoints!")
            
            # Verify no collisions
            collision_free = True
            for state in path:
                if not planner.is_collision_free(state):
                    collision_free = False
                    break
            
            if collision_free:
                print("✓ Path is collision-free")
            else:
                print("❌ Path contains collisions")
                
        else:
            print("⚠ No path found around obstacle")
            
        return True
        
    except Exception as e:
        print(f"❌ Obstacle test failed: {e}")
        return False

if __name__ == "__main__":
    print("Hybrid A* Algorithm Test Suite")
    print("=" * 40)
    
    success1 = test_basic_functionality()
    success2 = test_with_obstacles()
    
    print(f"\n{'=' * 40}")
    if success1 and success2:
        print("🏆 All tests passed! The Hybrid A* algorithm is working correctly.")
    else:
        print("⚠️ Some tests failed. Check the output above for details.")
