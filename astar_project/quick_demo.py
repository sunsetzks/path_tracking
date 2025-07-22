#!/usr/bin/env python3
"""
Quick demonstration of Hybrid A* algorithm functionality
"""
import sys
import os
sys.path.insert(0, os.path.dirname(__file__))

from astar_project.hybrid_astar import HybridAStar, VehicleModel, State, DirectionMode
import numpy as np

def quick_demo():
    """Quick demonstration of the algorithm"""
    print("=== Hybrid A* Quick Demo ===")
    
    # Create vehicle model
    vehicle = VehicleModel(wheelbase=2.5, max_steer=np.pi/4)
    
    # Create planner with relaxed parameters
    planner = HybridAStar(
        vehicle_model=vehicle,
        grid_resolution=2.0,  # Larger grid for faster search
        angle_resolution=np.pi/4,  # Fewer angles
        velocity=3.0,
        simulation_time=1.0,  # Longer simulation steps
        dt=0.2
    )
    
    # Reduce cost weights for easier pathfinding
    planner.w_steer = 5.0
    planner.w_turn = 5.0
    planner.w_cusp = 20.0
    planner.w_path = 2.0
    
    # Very simple scenario - straight line
    start = State(x=0, y=0, yaw=0, direction=DirectionMode.FORWARD)
    goal = State(x=6, y=0, yaw=0, direction=DirectionMode.FORWARD)  # Straight ahead
    
    print(f"Planning path from ({start.x},{start.y}) to ({goal.x},{goal.y})")
    print("Searching...")
    
    path = planner.plan_path(start, goal, max_iterations=50)
    
    if path:
        print(f"✅ SUCCESS! Path found with {len(path)} waypoints")
        print("\nPath summary:")
        for i, state in enumerate(path[::max(1, len(path)//5)]):  # Show every 5th point
            print(f"  {i:2d}: ({state.x:5.1f}, {state.y:5.1f}) heading={np.degrees(state.yaw):6.1f}° steer={np.degrees(state.steer):6.1f}°")
        
        # Calculate path length
        total_distance = 0
        for i in range(len(path) - 1):
            dx = path[i+1].x - path[i].x
            dy = path[i+1].y - path[i].y
            total_distance += np.sqrt(dx*dx + dy*dy)
        
        print(f"\nPath statistics:")
        print(f"  Total distance: {total_distance:.2f} m")
        print(f"  Direct distance: {np.sqrt((goal.x-start.x)**2 + (goal.y-start.y)**2):.2f} m")
        print(f"  Efficiency: {100 * np.sqrt((goal.x-start.x)**2 + (goal.y-start.y)**2) / total_distance:.1f}%")
        
        return True
    else:
        print("❌ No path found")
        return False

def demo_turning():
    """Demo with turning"""
    print("\n=== Turning Demo ===")
    
    vehicle = VehicleModel(wheelbase=2.0, max_steer=np.pi/3)
    planner = HybridAStar(
        vehicle_model=vehicle,
        grid_resolution=1.5,
        angle_resolution=np.pi/6,
        velocity=2.0,
        simulation_time=0.8
    )
    
    # More lenient settings
    planner.w_steer = 3.0
    planner.w_turn = 3.0
    planner.w_cusp = 15.0
    planner.w_path = 1.0
    
    start = State(x=0, y=0, yaw=0, direction=DirectionMode.FORWARD)
    goal = State(x=3, y=3, yaw=np.pi/2, direction=DirectionMode.FORWARD)  # Turn right
    
    print(f"Planning turning maneuver from ({start.x},{start.y}) to ({goal.x},{goal.y}) with {np.degrees(goal.yaw):.0f}° heading")
    
    path = planner.plan_path(start, goal, max_iterations=100)
    
    if path:
        print(f"✅ Turning path found with {len(path)} waypoints")
        
        # Show key turning points
        max_steer = max(abs(state.steer) for state in path)
        direction_changes = sum(1 for i in range(1, len(path)) 
                              if path[i].direction != path[i-1].direction)
        
        print(f"  Max steering angle: {np.degrees(max_steer):.1f}°")
        print(f"  Direction changes: {direction_changes}")
        
        return True
    else:
        print("❌ No turning path found")
        return False

def show_algorithm_features():
    """Show key features of the algorithm"""
    print("\n=== Algorithm Features Summary ===")
    
    features = [
        "✓ Hybrid A* path planning with continuous state space",
        "✓ Bicycle vehicle model with realistic kinematics",
        "✓ Forward simulation considering steering angle velocity",
        "✓ Multi-objective cost function:",
        "  - Steering angle cost (舵角代价)",
        "  - Turning cost for path smoothness (转弯代价)", 
        "  - Cusp cost for direction changes (尖点代价)",
        "  - Path smoothness cost (路径平滑性代价)",
        "✓ Configurable motion primitives with different steering rates",
        "✓ Collision detection with obstacle maps",
        "✓ Support for forward and backward motion",
        "✓ Adjustable cost weights for different scenarios"
    ]
    
    for feature in features:
        print(feature)

if __name__ == "__main__":
    print("Hybrid A* Path Planning Algorithm")
    print("混合A*路径规划算法演示")
    print("=" * 50)
    
    show_algorithm_features()
    
    success1 = quick_demo()
    success2 = demo_turning()
    
    print(f"\n{'=' * 50}")
    if success1 or success2:
        print("🎉 Hybrid A* algorithm is working!")
        print("   混合A*算法运行成功！")
        print("\nNext steps:")
        print("- Run 'python -m astar_project.demo' for full demonstrations")
        print("- Run 'python -m astar_project.advanced_examples' for complex scenarios")
        print("- Adjust parameters in hybrid_astar.py for your specific use case")
    else:
        print("⚠️ Algorithm needs parameter tuning")
        print("  Consider adjusting grid_resolution, simulation_time, or cost weights")
