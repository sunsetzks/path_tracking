"""
Hybrid A* Algorithm Demo
Demonstration of the hybrid A* path planning algorithm with various scenarios.
"""

import numpy as np
import matplotlib.pyplot as plt
from astar_project.hybrid_astar import HybridAStar, VehicleModel, State, DirectionMode


def create_obstacle_map_scenario1():
    """Create a simple obstacle map with corridors"""
    map_size = 60
    obstacle_map = np.zeros((map_size, map_size))
    
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


def create_obstacle_map_scenario2():
    """Create a parking lot scenario"""
    map_size = 50
    obstacle_map = np.zeros((map_size, map_size))
    
    # Parking spaces
    for i in range(5, 45, 8):
        obstacle_map[10:15, i:i+6] = 1  # Parking spots top
        obstacle_map[35:40, i:i+6] = 1  # Parking spots bottom
    
    # Central divider
    obstacle_map[22:28, 10:40] = 1
    
    return obstacle_map


def demo_basic_navigation():
    """Demo basic navigation with obstacles"""
    print("=== Demo 1: Basic Navigation ===")
    
    # Create vehicle model
    vehicle = VehicleModel(wheelbase=2.5, max_steer=np.pi/3)
    
    # Create hybrid A* planner with specific parameters
    planner = HybridAStar(
        vehicle_model=vehicle,
        grid_resolution=0.5,
        angle_resolution=np.pi/8,
        velocity=3.0,
        simulation_time=0.8,
        dt=0.1
    )
    
    # Set cost weights for smooth driving
    planner.w_steer = 8.0      # Moderate steering penalty
    planner.w_turn = 12.0      # Turn penalty
    planner.w_cusp = 100.0     # High cusp penalty
    planner.w_path = 3.0       # Path smoothness
    
    # Create obstacle map
    obstacle_map = create_obstacle_map_scenario1()
    planner.set_obstacle_map(obstacle_map, origin_x=-5, origin_y=-5)
    
    # Define start and goal
    start = State(x=2.0, y=2.0, yaw=0, direction=DirectionMode.FORWARD)
    goal = State(x=22.0, y=20.0, yaw=np.pi/2, direction=DirectionMode.FORWARD)
    
    # Plan path
    print("Planning path...")
    path = planner.plan_path(start, goal, max_iterations=3000)
    
    if path:
        print(f"✓ Path found with {len(path)} waypoints")
        planner.visualize_path(path, start, goal)
        return True
    else:
        print("✗ No path found!")
        return False


def demo_parallel_parking():
    """Demo parallel parking maneuver"""
    print("\n=== Demo 2: Parallel Parking ===")
    
    # Create vehicle model
    vehicle = VehicleModel(wheelbase=2.5, max_steer=np.pi/4)
    
    # Create planner optimized for tight maneuvering
    planner = HybridAStar(
        vehicle_model=vehicle,
        grid_resolution=0.3,
        angle_resolution=np.pi/12,
        velocity=1.5,  # Slower speed for precision
        simulation_time=0.6,
        dt=0.1
    )
    
    # Adjust cost weights for parking
    planner.w_steer = 5.0      # Lower steering penalty
    planner.w_turn = 8.0       # Lower turn penalty
    planner.w_cusp = 30.0      # Lower cusp penalty to allow reversing
    planner.w_path = 2.0
    
    # Create parking scenario
    obstacle_map = create_obstacle_map_scenario2()
    planner.set_obstacle_map(obstacle_map, origin_x=-5, origin_y=-5)
    
    # Start: in the road, Goal: in parking space
    start = State(x=5.0, y=25.0, yaw=0, direction=DirectionMode.FORWARD)
    goal = State(x=8.0, y=12.5, yaw=np.pi/2, direction=DirectionMode.FORWARD)
    
    print("Planning parking maneuver...")
    path = planner.plan_path(start, goal, max_iterations=5000)
    
    if path:
        print(f"✓ Parking path found with {len(path)} waypoints")
        planner.visualize_path(path, start, goal)
        return True
    else:
        print("✗ No parking path found!")
        return False


def demo_u_turn():
    """Demo U-turn maneuver"""
    print("\n=== Demo 3: U-Turn Maneuver ===")
    
    # Create vehicle model
    vehicle = VehicleModel(wheelbase=2.5, max_steer=np.pi/3)
    
    # Create planner for U-turn
    planner = HybridAStar(
        vehicle_model=vehicle,
        grid_resolution=0.4,
        angle_resolution=np.pi/10,
        velocity=2.0,
        simulation_time=0.7,
        dt=0.1
    )
    
    # Adjust weights for U-turn
    planner.w_steer = 6.0
    planner.w_turn = 10.0
    planner.w_cusp = 40.0  # Allow some direction changes
    planner.w_path = 4.0
    
    # Simple corridor for U-turn
    map_size = 40
    obstacle_map = np.zeros((map_size, map_size))
    
    # Create narrow corridor
    obstacle_map[:15, :] = 1        # Top obstacles
    obstacle_map[25:, :] = 1        # Bottom obstacles
    obstacle_map[:, :8] = 1         # Left wall
    obstacle_map[:, 32:] = 1        # Right wall
    
    planner.set_obstacle_map(obstacle_map, origin_x=-2, origin_y=-2)
    
    # Start going right, goal going left (U-turn)
    start = State(x=2.0, y=20.0, yaw=0, direction=DirectionMode.FORWARD)
    goal = State(x=2.0, y=20.0, yaw=np.pi, direction=DirectionMode.FORWARD)
    
    print("Planning U-turn...")
    path = planner.plan_path(start, goal, max_iterations=4000)
    
    if path:
        print(f"✓ U-turn path found with {len(path)} waypoints")
        planner.visualize_path(path, start, goal)
        return True
    else:
        print("✗ No U-turn path found!")
        return False


def analyze_path_quality(path):
    """Analyze the quality of a planned path"""
    if not path or len(path) < 2:
        return
        
    print("\n--- Path Quality Analysis ---")
    
    # Calculate total distance
    total_distance = 0
    for i in range(len(path) - 1):
        dx = path[i+1].x - path[i].x
        dy = path[i+1].y - path[i].y
        total_distance += np.sqrt(dx*dx + dy*dy)
    
    # Calculate steering statistics
    steering_angles = [abs(state.steer) for state in path]
    max_steer = max(steering_angles) if steering_angles else 0
    avg_steer = np.mean(steering_angles) if steering_angles else 0
    
    # Calculate curvature changes (smoothness)
    curvature_changes = 0
    for i in range(1, len(path) - 1):
        yaw_change1 = abs(path[i].yaw - path[i-1].yaw)
        yaw_change2 = abs(path[i+1].yaw - path[i].yaw)
        curvature_changes += abs(yaw_change2 - yaw_change1)
    
    # Direction changes
    direction_changes = 0
    for i in range(1, len(path)):
        if path[i].direction != path[i-1].direction:
            direction_changes += 1
    
    print(f"Total Distance: {total_distance:.2f} m")
    print(f"Path Points: {len(path)}")
    print(f"Max Steering: {np.degrees(max_steer):.1f}°")
    print(f"Avg Steering: {np.degrees(avg_steer):.1f}°")
    print(f"Direction Changes: {direction_changes}")
    print(f"Curvature Changes: {curvature_changes:.2f}")


def main():
    """Run all demos"""
    print("Hybrid A* Path Planning Demonstrations")
    print("=====================================")
    
    results = []
    
    # Run demos
    results.append(demo_basic_navigation())
    results.append(demo_parallel_parking())
    results.append(demo_u_turn())
    
    # Summary
    print(f"\n=== Summary ===")
    print(f"Successful demos: {sum(results)}/{len(results)}")
    
    if all(results):
        print("🎉 All demos completed successfully!")
    else:
        print("⚠️  Some demos failed - consider adjusting parameters")


if __name__ == "__main__":
    main()
