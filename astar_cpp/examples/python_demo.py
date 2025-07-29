"""
Python example demonstrating the C++ Hybrid A* implementation
"""

import numpy as np
import time
import matplotlib.pyplot as plt

try:
    import hybrid_astar_cpp as ha
except ImportError:
    print("Error: hybrid_astar_cpp module not found. Please build and install the C++ module first.")
    print("Run: ./build.sh")
    exit(1)


def create_test_environment():
    """Create a test environment with obstacles"""
    # Create 40x40 grid map
    obstacle_map = ha.create_obstacle_map(40, 40)
    
    # Add rectangular obstacles
    ha.add_rectangle_obstacle(obstacle_map, 15, 10, 25, 15)  # Central obstacle
    ha.add_rectangle_obstacle(obstacle_map, 8, 20, 12, 30)   # Side obstacle
    ha.add_rectangle_obstacle(obstacle_map, 25, 25, 35, 30)  # Corner obstacle
    
    return obstacle_map


def run_performance_comparison():
    """Compare different planner configurations"""
    print("=== Performance Comparison ===")
    
    vehicle = ha.VehicleModel(2.5, np.pi/4)
    obstacle_map = create_test_environment()
    
    configs = [
        {
            "name": "Coarse",
            "grid_res": 1.0,
            "angle_res": np.pi/6,
            "steer_res": np.pi/8,
            "sim_time": 1.0,
            "dt": 0.2
        },
        {
            "name": "Medium", 
            "grid_res": 0.5,
            "angle_res": np.pi/8,
            "steer_res": np.pi/16,
            "sim_time": 0.8,
            "dt": 0.1
        },
        {
            "name": "Fine",
            "grid_res": 0.25,
            "angle_res": np.pi/12,
            "steer_res": np.pi/24,
            "sim_time": 0.5,
            "dt": 0.05
        }
    ]
    
    start = ha.State(-8.0, -8.0, np.pi/4, ha.DirectionMode.NONE, 0.0)
    goal = ha.State(15.0, 15.0, np.pi/2, ha.DirectionMode.FORWARD, 0.0)

    print(f"{'Config':<10} {'Time (ms)':<12} {'Waypoints':<12} {'Distance':<12} {'Explored':<12}")
    print("-" * 60)
    
    for config in configs:
        planner = ha.HybridAStar(
            vehicle,
            config["grid_res"],
            config["angle_res"], 
            config["steer_res"],
            2.0,  # velocity
            config["sim_time"],
            config["dt"]
        )
        
        planner.set_obstacle_map(obstacle_map, -10.0, -10.0)
        
        start_time = time.time()
        path = planner.plan_path(start, goal, 3000)
        end_time = time.time()
        
        duration_ms = (end_time - start_time) * 1000
        stats = planner.get_statistics(path)
        
        if path:
            waypoints = int(stats["path_length_waypoints"])
            distance = stats["total_distance"]
            explored = int(stats["nodes_explored"])
            print(f"{config['name']:<10} {duration_ms:<12.1f} {waypoints:<12} {distance:<12.1f} {explored:<12}")
        else:
            explored = int(stats["nodes_explored"])
            print(f"{config['name']:<10} {duration_ms:<12.1f} {'N/A':<12} {'N/A':<12} {explored:<12}")


def run_detailed_example():
    """Run detailed example with path analysis"""
    print("\n=== Detailed Example ===")
    
    # Create vehicle and planner
    vehicle = ha.VehicleModel(wheelbase=2.5, max_steer=np.pi/4)
    planner = ha.HybridAStar(
        vehicle,
        grid_resolution=0.5,
        angle_resolution=np.pi/8,
        steer_resolution=np.pi/16,
        velocity=2.0,
        simulation_time=0.8,
        dt=0.1
    )
    
    # Set up environment
    obstacle_map = create_test_environment()
    planner.set_obstacle_map([], -10.0, -10.0)
    
    # Define start and goal
    start = ha.State(-5.0, -5.0, np.pi/6, ha.DirectionMode.NONE, 0.0)
    goal = ha.State(12.0, 12.0, np.pi*3/4, ha.DirectionMode.FORWARD, 0.0)
    
    print(f"Start: ({start.x:.1f}, {start.y:.1f}, {start.yaw:.2f} rad)")
    print(f"Goal:  ({goal.x:.1f}, {goal.y:.1f}, {goal.yaw:.2f} rad)")
    
    # Plan path
    print("\nPlanning path...")
    start_time = time.time()
    path = planner.plan_path(start, goal, 50000)
    end_time = time.time()
    
    duration = (end_time - start_time) * 1000
    print(f"Planning completed in {duration:.1f}ms")
    
    if path:
        print(f"✓ Path found with {len(path)} waypoints")
        
        # Extract detailed path
        detailed_path = planner.extract_detailed_path(path)
        print(f"Detailed path has {len(detailed_path)} states")
        
        # Analyze path
        stats = planner.get_statistics(path)
        print("\nPath Analysis:")
        print(f"  Total distance: {stats['total_distance']:.2f} m")
        print(f"  Average waypoint spacing: {stats['average_waypoint_spacing']:.2f} m")
        print(f"  Max steering angle: {stats['max_steering_angle']:.3f} rad ({np.degrees(stats['max_steering_angle']):.1f}°)")
        print(f"  Average steering angle: {stats['average_steering_angle']:.3f} rad")
        print(f"  Steering utilization: {stats['steering_utilization']:.1%}")
        print(f"  Direction changes: {int(stats['direction_changes'])}")
        print(f"  Max curvature: {stats['max_curvature']:.3f}")
        print(f"  Nodes explored: {int(stats['nodes_explored'])}")
        print(f"  Trajectories simulated: {int(stats['trajectories_simulated'])}")
        
        # Show key waypoints
        print(f"\nKey waypoints:")
        print(f"  Start: ({path[0].state.x:.1f}, {path[0].state.y:.1f}, {path[0].state.yaw:.2f})")
        if len(path) > 2:
            mid = len(path) // 2
            print(f"  Mid:   ({path[mid].state.x:.1f}, {path[mid].state.y:.1f}, {path[mid].state.yaw:.2f})")
        print(f"  End:   ({path[-1].state.x:.1f}, {path[-1].state.y:.1f}, {path[-1].state.yaw:.2f})")
        
        return path, detailed_path, obstacle_map
        
    else:
        print("✗ No path found")
        stats = planner.get_statistics(None)
        print(f"Nodes explored: {int(stats['nodes_explored'])}")
        return None, None, obstacle_map


def visualize_path(path, detailed_path, obstacle_map):
    """Create visualization of the planned path"""
    try:
        import matplotlib.pyplot as plt
        import matplotlib.patches as patches
    except ImportError:
        print("matplotlib not available, skipping visualization")
        return
    
    if not path:
        print("No path to visualize")
        return
    
    print("\n=== Creating Visualization ===")
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 7))
    
    # Plot 1: Waypoints and obstacles
    ax1.set_title("Path Waypoints")
    ax1.set_aspect('equal')
    
    # Draw obstacles
    for i, row in enumerate(obstacle_map):
        for j, cell in enumerate(row):
            if cell == 1:
                x = j * 0.5 - 10.0  # Convert grid to world coordinates
                y = i * 0.5 - 10.0
                rect = patches.Rectangle((x-0.25, y-0.25), 0.5, 0.5, 
                                       linewidth=0, facecolor='red', alpha=0.7)
                ax1.add_patch(rect)
    
    # Draw waypoint path
    waypoint_x = [node.state.x for node in path]
    waypoint_y = [node.state.y for node in path]
    ax1.plot(waypoint_x, waypoint_y, 'b-o', linewidth=2, markersize=4, label='Waypoints')
    
    # Mark start and goal
    ax1.plot(path[0].state.x, path[0].state.y, 'go', markersize=10, label='Start')
    ax1.plot(path[-1].state.x, path[-1].state.y, 'ro', markersize=10, label='Goal')
    
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    
    # Plot 2: Detailed trajectory
    ax2.set_title("Detailed Trajectory")
    ax2.set_aspect('equal')
    
    # Draw obstacles (same as above)
    for i, row in enumerate(obstacle_map):
        for j, cell in enumerate(row):
            if cell == 1:
                x = j * 0.5 - 10.0
                y = i * 0.5 - 10.0
                rect = patches.Rectangle((x-0.25, y-0.25), 0.5, 0.5, 
                                       linewidth=0, facecolor='red', alpha=0.7)
                ax2.add_patch(rect)
    
    # Draw detailed path
    if detailed_path:
        detailed_x = [state.x for state in detailed_path]
        detailed_y = [state.y for state in detailed_path]
        ax2.plot(detailed_x, detailed_y, 'g-', linewidth=1, alpha=0.8, label='Detailed trajectory')
        
        # Draw vehicle orientation at selected points
        step = max(1, len(detailed_path) // 20)  # Show ~20 orientation arrows
        for i in range(0, len(detailed_path), step):
            state = detailed_path[i]
            dx = 0.5 * np.cos(state.yaw)
            dy = 0.5 * np.sin(state.yaw)
            ax2.arrow(state.x, state.y, dx, dy, 
                     head_width=0.2, head_length=0.2, fc='blue', ec='blue', alpha=0.6)
    
    # Mark start and goal
    ax2.plot(path[0].state.x, path[0].state.y, 'go', markersize=10, label='Start')
    ax2.plot(path[-1].state.x, path[-1].state.y, 'ro', markersize=10, label='Goal')
    
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    ax2.set_xlabel('X (m)')
    ax2.set_ylabel('Y (m)')
    
    plt.tight_layout()
    
    # Save the plot
    plt.savefig('hybrid_astar_path.png', dpi=150, bbox_inches='tight')
    print("Visualization saved as 'hybrid_astar_path.png'")
    
    plt.show()


def demonstrate_vehicle_model():
    """Demonstrate vehicle model functionality"""
    print("\n=== Vehicle Model Demo ===")
    
    vehicle = ha.VehicleModel(wheelbase=2.5, max_steer=np.pi/4)
    print(f"Vehicle: wheelbase={vehicle.wheelbase}m, max_steer={np.degrees(vehicle.max_steer):.1f}°")
    
    # Test angle normalization
    test_angles = [0, np.pi/2, np.pi, 3*np.pi/2, 2*np.pi, -np.pi, -3*np.pi]
    print("\nAngle normalization tests:")
    for angle in test_angles:
        normalized = ha.VehicleModel.normalize_angle(angle)
        print(f"  {np.degrees(angle):6.1f}° -> {np.degrees(normalized):6.1f}°")
    
    # Test motion simulation
    print("\nMotion simulation tests:")
    initial_state = ha.State(0.0, 0.0, 0.0, ha.DirectionMode.FORWARD, 0.0)
    
    # Straight motion
    straight_states = vehicle.simulate_motion(initial_state, 2.0, 0.0, 0.1, 10)
    print(f"  Straight motion (10 steps): final position ({straight_states[-1].x:.2f}, {straight_states[-1].y:.2f})")
    
    # Left turn
    left_states = vehicle.simulate_motion(initial_state, 2.0, np.pi/4, 0.1, 10)
    print(f"  Left turn motion (10 steps): final position ({left_states[-1].x:.2f}, {left_states[-1].y:.2f})")
    
    # Backward motion
    backward_state = ha.State(0.0, 0.0, 0.0, ha.DirectionMode.BACKWARD, 0.0)
    backward_states = vehicle.simulate_motion(backward_state, 2.0, 0.0, 0.1, 10)
    print(f"  Backward motion (10 steps): final position ({backward_states[-1].x:.2f}, {backward_states[-1].y:.2f})")


def main():
    """Main demonstration function"""
    print("Hybrid A* C++ Python Bindings Demo")
    print("===================================")
    
    # Check that the module is available
    print(f"Module version: {ha.__version__}")
    print(f"Available constants: M_PI = {ha.M_PI}")
    
    # Demonstrate different aspects
    demonstrate_vehicle_model()
    run_performance_comparison()
    path, detailed_path, obstacle_map = run_detailed_example()
    
    # Visualize if matplotlib is available and path was found
    if path:
        visualize_path(path, detailed_path, obstacle_map)
    
    print("\n✓ Demo completed successfully!")


if __name__ == "__main__":
    main()
