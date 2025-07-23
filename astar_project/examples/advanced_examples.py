"""
Advanced Hybrid A* Examples
Complex scenarios demonstrating the capabilities of the Hybrid A* algorithm
"""

import numpy as np
import matplotlib.pyplot as plt
from astar_project.hybrid_astar import HybridAStar, VehicleModel, State, DirectionMode


class ScenarioGenerator:
    """Generate complex scenarios for testing"""
    
    @staticmethod
    def create_maze_scenario():
        """Create a maze-like environment"""
        map_size = 60
        obstacle_map = np.zeros((map_size, map_size))
        
        # Outer walls
        obstacle_map[0:3, :] = 1
        obstacle_map[-3:, :] = 1
        obstacle_map[:, 0:3] = 1
        obstacle_map[:, -3:] = 1
        
        # Internal maze structure
        # Vertical walls
        obstacle_map[10:50, 15:18] = 1
        obstacle_map[10:30, 35:38] = 1
        obstacle_map[30:50, 42:45] = 1
        
        # Horizontal walls
        obstacle_map[20:23, 5:40] = 1
        obstacle_map[35:38, 20:55] = 1
        obstacle_map[45:48, 10:30] = 1
        
        # Some scattered obstacles
        obstacle_map[25:30, 25:30] = 1
        obstacle_map[40:45, 8:13] = 1
        
        return obstacle_map
    
    @staticmethod
    def create_parking_garage():
        """Create a multi-level parking garage scenario"""
        map_size = 80
        obstacle_map = np.zeros((map_size, map_size))
        
        # Outer structure
        obstacle_map[0:5, :] = 1
        obstacle_map[-5:, :] = 1
        obstacle_map[:, 0:5] = 1
        obstacle_map[:, -5:] = 1
        
        # Parking spaces (multiple rows)
        for row in range(15, 65, 20):
            for col in range(10, 70, 12):
                # Create parking spaces
                obstacle_map[row:row+8, col:col+10] = 1
                # Leave entrance
                obstacle_map[row+8:row+10, col+3:col+7] = 0
        
        # Driving lanes (clear spaces between parking)
        for row in range(23, 65, 20):
            obstacle_map[row:row+5, 10:70] = 0
        
        # Central driving corridor
        obstacle_map[35:45, 35:45] = 0
        
        return obstacle_map
    
    @staticmethod
    def create_highway_scenario():
        """Create highway merging scenario"""
        map_size = 100
        obstacle_map = np.zeros((map_size, map_size))
        
        # Highway barriers
        obstacle_map[0:10, :] = 1
        obstacle_map[35:45, :60] = 1  # Median barrier
        obstacle_map[55:65, :60] = 1
        obstacle_map[90:, :] = 1
        
        # On-ramp barriers
        obstacle_map[45:55, 60:] = 1
        
        # Traffic islands
        obstacle_map[20:25, 75:85] = 1
        obstacle_map[70:75, 25:35] = 1
        
        return obstacle_map


def demo_complex_navigation():
    """Demo navigation through complex maze"""
    print("=== Advanced Demo 1: Complex Maze Navigation ===")
    
    # Create high-performance vehicle model
    vehicle = VehicleModel(wheelbase=2.0, max_steer=np.pi/3)
    
    # Create planner optimized for complex environments
    planner = HybridAStar(
        vehicle_model=vehicle,
        grid_resolution=0.4,
        angle_resolution=np.pi/12,
        velocity=2.5,
        simulation_time=0.6,
        dt=0.08
    )
    
    # Tune for maze navigation
    planner.w_steer = 6.0
    planner.w_turn = 8.0
    planner.w_cusp = 60.0
    planner.w_path = 4.0
    
    # Create complex maze
    obstacle_map = ScenarioGenerator.create_maze_scenario()
    planner.set_obstacle_map(obstacle_map, origin_x=-5, origin_y=-5)
    
    # Navigate from one corner to opposite corner
    start = State(x=2.0, y=2.0, yaw=np.pi/4, direction=DirectionMode.FORWARD)
    goal = State(x=22.0, y=22.0, yaw=-np.pi/4, direction=DirectionMode.FORWARD)
    
    print("Planning path through complex maze...")
    path = planner.plan_path(start, goal, max_iterations=8000)
    
    if path:
        print(f"✓ Complex path found with {len(path)} waypoints")
        planner.visualize_path(path, start, goal)
        analyze_advanced_path(path)
        return True
    else:
        print("✗ No path found through maze!")
        return False


def demo_precision_parking():
    """Demo precision parking in tight space"""
    print("\n=== Advanced Demo 2: Precision Parking ===")
    
    # Create precision vehicle model
    vehicle = VehicleModel(wheelbase=2.5, max_steer=np.pi/3.5)
    
    # Create planner for precision maneuvers
    planner = HybridAStar(
        vehicle_model=vehicle,
        grid_resolution=0.25,
        angle_resolution=np.pi/16,
        velocity=1.0,  # Very slow for precision
        simulation_time=0.8,
        dt=0.05
    )
    
    # Tune for precision parking
    planner.w_steer = 3.0      # Low steering penalty
    planner.w_turn = 5.0       # Low turn penalty
    planner.w_cusp = 25.0      # Allow backing up
    planner.w_path = 1.0       # Less emphasis on smoothness
    
    # Create parking garage
    obstacle_map = ScenarioGenerator.create_parking_garage()
    planner.set_obstacle_map(obstacle_map, origin_x=-10, origin_y=-10)
    
    # Park in tight space
    start = State(x=15.0, y=30.0, yaw=0, direction=DirectionMode.FORWARD)
    goal = State(x=8.0, y=18.0, yaw=np.pi/2, direction=DirectionMode.FORWARD)
    
    print("Planning precision parking maneuver...")
    path = planner.plan_path(start, goal, max_iterations=6000)
    
    if path:
        print(f"✓ Precision parking path found with {len(path)} waypoints")
        planner.visualize_path(path, start, goal)
        analyze_parking_maneuver(path)
        return True
    else:
        print("✗ No parking path found!")
        return False


def demo_highway_merging():
    """Demo highway merging scenario"""
    print("\n=== Advanced Demo 3: Highway Merging ===")
    
    # Create highway vehicle model
    vehicle = VehicleModel(wheelbase=3.0, max_steer=np.pi/6)  # Limited steering for highway
    
    # Create planner for highway scenarios
    planner = HybridAStar(
        vehicle_model=vehicle,
        grid_resolution=1.0,
        angle_resolution=np.pi/16,
        velocity=15.0,  # Highway speed
        simulation_time=0.4,
        dt=0.05
    )
    
    # Tune for highway driving (smooth, minimal steering)
    planner.w_steer = 20.0     # High steering penalty
    planner.w_turn = 25.0      # High turn penalty
    planner.w_cusp = 200.0     # No backing up on highway
    planner.w_path = 15.0      # High smoothness emphasis
    
    # Create highway scenario
    obstacle_map = ScenarioGenerator.create_highway_scenario()
    planner.set_obstacle_map(obstacle_map, origin_x=-10, origin_y=-10)
    
    # Merge from on-ramp to highway
    start = State(x=35.0, y=50.0, yaw=-np.pi/6, direction=DirectionMode.FORWARD)
    goal = State(x=70.0, y=25.0, yaw=0, direction=DirectionMode.FORWARD)
    
    print("Planning highway merging maneuver...")
    path = planner.plan_path(start, goal, max_iterations=4000)
    
    if path:
        print(f"✓ Highway merging path found with {len(path)} waypoints")
        planner.visualize_path(path, start, goal)
        analyze_highway_path(path)
        return True
    else:
        print("✗ No highway merging path found!")
        return False


def analyze_advanced_path(path):
    """Analyze advanced path characteristics"""
    if not path or len(path) < 2:
        return
        
    print("\n--- Advanced Path Analysis ---")
    
    # Calculate path metrics
    total_distance = 0
    total_rotation = 0
    max_curvature = 0
    steering_changes = 0
    
    for i in range(len(path) - 1):
        # Distance
        dx = path[i+1].x - path[i].x
        dy = path[i+1].y - path[i].y
        total_distance += np.sqrt(dx*dx + dy*dy)
        
        # Rotation
        dyaw = abs(path[i+1].yaw - path[i].yaw)
        if dyaw > np.pi:
            dyaw = 2*np.pi - dyaw
        total_rotation += dyaw
        
        # Steering changes
        if i > 0:
            steer_diff = abs(path[i+1].steer - path[i].steer)
            if steer_diff > 0.1:  # Significant steering change
                steering_changes += 1
    
    # Calculate average curvature
    if len(path) > 2:
        curvatures = []
        for i in range(1, len(path) - 1):
            # Approximate curvature using three points
            p1 = np.array([path[i-1].x, path[i-1].y])
            p2 = np.array([path[i].x, path[i].y])
            p3 = np.array([path[i+1].x, path[i+1].y])
            
            # Calculate curvature approximation
            v1 = p2 - p1
            v2 = p3 - p2
            
            if np.linalg.norm(v1) > 1e-6 and np.linalg.norm(v2) > 1e-6:
                cross_product = np.cross(v1, v2)
                curvature = abs(cross_product) / (np.linalg.norm(v1) * np.linalg.norm(v2))
                curvatures.append(curvature)
        
        if curvatures:
            max_curvature = max(curvatures)
    
    # Direction changes
    direction_changes = sum(1 for i in range(1, len(path)) 
                           if path[i].direction != path[i-1].direction)
    
    print(f"Path Length: {total_distance:.2f} m")
    print(f"Total Rotation: {np.degrees(total_rotation):.1f}°")
    print(f"Max Curvature: {max_curvature:.3f}")
    print(f"Steering Changes: {steering_changes}")
    print(f"Direction Changes: {direction_changes}")
    print(f"Path Efficiency: {(np.sqrt((path[-1].x - path[0].x)**2 + (path[-1].y - path[0].y)**2) / total_distance):.2f}")


def analyze_parking_maneuver(path):
    """Analyze parking maneuver characteristics"""
    print("\n--- Parking Maneuver Analysis ---")
    
    if not path:
        return
    
    # Count reversing segments
    reverse_segments = 0
    in_reverse = False
    
    for state in path:
        if state.direction == DirectionMode.BACKWARD:
            if not in_reverse:
                reverse_segments += 1
                in_reverse = True
        else:
            in_reverse = False
    
    # Calculate minimum turning radius used
    min_radius = float('inf')
    for state in path:
        if abs(state.steer) > 1e-6:
            radius = 2.5 / abs(np.tan(state.steer))  # Assuming wheelbase = 2.5
            min_radius = min(min_radius, radius)
    
    # Calculate space utilization
    x_range = max(s.x for s in path) - min(s.x for s in path)
    y_range = max(s.y for s in path) - min(s.y for s in path)
    
    print(f"Reversing Segments: {reverse_segments}")
    print(f"Min Turning Radius: {min_radius:.2f} m" if min_radius != float('inf') else "No turning")
    print(f"Maneuver Space: {x_range:.2f}m × {y_range:.2f}m")


def analyze_highway_path(path):
    """Analyze highway path characteristics"""
    print("\n--- Highway Path Analysis ---")
    
    if not path:
        return
    
    # Calculate smoothness metrics
    steering_variance = np.var([abs(s.steer) for s in path])
    yaw_changes = [abs(path[i+1].yaw - path[i].yaw) for i in range(len(path)-1)]
    max_yaw_change = max(yaw_changes) if yaw_changes else 0
    
    # Calculate average speed potential (based on curvature)
    avg_curvature = np.mean([abs(s.steer) for s in path])
    
    print(f"Steering Variance: {steering_variance:.4f}")
    print(f"Max Yaw Change: {np.degrees(max_yaw_change):.2f}°")
    print(f"Avg Steering: {np.degrees(avg_curvature):.2f}°")
    print(f"Path Smoothness Score: {1.0 / (1.0 + steering_variance * 100):.3f}")


def benchmark_performance():
    """Benchmark algorithm performance across scenarios"""
    print("\n=== Performance Benchmark ===")
    
    scenarios = [
        ("Simple", lambda: np.zeros((20, 20))),
        ("Medium", lambda: ScenarioGenerator.create_maze_scenario()[:40, :40]),
        ("Complex", ScenarioGenerator.create_maze_scenario),
    ]
    
    results = []
    
    for name, map_generator in scenarios:
        print(f"\nTesting {name} scenario...")
        
        vehicle = VehicleModel()
        planner = HybridAStar(vehicle_model=vehicle, grid_resolution=0.5)
        obstacle_map = map_generator()
        planner.set_obstacle_map(obstacle_map)
        
        start = State(x=2, y=2, yaw=0)
        goal = State(x=obstacle_map.shape[1]*0.5-5, y=obstacle_map.shape[0]*0.5-5, yaw=np.pi/2)
        
        import time
        start_time = time.time()
        path = planner.plan_path(start, goal, max_iterations=2000)
        end_time = time.time()
        
        result = {
            'scenario': name,
            'success': path is not None,
            'time': end_time - start_time,
            'path_length': len(path) if path else 0,
            'map_size': obstacle_map.shape[0] * obstacle_map.shape[1]
        }
        results.append(result)
        
        print(f"  Success: {result['success']}")
        print(f"  Time: {result['time']:.2f}s")
        print(f"  Path points: {result['path_length']}")
    
    # Summary
    print(f"\n--- Benchmark Summary ---")
    for result in results:
        success_rate = "✓" if result['success'] else "✗"
        print(f"{result['scenario']:10s}: {success_rate} {result['time']:6.2f}s {result['path_length']:4d} pts")


def main():
    """Run advanced demonstrations"""
    print("Advanced Hybrid A* Demonstrations")
    print("==================================")
    
    results = []
    
    # Run advanced demos
    results.append(demo_complex_navigation())
    results.append(demo_precision_parking())
    results.append(demo_highway_merging())
    
    # Run benchmark
    benchmark_performance()
    
    # Summary
    print(f"\n=== Final Summary ===")
    print(f"Successful advanced demos: {sum(results)}/{len(results)}")
    
    if all(results):
        print("🏆 All advanced demos completed successfully!")
        print("The Hybrid A* algorithm demonstrates robust performance")
        print("across diverse and challenging scenarios.")
    else:
        print("⚠️  Some advanced demos failed")
        print("Consider parameter tuning or increasing iteration limits")


if __name__ == "__main__":
    main()
