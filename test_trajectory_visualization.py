#!/usr/bin/env python3
"""
测试改进的可视化功能 - 使用前向仿真轨迹连接节点
Test improved visualization with forward simulation trajectory connections
"""

import sys
import os

# Add the project directory to path
sys.path.append('/home/zks/ws/path_tracking/astar_project')

import numpy as np
from astar_project.hybrid_astar import HybridAStar, State, VehicleModel
from astar_project.visualizer import HybridAStarVisualizer

def create_simple_test_case():
    """创建一个简单的测试案例"""
    # Vehicle model
    vehicle = VehicleModel(
        wheelbase=2.5,
        max_steer=0.6
    )
    
    # Create obstacle map (free space with minimal obstacles)
    map_width, map_height = 30, 20
    obstacle_map = np.zeros((map_height, map_width))
    
    # Add some simple obstacles
    obstacle_map[8:12, 12:16] = 1  # Rectangle obstacle
    
    # Start and goal states (closer and easier)
    start = State(x=3.0, y=3.0, yaw=0.0)
    goal = State(x=25.0, y=15.0, yaw=0.0)
    
    return vehicle, obstacle_map, start, goal

def test_improved_visualization():
    """测试改进的可视化功能"""
    print("测试改进的Hybrid A*可视化功能...")
    print("Testing improved Hybrid A* visualization...")
    
    # Create test case
    vehicle, obstacle_map, start, goal = create_simple_test_case()
    
    # Initialize planner
    planner = HybridAStar(
        vehicle_model=vehicle,
        grid_resolution=1.0,
        angle_resolution=np.pi/4
    )
    
    # Set obstacle map
    planner.set_obstacle_map(obstacle_map, origin_x=0, origin_y=0)
    
    # Plan path
    print("开始路径规划...")
    print("Starting path planning...")
    
    path = planner.plan_path(start, goal, max_iterations=2000)
    
    if path:
        print(f"路径规划成功！找到 {len(path)} 个路径点")
        print(f"Path planning successful! Found {len(path)} waypoints")
        
        # Create visualizer
        visualizer = HybridAStarVisualizer()
        
        # Test visualization with forward simulation trajectories
        print("可视化路径规划结果（包含前向仿真轨迹）...")
        print("Visualizing path planning results (with forward simulation trajectories)...")
        
        visualizer.visualize_path(
            path=path,
            start=start,
            goal=goal,
            explored_nodes=planner.explored_nodes,
            simulation_trajectories=planner.simulation_trajectories,
            obstacle_map=obstacle_map,
            map_origin_x=0,
            map_origin_y=0,
            grid_resolution=0.5,
            vehicle_model=vehicle,
            show_exploration=True,
            show_trajectories=True,
            show_costs=True
        )
        
        print("可视化完成！请查看图形输出。")
        print("Visualization complete! Please check the graphical output.")
        
    else:
        print("路径规划失败！")
        print("Path planning failed!")

if __name__ == "__main__":
    test_improved_visualization()
