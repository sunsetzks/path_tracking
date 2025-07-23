#!/usr/bin/env python3
"""
Enhanced demo for interactive Hybrid A* visualization.
This script demonstrates the full capabilities of the interactive visualization system.
"""

import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), 'astar_project'))

import numpy as np
import matplotlib.pyplot as plt
from astar_project.visualizer import HybridAStarVisualizer

# Mock classes for testing (same as before but with more realistic data)
class MockState:
    def __init__(self, x, y, yaw, steer=0.0, direction=1):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.steer = steer
        self.direction = direction

class MockNode:
    def __init__(self, state, f_cost=0.0, parent=None):
        self.state = state
        self.f_cost = f_cost
        self.parent = parent
        # Add mock trajectory states for more realistic visualization
        self.trajectory_states = []

class MockVehicleModel:
    def __init__(self):
        self.max_steer = 0.785  # 45 degrees in radians

def create_complex_path():
    """Create a more complex path with parking scenario"""
    path = []
    
    # Straight section
    for i in range(15):
        x = i * 0.8
        y = 0
        yaw = 0
        steer = 0
        path.append(MockState(x, y, yaw, steer))
    
    # Turning section (parking maneuver)
    for i in range(10):
        t = i / 9.0  # Normalized parameter
        x = 12 + 3 * t
        y = 3 * np.sin(t * np.pi)
        yaw = np.pi * t * 0.5
        steer = 0.5 * np.sin(t * np.pi * 2)  # Variable steering
        path.append(MockState(x, y, yaw, steer))
    
    # Reverse parking
    for i in range(8):
        t = i / 7.0
        x = 15 - 2 * t
        y = 3 + 2 * t
        yaw = np.pi * 0.5 + t * np.pi * 0.3
        steer = -0.3 * np.sin(t * np.pi)
        path.append(MockState(x, y, yaw, steer, direction=-1))  # Reverse
    
    return path

def create_realistic_exploration():
    """Create more realistic exploration data"""
    explored_nodes = []
    
    # Create a search tree that looks more realistic
    for layer in range(5):  # Different search layers
        num_nodes = 20 - layer * 3  # Fewer nodes in deeper layers
        for i in range(num_nodes):
            # Nodes tend to be closer to the path
            x = np.random.uniform(0, 18) + np.random.normal(0, 2)
            y = np.random.uniform(-2, 6) + np.random.normal(0, 1)
            yaw = np.random.uniform(-np.pi, np.pi)
            
            # Cost increases with distance from goal
            goal_x, goal_y = 13, 5
            dist_to_goal = np.sqrt((x - goal_x)**2 + (y - goal_y)**2)
            f_cost = 20 + dist_to_goal * 10 + np.random.uniform(0, 20)
            
            node = MockNode(MockState(x, y, yaw), f_cost)
            
            # Add some trajectory states for better visualization
            if np.random.random() > 0.5:  # 50% chance of having trajectory
                traj_states = []
                for j in range(3):
                    traj_x = x + j * 0.3 * np.cos(yaw)
                    traj_y = y + j * 0.3 * np.sin(yaw)
                    traj_yaw = yaw + np.random.normal(0, 0.1)
                    traj_states.append(MockState(traj_x, traj_y, traj_yaw))
                node.trajectory_states = traj_states
            
            explored_nodes.append(node)
    
    return explored_nodes

def create_complex_obstacle_map():
    """Create a more complex obstacle map"""
    obstacle_map = np.zeros((30, 25))
    
    # Building walls
    obstacle_map[5:25, 0:2] = 1    # Left wall
    obstacle_map[5:25, 23:25] = 1  # Right wall
    obstacle_map[23:25, 0:25] = 1  # Bottom wall
    
    # Parking spaces
    obstacle_map[15:20, 5:7] = 1   # Parking barrier 1
    obstacle_map[15:20, 15:17] = 1 # Parking barrier 2
    
    # Central obstacles
    obstacle_map[10:12, 10:15] = 1  # Central obstacle
    
    # Add some random obstacles
    for _ in range(10):
        x = np.random.randint(0, 25)
        y = np.random.randint(0, 30)
        size = np.random.randint(1, 3)
        obstacle_map[y:y+size, x:x+size] = 1
    
    return obstacle_map

def demo_interactive_visualization():
    """Comprehensive demo of interactive visualization features"""
    
    print("=" * 60)
    print("INTERACTIVE HYBRID A* VISUALIZATION DEMO")
    print("=" * 60)
    print()
    print("This demo showcases the interactive visualization features:")
    print("• Real-time element toggling with checkboxes")
    print("• Separate control for different visualization components")
    print("• Dynamic cost panel display")
    print("• Enhanced exploration tree visualization")
    print()
    print("INSTRUCTIONS:")
    print("1. A plot will open with control checkboxes on the right")
    print("2. Click checkboxes to toggle different elements:")
    print("   - Exploration Nodes: Search tree nodes")
    print("   - Trajectories: Forward simulation paths")
    print("   - Final Path: Planned route")
    print("   - Vehicle Arrows: Blue heading arrows")
    print("   - Steering Arrows: Red steering indicators")
    print("   - Waypoint Numbers: Path point labels")
    print("   - Cost Panel: Analysis charts")
    print("3. Try different combinations to analyze the solution")
    print()
    
    # Create realistic test data
    path = create_complex_path()
    start = MockState(0, 0, 0)
    goal = MockState(13, 5, np.pi/2)
    
    explored_nodes = create_realistic_exploration()
    
    # Create more realistic simulation trajectories
    simulation_trajectories = []
    for i in range(50):
        states = []
        start_x = np.random.uniform(0, 15)
        start_y = np.random.uniform(-1, 4)
        start_yaw = np.random.uniform(-np.pi, np.pi)
        
        for j in range(np.random.randint(3, 8)):
            x = start_x + j * 0.5 * np.cos(start_yaw) + np.random.normal(0, 0.1)
            y = start_y + j * 0.5 * np.sin(start_yaw) + np.random.normal(0, 0.1)
            yaw = start_yaw + np.random.normal(0, 0.1)
            states.append(MockState(x, y, yaw))
        
        direction = 1 if np.random.random() > 0.3 else -1  # 70% forward, 30% backward
        simulation_trajectories.append({
            'states': states,
            'direction': direction
        })
    
    # Create complex obstacle map
    obstacle_map = create_complex_obstacle_map()
    
    # Create vehicle model
    vehicle_model = MockVehicleModel()
    
    # Create and use visualizer
    visualizer = HybridAStarVisualizer()
    
    print("Opening interactive visualization...")
    print("Close the plot window when you're done exploring!")
    
    try:
        visualizer.visualize_path(
            path=path,
            start=start,
            goal=goal,
            explored_nodes=explored_nodes,
            simulation_trajectories=simulation_trajectories,
            obstacle_map=obstacle_map,
            map_origin_x=0,
            map_origin_y=-5,
            grid_resolution=0.8,
            vehicle_model=vehicle_model,
            show_exploration=True,
            show_trajectories=True,
            show_costs=False  # Start with cost panel hidden
        )
    except KeyboardInterrupt:
        print("\nDemo interrupted by user.")
    except Exception as e:
        print(f"\nDemo encountered an error: {e}")
    
    print("\nDemo completed!")
    print("The interactive visualization allows you to:")
    print("• Analyze search efficiency by toggling exploration nodes")
    print("• Understand motion primitives through trajectory display")
    print("• Examine path quality with steering and cost analysis")
    print("• Focus on specific aspects by hiding irrelevant elements")

if __name__ == "__main__":
    demo_interactive_visualization()
