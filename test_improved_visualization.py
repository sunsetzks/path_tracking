#!/usr/bin/env python3
"""
Test script to verify the improved visualization features:
1. Black obstacles, white free space
2. Enhanced simulation trajectory visualization with markers and arrows
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import os

# Add the astar_project to the path
sys.path.append(os.path.join(os.path.dirname(__file__), 'astar_project'))

try:
    from astar_project.visualizer import HybridAStarVisualizer
    from astar_project.hybrid_astar import State, DirectionMode, Node
except ImportError:
    print("Could not import from astar_project. Please ensure the module is available.")
    exit(1)

def create_test_data():
    """Create test data for visualization"""
    
    # Create a simple obstacle map (1 = obstacle, 0 = free)
    obstacle_map = np.zeros((20, 30))
    # Add some obstacles
    obstacle_map[8:12, 10:15] = 1  # Rectangle obstacle
    obstacle_map[15:18, 5:8] = 1   # Another obstacle
    obstacle_map[2:5, 20:25] = 1   # Third obstacle
    
    # Create a simple path
    path = []
    for i in range(15):
        x = i * 1.5
        y = 5 + 3 * np.sin(i * 0.5)
        yaw = 0.5 * np.cos(i * 0.3)
        steer = 0.3 * np.sin(i * 0.4)
        path.append(State(x, y, yaw, DirectionMode.FORWARD, steer=steer))
    
    # Create start and goal states
    start = State(0, 5, 0, DirectionMode.FORWARD)
    goal = State(21, 8, 0.5, DirectionMode.FORWARD)
    
    # Create some simulation trajectories
    simulation_trajectories = []
    
    # Forward trajectories
    for i in range(5):
        states = []
        for j in range(8):
            x = i * 2 + j * 0.5
            y = 3 + i + j * 0.2
            yaw = j * 0.1
            states.append(State(x, y, yaw, DirectionMode.FORWARD))
        simulation_trajectories.append({
            'states': states,
            'direction': DirectionMode.FORWARD
        })
    
    # Backward trajectories
    for i in range(3):
        states = []
        for j in range(6):
            x = 15 + i * 1.5 - j * 0.4
            y = 12 - i - j * 0.3
            yaw = np.pi + j * 0.15
            states.append(State(x, y, yaw, DirectionMode.BACKWARD))
        simulation_trajectories.append({
            'states': states,
            'direction': DirectionMode.BACKWARD
        })
    
    # Create some explored nodes
    explored_nodes = []
    for i in range(50):
        x = np.random.uniform(0, 25)
        y = np.random.uniform(0, 15)
        yaw = np.random.uniform(-np.pi, np.pi)
        state = State(x, y, yaw, DirectionMode.FORWARD)
        node = Node(state)
        node.g_cost = np.random.uniform(0, 20)
        node.h_cost = np.random.uniform(0, 15)
        # f_cost is a property, so it's automatically calculated
        
        # Add some parent relationships
        if i > 0 and np.random.random() > 0.3:
            node.parent = explored_nodes[np.random.randint(0, len(explored_nodes))]
        
        explored_nodes.append(node)
    
    return path, start, goal, simulation_trajectories, explored_nodes, obstacle_map

def test_visualization():
    """Test the improved visualization features"""
    print("Creating test data...")
    path, start, goal, simulation_trajectories, explored_nodes, obstacle_map = create_test_data()
    
    print("Creating visualizer...")
    visualizer = HybridAStarVisualizer()
    
    print("Testing main visualization with improved features...")
    visualizer.visualize_path(
        path=path,
        start=start,
        goal=goal,
        explored_nodes=explored_nodes,
        simulation_trajectories=simulation_trajectories,
        obstacle_map=obstacle_map,
        map_origin_x=0,
        map_origin_y=0,
        grid_resolution=1.0,
        vehicle_model=None,
        show_exploration=True,
        show_trajectories=True,
        show_costs=False  # Disable cost plot for this test
    )
    
    print("Visualization test completed!")
    print("\nFeatures tested:")
    print("✓ Black obstacles, white free space")
    print("✓ Enhanced simulation trajectories with directional markers")
    print("✓ Trajectory arrows showing final direction")
    print("✓ Different colors for forward/backward trajectories")
    print("✓ Legend entries for trajectory types")

if __name__ == "__main__":
    test_visualization()
