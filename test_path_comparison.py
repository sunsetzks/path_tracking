#!/usr/bin/env python3
"""
Comparison test: Old interpolation-based vs New simulation-based path visualization
"""

import numpy as np
import sys
import os
import matplotlib.pyplot as plt
from typing import List, Optional, Tuple
from matplotlib.figure import Figure
from matplotlib.axes import Axes

# Add the astar_project to the path
sys.path.append(os.path.join(os.path.dirname(__file__), 'astar_project'))

from astar_project.hybrid_astar import HybridAStar, VehicleModel, State, DirectionMode, Node


def compare_path_methods() -> None:
    """Compare old interpolation method vs new simulation trajectory method"""
    print("Comparing Path Reconstruction Methods")
    print("=" * 50)
    
    # Create vehicle model
    vehicle = VehicleModel(wheelbase=2.5, max_steer=np.pi/4)
    
    # Create hybrid A* planner
    planner = HybridAStar(
        vehicle_model=vehicle,
        grid_resolution=0.5,
        angle_resolution=np.pi/8,
        velocity=2.0,
        simulation_time=0.8,
        dt=0.1
    )
    
    # Simple scenario
    start = State(x=0.0, y=0.0, yaw=0.0, direction=DirectionMode.FORWARD)
    goal = State(x=8.0, y=5.0, yaw=np.pi/4, direction=DirectionMode.FORWARD)
    
    print("Planning path...")
    path_nodes = planner.plan_path(start, goal, max_iterations=2000)
    
    if path_nodes:
        print(f"Path found with {len(path_nodes)} nodes")
        
        # Method 1: Old way - just extract node states (simple interpolation)
        simple_path = [node.state for node in path_nodes]
        
        # Method 2: New way - use simulation trajectories
        detailed_path = planner.extract_detailed_path(path_nodes)
        
        print(f"\nComparison:")
        print(f"Simple path (node states only): {len(simple_path)} states")
        print(f"Detailed path (with simulations): {len(detailed_path)} states")
        print(f"Detail improvement factor: {len(detailed_path)/len(simple_path):.1f}x")
        
        # Calculate path distances
        def calculate_distance(path):
            total = 0
            for i in range(len(path)-1):
                dx = path[i+1].x - path[i].x
                dy = path[i+1].y - path[i].y
                total += np.sqrt(dx*dx + dy*dy)
            return total
        
        simple_distance = calculate_distance(simple_path)
        detailed_distance = calculate_distance(detailed_path)
        
        print(f"\nPath lengths:")
        print(f"Simple path distance: {simple_distance:.2f} m")
        print(f"Detailed path distance: {detailed_distance:.2f} m")
        
        # Visualize the difference
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
        
        # Plot 1: Simple path (old method)
        simple_x = [state.x for state in simple_path]
        simple_y = [state.y for state in simple_path]
        ax1.plot(simple_x, simple_y, 'bo-', linewidth=2, markersize=6, label='Node states')
        ax1.set_title(f'Old Method: Node States Only\n{len(simple_path)} points')
        ax1.grid(True, alpha=0.3)
        ax1.set_aspect('equal')
        ax1.legend()
        
        # Plot 2: Detailed path (new method)
        detailed_x = [state.x for state in detailed_path]
        detailed_y = [state.y for state in detailed_path]
        ax2.plot(detailed_x, detailed_y, 'r.-', linewidth=1, markersize=3, alpha=0.7, label='Simulation states')
        ax2.plot(simple_x, simple_y, 'bo', markersize=8, label='Node states', alpha=0.8)
        ax2.set_title(f'New Method: Simulation Trajectories\n{len(detailed_path)} points')
        ax2.grid(True, alpha=0.3)
        ax2.set_aspect('equal')
        ax2.legend()
        
        # Mark start and goal
        for ax in [ax1, ax2]:
            ax.plot(start.x, start.y, 'gs', markersize=10, label='Start')
            ax.plot(goal.x, goal.y, 'rs', markersize=10, label='Goal')
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
        
        plt.tight_layout()
        plt.suptitle('Path Reconstruction Methods Comparison', fontsize=14, y=0.98)
        plt.show()
        
        # Show trajectory breakdown
        print(f"\nTrajectory breakdown:")
        for i, node in enumerate(path_nodes[1:], 1):
            if node.trajectory_states:
                print(f"Node {i}: {len(node.trajectory_states)} trajectory states")
        
        print(f"\nKey advantages of new method:")
        print(f"1. Preserves vehicle dynamics from forward simulation")
        print(f"2. No interpolation artifacts")
        print(f"3. Maintains kinematic consistency")
        print(f"4. Shows actual planned motion sequence")
        print(f"5. Higher resolution path ({len(detailed_path)/len(simple_path):.1f}x more points)")
        
    else:
        print("No path found!")


if __name__ == "__main__":
    compare_path_methods()
