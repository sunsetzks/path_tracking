"""
Hybrid A* Path Planning Algorithm
Implementation with steering angle cost, path smoothness, turning cost, 
cusp cost, and forward simulation considering steering angle velocity.

Core algorithm without visualization dependencies.

Author: Your Name
Date: 2025-07-22
"""

import numpy as np
import heapq
import math
from typing import List, Tuple, Optional, Set
from dataclasses import dataclass, field
from enum import Enum


class DirectionMode(Enum):
    """Vehicle direction mode"""
    FORWARD = 1
    BACKWARD = -1


@dataclass
class State:
    """Vehicle state representation"""
    x: float
    y: float
    yaw: float  # heading angle in radians
    direction: DirectionMode = DirectionMode.FORWARD
    steer: float = 0.0  # steering angle in radians
    
    def __hash__(self):
        # For use in sets and dictionaries
        return hash((round(self.x, 2), round(self.y, 2), round(self.yaw, 2)))
    
    def __eq__(self, other):
        if not isinstance(other, State):
            return False
        return (abs(self.x - other.x) < 0.1 and 
                abs(self.y - other.y) < 0.1 and 
                abs(self.yaw - other.yaw) < 0.1)


@dataclass
class Node:
    """A* search node"""
    state: State
    g_cost: float = 0.0  # cost from start
    h_cost: float = 0.0  # heuristic cost to goal
    parent: Optional['Node'] = None
    steer_cost: float = 0.0  # steering angle cost
    turn_cost: float = 0.0   # turning cost
    cusp_cost: float = 0.0   # cusp (direction change) cost
    path_cost: float = 0.0   # path smoothness cost
    
    @property
    def f_cost(self) -> float:
        """Total cost"""
        return self.g_cost + self.h_cost
    
    def __lt__(self, other):
        return self.f_cost < other.f_cost


class VehicleModel:
    """Simple bicycle model for vehicle simulation"""
    
    def __init__(self, wheelbase: float = 2.5, max_steer: float = np.pi/4):
        """
        Args:
            wheelbase: Distance between front and rear axles (m)
            max_steer: Maximum steering angle (rad)
        """
        self.wheelbase = wheelbase
        self.max_steer = max_steer
    
    def simulate_motion(self, state: State, velocity: float, steer_rate: float, 
                       dt: float, steps: int = 1) -> List[State]:
        """
        Forward simulate vehicle motion with fixed linear velocity and steering rate
        
        Args:
            state: Initial state
            velocity: Linear velocity (m/s)
            steer_rate: Steering angle rate (rad/s)
            dt: Time step (s)
            steps: Number of simulation steps
            
        Returns:
            List of simulated states
        """
        states = []
        current_state = State(state.x, state.y, state.yaw, state.direction, state.steer)
        
        for _ in range(steps):
            # Update steering angle
            new_steer = current_state.steer + steer_rate * dt
            new_steer = np.clip(new_steer, -self.max_steer, self.max_steer)
            
            # Bicycle model kinematics
            if current_state.direction == DirectionMode.FORWARD:
                v = velocity
            else:
                v = -velocity
                
            # Update state using bicycle model
            new_x = current_state.x + v * np.cos(current_state.yaw) * dt
            new_y = current_state.y + v * np.sin(current_state.yaw) * dt
            new_yaw = current_state.yaw + v * np.tan(new_steer) / self.wheelbase * dt
            
            # Normalize yaw angle
            new_yaw = self.normalize_angle(new_yaw)
            
            current_state = State(new_x, new_y, new_yaw, current_state.direction, new_steer)
            states.append(current_state)
        
        return states
    
    @staticmethod
    def normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle


class HybridAStar:
    """Hybrid A* path planning algorithm"""
    
    def __init__(self, 
                 vehicle_model: VehicleModel,
                 grid_resolution: float = 1.0,
                 angle_resolution: float = np.pi/8,
                 steer_resolution: float = np.pi/16,
                 velocity: float = 2.0,
                 simulation_time: float = 1.0,
                 dt: float = 0.1):
        """
        Args:
            vehicle_model: Vehicle kinematic model
            grid_resolution: Grid resolution for discretization (m)
            angle_resolution: Angular resolution (rad)
            steer_resolution: Steering angle resolution (rad)
            velocity: Fixed linear velocity for simulation (m/s)
            simulation_time: Forward simulation time (s)
            dt: Simulation time step (s)
        """
        self.vehicle_model = vehicle_model
        self.grid_resolution = grid_resolution
        self.angle_resolution = angle_resolution
        self.steer_resolution = steer_resolution
        self.velocity = velocity
        self.simulation_time = simulation_time
        self.dt = dt
        self.simulation_steps = int(simulation_time / dt)
        
        # Cost weights
        self.w_steer = 10.0      # Steering angle cost weight
        self.w_turn = 15.0       # Turning cost weight
        self.w_cusp = 50.0       # Direction change cost weight
        self.w_path = 5.0        # Path smoothness cost weight
        self.w_obstacle = 1000.0 # Obstacle cost weight
        
        # Steering angle rates for motion primitives (rad/s)
        self.steer_rates = [-np.pi/2, -np.pi/4, 0, np.pi/4, np.pi/2]
        
        # Obstacle map
        self.obstacle_map = None
        self.map_width = 0
        self.map_height = 0
        self.map_origin_x = 0
        self.map_origin_y = 0
        
        # Data for visualization (stored but not processed here)
        self.explored_nodes = []  # Store explored nodes for visualization
        self.simulation_trajectories = []  # Store all simulation trajectories
    
    def set_obstacle_map(self, obstacle_map: np.ndarray, 
                        origin_x: float = 0, origin_y: float = 0):
        """Set obstacle map
        
        Args:
            obstacle_map: 2D numpy array where 1 indicates obstacle
            origin_x: Map origin x coordinate
            origin_y: Map origin y coordinate
        """
        self.obstacle_map = obstacle_map
        self.map_height, self.map_width = obstacle_map.shape
        self.map_origin_x = origin_x
        self.map_origin_y = origin_y
    
    def is_collision_free(self, state: State) -> bool:
        """Check if state is collision-free"""
        if self.obstacle_map is None:
            return True
            
        # Convert world coordinates to grid coordinates
        grid_x = int((state.x - self.map_origin_x) / self.grid_resolution)
        grid_y = int((state.y - self.map_origin_y) / self.grid_resolution)
        
        # Check bounds
        if (grid_x < 0 or grid_x >= self.map_width or 
            grid_y < 0 or grid_y >= self.map_height):
            return False
            
        # Check obstacle
        return self.obstacle_map[grid_y, grid_x] == 0
    
    def heuristic_cost(self, state: State, goal: State) -> float:
        """Calculate heuristic cost (Euclidean distance + angular difference)"""
        dx = goal.x - state.x
        dy = goal.y - state.y
        distance_cost = np.sqrt(dx*dx + dy*dy)
        
        # Angular difference cost
        angle_diff = abs(self.vehicle_model.normalize_angle(goal.yaw - state.yaw))
        angle_cost = angle_diff * 2.0
        
        return distance_cost + angle_cost
    
    def calculate_steering_cost(self, steer_angle: float) -> float:
        """Calculate cost based on steering angle magnitude"""
        return abs(steer_angle) / self.vehicle_model.max_steer
    
    def calculate_turning_cost(self, prev_yaw: float, current_yaw: float) -> float:
        """Calculate cost based on turning (yaw change)"""
        yaw_diff = abs(self.vehicle_model.normalize_angle(current_yaw - prev_yaw))
        return yaw_diff
    
    def calculate_cusp_cost(self, prev_direction: DirectionMode, 
                           current_direction: DirectionMode) -> float:
        """Calculate cost for direction changes (cusps)"""
        if prev_direction != current_direction:
            return 1.0
        return 0.0
    
    def calculate_path_smoothness_cost(self, states: List[State]) -> float:
        """Calculate path smoothness cost based on curvature changes"""
        if len(states) < 3:
            return 0.0
            
        smoothness_cost = 0.0
        for i in range(1, len(states) - 1):
            # Calculate curvature approximation
            p1 = np.array([states[i-1].x, states[i-1].y])
            p2 = np.array([states[i].x, states[i].y])
            p3 = np.array([states[i+1].x, states[i+1].y])
            
            # Calculate vectors
            v1 = p2 - p1
            v2 = p3 - p2
            
            # Avoid division by zero
            if np.linalg.norm(v1) < 1e-6 or np.linalg.norm(v2) < 1e-6:
                continue
                
            # Calculate angle between vectors (curvature approximation)
            cos_angle = np.dot(v1, v2) / (np.linalg.norm(v1) * np.linalg.norm(v2))
            cos_angle = np.clip(cos_angle, -1.0, 1.0)
            angle = np.arccos(cos_angle)
            
            smoothness_cost += angle
            
        return smoothness_cost
    
    def get_successors(self, node: Node) -> List[Node]:
        """Generate successor nodes using motion primitives"""
        successors = []
        
        for steer_rate in self.steer_rates:
            for direction in [DirectionMode.FORWARD, DirectionMode.BACKWARD]:
                # Create new state with current direction
                current_state = State(node.state.x, node.state.y, node.state.yaw, 
                                    direction, node.state.steer)
                
                # Simulate motion
                simulated_states = self.vehicle_model.simulate_motion(
                    current_state, self.velocity, steer_rate, 
                    self.dt, self.simulation_steps)
                
                if not simulated_states:
                    continue
                    
                final_state = simulated_states[-1]
                
                # Store simulation trajectory for visualization
                trajectory = {
                    'states': [current_state] + simulated_states,
                    'steer_rate': steer_rate,
                    'direction': direction,
                    'parent': node.state
                }
                self.simulation_trajectories.append(trajectory)
                
                # Check collision for all states in trajectory
                collision_free = True
                for state in simulated_states:
                    if not self.is_collision_free(state):
                        collision_free = False
                        break
                
                if not collision_free:
                    continue
                
                # Calculate costs
                motion_cost = self.velocity * self.simulation_time
                steer_cost = self.calculate_steering_cost(final_state.steer)
                
                if node.parent is not None:
                    turn_cost = self.calculate_turning_cost(node.parent.state.yaw, 
                                                           final_state.yaw)
                    cusp_cost = self.calculate_cusp_cost(node.parent.state.direction, 
                                                        direction)
                else:
                    turn_cost = 0.0
                    cusp_cost = 0.0
                
                # Path smoothness cost
                trajectory_states = [node.state] + simulated_states
                path_cost = self.calculate_path_smoothness_cost(trajectory_states)
                
                # Total g_cost
                total_g_cost = (node.g_cost + motion_cost + 
                              self.w_steer * steer_cost + 
                              self.w_turn * turn_cost + 
                              self.w_cusp * cusp_cost + 
                              self.w_path * path_cost)
                
                # Create successor node
                successor = Node(
                    state=final_state,
                    g_cost=total_g_cost,
                    parent=node,
                    steer_cost=steer_cost,
                    turn_cost=turn_cost,
                    cusp_cost=cusp_cost,
                    path_cost=path_cost
                )
                
                successors.append(successor)
        
        return successors
    
    def discretize_state(self, state: State) -> Tuple[int, int, int, int]:
        """Discretize continuous state for duplicate detection"""
        grid_x = int(state.x / self.grid_resolution)
        grid_y = int(state.y / self.grid_resolution)
        grid_yaw = int(state.yaw / self.angle_resolution)
        grid_steer = int(state.steer / self.steer_resolution)
        return (grid_x, grid_y, grid_yaw, grid_steer)
    
    def is_goal_reached(self, current: State, goal: State, 
                       position_tolerance: float = 1.0, 
                       angle_tolerance: float = np.pi/6) -> bool:
        """Check if goal is reached within tolerances"""
        position_error = np.sqrt((current.x - goal.x)**2 + (current.y - goal.y)**2)
        angle_error = abs(self.vehicle_model.normalize_angle(current.yaw - goal.yaw))
        
        return (position_error <= position_tolerance and 
                angle_error <= angle_tolerance)
    
    def reconstruct_path(self, goal_node: Node) -> List[State]:
        """Reconstruct path from goal node to start"""
        path = []
        current = goal_node
        
        while current is not None:
            path.append(current.state)
            current = current.parent
            
        return list(reversed(path))
    
    def plan_path(self, start: State, goal: State, 
                  max_iterations: int = 10000) -> Optional[List[State]]:
        """
        Plan path using Hybrid A* algorithm
        
        Args:
            start: Start state
            goal: Goal state
            max_iterations: Maximum search iterations
            
        Returns:
            List of states representing the path, or None if no path found
        """
        # Initialize visualization data
        self.explored_nodes = []
        self.simulation_trajectories = []
        
        # Initialize
        open_list = []
        closed_set: Set[Tuple[int, int, int, int]] = set()
        node_map = {}
        
        # Create start node
        start_node = Node(state=start, g_cost=0.0)
        start_node.h_cost = self.heuristic_cost(start, goal)
        
        heapq.heappush(open_list, start_node)
        start_key = self.discretize_state(start)
        node_map[start_key] = start_node
        
        iterations = 0
        
        while open_list and iterations < max_iterations:
            iterations += 1
            
            # Get node with lowest f_cost
            current_node = heapq.heappop(open_list)
            current_key = self.discretize_state(current_node.state)
            
            # Check if already processed
            if current_key in closed_set:
                continue
                
            closed_set.add(current_key)
            
            # Store explored node for visualization
            self.explored_nodes.append(current_node)
            
            # Check if goal reached
            if self.is_goal_reached(current_node.state, goal):
                print(f"Path found in {iterations} iterations")
                return self.reconstruct_path(current_node)
            
            # Generate successors
            successors = self.get_successors(current_node)
            
            for successor in successors:
                successor_key = self.discretize_state(successor.state)
                
                # Skip if already in closed set
                if successor_key in closed_set:
                    continue
                
                # Calculate heuristic cost
                successor.h_cost = self.heuristic_cost(successor.state, goal)
                
                # Check if this path to successor is better
                if successor_key in node_map:
                    existing_node = node_map[successor_key]
                    if successor.g_cost < existing_node.g_cost:
                        # Update existing node
                        existing_node.g_cost = successor.g_cost
                        existing_node.parent = successor.parent
                        existing_node.steer_cost = successor.steer_cost
                        existing_node.turn_cost = successor.turn_cost
                        existing_node.cusp_cost = successor.cusp_cost
                        existing_node.path_cost = successor.path_cost
                        heapq.heappush(open_list, existing_node)
                else:
                    # Add new node
                    node_map[successor_key] = successor
                    heapq.heappush(open_list, successor)
        
        print(f"No path found after {iterations} iterations")
        return None
    
    def get_visualization_data(self):
        """Get all data needed for visualization
        
        Returns:
            Dictionary containing visualization data
        """
        return {
            'explored_nodes': self.explored_nodes,
            'simulation_trajectories': self.simulation_trajectories,
            'obstacle_map': self.obstacle_map,
            'map_origin_x': self.map_origin_x,
            'map_origin_y': self.map_origin_y,
            'grid_resolution': self.grid_resolution,
            'vehicle_model': self.vehicle_model
        }
    
    def get_statistics(self, path: Optional[List[State]]) -> dict:
        """Get path and search statistics
        
        Args:
            path: The planned path
            
        Returns:
            Dictionary containing statistics
        """
        if not path:
            return {
                'path_found': False,
                'nodes_explored': len(self.explored_nodes),
                'trajectories_simulated': len(self.simulation_trajectories)
            }
        
        # Basic path statistics
        total_distance = sum(np.sqrt((path[i+1].x - path[i].x)**2 + 
                                   (path[i+1].y - path[i].y)**2) 
                           for i in range(len(path)-1))
        
        # Steering statistics
        steer_angles = [state.steer for state in path]
        max_steer = max(abs(s) for s in steer_angles)
        avg_steer = np.mean([abs(s) for s in steer_angles])
        
        # Direction changes
        direction_changes = 0
        for i in range(1, len(path)):
            if path[i].direction != path[i-1].direction:
                direction_changes += 1
        
        # Curvature analysis
        curvatures = []
        for i in range(1, len(path) - 1):
            p1 = np.array([path[i-1].x, path[i-1].y])
            p2 = np.array([path[i].x, path[i].y])
            p3 = np.array([path[i+1].x, path[i+1].y])
            
            a = np.linalg.norm(p2 - p1)
            b = np.linalg.norm(p3 - p2)
            c = np.linalg.norm(p3 - p1)
            
            if a > 1e-6 and b > 1e-6 and c > 1e-6:
                area = 0.5 * abs(np.cross(p2 - p1, p3 - p1))
                curvature = 4 * area / (a * b * c)
                curvatures.append(curvature)
        
        return {
            'path_found': True,
            'path_length_waypoints': len(path),
            'total_distance': total_distance,
            'average_waypoint_spacing': total_distance/(len(path)-1) if len(path) > 1 else 0,
            'max_steering_angle': max_steer,
            'average_steering_angle': avg_steer,
            'steering_utilization': max_steer/self.vehicle_model.max_steer if self.vehicle_model.max_steer > 0 else 0,
            'direction_changes': direction_changes,
            'max_curvature': max(curvatures) if curvatures else 0,
            'average_curvature': np.mean(curvatures) if curvatures else 0,
            'nodes_explored': len(self.explored_nodes),
            'trajectories_simulated': len(self.simulation_trajectories)
        }


# Example usage and testing
if __name__ == "__main__":
    # Create vehicle model
    vehicle = VehicleModel(wheelbase=2.5, max_steer=np.pi/4)
    
    # Create hybrid A* planner
    planner = HybridAStar(
        vehicle_model=vehicle,
        grid_resolution=0.5,
        angle_resolution=np.pi/8,
        velocity=2.0,
        simulation_time=0.5,
        dt=0.1
    )
    
    # Create simple obstacle map (optional)
    map_size = 50
    obstacle_map = np.zeros((map_size, map_size))
    
    # Add some obstacles
    obstacle_map[20:30, 15:25] = 1  # Rectangle obstacle
    obstacle_map[10:15, 35:45] = 1  # Another obstacle
    
    planner.set_obstacle_map(obstacle_map, origin_x=-10, origin_y=-10)
    
    # Define start and goal states
    start = State(x=-5.0, y=-5.0, yaw=np.pi/4, direction=DirectionMode.FORWARD)
    goal = State(x=15.0, y=15.0, yaw=np.pi/2, direction=DirectionMode.FORWARD)
    
    # Plan path
    print("Planning path with Hybrid A*...")
    path = planner.plan_path(start, goal, max_iterations=5000)
    
    if path:
        print(f"Path found with {len(path)} waypoints")
        
        # Print statistics
        stats = planner.get_statistics(path)
        print(f"\nPath Statistics:")
        for key, value in stats.items():
            if isinstance(value, float):
                print(f"  {key}: {value:.3f}")
            else:
                print(f"  {key}: {value}")
        
        # For visualization, you would use:
        # from .visualizer import HybridAStarVisualizer
        # visualizer = HybridAStarVisualizer()
        # viz_data = planner.get_visualization_data()
        # visualizer.visualize_path(path, start, goal, **viz_data)
        
    else:
        print("No path found!")
        stats = planner.get_statistics(None)
        print(f"Search Statistics:")
        for key, value in stats.items():
            print(f"  {key}: {value}")
