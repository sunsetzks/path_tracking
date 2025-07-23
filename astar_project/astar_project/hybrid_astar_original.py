"""
Hybrid A* Path Planning Algorithm
Implementation with steering angle cost, path smoothness, turning cost, 
cusp cost, and forward simulation considering steering angle velocity.

Author: Your Name
Date: 2025-07-22
"""

import numpy as np
import heapq
import math
from typing import List, Tuple, Optional, Set
from dataclasses import dataclass, field
from enum import Enum
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.colors import Normalize

# Optional scipy import for smooth path interpolation
try:
    from scipy.interpolate import interp1d
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False


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
        
        # Visualization data
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
    
    def visualize_path(self, path: List[State], start: State, goal: State, 
                      show_exploration=True, show_trajectories=True, show_costs=True):
        """Enhanced visualization of the planned path with exploration details"""
        if not path:
            print("No path to visualize")
            return
            
        # Create subplots
        if show_costs:
            fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 8))
        else:
            fig, ax1 = plt.subplots(1, 1, figsize=(15, 10))
        
        # Main path visualization
        self._plot_main_visualization(ax1, path, start, goal, show_exploration, show_trajectories)
        
        # Cost analysis visualization
        if show_costs:
            self._plot_cost_analysis(ax2, path)
        
        plt.tight_layout()
        plt.show()
        
        # Print detailed path statistics
        self._print_path_statistics(path)
    
    def _plot_main_visualization(self, ax, path, start, goal, show_exploration, show_trajectories):
        """Plot main path visualization with exploration details"""
        
        # Plot obstacle map if available
        if self.obstacle_map is not None:
            extent = (self.map_origin_x, 
                     self.map_origin_x + self.map_width * self.grid_resolution,
                     self.map_origin_y,
                     self.map_origin_y + self.map_height * self.grid_resolution)
            ax.imshow(self.obstacle_map, extent=extent, origin='lower', 
                     cmap='gray', alpha=0.3)
        
        # Plot exploration nodes with enhanced visibility (if enabled)
        if show_exploration and self.explored_nodes:
            # Plot parent-child connections first (behind nodes)
            print(f"Plotting {len(self.explored_nodes)} exploration connections...")
            connection_count = 0
            for node in self.explored_nodes:
                if node.parent is not None:
                    # Draw connection from parent to current node
                    ax.plot([node.parent.state.x, node.state.x], 
                           [node.parent.state.y, node.state.y], 
                           'cyan', linewidth=0.8, alpha=0.4, zorder=1)
                    connection_count += 1
            
            # Plot exploration nodes with better visibility
            explored_x = [node.state.x for node in self.explored_nodes]
            explored_y = [node.state.y for node in self.explored_nodes]
            
            # Use different colors based on cost for better insight
            f_costs = [node.f_cost for node in self.explored_nodes]
            if f_costs:
                # Normalize costs for color mapping
                min_cost = min(f_costs)
                max_cost = max(f_costs)
                if max_cost > min_cost:
                    normalized_costs = [(cost - min_cost) / (max_cost - min_cost) for cost in f_costs]
                else:
                    normalized_costs = [0.5] * len(f_costs)
                
                scatter = ax.scatter(explored_x, explored_y, c=normalized_costs, 
                                   cmap='viridis', s=20, alpha=0.7, zorder=2,
                                   edgecolors='white', linewidths=0.5,
                                   label=f'Explored Nodes ({len(self.explored_nodes)}, {connection_count} connections)')
                
                # Add colorbar for cost visualization
                if len(self.explored_nodes) > 10:  # Only add colorbar if meaningful
                    cbar = plt.colorbar(scatter, ax=ax, shrink=0.3, pad=0.02)
                    cbar.set_label('F-cost (normalized)', fontsize=8)
            else:
                ax.scatter(explored_x, explored_y, c='lightblue', s=20, alpha=0.7, 
                          edgecolors='white', linewidths=0.5, zorder=2,
                          label=f'Explored Nodes ({len(self.explored_nodes)})')
        
        # Plot simulation trajectories (if enabled)
        if show_trajectories and self.simulation_trajectories:
            # Sample trajectories to avoid overcrowding
            sample_rate = max(1, len(self.simulation_trajectories) // 100)
            sampled_trajectories = self.simulation_trajectories[::sample_rate]
            
            for traj in sampled_trajectories:
                states = traj['states']
                direction = traj['direction']
                
                x_coords = [s.x for s in states]
                y_coords = [s.y for s in states]
                
                # Different colors for forward/backward
                color = 'lightgreen' if direction == DirectionMode.FORWARD else 'lightcoral'
                alpha = 0.3
                
                ax.plot(x_coords, y_coords, color=color, alpha=alpha, linewidth=0.8)
        
        # Plot final path with enhanced details - smooth curves
        x_coords = np.array([state.x for state in path])
        y_coords = np.array([state.y for state in path])
        
        # Color the path by steering angle
        steer_angles = [state.steer for state in path]
        
        # Create smooth interpolated path for better visualization
        if len(path) > 3 and HAS_SCIPY:
            try:
                # Parameter for interpolation (arc length approximation)
                t = np.linspace(0, 1, len(path))
                
                # Create smooth interpolation
                f_x = interp1d(t, x_coords, kind='cubic', assume_sorted=True)
                f_y = interp1d(t, y_coords, kind='cubic', assume_sorted=True)
                
                # Generate more points for smooth curve
                t_smooth = np.linspace(0, 1, len(path) * 5)  # 5x more points
                x_smooth = f_x(t_smooth)
                y_smooth = f_y(t_smooth)
                
                # Interpolate steering angles as well
                f_steer = interp1d(t, steer_angles, kind='linear', assume_sorted=True)
                steer_smooth = f_steer(t_smooth)
                
                # Create segments for colored smooth path
                for i in range(len(x_smooth)-1):
                    # Color based on steering angle
                    steer_normalized = abs(steer_smooth[i]) / self.vehicle_model.max_steer
                    color_intensity = steer_normalized
                    
                    if steer_smooth[i] > 0:
                        color = cm.get_cmap('Reds')(0.3 + 0.7 * color_intensity)
                    elif steer_smooth[i] < 0:
                        color = cm.get_cmap('Blues')(0.3 + 0.7 * color_intensity)
                    else:
                        color = 'green'
                    
                    ax.plot([x_smooth[i], x_smooth[i+1]], [y_smooth[i], y_smooth[i+1]], 
                           color=color, linewidth=3, alpha=0.8)
                
            except Exception as e:
                print(f"Warning: Smooth interpolation failed ({e}), using linear path visualization")
                # Fall back to original linear segments
                for i in range(len(path)-1):
                    steer_normalized = abs(steer_angles[i]) / self.vehicle_model.max_steer
                    color_intensity = steer_normalized
                    
                    if steer_angles[i] > 0:
                        color = cm.get_cmap('Reds')(0.3 + 0.7 * color_intensity)
                    elif steer_angles[i] < 0:
                        color = cm.get_cmap('Blues')(0.3 + 0.7 * color_intensity)
                    else:
                        color = 'green'
                    
                    ax.plot([x_coords[i], x_coords[i+1]], [y_coords[i], y_coords[i+1]], 
                           color=color, linewidth=3, alpha=0.8)
        else:
            # For very short paths, use original method
            for i in range(len(path)-1):
                steer_normalized = abs(steer_angles[i]) / self.vehicle_model.max_steer
                color_intensity = steer_normalized
                
                if steer_angles[i] > 0:
                    color = cm.get_cmap('Reds')(0.3 + 0.7 * color_intensity)
                elif steer_angles[i] < 0:
                    color = cm.get_cmap('Blues')(0.3 + 0.7 * color_intensity)
                else:
                    color = 'green'
                
                ax.plot([x_coords[i], x_coords[i+1]], [y_coords[i], y_coords[i+1]], 
                       color=color, linewidth=3, alpha=0.8)
        
        # Plot vehicle orientations along path
        orientation_step = max(1, len(path) // 30)
        for i in range(0, len(path), orientation_step):
            state = path[i]
            
            # Vehicle body representation
            vehicle_length = 0.8
            vehicle_width = 0.4
            
            # Calculate vehicle corners
            cos_yaw = np.cos(state.yaw)
            sin_yaw = np.sin(state.yaw)
            
            # Vehicle outline
            front_x = state.x + vehicle_length/2 * cos_yaw
            front_y = state.y + vehicle_length/2 * sin_yaw
            rear_x = state.x - vehicle_length/2 * cos_yaw
            rear_y = state.y - vehicle_length/2 * sin_yaw
            
            # Draw vehicle as rectangle
            vehicle_corners_x = [
                rear_x - vehicle_width/2 * sin_yaw,
                rear_x + vehicle_width/2 * sin_yaw,
                front_x + vehicle_width/2 * sin_yaw,
                front_x - vehicle_width/2 * sin_yaw,
                rear_x - vehicle_width/2 * sin_yaw
            ]
            vehicle_corners_y = [
                rear_y + vehicle_width/2 * cos_yaw,
                rear_y - vehicle_width/2 * cos_yaw,
                front_y - vehicle_width/2 * cos_yaw,
                front_y + vehicle_width/2 * cos_yaw,
                rear_y + vehicle_width/2 * cos_yaw
            ]
            
            ax.plot(vehicle_corners_x, vehicle_corners_y, 'k-', linewidth=1, alpha=0.7)
            
            # Direction arrow
            arrow_length = 0.6
            dx = arrow_length * cos_yaw
            dy = arrow_length * sin_yaw
            ax.arrow(state.x, state.y, dx, dy, head_width=0.15, 
                    head_length=0.15, fc='darkblue', ec='darkblue', alpha=0.8)
            
            # Steering angle visualization
            if abs(state.steer) > 0.1:  # Only show if significant steering
                # Front wheel position
                front_wheel_x = state.x + vehicle_length/2 * cos_yaw
                front_wheel_y = state.y + vehicle_length/2 * sin_yaw
                
                # Steered wheel direction
                wheel_yaw = state.yaw + state.steer
                wheel_dx = 0.3 * np.cos(wheel_yaw)
                wheel_dy = 0.3 * np.sin(wheel_yaw)
                
                ax.arrow(front_wheel_x, front_wheel_y, wheel_dx, wheel_dy,
                        head_width=0.1, head_length=0.1, 
                        fc='red', ec='red', alpha=0.8)
        
        # Enhanced start and goal visualization
        # Start
        ax.plot(start.x, start.y, 'go', markersize=12, label='Start', zorder=10)
        start_dx = 1.2 * np.cos(start.yaw)
        start_dy = 1.2 * np.sin(start.yaw)
        ax.arrow(start.x, start.y, start_dx, start_dy, 
                head_width=0.4, head_length=0.4, fc='green', ec='darkgreen', 
                linewidth=2, zorder=10)
        
        # Goal
        ax.plot(goal.x, goal.y, 'ro', markersize=12, label='Goal', zorder=10)
        goal_dx = 1.2 * np.cos(goal.yaw)
        goal_dy = 1.2 * np.sin(goal.yaw)
        ax.arrow(goal.x, goal.y, goal_dx, goal_dy,
                head_width=0.4, head_length=0.4, fc='red', ec='darkred',
                linewidth=2, zorder=10)
        
        # Waypoint numbers (every 10th point)
        waypoint_step = max(1, len(path) // 15)
        for i in range(0, len(path), waypoint_step):
            ax.annotate(f'{i}', (path[i].x, path[i].y), 
                       xytext=(5, 5), textcoords='offset points',
                       fontsize=8, alpha=0.7, 
                       bbox=dict(boxstyle='round,pad=0.2', facecolor='yellow', alpha=0.7))
        
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title('Hybrid A* Path Planning - Detailed Visualization', fontsize=14)
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        
        # Add color bar for steering angle
        sm = cm.ScalarMappable(cmap=cm.get_cmap('RdBu_r'), 
                               norm=Normalize(vmin=-self.vehicle_model.max_steer, 
                                             vmax=self.vehicle_model.max_steer))
        sm.set_array([])
        cbar = plt.colorbar(sm, ax=ax, shrink=0.6)
        cbar.set_label('Steering Angle (rad)', fontsize=10)
    
    def _plot_cost_analysis(self, ax, path):
        """Plot cost analysis charts"""
        if len(path) < 2:
            return
            
        # Calculate costs along path
        path_indices = list(range(len(path)))
        steer_costs = []
        turn_costs = []
        curvatures = []
        speeds = []
        
        for i, state in enumerate(path):
            # Steering cost
            steer_cost = abs(state.steer) / self.vehicle_model.max_steer
            steer_costs.append(steer_cost)
            
            # Turn cost (yaw change rate)
            if i > 0:
                yaw_change = abs(self.vehicle_model.normalize_angle(
                    state.yaw - path[i-1].yaw)) / self.dt
                turn_costs.append(yaw_change)
            else:
                turn_costs.append(0)
            
            # Curvature approximation
            if i > 0 and i < len(path) - 1:
                p1 = np.array([path[i-1].x, path[i-1].y])
                p2 = np.array([state.x, state.y])
                p3 = np.array([path[i+1].x, path[i+1].y])
                
                # Calculate curvature
                a = np.linalg.norm(p2 - p1)
                b = np.linalg.norm(p3 - p2)
                c = np.linalg.norm(p3 - p1)
                
                if a > 1e-6 and b > 1e-6:
                    area = 0.5 * abs(np.cross(p2 - p1, p3 - p1))
                    curvature = 4 * area / (a * b * c) if c > 1e-6 else 0
                else:
                    curvature = 0
                curvatures.append(curvature)
            else:
                curvatures.append(0)
            
            # Speed (constant for this implementation)
            speeds.append(self.velocity)
        
        # Create subplots for different cost components
        ax.clear()
        
        # Multi-axis plot
        ax2 = ax.twinx()
        ax3 = ax.twinx()
        ax3.spines['right'].set_position(('outward', 60))
        
        # Plot different cost components
        line1 = ax.plot(path_indices, steer_costs, 'r-', linewidth=2, 
                       label='Steering Cost', alpha=0.8)
        line2 = ax2.plot(path_indices, turn_costs, 'b-', linewidth=2, 
                        label='Turn Rate (rad/s)', alpha=0.8)
        line3 = ax3.plot(path_indices, curvatures, 'g-', linewidth=2, 
                        label='Curvature', alpha=0.8)
        
        # Styling
        ax.set_xlabel('Path Index', fontsize=12)
        ax.set_ylabel('Steering Cost', color='r', fontsize=12)
        ax2.set_ylabel('Turn Rate (rad/s)', color='b', fontsize=12)
        ax3.set_ylabel('Curvature', color='g', fontsize=12)
        
        ax.tick_params(axis='y', labelcolor='r')
        ax2.tick_params(axis='y', labelcolor='b')
        ax3.tick_params(axis='y', labelcolor='g')
        
        ax.set_title('Path Cost Analysis', fontsize=14)
        ax.grid(True, alpha=0.3)
        
        # Combined legend
        lines = line1 + line2 + line3
        labels = [l.get_label() for l in lines]
        ax.legend(lines, labels, loc='upper right')
    
    def _print_path_statistics(self, path):
        """Print detailed path statistics"""
        if not path:
            return
            
        print(f"\n{'='*60}")
        print(f"DETAILED PATH STATISTICS")
        print(f"{'='*60}")
        
        # Basic statistics
        total_distance = sum(np.sqrt((path[i+1].x - path[i].x)**2 + 
                                   (path[i+1].y - path[i].y)**2) 
                           for i in range(len(path)-1))
        
        print(f"Path length: {len(path)} waypoints")
        print(f"Total distance: {total_distance:.2f} m")
        print(f"Average waypoint spacing: {total_distance/(len(path)-1):.2f} m")
        
        # Time and speed statistics
        total_time = (len(path) - 1) * self.simulation_time
        print(f"Estimated travel time: {total_time:.2f} s")
        print(f"Average speed: {total_distance/total_time:.2f} m/s")
        
        # Steering statistics
        steer_angles = [state.steer for state in path]
        max_steer = max(abs(s) for s in steer_angles)
        avg_steer = np.mean([abs(s) for s in steer_angles])
        
        print(f"\nSTEERING ANALYSIS:")
        print(f"Maximum steering angle: {np.degrees(max_steer):.1f}° ({max_steer:.3f} rad)")
        print(f"Average |steering|: {np.degrees(avg_steer):.1f}° ({avg_steer:.3f} rad)")
        print(f"Steering utilization: {max_steer/self.vehicle_model.max_steer*100:.1f}%")
        
        # Direction changes
        direction_changes = 0
        for i in range(1, len(path)):
            if path[i].direction != path[i-1].direction:
                direction_changes += 1
        
        print(f"Direction changes (cusps): {direction_changes}")
        
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
        
        if curvatures:
            print(f"\nCURVATURE ANALYSIS:")
            print(f"Maximum curvature: {max(curvatures):.4f}")
            print(f"Average curvature: {np.mean(curvatures):.4f}")
        
        # Exploration statistics
        if self.explored_nodes:
            print(f"\nSEARCH STATISTICS:")
            print(f"Nodes explored: {len(self.explored_nodes)}")
            print(f"Trajectories simulated: {len(self.simulation_trajectories)}")
            
            # Cost breakdown for final path
            if hasattr(path[-1], 'g_cost'):
                final_node = None
                # Find the final node in explored nodes
                for node in self.explored_nodes:
                    if (abs(node.state.x - path[-1].x) < 0.1 and 
                        abs(node.state.y - path[-1].y) < 0.1):
                        final_node = node
                        break
                
                if final_node:
                    print(f"\nFINAL PATH COSTS:")
                    print(f"Total cost (g): {final_node.g_cost:.2f}")
                    print(f"  - Motion cost component: ~{final_node.g_cost * 0.6:.2f}")
                    print(f"  - Steering cost component: {self.w_steer * final_node.steer_cost:.2f}")
                    print(f"  - Turn cost component: {self.w_turn * final_node.turn_cost:.2f}")
                    print(f"  - Cusp cost component: {self.w_cusp * final_node.cusp_cost:.2f}")
                    print(f"  - Path smoothness component: {self.w_path * final_node.path_cost:.2f}")
        
        print(f"{'='*60}")
    
    def visualize_detailed_search_tree(self, path: List[State], start: State, goal: State, 
                                      max_connections: int = 1000, node_spacing_filter: float = 0.5):
        """
        Detailed visualization of search tree with all connections clearly visible
        
        Args:
            path: Final path
            start: Start state  
            goal: Goal state
            max_connections: Maximum number of connections to show
            node_spacing_filter: Minimum distance between nodes to reduce clutter
        """
        if not self.explored_nodes:
            print("No exploration data available")
            return
            
        fig, ax = plt.subplots(1, 1, figsize=(16, 12))
        
        # Plot obstacle map with higher contrast
        if self.obstacle_map is not None:
            extent = (self.map_origin_x, 
                     self.map_origin_x + self.map_width * self.grid_resolution,
                     self.map_origin_y,
                     self.map_origin_y + self.map_height * self.grid_resolution)
            ax.imshow(self.obstacle_map, extent=extent, origin='lower', 
                     cmap='gray_r', alpha=0.7, zorder=0)
        
        # Filter nodes to reduce visual clutter while keeping structure
        filtered_nodes = []
        for i, node in enumerate(self.explored_nodes):
            # Keep every N-th node or nodes with special significance
            if (i % 3 == 0 or  # Every 3rd node
                node.parent is None or  # Root nodes
                abs(node.state.x - goal.x) + abs(node.state.y - goal.y) < 3.0):  # Near goal
                
                # Check spacing with already filtered nodes
                too_close = False
                for fn in filtered_nodes[-20:]:  # Check last 20 for performance
                    if (abs(node.state.x - fn.state.x) < node_spacing_filter and 
                        abs(node.state.y - fn.state.y) < node_spacing_filter):
                        too_close = True
                        break
                
                if not too_close:
                    filtered_nodes.append(node)
        
        print(f"Showing {len(filtered_nodes)} filtered nodes from {len(self.explored_nodes)} total")
        
        # Draw parent-child connections with enhanced visibility
        connection_count = 0
        parent_child_pairs = []
        
        for node in filtered_nodes:
            if node.parent is not None and connection_count < max_connections:
                # Find parent in filtered list
                parent_node = None
                for fn in filtered_nodes:
                    if (abs(fn.state.x - node.parent.state.x) < 0.1 and 
                        abs(fn.state.y - node.parent.state.y) < 0.1):
                        parent_node = fn
                        break
                
                if parent_node:
                    # Color by search depth/generation
                    depth_color = min(1.0, connection_count / max_connections)
                    color = cm.get_cmap('plasma')(depth_color)
                    
                    # Draw connection with gradient effect
                    ax.plot([parent_node.state.x, node.state.x], 
                           [parent_node.state.y, node.state.y], 
                           color=color, linewidth=1.5, alpha=0.7, zorder=1,
                           solid_capstyle='round')
                    
                    # Add small arrow to show direction
                    if connection_count % 5 == 0:  # Only for some connections
                        mid_x = (parent_node.state.x + node.state.x) / 2
                        mid_y = (parent_node.state.y + node.state.y) / 2
                        dx = node.state.x - parent_node.state.x
                        dy = node.state.y - parent_node.state.y
                        length = np.sqrt(dx*dx + dy*dy)
                        if length > 0.1:
                            dx_norm = dx / length * 0.3
                            dy_norm = dy / length * 0.3
                            ax.arrow(mid_x - dx_norm/2, mid_y - dy_norm/2, 
                                   dx_norm, dy_norm,
                                   head_width=0.15, head_length=0.15,
                                   fc=color, ec=color, alpha=0.8, zorder=2)
                    
                    parent_child_pairs.append((parent_node, node))
                    connection_count += 1
        
        # Draw nodes with enhanced visibility
        node_x = []
        node_y = []
        node_costs = []
        node_sizes = []
        node_colors = []
        
        for i, node in enumerate(filtered_nodes):
            node_x.append(node.state.x)
            node_y.append(node.state.y)
            node_costs.append(node.f_cost)
            
            # Variable size based on importance
            if node.parent is None:  # Start node
                size = 120
                node_colors.append('green')
            elif abs(node.state.x - goal.x) + abs(node.state.y - goal.y) < 2.0:  # Near goal
                size = 80
                node_colors.append('orange')
            else:
                size = max(20, 60 - min(40, node.f_cost))
                node_colors.append('lightblue')
            
            node_sizes.append(size)
        
        # Plot nodes
        for i in range(len(node_x)):
            ax.scatter(node_x[i], node_y[i], 
                      c=node_colors[i], s=node_sizes[i], 
                      alpha=0.9, zorder=3,
                      edgecolors='black', linewidths=1)
            
            # Add cost labels for important nodes
            if node_sizes[i] > 50:
                ax.annotate(f'{node_costs[i]:.1f}', 
                           (node_x[i], node_y[i]), 
                           xytext=(3, 3), textcoords='offset points',
                           fontsize=8, fontweight='bold',
                           bbox=dict(boxstyle='round,pad=0.2', 
                                   facecolor='white', alpha=0.8))
        
        # Highlight final path with thick line
        if path:
            path_x = [state.x for state in path]
            path_y = [state.y for state in path]
            ax.plot(path_x, path_y, 'red', linewidth=6, alpha=0.9, 
                   label='Final Path', zorder=5, solid_capstyle='round')
            
            # Mark path nodes
            ax.scatter(path_x, path_y, c='red', s=100, zorder=6, 
                      edgecolors='white', linewidths=2, marker='D')
        
        # Enhanced start and goal markers
        ax.scatter(start.x, start.y, c='darkgreen', s=200, 
                  marker='s', label='Start', zorder=10,
                  edgecolors='white', linewidths=3)
        ax.scatter(goal.x, goal.y, c='darkred', s=200, 
                  marker='*', label='Goal', zorder=10,
                  edgecolors='white', linewidths=3)
        
        # Add search statistics as text
        stats_text = f"""Search Statistics:
Total Nodes: {len(self.explored_nodes)}
Shown Nodes: {len(filtered_nodes)}
Connections: {connection_count}
Path Length: {len(path) if path else 0}"""
        
        ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, 
               fontsize=10, verticalalignment='top',
               bbox=dict(boxstyle='round,pad=0.5', facecolor='lightyellow', alpha=0.9))
        
        ax.set_xlabel('X (m)', fontsize=14, fontweight='bold')
        ax.set_ylabel('Y (m)', fontsize=14, fontweight='bold')
        ax.set_title('Detailed Search Tree Visualization\n(Parent-Child Connections & Node Costs)', 
                    fontsize=16, fontweight='bold')
        ax.legend(fontsize=12, loc='upper right')
        ax.grid(True, alpha=0.4, linestyle='--')
        ax.axis('equal')
        
        # Set better axis limits
        if filtered_nodes:
            all_x = [n.state.x for n in filtered_nodes] + [start.x, goal.x]
            all_y = [n.state.y for n in filtered_nodes] + [start.y, goal.y]
            margin = 2.0
            ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
            ax.set_ylim(min(all_y) - margin, max(all_y) + margin)
        
        plt.tight_layout()
        plt.show()
        
        print(f"Displayed search tree with {len(filtered_nodes)} nodes and {connection_count} connections")
    
    def visualize_search_tree(self, path: List[State], start: State, goal: State, 
                             max_nodes_to_show: int = 500):
        """
        Legacy method - calls the new detailed search tree visualization
        """
        return self.visualize_detailed_search_tree(path, start, goal, 
                                                  max_connections=max_nodes_to_show,
                                                  node_spacing_filter=0.3)
    
    def visualize_search_progress(self, path: List[State], start: State, goal: State, 
                                 max_nodes_to_show: int = 500):
        """Enhanced visualization of the search progress with clear connections"""
        if not self.explored_nodes:
            print("No exploration data available")
            return
            
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))
        
        # Left plot: Search tree with connections
        self._plot_search_tree(ax1, path, start, goal, max_nodes_to_show)
        
        # Right plot: Search progression over time
        self._plot_search_progression(ax2, path, start, goal, max_nodes_to_show)
        
        plt.tight_layout()
        plt.show()
    
    def _plot_search_tree(self, ax, path, start, goal, max_nodes_to_show):
        """Plot search tree with clear parent-child connections"""
        # Plot obstacle map
        if self.obstacle_map is not None:
            extent = (self.map_origin_x, 
                     self.map_origin_x + self.map_width * self.grid_resolution,
                     self.map_origin_y,
                     self.map_origin_y + self.map_height * self.grid_resolution)
            ax.imshow(self.obstacle_map, extent=extent, origin='lower', 
                     cmap='gray', alpha=0.3)
        
        # Limit nodes to show for performance
        nodes_to_show = min(len(self.explored_nodes), max_nodes_to_show)
        step = max(1, len(self.explored_nodes) // nodes_to_show)
        selected_nodes = self.explored_nodes[::step]
        
        # First pass: draw all parent-child connections
        print(f"Drawing search tree with {len(selected_nodes)} nodes...")
        connection_count = 0
        for node in selected_nodes:
            if node.parent is not None:
                # Check if parent is also in selected nodes (for cleaner visualization)
                parent_in_selection = any(abs(n.state.x - node.parent.state.x) < 0.1 and 
                                        abs(n.state.y - node.parent.state.y) < 0.1 
                                        for n in selected_nodes)
                
                if parent_in_selection or connection_count < 200:  # Limit connections for clarity
                    # Color connections by search depth/cost
                    depth_color = min(1.0, node.g_cost / 50.0)  # Normalize roughly
                    color = cm.get_cmap('plasma')(depth_color)
                    
                    ax.plot([node.parent.state.x, node.state.x], 
                           [node.parent.state.y, node.state.y], 
                           color=color, linewidth=1.2, alpha=0.6, zorder=1)
                    connection_count += 1
        
        # Second pass: draw nodes
        exploration_x = []
        exploration_y = []
        exploration_costs = []
        node_sizes = []
        
        for node in selected_nodes:
            exploration_x.append(node.state.x)
            exploration_y.append(node.state.y)
            exploration_costs.append(node.f_cost)
            # Larger nodes for important nodes (low f-cost)
            size = max(15, 50 - min(40, node.f_cost * 2))
            node_sizes.append(size)
        
        # Plot nodes with cost-based coloring
        if exploration_costs:
            scatter = ax.scatter(exploration_x, exploration_y, 
                               c=exploration_costs, cmap='viridis_r',  # Reverse so low cost = bright
                               s=node_sizes, alpha=0.8, zorder=3,
                               edgecolors='white', linewidths=1)
            
            # Colorbar
            cbar = plt.colorbar(scatter, ax=ax, shrink=0.6)
            cbar.set_label('F-cost (Lower=Better)', fontsize=10)
        
        # Highlight path nodes if available
        if path:
            path_x = [state.x for state in path]
            path_y = [state.y for state in path]
            ax.plot(path_x, path_y, 'red', linewidth=4, alpha=0.9, 
                   label='Final Path', zorder=4)
            ax.scatter(path_x, path_y, c='red', s=80, zorder=5, 
                      edgecolors='white', linewidths=2)
        
        # Start and goal
        ax.plot(start.x, start.y, 'go', markersize=18, label='Start', 
               zorder=10, markeredgecolor='white', markeredgewidth=2)
        ax.plot(goal.x, goal.y, 'ro', markersize=18, label='Goal', 
               zorder=10, markeredgecolor='white', markeredgewidth=2)
        
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title(f'Search Tree Structure\n({len(selected_nodes)} nodes, {connection_count} connections)', 
                    fontsize=12)
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
    
    def _plot_search_progression(self, ax, path, start, goal, max_nodes_to_show):
        """Plot search progression with temporal coloring"""
        # Plot obstacle map
        if self.obstacle_map is not None:
            extent = (self.map_origin_x, 
                     self.map_origin_x + self.map_width * self.grid_resolution,
                     self.map_origin_y,
                     self.map_origin_y + self.map_height * self.grid_resolution)
            ax.imshow(self.obstacle_map, extent=extent, origin='lower', 
                     cmap='gray', alpha=0.3)
        
        # Show exploration progression with temporal color gradient
        nodes_to_show = min(len(self.explored_nodes), max_nodes_to_show)
        step = max(1, len(self.explored_nodes) // nodes_to_show)
        
        exploration_x = []
        exploration_y = []
        exploration_order = []
        
        for i in range(0, len(self.explored_nodes), step):
            node = self.explored_nodes[i]
            exploration_x.append(node.state.x)
            exploration_y.append(node.state.y)
            exploration_order.append(i / len(self.explored_nodes))  # Normalized time
        
        # Color by exploration order (early = blue, late = red)
        scatter = ax.scatter(exploration_x, exploration_y, 
                           c=exploration_order, cmap='coolwarm', 
                           s=30, alpha=0.8, zorder=2,
                           edgecolors='white', linewidths=0.5)
        
        # Add arrows showing search direction for some nodes
        arrow_step = max(1, len(exploration_x) // 20)
        for i in range(arrow_step, len(exploration_x), arrow_step):
            if i > 0:
                dx = exploration_x[i] - exploration_x[i-arrow_step]
                dy = exploration_y[i] - exploration_y[i-arrow_step]
                if abs(dx) > 0.1 or abs(dy) > 0.1:  # Only if meaningful movement
                    ax.arrow(exploration_x[i-arrow_step], exploration_y[i-arrow_step], 
                           dx * 0.7, dy * 0.7, 
                           head_width=0.3, head_length=0.3, 
                           fc='orange', ec='darkorange', alpha=0.7, zorder=3)
        
        # Plot final path
        if path:
            x_coords = [state.x for state in path]
            y_coords = [state.y for state in path]
            ax.plot(x_coords, y_coords, 'lime', linewidth=4, 
                   label='Final Path', zorder=4, alpha=0.9)
        
        # Start and goal
        ax.plot(start.x, start.y, 'go', markersize=18, label='Start', 
               zorder=10, markeredgecolor='white', markeredgewidth=2)
        ax.plot(goal.x, goal.y, 'ro', markersize=18, label='Goal', 
               zorder=10, markeredgecolor='white', markeredgewidth=2)
        
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title(f'Search Progression Over Time\n({len(exploration_x)} nodes shown)', fontsize=12)
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        
        # Add colorbar for temporal progression
        cbar = plt.colorbar(scatter, ax=ax, shrink=0.6)
        cbar.set_label('Search Time (Blue=Early, Red=Late)', fontsize=10)


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
        
        # Enhanced visualization with all details
        print("\nShowing enhanced visualization...")
        planner.visualize_path(path, start, goal, 
                              show_exploration=True, 
                              show_trajectories=True, 
                              show_costs=True)
        
        # Show search progress visualization
        print("\nShowing search progress visualization...")
        planner.visualize_search_progress(path, start, goal, max_nodes_to_show=300)
        
        # Show detailed search tree with clear connections
        print("\nShowing detailed search tree with connections...")
        planner.visualize_detailed_search_tree(path, start, goal, 
                                              max_connections=800, 
                                              node_spacing_filter=0.4)
        
    else:
        print("No path found!")
        # Even if no path found, show exploration
        if planner.explored_nodes:
            print("Showing exploration data...")
            planner.visualize_search_progress([], start, goal)
