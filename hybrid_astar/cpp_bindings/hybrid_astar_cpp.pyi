"""
Type stubs for hybrid_astar_cpp

This module provides type annotations for the Hybrid A* path planning algorithm
C++ implementation with Python bindings.
"""

from typing import List, Optional, Dict, Tuple, Union, Any
import numpy as np

class DirectionMode:
    """Vehicle direction mode enumeration
    
    Note: These are integer constants, not enum objects.
    Use DirectionMode.FORWARD (1), DirectionMode.BACKWARD (-1), DirectionMode.NONE (0)
    directly as integer values in State constructor and other functions.
    """
    FORWARD: int = 1
    BACKWARD: int = -1
    NONE: int = 0

class Costs:
    """Cost components for path planning"""
    distance: float
    steer: float
    turn: float
    cusp: float
    
    def __init__(self, distance: float = 0.0, steer: float = 0.0, turn: float = 0.0, cusp: float = 0.0) -> None: ...

class State:
    """Vehicle state representation"""
    x: float
    y: float
    yaw: float
    direction: int
    steer: float
    
    def __init__(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0,
                 direction: int = DirectionMode.NONE, steer: float = 0.0) -> None: ...
    
    def __eq__(self, other: Any) -> bool: ...
    def __ne__(self, other: Any) -> bool: ...

class Node:
    """A* search node"""
    state: State
    g_cost: float
    h_cost: float
    parent: Optional[Any]  # Node
    costs: Costs
    forward_simulation_trajectory: List[State]
    
    def __init__(self, state: State = ..., g_cost: float = 0.0, parent: Optional[Any] = None) -> None: ...
    
    def f_cost(self) -> float: ...
    def steer_cost(self) -> float: ...
    def turn_cost(self) -> float: ...
    def cusp_cost(self) -> float: ...

class VehicleModel:
    """Simple bicycle model for vehicle simulation"""
    wheelbase: float
    max_steer: float
    
    def __init__(self, wheelbase: float = 2.5, max_steer: float = 0.7853981633974483) -> None: ...
    
    def simulate_motion(self, state: State, velocity: float, steer_rate: float, 
                       dt: float, steps: int = 1) -> List[State]: ...
    
    @staticmethod
    def normalize_angle(angle: float) -> float: ...

class HybridAStar:
    """Hybrid A* path planning algorithm"""
    def __init__(self, vehicle_model: VehicleModel, grid_resolution: float = 1.0,
                 angle_resolution: float = 0.39269908169872414, steer_resolution: float = 0.19634954084936207,
                 velocity: float = 2.0, simulation_time: float = 1.0, dt: float = 0.1) -> None: ...
    
    def set_obstacle_map(self, obstacle_map: List[List[int]], origin_x: float = 0.0, origin_y: float = 0.0) -> None: ...
    
    def is_collision_free(self, state: State) -> bool: ...
    
    def heuristic_cost(self, state: State, goal: State) -> float: ...
    
    def plan_path(self, start: State, goal: State, max_iterations: int = 10000) -> Optional[List[Node]]: ...
    
    def extract_detailed_path(self, path_nodes: List[Node]) -> List[State]: ...
    
    def get_statistics(self, path: Optional[List[Node]]) -> Dict[str, float]: ...
    
    def get_explored_nodes(self) -> List[Node]: ...
    def get_simulation_trajectories(self) -> List[List[State]]: ...
    def get_obstacle_map(self) -> List[List[int]]: ...
    def get_map_origin_x(self) -> float: ...
    def get_map_origin_y(self) -> float: ...
    def get_grid_resolution(self) -> float: ...
    def get_vehicle_model(self) -> VehicleModel: ...

def create_obstacle_map(width: int, height: int) -> List[List[int]]: ...

def add_rectangle_obstacle(obstacle_map: List[List[int]], start_x: int, start_y: int, 
                          end_x: int, end_y: int) -> List[List[int]]: ...

M_PI: float
__version__: str