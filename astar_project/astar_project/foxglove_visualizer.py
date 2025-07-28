"""
Foxglove-based Visualization for Hybrid A* Path Planning

This module provides real-time visualization capabilities for the Hybrid A* algorithm
using the Foxglove WebSocket server. It creates interactive 3D visualizations that can
be viewed in Foxglove Studio.

Features:
- Real-time path planning visualization with separated channels
- Interactive exploration nodes displayed as spheres (colored by cost)
- Node simulation trajectories displayed as lines 
- Vehicle state visualization with orientation arrows
- Dedicated start/goal position visualization channel
- Cost analysis and statistics
- Obstacle map visualization
- Steering angle visualization with color coding

Visualization Channels:
- /hybrid_astar/visualization/scene: Obstacles visualization
- /hybrid_astar/visualization/path: Final planned path with vehicle orientation arrows
- /hybrid_astar/visualization/exploration: Exploration nodes and simulation trajectories
- /hybrid_astar/visualization/start_goal: Start and goal positions with orientation arrows
- /hybrid_astar/visualization/statistics: Path planning statistics
- /hybrid_astar/visualization/path_data: Raw path data with node structure information

Author: Your Name
Date: 2025-07-28
"""

import asyncio
import json
import math
import time
import os
from pathlib import Path
from typing import List, Optional, Dict, Any, Tuple
import numpy as np

try:
    import foxglove
    from foxglove import Channel, Schema, open_mcap
    from foxglove.mcap import MCAPWriter
    from foxglove.schemas import (
        Color,
        Point3,
        Vector3,
        Quaternion,
        Pose,
        LinePrimitive,
        ArrowPrimitive,
        TriangleListPrimitive,
        SceneUpdate,
        SceneEntity,
        Timestamp,
    )
    from foxglove.websocket import (
        Capability,
        WebSocketServer,
        ServerListener,
        Client,
        ChannelView,
    )
    
    FOXGLOVE_AVAILABLE = True
except ImportError:
    print("Warning: Foxglove SDK not available. Please install with: pip install foxglove-sdk")
    FOXGLOVE_AVAILABLE = False

# Import from our hybrid A* implementation
try:
    from .hybrid_astar import State, Node, DirectionMode, VehicleModel, HybridAStar
except ImportError:
    from astar_project.hybrid_astar import State, Node, DirectionMode, VehicleModel, HybridAStar

class FoxgloveHybridAStarVisualizer:
    """
    Foxglove-based real-time visualizer for Hybrid A* path planning
    
    This class creates visualization data for Foxglove Studio using 3D primitives and JSON data.
    The visualization is separated into multiple topics for better control:
    - /hybrid_astar/visualization/scene: Obstacles visualization
    - /hybrid_astar/visualization/path: Final planned path with vehicle orientation arrows
    - /hybrid_astar/visualization/exploration: Exploration nodes and simulation trajectories
    - /hybrid_astar/visualization/start_goal: Start and goal positions with orientation arrows
    - /hybrid_astar/visualization/statistics: Path planning statistics
    - /hybrid_astar/visualization/path_data: Raw path data with node structure information
    
    Based on the official Foxglove SDK examples.
    """
    
    def __init__(self, port: int = 8765, mcap_output_path: Optional[str] = None) -> None:
        """
        Initialize the Foxglove visualizer
        
        Args:
            port: WebSocket server port
            mcap_output_path: Optional path to save MCAP file. If None, defaults to log/hybrid_astar_<timestamp>.mcap
        """
        if not FOXGLOVE_AVAILABLE:
            raise ImportError("Foxglove SDK is not available. Please install it first.")
            
        self.port: int = port
        
        # Set default MCAP output path to log directory
        if mcap_output_path is None:
            # Get project root directory (assume we're in astar_project/astar_project/)
            project_root = Path(__file__).parent.parent
            log_dir = project_root / "logs"
            log_dir.mkdir(exist_ok=True)  # Create log directory if it doesn't exist
            
            timestamp = int(time.time())
            mcap_filename = f"hybrid_astar_{timestamp}.mcap"
            self.mcap_output_path = str(log_dir / mcap_filename)
        else:
            self.mcap_output_path = mcap_output_path
        
        # Visualization settings
        self.settings: Dict[str, float | bool] = {
            'path_line_thickness': 0.05,
            'path_alpha': 0.5,  # Final path transparency (1.0=opaque, 0.0=fully transparent)
            'max_exploration_nodes': 100000,  # Limit for performance
            'exploration_sphere_size': 0.03,  # Size of exploration node spheres
            'exploration_line_thickness': 0.01,  # Thickness for simulation trajectory lines
            'show_final_path_arrows': False,  # Whether to show arrows along the final path
        }
        
        # Current data
        self.current_data: Dict[str, Any] = {}
        
        # Channels (created when server starts)
        self.scene_channel: Optional[Any] = None
        self.path_channel: Optional[Any] = None
        self.exploration_channel: Optional[Any] = None
        self.start_goal_channel: Optional[Any] = None
        self.stats_channel: Optional[Channel] = None
        self.path_data_channel: Optional[Channel] = None
        self.mcap_sink: Optional[MCAPWriter] = None
    
    def start_server(self) -> Any:
        """Start the Foxglove WebSocket server with channels"""
        if not FOXGLOVE_AVAILABLE:
            raise ImportError("Foxglove SDK is not available")
            
        print(f"Starting Foxglove server on ws://localhost:{self.port}")
        
        # Import required classes
        from foxglove import start_server, Channel
        from foxglove.channels import SceneUpdateChannel
        
        # Start the WebSocket server (global context)
        server = start_server(port=self.port)
        
        # Add MCAP sink if output path specified
        if self.mcap_output_path:
            print(f"Adding MCAP sink: {self.mcap_output_path}")
            self.mcap_sink = open_mcap(self.mcap_output_path)  # Returns MCAPWriter
        
        # Create channels for 3D visualization
        self.scene_channel = SceneUpdateChannel(topic="/hybrid_astar/visualization/scene")
        self.path_channel = SceneUpdateChannel(topic="/hybrid_astar/visualization/path")
        self.exploration_channel = SceneUpdateChannel(topic="/hybrid_astar/visualization/exploration")
        self.start_goal_channel = SceneUpdateChannel(topic="/hybrid_astar/visualization/start_goal")
        self.stats_channel = Channel(topic="/hybrid_astar/visualization/statistics")
        self.path_data_channel = Channel(topic="/hybrid_astar/visualization/path_data")
        
        print(f"✓ Foxglove server started on ws://localhost:{self.port}")
        if self.mcap_output_path:
            print(f"✓ MCAP recording to: {self.mcap_output_path}")
        print(f"→ Connect Foxglove Studio to this WebSocket URL")
        print(f"→ Add a 3D panel and subscribe to '/hybrid_astar/visualization/scene' topic")
        print(f"→ Add a 3D panel and subscribe to '/hybrid_astar/visualization/path' topic")
        print(f"→ Add a 3D panel and subscribe to '/hybrid_astar/visualization/exploration' topic")
        print(f"→ Add a 3D panel and subscribe to '/hybrid_astar/visualization/start_goal' topic")
        print(f"→ Add a Plot panel for '/hybrid_astar/visualization/statistics' topic")
        print(f"→ Add a Raw Messages panel for '/hybrid_astar/visualization/path_data' topic")
        
        return server
    
    def stop_server(self) -> None:
        """Stop the server and close MCAP sink if active"""
        if self.mcap_sink:
            self.mcap_sink.close()  # MCAPWriter.close() method
            print(f"✓ MCAP file saved: {self.mcap_output_path}")
        print("Foxglove visualizer stopped")
    
    def log_scene_update(self, scene_update: SceneUpdate) -> None:
        """Log a SceneUpdate to Foxglove"""
        if self.scene_channel:
            self.scene_channel.log(scene_update)
    
    def log_path_update(self, scene_update: SceneUpdate) -> None:
        """Log a SceneUpdate to the path channel"""
        if self.path_channel:
            self.path_channel.log(scene_update)
    
    def log_exploration_update(self, scene_update: SceneUpdate) -> None:
        """Log a SceneUpdate to the exploration channel"""
        if self.exploration_channel:
            self.exploration_channel.log(scene_update)
    
    def log_start_goal_update(self, scene_update: SceneUpdate) -> None:
        """Log a SceneUpdate to the start/goal channel"""
        if self.start_goal_channel:
            self.start_goal_channel.log(scene_update)
    
    def log_statistics(self, stats_dict: Dict[str, Any]) -> None:
        """Log statistics as JSON"""
        if self.stats_channel:
            self.stats_channel.log(stats_dict)
    
    def log_path_data(self, path_data: List[Dict[str, Any]]) -> None:
        """Log path data as JSON with node structure information"""
        if self.path_data_channel:
            self.path_data_channel.log(path_data)
    
    def visualize_path_planning(self, 
                              path: List[State],
                              start: State,
                              goal: State,
                              explored_nodes: Optional[List[Node]] = None,
                              simulation_trajectories: Optional[List[Any]] = None,
                              obstacle_map: Optional[np.ndarray] = None,
                              map_origin_x: float = 0,
                              map_origin_y: float = 0,
                              grid_resolution: float = 1.0,
                              vehicle_model: Optional[VehicleModel] = None,
                              path_nodes: Optional[List[Node]] = None) -> None:
        """
        Visualize complete path planning results with 3D primitives
        
        Args:
            path: Final planned path (list of states)
            start: Start state
            goal: Goal state
            explored_nodes: Nodes explored during search
            simulation_trajectories: Forward simulation trajectories
            obstacle_map: 2D obstacle map
            map_origin_x: Map origin X coordinate
            map_origin_y: Map origin Y coordinate
            grid_resolution: Grid resolution
            vehicle_model: Vehicle kinematic model
            path_nodes: Original path nodes with full information (optional)
        """
        
        # Store current data
        self.current_data = {
            'path': path,
            'start': start,
            'goal': goal,
            'explored_nodes': explored_nodes,
            'simulation_trajectories': simulation_trajectories,
            'obstacle_map': obstacle_map,
            'map_origin_x': map_origin_x,
            'map_origin_y': map_origin_y,
            'grid_resolution': grid_resolution,
            'vehicle_model': vehicle_model,
            'path_nodes': path_nodes  # Store original nodes for detailed data
        }
        
        # Start server if not running
        if hasattr(self, 'scene_channel') and self.scene_channel is None:
            self.start_server()
        
        # Create and send separate scene updates
        path_scene_update = self.create_path_scene_update()
        if path_scene_update:
            self.log_path_update(path_scene_update)
        
        exploration_scene_update = self.create_exploration_scene_update()
        if exploration_scene_update:
            self.log_exploration_update(exploration_scene_update)
        
        # Create start/goal scene update
        start_goal_scene_update = self.create_start_goal_scene_update()
        if start_goal_scene_update:
            self.log_start_goal_update(start_goal_scene_update)
        
        # Create main scene update (obstacles only now)
        scene_update = self.create_scene_update()
        if scene_update:
            self.log_scene_update(scene_update)
        
        # Create and send statistics
        stats = self.create_statistics()
        if stats:
            self.log_statistics(stats)
        
        # Create and send path data
        path_data = self.create_path_data()
        if path_data:
            self.log_path_data(path_data)
    
    def create_scene_update(self) -> Optional[SceneUpdate]:
        """Create a Foxglove SceneUpdate with obstacles visualization primitives"""
        from foxglove.schemas import (
            SceneUpdate, SceneEntity, 
            CubePrimitive, Color, Vector3, Pose, Quaternion
        )
        
        entities: List[SceneEntity] = []
        
        # Visualize obstacles as cubes
        obstacle_map: Optional[np.ndarray] = self.current_data.get('obstacle_map')
        if obstacle_map is not None:
            cubes: List[CubePrimitive] = []
            origin_x: float = self.current_data['map_origin_x']
            origin_y: float = self.current_data['map_origin_y']
            resolution: float = self.current_data['grid_resolution']
            
            height: int
            width: int
            height, width = obstacle_map.shape
            
            # Sample obstacles for performance (max 500 cubes)
            obstacle_positions: List[Tuple[float, float]] = []
            for y in range(height):
                for x in range(width):
                    if obstacle_map[y, x] == 1:
                        world_x: float = origin_x + x * resolution + resolution/2
                        world_y: float = origin_y + y * resolution + resolution/2
                        obstacle_positions.append((world_x, world_y))
            
            if len(obstacle_positions) > 500:
                step: int = len(obstacle_positions) // 500
                obstacle_positions = obstacle_positions[::step]
            
            for world_x, world_y in obstacle_positions:
                cubes.append(CubePrimitive(
                    pose=Pose(
                        position=Vector3(x=world_x, y=world_y, z=resolution/2),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                    ),
                    size=Vector3(x=resolution, y=resolution, z=resolution),
                    color=Color(r=0.2, g=0.2, b=0.2, a=0.8)  # Dark gray
                ))
            
            if cubes:
                obstacle_entity = SceneEntity(
                    id="obstacles",
                    cubes=cubes
                )
                entities.append(obstacle_entity)
        
        if not entities:
            return None
        
        return SceneUpdate(
            deletions=[],
            entities=entities
        )
    
    def create_start_goal_scene_update(self) -> Optional[SceneUpdate]:
        """Create a SceneUpdate with start and goal position visualization primitives"""
        from foxglove.schemas import (
            SceneUpdate, SceneEntity, ArrowPrimitive, 
            SpherePrimitive, Color, Vector3, Pose, Quaternion
        )
        
        entities: List[SceneEntity] = []
        
        # Visualize start position
        start: Optional[State] = self.current_data.get('start')
        if start:
            start_yaw: float = start.yaw
            start_quat_z: float = math.sin(start_yaw / 2.0)
            start_quat_w: float = math.cos(start_yaw / 2.0)
            
            start_entity = SceneEntity(
                id="start_position",
                spheres=[SpherePrimitive(
                    pose=Pose(
                        position=Vector3(x=float(start.x), y=float(start.y), z=0.3),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                    ),
                    size=Vector3(x=0.8, y=0.8, z=0.8),
                    color=Color(r=0.0, g=1.0, b=0.0, a=1.0)  # Green
                )],
                arrows=[ArrowPrimitive(
                    pose=Pose(
                        position=Vector3(x=float(start.x), y=float(start.y), z=0.3),
                        orientation=Quaternion(x=0.0, y=0.0, z=start_quat_z, w=start_quat_w)
                    ),
                    shaft_length=1.2,
                    shaft_diameter=0.15,
                    head_length=0.3,
                    head_diameter=0.3,
                    color=Color(r=0.0, g=0.8, b=0.0, a=0.9)
                )]
            )
            entities.append(start_entity)
        
        # Visualize goal position
        goal: Optional[State] = self.current_data.get('goal')
        if goal:
            goal_yaw: float = goal.yaw
            goal_quat_z: float = math.sin(goal_yaw / 2.0)
            goal_quat_w: float = math.cos(goal_yaw / 2.0)
            
            goal_entity = SceneEntity(
                id="goal_position",
                spheres=[SpherePrimitive(
                    pose=Pose(
                        position=Vector3(x=float(goal.x), y=float(goal.y), z=0.3),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                    ),
                    size=Vector3(x=0.8, y=0.8, z=0.8),
                    color=Color(r=1.0, g=0.0, b=0.0, a=1.0)  # Red
                )],
                arrows=[ArrowPrimitive(
                    pose=Pose(
                        position=Vector3(x=float(goal.x), y=float(goal.y), z=0.3),
                        orientation=Quaternion(x=0.0, y=0.0, z=goal_quat_z, w=goal_quat_w)
                    ),
                    shaft_length=1.2,
                    shaft_diameter=0.15,
                    head_length=0.3,
                    head_diameter=0.3,
                    color=Color(r=0.8, g=0.0, b=0.0, a=0.9)
                )]
            )
            entities.append(goal_entity)
        
        if not entities:
            return None
        
        return SceneUpdate(
            deletions=[],
            entities=entities
        )
    
    def create_path_scene_update(self) -> Optional[SceneUpdate]:
        """Create a SceneUpdate with path visualization primitives"""
        from foxglove.schemas import (
            SceneUpdate, SceneEntity, LinePrimitive, ArrowPrimitive, 
            Color, Point3, Vector3, Pose, Quaternion
        )

        entities: List[SceneEntity] = []

        # Visualize final path as thick lines
        path: List[State] = self.current_data.get('path', [])
        if len(path) > 1:
            path_lines: List[Point3] = []
            for state in path:
                path_lines.append(Point3(x=float(state.x), y=float(state.y), z=0.1))

            path_alpha = self.settings.get('path_alpha', 1.0)
            path_entity = SceneEntity(
                id="final_path",
                lines=[LinePrimitive(
                    pose=Pose(
                        position=Vector3(x=0.0, y=0.0, z=0.0),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                    ),
                    thickness=self.settings['path_line_thickness'],
                    scale_invariant=False,
                    points=path_lines,
                    color=Color(r=1.0, g=0.0, b=1.0, a=path_alpha)  # Magenta with configurable alpha
                )]
            )
            entities.append(path_entity)

            # Vehicle orientation arrows along path (configurable)
            if self.settings.get('show_final_path_arrows', True):
                arrows: List[ArrowPrimitive] = []
                step: int = max(1, len(path) // 20)  # Show ~20 arrows
                for i in range(0, len(path), step):
                    state: State = path[i]
                    # Calculate quaternion from yaw angle
                    yaw: float = state.yaw
                    quat_z: float = math.sin(yaw / 2.0)
                    quat_w: float = math.cos(yaw / 2.0)

                    arrows.append(ArrowPrimitive(
                        pose=Pose(
                            position=Vector3(x=float(state.x), y=float(state.y), z=0.2),
                            orientation=Quaternion(x=0.0, y=0.0, z=quat_z, w=quat_w)
                        ),
                        shaft_length=0.8,
                        shaft_diameter=0.1,
                        head_length=0.2,
                        head_diameter=0.2,
                        color=Color(r=0.0, g=0.0, b=1.0, a=0.8)  # Blue
                    ))

                if arrows:
                    arrow_entity = SceneEntity(
                        id="path_arrows",
                        arrows=arrows
                    )
                    entities.append(arrow_entity)

        if not entities:
            return None

        return SceneUpdate(
            deletions=[],
            entities=entities
        )
    
    def create_exploration_scene_update(self) -> Optional[SceneUpdate]:
        """Create a SceneUpdate with exploration nodes visualization primitives"""
        from foxglove.schemas import (
            SceneUpdate, SceneEntity, LinePrimitive, SpherePrimitive, 
            Color, Point3, Vector3, Pose, Quaternion
        )
        
        entities: List[SceneEntity] = []
        
        # Visualize exploration nodes as spheres and their simulation trajectories as lines
        explored_nodes: List[Node] = self.current_data.get('explored_nodes', [])
        if explored_nodes:
            # Limit nodes for performance
            max_nodes: int = int(self.settings['max_exploration_nodes'])
            if len(explored_nodes) > max_nodes:
                step = len(explored_nodes) // max_nodes
                explored_nodes = explored_nodes[::step]
            
            # Visualize exploration nodes as spheres
            spheres: List[SpherePrimitive] = []
            for node in explored_nodes:
                # Color based on f_cost (normalized) for better visualization insight
                # Use a normalized cost color from blue (low cost) to red (high cost)
                f_costs = [n.f_cost for n in explored_nodes]
                if f_costs:
                    min_cost = min(f_costs)
                    max_cost = max(f_costs)
                    if max_cost > min_cost:
                        normalized_cost = (node.f_cost - min_cost) / (max_cost - min_cost)
                    else:
                        normalized_cost = 0.5
                else:
                    normalized_cost = 0.5
                
                # Color gradient: blue (low cost) to red (high cost)
                sphere_color = Color(
                    r=float(normalized_cost), 
                    g=0.0, 
                    b=float(1.0 - normalized_cost), 
                    a=0.7
                )
                
                spheres.append(SpherePrimitive(
                    pose=Pose(
                        position=Vector3(x=float(node.state.x), y=float(node.state.y), z=0.1),
                        orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                    ),
                    size=Vector3(
                        x=self.settings['exploration_sphere_size'], 
                        y=self.settings['exploration_sphere_size'], 
                        z=self.settings['exploration_sphere_size']
                    ),  # Configurable sphere size
                    color=sphere_color
                ))
            
            if spheres:
                exploration_nodes_entity = SceneEntity(
                    id="exploration_nodes",
                    spheres=spheres
                )
                entities.append(exploration_nodes_entity)
            
            # Visualize simulation trajectories as lines
            trajectory_line_primitives: List[LinePrimitive] = []
            
            for node in explored_nodes:
                if hasattr(node, 'trajectory_states') and node.trajectory_states and len(node.trajectory_states) > 1:
                    # Create separate line primitive for each trajectory to avoid unwanted connections
                    trajectory_points: List[Point3] = []
                    trajectory_colors: List[Color] = []
                    
                    for state in node.trajectory_states:
                        trajectory_points.append(Point3(x=float(state.x), y=float(state.y), z=0.05))
                        # Color based on direction (green for forward, red for backward)
                        if hasattr(state, 'direction') and state.direction == DirectionMode.BACKWARD:
                            traj_color = Color(r=1.0, g=0.0, b=0.0, a=0.4)  # Red for backward
                        else:
                            traj_color = Color(r=0.0, g=1.0, b=0.0, a=0.4)  # Green for forward
                        trajectory_colors.append(traj_color)
                    
                    if trajectory_points:
                        # Create individual LinePrimitive for this trajectory
                        trajectory_line_primitives.append(LinePrimitive(
                            pose=Pose(
                                position=Vector3(x=0.0, y=0.0, z=0.0),
                                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                            ),
                            thickness=self.settings['exploration_line_thickness'],
                            scale_invariant=False,
                            points=trajectory_points,
                            colors=trajectory_colors
                        ))
            
            if trajectory_line_primitives:
                simulation_trajectories_entity = SceneEntity(
                    id="simulation_trajectories",
                    lines=trajectory_line_primitives
                )
                entities.append(simulation_trajectories_entity)
        
        if not entities:
            return None
        
        return SceneUpdate(
            deletions=[],
            entities=entities
        )
    
    def create_statistics(self) -> Optional[Dict[str, Any]]:
        """Create statistics data"""
        path = self.current_data.get('path', [])
        explored_nodes = self.current_data.get('explored_nodes', [])
        
        # Handle None values
        if path is None:
            path = []
        if explored_nodes is None:
            explored_nodes = []
        
        stats = {
            'timestamp': time.time(),
            'path_length': len(path),
            'nodes_explored': len(explored_nodes),
            'total_distance': 0.0,
            'max_steering_angle': 0.0,
            'avg_steering_angle': 0.0,
            'direction_changes': 0
        }
        
        if len(path) > 1:
            # Calculate total distance
            total_distance = sum(
                math.sqrt((path[i+1].x - path[i].x)**2 + (path[i+1].y - path[i].y)**2)
                for i in range(len(path)-1)
            )
            stats['total_distance'] = float(total_distance)
            
            # Steering statistics
            steer_angles = [abs(getattr(state, 'steer', 0.0)) for state in path]
            if steer_angles:
                stats['max_steering_angle'] = float(max(steer_angles))
                stats['avg_steering_angle'] = float(sum(steer_angles) / len(steer_angles))
            
            # Direction changes
            direction_changes = 0
            for i in range(1, len(path)):
                if (hasattr(path[i], 'direction') and hasattr(path[i-1], 'direction') 
                    and path[i].direction != path[i-1].direction):
                    direction_changes += 1
            stats['direction_changes'] = direction_changes
        
        return stats
    
    def create_path_data(self) -> Optional[List[Dict[str, Any]]]:
        """Create path data with node structure information"""
        path: list[State] = self.current_data.get('path', [])
        path_nodes: list[Node] = self.current_data.get('path_nodes', [])
        
        if not path:
            return None

        path_data = []

        # If we have original path nodes, use their detailed information
        if path_nodes and len(path_nodes) == len(path):
            for i, (state, node) in enumerate(zip(path, path_nodes)):
                node_data = {
                    'index': i,
                    'timestamp': time.time(),
                    'state': {
                        'x': float(state.x),
                        'y': float(state.y),
                        'yaw': float(state.yaw),
                        'direction': state.direction.name if hasattr(state, 'direction') else 'NONE',
                        'steer': float(getattr(state, 'steer', 0.0))
                    },
                    'costs': {
                        'g_cost': float(node.g_cost),
                        'h_cost': float(node.h_cost),
                        'f_cost': float(node.f_cost),
                        'distance': float(node.costs.distance) if hasattr(node, 'costs') else 0.0,
                        'steer': float(node.costs.steer) if hasattr(node, 'costs') else 0.0,
                        'turn': float(node.costs.turn) if hasattr(node, 'costs') else 0.0,
                        'cusp': float(node.costs.cusp) if hasattr(node, 'costs') else 0.0
                    },
                    'trajectory_length': len(node.trajectory_states) if hasattr(node, 'trajectory_states') and node.trajectory_states else 0
                }
                
                # Add motion information relative to previous node
                if i > 0:
                    prev_state = path[i-1]
                    distance = math.sqrt((state.x - prev_state.x)**2 + (state.y - prev_state.y)**2)
                    
                    vehicle_model = self.current_data.get('vehicle_model')
                    if vehicle_model:
                        yaw_change = abs(vehicle_model.normalize_angle(state.yaw - prev_state.yaw))
                    else:
                        yaw_diff = state.yaw - prev_state.yaw
                        # Simple angle normalization
                        while yaw_diff > math.pi:
                            yaw_diff -= 2 * math.pi
                        while yaw_diff < -math.pi:
                            yaw_diff += 2 * math.pi
                        yaw_change = abs(yaw_diff)
                    
                    node_data['motion'] = {
                        'distance_from_prev': float(distance),
                        'yaw_change_from_prev': float(yaw_change),
                        'direction_change': state.direction != prev_state.direction if hasattr(state, 'direction') and hasattr(prev_state, 'direction') else False
                    }
                
                path_data.append(node_data)
        else:
            # Fallback: create simplified node data from states only
            for i, state in enumerate(path):
                node_data = {
                    'index': i,
                    'timestamp': time.time(),
                    'state': {
                        'x': float(state.x),
                        'y': float(state.y),
                        'yaw': float(state.yaw),
                        'direction': state.direction.name if hasattr(state, 'direction') else 'NONE',
                        'steer': float(getattr(state, 'steer', 0.0))
                    }
                }
                
                # Add motion information relative to previous state
                if i > 0:
                    prev_state = path[i-1]
                    distance = math.sqrt((state.x - prev_state.x)**2 + (state.y - prev_state.y)**2)
                    
                    vehicle_model = self.current_data.get('vehicle_model')
                    if vehicle_model:
                        yaw_change = abs(vehicle_model.normalize_angle(state.yaw - prev_state.yaw))
                    else:
                        yaw_diff = state.yaw - prev_state.yaw
                        # Simple angle normalization
                        while yaw_diff > math.pi:
                            yaw_diff -= 2 * math.pi
                        while yaw_diff < -math.pi:
                            yaw_diff += 2 * math.pi
                        yaw_change = abs(yaw_diff)
                    
                    node_data['motion'] = {
                        'distance_from_prev': float(distance),
                        'yaw_change_from_prev': float(yaw_change),
                        'direction_change': state.direction != prev_state.direction if hasattr(state, 'direction') and hasattr(prev_state, 'direction') else False
                    }
                
                path_data.append(node_data)
        
        return path_data
    
    def visualize_live_planning(self, planner: HybridAStar, start: State, goal: State) -> Optional[List[Node]]:
        """
        Visualize live path planning process
        
        Args:
            planner: Hybrid A* planner instance
            start: Start state
            goal: Goal state
        """
        print("Starting live path planning visualization...")
        
        # Start server if not running
        if hasattr(self, 'scene_channel') and self.scene_channel is None:
            self.start_server()
        
        # Store planner reference
        self.planner = planner
        
        # Run planning
        result = planner.plan_path(start, goal)
        
        if result:
            # Convert path nodes to path states
            path_states = [node.state for node in result]
            detailed_path = planner.extract_detailed_path(result)
            
            # Get visualization data from planner
            viz_data = planner.get_visualization_data()
            
            # Update visualization with final result, including original nodes
            self.visualize_path_planning(
                path=detailed_path,
                start=start,
                goal=goal,
                explored_nodes=viz_data.get('explored_nodes', []),
                simulation_trajectories=viz_data.get('simulation_trajectories', []),
                obstacle_map=viz_data.get('obstacle_map'),
                map_origin_x=viz_data.get('map_origin_x', 0),
                map_origin_y=viz_data.get('map_origin_y', 0),
                grid_resolution=viz_data.get('grid_resolution', 1.0),
                vehicle_model=viz_data.get('vehicle_model'),
                path_nodes=result  # Pass original node objects for detailed data
            )
            print(f"✓ Path visualization updated with {len(path_states)} waypoints")
        else:
            print("✗ No path found")
        
        return result

    def start_mcap_only_recording(self, output_path: Optional[str] = None) -> None:
        """
        Start MCAP recording without WebSocket server
        
        Args:
            output_path: Optional custom output path. If None, uses default log directory
        """
        if output_path:
            self.mcap_output_path = output_path
        
        if self.mcap_output_path:
            print(f"Starting MCAP recording: {self.mcap_output_path}")
            self.mcap_sink = open_mcap(self.mcap_output_path)  # Returns MCAPWriter
        else:
            raise ValueError("No MCAP output path specified")


async def run_mcap_only_example() -> None:
    """Example of MCAP-only recording using default log directory"""
    if not FOXGLOVE_AVAILABLE:
        print("Foxglove SDK not available. Please install it first.")
        return
    
    print("Creating MCAP-only recording example...")
    
    # Create vehicle model
    vehicle = VehicleModel(wheelbase=2.5, max_steer=np.pi/4)
    
    # Create planner
    planner = HybridAStar(
        vehicle_model=vehicle,
        grid_resolution=0.5,
        angle_resolution=np.pi/8,
        velocity=2.0,
        simulation_time=0.5,
        dt=0.1
    )
    
    # Create simple obstacle map
    map_size = 50
    obstacle_map = np.zeros((map_size, map_size))
    obstacle_map[20:30, 15:25] = 1  # Rectangle obstacle
    obstacle_map[10:15, 35:45] = 1  # Another obstacle
    
    planner.set_obstacle_map(obstacle_map, origin_x=-10, origin_y=-10)
    
    # Define start and goal
    start = State(x=-5.0, y=-5.0, yaw=np.pi/4, direction=DirectionMode.NONE)
    goal = State(x=15.0, y=15.0, yaw=np.pi/2, direction=DirectionMode.FORWARD)
    
    # Create visualizer with default MCAP recording to log directory
    visualizer = FoxgloveHybridAStarVisualizer()
    
    try:
        # Start MCAP recording only (no WebSocket server)
        visualizer.start_mcap_only_recording()
        
        # Run planning and record results
        print("Running path planning and recording to MCAP...")
        result = planner.plan_path(start, goal)
        
        if result:
            # Convert path nodes to path states
            path_states = [node.state for node in result]
            
            # Get visualization data from planner
            viz_data = planner.get_visualization_data()
            
            # Record visualization data to MCAP
            visualizer.visualize_path_planning(
                path=path_states,
                start=start,
                goal=goal,
                explored_nodes=viz_data.get('explored_nodes', []),
                simulation_trajectories=viz_data.get('simulation_trajectories', []),
                obstacle_map=viz_data.get('obstacle_map'),
                map_origin_x=viz_data.get('map_origin_x', 0),
                map_origin_y=viz_data.get('map_origin_y', 0),
                grid_resolution=viz_data.get('grid_resolution', 1.0),
                vehicle_model=viz_data.get('vehicle_model'),
                path_nodes=result  # Pass original node objects for detailed data
            )
            print(f"✓ Path recorded with {len(path_states)} waypoints")
        else:
            print("✗ No path found")
            
    finally:
        visualizer.stop_server()


async def run_with_default_mcap() -> None:
    """Example usage with default MCAP recording to log directory"""
    if not FOXGLOVE_AVAILABLE:
        print("Foxglove SDK not available. Please install it first.")
        return
    
    print("Creating Foxglove Hybrid A* Visualizer with default MCAP recording...")
    
    # Create vehicle model
    vehicle = VehicleModel(wheelbase=2.5, max_steer=np.pi/4)
    
    # Create planner
    planner = HybridAStar(
        vehicle_model=vehicle,
        grid_resolution=0.5,
        angle_resolution=np.pi/8,
        velocity=2.0,
        simulation_time=0.5,
        dt=0.1
    )
    
    # Create simple obstacle map
    map_size = 50
    obstacle_map = np.zeros((map_size, map_size))
    obstacle_map[20:30, 15:25] = 1  # Rectangle obstacle
    obstacle_map[10:15, 35:45] = 1  # Another obstacle
    
    planner.set_obstacle_map(obstacle_map, origin_x=-10, origin_y=-10)
    
    # Define start and goal
    start = State(x=-5.0, y=-5.0, yaw=np.pi/4, direction=DirectionMode.NONE)
    goal = State(x=15.0, y=15.0, yaw=np.pi/2, direction=DirectionMode.FORWARD)
    
    # Create visualizer with default MCAP recording to log directory
    visualizer = FoxgloveHybridAStarVisualizer(port=8760)
    
    try:
        # Plan path with visualization and recording
        print("Planning path with visualization and MCAP recording...")
        path = visualizer.visualize_live_planning(planner, start, goal)
        
        if path:
            print(f"✓ Path found with {len(path)} waypoints")
        else:
            print("✗ No path found")
        
        # Keep server running for viewing while also recording to MCAP
        print("Visualization server running and recording to MCAP...")
        print("Connect Foxglove Studio to the displayed WebSocket URL")
        print("Press Ctrl+C to stop...")
        
        while True:
            await asyncio.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\nStopping visualizer...")
    finally:
        visualizer.stop_server()


async def run_example() -> None:
    """Example usage of the Foxglove visualizer"""
    if not FOXGLOVE_AVAILABLE:
        print("Foxglove SDK not available. Please install it first.")
        return
    
    print("Creating Foxglove Hybrid A* Visualizer example...")
    
    # Create vehicle model
    vehicle = VehicleModel(wheelbase=2.5, max_steer=np.pi/4)
    
    # Create planner
    planner = HybridAStar(
        vehicle_model=vehicle,
        grid_resolution=0.5,
        angle_resolution=np.pi/8,
        velocity=2.0,
        simulation_time=0.5,
        dt=0.1
    )
    
    # Create simple obstacle map
    map_size = 50
    obstacle_map = np.zeros((map_size, map_size))
    obstacle_map[20:30, 15:25] = 1  # Rectangle obstacle
    obstacle_map[10:15, 35:45] = 1  # Another obstacle
    
    planner.set_obstacle_map(obstacle_map, origin_x=-10, origin_y=-10)
    
    # Define start and goal
    start = State(x=-5.0, y=-5.0, yaw=np.pi/4, direction=DirectionMode.NONE)
    goal = State(x=15.0, y=15.0, yaw=np.pi/2, direction=DirectionMode.FORWARD)
    
    # Create visualizer
    visualizer = FoxgloveHybridAStarVisualizer(port=8765)
    
    try:
        # Plan path with visualization
        print("Planning path with visualization...")
        path = visualizer.visualize_live_planning(planner, start, goal)
        
        if path:
            print(f"✓ Path found with {len(path)} waypoints")
        else:
            print("✗ No path found")
        
        # Keep server running for viewing
        print("Visualization server running. Connect Foxglove Studio to the displayed WebSocket URL")
        print("Press Ctrl+C to stop...")
        
        while True:
            await asyncio.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\nStopping visualizer...")
    finally:
        visualizer.stop_server()


def matplotlib_fallback_visualization(path: List[State], 
                                    start: State, 
                                    goal: State,
                                    explored_nodes: Optional[List[Node]] = None,
                                    obstacle_map: Optional[np.ndarray] = None,
                                    map_origin_x: float = 0,
                                    map_origin_y: float = 0,
                                    grid_resolution: float = 1.0) -> None:
    """
    Fallback visualization using matplotlib when Foxglove is not available
    """
    try:
        import matplotlib.pyplot as plt
        import matplotlib.patches as patches
    except ImportError:
        print("Neither Foxglove nor matplotlib available for visualization")
        return
    
    fig, ax = plt.subplots(1, 1, figsize=(12, 10))
    
    # Plot obstacle map
    if obstacle_map is not None:
        height, width = obstacle_map.shape
        for y in range(height):
            for x in range(width):
                if obstacle_map[y, x] == 1:
                    world_x = map_origin_x + x * grid_resolution
                    world_y = map_origin_y + y * grid_resolution
                    rect = patches.Rectangle(
                        (world_x, world_y), grid_resolution, grid_resolution,
                        linewidth=0, edgecolor='none', facecolor='black', alpha=0.8
                    )
                    ax.add_patch(rect)
    
    # Plot exploration nodes as circles and their simulation trajectories
    if explored_nodes:
        # Plot simulation trajectories as lines
        for node in explored_nodes:
            if hasattr(node, 'trajectory_states') and node.trajectory_states and len(node.trajectory_states) > 1:
                traj_x = [s.x for s in node.trajectory_states]
                traj_y = [s.y for s in node.trajectory_states]
                color = 'green' if hasattr(node.trajectory_states[0], 'direction') and node.trajectory_states[0].direction == DirectionMode.FORWARD else 'red'
                ax.plot(traj_x, traj_y, color=color, alpha=0.3, linewidth=0.8, linestyle='-')
        
        # Plot exploration nodes as circles (colored by cost)
        node_x = [node.state.x for node in explored_nodes]
        node_y = [node.state.y for node in explored_nodes]
        
        # Color by f_cost for better insight
        f_costs = [node.f_cost for node in explored_nodes]
        if f_costs:
            ax.scatter(node_x, node_y, c=f_costs, cmap='coolwarm', s=30, alpha=0.7, 
                      edgecolors='black', linewidths=0.5, zorder=3,
                      label=f'Exploration Nodes ({len(explored_nodes)})')
        else:
            ax.scatter(node_x, node_y, c='lightblue', s=30, alpha=0.7, 
                      edgecolors='black', linewidths=0.5, zorder=3,
                      label=f'Exploration Nodes ({len(explored_nodes)})')
    
    # Plot final path
    if len(path) > 1:
        path_x = [state.x for state in path]
        path_y = [state.y for state in path]
        ax.plot(path_x, path_y, 'magenta', linewidth=3, label='Final Path')
        
        # Plot vehicle orientations
        step = max(1, len(path) // 20)
        for i in range(0, len(path), step):
            state = path[i]
            dx = 0.8 * math.cos(state.yaw)
            dy = 0.8 * math.sin(state.yaw)
            ax.arrow(state.x, state.y, dx, dy, 
                    head_width=0.3, head_length=0.2, fc='blue', ec='blue', alpha=0.7)
    
    # Plot start and goal
    ax.plot(start.x, start.y, 'go', markersize=10, label='Start')
    ax.plot(goal.x, goal.y, 'ro', markersize=10, label='Goal')
    
    # Start and goal orientation arrows
    for state, color, label in [(start, 'green', 'Start'), (goal, 'red', 'Goal')]:
        dx = 1.2 * math.cos(state.yaw)
        dy = 1.2 * math.sin(state.yaw)
        ax.arrow(state.x, state.y, dx, dy, 
                head_width=0.4, head_length=0.3, fc=color, ec=color, alpha=0.8)
    
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.legend()
    ax.set_title('Hybrid A* Path Planning Visualization')
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    
    plt.tight_layout()
    plt.show()


async def run_foxglove_example() -> None:
    """Example usage of the Foxglove visualizer"""
    if not FOXGLOVE_AVAILABLE:
        print("Foxglove SDK not available. Please install it first.")
        return
    
    print("Creating Foxglove Hybrid A* Visualizer example...")
    
    # Create vehicle model
    vehicle = VehicleModel(wheelbase=2.5, max_steer=np.pi/4)
    
    # Create planner
    planner = HybridAStar(
        vehicle_model=vehicle,
        grid_resolution=0.5,
        angle_resolution=np.pi/8,
        velocity=2.0,
        simulation_time=0.5,
        dt=0.1
    )
    
    # Create simple obstacle map
    map_size = 50
    obstacle_map = np.zeros((map_size, map_size))
    obstacle_map[20:30, 15:25] = 1  # Rectangle obstacle
    obstacle_map[10:15, 35:45] = 1  # Another obstacle
    
    planner.set_obstacle_map(obstacle_map, origin_x=-10, origin_y=-10)
    
    # Define start and goal
    start = State(x=-5.0, y=-5.0, yaw=np.pi/4, direction=DirectionMode.NONE)
    goal = State(x=15.0, y=15.0, yaw=np.pi/2, direction=DirectionMode.FORWARD)
    
    # Create visualizer
    visualizer = FoxgloveHybridAStarVisualizer(port=8760)
    
    try:
        # Plan path with visualization
        print("Planning path with visualization...")
        path = visualizer.visualize_live_planning(planner, start, goal)
        
        if path:
            print(f"✓ Path found with {len(path)} waypoints")
        else:
            print("✗ No path found")
        
        # Keep server running for viewing
        print("Visualization server running. Connect Foxglove Studio to the displayed WebSocket URL")
        print("Press Ctrl+C to stop...")
        
        while True:
            await asyncio.sleep(1.0)
            
    except KeyboardInterrupt:
        print("\nStopping visualizer...")
    finally:
        visualizer.stop_server()


if __name__ == "__main__":
    import argparse

    asyncio.run(run_foxglove_example())
