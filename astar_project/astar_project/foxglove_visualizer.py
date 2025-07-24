"""
Foxglove-based Visualization for Hybrid A* Path Planning

This module provides real-time visualization capabilities for the Hybrid A* algorithm
using the Foxglove WebSocket server. It creates interactive 3D visualizations that can
be viewed in Foxglove Studio.

Features:
- Real-time path planning visualization
- Interactive exploration tree display
- Vehicle state visualization with orientation arrows
- Cost analysis and statistics
- Obstacle map visualization
- Steering angle visualization with color coding

Author: Your Name
Date: 2025-07-24
"""

import asyncio
import json
import math
import time
from typing import List, Optional, Dict, Any, Tuple
import numpy as np

try:
    import foxglove
    from foxglove import Channel, Schema
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


"""
Foxglove-based Visualization for Hybrid A* Path Planning

This module provides real-time visualization capabilities for the Hybrid A* algorithm
using the Foxglove WebSocket server. It creates interactive 3D visualizations that can
be viewed in Foxglove Studio.

Features:
- Real-time path planning visualization
- Interactive exploration tree display
- Vehicle state visualization with orientation arrows
- Cost analysis and statistics
- Obstacle map visualization
- Steering angle visualization with color coding

Author: Your Name
Date: 2025-07-24
"""

import asyncio
import json
import math
import time
from typing import List, Optional, Dict, Any, Tuple
import numpy as np

try:
    import foxglove
    from foxglove import Channel, Schema
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
    
    This class creates a WebSocket server that streams visualization data to Foxglove Studio.
    It provides real-time updates of the path planning process, including exploration tree,
    final path, vehicle states, and cost analysis.
    """
    
    def __init__(self, port: int = 8765, host: str = "localhost"):
        """
        Initialize the Foxglove visualizer
        
        Args:
            port: WebSocket server port
            host: WebSocket server host
        """
        if not FOXGLOVE_AVAILABLE:
            raise ImportError("Foxglove SDK is not available. Please install it first.")
            
        self.port = port
        self.host = host
        self.server: Optional[WebSocketServer] = None
        
        # Channels for different visualization elements
        self.channels = {}
        
        # Visualization settings
        self.settings = {
            'exploration_line_thickness': 0.05,
            'path_line_thickness': 0.15,
            'arrow_scale': 1.0,
            'vehicle_scale': 2.5,  # Vehicle wheelbase
            'max_exploration_nodes': 1000,  # Limit for performance
            'update_rate': 10.0,  # Hz
        }
        
        # Current data
        self.current_data = {}
        self.is_running = False
        
        # Setup channels
        self._setup_channels()
    
    def _setup_channels(self):
        """Setup channels for different visualization types"""
        
        # Path visualization channel (JSON format for simplicity)
        path_schema = {
            "type": "object",
            "properties": {
                "path": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "x": {"type": "number"},
                            "y": {"type": "number"},
                            "yaw": {"type": "number"},
                            "steer": {"type": "number"}
                        }
                    }
                },
                "exploration_nodes": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "x": {"type": "number"},
                            "y": {"type": "number"},
                            "parent_x": {"type": "number"},
                            "parent_y": {"type": "number"},
                            "direction": {"type": "string"}
                        }
                    }
                },
                "start": {
                    "type": "object",
                    "properties": {
                        "x": {"type": "number"},
                        "y": {"type": "number"},
                        "yaw": {"type": "number"}
                    }
                },
                "goal": {
                    "type": "object", 
                    "properties": {
                        "x": {"type": "number"},
                        "y": {"type": "number"},
                        "yaw": {"type": "number"}
                    }
                },
                "obstacles": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "properties": {
                            "x": {"type": "number"},
                            "y": {"type": "number"},
                            "width": {"type": "number"},
                            "height": {"type": "number"}
                        }
                    }
                }
            }
        }
        
        # Statistics schema
        stats_schema = {
            "type": "object",
            "properties": {
                "timestamp": {"type": "number"},
                "path_length": {"type": "number"},
                "nodes_explored": {"type": "number"},
                "total_distance": {"type": "number"},
                "max_steering_angle": {"type": "number"},
                "avg_steering_angle": {"type": "number"},
                "direction_changes": {"type": "number"}
            }
        }
        
        self.path_channel = Channel(
            topic="/hybrid_astar/visualization",
            schema=path_schema
        )
        
        self.stats_channel = Channel(
            topic="/hybrid_astar/statistics",
            schema=stats_schema
        )
    
    def start_server(self):
        """Start the Foxglove WebSocket server"""
        if not FOXGLOVE_AVAILABLE:
            raise ImportError("Foxglove SDK is not available")
            
        print(f"Starting Foxglove server on {self.host}:{self.port}")
        
        self.server = foxglove.start_server(
            host=self.host,
            port=self.port,
            name="Hybrid A* Visualizer"
        )
        
        print(f"Foxglove server started on port {self.server.port}")
        print(f"Connect Foxglove Studio to ws://{self.host}:{self.server.port}")
        self.is_running = True
        
        return self.server
    
    def stop_server(self):
        """Stop the Foxglove WebSocket server"""
        if self.server:
            self.server.stop()
            self.is_running = False
            print("Foxglove server stopped")
    
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
                              vehicle_model: Optional[VehicleModel] = None) -> None:
        """
        Visualize complete path planning results
        
        Args:
            path: Final planned path
            start: Start state
            goal: Goal state
            explored_nodes: Nodes explored during search
            simulation_trajectories: Forward simulation trajectories
            obstacle_map: 2D obstacle map
            map_origin_x: Map origin X coordinate
            map_origin_y: Map origin Y coordinate
            grid_resolution: Grid resolution
            vehicle_model: Vehicle kinematic model
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
            'vehicle_model': vehicle_model
        }
        
        # Start server if not running
        if not self.is_running:
            self.start_server()
        
        # Update visualization
        self._update_visualization()
    
    def _update_visualization(self):
        """Update all visualization elements"""
        if not self.server or not self.is_running:
            print("Server not running")
            return
        
        # Create visualization data
        viz_data = self._create_visualization_data()
        
        # Send visualization data
        if viz_data:
            self.path_channel.log(viz_data)
        
        # Send statistics
        stats = self._create_statistics()
        if stats:
            self.stats_channel.log(stats)
    
    def _create_visualization_data(self) -> Optional[Dict[str, Any]]:
        """Create visualization data dictionary"""
        
        viz_data = {}
        
        # Path data
        path = self.current_data.get('path', [])
        if path:
            viz_data['path'] = [
                {
                    'x': float(state.x),
                    'y': float(state.y),
                    'yaw': float(state.yaw),
                    'steer': float(getattr(state, 'steer', 0.0))
                }
                for state in path
            ]
        
        # Exploration nodes
        explored_nodes = self.current_data.get('explored_nodes', [])
        if explored_nodes:
            exploration_data = []
            
            # Limit nodes for performance
            max_nodes = self.settings['max_exploration_nodes']
            if len(explored_nodes) > max_nodes:
                step = len(explored_nodes) // max_nodes
                explored_nodes = explored_nodes[::step]
            
            for node in explored_nodes:
                if node.parent is not None:
                    exploration_data.append({
                        'x': float(node.state.x),
                        'y': float(node.state.y),
                        'parent_x': float(node.parent.state.x),
                        'parent_y': float(node.parent.state.y),
                        'direction': str(node.state.direction.name) if hasattr(node.state.direction, 'name') else 'FORWARD'
                    })
            
            viz_data['exploration_nodes'] = exploration_data
        
        # Start and goal
        start = self.current_data.get('start')
        if start:
            viz_data['start'] = {
                'x': float(start.x),
                'y': float(start.y),
                'yaw': float(start.yaw)
            }
        
        goal = self.current_data.get('goal')
        if goal:
            viz_data['goal'] = {
                'x': float(goal.x),
                'y': float(goal.y),
                'yaw': float(goal.yaw)
            }
        
        # Obstacles
        obstacle_map = self.current_data.get('obstacle_map')
        if obstacle_map is not None:
            obstacles = []
            origin_x = self.current_data['map_origin_x']
            origin_y = self.current_data['map_origin_y']
            resolution = self.current_data['grid_resolution']
            
            height, width = obstacle_map.shape
            
            for y in range(height):
                for x in range(width):
                    if obstacle_map[y, x] == 1:  # Obstacle
                        world_x = origin_x + x * resolution
                        world_y = origin_y + y * resolution
                        
                        obstacles.append({
                            'x': float(world_x),
                            'y': float(world_y),
                            'width': float(resolution),
                            'height': float(resolution)
                        })
            
            # Limit obstacles for performance
            if len(obstacles) > 1000:
                step = len(obstacles) // 1000
                obstacles = obstacles[::step]
            
            viz_data['obstacles'] = obstacles
        
        return viz_data if viz_data else None
    
    def _create_statistics(self) -> Optional[Dict[str, Any]]:
        """Create statistics data"""
        path = self.current_data.get('path', [])
        explored_nodes = self.current_data.get('explored_nodes', [])
        
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
    
    def visualize_live_planning(self, planner: HybridAStar, start: State, goal: State):
        """
        Visualize live path planning process
        
        Args:
            planner: Hybrid A* planner instance
            start: Start state
            goal: Goal state
        """
        print("Starting live path planning visualization...")
        
        # Start server if not running
        if not self.is_running:
            self.start_server()
        
        # Store planner reference
        self.planner = planner
        
        # Run planning
        result = planner.plan_path(start, goal)
        
        if result:
            # Extract states from nodes
            path_states = [node.state for node in result]
            
            # Update visualization with final result
            viz_data = planner.get_visualization_data()
            self.current_data = {
                'path': path_states,
                'start': start,
                'goal': goal,
                **viz_data
            }
            
            self._update_visualization()
            print(f"✓ Path visualization updated with {len(path_states)} waypoints")
        else:
            print("✗ No path found")
        
        return result


async def run_example():
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
                                    grid_resolution: float = 1.0):
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
    
    # Plot exploration tree
    if explored_nodes:
        for node in explored_nodes:
            if node.parent is not None:
                color = 'green' if node.state.direction == DirectionMode.FORWARD else 'red'
                ax.plot([node.parent.state.x, node.state.x], 
                       [node.parent.state.y, node.state.y], 
                       color=color, alpha=0.3, linewidth=0.5)
    
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


async def run_foxglove_example():
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
    if FOXGLOVE_AVAILABLE:
        asyncio.run(run_foxglove_example())
    else:
        print("Please install Foxglove SDK: pip install foxglove-sdk")
    if FOXGLOVE_AVAILABLE:
        asyncio.run(run_foxglove_example())
    else:
        print("Please install Foxglove SDK: pip install foxglove-sdk")
