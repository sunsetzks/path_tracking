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
    
    This class creates visualization data for Foxglove Studio using 3D primitives and JSON data.
    Based on the official Foxglove SDK examples.
    """
    
    def __init__(self, port: int = 8765) -> None:
        """
        Initialize the Foxglove visualizer
        
        Args:
            port: WebSocket server port
        """
        if not FOXGLOVE_AVAILABLE:
            raise ImportError("Foxglove SDK is not available. Please install it first.")
            
        self.port: int = port
        
        # Visualization settings
        self.settings: Dict[str, float] = {
            'exploration_line_thickness': 0.05,
            'path_line_thickness': 0.15,
            'max_exploration_nodes': 1000,  # Limit for performance
        }
        
        # Current data
        self.current_data: Dict[str, Any] = {}
        self.is_running: bool = False
        
        # Channels (created when server starts)
        self.scene_channel: Optional[Any] = None
        self.stats_channel: Optional[Any] = None
    
    def start_server(self) -> Any:
        """Start the Foxglove WebSocket server with channels"""
        if not FOXGLOVE_AVAILABLE:
            raise ImportError("Foxglove SDK is not available")
            
        print(f"Starting Foxglove server on ws://localhost:{self.port}")
        
        # Import required classes
        from foxglove import start_server, Channel
        
        # Start the WebSocket server (global context)
        server = start_server(port=self.port)
        
        # Create channels for 3D visualization
        self.scene_channel = Channel(topic="/hybrid_astar/scene")
        self.stats_channel = Channel(topic="/hybrid_astar/statistics")
        
        print(f"✓ Foxglove server started on ws://localhost:{self.port}")
        print(f"→ Connect Foxglove Studio to this WebSocket URL")
        print(f"→ Add a 3D panel and subscribe to '/hybrid_astar/scene' topic")
        print(f"→ Add a Plot panel for '/hybrid_astar/statistics' topic")
        
        self.is_running = True
        return server
    
    def stop_server(self) -> None:
        """Stop the server (note: Foxglove server runs globally)"""
        self.is_running = False
        print("Foxglove visualizer stopped")
    
    def log_scene_update(self, scene_update: SceneUpdate) -> None:
        """Log a SceneUpdate to Foxglove"""
        if self.scene_channel and self.is_running:
            self.scene_channel.log(scene_update)
    
    def log_statistics(self, stats_dict: Dict[str, Any]) -> None:
        """Log statistics as JSON"""
        if self.stats_channel and self.is_running:
            self.stats_channel.log(stats_dict)
    
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
        Visualize complete path planning results with 3D primitives
        
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
        
        # Create and send 3D scene update
        scene_update = self.create_scene_update()
        if scene_update:
            self.log_scene_update(scene_update)
        
        # Create and send statistics
        stats = self.create_statistics()
        if stats:
            self.log_statistics(stats)
    
    def create_scene_update(self) -> Optional[SceneUpdate]:
        """Create a Foxglove SceneUpdate with 3D visualization primitives"""
        from foxglove.schemas import (
            SceneUpdate, SceneEntity, LinePrimitive, ArrowPrimitive, 
            CubePrimitive, SpherePrimitive, Color, Point3, Vector3, Pose, Quaternion
        )
        
        entities: List[SceneEntity] = []
        
        # 1. Visualize exploration tree as lines
        explored_nodes: List[Node] = self.current_data.get('explored_nodes', [])
        if explored_nodes:
            lines: List[Point3] = []
            colors: List[Color] = []
            
            # Limit nodes for performance
            max_nodes: int = int(self.settings['max_exploration_nodes'])
            if len(explored_nodes) > max_nodes:
                step = len(explored_nodes) // max_nodes
                explored_nodes = explored_nodes[::step]
            
            for node in explored_nodes:
                if node.parent is not None:
                    # Create line from parent to current node
                    lines.extend([
                        Point3(x=float(node.parent.state.x), y=float(node.parent.state.y), z=0.0),
                        Point3(x=float(node.state.x), y=float(node.state.y), z=0.0)
                    ])
                    
                    # Color based on direction (green for forward, red for backward)
                    if hasattr(node.state, 'direction') and str(node.state.direction) == 'BACKWARD':
                        color = Color(r=1.0, g=0.0, b=0.0, a=0.3)  # Red for backward
                    else:
                        color = Color(r=0.0, g=1.0, b=0.0, a=0.3)  # Green for forward
                    colors.extend([color, color])
            
            if lines:
                exploration_entity = SceneEntity(
                    id="exploration_tree",
                    lines=[LinePrimitive(
                        pose=Pose(
                            position=Vector3(x=0.0, y=0.0, z=0.0),
                            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
                        ),
                        thickness=self.settings['exploration_line_thickness'],
                        scale_invariant=False,
                        points=lines,
                        colors=colors
                    )]
                )
                entities.append(exploration_entity)
        
        # 2. Visualize final path as thick lines
        path: List[State] = self.current_data.get('path', [])
        if len(path) > 1:
            path_lines: List[Point3] = []
            for i in range(len(path) - 1):
                path_lines.extend([
                    Point3(x=float(path[i].x), y=float(path[i].y), z=0.1),
                    Point3(x=float(path[i+1].x), y=float(path[i+1].y), z=0.1)
                ])
            
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
                    color=Color(r=1.0, g=0.0, b=1.0, a=1.0)  # Magenta
                )]
            )
            entities.append(path_entity)
            
            # 3. Vehicle orientation arrows along path
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
        
        # 4. Visualize start and goal positions
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
        
        # 5. Visualize obstacles as cubes
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
    
    def create_statistics(self) -> Optional[Dict[str, Any]]:
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
            self.visualize_path_planning(
                path=path_states,
                start=start,
                goal=goal,
                explored_nodes=planner.explored_nodes if hasattr(planner, 'explored_nodes') else [],
                obstacle_map=planner.obstacle_map if hasattr(planner, 'obstacle_map') else None,
                map_origin_x=planner.map_origin_x if hasattr(planner, 'map_origin_x') else 0,
                map_origin_y=planner.map_origin_y if hasattr(planner, 'map_origin_y') else 0,
                grid_resolution=planner.grid_resolution if hasattr(planner, 'grid_resolution') else 1.0
            )
            print(f"✓ Path visualization updated with {len(path_states)} waypoints")
        else:
            print("✗ No path found")
        
        return result


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
    if FOXGLOVE_AVAILABLE:
        asyncio.run(run_foxglove_example())
    else:
        print("Please install Foxglove SDK: pip install foxglove-sdk")
    if FOXGLOVE_AVAILABLE:
        asyncio.run(run_foxglove_example())
    else:
        print("Please install Foxglove SDK: pip install foxglove-sdk")
