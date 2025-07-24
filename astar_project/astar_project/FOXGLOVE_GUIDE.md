"""
Installation and Usage Guide for Foxglove Visualizer

This guide explains how to install and use the Foxglove-based visualization system
for the Hybrid A* path planning algorithm.
"""

# INSTALLATION GUIDE

## 1. Install Foxglove SDK and Dependencies

# Install Foxglove SDK for Python
pip install foxglove-sdk

# Install Foxglove Protobuf schemas
pip install foxglove-schemas-protobuf

# Install additional dependencies
pip install numpy asyncio

## 2. Install Foxglove Studio (Optional but Recommended)

# Download Foxglove Studio from:
# https://foxglove.dev/download

# Or use the web version at:
# https://studio.foxglove.dev/

## 3. Usage Examples

### Basic Usage with Static Data

```python
import asyncio
from astar_project.foxglove_visualizer import FoxgloveHybridAStarVisualizer
from astar_project.hybrid_astar import VehicleModel, HybridAStar, State, DirectionMode
import numpy as np

async def basic_example():
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
    
    # Create obstacle map
    map_size = 50
    obstacle_map = np.zeros((map_size, map_size))
    obstacle_map[20:30, 15:25] = 1  # Rectangle obstacle
    planner.set_obstacle_map(obstacle_map, origin_x=-10, origin_y=-10)
    
    # Define start and goal
    start = State(x=-5.0, y=-5.0, yaw=np.pi/4, direction=DirectionMode.NONE)
    goal = State(x=15.0, y=15.0, yaw=np.pi/2, direction=DirectionMode.FORWARD)
    
    # Plan path
    path_nodes = planner.plan_path(start, goal)
    
    if path_nodes:
        # Extract states from nodes
        path_states = [node.state for node in path_nodes]
        
        # Create visualizer
        visualizer = FoxgloveHybridAStarVisualizer(port=8765)
        
        # Get visualization data
        viz_data = planner.get_visualization_data()
        
        # Visualize results
        visualizer.visualize_path_planning(
            path=path_states,
            start=start,
            goal=goal,
            **viz_data
        )
        
        print("Visualization server running on ws://localhost:8765")
        print("Connect Foxglove Studio to view the visualization")
        
        # Keep server running
        try:
            while True:
                await asyncio.sleep(1.0)
        except KeyboardInterrupt:
            await visualizer.stop_server()

# Run the example
asyncio.run(basic_example())
```

### Live Planning Visualization

```python
async def live_planning_example():
    # Setup (same as above)
    vehicle = VehicleModel(wheelbase=2.5, max_steer=np.pi/4)
    planner = HybridAStar(vehicle_model=vehicle)
    
    # Create visualizer
    visualizer = FoxgloveHybridAStarVisualizer(port=8765)
    
    # Define start and goal
    start = State(x=-5.0, y=-5.0, yaw=np.pi/4, direction=DirectionMode.NONE)
    goal = State(x=15.0, y=15.0, yaw=np.pi/2, direction=DirectionMode.FORWARD)
    
    # Run live planning with visualization
    path = await visualizer.visualize_live_planning(planner, start, goal)
    
    if path:
        print(f"Path found with {len(path)} waypoints")
    
    # Keep server running for viewing
    try:
        while True:
            await asyncio.sleep(1.0)
    except KeyboardInterrupt:
        await visualizer.stop_server()

asyncio.run(live_planning_example())
```

## 4. Connecting Foxglove Studio

1. Start your Python script with the visualizer
2. Open Foxglove Studio
3. Click "Open Connection"
4. Select "WebSocket"
5. Enter the URL: `ws://localhost:8765`
6. Click "Connect"

## 5. Visualization Features

The Foxglove visualizer provides the following features:

### Path Planning Visualization
- **Exploration Tree**: Green lines for forward exploration, red for backward
- **Final Path**: Magenta line showing the optimal path
- **Vehicle Arrows**: Blue arrows showing vehicle heading direction
- **Steering Arrows**: Orange arrows showing front wheel steering direction
- **Obstacle Map**: Black triangles representing obstacles
- **Start/Goal Markers**: Green arrow for start, red arrow for goal

### Real-time Statistics
- Path length (number of waypoints)
- Total distance traveled
- Maximum and average steering angles
- Number of direction changes
- Number of nodes explored

### Interactive Features
- 3D visualization with zoom, pan, and rotate
- Layer visibility controls
- Real-time updates during planning
- Statistics panel with live metrics

## 6. Customization Options

### Visualization Settings
```python
visualizer = FoxgloveHybridAStarVisualizer(port=8765)

# Customize visualization settings
visualizer.settings.update({
    'exploration_line_thickness': 0.1,  # Thicker exploration lines
    'path_line_thickness': 0.2,         # Thicker path lines
    'arrow_scale': 1.5,                 # Larger arrows
    'max_exploration_nodes': 500,       # Fewer nodes for performance
    'update_rate': 5.0,                 # 5 Hz update rate
})
```

### Color Customization
```python
# Customize colors
visualizer.colors.update({
    'final_path': visualizer._create_color(0.0, 1.0, 1.0, 1.0),  # Cyan path
    'vehicle_heading': visualizer._create_color(1.0, 1.0, 0.0, 1.0),  # Yellow arrows
})
```

## 7. Performance Tips

1. **Limit Exploration Nodes**: Set `max_exploration_nodes` to reduce rendering load
2. **Reduce Update Rate**: Lower `update_rate` for better performance
3. **Simplify Obstacle Maps**: Use coarser resolution for large maps
4. **Network Optimization**: Use localhost for best performance

## 8. Troubleshooting

### Common Issues

**"Foxglove SDK not available"**
- Install with: `pip install foxglove-sdk foxglove-schemas-protobuf`

**"Connection refused" in Foxglove Studio**
- Check that your Python script is running
- Verify the port number matches (default: 8765)
- Try `ws://127.0.0.1:8765` instead of `localhost`

**Slow visualization**
- Reduce `max_exploration_nodes`
- Lower `update_rate`
- Simplify obstacle maps

**No data visible in Foxglove Studio**
- Check that channels are properly advertised
- Verify data is being sent (check console output)
- Try refreshing the connection

### Debug Mode
```python
# Enable debug output
import logging
logging.basicConfig(level=logging.DEBUG)

# The visualizer will print detailed information about data being sent
```

## 9. Advanced Features

### Custom Channels
```python
# Add custom visualization channels
await visualizer.server.advertise_channel(
    channel="custom_data",
    encoding="json",
    schema_name="",
)

# Send custom data
await visualizer.server.send_message(
    channel="custom_data",
    timestamp=time.time_ns(),
    data=json.dumps({"custom": "data"}).encode()
)
```

### Asset Server Integration
```python
# Serve 3D models and other assets
def asset_handler(uri: str) -> Optional[bytes]:
    if uri.startswith("package://models/"):
        # Load and return asset data
        return load_asset_file(uri)
    return None

visualizer = FoxgloveHybridAStarVisualizer(asset_handler=asset_handler)
```
