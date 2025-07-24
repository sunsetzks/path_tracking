# Foxglove Visualization for Hybrid A* Path Planning

This directory contains a new visualization system for the Hybrid A* path planning algorithm using [Foxglove Studio](https://foxglove.dev/). This provides real-time, interactive 3D visualization capabilities that can be viewed in a web browser or desktop application.

## Overview

The Foxglove-based visualization system offers several advantages over traditional matplotlib visualizations:

- **Real-time Updates**: Stream path planning data in real-time to Foxglove Studio
- **Interactive 3D View**: Pan, zoom, and rotate to explore the planning space
- **Multiple Visualization Layers**: Toggle different elements on/off
- **Professional Interface**: Clean, modern visualization interface
- **Network-based**: Can be viewed remotely or shared across devices
- **Extensible**: Easy to add custom visualization elements

## Files

### Core Files

- **`foxglove_visualizer.py`**: Main Foxglove visualization implementation
- **`foxglove_demo.py`**: Demo script with examples and fallback visualization
- **`FOXGLOVE_GUIDE.md`**: Detailed installation and usage guide

### Key Components

1. **FoxgloveHybridAStarVisualizer**: Main visualization class
2. **WebSocket Server**: Streams data to Foxglove Studio
3. **Visualization Channels**: Different data streams (path, exploration, arrows, etc.)
4. **Real-time Updates**: Live visualization during planning

## Quick Start

### 1. Installation

```bash
# Install Foxglove SDK
pip install foxglove-sdk foxglove-schemas-protobuf

# Install Foxglove Studio (optional)
# Download from: https://foxglove.dev/download
# Or use web version: https://studio.foxglove.dev/
```

### 2. Basic Usage

```python
import asyncio
from astar_project.foxglove_visualizer import FoxgloveHybridAStarVisualizer
from astar_project.hybrid_astar import *

async def example():
    # Create planner and plan path
    planner = HybridAStar(...)
    path = planner.plan_path(start, goal)
    
    # Create visualizer
    visualizer = FoxgloveHybridAStarVisualizer(port=8765)
    
    # Visualize results
    visualizer.visualize_path_planning(
        path=[node.state for node in path],
        start=start,
        goal=goal,
        **planner.get_visualization_data()
    )
    
    # Server runs at ws://localhost:8765

asyncio.run(example())
```

### 3. Run Demo

```bash
# Run the demo script
python -m astar_project.foxglove_demo

# Or run directly
cd astar_project
python foxglove_demo.py
```

### 4. Connect Foxglove Studio

1. Open Foxglove Studio
2. Click "Open Connection"
3. Select "WebSocket"
4. Enter URL: `ws://localhost:8765`
5. Click "Connect"

## Visualization Features

### Available Channels

The visualizer creates several channels that can be viewed in Foxglove Studio:

- **`exploration_tree`**: Search tree with parent-child connections
  - Green lines: Forward exploration
  - Red lines: Backward exploration

- **`final_path`**: Optimal path from start to goal
  - Magenta line with high visibility

- **`vehicle_arrows`**: Vehicle orientation indicators
  - Blue arrows showing vehicle heading direction

- **`steering_arrows`**: Steering angle visualization
  - Orange arrows at front wheels showing steering direction

- **`obstacle_map`**: Environment obstacles
  - Black triangles representing blocked areas

- **`start_goal`**: Start and goal markers
  - Green arrow: Start position and orientation
  - Red arrow: Goal position and orientation

- **`statistics`**: Real-time planning statistics (JSON)
  - Path length, distance, steering angles, etc.

### Interactive Features

- **3D Navigation**: Mouse controls for pan, zoom, rotate
- **Layer Control**: Toggle visibility of different elements
- **Time Scrubbing**: Review planning process over time
- **Statistics Panel**: Live metrics and performance data
- **Camera Control**: Multiple view angles and perspectives

### Real-time Capabilities

- **Live Planning**: Watch the search algorithm work in real-time
- **Progressive Updates**: See exploration tree grow during search
- **Dynamic Statistics**: Real-time cost and performance metrics
- **Interactive Parameters**: Modify planning parameters on-the-fly

## Comparison with Original Visualizer

| Feature | Original (matplotlib) | Foxglove |
|---------|----------------------|----------|
| Real-time updates | ❌ | ✅ |
| Interactive 3D | ❌ | ✅ |
| Network streaming | ❌ | ✅ |
| Professional UI | ❌ | ✅ |
| Layer control | ❌ | ✅ |
| Statistics panel | ❌ | ✅ |
| Extensibility | Limited | High |
| Setup complexity | Low | Medium |

## Performance Considerations

### Optimization Settings

The visualizer includes several performance optimization options:

```python
visualizer.settings.update({
    'max_exploration_nodes': 500,    # Limit nodes for performance
    'update_rate': 10.0,             # Hz - lower for better performance
    'exploration_line_thickness': 0.05,  # Thinner lines render faster
})
```

### Network Performance

- **Local Server**: Use `localhost` for best performance
- **Remote Viewing**: Can stream over network to remote Foxglove clients
- **Bandwidth**: Adjust update rate and node limits for network conditions

### Large-scale Planning

For large planning problems:

1. **Limit Exploration Nodes**: Set `max_exploration_nodes` to 200-500
2. **Reduce Update Rate**: Use 1-5 Hz for complex visualizations
3. **Simplify Geometry**: Use coarser obstacle maps
4. **Batch Updates**: Group multiple updates together

## Customization

### Colors and Styling

```python
# Customize visualization colors
visualizer.colors.update({
    'final_path': visualizer._create_color(0.0, 1.0, 1.0, 1.0),  # Cyan
    'vehicle_heading': visualizer._create_color(1.0, 1.0, 0.0, 1.0),  # Yellow
})
```

### Custom Channels

```python
# Add custom visualization elements
await visualizer.server.advertise_channel(
    channel="custom_markers",
    encoding="protobuf",
    schema_name="foxglove.ArrowPrimitive"
)
```

### Asset Integration

```python
# Serve 3D models and assets
def asset_handler(uri: str) -> Optional[bytes]:
    if uri.startswith("package://models/"):
        return load_3d_model(uri)
    return None

visualizer = FoxgloveHybridAStarVisualizer(asset_handler=asset_handler)
```

## Troubleshooting

### Common Issues

1. **Installation Problems**
   ```bash
   pip install --upgrade foxglove-sdk foxglove-schemas-protobuf
   ```

2. **Connection Issues**
   - Check server is running: `ws://localhost:8765`
   - Try different port if 8765 is in use
   - Verify firewall settings

3. **Performance Issues**
   - Reduce `max_exploration_nodes`
   - Lower `update_rate`
   - Use simpler obstacle maps

4. **No Data Visible**
   - Check channel names in Foxglove Studio
   - Verify data is being sent (console logs)
   - Try refreshing connection

### Debug Mode

```python
import logging
logging.basicConfig(level=logging.DEBUG)

# Enables detailed logging of data transmission
```

## Future Enhancements

Potential future improvements to the Foxglove visualization:

1. **Parameter Server**: Live parameter adjustment through Foxglove
2. **Service Integration**: Call planning services from Foxglove
3. **Recording/Playback**: Save and replay planning sessions
4. **Multi-robot Support**: Visualize multiple planning instances
5. **Cost Heatmaps**: Visual cost field representation
6. **Path Comparison**: Side-by-side algorithm comparison

## Contributing

To contribute to the Foxglove visualization:

1. Follow the existing code style
2. Add tests for new features
3. Update documentation
4. Consider performance impact
5. Test with various scenarios

## License

This visualization system follows the same license as the main project.
