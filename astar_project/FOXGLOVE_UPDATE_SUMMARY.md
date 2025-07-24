# Foxglove Visualization Update Summary

## Overview

The Foxglove visualization code has been updated to use the modern Foxglove SDK API (v0.10.1+) instead of the deprecated protobuf-based approach. The new implementation provides a simpler, more maintainable solution for visualizing Hybrid A* path planning results.

## Key Changes

### 1. Modern SDK API Usage

**Before (Old API):**
```python
import foxglove_schemas_protobuf.Color_pb2 as Color
import foxglove_schemas_protobuf.LinePrimitive_pb2 as LinePrimitive
from foxglove.websocket import start_server, FoxgloveServer
```

**After (New API):**
```python
import foxglove
from foxglove import Channel, Schema
from foxglove.websocket import WebSocketServer
```

### 2. Simplified Data Format

The new implementation uses JSON-based schemas instead of complex protobuf messages, making it easier to understand and debug:

```python
# JSON schema for visualization data
path_schema = {
    "type": "object",
    "properties": {
        "path": {"type": "array", "items": {...}},
        "exploration_nodes": {"type": "array", "items": {...}},
        "start": {"type": "object", "properties": {...}},
        "goal": {"type": "object", "properties": {...}},
        "obstacles": {"type": "array", "items": {...}}
    }
}
```

### 3. Improved Server Management

**Before:**
- Complex async server setup with manual channel advertising
- Manual message serialization and sending

**After:**
- Simple server creation: `foxglove.start_server()`
- Automatic channel management with `Channel.log()`

### 4. Enhanced Fallback Support

Added robust matplotlib fallback visualization when Foxglove SDK is not available:

```python
def matplotlib_fallback_visualization(path, start, goal, explored_nodes=None, ...):
    """Fallback visualization using matplotlib when Foxglove is not available"""
```

## New Features

### 1. Comprehensive Data Visualization

The updated visualizer sends rich data including:
- **Path**: Final planned path with position and orientation
- **Exploration Tree**: Nodes explored during search with parent-child relationships
- **Start/Goal**: Markers with orientation arrows
- **Obstacles**: Grid-based obstacle map
- **Statistics**: Real-time planning metrics

### 2. Performance Optimizations

- **Node Limiting**: Automatically limits exploration nodes for performance
- **Obstacle Batching**: Limits obstacle count to prevent overwhelming
- **Efficient Data Structures**: Uses simple dictionaries instead of complex protobuf objects

### 3. Better Error Handling

- Graceful degradation when Foxglove SDK is unavailable
- Clear error messages and troubleshooting tips
- Automatic fallback to matplotlib visualization

## Usage Examples

### Basic Usage

```python
from astar_project.foxglove_visualizer import FoxgloveHybridAStarVisualizer

# Create visualizer
visualizer = FoxgloveHybridAStarVisualizer(port=8765)

# Visualize path planning results
visualizer.visualize_path_planning(
    path=path_states,
    start=start_state,
    goal=goal_state,
    explored_nodes=explored_nodes,
    obstacle_map=obstacle_map,
    map_origin_x=-10,
    map_origin_y=-10,
    grid_resolution=0.5
)
```

### Live Planning Visualization

```python
# Plan and visualize simultaneously
path = visualizer.visualize_live_planning(planner, start, goal)
```

### Matplotlib Fallback

```python
from astar_project.foxglove_visualizer import matplotlib_fallback_visualization

# Use when Foxglove is not available
matplotlib_fallback_visualization(
    path=path_states,
    start=start_state,
    goal=goal_state,
    explored_nodes=explored_nodes,
    obstacle_map=obstacle_map
)
```

## Installation Requirements

### For Foxglove Visualization

```bash
pip install foxglove-sdk
```

### For Matplotlib Fallback

```bash
pip install matplotlib
```

## Foxglove Studio Configuration

1. **Install Foxglove Studio**: Download from https://foxglove.dev/download
2. **Connect to WebSocket**: Use `ws://localhost:8765` 
3. **Add Panels**: 
   - JSON panel for `/hybrid_astar/visualization`
   - Raw Messages panel for `/hybrid_astar/statistics`
4. **Configure Views**: Create custom layouts for different visualization needs

## Topics Published

### `/hybrid_astar/visualization`
- **Type**: JSON
- **Content**: Complete visualization data including path, exploration tree, obstacles
- **Schema**: Structured JSON with typed fields

### `/hybrid_astar/statistics`
- **Type**: JSON  
- **Content**: Planning metrics and performance data
- **Fields**: path_length, nodes_explored, total_distance, steering_angles, etc.

## Benefits of the Update

1. **Simpler API**: Easier to understand and maintain
2. **Better Performance**: Reduced memory usage and faster serialization
3. **Enhanced Debugging**: JSON format is human-readable
4. **Improved Reliability**: Better error handling and fallback options
5. **Future-Proof**: Uses latest SDK version with ongoing support

## Testing

Run the demo script to test the updated visualization:

```bash
cd /home/zks/ws/path_tracking/astar_project
python foxglove_demo.py
```

The demo will:
1. Create a test scenario with obstacles
2. Plan a path using Hybrid A*
3. Visualize results using Foxglove (if available) or matplotlib
4. Provide connection instructions for Foxglove Studio

## Migration Notes

If you have existing code using the old Foxglove API:

1. **Update imports**: Replace protobuf imports with modern SDK imports
2. **Simplify schemas**: Convert protobuf schemas to JSON schemas  
3. **Update server creation**: Use `foxglove.start_server()` instead of manual setup
4. **Modify data sending**: Use `Channel.log()` instead of manual message sending
5. **Add fallback**: Implement matplotlib fallback for better user experience

The updated implementation is backward-compatible in terms of functionality while providing a much cleaner and more maintainable codebase.
