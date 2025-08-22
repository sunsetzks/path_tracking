# Foxglove Channel-Based Architecture

This document describes the new channel-based architecture for the Foxglove Helper library, providing a more modular and extensible approach to managing visualization data.

## Overview

The new architecture introduces several key components:

- **BaseChannel**: Abstract base class for all channel types
- **ChannelManager**: Centralized management of channels and server lifecycle
- **Channel Types**: Specialized channels for different data types
- **Utility Functions**: Helper functions for creating complex data structures
- **Enhanced Helper**: Drop-in replacement with advanced features

## Architecture Components

### 1. Channel Types

The system supports multiple channel types, each optimized for specific data formats:

#### SceneUpdateChannel
For 3D scene visualization with primitives (cubes, spheres, arrows, lines).

```python
from forglove_helper.channel_manager import ChannelManager, ChannelType

manager = ChannelManager()
manager.start_server()

# Create a scene channel
scene_channel = manager.create_scene_channel("main_scene", "/visualization/scene")

# Publish 3D content
sphere = PrimitiveUtils.create_sphere((1, 2, 3), radius=0.5, color=(1, 0, 0, 1))
entity = SceneEntity(id="my_sphere", frame_id="root", spheres=[sphere])
scene_channel.publish([entity])
```

#### DataChannel
For JSON data messages with optional schema validation.

```python
# Create a data channel
data_channel = manager.create_data_channel("telemetry", "/robot/telemetry")

# Publish structured data
data_channel.publish({
    "timestamp": time.time(),
    "position": {"x": 1.0, "y": 2.0, "z": 3.0},
    "velocity": 5.2,
    "status": "active"
})
```

#### TfChannel
For coordinate transformations between reference frames.

```python
# Create a transform channel
tf_channel = manager.create_tf_channel("transforms", "/tf")

# Publish a transformation
tf_channel.publish_transform(
    parent_frame="map",
    child_frame="base_link",
    translation=(1.0, 2.0, 0.0),
    rotation=(0.0, 0.0, 0.0, 1.0)  # quaternion
)
```

#### GridChannel
For occupancy grids and map data.

```python
# Create a grid channel
grid_channel = manager.create_grid_channel("map", "/occupancy_grid")

# Create and publish a test grid
test_grid = GridUtils.create_simple_test_grid(size=100, resolution=0.1)
grid_channel.publish(test_grid)
```

#### PointCloudChannel
For 3D point cloud data.

```python
# Create a point cloud channel
pc_channel = manager.create_pointcloud_channel("lidar", "/pointcloud")

# Generate test point cloud
import numpy as np
points = np.random.rand(1000, 3) * 10  # 1000 random points
colors = np.random.randint(0, 255, (1000, 3), dtype=np.uint8)

pointcloud = PointCloudUtils.create_point_cloud(points, colors)
pc_channel.publish(pointcloud)
```

#### LaserScanChannel
For 2D laser scan data.

```python
# Create a laser scan channel
laser_channel = manager.create_laser_channel("scan", "/laser_scan")

# Publish laser scan data
ranges = [1.0, 2.0, 3.0, 2.5, 1.8] * 72  # 360 degree scan
laser_scan = LaserScanUtils.create_laser_scan(ranges)
laser_channel.publish(laser_scan)
```

#### ProtoChannel
For custom protobuf messages.

```python
# Create a protobuf channel
proto_channel = manager.create_proto_channel("custom", "/proto_data", MyProtoClass)

# Publish protobuf message
message = MyProtoClass()
message.field1 = "value"
proto_channel.publish(message)
```

### 2. ChannelManager

The `ChannelManager` class provides centralized control over all channels and server operations.

```python
from forglove_helper.channel_manager import ChannelManager

# Create manager
manager = ChannelManager(port=8765)

# Start server with MCAP recording
manager.start_server(enable_mcap=True)

# Create channels
scene_ch = manager.create_scene_channel("scene", "/viz/scene")
data_ch = manager.create_data_channel("data", "/robot/status")

# Publish data
manager.publish("scene", scene_entities)
manager.publish("data", {"status": "running"})

# Monitor status
manager.print_status()

# Stop server
manager.stop_server()
```

### 3. Context Manager Support

All components support context manager usage for automatic cleanup:

```python
with ChannelManager(port=8765) as manager:
    manager.start_server()
    
    scene_ch = manager.create_scene_channel("scene", "/viz")
    
    # Do work...
    scene_ch.publish(my_entities)
    
    # Automatic cleanup on exit
```

### 4. Utility Functions

The system provides utility functions for creating complex data structures:

#### PrimitiveUtils
Helper functions for 3D primitives:

```python
from forglove_helper.channel_utils import PrimitiveUtils

# Create primitives
cube = PrimitiveUtils.create_cube((0, 0, 0), size=(1, 1, 1))
sphere = PrimitiveUtils.create_sphere((1, 0, 0), radius=0.5)
line = PrimitiveUtils.create_line([(0,0,0), (1,1,1), (2,0,0)])
arrow = PrimitiveUtils.create_arrow((0, 0, 1), direction=math.pi/4)
```

#### TransformUtils
Helper functions for coordinate transformations:

```python
from forglove_helper.channel_utils import TransformUtils

# Create transforms
identity = TransformUtils.create_identity_transform("parent", "child")
translation = TransformUtils.create_translation_transform("map", "robot", 1, 2, 0)
rotation = TransformUtils.create_rotation_transform("base", "sensor", 0, 0, math.pi/2)

# Convert Euler to quaternion
qx, qy, qz, qw = TransformUtils.euler_to_quaternion(roll, pitch, yaw)
```

#### GridUtils
Helper functions for grid/map data:

```python
from forglove_helper.channel_utils import GridUtils
import numpy as np

# Create occupancy grid
data = np.zeros((100, 100), dtype=np.uint8)
data[20:30, 20:30] = 100  # Add obstacle

grid = GridUtils.create_occupancy_grid(
    width=100, height=100, resolution=0.1,
    data=data, frame_id="map"
)

# Create test grid
test_grid = GridUtils.create_simple_test_grid(size=100, resolution=0.1)
```

## Migration Guide

### From Original FoxgloveHelper

The new `EnhancedFoxgloveHelper` provides backward compatibility:

```python
# Old way
from forglove_helper.foxglove_helper import FoxgloveHelper

helper = FoxgloveHelper()
helper.start_server()
# ... use helper
helper.stop_server()

# New way (drop-in replacement)
from forglove_helper.foxglove_helper import EnhancedFoxgloveHelper

helper = EnhancedFoxgloveHelper()
helper.start_server()
# ... use helper with all new features
helper.stop_server()
```

### New Features in EnhancedFoxgloveHelper

```python
helper = EnhancedFoxgloveHelper()
helper.start_server()

# Create additional channel types
tf_channel = helper.create_tf_channel("tf", "/tf")
grid_channel = helper.create_grid_channel("map", "/grid")

# Publish transforms directly
helper.publish_transform("map", "robot", (1, 2, 0), (0, 0, 0, 1))

# Advanced status monitoring
helper.print_status()
status = helper.get_status()
```

## Complete Example

Here's a complete example demonstrating multiple channel types:

```python
#!/usr/bin/env python3
import asyncio
import time
import math
import numpy as np

from forglove_helper.channel_manager import ChannelManager, ChannelType
from forglove_helper.channel_utils import (
    PrimitiveUtils, TransformUtils, GridUtils, PointCloudUtils
)
from foxglove.schemas import SceneEntity

async def main():
    # Create and start manager
    with ChannelManager(port=8765) as manager:
        manager.start_server(enable_mcap=True)
        
        # Create channels
        scene_ch = manager.create_scene_channel("scene", "/viz/scene")
        data_ch = manager.create_data_channel("telemetry", "/robot/data")
        tf_ch = manager.create_tf_channel("tf", "/tf")
        grid_ch = manager.create_grid_channel("map", "/grid")
        pc_ch = manager.create_pointcloud_channel("lidar", "/pointcloud")
        
        # Publish static data
        test_grid = GridUtils.create_simple_test_grid(100, 0.1)
        grid_ch.publish(test_grid)
        
        # Animation loop
        start_time = time.time()
        while time.time() - start_time < 30:  # 30 second demo
            t = time.time() - start_time
            
            # Animated 3D scene
            sphere = PrimitiveUtils.create_sphere(
                (3 * math.cos(t), 3 * math.sin(t), 0.5),
                radius=0.3, color=(1, 0.5, 0, 1)
            )
            arrow = PrimitiveUtils.create_arrow(
                (0, 0, 0.3), direction=t, length=1.5
            )
            
            entities = [
                SceneEntity(id="sphere", frame_id="root", spheres=[sphere]),
                SceneEntity(id="arrow", frame_id="root", arrows=[arrow])
            ]
            scene_ch.publish(entities)
            
            # Robot telemetry
            data_ch.publish({
                "time": t,
                "position": [3 * math.cos(t), 3 * math.sin(t), 0],
                "heading": t,
                "speed": 3.0
            })
            
            # Transform updates
            tf_ch.publish_transform(
                "map", "robot",
                (3 * math.cos(t), 3 * math.sin(t), 0),
                (0, 0, math.sin(t/2), math.cos(t/2))
            )
            
            # Occasional point cloud
            if int(t) % 5 == 0 and t - int(t) < 0.1:
                test_pc = PointCloudUtils.create_test_point_cloud(2000, 5.0)
                pc_ch.publish(test_pc)
            
            await asyncio.sleep(0.1)  # 10 Hz
        
        # Print final status
        manager.print_status()

if __name__ == "__main__":
    asyncio.run(main())
```

## Running the Examples

### Channel Demo
```bash
cd examples
python channel_demo.py
```

### Enhanced Helper Demo
```bash
python -m forglove_helper.foxglove_helper
```

### Original Helper (for comparison)
```bash
python -m forglove_helper.foxglove_helper --original
```

## Configuration

### MCAP Recording
```python
# Enable MCAP recording
manager = ChannelManager(mcap_output_path="my_recording.mcap")
manager.start_server(enable_mcap=True)

# Or enable later
manager.enable_mcap_recording("custom_path.mcap")
```

### Custom Port
```python
manager = ChannelManager(port=9876)
```

### Logging
```python
import logging
logging.basicConfig(level=logging.INFO)

# Channel operations will be logged
```

## Benefits of the New Architecture

1. **Modularity**: Each channel type handles specific data formats
2. **Extensibility**: Easy to add new channel types
3. **Type Safety**: Better type hints and validation
4. **Performance**: Optimized for different data types
5. **Monitoring**: Built-in status and statistics
6. **Backward Compatibility**: Existing code continues to work
7. **Context Management**: Automatic resource cleanup
8. **Utility Functions**: Rich helpers for complex data structures

## Future Enhancements

- Additional channel types (Image, Audio, etc.)
- Dynamic channel discovery
- Channel synchronization
- Performance metrics
- Remote channel management
- Plugin system for custom channels

## Troubleshooting

### Common Issues

1. **ImportError: New channel system not available**
   - Ensure all new files are in the Python path
   - Check for circular import issues

2. **Channel creation fails**
   - Verify unique channel names
   - Check topic name conflicts
   - Ensure server is started

3. **MCAP recording issues**
   - Check file permissions in logs/ directory
   - Verify disk space
   - Ensure proper cleanup

### Debug Mode
```python
import logging
logging.basicConfig(level=logging.DEBUG)

# Enables detailed logging for troubleshooting
```
