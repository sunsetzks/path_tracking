# PlotUtils - Matplotlib-like Plotting for Foxglove

The `PlotUtils` class provides matplotlib-like plotting functionality for creating 2D and 3D visualizations in Foxglove Studio. It creates Foxglove-compatible 3D primitives that can be visualized in real-time.

## Features

- **2D & 3D Line Plots**: Similar to matplotlib's `plot()` function
- **2D & 3D Scatter Plots**: Similar to matplotlib's `scatter()` function
- **Multiple Marker Styles**: Circle, square, diamond markers
- **Color Support**: Named colors, RGB/RGBA tuples, color arrays
- **Coordinate Axes**: Built-in axis creation for reference
- **Easy Integration**: Works seamlessly with existing Foxglove channels

## Quick Start

```python
from forglove_helper.channel_utils import PlotUtils
import numpy as np

# Create sample data
x = np.linspace(0, 10, 100)
y = np.sin(x)

# Create a line plot
primitives = PlotUtils.plot(x, y, color=(1.0, 0.0, 0.0, 1.0))

# Create a scatter plot
x_scatter = np.random.uniform(0, 10, 20)
y_scatter = np.random.uniform(-1, 1, 20)
scatter_primitives = PlotUtils.scatter(x_scatter, y_scatter, c='blue', marker='circle', s=50)
```

## API Reference

### PlotUtils.plot()

Creates a 2D or 3D line plot.

```python
def plot(x, y, z=None, color=(1.0, 1.0, 1.0, 1.0), linewidth=0.05,
         marker='line', markersize=0.1, label='', frame_id='plot')
```

**Parameters:**
- `x`, `y`: Coordinate arrays (lists or numpy arrays)
- `z`: Optional Z coordinates for 3D plots (default: 0 for 2D)
- `color`: RGBA color tuple (0-1 range)
- `linewidth`: Line thickness
- `marker`: Marker style ('line', 'circle', 'square', 'diamond')
- `markersize`: Size of markers (if marker != 'line')
- `label`: Label for identification
- `frame_id`: Frame ID for the plot

### PlotUtils.scatter()

Creates a 2D or 3D scatter plot.

```python
def scatter(x, y, z=None, s=50, c=(1.0, 0.0, 0.0, 1.0), marker='circle',
            alpha=1.0, label='', frame_id='scatter')
```

**Parameters:**
- `x`, `y`: Coordinate arrays
- `z`: Optional Z coordinates for 3D scatter
- `s`: Marker size(s) - single value or array
- `c`: Color(s) - can be:
  - String: 'red', 'blue', 'green', etc.
  - RGB tuple: (1.0, 0.5, 0.0)
  - RGBA tuple: (1.0, 0.5, 0.0, 0.8)
  - List of colors for multiple points
- `marker`: Marker style ('circle', 'square', 'diamond')
- `alpha`: Transparency (0-1)
- `label`: Label for identification
- `frame_id`: Frame ID for the plot

### PlotUtils.create_axes()

Creates coordinate axes for reference.

```python
def create_axes(xlim=(-10, 10), ylim=(-10, 10), zlim=None,
                color=(0.5, 0.5, 0.5, 1.0), linewidth=0.02)
```

## Color Support

The PlotUtils supports various color formats:

```python
# Named colors
PlotUtils.scatter(x, y, c='red')
PlotUtils.scatter(x, y, c='blue')

# RGB tuples (alpha=1.0 automatically added)
PlotUtils.scatter(x, y, c=(0.5, 0.5, 0.5))

# RGBA tuples
PlotUtils.scatter(x, y, c=(1.0, 0.0, 0.0, 0.8))

# Multiple colors for different points
colors = [(1, 0, 0, 1), (0, 1, 0, 1), (0, 0, 1, 1)]
PlotUtils.scatter(x, y, c=colors)
```

## Examples

### Basic 2D Plot

```python
import numpy as np
from forglove_helper.channel_utils import PlotUtils

# Create data
x = np.linspace(0, 10, 100)
y = np.sin(x)

# Create plot
primitives = PlotUtils.plot(x, y, color=(1.0, 0.0, 0.0, 1.0))
```

### 3D Scatter Plot

```python
import numpy as np
from forglove_helper.channel_utils import PlotUtils

# Create 3D data
x = np.random.normal(0, 2, 50)
y = np.random.normal(0, 2, 50)
z = np.random.normal(0, 2, 50)

# Create 3D scatter plot with colors based on z-coordinate
colors = []
for zi in z:
    t = (zi - z.min()) / (z.max() - z.min())
    colors.append((t, 0.5, 1.0-t, 1.0))

primitives = PlotUtils.scatter(x, y, z, c=colors, marker='circle', s=30)
```

### Integration with Foxglove Channels

```python
from forglove_helper.channel_manager import ChannelManager
from forglove_helper.channel_utils import PlotUtils
from foxglove.schemas import SceneEntity, SceneUpdate, Timestamp
import time

# Initialize channel manager
channel_manager = ChannelManager(port=8765)
scene_channel = channel_manager.create_scene_channel("my_plots")

# Start server
channel_manager.start_server()

# Create your plot data
x = np.linspace(-5, 5, 100)
y = np.sin(x)
plot_primitives = PlotUtils.plot(x, y, color=(0.0, 1.0, 0.0, 1.0))

# Create scene entity
entities = [SceneEntity(
    id="sine_wave",
    timestamp=Timestamp(sec=int(time.time()), nsec=0),
    frame_id="my_plots",
    lifetime=None,
    frame_locked=False,
    lines=plot_primitives if plot_primitives else [],
    cubes=[],
    spheres=[]
)]

# Publish to Foxglove
scene_update = SceneUpdate(deletions=[], entities=entities)
scene_channel.publish(scene_update)

# Don't forget to stop the server when done
# channel_manager.stop_server()
```

## Running the Examples

### Simple Example
```bash
cd examples
PYTHONPATH=../src python3 simple_plot_example.py
```

### Full Demo
```bash
cd examples
PYTHONPATH=../src python3 plot_demo.py
```

### Test Suite
```bash
cd examples
PYTHONPATH=../src python3 test_plot_utils.py
```

Connect Foxglove Studio to `ws://localhost:8765` to view the visualizations.

## Notes

- All coordinates are in meters
- Colors use RGBA format with values 0-1
- Line plots use `LinePrimitive` objects
- Scatter plots use `SpherePrimitive` and `CubePrimitive` objects
- The utilities integrate seamlessly with the existing Foxglove channel system
- All functions return lists of Foxglove primitive objects that can be published directly

## Dependencies

- numpy
- foxglove-sdk
- typing (built-in)

The PlotUtils is part of the forglove-helper package and works with the existing channel management system for easy integration into your visualization projects.
