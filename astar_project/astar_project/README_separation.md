# Hybrid A* Algorithm - Separated Implementation

This directory contains a refactored version of the Hybrid A* path planning algorithm where the core algorithm has been separated from the visualization components.

## File Structure

```
astar_project/
├── hybrid_astar.py          # Core algorithm (no visualization dependencies)
├── visualizer.py            # Visualization module (matplotlib-based)
├── demo_separated.py        # Demo showing separated usage
├── test_simple.py          # Test script for separation verification
├── hybrid_astar_original.py # Original combined implementation (backup)
└── README_separation.md    # This file
```

## Key Benefits of Separation

### 🚀 **Performance & Dependencies**
- Core algorithm has no matplotlib dependency
- Can run in headless environments (servers, containers)
- Faster imports and reduced memory footprint
- No GUI dependencies for pure computational use

### 🧩 **Modularity**
- Clean separation of concerns
- Visualization is completely optional
- Easy to swap visualization backends
- Better unit testing capabilities

### 🔧 **Maintainability**
- Cleaner, more focused code
- Easier to debug algorithm issues
- Independent development of visualization features
- Reduced coupling between components

## Usage Examples

### Core Algorithm Only (No Visualization)

```python
from hybrid_astar import HybridAStar, VehicleModel, State, DirectionMode
import numpy as np

# Create vehicle and planner
vehicle = VehicleModel(wheelbase=2.5, max_steer=np.pi/4)
planner = HybridAStar(vehicle_model=vehicle)

# Define start and goal
start = State(x=0.0, y=0.0, yaw=0.0, direction=DirectionMode.FORWARD)
goal = State(x=10.0, y=10.0, yaw=np.pi/2, direction=DirectionMode.FORWARD)

# Plan path
path = planner.plan_path(start, goal)

if path:
    # Get statistics
    stats = planner.get_statistics(path)
    print(f"Path found: {stats['total_distance']:.2f}m")
```

### With Optional Visualization

```python
# Import visualization only when needed
from visualizer import HybridAStarVisualizer

# ... (same setup as above) ...

path = planner.plan_path(start, goal)

if path:
    # Create visualizer
    visualizer = HybridAStarVisualizer()
    
    # Get visualization data from algorithm
    viz_data = planner.get_visualization_data()
    
    # Show visualization
    visualizer.visualize_path(
        path=path,
        start=start, 
        goal=goal,
        **viz_data
    )
```

## Core Algorithm API

### Classes

- **`HybridAStar`**: Main path planning algorithm
- **`VehicleModel`**: Bicycle model for vehicle simulation  
- **`State`**: Vehicle state representation
- **`Node`**: Search tree node
- **`DirectionMode`**: Forward/backward direction enum

### Key Methods

- **`plan_path(start, goal)`**: Plan path from start to goal
- **`get_statistics(path)`**: Get path and search statistics
- **`get_visualization_data()`**: Get data for visualization

## Visualization API

### Classes

- **`HybridAStarVisualizer`**: Handles all visualization functionality

### Key Methods

- **`visualize_path(...)`**: Enhanced path visualization
- **`visualize_search_progress(...)`**: Show search tree and progression
- **`visualize_detailed_search_tree(...)`**: Detailed tree with connections

## Testing

Run the separation test to verify everything works:

```bash
python test_simple.py
```

This will verify:
- Core algorithm imports without matplotlib
- Visualization module works independently
- No dependency leakage between modules

## Migration from Original Code

The original `hybrid_astar.py` has been backed up as `hybrid_astar_original.py`. 

### Changes Made:

1. **Removed from core algorithm:**
   - All `matplotlib` imports
   - All `visualize_*` methods
   - All `_plot_*` methods
   - All `plt.*` calls

2. **Added to core algorithm:**
   - `get_visualization_data()` method
   - `get_statistics()` method
   - Better data storage for visualization

3. **New visualization module:**
   - All visualization functionality moved here
   - Enhanced and modular visualization methods
   - Flexible parameter handling

## Deployment Scenarios

### Production Server (No Visualization)
```python
# Only import core algorithm - no matplotlib dependency
from hybrid_astar import HybridAStar, VehicleModel
# Run path planning without any visualization overhead
```

### Development/Analysis (With Visualization)
```python  
# Import both modules when visualization is needed
from hybrid_astar import HybridAStar
from visualizer import HybridAStarVisualizer
# Full visualization capabilities available
```

### Headless Environment
```python
# Algorithm works perfectly in Docker, servers, etc.
# No X11, no GUI dependencies required
```

## Future Extensibility

The separated architecture makes it easy to:
- Add new visualization backends (plotly, bokeh, etc.)
- Create web-based visualizations
- Add real-time visualization for live planning
- Implement 3D visualization
- Create custom analysis tools

## Verification

The separation has been tested and verified:
- ✅ Core algorithm works independently
- ✅ No matplotlib dependency in core
- ✅ Visualization is completely optional
- ✅ All functionality preserved
- ✅ Performance improved for non-visualization use cases

This refactoring maintains full backward compatibility while providing significant architectural improvements.
