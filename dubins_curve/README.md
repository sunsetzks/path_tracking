# Dubins Curve Package

A Python implementation of Dubins curves for path planning applications in robotics and autonomous vehicles.

## Overview

Dubins curves are the shortest paths between two points with constraints on the minimum turning radius. This package provides a complete implementation of the Dubins path algorithm with support for all six path types:

- LSL: Left-Straight-Left
- RSR: Right-Straight-Right
- LSR: Left-Straight-Right
- RSL: Right-Straight-Left
- RLR: Right-Left-Right
- LRL: Left-Right-Left

## Features

- Complete Dubins path algorithm implementation
- Support for all six path types
- Configurable turning radius
- Path discretization and point generation
- Comprehensive test suite
- Visualization examples
- Type hints and documentation

## Installation

### From Source

```bash
git clone <repository-url>
cd dubins_curve
pip install -e .
```

### Dependencies

- Python 3.7+
- matplotlib (for examples and visualization)

## Quick Start

```python
from dubins_curve.dubins import DubinsPath
from dubins_curve.types import Pose

# Create start and end poses
start = Pose(0, 0, 0)  # x, y, theta (radians)
end = Pose(4, 0, 0)    # 4 units away, same orientation

# Calculate Dubins path
dubins = DubinsPath(turning_radius=1.0)
result = dubins.path(start, end)

if result.success:
    print(f"Path type: {result.path_type}")
    print(f"Total length: {result.total_length:.3f}")
    
    # Get path points for visualization
    path_points = dubins.get_path_points(result, num_points=100)
    print(f"Generated {len(path_points)} path points")
```

## Usage Examples

### Basic Path Calculation

```python
from dubins_curve.dubins import DubinsPath
from dubins_curve.types import Pose
import math

# Define poses
start = Pose(0, 0, math.pi/4)    # 45-degree orientation
end = Pose(4, 3, -math.pi/4)    # -45-degree orientation

# Calculate path
dubins = DubinsPath(turning_radius=1.0)
result = dubins.path(start, end)

if result.success:
    print(f"Path found: {result.path_type}")
    print(f"Length: {result.total_length:.3f}")
    
    # Access path segments
    for i, segment in enumerate(result.segments):
        print(f"Segment {i+1}: {segment.segment_type}, length: {segment.length:.3f}")
```

### Path Visualization

Run the examples to see visualizations:

```bash
cd dubins_curve/examples
python basic_example.py
```

This will display several examples showing:
- Basic path calculation
- Paths with different orientations
- Multiple paths with different turning radii
- Continuous path planning

## API Reference

### DubinsPath

Main class for calculating Dubins paths.

#### Methods

- `__init__(turning_radius=1.0)`: Initialize with turning radius
- `path(start, end)`: Calculate shortest path between two poses
- `get_path_points(path_result, num_points=100)`: Generate discretized path points

#### Returns

Returns a `DubinsPathResult` object with:
- `path_type`: String identifying the path type (e.g., 'LSL')
- `total_length`: Total path length
- `segments`: List of path segments
- `success`: Boolean indicating success
- `error_message`: Error message if failed

### Pose

Represents a 2D pose with position and orientation.

#### Properties

- `x`: X coordinate
- `y`: Y coordinate  
- `theta`: Orientation in radians (normalized to [-π, π])

#### Methods

- `distance_to(other_pose)`: Calculate Euclidean distance
- `angle_to(other_pose)`: Calculate angle to another pose

### DubinsSegment

Represents a single segment of a Dubins path.

#### Properties

- `segment_type`: 'L', 'R', or 'S' (left, right, straight)
- `length`: Segment length
- `start_pose`: Starting pose of segment
- `end_pose`: Ending pose of segment

## Testing

Run the test suite:

```bash
cd dubins_curve
python -m pytest tests/
```

Or run a specific test file:

```bash
python -m pytest tests/test_dubins.py
```

## Development

### Setting up Development Environment

```bash
# Clone the repository
git clone <repository-url>
cd dubins_curve

# Install in development mode
pip install -e ".[dev]"

# Run tests
python -m pytest tests/

# Run examples
python examples/basic_example.py
```

### Code Style

This project uses:
- Black for code formatting
- flake8 for linting
- mypy for type checking

```bash
# Format code
black .

# Lint code
flake8 .

# Type check
mypy .
```

## Mathematical Background

Dubins paths are composed of circular arcs and straight line segments. The algorithm considers all six possible combinations of left turns (L), right turns (R), and straight segments (S):

1. **LSL**: Left turn → Straight line → Left turn
2. **RSR**: Right turn → Straight line → Right turn
3. **LSR**: Left turn → Straight line → Right turn
4. **RSL**: Right turn → Straight line → Left turn
5. **RLR**: Right turn → Left turn → Right turn
6. **LRL**: Left turn → Right turn → Left turn

For each path type, the algorithm calculates the optimal parameters that minimize the total path length while respecting the turning radius constraint.

## Applications

Dubins curves are commonly used in:

- Autonomous vehicle path planning
- Robot navigation
- UAV trajectory planning
- Autonomous boat navigation
- Any application requiring smooth, curvature-constrained paths

## License

MIT License - see LICENSE file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## References

- Dubins, L. E. (1957). "On Curves of Minimal Length with a Constraint on Average Curvature, and with Prescribed Initial and Terminal Positions and Tangents". American Journal of Mathematics.
- LaValle, S. M. (2006). "Planning Algorithms". Cambridge University Press.