# Pure Python Dubins Path Planning

This is a pure Python implementation of the Dubins path planning algorithms, originally based on the [pydubins](https://github.com/AndrewWalker/pydubins) library by Andrew Walker. This implementation removes the dependency on Cython and C compilation while maintaining the same API.

## Features

- Pure Python implementation (no C compilation required)
- Same API as the original pydubins library
- Support for all 6 Dubins path types (LSL, LSR, RSL, RSR, RLR, LRL)
- Backward motion support
- Path sampling and visualization utilities
- Comprehensive error handling

## Installation

```bash
pip install -e .
```

## Quick Start

```python
import pure_python_dubins as dubins

# Define start and goal configurations
q0 = (0.0, 0.0, 0.0)      # (x, y, theta)
q1 = (4.0, 0.0, 0.0)      # (x, y, theta)
rho = 1.0                  # turning radius

# Find the shortest Dubins path
path = dubins.shortest_path(q0, q1, rho)

# Sample the path
points, distances = dubins.path_sample(q0, q1, rho, step_size=0.1)

# Get path information
print(f"Path length: {path.path_length()}")
print(f"Path type: {path.path_type()}")
```

## API Reference

### Main Functions

- `shortest_path(q0, q1, rho)` - Find the shortest Dubins path
- `path(q0, q1, rho, path_type)` - Find a path of specific type
- `path_sample(q0, q1, rho, step_size)` - Sample a path at regular intervals
- `backward_path_sample(q0, q1, rho, step_size)` - Sample for backward motion
- `norm_path(alpha, beta, delta, path_type)` - Normalized path calculation

### DubinsPath Class

- `path_length()` - Get total path length
- `segment_length(i)` - Get length of i-th segment
- `path_type()` - Get path type
- `sample(t)` - Sample path at distance t
- `sample_many(step_size)` - Sample entire path
- `extract_subpath(t)` - Extract subpath

### Path Types

- `dubins.LSL` - Left-Straight-Left
- `dubins.LSR` - Left-Straight-Right
- `dubins.RSL` - Right-Straight-Left
- `dubins.RSR` - Right-Straight-Right
- `dubins.RLR` - Right-Left-Right
- `dubins.LRL` - Left-Right-Left

## Examples

See the `demos/` directory for comprehensive examples:

### For environments with display (GUI):
```bash
cd pure_python_dubins
python demos/demo.py
```

This will generate several visualizations comparing forward and backward Dubins paths and display them interactively.

### For headless environments (no display):
```bash
cd pure_python_dubins
python demos/demo_headless.py
```

This will generate several PNG image files with the visualizations:
- `forward_backward_comparison.png` - Comparison of forward vs backward paths
- `forward_backward_comparison_angles.png` - Paths with different orientations
- `backward_only_path.png` - Backward motion path example
- `samples.png` - Multiple Dubins path examples

The headless demo is particularly useful for:
- Server environments without display
- Automated testing
- Batch processing
- Generating figures for documentation

## Testing

```bash
python -m pytest tests/
```

## Performance

This pure Python implementation is slower than the original C-based pydubins but offers the advantage of:

- No compilation required
- Easier to understand and modify
- Cross-platform compatibility
- No external dependencies

## License

MIT License - see the original pydubins project for details.

## Differences from Original pydubins

1. **Pure Python**: No Cython or C compilation required
2. **Backward Motion**: Added support for backward path planning
3. **Error Handling**: More comprehensive error messages
4. **Type Hints**: Added type annotations for better code clarity
5. **Documentation**: Improved docstrings and examples

## Contributing

Contributions are welcome! Please feel free to submit issues and pull requests.

## References

- Dubins, L. E. (1957). On curves of minimal length with a constraint on average curvature, and with prescribed initial and terminal positions and tangents. American Journal of Mathematics, 79(3), 497-516.
- Original pydubins: https://github.com/AndrewWalker/pydubins