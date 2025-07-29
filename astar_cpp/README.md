# Hybrid A* C++ Implementation

A high-performance C++ implementation of the Hybrid A* path planning algorithm with Python bindings using pybind11.

## Features

- **High Performance**: Native C++ implementation for maximum speed
- **Python Integration**: Seamless Python bindings via pybind11
- **Flexible Vehicle Model**: Configurable bicycle kinematic model
- **Obstacle Avoidance**: Support for grid-based obstacle maps
- **Comprehensive Cost Functions**: Distance, steering, turning, and cusp costs
- **Motion Primitives**: Forward simulation with steering angle velocity
- **Detailed Path Output**: Both waypoint and detailed trajectory extraction

## Architecture

```
astar_cpp/
├── include/           # Header files
│   └── hybrid_astar.hpp
├── src/              # Implementation files
│   └── hybrid_astar.cpp
├── python/           # Python bindings
│   └── pybind_module.cpp
├── tests/            # C++ tests
│   └── test_main.cpp
├── examples/         # Demo applications
│   └── demo.cpp
├── CMakeLists.txt    # Build configuration
├── setup.py          # Python package setup
└── README.md
```

## Building

### Prerequisites

- C++17 compatible compiler (GCC 7+, Clang 5+, MSVC 2019+)
- CMake 3.14+
- Python 3.7+ (for Python bindings)
- pybind11

### C++ Library and Tests

```bash
# Create build directory
mkdir build && cd build

# Configure with CMake
cmake ..

# Build
make -j$(nproc)

# Run tests
./hybrid_astar_test

# Run demo
./hybrid_astar_example
```

### Python Bindings

```bash
# Install in development mode
pip install -e .

# Or build wheel
pip install build
python -m build

# Install wheel
pip install dist/hybrid_astar_cpp-*.whl
```

## Usage

### C++ Usage

```cpp
#include "hybrid_astar.hpp"
using namespace hybrid_astar;

// Create vehicle model
VehicleModel vehicle(2.5, M_PI/4);  // wheelbase=2.5m, max_steer=45°

// Create planner
HybridAStar planner(vehicle, 
                   1.0,      // grid_resolution
                   M_PI/8,   // angle_resolution  
                   M_PI/16,  // steer_resolution
                   2.0,      // velocity
                   1.0,      // simulation_time
                   0.1);     // dt

// Set obstacle map (optional)
std::vector<std::vector<int>> obstacle_map(50, std::vector<int>(50, 0));
planner.set_obstacle_map(obstacle_map);

// Plan path
State start(0.0, 0.0, 0.0, DirectionMode::NONE, 0.0);
State goal(10.0, 10.0, M_PI/2, DirectionMode::FORWARD, 0.0);
auto path = planner.plan_path(start, goal);

if (path.has_value()) {
    std::cout << "Path found with " << path->size() << " waypoints" << std::endl;
    
    // Get detailed trajectory
    auto detailed_path = planner.extract_detailed_path(*path);
    
    // Get statistics
    auto stats = planner.get_statistics(path);
}
```

### Python Usage

```python
import hybrid_astar_cpp as ha
import numpy as np

# Create vehicle model
vehicle = ha.VehicleModel(wheelbase=2.5, max_steer=np.pi/4)

# Create planner
planner = ha.HybridAStar(
    vehicle_model=vehicle,
    grid_resolution=1.0,
    angle_resolution=np.pi/8,
    steer_resolution=np.pi/16,
    velocity=2.0,
    simulation_time=1.0,
    dt=0.1
)

# Create obstacle map
obstacle_map = ha.create_obstacle_map(50, 50)
ha.add_rectangle_obstacle(obstacle_map, 20, 20, 30, 30)
planner.set_obstacle_map(obstacle_map)

# Plan path
start = ha.State(0.0, 0.0, 0.0, ha.DirectionMode.NONE, 0.0)
goal = ha.State(10.0, 10.0, np.pi/2, ha.DirectionMode.FORWARD, 0.0)
path = planner.plan_path(start, goal)

if path:
    print(f"Path found with {len(path)} waypoints")
    
    # Get detailed trajectory
    detailed_path = planner.extract_detailed_path(path)
    
    # Get statistics
    stats = planner.get_statistics(path)
    for key, value in stats.items():
        print(f"{key}: {value}")
```

## Key Classes

### State
Represents vehicle state with position, orientation, direction, and steering angle.

### VehicleModel
Implements bicycle kinematic model for motion simulation.

### HybridAStar
Main planning algorithm with configurable parameters and cost functions.

### Node
Search node containing state, costs, and parent relationships.

## Algorithm Features

- **Motion Primitives**: Multiple steering rates for comprehensive search
- **Cost Functions**: Weighted combination of distance, steering, turning, and cusp costs
- **Collision Checking**: Grid-based obstacle avoidance
- **Goal Tolerance**: Configurable position and angle tolerances
- **Heuristic**: Euclidean distance + angular difference

## Performance

The C++ implementation provides significant performance improvements over pure Python:

- **10-100x faster** path planning depending on problem complexity
- **Reduced memory usage** through efficient data structures
- **Parallel compilation** support for faster builds
- **Optimized algorithms** with minimal overhead

## Testing

```bash
# Run C++ tests
cd build && ./hybrid_astar_test

# Run Python tests (after installation)
pytest tests/

# Benchmark performance
python examples/benchmark.py
```

## Integration with Path Tracking Project

This C++ implementation is designed to integrate seamlessly with the existing Python path tracking project:

```python
# Convert from Python VehicleState to C++ State
from PathTracking.vehicle_model import VehicleState
import hybrid_astar_cpp as ha

def convert_to_cpp_state(vehicle_state: VehicleState) -> ha.State:
    direction = ha.DirectionMode.FORWARD if vehicle_state.velocity >= 0 else ha.DirectionMode.BACKWARD
    return ha.State(
        vehicle_state.position_x,
        vehicle_state.position_y, 
        vehicle_state.yaw_angle,
        direction,
        vehicle_state.steering_angle
    )

# Use C++ planner in Python workflow
cpp_start = convert_to_cpp_state(python_vehicle_state)
cpp_goal = convert_to_cpp_state(goal_state)
path = cpp_planner.plan_path(cpp_start, cpp_goal)
```

## License

MIT License - see LICENSE file for details.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Ensure all tests pass
5. Submit a pull request

## Acknowledgments

Based on the original Python implementation in the path_tracking project, with optimizations and enhancements for production use.
