# Hybrid A* C++ Implementation

A high-performance, pure C++ implementation of the Hybrid A* path planning algorithm with no Python dependencies. This project is designed to be a standalone CMake-based library for robotics and autonomous vehicle applications.

## Features

- **Pure C++17**: No Python dependencies, fully self-contained C++ library
- **High Performance**: Optimized C++ implementation for maximum speed
- **CMake Build System**: Modern, cross-platform build configuration
- **Comprehensive Testing**: Full test suite with unit tests and performance benchmarks
- **Multiple Demo Applications**: Basic and advanced usage examples
- **Flexible Vehicle Model**: Configurable bicycle kinematic model
- **Obstacle Avoidance**: Support for grid-based obstacle maps
- **Cost Functions**: Distance, steering, turning, and cusp costs
- **Motion Primitives**: Forward simulation with steering angle velocity
- **Detailed Path Output**: Both waypoint and detailed trajectory extraction

## Project Structure

```
hybrid_astar_cpp/
├── include/                    # Header files
│   └── hybrid_astar.hpp        # Main API header
├── src/                       # Implementation files
│   └── hybrid_astar.cpp        # Core algorithm implementation
├── examples/                  # Demo applications
│   ├── demo.cpp               # Basic demo application
│   └── advanced_demo.cpp      # Advanced demo with complex scenarios
├── tests/                     # Test suite
│   └── test_main.cpp          # Comprehensive unit tests
├── CMakeLists.txt             # CMake build configuration
├── HybridAStarConfig.cmake.in # CMake package config template
└── README.md                  # This file
```

## Building

### Prerequisites

- C++17 compatible compiler (GCC 7+, Clang 5+, MSVC 2019+)
- CMake 3.14+
- CMake build tools (make, ninja, etc.)

### Build Steps

1. Create a build directory:
   ```bash
   mkdir build && cd build
   ```

2. Configure with CMake:
   ```bash
   cmake ..
   ```

3. Build the project:
   ```bash
   make -j$(nproc)
   ```

4. Run tests (optional):
   ```bash
   make test
   ```

5. Install (optional):
   ```bash
   make install
   ```

### Build Targets

- `hybrid_astar_lib`: Static library containing the Hybrid A* implementation
- `hybrid_astar_demo`: Basic demo application
- `hybrid_astar_advanced_demo`: Advanced demo application
- `hybrid_astar_test`: Test executable
- `run_demo`: Custom target to run the basic demo
- `run_advanced_demo`: Custom target to run the advanced demo
- `run_tests`: Custom target to run all tests

## Usage

### C++ Usage

```cpp
#include "hybrid_astar.hpp"
#include <iostream>

using namespace hybrid_astar;

int main() {
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
    // Add obstacles...
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
        std::cout << "Total distance: " << stats["total_distance"] << "m" << std::endl;
    }
    
    return 0;
}
```

### Using as a Library

You can also use this as a CMake library in your own projects:

```cmake
find_package(HybridAStar REQUIRED)

add_executable(my_app my_app.cpp)
target_link_libraries(my_app HybridAStar::hybrid_astar_lib)
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

The C++ implementation provides excellent performance:

- **Fast Planning**: Typically completes in 1-100ms depending on problem complexity
- **Memory Efficient**: Smart pointer management for automatic memory management
- **Scalable**: Handles environments up to 50x50 grid effectively
- **Configurable**: Adjustable resolution for speed vs. precision trade-offs

## Testing

Run the comprehensive test suite:

```bash
# Build and run tests
make test

# Or run specific test categories
./hybrid_astar_test --test=basic
./hybrid_astar_test --test=planning
./hybrid_astar_test --test=advanced
```

## Demo Applications

### Basic Demo
Run the basic demo to see simple path planning:

```bash
./hybrid_astar_demo
```

### Advanced Demo
Run the advanced demo to see complex scenarios:

```bash
./hybrid_astar_advanced_demo
```

The advanced demo includes:
- Random obstacle scenarios
- Parking maneuvers
- Maze navigation
- Performance benchmarking

## Configuration Options

The HybridAStar class constructor accepts the following parameters:

```cpp
HybridAStar(const VehicleModel& vehicle_model,
            double grid_resolution = 1.0,
            double angle_resolution = M_PI/8,
            double steer_resolution = M_PI/16,
            double velocity = 2.0,
            double simulation_time = 1.0,
            double dt = 0.1);
```

### Parameters

- `vehicle_model`: Vehicle kinematic model
- `grid_resolution`: Grid cell size in meters
- `angle_resolution`: Angular discretization in radians
- `steer_resolution`: Steering angle discretization in radians
- `velocity`: Forward velocity in m/s
- `simulation_time`: Simulation time for motion primitives in seconds
- `dt`: Time step for simulation in seconds

## Cost Weights

The algorithm uses configurable cost weights:

```cpp
// These can be modified in the HybridAStar class
double w_steer_ = 10.0;    // Steering cost weight
double w_turn_ = 15.0;     // Turning cost weight
double w_cusp_ = 10.0;     // Cusp cost weight
double w_obstacle_ = 1000.0; // Obstacle penalty weight
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

Based on the original Python implementation in the path_tracking project, with optimizations and enhancements for production use in pure C++.

## Changelog

### Version 1.0.0
- Initial release
- Complete C++17 implementation
- CMake build system
- Comprehensive test suite
- Basic and advanced demo applications
- No Python dependencies