# Collision Detection Interface for Hybrid A*

This directory contains a flexible collision detection interface that allows users to implement custom collision detection algorithms for the Hybrid A* path planner.

## Overview

The collision detection system has been refactored from a hard-coded obstacle map approach to a modular interface that supports different types of collision detection:

1. **GridCollisionDetector**: For traditional 2D grid-based obstacle maps
2. **GeometricCollisionDetector**: For simple geometric shapes (circles and rectangles)
3. **Custom Detectors**: Users can implement their own collision detection by inheriting from `CollisionDetector`

## Interface Design

### Base Class: `CollisionDetector`

```cpp
class CollisionDetector {
public:
    virtual ~CollisionDetector() = default;
    
    // Check if a single state is collision-free
    virtual bool is_collision_free(const State& state) const = 0;
    
    // Check if an entire trajectory is collision-free (default implementation provided)
    virtual bool is_trajectory_collision_free(const std::vector<State>& trajectory) const;
};
```

### Grid-Based Collision Detection

```cpp
// Create a grid collision detector
std::vector<std::vector<int>> obstacle_map(10, std::vector<int>(10, 0));
// Add obstacles (1 = obstacle, 0 = free)
obstacle_map[5][5] = 1;

auto grid_detector = std::make_shared<GridCollisionDetector>(
    obstacle_map, 
    0.5,    // grid resolution in meters
    0.0,    // origin x
    0.0     // origin y
);

// Set in planner
HybridAStar planner(config);
planner.set_collision_detector(grid_detector);
```

### Geometric Collision Detection

```cpp
// Create geometric collision detector
auto geometric_detector = std::make_shared<GeometricCollisionDetector>(0.5);  // vehicle radius

// Add obstacles
geometric_detector->add_circle_obstacle(3.0, 3.0, 1.0);           // Circle at (3,3), radius 1m
geometric_detector->add_rectangle_obstacle(1.0, 5.0, 2.0, 7.0);   // Rectangle from (1,5) to (2,7)

// Set in planner
planner.set_collision_detector(geometric_detector);
```

### Custom Collision Detection

You can implement your own collision detection by inheriting from `CollisionDetector`:

```cpp
class CustomCollisionDetector : public CollisionDetector {
public:
    bool is_collision_free(const State& state) const override {
        // Your custom collision detection logic here
        return true;  // or false if collision detected
    }
};

auto custom_detector = std::make_shared<CustomCollisionDetector>();
planner.set_collision_detector(custom_detector);
```

## Migration from Old Interface

### Before (Hard-coded Obstacle Map)
```cpp
HybridAStar planner(config);
planner.set_obstacle_map(obstacle_map, origin_x, origin_y);
```

### After (Flexible Collision Detection)
```cpp
HybridAStar planner(config);
auto detector = std::make_shared<GridCollisionDetector>(obstacle_map, grid_resolution, origin_x, origin_y);
planner.set_collision_detector(detector);
```

## Benefits

1. **Modularity**: Different collision detection algorithms can be easily swapped
2. **Extensibility**: Custom collision detection implementations can be added without modifying core code
3. **Performance**: Specialized detectors can be optimized for specific use cases
4. **Testing**: Easier to unit test collision detection separately from path planning
5. **Reusability**: Collision detectors can be used across different planners

## Examples

See `collision_detector_example.cpp` for a complete demonstration of:
- Grid-based collision detection
- Geometric collision detection  
- Planning with different collision detectors
- Performance statistics

## Building and Running

```bash
# Build
mkdir build && cd build
cmake ..
make

# Run the collision detector example
./examples/collision_detector_example
```

## Performance Considerations

- **GridCollisionDetector**: Fast for large environments with many obstacles
- **GeometricCollisionDetector**: Fast for simple geometric environments
- **Custom Detectors**: Performance depends on implementation

The interface also supports trajectory-level collision checking, which can be optimized in custom implementations for better performance.
