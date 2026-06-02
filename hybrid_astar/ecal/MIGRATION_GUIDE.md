# Migration Guide: From Obstacle Map to Collision Detector

This guide helps you migrate from the old hardcoded obstacle map interface to the new flexible collision detector interface.

## Quick Migration

### Old Code
```cpp
#include "hybrid_astar.hpp"

// Create planner
HybridAStar planner(config);

// Set obstacle map
std::vector<std::vector<int>> obstacle_map = /* your obstacle map */;
double origin_x = 0.0;
double origin_y = 0.0;
planner.set_obstacle_map(obstacle_map, origin_x, origin_y);
```

### New Code
```cpp
#include "hybrid_astar.hpp"
#include "collision_detector.hpp"

// Create planner
HybridAStar planner(config);

// Set collision detector
std::vector<std::vector<int>> obstacle_map = /* your obstacle map */;
double origin_x = 0.0;
double origin_y = 0.0;
auto collision_detector = std::make_shared<GridCollisionDetector>(
    obstacle_map, config.grid_resolution, origin_x, origin_y);
planner.set_collision_detector(collision_detector);
```

## What Changed

1. **Header File**: Added `#include "collision_detector.hpp"`
2. **Method Name**: `set_obstacle_map()` → `set_collision_detector()`
3. **Parameters**: Wrapped obstacle map in a `GridCollisionDetector` object
4. **Flexibility**: Can now use different collision detection algorithms

## Benefits of the New Interface

1. **Modularity**: Swap collision detection algorithms easily
2. **Extensibility**: Add custom collision detection without modifying core code
3. **Performance**: Optimize collision detection for your specific use case
4. **Testing**: Test collision detection separately from path planning

## Migration Steps

1. **Add Include**: Add `#include "collision_detector.hpp"` to your source files
2. **Replace Method Call**: Replace `planner.set_obstacle_map()` with `planner.set_collision_detector()`
3. **Wrap Obstacle Map**: Create a `GridCollisionDetector` object with your obstacle map
4. **Compile and Test**: Build and verify functionality

## Alternative Collision Detectors

### Geometric Collision Detector
For simple geometric environments:
```cpp
auto geometric_detector = std::make_shared<GeometricCollisionDetector>(0.5);  // vehicle radius
geometric_detector->add_circle_obstacle(3.0, 3.0, 1.0);     // circle obstacle
geometric_detector->add_rectangle_obstacle(1.0, 5.0, 2.0, 7.0);  // rectangle obstacle
planner.set_collision_detector(geometric_detector);
```

### Custom Collision Detector
For advanced use cases:
```cpp
class MyCustomDetector : public CollisionDetector {
public:
    bool is_collision_free(const State& state) const override {
        // Your custom collision detection logic
        return true;  // or false if collision detected
    }
};

auto custom_detector = std::make_shared<MyCustomDetector>();
planner.set_collision_detector(custom_detector);
```

## Troubleshooting

### Compile Errors
- **Error**: `'class hybrid_astar::HybridAStar' has no member named 'set_obstacle_map'`
  - **Solution**: Replace with `set_collision_detector()` and create a `GridCollisionDetector`

- **Error**: `'GridCollisionDetector' is not a member of 'hybrid_astar'`
  - **Solution**: Add `#include "collision_detector.hpp"`

### Runtime Issues
- **Issue**: Path planning behaves differently
  - **Check**: Ensure grid resolution matches between old and new implementations
  - **Check**: Verify obstacle map coordinates and origin are correct

### Performance
- **Grid-based**: Similar performance to old implementation
- **Geometric**: Faster for simple environments with few obstacles
- **Custom**: Performance depends on your implementation

For detailed examples, see `collision_detector_example.cpp` and the documentation in `COLLISION_DETECTION.md`.
