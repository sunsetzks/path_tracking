# Hybrid A* Path Planning

A comprehensive collection of Hybrid A* path planning algorithms with multiple implementations.

## 📁 Directory Structure

```
hybrid_astar/
├── python/              # Pure Python implementation with Foxglove visualization
├── cpp/                 # Complete C++ core algorithm (modular design)
├── cpp_bindings/        # C++ with Python bindings (pybind11)
├── ecal/                # eCAL middleware integration
├── docs/                # Documentation
├── pyproject.toml       # Python package configuration
└── README.md            # This file
```

## 🔧 Implementations

### 1. Python Implementation (`python/`)

Pure Python implementation with rich visualization support.

**Features:**
- Full Hybrid A* algorithm
- Foxglove Studio visualization
- Interactive controls (Space to pause, Q to quit, D for diagnostics)
- Steering angle cost, turning cost, cusp cost

**Usage:**
```python
from python.astar_project.hybrid_astar import HybridAStar, State, DirectionMode

# Create planner
planner = HybridAStar(
    resolution=0.1,
    max_steer_angle=30.0
)

# Plan path
path = planner.plan(start, goal, obstacles)
```

**Run Demo:**
```bash
cd python
python examples/demo.py
```

### 2. C++ Core (`cpp/`)

Modular C++ implementation with clean architecture.

**Features:**
- Complete algorithm implementation
- Modular collision detection (grid, geometric, custom)
- Vehicle model abstraction
- No visualization dependencies

**Components:**
- `HybridAStar` - Main algorithm class
- `VehicleModel` - Vehicle kinematics
- `CollisionDetector` - Collision detection interface
- `ObstacleMap` - Obstacle management

**Build:**
```bash
cd cpp
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### 3. C++ Bindings (`cpp_bindings/`)

C++ implementation with Python bindings for performance-critical applications.

**Features:**
- pybind11 bindings
- Same API as Python implementation
- Automatic type conversion

**Build & Install:**
```bash
cd cpp_bindings
pip install -e .
```

**Usage:**
```python
import hybrid_astar_cpp

# Use the same API
planner = hybrid_astar_cpp.HybridAStar(...)
```

### 4. eCAL Integration (`ecal/`)

Middleware integration for distributed systems.

**Features:**
- eCAL middleware support
- Real-time visualization
- Protobuf message protocol
- Distributed system architecture

**Dependencies:**
- `hybrid_astar_core` (C++ core)
- eCAL middleware
- Foxglove Studio

## 🚀 Quick Start

### Prerequisites

```bash
# Activate virtual environment
source /home/zks/ws/path_tracking_experiments/.venv/bin/activate

# Install Python dependencies
pip install -e .
```

### Running Examples

```bash
# Python visualization demo
cd python/examples
python demo.py

# C++ core demo
cd cpp/examples
mkdir build && cd build
cmake .. && make
./basic_demo

# C++ bindings demo
cd cpp_bindings
python python_demo.py
```

## 🧪 Testing

```bash
# Python tests
cd python
pytest tests/

# C++ tests
cd cpp
cd build
ctest

# C++ bindings tests
cd cpp_bindings
pytest tests/
```

## 📊 Visualization

### Foxglove Studio

The Python implementation includes Foxglove visualization:

```bash
# Start Foxglove server
python python/astar_project/foxglove_visualizer.py

# Open Foxglove Studio
# Connect to ws://localhost:8765
```

### Matplotlib Fallback

If Foxglove is not available, matplotlib visualization is used automatically.

## 🏗️ Architecture

### State Representation

```python
@dataclass
class State:
    x: float          # X position
    y: float          # Y position
    yaw: float        # Heading angle (radians)
    direction: DirectionMode  # FORWARD, BACKWARD, NONE
    steer: float      # Steering angle (radians)
```

### Cost Components

- **Distance**: Path length cost
- **Steering**: Steering angle magnitude
- **Turning**: Turning rate
- **Cusp**: Direction change penalty

### Collision Detection

- **GridCollisionDetector**: Grid-based detection
- **GeometricCollisionDetector**: Shape-based detection
- **CustomDetector**: User-defined interface

## 📚 Documentation

See the `docs/` directory for detailed documentation:
- [Algorithm Overview](docs/algorithm.md)
- [API Reference](docs/api.md)
- [Configuration Guide](docs/configuration.md)

## 🔗 Integration

### With PathTracking Library

```python
from PathTracking.vehicle_model import VehicleModel, VehicleState
from astar_project.hybrid_astar import HybridAStar, State

# Convert between state representations
vehicle_state = VehicleState(...)
planning_state = State(
    x=vehicle_state.position_x,
    y=vehicle_state.position_y,
    yaw=vehicle_state.yaw_angle,
    direction=DirectionMode.FORWARD,
    steer=vehicle_state.steering_angle
)
```

### With eCAL

```cpp
#include "hybrid_astar.hpp"
#include "visualization_publisher.hpp"

// Create planner and publisher
HybridAStar planner(config);
VisualizationPublisher publisher;

// Plan and publish
auto path = planner.plan(start, goal);
publisher.publishPath(path);
```

## 🛠️ Development

### Code Quality

```bash
# Format code
black .

# Sort imports
isort .

# Lint
flake8 .
```

### Pre-commit Hooks

```bash
pre-commit install
```

## 📄 License

MIT License

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## 📞 Support

For issues and questions, please open an issue on GitHub.
