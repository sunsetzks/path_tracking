# GitHub Copilot Instructions for Path Tracking Project

## Project Overview
This is a comprehensive Python library for path tracking and vehicle control algorithms, featuring modular components for robotics and autonomous vehicle applications. The project uses a workspace structure with multiple sub-projects managed via `pyproject.toml` and Python virtual environments (venv).

## Key Architecture Patterns

### Configuration System
- **Centralized YAML Config**: Use `PathTracking/config.yaml` as the single source of truth for parameters
- **Dataclass-based**: All configs are dataclasses in `PathTracking/config.py` (VehicleConfig, PurePursuitConfig, etc.)
- **Component-specific**: Each component takes its specific config object: `VehicleModel(config=config.vehicle)`
- **Loading**: Always use `from PathTracking.config import load_config; config = load_config()`

### Multi-Project Workspace
```
PathTracking/        # Core path tracking library (bicycle model, pure pursuit)
astar_project/       # Hybrid A* path planning with Foxglove visualization  
PathSmoothing/       # Cubic splines and gradient-based path smoothing
control/             # Advanced control algorithms (SMC, Kalman, etc.)
experiments/         # Research implementations and comparisons
```

### State Management Patterns
- **VehicleState**: Structured state with `position_x, position_y, yaw_angle, velocity, steering_angle`
- **Hybrid A* State**: Different representation with `x, y, yaw, direction, steer` for planning
- **Conversion**: Use explicit conversion between state representations when crossing boundaries

### Visualization Architecture
- **Matplotlib**: Standard visualization in `PathTracking/` with interactive controls
- **Foxglove**: Real-time 3D visualization in `astar_project/` with WebSocket server and MCAP recording
- **Separation**: Visualization code is modular and optional (graceful ImportError handling)

## Essential Development Workflows

### Environment Setup
```bash
# Activate virtual environment
source venv/bin/activate

# Install dependencies for specific sub-project
pip install -e PathTracking/
pip install -e astar_project/
pip install -e PathSmoothing/
```

### Running Examples
```bash
# Ensure virtual environment is activated
source venv/bin/activate

# Pure pursuit simulation with interactive GUI
python PathTracking/examples/pure_pursuit_examples.py --simulation_choice 2

# Hybrid A* with Foxglove visualization 
python astar_project/examples/demo.py

# Vehicle model testing with comprehensive plots
python PathTracking/examples/vehicle_model_example.py
```

### Testing Patterns
```bash
# Ensure virtual environment is activated
source venv/bin/activate

# Run tests from project root
pytest tests/
```
- **pytest**: Use `pytest tests/` from project root (with venv activated)
- **Coverage**: Configured for HTML reports in `htmlcov/`
- **Integration tests**: Focus on end-to-end workflows, not just unit tests
- **Test structure**: Group by component (TestVehicleModel, TestHybridAStar, etc.)

### Code Quality Tools
```bash
# Ensure virtual environment is activated first
source venv/bin/activate

# Pre-commit hooks configured for:
black --line-length=88    # Formatting
isort --profile=black     # Import sorting
flake8                    # Linting with custom config in .flake8
```

## Project-Specific Conventions

### Import Patterns
```python
# Configuration loading (always first)
from PathTracking.config import load_config
config = load_config()

# Component imports
from PathTracking.vehicle_model import VehicleModel, VehicleState
from astar_project.hybrid_astar import HybridAStar, State, DirectionMode
```

### Error Handling
- **Graceful degradation**: Optional dependencies with try/except (Foxglove, scipy)
- **Fallback visualization**: matplotlib_fallback_visualization() when Foxglove unavailable
- **State validation**: Check trajectory continuity, collision-free paths

### Logging Standards
- **loguru**: Structured logging with VSCode-clickable paths
- **Setup**: Use `from PathTracking.utils.logger_config import setup_logger; setup_logger()`
- **Levels**: debug/info for normal flow, warning for conflicts, error for failures

## Integration Points

### State Transitions
- **PathTracking → astar_project**: Convert VehicleState to planning State
- **Planning → Control**: Extract detailed paths using `extract_detailed_path()`
- **Visualization**: Both matplotlib and Foxglove accept same data structures

### Configuration Flow
1. Load from YAML with `load_config()`
2. Customize specific sections: `config.vehicle.wheelbase = 3.0`
3. Pass to components: `VehicleModel(config=config.vehicle)`
4. Components use dataclass fields directly

### Performance Considerations
- **Large datasets**: Filter exploration nodes for visualization (max_exploration_nodes)
- **Real-time**: Use background processes for Foxglove server
- **Memory**: MCAP recording to `logs/` directory with timestamps

## Critical Files for Context
- `PathTracking/config.yaml` - Central configuration
- `PathTracking/vehicle_model.py` - Core kinematic model  
- `astar_project/hybrid_astar.py` - Path planning algorithm
- `astar_project/foxglove_visualizer.py` - Real-time visualization
- `PathTracking/examples/pure_pursuit_examples.py` - Reference implementation patterns

## Debug & Development Tips
- **Interactive simulations**: Use Space to pause, Q to quit, D for diagnostics
- **Path analysis**: Check trajectory continuity with distance calculations between waypoints
- **Visualization issues**: Verify FOXGLOVE_AVAILABLE flag, fallback to matplotlib
- **Configuration debugging**: Print config objects with `pformat(config.__dict__)`
