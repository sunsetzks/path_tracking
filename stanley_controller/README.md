# Stanley Controller Implementation

A comprehensive Stanley controller implementation for autonomous vehicle path tracking with simulation capabilities.

## Overview

This package provides an enhanced Stanley controller implementation with the following features:

- **Enhanced Stanley Controller**: Improved version of the classic Stanley algorithm with better performance
- **Realistic Vehicle Dynamics**: Bicycle model with tire dynamics and actuator limitations
- **Comprehensive Simulation Environment**: Support for obstacles, multiple scenarios, and performance evaluation
- **Advanced Visualization**: Real-time plotting, animations, and performance metrics
- **Multiple Example Scenarios**: Basic path following, obstacle avoidance, and comprehensive testing

## Features

### Core Components

1. **Stanley Controller** (`stanley_controller.py`)
   - Enhanced control algorithm with configurable parameters
   - Cross-track error and heading error correction
   - Speed control integration
   - Path tracking utilities

2. **Vehicle Dynamics** (`vehicle_dynamics.py`)
   - Realistic bicycle model with tire slip dynamics
   - Actuator dynamics (steering, throttle, brake)
   - Aerodynamic drag effects
   - Configurable vehicle parameters

3. **Simulation Environment** (`simulation.py`)
   - Multi-scenario support (straight line, circular, figure-eight, highway)
   - Obstacle avoidance capabilities
   - Performance metrics calculation
   - Batch evaluation tools

4. **Visualization** (`visualization.py`)
   - Real-time trajectory plotting
   - Vehicle animation with proper steering visualization
   - Performance dashboard
   - Error analysis plots

### Key Improvements

- **Enhanced Control Algorithm**: Improved gain scheduling and lookahead distance
- **Realistic Dynamics**: Tire slip, actuator lag, and physical constraints
- **Comprehensive Testing**: Multiple scenarios and performance evaluation
- **Professional Visualization**: Publication-quality plots and animations

## Installation

### Prerequisites

- Python 3.7+
- NumPy
- Matplotlib
- SciPy

### Install Dependencies

```bash
pip install numpy matplotlib scipy
```

### Package Structure

```
stanley_controller/
├── __init__.py
├── stanley_controller.py      # Main controller implementation
├── vehicle_dynamics.py       # Vehicle dynamics models
├── simulation.py             # Simulation environment
├── visualization.py          # Visualization tools
├── utils/                    # Utility functions
│   ├── __init__.py
│   ├── angle.py              # Angle utilities
│   └── se2.py                # SE2 transformation utilities
└── examples/                 # Example scripts
    ├── __init__.py
    ├── basic_demo.py         # Basic path following demo
    ├── obstacle_avoidance_demo.py  # Obstacle avoidance demo
    └── comprehensive_test.py # Comprehensive test suite
```

## Quick Start

### Basic Usage

```python
import numpy as np
from stanley_controller import StanleyController, VehicleDynamics, SimulationEnvironment
from stanley_controller.utils.se2 import SE2
from stanley_controller.stanley_controller import ControlParams

# Create controller
control_params = ControlParams(
    k_cross_track=0.5,
    k_heading=1.0,
    max_steer_angle=np.radians(30.0),
    wheelbase=2.9
)
controller = StanleyController(control_params)

# Create vehicle dynamics
vehicle_dynamics = VehicleDynamics()

# Create simulation environment
config = SimulationConfig(dt=0.1, max_time=50.0)
sim_env = SimulationEnvironment(config)

# Define path and initial state
path_points = np.array([[0, 0], [10, 0], [20, 10], [30, 10]])
path_yaw = np.array([0, 0, np.pi/4, np.pi/4])
initial_state = SE2(x=0, y=0, theta=0)

# Run simulation
result = sim_env.simulate(
    controller, vehicle_dynamics, initial_state,
    path_points, path_yaw, target_speed=5.0
)

print(f"Success: {result['success']}")
print(f"Time: {result['time']:.2f} s")
```

### Running Examples

#### Basic Demo

```bash
cd stanley_controller/examples
python basic_demo.py
```

This demonstrates basic path following on a circular trajectory.

#### Obstacle Avoidance Demo

```bash
cd stanley_controller/examples
python obstacle_avoidance_demo.py
```

This shows the controller navigating around obstacles.

#### Comprehensive Test

```bash
cd stanley_controller/examples
python comprehensive_test.py
```

This runs multiple scenarios and generates detailed performance analysis.

## API Reference

### StanleyController

```python
class StanleyController:
    def __init__(self, params: ControlParams)
    def compute_control(self, state, path_points, path_yaw, target_speed) -> Tuple[float, float, int]
```

**Parameters:**
- `params`: Control parameters including gains and limits

**Returns:**
- Steering angle, acceleration, target index

### VehicleDynamics

```python
class VehicleDynamics:
    def __init__(self, params: VehicleParameters)
    def update(self, dt, steering_command, throttle_command, brake_command)
    def get_state(self) -> SE2
```

### SimulationEnvironment

```python
class SimulationEnvironment:
    def __init__(self, config: SimulationConfig)
    def add_obstacle(self, x, y, radius)
    def simulate(self, controller, vehicle_dynamics, initial_state, path_points, path_yaw, target_speed)
```

### Visualizer

```python
class Visualizer:
    def plot_trajectory_comparison(self, results, scenarios)
    def animate_simulation(self, result, reference_path)
    def plot_error_analysis(self, result)
    def create_dashboard(self, results)
```

## Configuration

### Control Parameters

```python
@dataclass
class ControlParams:
    k_cross_track: float = 0.5      # Cross-track error gain
    k_heading: float = 1.0          # Heading error gain
    max_steer_angle: float = 0.5    # Maximum steering angle (rad)
    max_acceleration: float = 2.0   # Maximum acceleration (m/s²)
    max_deceleration: float = -3.0  # Maximum deceleration (m/s²)
    wheelbase: float = 2.9          # Vehicle wheelbase (m)
    lookahead_distance: float = 5.0 # Lookahead distance (m)
```

### Vehicle Parameters

```python
@dataclass
class VehicleParameters:
    wheelbase: float = 2.9          # Distance between axles (m)
    track_width: float = 1.6        # Distance between wheels (m)
    mass: float = 1500.0            # Vehicle mass (kg)
    inertia_z: float = 2875.0       # Yaw moment of inertia (kg⋅m²)
    cornering_stiffness_front: float = 80000.0  # Front tire stiffness (N/rad)
    cornering_stiffness_rear: float = 100000.0  # Rear tire stiffness (N/rad)
```

## Scenarios

The package includes several built-in scenarios:

1. **Straight Line**: Basic straight path following
2. **Circular Path**: Circular trajectory tracking
3. **Figure Eight**: Complex path with crossovers
4. **Highway**: Curved highway with multiple lanes
5. **Parking**: Parking maneuver scenario

## Performance Metrics

The simulation provides comprehensive performance metrics:

- **Success Rate**: Percentage of successful path completions
- **Tracking Error**: Average distance from reference path
- **Completion Time**: Time to complete the path
- **Control Smoothness**: Steering rate and acceleration metrics
- **Collision Rate**: Frequency of obstacle collisions

## Visualization Features

- **Real-time Animation**: Vehicle movement with proper steering visualization
- **Trajectory Comparison**: Multiple runs on the same plot
- **Error Analysis**: Distance and heading error plots
- **Performance Dashboard**: Comprehensive overview of results
- **Control Input Plots**: Steering and acceleration over time

## Advanced Usage

### Custom Scenarios

```python
def create_custom_scenario():
    # Define custom path points and yaw angles
    path_points = np.array([[x1, y1], [x2, y2], ...])
    path_yaw = np.array([yaw1, yaw2, ...])
    return path_points, path_yaw
```

### Batch Evaluation

```python
from stanley_controller.simulation import BatchEvaluator

evaluator = BatchEvaluator(config)
results = evaluator.evaluate_scenarios(controller, scenarios, speeds, initial_states)
summary = evaluator.summarize_results()
```

### Custom Visualization

```python
from stanley_controller.visualization import Visualizer

visualizer = Visualizer()
visualizer.plot_trajectory_comparison(results, scenarios)
visualizer.create_dashboard(results)
```

## Troubleshooting

### Common Issues

1. **Import Errors**: Ensure all dependencies are installed
2. **Simulation Instability**: Reduce time step or adjust control gains
3. **Poor Tracking Performance**: Tune control parameters or increase lookahead distance
4. **Animation Issues**: Check matplotlib backend compatibility

### Performance Tips

- Use smaller time steps for more accurate simulations
- Adjust control gains based on vehicle characteristics
- Use appropriate lookahead distances for different speeds
- Enable collision checking for obstacle scenarios

## Contributing

This implementation is designed to be extensible. Key areas for enhancement:

- Additional vehicle dynamics models
- More sophisticated control algorithms
- Integration with real vehicle hardware
- Advanced path planning integration
- Machine learning-based parameter tuning

## References

1. **Stanley: The robot that won the DARPA grand challenge**
   - Original paper on the Stanley controller algorithm

2. **Autonomous Automobile Path Tracking**
   - Comprehensive reference on path tracking algorithms

3. **Bicycle Model Dynamics**
   - Standard vehicle dynamics model used in automotive control

## License

This implementation is provided for educational and research purposes. Please ensure compliance with applicable licenses when using in commercial applications.

## Support

For questions, issues, or contributions, please refer to the project documentation or create an issue in the project repository.