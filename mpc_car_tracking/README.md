# MPC Car Tracking

A comprehensive implementation of Model Predictive Control (MPC) for vehicle trajectory tracking using Python. This package provides a complete framework for vehicle dynamics modeling, MPC controller design, trajectory planning, simulation, and visualization.

## Features

- **Vehicle Dynamics**: Kinematic bicycle model with physical constraints
- **MPC Controller**: Optimization-based controller using CVXPY
- **Trajectory Planning**: Multiple trajectory types (straight, circular, figure-8, lane change, spline)
- **Simulation Environment**: Complete simulation framework with disturbances
- **Visualization**: Rich plotting and animation capabilities
- **Scenarios**: Predefined test scenarios for validation

## Installation

### Prerequisites

- Python 3.12 or higher
- pip or conda package manager

### Install Dependencies

```bash
pip install numpy scipy matplotlib cvxpy control
```

### Install the Package

```bash
# Clone and install in development mode
git clone <repository-url>
cd mpc_car_tracking
pip install -e .
```

## Quick Start

### Run Demo Scenarios

```bash
# Run circular track demo
python -m mpc_car_tracking --scenario circular_track

# Run all scenarios with comparison
python -m mpc_car_tracking --scenario all

# Run without plots (for headless environments)
python -m mpc_car_tracking --scenario straight_line --no-plots

# Save plots to files
python -m mpc_car_tracking --scenario figure_eight --save-plots --plot-dir ./results
```

### Available Scenarios

- `straight_line`: Simple straight-line trajectory tracking
- `circular_track`: Circular path following
- `figure_eight`: Figure-8 pattern tracking
- `lane_change`: Highway lane change maneuver
- `slalom`: Slalom course navigation
- `all`: Run all scenarios with comparison

### Python API Usage

```python
import mpc_car_tracking as mpc

# Run a demo scenario
result = mpc.run_demo('circular_track', show_plots=True)

# Access simulation data
sim_data = result['simulation_data']
metrics = result['metrics']

print(f"Max tracking error: {metrics['max_position_error']:.3f} m")
print(f"Mean solve time: {metrics['mean_solve_time']:.3f} s")
```

## Components

### 1. Vehicle Model (`vehicle_model.py`)

Implements the kinematic bicycle model:

```python
from mpc_car_tracking import VehicleState, VehicleControl, BicycleModel

# Create vehicle model
model = BicycleModel()

# Define initial state
state = VehicleState(x=0, y=0, yaw=0, v=10)  # 10 m/s forward

# Apply control input
control = VehicleControl(acceleration=1.0, steering_angle=0.1)
next_state = model.step(state, control, dt=0.1)
```

### 2. MPC Controller (`mpc_controller.py`)

Optimization-based trajectory tracking:

```python
from mpc_car_tracking import MPCController, MPCParameters

# Configure MPC parameters
mpc_params = MPCParameters(
    prediction_horizon=20,
    control_horizon=20,
    dt=0.1
)

# Create controller
controller = MPCController(model, mpc_params)

# Get optimal control
control, result = controller.get_control(current_state, reference_trajectory)
```

### 3. Trajectory Planner (`trajectory_planner.py`)

Generate reference trajectories:

```python
from mpc_car_tracking import TrajectoryPlanner

planner = TrajectoryPlanner()

# Create circular trajectory
trajectory = planner.circular_path(
    center=(0, 0), 
    radius=20, 
    target_speed=10,
    num_points=100
)

# Create lane change maneuver
lane_change = planner.lane_change(
    start_state=VehicleState(x=0, y=0, yaw=0, v=15),
    lane_offset=3.5,
    lane_change_distance=50
)
```

### 4. Simulation (`simulator.py`)

Complete simulation environment:

```python
from mpc_car_tracking import MPCSimulator, SimulationConfig

# Configure simulation
config = SimulationConfig(
    dt=0.1,
    total_time=20.0,
    save_data=True
)

# Run simulation
simulator = MPCSimulator(config)
result = simulator.run_scenario('circular_track')
```

### 5. Visualization (`visualization.py`)

Rich plotting capabilities:

```python
from mpc_car_tracking import MPCVisualizer

visualizer = MPCVisualizer()

# Plot tracking results
fig = visualizer.plot_trajectory_tracking(simulation_data)

# Create animation
anim = visualizer.animate_simulation(simulation_data, vehicle_params)

# Compare scenarios
fig = visualizer.compare_scenarios(scenario_results)
```

## Configuration

### Vehicle Parameters

```python
from mpc_car_tracking import VehicleParameters

params = VehicleParameters(
    wheelbase=2.7,              # Distance between axles (m)
    max_speed=30.0,             # Maximum speed (m/s)
    max_acceleration=3.0,       # Maximum acceleration (m/s²)
    max_deceleration=-5.0,      # Maximum deceleration (m/s²)
    max_steering_angle=π/3,     # Maximum steering angle (rad)
    max_steering_rate=π/2       # Maximum steering rate (rad/s)
)
```

### MPC Parameters

```python
from mpc_car_tracking import MPCParameters
import numpy as np

mpc_params = MPCParameters(
    prediction_horizon=20,      # Prediction steps
    control_horizon=20,         # Control steps
    dt=0.1,                     # Time step (s)
    Q=np.diag([10, 10, 1, 1]),  # State cost weights [x, y, yaw, v]
    R=np.diag([0.1, 1.0]),      # Control cost weights [accel, steering]
    Q_terminal=None,            # Terminal cost (auto-computed if None)
    max_iterations=1000,        # Solver max iterations
    solver_verbose=False        # Solver output
)
```

### Simulation Configuration

```python
from mpc_car_tracking import SimulationConfig, DisturbanceModel

# Configure disturbances
disturbance = DisturbanceModel(
    wind_force=np.array([1.0, 0.5]),    # Constant wind force [fx, fy] (N)
    wind_noise_std=0.1,                 # Wind noise std dev
    measurement_noise_std=0.01,         # State measurement noise
    actuator_delay=0.05,                # Actuator delay (s)
    actuator_noise_std=0.001            # Actuator noise
)

config = SimulationConfig(
    dt=0.1,                     # Simulation time step
    total_time=30.0,            # Total simulation time
    real_time_factor=1.0,       # Real-time factor (0=fastest)
    save_data=True,             # Save simulation data
    disturbance=disturbance     # Disturbance model
)
```

## Testing

Run the test suite:

```bash
# Install pytest
pip install pytest

# Run all tests
pytest tests/

# Run specific test file
pytest tests/test_vehicle_model.py -v

# Run with coverage
pip install pytest-cov
pytest tests/ --cov=src/mpc_car_tracking
```

## Performance Metrics

The package automatically computes performance metrics:

- **Tracking Errors**: Position, heading, and velocity errors
- **Computational Performance**: Solve times and success rates
- **Control Effort**: Control input magnitudes and rates
- **Robustness**: Performance under disturbances

## Examples

### Custom Scenario

```python
import numpy as np
from mpc_car_tracking import *

# Create custom trajectory
planner = TrajectoryPlanner()
waypoints = [(0, 0), (10, 5), (20, 10), (30, 5), (40, 0)]
trajectory = planner.spline_path(waypoints, target_speed=12, num_points=100)

# Setup simulation
config = SimulationConfig(dt=0.1, total_time=15.0)
simulator = MPCSimulator(config)

# Run simulation
initial_state = VehicleState(x=0, y=0, yaw=0, v=12)
reference_states = [tp.state for tp in trajectory]
sim_data = simulator.run_simulation(initial_state, reference_states)

# Visualize results
visualizer = MPCVisualizer()
fig = visualizer.plot_trajectory_tracking(sim_data, title="Custom Scenario")
```

### Parameter Sensitivity Study

```python
# Test different MPC horizons
horizons = [10, 15, 20, 25]
results = {}

for horizon in horizons:
    mpc_params = MPCParameters(prediction_horizon=horizon)
    config = SimulationConfig(mpc_params=mpc_params)
    simulator = MPCSimulator(config)
    
    data = simulator.run_scenario('circular_track')
    metrics = simulator.get_performance_metrics()
    results[f'H={horizon}'] = data

# Compare results
visualizer = MPCVisualizer()
fig = visualizer.compare_scenarios(results, title="Horizon Sensitivity")
```

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make changes with tests
4. Run the test suite
5. Submit a pull request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## References

- Rajamani, R. (2012). Vehicle dynamics and control. Springer.
- Rawlings, J. B., Mayne, D. Q., & Diehl, M. (2017). Model predictive control: theory, computation, and design.
- Kong, J., et al. (2015). Kinematic and dynamic vehicle models for autonomous driving control design.

## Citation

```bibtex
@software{mpc_car_tracking,
  title={MPC Car Tracking: A Python Implementation},
  author={Your Name},
  year={2024},
  url={https://github.com/username/mpc_car_tracking}
}
```