# SMC Controller for Vehicle Trajectory Tracking

This package provides a Python implementation of Sliding Mode Control (SMC) for autonomous vehicle trajectory tracking. It has been converted from MATLAB code and provides both kinematic and dynamic sliding mode controllers.

## Features

- **Kinematic Sliding Mode Controller**: Tracks trajectories using sliding mode control theory
- **Bicycle Vehicle Model**: Realistic vehicle dynamics with nonlinear tire characteristics
- **Trajectory Support**: Flexible trajectory representation and transformation
- **Comprehensive Simulation**: Full simulation framework with performance benchmarking
- **Noise Support**: Measurement noise simulation for robustness testing
- **Visualization**: Built-in plotting and analysis tools

## Installation

```bash
cd /path/to/smc_controller
pip install -e .
```

## Quick Start

```python
import numpy as np
from smc_controller import KinSlidingController, VehicleModel, create_test_trajectory, simulate, default_options

# Create a circular trajectory
trajectory = create_test_trajectory()

# Initialize controller and vehicle model
controller = KinSlidingController()
model = VehicleModel()

# Set initial state [x, y, psi, vx, vy, omega, ax]
x0 = np.array([0.0, 50.0, np.pi/2, 10.0, 0.0, 0.0, 0.0])

# Run simulation
options = default_options()
result = simulate(trajectory, trajectory, controller, model, x0, options)

# Print results
print(f"Max tracking error: {result['max_error']}")
print(f"Average tracking error: {result['avg_error']}")
```

## Examples

Run the basic demonstration:

```bash
python -m smc_controller.examples.basic_demo
```

Run comprehensive tests:

```bash
python -m smc_controller.examples.comprehensive_test
```

## Components

### Controllers

- `KinSlidingController`: Kinematic sliding mode controller for trajectory tracking

### Vehicle Models

- `VehicleModel`: Bicycle model with nonlinear tire characteristics

### Trajectories

- `Trajectory`: Flexible trajectory representation with coordinate transformations

### Utilities

- `get_local_error`: Calculate tracking error in local vehicle frame
- `sgn`: Analytical sign approximation function
- `saturate`: Value saturation between bounds

## API Reference

### KinSlidingController

Main controller class for kinematic sliding mode control.

```python
controller = KinSlidingController()
controller.k0 = 0.05  # Control gains
controller.k1 = 0.25
controller.k2 = 0.5
```

### VehicleModel

Vehicle dynamics model.

```python
from smc_controller import VehicleModel, CARParameters

params = CARParameters.default()
model = VehicleModel(params)
```

### Trajectory

Trajectory representation.

```python
from smc_controller import Trajectory

# Create custom trajectory
traj = Trajectory()
traj.T = 10.0
traj.X = lambda t: 10 * t  # X position as function of time
traj.Y = lambda t: 0.0     # Y position
# ... define other trajectory functions
```

### Simulation

Main simulation function.

```python
from smc_controller import simulate, default_options

options = default_options()
result = simulate(trajectory, desired_trajectory, controller, model, x0, options)
```

## Parameters

The `CARParameters` class contains all vehicle parameters:

- **Mass & Inertia**: `m`, `J`
- **Geometry**: `l_F`, `l_R`, `L`, `h`
- **Tire Parameters**: `B_F`, `C_F`, `B_R`, `C_R`, `mu0`
- **Controller Gains**: `kA0`, `kA1`, `cf`, `cr`
- **Limits**: `MAX_delta`

## Performance Metrics

The simulation returns comprehensive performance metrics:

- **Tracking Error**: Maximum and average lateral/longitudinal errors
- **Tire Saturation**: Average tire force utilization
- **Control Effort**: Steering angle and acceleration commands
- **Computational Performance**: Simulation timing

## License

This code is converted from MATLAB and maintains compatibility with the original research work.

## References

This implementation is based on the kinematic sliding mode control approach for vehicle trajectory tracking, featuring:

- Sliding surface design for trajectory tracking
- Lyapunov stability analysis
- Nonlinear tire force modeling
- Coordinate frame transformations
- Robust control design

For more details, see the original research papers on sliding mode control for autonomous vehicles.
