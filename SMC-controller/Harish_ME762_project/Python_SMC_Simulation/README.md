# SMC Simulation - Python Translation

This is a Python translation of the MATLAB Sliding Mode Control (SMC) simulation code for autonomous vehicle path tracking.

## Overview

The original MATLAB code implements a kinematic sliding mode controller for vehicle path tracking with the following components:

- **Vehicle Model**: Bicycle model with nonlinear tire dynamics (Pacejka model)
- **Controller**: Kinematic Sliding Mode Controller for rear-axle control
- **Trajectory**: Reference trajectory generation and tracking
- **Simulation**: Complete simulation framework with performance metrics

## Project Structure

```
Python_SMC_Simulation/
├── models/                 # Vehicle models and parameters
│   ├── __init__.py
│   ├── vehicle_parameters.py  # Vehicle parameters (CARparameters.m)
│   ├── vmodel_A.py           # Vehicle dynamics model (vmodel_A.m)
│   └── input_mappings.py     # Input mapping functions
├── controllers/           # Controllers
│   ├── __init__.py
│   └── kinematic_sliding_controller.py  # SMC controller (KinSliding.m)
├── trajectory/            # Trajectory handling
│   ├── __init__.py
│   └── trajectory.py      # Trajectory class
├── utils/                 # Utility functions
│   ├── __init__.py
│   └── math_utils.py      # Math utilities (sign, saturate, etc.)
├── simulation.py          # Main simulation framework (simulate.m)
├── test_smc_simulation.py # Test script (TEST.m)
└── README.md
```

## Key Components

### 1. Vehicle Model (`vmodel_A.py`)

Bicycle model with nonlinear tire dynamics:
- **States**: [X, Y, ψ, vx, vy, ω] (position, heading, velocities)
- **Inputs**: [δ, ω_f, ω_r] (steering angle, wheel angular velocities)
- **Features**: Nonlinear Pacejka tire model, normal force equilibrium

### 2. Sliding Mode Controller (`kinematic_sliding_controller.py`)

Kinematic sliding mode controller with:
- **Control Point**: Rear axle
- **Sliding Surfaces**: Two-dimensional error tracking
- **Gains**: Tunable parameters (k0, k1, k2, p1, p2, q1, q2)

### 3. Trajectory System (`trajectory.py`)

Simplified trajectory generation:
- **Single Lane Change**: Smooth S-curve trajectory
- **Double Lane Change**: More complex double lane change
- **Straight Line**: Default trajectory for testing

### 4. Simulation Framework (`simulation.py`)

Complete simulation environment:
- **ODE Integration**: Using scipy's solve_ivp
- **Performance Metrics**: Tracking errors, tire saturation
- **Results**: Comprehensive benchmark data

## Usage

### Basic Test

```python
from test_smc_simulation import test_smc_simulation

# Run single lane change simulation
benchmark = test_smc_simulation('single_lane_change')

# Run double lane change simulation
benchmark = test_smc_simulation('double_lane_change')
```

### Custom Simulation

```python
import numpy as np
from models.vehicle_parameters import get_default_options
from models.vmodel_A import vmodel_A
from controllers.kinematic_sliding_controller import KinSliding
from trajectory.trajectory import Trajectory
from simulation import simulate, get_vehicle_x0

# Setup
options = get_default_options()
controller = KinSliding()
model = vmodel_A

# Load trajectory
tau = Trajectory()
tau.load('01_single_lane_change')

# Initialize
x0 = get_vehicle_x0(model, tau, options)
controller = controller.init(model, options)

# Run simulation
benchmark = simulate(tau, tau, controller, model, x0, options)
```

## Key Parameters

### Vehicle Parameters (`vehicle_parameters.py`)
- `m`: Mass [kg]
- `l_F`, `l_R`: Distance to front/rear axle [m]
- `mu0`: Friction coefficient
- `B_F`, `C_F`: Front tire parameters
- `B_R`, `C_R`: Rear tire parameters
- `MAX_delta`: Maximum steering angle [rad]

### Controller Parameters (`kinematic_sliding_controller.py`)
- `k0`, `k1`, `k2`: Sliding surface gains
- `p1`, `p2`, `q1`, `q2`: Controller gains

## Results

The simulation provides:
- **Tracking Errors**: Lateral and heading errors
- **Performance Metrics**: Max/average errors, tire saturation
- **Vehicle States**: Position, velocity, yaw rate
- **Control Inputs**: Steering angle, wheel speeds

## Dependencies

- numpy
- scipy
- matplotlib (for visualization)

## Notes

### Differences from Original MATLAB Code

1. **Trajectory Generation**: Simplified from polynomial-based to function-based trajectories
2. **ODE Solver**: Uses scipy's solve_ivp instead of MATLAB's ode45
3. **Data Loading**: MATLAB .mat files replaced with programmatic trajectory generation
4. **Visualization**: Python matplotlib instead of MATLAB plotting

### Extensions Needed

1. **Full Trajectory Loading**: Implement .mat file loading for exact trajectory reproduction
2. **Internal Dynamics**: Add solveID() method for more accurate trajectory tracking
3. **Discrete Controller**: Implement discrete-time controller option
4. **Advanced Scenarios**: Add more complex trajectory scenarios

## Original MATLAB Code Reference

The translation is based on:
- `simulate.m`: Main simulation framework
- `KinSliding.m`: Sliding mode controller
- `vmodel_A.m`: Vehicle dynamics model
- `CARparameters.m`: Vehicle parameters
- `Trajectory.m`: Trajectory handling

## License

This code is a translation of the original MATLAB implementation for educational and research purposes.
