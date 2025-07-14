# Sliding Mode Control (SMC) Implementation

This directory contains a complete implementation of Sliding Mode Control for vehicle position and velocity control to the origin.

## Overview

The sliding mode controller is designed to bring a vehicle from an initial state `(p₀, v₀)` to the origin `(0, 0)` with zero velocity, while being robust against bounded disturbances.

### System Dynamics

The controlled system follows double integrator dynamics:
- `ṗ(t) = v(t)` (position derivative equals velocity)
- `v̇(t) = a(t) + d(t)` (velocity derivative equals acceleration plus disturbance)

where:
- `p(t)`: position
- `v(t)`: velocity  
- `a(t)`: control input (acceleration)
- `d(t)`: bounded disturbance with `|d(t)| ≤ D`

### Sliding Mode Controller Design

#### Sliding Surface
The sliding surface is defined as:
```
s(t) = p(t) + λv(t)
```

where `λ > 0` is the sliding surface slope parameter that determines the convergence characteristics.

#### Control Law
The control law consists of two components:

1. **Equivalent Control** (maintains sliding when no disturbance):
   ```
   a_eq(t) = -(1/λ)v(t)
   ```

2. **Switching Control** (provides robustness against disturbances):
   ```
   a_sw(t) = -K · sat(s(t)/Φ)
   ```

3. **Total Control Input**:
   ```
   a(t) = a_eq(t) + a_sw(t) = -(1/λ)v(t) - K · sat(s(t)/Φ)
   ```

#### Parameters

| Parameter | Description | Typical Range |
|-----------|-------------|---------------|
| `λ` | Sliding surface slope | 0.5 - 2.0 |
| `K` | Switching gain (must be > D) | 1.5 - 3.0 |
| `Φ` | Boundary layer thickness | 0.05 - 0.2 |

### Stability Analysis

The controller ensures finite-time convergence to the origin using Lyapunov stability theory:

- **Lyapunov Function**: `V(t) = 0.5 · s(t)²`
- **Stability Condition**: `K > D` (switching gain exceeds disturbance bound)
- **Convergence**: The sliding surface `s(t)` reaches zero in finite time, after which the system follows the trajectory `v(t) = -(1/λ)p(t)` to the origin

## Files

- `sliding_mode_control.py`: Main SMC implementation with classes:
  - `SlidingModeController`: The sliding mode controller
  - `VehicleSystem`: Vehicle dynamics simulation
  - `SMCSimulator`: Complete simulation framework

- `smc_example.py`: Example scripts demonstrating:
  - Basic SMC without disturbance
  - SMC with sinusoidal disturbance
  - Parameter comparison studies

- `__init__.py`: Package initialization
- `README.md`: This documentation

## Usage

### Basic Usage

```python
from control import SlidingModeController, VehicleSystem, SMCSimulator

# Create controller
controller = SlidingModeController(
    lambda_param=1.0,  # Sliding surface slope
    K=2.0,            # Switching gain
    phi=0.1           # Boundary layer thickness
)

# Create vehicle system
vehicle = VehicleSystem(
    initial_position=10.0,  # Start at 10m
    initial_velocity=5.0    # Start with 5m/s
)

# Run simulation
simulator = SMCSimulator(controller, vehicle)
results = simulator.run_simulation()

print(f"Converged: {results['converged']}")
print(f"Final time: {results['final_time']:.2f}s")
```

### With Disturbance

```python
# Define disturbance function
disturbance_func = lambda t: 0.8 * np.sin(2 * np.pi * 0.5 * t)

# Create vehicle with disturbance
vehicle = VehicleSystem(
    initial_position=10.0,
    initial_velocity=5.0,
    disturbance_function=disturbance_func
)

# Run simulation (controller automatically handles disturbance)
simulator = SMCSimulator(controller, vehicle)
results = simulator.run_simulation()
```

## Running Examples

To run the complete examples:

```bash
cd control
python smc_example.py
```

Or run the main simulation:

```bash
python sliding_mode_control.py
```

## Features

- **Robust Control**: Handles bounded disturbances automatically
- **Finite-Time Convergence**: Guaranteed convergence to origin
- **Chattering Reduction**: Uses `tanh` function instead of `sign` function
- **Actuator Limits**: Respects acceleration constraints
- **Comprehensive Visualization**: Plots position, velocity, phase portrait, control input, sliding surface, and Lyapunov function
- **Parameter Studies**: Easy comparison of different parameter settings
- **Multiple Disturbance Types**: Support for various disturbance patterns

## Theory Background

The implementation is based on the sliding mode control theory where:

1. **Reaching Phase**: The system state reaches the sliding surface `s(t) = 0`
2. **Sliding Phase**: The system slides along the surface towards the origin
3. **Convergence**: The system converges to the origin following the trajectory defined by the sliding surface

The key advantages of SMC are:
- **Robustness**: Insensitive to matched disturbances and model uncertainties
- **Finite-Time Convergence**: Reaches the sliding surface in finite time
- **Order Reduction**: The system behavior is determined by the sliding surface design

## Mathematical Verification

The controller satisfies the sliding condition:
```
s(t) · ṡ(t) < 0  (when s(t) ≠ 0)
```

This ensures that the system state moves towards the sliding surface, providing the reaching condition for sliding mode control. 