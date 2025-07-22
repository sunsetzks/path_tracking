# Vehicle Dynamics Simulation

This project implements a vehicle dynamics simulation using the bicycle model, which is commonly used in automotive control and autonomous driving research. The simulation provides a realistic model of vehicle motion based on physical parameters and control inputs.

## Features

- **Kinematic Bicycle Model**: Implements the standard bicycle model for vehicle dynamics
- **Realistic Vehicle Parameters**: Configurable vehicle dimensions, mass, and dynamic limits
- **Comprehensive Visualization**: Multiple plotting functions for trajectory, state variables, and animation
- **Example Scenarios**: Includes examples for sine wave, step response, and figure-eight trajectories
- **Modular Design**: Easy to extend and modify for specific use cases

## Installation

This project requires Python 3.7+ and the following packages:
- numpy
- matplotlib

Install the required packages using pip:

```bash
pip install numpy matplotlib
```

## Usage

The simulation can be used in several ways:

### 1. Basic Simulation

```python
from vehicle_dynamics import VehicleSimulation, VehicleParameters

# Create simulation with default parameters
params = VehicleParameters()
sim = VehicleSimulation(params)

# Run simulation with constant acceleration and steering rate
for i in range(100):
    # Acceleration = 1.0 m/s², Steering rate = 0.1 rad/s
    sim.step(1.0, 0.1)
```

### 2. Using simulate_trajectory Function

```python
from vehicle_dynamics import simulate_trajectory

def control_function(t, state):
    # Return (acceleration, steering_rate)
    return 1.0, 0.1 * np.sin(0.5 * t)

# Simulate for 20 seconds
sim = VehicleSimulation()
sim = simulate_trajectory(sim, 20.0, control_function)
```

### 3. Visualization

```python
from visualization import plot_vehicle_trajectory, plot_state_variables

# Plot trajectory
fig1 = plot_vehicle_trajectory(sim)

# Plot state variables
fig2 = plot_state_variables(sim)

# Show plots
import matplotlib.pyplot as plt
plt.show()
```

### 4. Animation

```python
from visualization import animate_vehicle_simulation
from matplotlib.animation import FuncAnimation

# Create animation
fig, ax, update_fn = animate_vehicle_simulation(sim, interval=50)
anim = FuncAnimation(fig, update_fn, frames=len(sim.history['t']),
                    interval=50, blit=False, repeat=True)
plt.show()
```

## Vehicle Parameters

The `VehicleParameters` class allows you to configure various vehicle characteristics:

```python
params = VehicleParameters(
    length=4.0,           # Vehicle length [m]
    width=1.8,            # Vehicle width [m]
    wheelbase=2.7,        # Wheelbase [m]
    mass=1500.0,          # Vehicle mass [kg]
    max_steer=0.6,        # Maximum steering angle [rad]
    max_accel=3.0,        # Maximum acceleration [m/s^2]
    max_decel=-6.0,       # Maximum deceleration [m/s^2]
    max_speed=50.0,       # Maximum speed [m/s]
    dt=0.05               # Simulation timestep [s]
)
```

## Vehicle State

The `VehicleState` class represents the current state of the vehicle with the following attributes:
- `x`, `y`: Position coordinates [m]
- `yaw`: Heading angle [rad]
- `v`: Velocity [m/s]
- `steer`: Steering angle [rad]

## Simulation Class

The `VehicleSimulation` class provides the main interface for running simulations:

```python
sim = VehicleSimulation(params)

# Reset simulation to initial state
sim.reset(x=0.0, y=0.0, yaw=0.0, v=0.0, steer=0.0)

# Execute one simulation step
state = sim.step(acceleration=1.0, delta_dot=0.1)

# Get current state as array
state_array = sim.get_state_array()

# Get simulation history
history_array = sim.get_history_array()
```

## Examples

Run the example script to see various simulation scenarios:

```bash
python example_simulation.py
```

This will demonstrate:
- Sine wave trajectory
- Steering step response
- Figure-eight trajectory
- Animation of vehicle motion

## References

1. Rajamani, R. (2011). Vehicle Dynamics and Control. Springer.
2. Kong, J., Pfeiffer, M., Schildbach, G., & Borrelli, F. (2015). Kinematic and dynamic vehicle models for autonomous driving control design.

## License

This project is licensed under the MIT License - see the LICENSE file for details.