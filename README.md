# Path Tracking Library

A comprehensive Python library for path tracking and vehicle control algorithms, featuring multiple control strategies for autonomous vehicle navigation.

## Features

### Control Algorithms
- **Pure Pursuit Control** - Classic path following algorithm
- **Stanley Control** - Lateral control for path tracking  
- **LQR Controllers** - Linear Quadratic Regulator for speed and steering control
  - Discrete LQR steering control
  - Continuous LQR steering control
  - Combined LQR speed and steering control
- **Model Predictive Control (MPC)** - Speed and steering control with constraints
- **CGMRES NMPC** - Continuous GMRES-based Nonlinear Model Predictive Control
- **Rear Wheel Feedback Control** - Alternative control approach
- **Move to Pose** - Point-to-point navigation control

### Path Planning
- **Cubic Spline Planner** - Smooth path generation with continuity constraints
- **Trajectory Management** - Waypoint handling and trajectory interpolation

### Vehicle Modeling
- **Vehicle Dynamics** - Bicycle model and kinematic models with realistic noise simulation
- **Noise Modeling** - Comprehensive noise simulation including odometry and GPS-like noise models
- **Control Input Noise** - Independent control over actuator noise for testing robustness
- **Vehicle Visualization** - Realistic vehicle display with four wheels and steering

### Utilities
- **Angle Utilities** - Angle normalization and rotation matrices
- **Plotting Tools** - Enhanced visualization capabilities
- **Vehicle Display** - Advanced vehicle rendering

## Installation

### Using UV (Recommended)

```bash
# Install uv if you haven't already
curl -LsSf https://astral.sh/uv/install.sh | sh

# Clone the repository
git clone <your-repo-url>
cd path_tracking

# Create virtual environment and install in editable mode
uv venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
uv pip install -e .
```

### Using pip

```bash
# Clone the repository
git clone <your-repo-url>
cd path_tracking

# Create virtual environment
python -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install in editable mode
pip install -e .
```

## Quick Start

```python
import numpy as np
import matplotlib.pyplot as plt
from PathTracking.pure_pursuit import PurePursuitController
from PathTracking.vehicle_model import VehicleModel
from PathTracking.trajectory import Trajectory, Waypoint

# Create a simple trajectory
waypoints = [
    Waypoint(0, 0, 0, 5),
    Waypoint(10, 0, 0, 5),
    Waypoint(20, 10, np.pi/4, 5),
    Waypoint(30, 20, np.pi/2, 5)
]
trajectory = Trajectory(waypoints)

# Initialize vehicle and controller
vehicle = VehicleModel()
controller = PurePursuitController(lookahead_distance=3.0)

# Simulation loop
x_history, y_history = [], []
for _ in range(100):
    # Get current vehicle state
    state = vehicle.get_state()
    
    # Calculate control input
    steering, target_idx = controller.pure_pursuit_steer_control(
        state, trajectory.get_course()
    )
    
    # Update vehicle
    vehicle.update(acceleration=0.1, steering_angle=steering, dt=0.1)
    
    # Store position
    x_history.append(state.x)
    y_history.append(state.y)

# Plot results
plt.figure(figsize=(10, 6))
plt.plot(x_history, y_history, 'b-', label='Vehicle Path')
trajectory.plot()
plt.legend()
plt.axis('equal')
plt.grid(True)
plt.show()
```

## Development

### Setting up Development Environment

```bash
# Install with development dependencies
uv pip install -e ".[dev]"

# Run tests
pytest

# Format code
black PathTracking/

# Type checking
mypy PathTracking/

# Linting
flake8 PathTracking/
```

### Running Examples

```bash
# Test vehicle display
python test_vehicle_display.py

# Run trajectory example
cd PathTracking
python trajectory_example.py

# Test individual controllers
cd PathTracking/pure_pursuit
python pure_pursuit.py
```

## Project Structure

```
path_tracking/
├── PathTracking/                    # Main package
│   ├── cgmres_nmpc/                # CGMRES NMPC controller
│   ├── CubicSpline/                # Cubic spline path planning
│   ├── lqr_speed_steer_control/    # LQR controllers
│   ├── lqr_steer_control/          # LQR steering control variants
│   ├── model_predictive_speed_and_steer_control/  # MPC controller
│   ├── move_to_pose/               # Point-to-point navigation
│   ├── pure_pursuit/               # Pure pursuit controller
│   ├── rear_wheel_feedback_control/  # Rear wheel feedback
│   ├── stanley_control/            # Stanley controller
│   ├── utils/                      # Utility functions
│   ├── trajectory.py               # Trajectory management
│   ├── vehicle_model.py            # Vehicle dynamics
│   └── ...
├── tests/                          # Test files (to be added)
├── pyproject.toml                  # Project configuration
└── README.md                       # This file
```

## Algorithms Overview

### Pure Pursuit
Geometric path tracking algorithm that calculates steering commands based on a lookahead point on the desired path.

### Stanley Control  
Front-wheel oriented control that minimizes both cross-track error and heading error.

### LQR Control
Linear Quadratic Regulator that optimally minimizes a cost function for state tracking and control effort.

### Model Predictive Control (MPC)
Optimization-based control that handles constraints and predicts future vehicle behavior.

### CGMRES NMPC
Continuous GMRES-based approach for nonlinear model predictive control with real-time capabilities.

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Citation

If you use this library in your research, please cite:

```bibtex
@software{path_tracking,
  title={Path Tracking Library},
  author={Path Tracking Contributors},
  year={2024},
  url={https://github.com/yourusername/path_tracking}
}
``` 