# PathTracking Configuration Management

This document describes the configuration management system for the PathTracking project, which provides centralized parameter management for all components.

## Overview

The configuration system allows you to:

- **Centralize all parameters** in one place instead of hardcoding them throughout the codebase
- **Use preset configurations** for different scenarios (forward driving, reverse driving, parking, high-speed)
- **Load configurations from YAML files** for easy customization
- **Override parameters with environment variables** for deployment flexibility
- **Mix configuration defaults with custom parameters** for fine-tuned control

## Quick Start

### Basic Usage

```python
from PathTracking.vehicle_model import VehicleModel
from PathTracking.pure_pursuit import PurePursuitController
from PathTracking.velocity_planning import VelocityController

# All components will automatically use global configuration
vehicle = VehicleModel()
controller = PurePursuitController(wheelbase=2.9)
velocity_controller = VelocityController()
```

### Using Presets

```python
from PathTracking.config import load_preset_config
from PathTracking.vehicle_model import VehicleModel

# Load configuration optimized for reverse driving
load_preset_config('reverse')

# Vehicle will now use reverse driving parameters
vehicle = VehicleModel()
```

## Configuration Structure

The configuration is organized into five main sections:

### 1. Vehicle Configuration (`vehicle`)

Physical parameters and kinematic limits for the vehicle model:

```yaml
vehicle:
  # Physical parameters
  wheelbase: 2.9                    # Distance between front and rear axles [m]
  vehicle_length: 4.5               # Total vehicle length [m]
  vehicle_width: 2.0                # Total vehicle width [m]
  
  # Kinematic limits
  max_steering_angle: 45.0          # Maximum steering angle [degrees]
  max_velocity: 50.0                # Maximum forward velocity [m/s]
  min_velocity: -10.0               # Maximum reverse velocity [m/s]
  max_steering_rate: 30.0           # Maximum steering rate [degrees/s]
  max_acceleration: 3.0             # Maximum acceleration [m/s²]
  max_deceleration: 5.0             # Maximum deceleration [m/s²]
  
  # Delay parameters
  steering_delay: 0.0               # Time delay for steering commands [s]
  acceleration_delay: 0.0           # Time delay for acceleration commands [s]
  
  # Control gains
  steering_rate_gain: 5.0           # Gain for converting steering error to steering rate
  acceleration_gain: 2.0            # Gain for converting velocity error to acceleration
```

### 2. Pure Pursuit Configuration (`pure_pursuit`)

Parameters for the Pure Pursuit path tracking controller:

```yaml
pure_pursuit:
  min_lookahead: 1.0                # Minimum lookahead distance [m]
  k_gain: 10.0                      # Lookahead distance gain
  max_steering_angle: 45.0          # Maximum steering angle [degrees]
  goal_tolerance: 0.5               # Distance tolerance to consider goal reached [m]
  velocity_tolerance: 0.1           # Velocity tolerance to consider vehicle stopped [m/s]
```

### 3. Velocity Controller Configuration (`velocity_controller`)

Parameters for velocity planning and control:

```yaml
velocity_controller:
  max_forward_velocity: 5.0         # Maximum forward velocity [m/s]
  max_backward_velocity: 2.0        # Maximum backward velocity [m/s]
  max_acceleration: 1.0             # Maximum acceleration magnitude [m/s²]
  max_deceleration: 2.0             # Maximum deceleration magnitude [m/s²]
  goal_tolerance: 0.5               # Distance tolerance to consider goal reached [m]
  velocity_tolerance: 0.1           # Velocity tolerance to consider vehicle stopped [m/s]
  conservative_braking_factor: 1.2  # Safety factor for deceleration distance
  min_velocity: 0.1                 # Minimum velocity magnitude [m/s]
```

### 4. Trajectory Configuration (`trajectory`)

Parameters for trajectory interpolation and sampling:

```yaml
trajectory:
  discretization_distance: 0.1      # Default distance between discretized points [m]
  default_sample_count: 100         # Initial default sample count
```

### 5. Simulation Configuration (`simulation`)

Parameters for simulation and visualization:

```yaml
simulation:
  time_step: 0.1                    # Default simulation time step [s]
  max_time: 60.0                    # Default maximum simulation time [s]
  plot_margin: 5.0                  # Margin around trajectory for plot view [m]
  figure_size: [16, 8]              # Default figure size [width, height]
  animation_interval: 50            # Matplotlib animation interval [ms]
```

## Usage Methods

### 1. Default Configuration

Components automatically use global configuration when no parameters are specified:

```python
from PathTracking.vehicle_model import VehicleModel

# Uses default configuration values
vehicle = VehicleModel()
```

### 2. Configuration from File

Load configuration from a YAML file:

```python
from PathTracking.config import ConfigManager

# Load from custom file
config_manager = ConfigManager('my_config.yaml')

# Now all components will use this configuration
vehicle = VehicleModel()
```

### 3. Preset Configurations

Use predefined configurations optimized for specific scenarios:

```python
from PathTracking.config import load_preset_config

# Available presets: 'forward', 'reverse', 'parking', 'high_speed'
load_preset_config('parking')

# Components now use parking-optimized parameters
vehicle = VehicleModel()
```

### 4. Mixed Configuration

Combine configuration defaults with custom parameters:

```python
from PathTracking.vehicle_model import VehicleModel

# Use config defaults for most parameters, but override some
vehicle = VehicleModel(
    wheelbase=3.2,      # Custom wheelbase
    max_velocity=40.0,  # Custom max velocity
    # Other parameters use config defaults
)
```

### 5. Environment Variables

Override configuration with environment variables:

```bash
# Set environment variables
export PT_VEHICLE_WHEELBASE=3.5
export PT_PUREPURSUIT_MIN_LOOKAHEAD=2.0
export PT_VELOCITY_MAX_FORWARD_VELOCITY=8.0
```

```python
from PathTracking.config import get_config_manager

# Configuration will include environment variable overrides
config = get_config_manager()
```

Environment variable naming pattern: `PT_<SECTION>_<PARAMETER>`
- `PT_VEHICLE_WHEELBASE` → `vehicle.wheelbase`
- `PT_PUREPURSUIT_MIN_LOOKAHEAD` → `pure_pursuit.min_lookahead`
- `PT_VELOCITY_MAX_FORWARD_VELOCITY` → `velocity_controller.max_forward_velocity`

## Preset Configurations

The system includes four predefined configuration presets:

### Forward Driving (`forward`)
Optimized for forward motion with higher speeds and larger lookahead distances:
- Max forward velocity: 6.0 m/s
- Larger lookahead distances for stability at speed
- Moderate acceleration limits

### Reverse Driving (`reverse`)
Optimized for reverse motion with reduced speeds and smaller lookahead:
- Max backward velocity: 2.5 m/s
- Smaller lookahead for better control
- Conservative acceleration limits
- Tighter goal tolerance

### Parking (`parking`)
Optimized for precise low-speed maneuvers:
- Very low speed limits (2.0 m/s max)
- Minimal lookahead for precise control
- Conservative acceleration
- Very tight goal tolerance (0.3m)

### High Speed (`high_speed`)
Optimized for highway/high-speed scenarios:
- High speed limits (15.0 m/s max)
- Large lookahead distances
- Higher acceleration limits
- Relaxed tolerances

## API Reference

### ConfigManager Class

```python
from PathTracking.config import ConfigManager

# Initialize with optional config file
config_manager = ConfigManager('config.yaml')

# Load preset
config_manager.load_preset('forward')

# Save current configuration
config_manager.save_to_file('my_config.yaml')

# Access configuration sections
vehicle_config = config_manager.vehicle
pure_pursuit_config = config_manager.pure_pursuit
```

### Convenience Functions

```python
from PathTracking.config import (
    get_config_manager,
    load_preset_config,
    get_vehicle_config,
    get_pure_pursuit_config,
    get_velocity_controller_config,
    reset_config_manager
)

# Get global config manager
config = get_config_manager()

# Load preset globally
load_preset_config('reverse')

# Get specific configuration sections
vehicle_config = get_vehicle_config()

# Reset for testing
reset_config_manager()
```

## Examples

### Creating a Custom Configuration File

Create `my_robot_config.yaml`:

```yaml
# Configuration for my specific robot
vehicle:
  wheelbase: 3.2
  max_steering_angle: 35.0
  max_velocity: 25.0
  steering_delay: 0.1
  acceleration_delay: 0.05

pure_pursuit:
  min_lookahead: 1.5
  k_gain: 8.0

velocity_controller:
  max_forward_velocity: 6.0
  max_backward_velocity: 3.0
  max_acceleration: 1.5
```

### Using the Custom Configuration

```python
from PathTracking.config import ConfigManager
from PathTracking.vehicle_model import VehicleModel
from PathTracking.pure_pursuit import PurePursuitController

# Load custom configuration
config_manager = ConfigManager('my_robot_config.yaml')

# Create components with custom config
vehicle = VehicleModel()
controller = PurePursuitController(wheelbase=config_manager.vehicle.wheelbase)

print(f"Using wheelbase: {vehicle.kinematics.wheelbase}m")
```

### Scenario-Specific Usage

```python
from PathTracking.config import load_preset_config, reset_config_manager
from PathTracking.vehicle_model import VehicleModel

# Parking scenario
reset_config_manager()
load_preset_config('parking')
parking_vehicle = VehicleModel()

# Highway scenario  
reset_config_manager()
load_preset_config('high_speed')
highway_vehicle = VehicleModel()

print(f"Parking max velocity: {parking_vehicle.kinematics.max_velocity}m/s")
print(f"Highway max velocity: {highway_vehicle.kinematics.max_velocity}m/s")
```

## Best Practices

1. **Use presets as starting points**: Load an appropriate preset, then customize specific parameters if needed.

2. **Keep configuration files in version control**: Track configuration changes alongside code changes.

3. **Use environment variables for deployment**: Override sensitive or environment-specific parameters with environment variables.

4. **Document custom configurations**: Include comments in YAML files explaining parameter choices.

5. **Test with different configurations**: Verify your application works with different preset configurations.

6. **Reset config manager in tests**: Use `reset_config_manager()` to ensure test isolation.

## Running the Example

To see the configuration system in action:

```bash
cd PathTracking
python config_example.py
```

This will demonstrate all the configuration features with various examples. 