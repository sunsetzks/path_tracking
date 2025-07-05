# Vehicle Model Noise Features

## Overview

The vehicle model now supports realistic noise simulation to better represent real-world vehicle behavior. This feature adds various types of noise to the vehicle dynamics, making the simulation more realistic for testing control algorithms and estimators.

Three noise models are available:
1. **Odometry Noise Model**: Simulates dead-reckoning sensors (wheel encoders, IMU)
2. **Global Localization Noise Model**: Simulates GPS/GNSS-based positioning systems
3. **Simple Noise Model**: Adds basic Gaussian noise to true position without accumulation

## Noise Models

### 1. Odometry Noise Model
This model simulates traditional dead-reckoning sensors where noise accumulates over time:
- **Characteristics**: Unbounded drift, cumulative errors
- **Use Cases**: Pure odometry-based navigation, wheel encoders, IMU integration
- **Error Growth**: Linear to quadratic growth over time

### 2. Global Localization Noise Model
This model simulates GPS/GNSS-like positioning systems:
- **Characteristics**: Bounded errors, periodic corrections, measurement delays
- **Use Cases**: GPS-aided navigation, RTK-GPS, global positioning systems
- **Error Growth**: Bounded with periodic corrections to prevent drift

### 3. Simple Noise Model
This model adds basic Gaussian noise to the true state:
- **Characteristics**: Independent noise at each time step, no accumulation
- **Use Cases**: Basic sensor noise simulation, testing noise robustness
- **Error Growth**: Constant noise level, no drift over time

## Noise Types

### 1. State Noise
- **Position Noise**: Gaussian noise added to X and Y coordinates
- **Yaw Noise**: Angular noise added to vehicle heading
- **Velocity Noise**: Speed measurement noise
- **Steering Noise**: Steering angle measurement noise

### 2. Process Noise
- **Control Input Noise**: Noise added to steering rate and acceleration commands
- Simulates actuator uncertainties and disturbances
- Can be controlled independently with `control_input_noise_enabled` flag

### 3. Measurement Noise
- **Sensor Noise**: Noise added to any sensor measurements
- Can be applied to individual sensor readings

### 4. Global Localization Specific
- **Global Position Noise**: GPS-like position uncertainty
- **Global Yaw Noise**: Compass/magnetometer-like heading uncertainty

### 5. Simple Noise Specific
- **Simple Position Noise**: Direct Gaussian noise on position coordinates
- **Simple Yaw Noise**: Direct Gaussian noise on yaw angle
- **Simple Velocity Noise**: Direct Gaussian noise on velocity

## Configuration Parameters

### Odometry Noise Parameters
```python
config.vehicle.odometry_position_noise_std = 0.1  # [m]
config.vehicle.odometry_yaw_noise_std = 0.05      # [rad]
config.vehicle.odometry_velocity_noise_std = 0.02 # [m/s]
```

### Global Localization Noise Parameters
```python
config.vehicle.global_position_noise_std = 0.05      # [m]
config.vehicle.global_yaw_noise_std = 0.02           # [rad]
config.vehicle.global_measurement_frequency = 1.0    # [Hz]
config.vehicle.global_measurement_delay = 0.1        # [s]
```

### Simple Noise Parameters
```python
config.vehicle.simple_position_noise_std = 0.1   # [m]
config.vehicle.simple_yaw_noise_std = 0.02       # [rad]
config.vehicle.simple_velocity_noise_std = 0.05  # [m/s]
```

### Control Input Noise Parameters
```python
config.vehicle.control_input_noise_enabled = True
config.vehicle.steering_noise_std = 0.01         # [rad/s]
```

### General Parameters
```python
config.vehicle.noise_seed = 42                   # For reproducible results
config.vehicle.default_state_type = "true"       # Options: "true", "odometry", "global", "simple"
```

## Usage Examples

### Using Different State Types
```python
# Get different state estimates
true_state = vehicle.get_true_state()        # Perfect state
odometry_state = vehicle.get_odometry_state() # Dead reckoning with drift
global_state = vehicle.get_global_state()    # GPS-like with bounded errors
simple_state = vehicle.get_simple_state()    # Basic Gaussian noise

# Or use the generic method with state type
state = vehicle.get_state("simple")          # Gets simple noise state
```

### Setting Default State Type
```python
config.vehicle.default_state_type = "simple"
vehicle = VehicleModel(config.vehicle, initial_state)

# Now get_state() returns simple noise state by default
current_state = vehicle.get_state()
```

### Controlling Noise
```python
# Enable/disable noise for all estimators
vehicle.set_noise_enabled(True)

# Enable/disable control input noise specifically
vehicle.set_control_input_noise_enabled(False)

# Reset noise seed for reproducibility
vehicle.reset_noise_seed(42)
```

## Comparison of Noise Models

| Feature | Odometry | Global | Simple |
|---------|----------|--------|--------|
| Error Accumulation | Yes | No | No |
| Measurement Delays | No | Yes | No |
| Periodic Corrections | No | Yes | No |
| Computational Cost | Medium | High | Low |
| Realism | High | High | Low |
| Use Case | Dead reckoning | GPS/GNSS | Basic testing |

## Best Practices

1. **Choose the Right Model**: 
   - Use odometry for wheel encoder/IMU simulation
   - Use global for GPS/GNSS simulation
   - Use simple for basic noise testing

2. **Tune Parameters Realistically**:
   - Odometry: Small noise that accumulates
   - Global: Larger noise but bounded
   - Simple: Moderate noise for general testing

3. **Set Appropriate Seeds**:
   - Use fixed seeds for reproducible results
   - Use different seeds for Monte Carlo testing

4. **Monitor Performance**:
   - Check that noise levels are realistic
   - Verify that control algorithms remain stable

## Examples

See the following example files for detailed usage:
- `PathTracking/examples/simple_noise_example.py` - Simple noise model usage
- `PathTracking/noise_model_comparison.py` - Comparison of all noise models

## API Reference

### VehicleModel Methods

#### `set_noise_enabled(enabled: bool)`
Enable or disable noise generation.

#### `get_noise_enabled() -> bool`
Get current noise enabled status.

#### `reset_noise_seed(seed: Optional[int] = None)`
Reset noise random seed for reproducibility.

#### `get_clean_state() -> VehicleState`
Get current vehicle state without noise (for debugging/analysis).

#### `get_noisy_measurement(measurement: float) -> float`
Apply measurement noise to a sensor reading.

#### `set_control_input_noise_enabled(enabled: bool)`
Enable or disable control input noise specifically.

#### `get_control_input_noise_enabled() -> bool`
Get current control input noise enabled status.

### Configuration Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `control_input_noise_enabled` | bool | True | Enable/disable control input noise (master switch) |
| `steering_noise_std` | float | 0.01 | Steering angle noise standard deviation [rad] |
| `noise_seed` | int/null | null | Random seed for reproducible noise |
| `odometry_position_noise_std` | float | 0.01 | Odometry position noise standard deviation [m] |
| `odometry_yaw_noise_std` | float | 0.005 | Odometry yaw angle noise standard deviation [rad] |
| `odometry_velocity_noise_std` | float | 0.02 | Odometry velocity noise standard deviation [m/s] |
| `global_position_noise_std` | float | 0.5 | Global position noise standard deviation [m] |
| `global_yaw_noise_std` | float | 0.02 | Global yaw angle noise standard deviation [rad] |
| `global_measurement_frequency` | float | 1.0 | Global measurement frequency [Hz] |
| `global_measurement_delay` | float | 0.1 | Global measurement delay [s] |
| `default_state_type` | str | "true" | Default state type ("true", "odometry", "global", "simple") |

## Typical Noise Levels

### Conservative (Low Noise)
```yaml
# Control noise
steering_noise_std: 0.005             # ~0.3° steering uncertainty
# Odometry noise
odometry_position_noise_std: 0.005    # 0.5cm position uncertainty per meter
odometry_yaw_noise_std: 0.002         # ~0.1° heading uncertainty per radian
odometry_velocity_noise_std: 0.01     # 1cm/s velocity uncertainty
# Global localization noise
global_position_noise_std: 0.3        # 30cm GPS accuracy
global_yaw_noise_std: 0.01            # ~0.6° GPS heading accuracy
```

### Realistic (Medium Noise)
```yaml
# Control noise
steering_noise_std: 0.01              # ~0.6° steering uncertainty
# Odometry noise
odometry_position_noise_std: 0.01     # 1cm position uncertainty per meter
odometry_yaw_noise_std: 0.005         # ~0.3° heading uncertainty per radian
odometry_velocity_noise_std: 0.02     # 2cm/s velocity uncertainty
# Global localization noise
global_position_noise_std: 0.5        # 50cm GPS accuracy
global_yaw_noise_std: 0.02            # ~1.1° GPS heading accuracy
```

### Challenging (High Noise)
```yaml
# Control noise
steering_noise_std: 0.02              # ~1.1° steering uncertainty
# Odometry noise
odometry_position_noise_std: 0.02     # 2cm position uncertainty per meter
odometry_yaw_noise_std: 0.01          # ~0.6° heading uncertainty per radian
odometry_velocity_noise_std: 0.05     # 5cm/s velocity uncertainty
# Global localization noise
global_position_noise_std: 1.0        # 1m GPS accuracy
global_yaw_noise_std: 0.05            # ~2.9° GPS heading accuracy
```

## Applications

### 1. Control Algorithm Testing
Test the robustness of control algorithms under realistic noise conditions.

### 2. State Estimation
Develop and test Kalman filters and other state estimators.

### 3. Monte Carlo Simulation
Run multiple simulations with different noise realizations to assess system performance.

### 4. Sensor Fusion
Test multi-sensor fusion algorithms with realistic sensor noise.

## Notes

- All noise is generated using Gaussian (normal) distributions
- Control input noise can be disabled by setting `control_input_noise_enabled: false`
- Individual estimators (odometry and global) have their own noise characteristics
- Use `noise_seed` for reproducible results in testing
- The noise generator uses numpy's RandomState for consistent behavior
- Control input noise is added to steering commands before applying vehicle limits
- Odometry noise accumulates over time (unbounded drift)
- Global localization noise is bounded and doesn't accumulate indefinitely

## Example Scripts

### 1. Basic Noise Demonstration
See `noise_example.py` for a complete demonstration of the noise functionality, including:
- Noise configuration
- Trajectory comparison (clean vs noisy)
- Statistical analysis
- Visualization

Run the example with:
```bash
cd PathTracking
python noise_example.py
```

### 2. Noise Model Comparison
See `noise_model_comparison.py` for a detailed comparison between odometry and global localization noise models:
- Side-by-side comparison of both noise models
- Long-term drift analysis
- Error statistics and improvement metrics
- Comprehensive visualization

Run the comparison with:
```bash
cd PathTracking
python noise_model_comparison.py
```

## Key Differences Summary

| Aspect | Odometry Noise | Global Localization Noise |
|--------|----------------|---------------------------|
| **Error Growth** | Unbounded, cumulative | Bounded, periodic corrections |
| **Drift Behavior** | Continuous drift | Limited drift with corrections |
| **Update Frequency** | Every time step | Configurable (e.g., 1 Hz) |
| **Measurement Delay** | None | Configurable (e.g., 100ms) |
| **Bias Handling** | Cumulative | Persistent but corrected |
| **Use Cases** | Dead reckoning, wheel odometry | GPS/GNSS navigation |
| **Computational Cost** | Lower | Slightly higher |

## Choosing the Right Model

- **Use Odometry Noise** for:
  - Pure dead-reckoning scenarios
  - Wheel encoder-based navigation
  - IMU integration testing
  - Short-term navigation accuracy analysis

- **Use Global Localization Noise** for:
  - GPS-aided navigation systems
  - Long-term positioning accuracy
  - Sensor fusion algorithm testing
  - Realistic outdoor vehicle simulation 