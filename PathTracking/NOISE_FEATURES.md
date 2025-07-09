# Vehicle Model Noise Features

## Overview

The vehicle model supports realistic noise simulation to better represent real-world vehicle behavior. This feature adds noise to the vehicle dynamics, making the simulation more realistic for testing control algorithms and control robustness.

The simple noise model is available:
- **Simple Noise Model**: Adds basic Gaussian noise to true position without accumulation

## Noise Model

### Simple Noise Model
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

### 4. Simple Noise Specific
- **Simple Position Noise**: Direct Gaussian noise on position coordinates
- **Simple Yaw Noise**: Direct Gaussian noise on yaw angle
- **Simple Velocity Noise**: Direct Gaussian noise on velocity

## Configuration Parameters

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
config.vehicle.default_state_type = "true"       # Options: "true", "simple"
```

## Usage Examples

### Using Different State Types
```python
# Get different state estimates
true_state = vehicle.get_true_state()        # Perfect state
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

## Noise Model Characteristics

| Feature | Simple |
|---------|--------|
| Error Accumulation | No |
| Measurement Delays | No |
| Periodic Corrections | No |
| Computational Cost | Low |
| Realism | Low |
| Use Case | Basic testing |

## Best Practices

1. **Parameter Tuning**:
   - Simple: Moderate noise for general testing

2. **Set Appropriate Seeds**:
   - Use fixed seeds for reproducible results
   - Use different seeds for Monte Carlo testing

3. **Monitor Performance**:
   - Check that noise levels are realistic
   - Verify that control algorithms remain stable

## Examples

See the following example files for detailed usage:
- `PathTracking/examples/simple_noise_example.py` - Simple noise model usage

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
| `simple_position_noise_std` | float | 0.1 | Simple position noise standard deviation [m] |
| `simple_yaw_noise_std` | float | 0.02 | Simple yaw angle noise standard deviation [rad] |
| `simple_velocity_noise_std` | float | 0.05 | Simple velocity noise standard deviation [m/s] |
| `default_state_type` | string | "true" | Default state type ("true" or "simple") |

## Recommended Noise Levels

### Conservative (Low Noise)
```yaml
# Control noise
steering_noise_std: 0.005             # ~0.3° steering uncertainty
# Simple noise
simple_position_noise_std: 0.05       # 5cm position uncertainty
simple_yaw_noise_std: 0.01            # ~0.6° heading uncertainty
simple_velocity_noise_std: 0.02       # 2cm/s velocity uncertainty
```

### Realistic (Medium Noise)
```yaml
# Control noise
steering_noise_std: 0.01              # ~0.6° steering uncertainty
# Simple noise
simple_position_noise_std: 0.1        # 10cm position uncertainty
simple_yaw_noise_std: 0.02            # ~1.1° heading uncertainty
simple_velocity_noise_std: 0.05       # 5cm/s velocity uncertainty
```

### Challenging (High Noise)
```yaml
# Control noise
steering_noise_std: 0.02              # ~1.1° steering uncertainty
# Simple noise
simple_position_noise_std: 0.2        # 20cm position uncertainty
simple_yaw_noise_std: 0.05            # ~2.9° heading uncertainty
simple_velocity_noise_std: 0.1        # 10cm/s velocity uncertainty
```

## Applications

### 1. Control Algorithm Testing
Test the robustness of control algorithms under realistic noise conditions.

### 2. Basic State Estimation
Develop and test basic state estimation algorithms.

### 3. Monte Carlo Simulation
Run multiple simulations with different noise realizations to assess system performance.

### 4. Sensor Noise Simulation
Test algorithms with realistic sensor noise characteristics.

## Notes

- All noise is generated using Gaussian (normal) distributions
- Control input noise can be disabled by setting `control_input_noise_enabled: false`
- The simple estimator has its own noise characteristics
- Use `noise_seed` for reproducible results in testing
- The noise generator uses numpy's RandomState for consistent behavior
- Control input noise is added to steering commands before applying vehicle limits
- Simple noise does not accumulate over time (independent at each time step)

## Example Script

### Basic Noise Demonstration
See `simple_noise_example.py` for a complete demonstration of the noise functionality, including:
- Noise configuration
- Trajectory comparison (clean vs noisy)
- Statistical analysis
- Visualization

Run the example with:
```bash
cd PathTracking
python examples/simple_noise_example.py
```

## Key Features Summary

| Aspect | Simple Noise |
|--------|--------------|
| **Error Growth** | Constant, no accumulation |
| **Drift Behavior** | No drift |
| **Update Frequency** | Every time step |
| **Measurement Delay** | None |
| **Bias Handling** | None |
| **Use Cases** | Basic noise testing |
| **Computational Cost** | Low |

## Choosing the Right Configuration

- **Use Simple Noise** for:
  - Basic algorithm testing
  - General noise robustness evaluation
  - Quick simulation setups
  - Educational demonstrations 