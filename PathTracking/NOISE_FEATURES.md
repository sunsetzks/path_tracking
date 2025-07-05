# Vehicle Model Noise Features

## Overview

The vehicle model now supports realistic noise simulation to better represent real-world vehicle behavior. This feature adds various types of noise to the vehicle dynamics, making the simulation more realistic for testing control algorithms and estimators.

Two noise models are available:
1. **Odometry Noise Model**: Simulates dead-reckoning sensors (wheel encoders, IMU)
2. **Global Localization Noise Model**: Simulates GPS/GNSS-based positioning systems

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
- **Measurement Frequency**: Simulates GPS update rates (typically 1-10 Hz)
- **Measurement Delays**: Simulates processing delays in GPS systems

## Configuration

### YAML Configuration (config.yaml)
```yaml
vehicle:
  # Noise parameters
  noise_enabled: true               # Enable/disable noise simulation
  control_input_noise_enabled: true # Enable/disable control input noise (master switch)
  noise_model: "odometry"           # Noise model: "odometry" or "global_localization"
  position_noise_std: 0.01          # Standard deviation for position noise [m]
  yaw_noise_std: 0.005              # Standard deviation for yaw angle noise [rad]
  velocity_noise_std: 0.02          # Standard deviation for velocity noise [m/s]
  steering_noise_std: 0.01          # Standard deviation for steering angle noise [rad]
  process_noise_std: 0.001          # Standard deviation for process noise [various units]
  measurement_noise_std: 0.005      # Standard deviation for measurement noise [various units]
  noise_seed: 42                    # Random seed for reproducible noise (null for random)
  
  # Global localization noise parameters (for GPS-like systems)
  global_position_noise_std: 0.5    # Standard deviation for global position noise [m]
  global_yaw_noise_std: 0.02        # Standard deviation for global yaw angle noise [rad]
  global_measurement_frequency: 1.0 # Frequency of global measurements [Hz]
  global_measurement_delay: 0.1     # Delay of global measurements [s]
```

### Programmatic Configuration
```python
from config import load_config
from vehicle_model import VehicleModel

# Configure odometry noise model
config = load_config()
config.vehicle.noise_enabled = True
config.vehicle.control_input_noise_enabled = True  # Enable control input noise
config.vehicle.noise_model = "odometry"
config.vehicle.position_noise_std = 0.05
config.vehicle.noise_seed = 42

# Create vehicle with odometry noise
vehicle_odometry = VehicleModel(config.vehicle)

# Configure global localization noise model
config.vehicle.noise_model = "global_localization"
config.vehicle.global_position_noise_std = 0.5
config.vehicle.global_measurement_frequency = 1.0

# Create vehicle with global localization noise
vehicle_global = VehicleModel(config.vehicle)
```

## Usage Examples

### Basic Noise Control
```python
# Enable/disable all noise
vehicle.set_noise_enabled(True)
print(f"Noise enabled: {vehicle.get_noise_enabled()}")

# Enable/disable control input noise specifically
vehicle.set_control_input_noise_enabled(False)
print(f"Control input noise enabled: {vehicle.get_control_input_noise_enabled()}")

# Reset random seed for reproducibility
vehicle.reset_noise_seed(123)
```

### Comparing Clean vs Noisy Behavior
```python
# Create two vehicles - one clean, one noisy
vehicle_clean = VehicleModel(config.vehicle)
vehicle_clean.set_noise_enabled(False)

vehicle_noisy = VehicleModel(config.vehicle)
vehicle_noisy.set_noise_enabled(True)

# Simulate with same control inputs
for control in control_sequence:
    clean_state = vehicle_clean.update_with_rates(control, time_step)
    noisy_state = vehicle_noisy.update_with_rates(control, time_step)
    
    # Compare results
    position_error = np.sqrt((noisy_state.position_x - clean_state.position_x)**2 + 
                            (noisy_state.position_y - clean_state.position_y)**2)
```

### Sensor Measurement Noise
```python
# Add noise to sensor readings
gps_reading = 10.5  # Original GPS measurement
noisy_gps = vehicle.get_noisy_measurement(gps_reading)
print(f"GPS reading: {gps_reading:.3f} -> {noisy_gps:.3f}")
```

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
| `noise_enabled` | bool | False | Enable/disable noise simulation |
| `control_input_noise_enabled` | bool | True | Enable/disable control input noise (master switch) |
| `position_noise_std` | float | 0.01 | Position noise standard deviation [m] |
| `yaw_noise_std` | float | 0.005 | Yaw angle noise standard deviation [rad] |
| `velocity_noise_std` | float | 0.02 | Velocity noise standard deviation [m/s] |
| `steering_noise_std` | float | 0.01 | Steering angle noise standard deviation [rad] |
| `process_noise_std` | float | 0.001 | Process noise standard deviation |
| `measurement_noise_std` | float | 0.005 | Measurement noise standard deviation |
| `noise_seed` | int/null | null | Random seed for reproducible noise |

## Typical Noise Levels

### Conservative (Low Noise)
```yaml
position_noise_std: 0.01      # 1cm position uncertainty
yaw_noise_std: 0.005          # ~0.3° heading uncertainty
velocity_noise_std: 0.02      # 2cm/s velocity uncertainty
steering_noise_std: 0.01      # ~0.6° steering uncertainty
process_noise_std: 0.001      # Low process noise
```

### Realistic (Medium Noise)
```yaml
position_noise_std: 0.05      # 5cm position uncertainty
yaw_noise_std: 0.02           # ~1.1° heading uncertainty
velocity_noise_std: 0.1       # 10cm/s velocity uncertainty
steering_noise_std: 0.02      # ~1.1° steering uncertainty
process_noise_std: 0.01       # Medium process noise
```

### Challenging (High Noise)
```yaml
position_noise_std: 0.1       # 10cm position uncertainty
yaw_noise_std: 0.05           # ~2.9° heading uncertainty
velocity_noise_std: 0.2       # 20cm/s velocity uncertainty
steering_noise_std: 0.05      # ~2.9° steering uncertainty
process_noise_std: 0.02       # High process noise
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
- Noise can be disabled completely by setting `noise_enabled: false`
- Use `noise_seed` for reproducible results in testing
- The noise generator uses numpy's RandomState for consistent behavior
- Process noise is added to control inputs before applying vehicle limits

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