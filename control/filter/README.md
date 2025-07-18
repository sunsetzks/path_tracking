# Filter Implementations for Path Tracking and Control Systems

This directory contains comprehensive filter implementations commonly used in robotics, path tracking, and control systems. All filters are implemented with English comments and include simulation tests.

## Available Filters

### 1. Kalman Filter (`kalman_filter.py`)

**Purpose**: Optimal state estimation for linear systems with Gaussian noise

**Use Cases**:
- Vehicle position and velocity tracking
- Sensor fusion for linear systems
- State estimation with noisy measurements

**Key Features**:
- Standard Kalman Filter implementation
- Specialized vehicle tracking configuration
- History tracking for analysis
- Joseph form covariance update for numerical stability

**Example Usage**:
```python
from kalman_filter import create_vehicle_kalman_filter

# Create filter for vehicle tracking
dt = 0.1  # Time step
kf = create_vehicle_kalman_filter(dt)

# Initialize with initial state and covariance
initial_state = np.array([x0, y0, vx0, vy0])
initial_cov = np.eye(4)
kf.set_initial_state(initial_state, initial_cov)

# Process measurements
for measurement in measurements:
    estimate = kf.filter_step(measurement)  # [x, y] position measurement
```

### 2. Low-pass Filter (`lowpass_filter.py`)

**Purpose**: Noise reduction and signal smoothing

**Use Cases**:
- Removing high-frequency noise from sensors
- Smoothing control signals
- Anti-aliasing before sampling

**Key Features**:
- First-order and second-order Butterworth implementations
- Configurable cutoff frequency
- Real-time and batch processing modes

**Example Usage**:
```python
from lowpass_filter import LowPassFilter, ButterworthLowPassFilter

# Create simple low-pass filter
lpf = LowPassFilter(cutoff_freq=5.0, sample_rate=100.0)

# Filter signal in real-time
for signal_value in signal_data:
    filtered_value = lpf.filter(signal_value)

# Or filter batch of data
filtered_signal = lpf.filter_batch(signal_array)
```

### 3. Moving Average Filter (`moving_average_filter.py`)

**Purpose**: Signal smoothing and trend following

**Use Cases**:
- Smoothing noisy signals
- Trend detection
- Simple low-pass filtering

**Key Features**:
- Simple Moving Average (SMA)
- Exponential Moving Average (EMA)
- Weighted Moving Average with custom weights
- Step response optimization

**Example Usage**:
```python
from moving_average_filter import MovingAverageFilter, ExponentialMovingAverageFilter

# Simple moving average
ma = MovingAverageFilter(window_size=10)

# Exponential moving average (more responsive)
ema = ExponentialMovingAverageFilter(alpha=0.3)

for value in data:
    smoothed_value = ma.filter(value)
    responsive_value = ema.filter(value)
```

### 4. Complementary Filter (`complementary_filter.py`)

**Purpose**: Sensor fusion, especially for IMU applications

**Use Cases**:
- Fusing accelerometer and gyroscope data
- Attitude estimation
- Combining high-frequency and low-frequency sensors

**Key Features**:
- Basic complementary filter
- 2D attitude estimation (roll/pitch)
- Adaptive filter with dynamic alpha adjustment
- IMU simulation for testing

**Example Usage**:
```python
from complementary_filter import ComplementaryFilter

# Create filter for attitude estimation
alpha = 0.98  # Trust gyroscope more (0.95-0.99 typical)
cf = ComplementaryFilter(alpha)

# Update with sensor data
dt = 0.02  # 50 Hz update rate
for i in range(len(gyro_data)):
    angle = cf.update(gyro_data[i], accel_angle[i], dt)
```

### 5. Particle Filter (`particle_filter.py`)

**Purpose**: Non-linear state estimation with non-Gaussian noise

**Use Cases**:
- Tracking objects with non-linear motion
- Localization in complex environments
- Multi-modal probability distributions

**Key Features**:
- Generic particle filter framework
- Specialized vehicle tracking implementation
- Non-linear motion model support
- Systematic resampling

**Example Usage**:
```python
from particle_filter import VehicleParticleFilter

# Create particle filter
pf = VehicleParticleFilter(num_particles=1000)
pf.set_time_step(0.1)

# Initialize particles
initial_state = np.array([x0, y0, vx0, vy0])
initial_cov = np.eye(4)
pf.initialize_particles(initial_state, initial_cov)

# Track object
for measurement in measurements:
    estimate = pf.filter_step(measurement, pf.motion_model, pf.measurement_model)
```

## Running Simulations and Tests

### Quick Demo
Run the demonstration script to see all filters in action:
```bash
cd control/filter
python demo_filters.py
```

### Comprehensive Testing
Run the full test suite for detailed performance analysis:
```bash
cd control/filter
python test_filters.py
```

### Individual Filter Testing
Each filter module can be run independently:
```bash
python kalman_filter.py
python lowpass_filter.py
python moving_average_filter.py
python complementary_filter.py
python particle_filter.py
```

## Filter Selection Guide

| Filter Type | Linear System | Non-linear System | Gaussian Noise | Non-Gaussian Noise | Real-time | Computational Cost |
|-------------|---------------|-------------------|----------------|-------------------|-----------|-------------------|
| Kalman | ✅ Optimal | ❌ | ✅ Optimal | ❌ | ✅ | Low |
| Low-pass | ✅ | ✅ | ✅ | ✅ | ✅ | Very Low |
| Moving Average | ✅ | ✅ | ✅ | ✅ | ✅ | Very Low |
| Complementary | ✅ | Limited | ✅ | ✅ | ✅ | Low |
| Particle | ✅ | ✅ Excellent | ✅ | ✅ Excellent | ✅ | High |

## Performance Characteristics

### Kalman Filter
- **Strengths**: Optimal for linear systems, fast convergence, mathematically proven
- **Weaknesses**: Assumes Gaussian noise, linear system model
- **Best for**: Vehicle tracking, sensor fusion in linear systems

### Low-pass Filter
- **Strengths**: Very simple, fast, good noise reduction
- **Weaknesses**: Introduces phase lag, fixed frequency response
- **Best for**: Signal conditioning, noise removal

### Moving Average Filter
- **Strengths**: Simple implementation, good for trends, no assumptions
- **Weaknesses**: Fixed delay, may overshoot, limited noise reduction
- **Best for**: Signal smoothing, trend detection

### Complementary Filter
- **Strengths**: Simple sensor fusion, good for IMU, tunable response
- **Weaknesses**: Limited to 2 sensors, assumes specific noise characteristics
- **Best for**: IMU attitude estimation, accelerometer/gyroscope fusion

### Particle Filter
- **Strengths**: Handles non-linear systems, non-Gaussian noise, multi-modal distributions
- **Weaknesses**: Computationally expensive, requires tuning, can degrade
- **Best for**: Non-linear tracking, complex noise models, multiple hypotheses

## Dependencies

All implementations require:
- `numpy` - Numerical computations
- `matplotlib` - Plotting and visualization
- `scipy` - Statistical functions (particle filter only)

## Integration with Path Tracking

These filters can be easily integrated with the main PathTracking module:

```python
# Example: Using Kalman filter with vehicle tracking
from PathTracking import VehicleModel
from control.filter import KalmanFilter

# Create vehicle model and filter
vehicle = VehicleModel()
kf = create_vehicle_kalman_filter(dt=0.1)

# In tracking loop:
measured_position = get_gps_measurement()  # [x, y]
estimated_state = kf.filter_step(measured_position)
filtered_position = estimated_state[:2]
```

## Contributing

When adding new filters:
1. Follow the existing interface pattern
2. Include comprehensive documentation in English
3. Add simulation tests
4. Update this README
5. Ensure all code is commented in English

## References

1. Kalman, R.E. (1960). "A New Approach to Linear Filtering and Prediction Problems"
2. Welch, G. & Bishop, G. (2006). "An Introduction to the Kalman Filter"
3. Mahony, R. et al. (2008). "Nonlinear Complementary Filters on the Special Orthogonal Group"
4. Arulampalam, M.S. et al. (2002). "A Tutorial on Particle Filters" 