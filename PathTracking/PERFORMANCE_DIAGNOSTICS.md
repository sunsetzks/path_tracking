# Pure Pursuit Performance Diagnostics System

## Overview

The Performance Diagnostics System provides comprehensive analysis and visualization of Pure Pursuit controller performance, including real-time tracking of commanded vs actual vehicle behavior, error analysis, and detailed performance metrics.

## Features

### 🔍 Data Collection
- **Real-time data capture**: Collects diagnostic data at each simulation step
- **Comprehensive metrics**: Vehicle state, control commands, errors, and performance indicators
- **Historical tracking**: Maintains history of up to 1000 data points (configurable)

### 📊 Performance Metrics

#### Control Tracking Performance
- **Velocity Tracking**: Commanded vs actual velocity with error statistics
- **Steering Tracking**: Commanded vs actual steering angle with error analysis
- **Acceleration Analysis**: Real-time acceleration/deceleration tracking

#### Path Tracking Performance
- **Lateral Error**: Cross-track error from reference path
- **Longitudinal Error**: Along-track error relative to goal
- **Angular Error**: Heading error relative to goal orientation

#### Direction Analysis
- **Direction Conflicts**: Detection when path direction conflicts with optimal robot direction
- **Conflict Statistics**: Count and percentage of direction conflicts

### 📈 Visualization
- **Real-time Charts**: 6-panel diagnostic dashboard showing:
  1. Velocity tracking performance
  2. Steering tracking performance  
  3. Path tracking errors
  4. Control performance metrics
  5. Goal approach analysis
  6. Tracking error statistics

### 💾 Data Export
- **CSV Export**: Complete diagnostic data export for offline analysis
- **Chart Export**: High-resolution diagnostic charts (PNG format)

## Usage

### Basic Usage

```python
from PathTracking.pure_pursuit_examples import PerformanceDiagnostics

# Initialize diagnostics
diagnostics = PerformanceDiagnostics(max_history_size=1000)

# In simulation loop
diagnostics.add_data_point(
    time=current_time,
    vehicle_state=vehicle_state,
    commanded_velocity=target_velocity,
    commanded_steering=steering_angle,
    controller=pure_pursuit_controller,
    dt=time_step
)

# Generate summary
print(diagnostics.get_diagnostic_summary())

# Generate charts
diagnostics.plot_diagnostic_charts()

# Export data
diagnostics.export_data_to_csv("diagnostic_data.csv")
```

### Simulation Integration

The diagnostic system is integrated into all simulation functions. Use the `enable_diagnostics` parameter:

```python
# Run simulation with diagnostics
diagnostics = run_simulation(
    vehicle_model=vehicle_model,
    controller=controller,
    enable_diagnostics=True  # Enable diagnostics
)
```

### Interactive Controls

During simulation with diagnostics enabled:
- **Space**: Pause/Resume simulation
- **Q/ESC**: Quit simulation
- **D**: Show diagnostic charts
- **S**: Save diagnostic data to CSV

## Command Line Interface

### Available Simulation Options

```bash
python PathTracking/pure_pursuit_examples.py [choice]
```

Options:
1. **Forward Driving Simulation**: High-speed S-curve trajectory
2. **Reverse Driving Simulation**: Backing and parking maneuvers [DEFAULT]
3. **Both Simulations**: Run forward and reverse simulations
4. **Direction Conflict Test**: Demonstrates direction conflict detection
5. **Diagnostic Analysis**: Comprehensive performance analysis with charts
6. **Simple Diagnostic Demo**: Command-line demo without GUI

### Example Commands

```bash
# Run diagnostic analysis
python PathTracking/pure_pursuit_examples.py 5

# Run simple diagnostic demo (no GUI)
python PathTracking/pure_pursuit_examples.py 6

# Run default reverse simulation with diagnostics
python PathTracking/pure_pursuit_examples.py
```

## Diagnostic Data Structure

### DiagnosticData Class

Each data point contains:

```python
@dataclass
class DiagnosticData:
    # Time information
    time: float
    
    # Vehicle state
    position_x: float
    position_y: float
    yaw_angle: float
    actual_velocity: float
    actual_steering_angle: float
    
    # Control commands
    commanded_velocity: float
    commanded_steering_angle: float
    
    # Control computation data
    lookahead_distance: float
    target_point_x: float
    target_point_y: float
    path_direction: float
    robot_direction: float
    direction_conflict: bool
    
    # Performance metrics
    distance_to_goal: float
    stopping_distance: float
    max_velocity_for_distance: float
    current_acceleration: float
    
    # Error metrics
    longitudinal_error: float
    lateral_error: float
    angular_error: float
    
    # Status flags
    goal_reached: bool
```

## Performance Analysis

### Key Performance Indicators (KPIs)

#### Excellent Performance Criteria
- Velocity tracking error mean < 0.2 m/s
- Steering tracking error mean < 3°
- Lateral error mean < 0.2 m
- Direction conflict rate < 10%

#### Warning Thresholds
- ⚠️ High velocity tracking error: > 0.5 m/s
- ⚠️ High steering tracking error: > 5°
- ⚠️ High lateral error: > 0.3 m
- ⚠️ High direction conflict rate: > 10%

### Diagnostic Summary Example

```
🔍 PERFORMANCE DIAGNOSTICS SUMMARY
==================================================

📊 Basic Statistics:
  Data Points: 173
  Total Distance: 48.06 m
  Average Velocity: 2.78 m/s
  Max Acceleration: 2.00 m/s²
  Max Deceleration: -2.50 m/s²

🎯 Control Tracking Performance:
  Velocity Tracking Error:
    Mean: 0.191 m/s
    Std:  0.046 m/s
    Max:  0.250 m/s
  Steering Tracking Error:
    Mean: 1.55°
    Std:  6.14°
    Max:  45.00°

🛣️ Path Tracking Performance:
  Lateral Error:
    Mean: 3.505 m
    Std:  0.965 m
    Max:  4.268 m
  Angular Error:
    Mean: 4.09°
    Std:  3.32°
    Max:  12.91°

🔄 Direction Analysis:
  Direction Conflicts: 0
  Conflict Rate: 0.0%
```

## Tuning Recommendations

Based on diagnostic results:

### High Velocity Tracking Error
- Increase `acceleration_gain` in vehicle configuration
- Reduce `acceleration_delay` if possible
- Check velocity controller parameters

### High Steering Tracking Error  
- Increase `steering_rate_gain` in vehicle configuration
- Reduce `steering_delay` if possible
- Check maximum steering rate limits

### High Lateral Error
- Adjust `min_lookahead` distance
- Tune `k_gain` for lookahead calculation
- Check trajectory smoothness

### Direction Conflicts
- Review trajectory design for sharp turns
- Consider allowing reverse motion in tight maneuvers
- Adjust path planning to match vehicle capabilities

## File Outputs

### CSV Data Export
Contains all diagnostic data points with timestamps for detailed offline analysis.

**Filename format**: `diagnostic_data_[timestamp].csv` or `comprehensive_diagnostic_data.csv`

### Diagnostic Charts
Six-panel visualization showing:
- Velocity and steering tracking
- Path tracking errors
- Control performance metrics
- Goal approach analysis
- Error statistics

**Filename format**: User-specified or displayed interactively

## Integration with Real Vehicles

The diagnostic system is designed to work with both simulation and real vehicle data:

1. **Simulation**: Direct integration with vehicle model
2. **Real Vehicle**: Collect actual sensor data and compare with commands
3. **Hybrid**: Use simulation for development, real data for validation

### Real Vehicle Integration Example

```python
# Collect real vehicle data
actual_velocity = get_vehicle_velocity()  # From vehicle sensors
actual_steering = get_steering_angle()   # From vehicle sensors
commanded_velocity = controller_output[1]
commanded_steering = controller_output[0]

# Create vehicle state from real sensors
vehicle_state = VehicleState(
    position_x=gps_x,
    position_y=gps_y,
    yaw_angle=imu_yaw,
    velocity=actual_velocity,
    steering_angle=actual_steering
)

# Add to diagnostics
diagnostics.add_data_point(
    time=current_time,
    vehicle_state=vehicle_state,
    commanded_velocity=commanded_velocity,
    commanded_steering=commanded_steering,
    controller=controller,
    dt=dt
)
```

## Best Practices

1. **Regular Monitoring**: Check diagnostic summaries after each test run
2. **Baseline Establishment**: Record performance metrics for known good configurations
3. **Systematic Tuning**: Change one parameter at a time and observe diagnostic impact
4. **Data Archival**: Save diagnostic data for regression testing
5. **Performance Trends**: Track performance metrics over time to detect degradation

## Troubleshooting

### Common Issues

**No diagnostic data collected**
- Ensure `enable_diagnostics=True` in simulation
- Check that `add_data_point()` is called in simulation loop

**Charts not displaying**
- Ensure matplotlib backend is properly configured
- Try saving charts to file instead of displaying

**High memory usage**
- Reduce `max_history_size` for long simulations
- Export data periodically and clear history

**CSV export fails**
- Check file permissions in output directory
- Ensure sufficient disk space

## Future Enhancements

- Real-time streaming to external monitoring systems
- Machine learning-based performance prediction
- Automated parameter tuning recommendations
- Integration with ROS diagnostic tools
- Web-based dashboard for remote monitoring 