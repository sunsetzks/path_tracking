# CSV Performance Diagnostics Log Directory

This directory contains CSV files exported from the PathTracking performance diagnostics system.

## Directory Structure

```
log/
└── csv/
    ├── README.md                          # This file
    ├── final_diagnostic_data_328_20240315_143022.csv      # Example: Simulation with 328 data points
    ├── comprehensive_diagnostic_data_20240315_143045.csv  # Example: Comprehensive diagnostic data
    ├── simple_diagnostic_demo_20240315_143100.csv         # Example: Simple diagnostic demo
    └── final_diagnostic_data_221_20240315_142955.csv      # Example: Simulation with 221 data points
```

## CSV File Format

Each CSV file contains the following columns:

### Time and Position Data
- `time`: Simulation time (seconds)
- `position_x`: Vehicle X position (meters)
- `position_y`: Vehicle Y position (meters)
- `yaw_angle`: Vehicle heading angle (radians)

### Vehicle State
- `actual_velocity`: Measured vehicle velocity (m/s)
- `actual_steering_angle`: Measured steering angle (radians)

### Control Commands
- `commanded_velocity`: Commanded velocity from controller (m/s)
- `commanded_steering_angle`: Commanded steering angle from controller (radians)

### Control Computation Data
- `lookahead_distance`: Pure pursuit lookahead distance (meters)
- `target_point_x`: Target point X coordinate (meters)
- `target_point_y`: Target point Y coordinate (meters)
- `path_direction`: Path direction angle (radians)
- `robot_direction`: Robot direction angle (radians)
- `direction_conflict`: Boolean indicating direction conflict

### Performance Metrics
- `distance_to_goal`: Distance to goal point (meters)
- `stopping_distance`: Required stopping distance (meters)
- `max_velocity_for_distance`: Maximum safe velocity for distance (m/s)
- `current_acceleration`: Current acceleration (m/s²)

### Error Metrics
- `longitudinal_error`: Along-track error (meters)
- `lateral_error`: Cross-track error (meters)
- `angular_error`: Heading error (radians)

### Status
- `goal_reached`: Boolean indicating if goal was reached

## Usage

### Automatic CSV Export

When running simulations with diagnostics enabled, CSV files are automatically saved to this directory:

```python
from PathTracking.performance_diagnostics import PerformanceDiagnostics

# Create diagnostics instance
diagnostics = PerformanceDiagnostics()

# ... run simulation and collect data ...

# Export to CSV (automatically saves to log/csv/)
diagnostics.export_data_to_csv("my_simulation_data.csv")
```

### Loading and Analyzing CSV Files

Use the enhanced CSV loader to interactively select and analyze CSV files:

```bash
cd PathTracking
python csv_loader_example.py
```

This will:
1. List all available CSV files in this directory
2. Show file information (size, data points, duration, modification time)
3. Allow you to select a file for analysis
4. Display diagnostic summary and charts
5. Optionally save charts to PNG files

### Programmatic Loading

You can also load CSV files programmatically:

```python
from PathTracking.performance_diagnostics import PerformanceDiagnostics

# Method 1: Load and display in one step
diagnostics = PerformanceDiagnostics.load_and_display(
    filename="log/csv/my_data.csv",
    show_summary=True,
    show_charts=True
)

# Method 2: Load and analyze step by step
diagnostics = PerformanceDiagnostics()
if diagnostics.load_from_csv("log/csv/my_data.csv"):
    print(diagnostics.get_diagnostic_summary())
    diagnostics.plot_diagnostic_charts()
```

## File Naming Conventions

Recommended naming patterns:
- `final_diagnostic_data_<datapoints>_<timestamp>.csv` - Final simulation results with timestamp
- `comprehensive_diagnostic_data_<timestamp>.csv` - Full diagnostic analysis with timestamp
- `simple_diagnostic_demo_<timestamp>.csv` - Simple demonstration data with timestamp
- `<experiment_name>_<timestamp>.csv` - Custom experiment data

## Chart Output

When analyzing CSV files, diagnostic charts can be saved as PNG files in the same directory with the naming pattern:
- `<csv_filename>_charts.png`

## Data Retention

CSV files are retained indefinitely for analysis and comparison. Consider archiving or removing old files periodically to manage disk space.

## Integration with Simulations

All simulation functions in the PathTracking module automatically export diagnostic data to this directory when `enable_diagnostics=True` is used. 