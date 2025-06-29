# MPC Controller Array Sizes Explanation

This document explains the dimensions and purposes of all arrays used in the Model Predictive Control (MPC) path tracking controller.

## Key Parameters

- **STATE_DIMENSION**: 4 (x, y, velocity, yaw)
- **CONTROL_DIMENSION**: 2 (acceleration, steering_angle)
- **PREDICTION_HORIZON**: 5 time steps
- **TIME_STEP**: 0.2 seconds
- **MPC Prediction Window**: 1.0 seconds (5 × 0.2s)

## Array Categories

### 1. Input Trajectory Arrays
These arrays define the reference path that the vehicle should follow:

| Array Name | Shape | Description |
|------------|-------|-------------|
| `trajectory_x` | (162,) | X coordinates of reference path |
| `trajectory_y` | (162,) | Y coordinates of reference path |
| `trajectory_yaw` | (162,) | Yaw angles along reference path |
| `trajectory_curvature` | (162,) | Curvature at each path point |
| `speed_profile` | (162,) | Desired speed at each path point |

**Note**: 162 points is specific to the parking path example with 1.0m resolution.

### 2. State Space Model Matrices
These matrices define the linearized vehicle dynamics:

| Matrix Name | Shape | Description |
|-------------|-------|-------------|
| `A_matrix` | (4, 4) | State transition matrix |
| `B_matrix` | (4, 2) | Control input matrix |
| `C_vector` | (4,) | Linearization offset vector |

### 3. MPC Optimization Variables
These are the decision variables in the optimization problem:

| Variable Name | Shape | Description |
|---------------|-------|-------------|
| `state_variables` | (4, 6) | Predicted states over horizon |
| `control_variables` | (2, 5) | Control inputs over horizon |

**Explanation**:
- State variables have 6 columns: initial state + 5 future predictions
- Control variables have 5 columns: one control input per prediction step
- Rows represent different state/control dimensions

### 4. Reference Arrays for MPC
These arrays provide the reference for the MPC optimization:

| Array Name | Shape | Description |
|------------|-------|-------------|
| `reference_trajectory` | (4, 6) | Reference states over horizon |
| `reference_steering` | (1, 6) | Reference steering over horizon |

### 5. Cost Matrices
These matrices define the optimization objective:

| Matrix Name | Shape | Description |
|-------------|-------|-------------|
| `INPUT_COST_MATRIX` | (2, 2) | Cost weights for control inputs |
| `INPUT_RATE_COST_MATRIX` | (2, 2) | Cost weights for control rates |
| `STATE_COST_MATRIX` | (4, 4) | Cost weights for state tracking |
| `TERMINAL_STATE_COST_MATRIX` | (4, 4) | Cost weights for terminal state |

### 6. MPC Output Arrays
These arrays contain the solution from the optimization:

| Array Name | Shape | Description |
|------------|-------|-------------|
| `predicted_x` | (6,) | Predicted x positions |
| `predicted_y` | (6,) | Predicted y positions |
| `predicted_velocity` | (6,) | Predicted velocities |
| `predicted_yaw` | (6,) | Predicted yaw angles |
| `acceleration_sequence` | (5,) | Optimal acceleration inputs |
| `steering_sequence` | (5,) | Optimal steering inputs |

### 7. Applied Control (Scalars)
Only the first element of each sequence is applied:

| Variable Name | Type | Description |
|---------------|------|-------------|
| `applied_acceleration` | scalar | First acceleration command |
| `applied_steering` | scalar | First steering command |

### 8. Simulation History Arrays
These arrays grow during simulation (example for 100 time steps):

| Array Name | Shape | Description |
|------------|-------|-------------|
| `time_history` | (N,) | Time stamps |
| `x_history` | (N,) | Vehicle x position history |
| `y_history` | (N,) | Vehicle y position history |
| `yaw_history` | (N,) | Vehicle yaw history |
| `velocity_history` | (N,) | Vehicle velocity history |
| `steering_history` | (N,) | Applied steering history |
| `acceleration_history` | (N,) | Applied acceleration history |

Where N grows with each simulation step.

## Memory Usage Estimates

For the parking path example:
- **Trajectory arrays**: ~5.1 KB (4 arrays × 162 points × 8 bytes)
- **MPC optimization variables**: ~0.266 KB (34 variables × 8 bytes)
- **Simulation history (100 steps)**: ~5.5 KB (7 arrays × 100 points × 8 bytes)

## Data Flow

1. **Input**: Full trajectory arrays (162 points)
2. **Reference Generation**: Extract relevant points for MPC horizon (6 points)
3. **Optimization**: Solve for optimal control over horizon (5 control inputs)
4. **Application**: Apply first control input (2 scalars)
5. **History**: Store results in growing arrays

## Key Relationships

- **Prediction Horizon + 1 = State Variable Columns**: We need the initial state plus predictions
- **Prediction Horizon = Control Variable Columns**: One control per prediction step
- **State Dimension = Number of State Rows**: [x, y, velocity, yaw]
- **Control Dimension = Number of Control Rows**: [acceleration, steering]

## Example: State Variables Matrix

```
state_variables = [
    [x₀, x₁, x₂, x₃, x₄, x₅],     # x positions (6 time steps)
    [y₀, y₁, y₂, y₃, y₄, y₅],     # y positions (6 time steps)
    [v₀, v₁, v₂, v₃, v₄, v₅],     # velocities (6 time steps)
    [ψ₀, ψ₁, ψ₂, ψ₃, ψ₄, ψ₅]      # yaw angles (6 time steps)
]
```

## Example: Control Variables Matrix

```
control_variables = [
    [a₁, a₂, a₃, a₄, a₅],         # accelerations (5 control steps)
    [δ₁, δ₂, δ₃, δ₄, δ₅]          # steering angles (5 control steps)
]
```

Where:
- Subscript 0 represents current time
- Subscripts 1-5 represent future predictions
- Only a₁ and δ₁ (first column) are actually applied to the vehicle 