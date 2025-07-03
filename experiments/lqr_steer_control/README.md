# LQR Steering Control for Path Tracking

This directory contains implementations of both discrete-time and continuous-time LQR (Linear Quadratic Regulator) controllers for vehicle path tracking.

## Files

- `lqr_steer_control.py` - Original discrete-time LQR controller
- `continuous_lqr_steer_control.py` - Continuous-time LQR controller
- `compare_lqr_controllers.py` - Comparison script for both controllers

## Overview

### Discrete LQR Controller (`lqr_steer_control.py`)

The discrete LQR controller solves the following optimization problem:
```
minimize: Σ(x[k]ᵀQx[k] + u[k]ᵀRu[k])
subject to: x[k+1] = Ax[k] + Bu[k]
```

**State vector:** `x = [lateral_error, lateral_error_rate, heading_error, heading_error_rate]`
**Control input:** `u = [steering_angle]`

**State-space matrices:**
```
A = [1  dt  0   0 ]    B = [0    ]
    [0  0   v   0 ]        [0    ]
    [0  0   1   dt]        [0    ]
    [0  0   0   0 ]        [v/L  ]
```

### Continuous LQR Controller (`continuous_lqr_steer_control.py`)

The continuous LQR controller solves:
```
minimize: ∫(xᵀQx + uᵀRu)dt
subject to: dx/dt = Ax + Bu
```

**State vector:** Same as discrete version
**Control input:** Same as discrete version

**State-space matrices:** Converted from discrete-time using:
```
A_c = (A_d - I) / dt
B_c = B_d / dt
Q_c = Q * dt
R_c = R / dt
```

## Key Differences

1. **Mathematical Foundation:**
   - Discrete: Based on discrete-time Algebraic Riccati Equation (DARE)
   - Continuous: Based on continuous-time Algebraic Riccati Equation (CARE)

2. **Implementation:**
   - Discrete: Uses `solve_DARE()` and matrix iterations
   - Continuous: Uses `scipy.linalg.solve_continuous_are()`

3. **Performance:**
   - Discrete: Typically better tracking performance due to direct design for sampled system
   - Continuous: Slightly worse performance due to discretization approximations

## Usage

### Run Individual Controllers

```bash
# Run discrete LQR controller
python lqr_steer_control.py

# Run continuous LQR controller  
python continuous_lqr_steer_control.py
```

### Compare Both Controllers

```bash
python compare_lqr_controllers.py
```

This script will:
- Run both controllers on the same path
- Display comparative plots
- Print performance statistics

## Parameters

Both controllers use the same tuning parameters:

```python
# LQR weights
Q = np.eye(4)  # State penalty matrix
R = np.eye(1)  # Control penalty matrix

# Vehicle parameters
L = 0.5        # Wheelbase [m]
dt = 0.1       # Time step [s]
max_steer = 45°# Maximum steering angle

# Speed control
Kp = 1.0       # Proportional gain for speed control
```

## Example Performance Results

Typical performance comparison on the test path:

| Controller Type | Mean Error (m) | Max Error (m) | RMS Error (m) |
|----------------|----------------|---------------|---------------|
| Discrete LQR   | 0.088         | 0.275         | 0.107         |
| Continuous LQR | 0.122         | 0.300         | 0.146         |

## Theory Background

### LQR Controller Design

The LQR controller finds the optimal feedback gain matrix K that minimizes the quadratic cost function while ensuring system stability.

**For discrete systems:**
- Solves: `A'XA - X - A'XB(R + B'XB)⁻¹B'XA + Q = 0`
- Gain: `K = (R + B'XB)⁻¹B'XA`

**For continuous systems:**
- Solves: `A'X + XA - XBR⁻¹B'X + Q = 0`
- Gain: `K = R⁻¹B'X`

### Vehicle Model

Both controllers use a bicycle model for vehicle dynamics:
```
ẋ = v cos(ψ)
ẏ = v sin(ψ)  
ψ̇ = (v/L) tan(δ)
```

Where:
- `(x,y)` = vehicle position
- `ψ` = vehicle heading angle
- `v` = velocity
- `δ` = steering angle
- `L` = wheelbase

## References

1. Atsushi Sakai's original discrete LQR implementation
2. Anderson, B.D.O. and Moore, J.B. "Optimal Control: Linear Quadratic Methods"
3. Rajamani, R. "Vehicle Dynamics and Control" 