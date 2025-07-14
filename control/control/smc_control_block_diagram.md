# Sliding Mode Control (SMC) Block Diagram

This document provides a comprehensive block diagram representation of the Sliding Mode Control system implemented in `sliding_mode_control.py`.

## System Overview

The SMC system controls a vehicle to reach the origin (position = 0, velocity = 0) from any initial state, with robustness against bounded disturbances.

## Main Control Block Diagram

```mermaid
flowchart LR
    %% Reference inputs
    r_p["r_p = 0"] --> sum1["⊕"]
    r_v["r_v = 0"] --> sum2["⊕"]
    
    %% Error signals
    sum1 --> |"e_p"| slide["Sliding Surface<br/>s = p + λv"]
    sum2 --> |"e_v"| slide
    
    %% SMC Controller
    slide --> |"s"| smc["SMC Controller<br/>a = -v/λ - K·tanh(s/Φ)"]
    sum2 --> |"v"| smc
    
    %% Control output
    smc --> |"a"| sat["Saturation<br/>±a_max"]
    
    %% Plant
    sat --> |"a_cmd"| plant["Vehicle Plant<br/>ṗ = v<br/>v̇ = a + d"]
    dist["d(t)"] --> plant
    
    %% Outputs
    plant --> |"p"| pos["p(t)"]
    plant --> |"v"| vel["v(t)"]
    
    %% Feedback
    pos --> |"-"| sum1
    vel --> |"-"| sum2
    
    %% Styling
    classDef ref fill:#e8f5e8,stroke:#333,stroke-width:2px
    classDef ctrl fill:#fff2cc,stroke:#333,stroke-width:2px
    classDef plant fill:#f8cecc,stroke:#333,stroke-width:2px
    classDef signal fill:#dae8fc,stroke:#333,stroke-width:2px
    
    class r_p,r_v ref
    class sum1,sum2,slide,smc,sat ctrl
    class plant,dist plant
    class pos,vel signal
```

## Detailed SMC Controller Structure

```mermaid
flowchart LR
    %% Inputs
    p_in["p(t)"] --> slide_calc["s = p + λv"]
    v_in["v(t)"] --> slide_calc
    v_in --> equiv["a_eq = -v/λ"]
    
    %% Parameters
    lambda["λ"] --> slide_calc
    lambda --> equiv
    K["K"] --> switch["a_sw = -K·tanh(s/Φ)"]
    phi["Φ"] --> switch
    
    %% Control calculation
    slide_calc --> |"s"| switch
    equiv --> |"a_eq"| sum["⊕"]
    switch --> |"a_sw"| sum
    
    %% Output
    sum --> |"a_total"| sat["Saturation<br/>±a_max"]
    sat --> |"a_cmd"| out["Output"]
    
    %% Styling
    classDef input fill:#e8f5e8,stroke:#333,stroke-width:2px
    classDef param fill:#fff2cc,stroke:#333,stroke-width:2px
    classDef calc fill:#f8cecc,stroke:#333,stroke-width:2px
    classDef output fill:#dae8fc,stroke:#333,stroke-width:2px
    
    class p_in,v_in input
    class lambda,K,phi param
    class slide_calc,equiv,switch,sum,sat calc
    class out output
```

## Vehicle System Dynamics

```mermaid
flowchart LR
    %% Inputs
    a_cmd["a_cmd"] --> sum_d["⊕"]
    d_t["d(t)"] --> sum_d
    
    %% Double integrator
    sum_d --> |"a_total"| int2["∫<br/>v̇ = a"]
    int2 --> |"v(t)"| int1["∫<br/>ṗ = v"]
    int1 --> |"p(t)"| p_out["p(t)"]
    
    %% Velocity output
    int2 --> |"v(t)"| v_out["v(t)"]
    
    %% Initial conditions
    v0["v(0)"] --> int2
    p0["p(0)"] --> int1
    
    %% Styling
    classDef input fill:#e8f5e8,stroke:#333,stroke-width:2px
    classDef integrator fill:#fff2cc,stroke:#333,stroke-width:2px
    classDef output fill:#dae8fc,stroke:#333,stroke-width:2px
    classDef ic fill:#f8cecc,stroke:#333,stroke-width:2px
    
    class a_cmd,d_t input
    class sum_d,int1,int2 integrator
    class p_out,v_out output
    class v0,p0 ic
```

## Simplified Control Structure

```mermaid
flowchart LR
    %% Simple linear structure
    ref["Reference<br/>r = 0"] --> err["Error<br/>e = r - y"]
    err --> ctrl["SMC<br/>u = f(e,ė)"]
    ctrl --> plant["Plant<br/>G(s) = 1/s²"]
    plant --> out["Output<br/>y"]
    
    %% Disturbance
    dist["Disturbance<br/>d"] --> plant
    
    %% Feedback
    out --> |"Feedback"| err
    
    %% Styling
    classDef block fill:#ffffff,stroke:#333,stroke-width:2px
    classDef signal stroke:#666,stroke-width:1px
    
    class ref,err,ctrl,plant,out,dist block
```

## Control Law Mathematical Breakdown

### Sliding Surface Design
- **Sliding Surface**: `s(t) = p(t) + λv(t)`
- **Purpose**: Combines position and velocity errors with adjustable coupling
- **Parameter λ**: Controls the slope of the sliding line in phase space

### Control Components

#### 1. Equivalent Control
```
a_eq = -v(t)/λ
```
- **Purpose**: Maintains sliding motion when no disturbance is present
- **Behavior**: Provides smooth control when system is on sliding surface

#### 2. Switching Control
```
a_sw = -K · tanh(s(t)/Φ)
```
- **Purpose**: Ensures robustness against bounded disturbances
- **K**: Switching gain (must be > disturbance bound)
- **Φ**: Boundary layer thickness (reduces chattering)
- **tanh**: Smooth approximation of sign function

#### 3. Total Control Law
```
a_total = a_eq + a_sw = -v(t)/λ - K · tanh(s(t)/Φ)
```

### Stability Analysis

#### Lyapunov Function
```
V(t) = 0.5 · s(t)²
```

#### Stability Condition
```
V̇(t) = s(t) · ṡ(t) < 0  (when |s| > Φ)
```

This ensures the system converges to the sliding surface and then to the origin.

## Parameter Tuning Guidelines

| Parameter | Symbol | Effect | Typical Range |
|-----------|--------|---------|---------------|
| Sliding slope | λ | Convergence speed and overshoot | 0.5 - 2.0 |
| Switching gain | K | Disturbance rejection | 1.5 - 5.0 |
| Boundary layer | Φ | Chattering vs. accuracy | 0.01 - 0.5 |
| Acceleration limits | a_min, a_max | Actuator constraints | ±5.0 m/s² |

## Implementation Notes

1. **Chattering Reduction**: Uses `tanh` instead of `sign` function
2. **Actuator Saturation**: Clips control output to realistic limits
3. **Numerical Integration**: Euler method with configurable time step
4. **Convergence Detection**: Threshold-based stopping criterion
5. **Disturbance Handling**: Supports arbitrary disturbance functions

## Usage Example

```python
# Create controller
controller = SlidingModeController(
    lambda_param=1.0,  # Sliding surface slope
    K=2.0,            # Switching gain  
    phi=0.1           # Boundary layer thickness
)

# Create vehicle system
vehicle = VehicleSystem(
    initial_position=10.0,
    initial_velocity=5.0,
    disturbance_function=lambda t: 0.8 * np.sin(2*np.pi*0.5*t)
)

# Run simulation
simulator = SMCSimulator(controller, vehicle)
results = simulator.run_simulation()
```

This block diagram representation provides a complete understanding of the SMC control system architecture and its mathematical foundations. 