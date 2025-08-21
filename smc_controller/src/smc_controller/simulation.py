"""
Simulation framework for SMC trajectory tracking.

Converted from MATLAB simulate.m
"""

import numpy as np
from typing import Dict, Any, Callable, Tuple
from scipy.integrate import odeint, solve_ivp
from .parameters import CARParameters, SimulationOptions
from .utils import generate_noise
from .vehicle_model import VehicleModel


def my_ode4_fixed_state(ode_func: Callable, tspan: np.ndarray,
                       x0: np.ndarray, int_state0: list = None) -> Tuple[np.ndarray, np.ndarray, list]:
    """
    Fixed-step ODE solver with internal state support.

    Args:
        ode_func: ODE function
        tspan: Time span
        x0: Initial state
        int_state0: Initial internal state

    Returns:
        Tuple of (time, states, internal states)
    """
    dt = tspan[1] - tspan[0]
    n_steps = len(tspan)

    X = np.zeros((n_steps, len(x0)))
    X[0] = x0

    int_state = [int_state0] if int_state0 is not None else []

    for i in range(1, n_steps):
        t = tspan[i-1]
        x = X[i-1]

        if int_state:
            # For discrete controllers with internal state
            dx, int_state_new = ode_func(t, x, True, int_state[-1])
            int_state.append(int_state_new)
        else:
            dx = ode_func(t, x)

        # Simple Euler integration (could be improved to RK4)
        X[i] = x + dt * dx

    T = tspan.reshape(-1, 1)
    return T, X, int_state


def full_system_disc(t: float, x: np.ndarray, xi_dx: Callable,
                    xi_m: Callable, tau: Any, model: VehicleModel,
                    controller: Any, p: CARParameters, pc: CARParameters,
                    new_tick: bool, int_state: Any) -> Tuple[np.ndarray, Any]:
    """
    Full system dynamics for discrete controllers.

    Args:
        t: Time
        x: State
        xi_dx: Process disturbance
        xi_m: Measurement noise
        tau: Trajectory
        model: Vehicle model
        controller: Controller
        p: Real parameters
        pc: Controller parameters
        new_tick: New control tick flag
        int_state: Internal controller state

    Returns:
        Tuple of (state derivative, updated internal state)
    """
    # Get control input
    u, int_state_new = controller.compute_input(x + xi_m(t), tau, t, pc, new_tick, int_state)

    # Get model dynamics
    dx, y = model(t, x, u, p)

    # Add process disturbance
    dx += xi_dx(t)

    return dx, int_state_new


def simulate(tau: Any, tau_d: Any, controller: Any, model: VehicleModel,
            x0: np.ndarray, options: SimulationOptions) -> Dict[str, Any]:
    """
    Main simulation function.

    Args:
        tau: Reference trajectory
        tau_d: Desired trajectory (for tracking)
        controller: Controller object
        model: Vehicle model
        x0: Initial state
        options: Simulation options

    Returns:
        Benchmark results dictionary
    """
    # Setup
    h = options.stepSize
    p = options.p
    pc = options.pc

    # Generate noise
    xi_m = generate_noise(options.V, tau.T, h)

    # Time span
    tspan = np.arange(0, tau.T + h, h)

    # Check if controller is discrete
    is_discrete = hasattr(controller, 'type') and controller.type == 'discrete'

    print('Discrete' if is_discrete else 'Continuous')

    if is_discrete:
        # Discrete controller simulation
        odef = lambda t, x, new_tick, int_state: full_system_disc(
            t, x, lambda t: 0, xi_m, tau_d, model, controller, p, pc, new_tick, int_state
        )
        int_state0 = controller.int_state0
        T, X, int_state = my_ode4_fixed_state(odef, tspan, x0, int_state0)
        T = T.flatten()
    else:
        # Continuous controller simulation
        def odef(t, x):
            u = controller.compute_input(x + xi_m(t), tau_d, t, pc)
            dx, y = model(t, x, u, p)
            return dx

        # Use scipy's solve_ivp for continuous simulation
        sol = solve_ivp(odef, [0, tau.T], x0, t_eval=tspan, method='RK45')
        T = sol.t
        X = sol.y.T

        # Pad X if necessary
        if X.shape[1] < 7:
            X = np.pad(X, ((0, 0), (0, 7 - X.shape[1])), mode='constant')

    # Initialize control input array
    if is_discrete:
        U = np.zeros((len(T), len(controller.compute_input(X[0], tau_d, T[0], pc, 0, int_state[0]))))
    else:
        U = np.zeros((len(T), len(controller.compute_input(X[0], tau_d, T[0], pc))))

    # Initialize error arrays
    ERROR = np.zeros((len(T), 2))
    MUXY = np.zeros((len(T), 2))

    # Simulation loop
    for k in range(len(T)):
        # Compute control input
        if is_discrete:
            U[k] = controller.compute_input(X[k], tau_d, T[k], pc, 0, int_state[k])
        else:
            U[k] = controller.compute_input(X[k], tau_d, T[k], pc)

        # Compute tracking error
        Xd = tau_d.X(T[k])
        Yd = tau_d.Y(T[k])
        theta_d = tau_d.theta(T[k])

        ct = np.cos(theta_d)
        st = np.sin(theta_d)
        Rti = np.array([[ct, st], [-st, ct]])

        error_cart = np.array([X[k, 0] - Xd, X[k, 1] - Yd])
        ERROR[k] = Rti @ error_cart

        # Compute tire saturation
        dx, y = model(0, X[k], U[k], p)
        sf = y[0:2]  # Front slip
        sr = y[2:4]  # Rear slip
        MUXY[k] = [np.linalg.norm(sf), np.linalg.norm(sr)]

    # Extract error components
    ET = ERROR[:, 0]
    EN = ERROR[:, 1]
    MUF = MUXY[:, 0]
    MUR = MUXY[:, 1]

    # Calculate averages
    IntET = np.trapz(np.abs(ET), T) / tau.T
    IntEN = np.trapz(np.abs(EN), T) / tau.T
    IntMUF = np.trapz(MUF, T) / tau.T
    IntMUR = np.trapz(MUR, T) / tau.T

    # Prepare benchmark results
    benchmark = {
        'controller': controller.name,
        'model': model,
        'scenario': tau.id,
        'max_error': [np.max(np.abs(ET)), np.max(np.abs(EN))],
        'avg_error': [IntET, IntEN],
        'end_error': [ET[-1], EN[-1]],
        'avg_musat': [IntMUF, IntMUR],
        'total_time': T[-1],
        'data': {
            'error': np.column_stack([ET, EN]),
            'U': U,
            'control_point': controller.control_point,
            'X': X,
            'T': T,
            'tau': tau,
            'tauD': tau_d,
            'options': options
        }
    }

    if is_discrete:
        benchmark['data']['intState'] = int_state

    return benchmark
