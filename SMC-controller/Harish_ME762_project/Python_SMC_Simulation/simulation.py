"""
Main simulation framework
Translated from MATLAB simulate.m
"""

import numpy as np
import time
from scipy.integrate import odeint, solve_ivp
from typing import Dict, Any, Callable


def generate_noise(V: float, T: float, step_size: float) -> Callable[[float], float]:
    """
    Generate noise function
    For now, returns zero noise (simplified)
    """
    def noise(t):
        return 0.0
    return noise


def simulate(tau, tauD, controller, model, x0, options) -> Dict[str, Any]:
    """
    Main simulation function
    Translated from MATLAB simulate.m

    Args:
        tau: reference trajectory
        tauD: transformed reference trajectory
        controller: controller object
        model: vehicle model function
        x0: initial state
        options: simulation options

    Returns:
        benchmark: simulation results dictionary
    """
    # Start timing
    start_time = time.time()

    # Time parameters
    h = options['stepSize']
    tspan = np.arange(0, tau.T + h, h)

    # Parameters
    p = options['p']
    pc = options['pc']

    # Noise function
    xi_m = generate_noise(options['V'], tau.T, h)

    # Check if controller is discrete
    is_discrete = hasattr(controller, 'intState0') and controller.intState0 is not None

    # Initialize variables
    T = tspan
    X = []
    intState = [] if is_discrete else None

    if is_discrete:
        print('Discrete controller simulation')
        # For now, implement continuous version
        # Discrete implementation would require custom ODE solver
        print('Using continuous approximation for discrete controller')

    print('Continuous controller simulation')

    # Define ODE function
    def odefun(t, x):
        # Get control input
        u = controller.compute_input(x + xi_m(t), tauD, t, pc)
        # Get state derivatives from model
        dx, y = model(t, x, u, p)
        return dx[:6]  # Return only the 6 vehicle states

    # Solve ODE
    sol = solve_ivp(odefun, [0, tau.T], x0[:6], t_eval=tspan, method='RK45')
    T = sol.t
    X = sol.y.T

    end_time = time.time()
    print(f"Simulation completed in {end_time - start_time:.2f} seconds")

    # Initialize control inputs array
    if is_discrete:
        U = np.zeros((len(T), len(controller.compute_input(X[0, :], tauD, T[0], pc, 0, None))))
    else:
        U = np.zeros((len(T), len(controller.compute_input(X[0, :], tauD, T[0], pc))))

    # Calculate tracking errors and control inputs
    ERROR = np.zeros((len(T), 2))
    MUXY = np.zeros((len(T), 2))

    for k in range(len(T)):
        # Get control input
        if is_discrete:
            U[k, :] = controller.compute_input(X[k, :], tauD, T[k], pc, 0, intState[k] if intState else None)
        else:
            U[k, :] = controller.compute_input(X[k, :], tauD, T[k], pc)

        # Calculate tracking errors
        Xd = tau.X(T[k])
        Yd = tau.Y(T[k])
        theta = tau.theta(T[k])

        ct = np.cos(theta)
        st = np.sin(theta)
        Rti = np.array([[ct, st], [-st, ct]])

        ERROR[k, :] = Rti @ (X[k, :2] - np.array([Xd, Yd]))

        # Tire saturation
        dx, y = model(0, X[k, :], U[k, :], p)
        sf = y[:2]  # front slip vector
        sr = y[2:4]  # rear slip vector
        MUXY[k, :] = [np.linalg.norm(sf), np.linalg.norm(sr)]

    # Calculate performance metrics
    ET = ERROR[:, 0]
    EN = ERROR[:, 1]
    MUF = MUXY[:, 0]
    MUR = MUXY[:, 1]

    # Average errors over time
    IntET = np.trapz(np.abs(ET), T) / tau.T
    IntEN = np.trapz(np.abs(EN), T) / tau.T
    IntMUF = np.trapz(MUF, T) / tau.T
    IntMUR = np.trapz(MUR, T) / tau.T

    # Create benchmark results
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
            'tauD': tauD,
            'options': options
        }
    }

    if is_discrete:
        benchmark['data']['intState'] = intState

    return benchmark


def get_vehicle_x0(model, tau, options):
    """
    Get initial vehicle state
    Translated from MATLAB getVehicleX0.m

    Args:
        model: vehicle model function
        tau: trajectory object
        options: simulation options

    Returns:
        x0: initial state vector
    """
    # Initial time
    t0 = 0.0

    # Get initial reference values
    X0 = tau.X(t0)
    Y0 = tau.Y(t0)
    theta0 = tau.theta(t0)
    vx0 = tau.dX(t0)
    vy0 = tau.dY(t0)
    omega0 = tau.dtheta(t0)

    # Initial state [X, Y, psi, vx, vy, omega]
    x0 = np.array([X0, Y0, theta0, vx0, vy0, omega0])

    return x0
