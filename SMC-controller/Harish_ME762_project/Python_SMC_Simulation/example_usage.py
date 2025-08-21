#!/usr/bin/env python3
"""
Example usage of the SMC simulation package
"""

import numpy as np
import matplotlib.pyplot as plt
from .models.vehicle_parameters import get_default_options
from .models.vmodel_A import vmodel_A
from .controllers.kinematic_sliding_controller import KinSliding
from .trajectory.trajectory import Trajectory
from .simulation import simulate, get_vehicle_x0


def run_example_simulation():
    """Run a basic example simulation"""

    print("=== SMC Simulation Example ===")

    # Setup simulation components
    options = get_default_options()
    controller = KinSliding()
    model = vmodel_A

    # Create trajectory
    tau = Trajectory()
    tau.load('01_single_lane_change')  # Single lane change

    # Simple trajectory transformation (no complex internal dynamics for this example)
    tauD = tau.transform(-options['p']['l_R'])

    # Get initial state
    x0 = get_vehicle_x0(model, tau, options)
    print(f"Initial state: X={x0[0]:.2f}, Y={x0[1]:.2f}, ψ={np.rad2deg(x0[2]):.2f}°")

    # Initialize controller
    controller = controller.init(model, options)

    # Run simulation
    print("Running simulation...")
    benchmark = simulate(tau, tauD, controller, model, x0, options)

    # Print results
    print(f"\nResults:")
    print(f"  Final position: X={benchmark['data']['X'][-1, 0]:.2f}, Y={benchmark['data']['X'][-1, 1]:.2f}")
    print(f"  Max lateral error: {benchmark['max_error'][0]:.3f} m")
    print(f"  Avg lateral error: {benchmark['avg_error'][0]:.3f} m")
    print(f"  Simulation time: {benchmark['total_time']:.1f} s")

    # Simple visualization
    plot_results(benchmark)

    return benchmark


def plot_results(benchmark):
    """Simple plotting of results"""

    data = benchmark['data']
    T = data['T']
    X = data['X']
    tau = data['tau']
    errors = data['error']

    fig, axes = plt.subplots(2, 2, figsize=(12, 8))

    # Trajectory
    axes[0, 0].plot(X[:, 0], X[:, 1], 'b-', linewidth=2, label='Vehicle')
    t_ref = np.linspace(0, tau.T, 100)
    axes[0, 0].plot(tau.X(t_ref), tau.Y(t_ref), 'r--', linewidth=2, label='Reference')
    axes[0, 0].set_xlabel('X [m]')
    axes[0, 0].set_ylabel('Y [m]')
    axes[0, 0].set_title('Trajectory')
    axes[0, 0].legend()
    axes[0, 0].grid(True)
    axes[0, 0].axis('equal')

    # Errors
    axes[0, 1].plot(T, errors[:, 0], 'r-', label='Lateral error')
    axes[0, 1].plot(T, errors[:, 1], 'b-', label='Heading error')
    axes[0, 1].set_xlabel('Time [s]')
    axes[0, 1].set_ylabel('Error')
    axes[0, 1].set_title('Tracking Errors')
    axes[0, 1].legend()
    axes[0, 1].grid(True)

    # Velocity
    axes[1, 0].plot(T, X[:, 3], 'b-', label='Longitudinal')
    axes[1, 0].plot(T, X[:, 4], 'r-', label='Lateral')
    axes[1, 0].set_xlabel('Time [s]')
    axes[1, 0].set_ylabel('Velocity [m/s]')
    axes[1, 0].set_title('Vehicle Velocities')
    axes[1, 0].legend()
    axes[1, 0].grid(True)

    # Yaw rate
    axes[1, 1].plot(T, X[:, 5], 'b-', label='Yaw rate')
    axes[1, 1].set_xlabel('Time [s]')
    axes[1, 1].set_ylabel('Yaw rate [rad/s]')
    axes[1, 1].set_title('Yaw Rate')
    axes[1, 1].legend()
    axes[1, 1].grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    run_example_simulation()
