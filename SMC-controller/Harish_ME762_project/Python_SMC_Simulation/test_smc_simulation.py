"""
Test script for SMC simulation
Equivalent to MATLAB TEST.m
"""

import numpy as np
from .models.vehicle_parameters import get_default_options
from .models.vmodel_A import vmodel_A
from .controllers.kinematic_sliding_controller import KinSliding
from .trajectory.trajectory import Trajectory
from .simulation import simulate, get_vehicle_x0


def test_smc_simulation(scenario='single_lane_change'):
    """
    Test the SMC simulation with specified scenario

    Args:
        scenario: 'single_lane_change' or 'double_lane_change'
    """

    print("=== SMC Simulation Test ===")

    # Specify scenario
    if scenario == 'single_lane_change':
        scenario_name = '01_single_lane_change'
    elif scenario == 'double_lane_change':
        scenario_name = '02_double_lane_change'
    else:
        scenario_name = 'default'

    print(f"Scenario: {scenario_name}")

    # Initialize controller
    controller = KinSliding()
    model = vmodel_A
    options = get_default_options()

    # Set friction coefficient
    options['p']['mu0'] = 1.0
    options['pc']['mu0'] = 1.0

    # Load trajectory
    tau = Trajectory()
    tau.load(scenario_name)

    # Transform trajectory based on control point
    if controller.control_point == 'CO':
        # Center of mass control - would need more complex transformation
        tau.solveID(options['pc'])  # Not implemented yet
        tauD = tau.transform(options['p']['J'] / options['p']['l_R'] / options['p']['m']).make_static(100)
    elif controller.control_point == 'REAR':
        # Rear axle control
        tau.solveID(options['pc'])  # Not implemented yet
        tauD = tau.transform(-options['p']['l_R']).make_static(100)
    else:
        tauD = tau

    # Scale cornering stiffness if friction is reduced
    if options['pc']['mu0'] < 1.0:
        options['p']['cf'] = options['p']['cf'] * options['p']['mu0']
        options['p']['cr'] = options['p']['cr'] * options['p']['mu0']
        options['pc']['cf'] = options['pc']['cf'] * options['pc']['mu0']
        options['pc']['cr'] = options['pc']['cr'] * options['pc']['mu0']

    # Get initial state
    options['n'] = len(model(0, np.zeros(10), np.zeros(5), options['p']))
    x0 = get_vehicle_x0(model, tau, options)

    # Add some initial perturbation
    x0 = x0 + np.array([0, -0.2, np.deg2rad(-3), 0, 0, 0])

    # Initialize controller
    controller = controller.init(model, options)

    # Run simulation
    print("Running simulation...")
    benchmark = simulate(tau, tauD, controller, model, x0, options)

    # Display results
    print("\n=== Simulation Results ===")
    print(f"Controller: {benchmark['controller']}")
    print(f"Scenario: {benchmark['scenario']}")
    print(f"Max lateral error: {benchmark['max_error'][0]:.4f} m")
    print(f"Max heading error: {benchmark['max_error'][1]:.4f} rad")
    print(f"Avg lateral error: {benchmark['avg_error'][0]:.4f} m")
    print(f"Avg heading error: {benchmark['avg_error'][1]:.4f} rad")
    print(f"End lateral error: {benchmark['end_error'][0]:.4f} m")
    print(f"End heading error: {benchmark['end_error'][1]:.4f} rad")
    print(f"Avg front tire saturation: {benchmark['avg_musat'][0]:.4f}")
    print(f"Avg rear tire saturation: {benchmark['avg_musat'][1]:.4f}")
    print(f"Simulation time: {benchmark['total_time']:.2f} s")

    # Visualize results if requested
    if options['DISPLAY_OUTPUT']:
        visualize_results(benchmark)

    return benchmark


def visualize_results(benchmark):
    """
    Visualize simulation results
    """
    import matplotlib.pyplot as plt

    data = benchmark['data']
    T = data['T']
    X = data['X']
    tau = data['tau']
    errors = data['error']

    plt.figure(figsize=(15, 10))

    # Trajectory plot
    plt.subplot(2, 3, 1)
    plt.plot(X[:, 0], X[:, 1], 'b-', linewidth=2, label='Vehicle path')
    plt.plot(tau.X(T), tau.Y(T), 'r--', linewidth=2, label='Reference path')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Vehicle Trajectory')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')

    # Position errors
    plt.subplot(2, 3, 2)
    plt.plot(T, errors[:, 0], 'r-', label='Lateral error')
    plt.plot(T, errors[:, 1], 'b-', label='Heading error')
    plt.xlabel('Time [s]')
    plt.ylabel('Error')
    plt.title('Tracking Errors')
    plt.legend()
    plt.grid(True)

    # Velocity
    plt.subplot(2, 3, 3)
    plt.plot(T, X[:, 3], 'b-', label='Longitudinal velocity')
    plt.plot(T, X[:, 4], 'r-', label='Lateral velocity')
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [m/s]')
    plt.title('Vehicle Velocities')
    plt.legend()
    plt.grid(True)

    # Control inputs
    U = data['U']
    plt.subplot(2, 3, 4)
    if U.shape[1] >= 2:
        plt.plot(T, np.rad2deg(U[:, 0]), 'b-', label='Steering angle')
        plt.xlabel('Time [s]')
        plt.ylabel('Steering angle [deg]')
        plt.title('Control Inputs')
        plt.legend()
        plt.grid(True)

    # Yaw rate
    plt.subplot(2, 3, 5)
    plt.plot(T, X[:, 5], 'b-', label='Yaw rate')
    plt.xlabel('Time [s]')
    plt.ylabel('Yaw rate [rad/s]')
    plt.title('Yaw Rate')
    plt.legend()
    plt.grid(True)

    # Tire saturation
    plt.subplot(2, 3, 6)
    plt.plot(T, np.ones_like(T) * benchmark['avg_musat'][0], 'r-', label='Front tire avg')
    plt.plot(T, np.ones_like(T) * benchmark['avg_musat'][1], 'b-', label='Rear tire avg')
    plt.xlabel('Time [s]')
    plt.ylabel('Tire saturation')
    plt.title('Tire Saturation')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    # Run test with single lane change
    benchmark = test_smc_simulation('single_lane_change')

    # Optionally run double lane change
    # benchmark = test_smc_simulation('double_lane_change')
