"""
Basic demonstration of the SMC trajectory tracking controller.

This example shows how to use the kinematic sliding mode controller
to track a circular trajectory.
"""

# Set matplotlib backend to Qt for interactive plotting
import matplotlib
matplotlib.use('Qt5Agg')

import numpy as np
import matplotlib.pyplot as plt
from smc_controller import (
    KinSlidingController,
    VehicleModel,
    create_test_trajectory,
    simulate,
    default_options,
    CARParameters
)


def main():
    """Run basic SMC controller demonstration."""
    print("SMC Controller Basic Demo")
    print("=" * 40)

    # Create trajectory
    print("Creating test trajectory...")
    tau = create_test_trajectory()
    tau_d = tau.transform(0.0)  # No transformation for basic demo

    # Create controller
    print("Initializing controller...")
    controller = KinSlidingController()

    # Create vehicle model
    print("Setting up vehicle model...")
    params = CARParameters.default()
    model = VehicleModel(params)

    # Set initial state [X, Y, psi, vx, vy, omega]
    x0 = np.array([0.0, 50.0, np.pi/2, 10.0, 0.0, 0.0])  # Start at (0, 50) going north

    # Set simulation options
    options = default_options()

    # Run simulation
    print("Running simulation...")
    benchmark = simulate(tau, tau_d, controller, model, x0, options)

    # Display results
    print("\nSimulation Results:")
    print(f"Controller: {benchmark['controller']}")
    print(f"Scenario: {benchmark['scenario']}")
    print(f"Max Error: Lateral={benchmark['max_error'][0]:.3f}m, Longitudinal={benchmark['max_error'][1]:.3f}m")
    print(f"Avg Error: Lateral={benchmark['avg_error'][0]:.3f}m, Longitudinal={benchmark['avg_error'][1]:.3f}m")
    print(f"End Error: Lateral={benchmark['end_error'][0]:.3f}m, Longitudinal={benchmark['end_error'][1]:.3f}m")
    print(f"Avg Tire Saturation: Front={benchmark['avg_musat'][0]:.3f}, Rear={benchmark['avg_musat'][1]:.3f}")

    # Plot results
    print("\nPlotting results...")
    plot_results(benchmark)

    print("\nDemo completed!")


def plot_results(benchmark):
    """Plot simulation results."""
    data = benchmark['data']
    T = data['T']
    X = data['X']
    error = data['error']
    U = data['U']

    fig, axes = plt.subplots(2, 2, figsize=(12, 10))

    # Trajectory tracking
    ax1 = axes[0, 0]
    ax1.plot(X[:, 0], X[:, 1], 'b-', label='Actual Path', linewidth=2)
    ax1.plot([benchmark['data']['tau'].X(t) for t in T],
             [benchmark['data']['tau'].Y(t) for t in T], 'r--', label='Reference Path', linewidth=2)
    ax1.set_xlabel('X Position [m]')
    ax1.set_ylabel('Y Position [m]')
    ax1.set_title('Trajectory Tracking')
    ax1.legend()
    ax1.grid(True)
    ax1.axis('equal')

    # Tracking error
    ax2 = axes[0, 1]
    ax2.plot(T, error[:, 0], 'r-', label='Lateral Error', linewidth=2)
    ax2.plot(T, error[:, 1], 'b-', label='Longitudinal Error', linewidth=2)
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Error [m]')
    ax2.set_title('Tracking Error')
    ax2.legend()
    ax2.grid(True)

    # Control inputs
    ax3 = axes[1, 0]
    ax3.plot(T, np.rad2deg(U[:, 0]), 'g-', label='Steering Angle [deg]', linewidth=2)
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Steering Angle [deg]')
    ax3.set_title('Control Inputs')
    ax3.legend()
    ax3.grid(True)

    # Vehicle velocity
    ax4 = axes[1, 1]
    velocity = np.sqrt(X[:, 3]**2 + X[:, 4]**2)
    ax4.plot(T, X[:, 3], 'b-', label='Longitudinal Velocity', linewidth=2)
    ax4.plot(T, velocity, 'r--', label='Total Velocity', linewidth=2)
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Velocity [m/s]')
    ax4.set_title('Vehicle Velocity')
    ax4.legend()
    ax4.grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
