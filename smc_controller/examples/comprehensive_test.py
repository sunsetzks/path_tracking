"""
Comprehensive test of SMC controller performance.

Tests the controller with different trajectories and conditions.
"""

# Set matplotlib backend to Qt for interactive plotting
import matplotlib
matplotlib.use('Qt5Agg')

import numpy as np
import matplotlib.pyplot as plt
from smc_controller import (
    KinSlidingController,
    VehicleModel,
    simulate,
    default_options,
    CARParameters,
    Trajectory
)


def create_lane_change_trajectory() -> Trajectory:
    """Create a lane change trajectory."""
    traj = Trajectory()
    traj.id = "lane_change"
    traj.T = 6.0

    # Lane change parameters
    lane_width = 3.5
    duration = traj.T

    # Smooth lane change using polynomial
    def lane_change_profile(t):
        # Normalize time
        s = t / duration
        # Polynomial for smooth transition: 3*s^2 - 2*s^3
        transition = 3 * s**2 - 2 * s**3
        return lane_width * transition

    # Straight road with lane change
    traj.X = lambda t: 20 * t  # Constant speed forward
    traj.Y = lambda t: lane_change_profile(t)
    traj.theta = lambda t: 0.0  # Moving straight ahead
    traj.dX = lambda t: 20.0  # Constant longitudinal speed
    traj.dY = lambda t: (lane_width / duration) * (6 * (t/duration) - 6 * (t/duration)**2)
    traj.dtheta = lambda t: 0.0
    traj.ddX = lambda t: 0.0
    traj.ddY = lambda t: (lane_width / duration**2) * (6 - 12 * (t/duration))
    traj.ddtheta = lambda t: 0.0
    traj.dddtheta = lambda t: 0.0
    traj.v = lambda t: 20.0
    traj.a = lambda t: 0.0
    traj.kappa = lambda t: 0.0  # Straight line

    return traj


def run_scenario(name: str, tau: Trajectory, x0: np.ndarray, noise_level: float = 0.0):
    """Run a simulation scenario."""
    print(f"\nRunning scenario: {name}")
    print("-" * 30)

    # Setup
    tau_d = tau.transform(0.0)
    controller = KinSlidingController()
    model = VehicleModel()
    options = default_options()
    options.V = noise_level

    # Run simulation
    benchmark = simulate(tau, tau_d, controller, model, x0, options)

    # Print results
    print(f"Max Error: {benchmark['max_error']}")
    print(f"Avg Error: {benchmark['avg_error']}")
    print(f"Success: {benchmark['avg_error'][0] < 0.5 and benchmark['avg_error'][1] < 0.5}")

    return benchmark


def main():
    """Run comprehensive tests."""
    print("SMC Controller Comprehensive Test")
    print("=" * 50)

    scenarios = []

    # Scenario 1: Circle tracking
    print("\n1. Circle Tracking")
    tau_circle = Trajectory()
    tau_circle.id = "circle"
    tau_circle.T = 20.0
    radius = 50.0
    speed = 15.0

    tau_circle.X = lambda t: radius * np.cos(2 * np.pi * t / tau_circle.T)
    tau_circle.Y = lambda t: radius * np.sin(2 * np.pi * t / tau_circle.T)
    tau_circle.theta = lambda t: np.pi/2 + 2 * np.pi * t / tau_circle.T
    tau_circle.dX = lambda t: -radius * 2 * np.pi / tau_circle.T * np.sin(2 * np.pi * t / tau_circle.T)
    tau_circle.dY = lambda t: radius * 2 * np.pi / tau_circle.T * np.cos(2 * np.pi * t / tau_circle.T)
    tau_circle.dtheta = lambda t: 2 * np.pi / tau_circle.T
    tau_circle.ddX = lambda t: -radius * (2 * np.pi / tau_circle.T)**2 * np.cos(2 * np.pi * t / tau_circle.T)
    tau_circle.ddY = lambda t: -radius * (2 * np.pi / tau_circle.T)**2 * np.sin(2 * np.pi * t / tau_circle.T)
    tau_circle.ddtheta = lambda t: 0.0
    tau_circle.dddtheta = lambda t: 0.0
    tau_circle.v = lambda t: speed
    tau_circle.a = lambda t: 0.0
    tau_circle.kappa = lambda t: 1.0 / radius

    x0_circle = np.array([0.0, radius, np.pi/2, speed, 0.0, 0.0])
    result_circle = run_scenario("Circle Tracking", tau_circle, x0_circle)
    scenarios.append(("Circle", result_circle))

    # Scenario 2: Lane change
    print("\n2. Lane Change")
    tau_lane = create_lane_change_trajectory()
    x0_lane = np.array([0.0, 0.0, 0.0, 20.0, 0.0, 0.0])
    result_lane = run_scenario("Lane Change", tau_lane, x0_lane)
    scenarios.append(("Lane Change", result_lane))

    # Scenario 3: Circle with noise
    print("\n3. Circle with Measurement Noise")
    result_noisy = run_scenario("Circle (Noisy)", tau_circle, x0_circle, noise_level=0.1)
    scenarios.append(("Circle (Noisy)", result_noisy))

    # Summary
    print("\n" + "=" * 50)
    print("SUMMARY")
    print("=" * 50)

    for name, result in scenarios:
        success = result['avg_error'][0] < 0.5 and result['avg_error'][1] < 0.5
        print(f"{name:15s}: Max Error={result['max_error']}, Success={'✓' if success else '✗'}")

    # Plot comparison
    plot_comparison(scenarios)


def plot_comparison(scenarios):
    """Plot comparison of different scenarios."""
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    colors = ['blue', 'red', 'green']
    names = [name for name, _ in scenarios]

    for i, (name, result) in enumerate(scenarios):
        data = result['data']
        T = data['T']
        X = data['X']
        error = data['error']
        color = colors[i]

        # Trajectory
        axes[0, 0].plot(X[:, 0], X[:, 1], color=color, label=name, linewidth=2)

        # Error
        axes[0, 1].plot(T, error[:, 0], color=color, label=f'{name} (Lateral)', linewidth=2)
        axes[1, 0].plot(T, error[:, 1], color=color, label=f'{name} (Longitudinal)', linewidth=2)

        # Velocity
        velocity = np.sqrt(X[:, 3]**2 + X[:, 4]**2)
        axes[1, 1].plot(T, velocity, color=color, label=name, linewidth=2)

    # Format plots
    axes[0, 0].set_xlabel('X [m]')
    axes[0, 0].set_ylabel('Y [m]')
    axes[0, 0].set_title('Trajectories')
    axes[0, 0].legend()
    axes[0, 0].grid(True)
    axes[0, 0].axis('equal')

    axes[0, 1].set_xlabel('Time [s]')
    axes[0, 1].set_ylabel('Lateral Error [m]')
    axes[0, 1].set_title('Lateral Tracking Error')
    axes[0, 1].legend()
    axes[0, 1].grid(True)

    axes[1, 0].set_xlabel('Time [s]')
    axes[1, 0].set_ylabel('Longitudinal Error [m]')
    axes[1, 0].set_title('Longitudinal Tracking Error')
    axes[1, 0].legend()
    axes[1, 0].grid(True)

    axes[1, 1].set_xlabel('Time [s]')
    axes[1, 1].set_ylabel('Velocity [m/s]')
    axes[1, 1].set_title('Vehicle Speed')
    axes[1, 1].legend()
    axes[1, 1].grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
