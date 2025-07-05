"""

Comparison between Discrete LQR and Continuous LQR path tracking controllers.

This script runs both controllers on the same path and compares their performance.

author: Based on Atsushi Sakai's original work (@Atsushi_twi)

"""

import pathlib
import sys

import matplotlib.pyplot as plt
import numpy as np

sys.path.append(str(pathlib.Path(__file__).parent.parent))
import continuous_lqr_steer_control as continuous_lqr

# Import both controllers
import lqr_steer_control as discrete_lqr
from CubicSpline import cubic_spline_planner


def run_comparison():
    """
    Run both discrete and continuous LQR controllers and compare results
    """
    print("Running comparison between Discrete LQR and Continuous LQR controllers...")

    # Define the same path for both controllers
    ax = [0.0, 6.0, 12.5, 10.0, 7.5, 3.0, -1.0]
    ay = [0.0, -3.0, -5.0, 6.5, 3.0, 5.0, -2.0]
    goal = [ax[-1], ay[-1]]

    # Generate spline path
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(ax, ay, ds=0.1)
    target_speed = 10.0 / 3.6  # simulation parameter km/h -> m/s

    # Calculate speed profile
    sp_discrete = discrete_lqr.calc_speed_profile(cx, cy, cyaw, target_speed)
    sp_continuous = continuous_lqr.calc_speed_profile(cx, cy, cyaw, target_speed)

    # Turn off animation for both controllers during comparison
    discrete_lqr.show_animation = False
    continuous_lqr.show_animation = False

    print("Running Discrete LQR controller...")
    t_d, x_d, y_d, yaw_d, v_d = discrete_lqr.closed_loop_prediction(cx, cy, cyaw, ck, sp_discrete, goal)

    print("Running Continuous LQR controller...")
    t_c, x_c, y_c, yaw_c, v_c = continuous_lqr.closed_loop_prediction(cx, cy, cyaw, ck, sp_continuous, goal)

    # Calculate tracking errors
    error_d = calculate_tracking_error(x_d, y_d, cx, cy)
    error_c = calculate_tracking_error(x_c, y_c, cx, cy)

    # Plot comparison
    plot_comparison(
        ax,
        ay,
        cx,
        cy,
        cyaw,
        ck,
        s,
        x_d,
        y_d,
        yaw_d,
        v_d,
        t_d,
        error_d,
        x_c,
        y_c,
        yaw_c,
        v_c,
        t_c,
        error_c,
    )

    # Print statistics
    print_statistics(error_d, error_c, t_d, t_c)


def calculate_tracking_error(x_traj, y_traj, cx, cy):
    """
    Calculate the tracking error for a given trajectory
    """
    errors = []
    for i in range(len(x_traj)):
        # Find nearest point on reference path
        dx = [x_traj[i] - icx for icx in cx]
        dy = [y_traj[i] - icy for icy in cy]
        d = [idx**2 + idy**2 for (idx, idy) in zip(dx, dy)]
        min_dist = min(d)
        errors.append(np.sqrt(min_dist))

    return errors


def plot_comparison(
    ax,
    ay,
    cx,
    cy,
    cyaw,
    ck,
    s,
    x_d,
    y_d,
    yaw_d,
    v_d,
    t_d,
    error_d,
    x_c,
    y_c,
    yaw_c,
    v_c,
    t_c,
    error_c,
):
    """
    Plot comparison between discrete and continuous LQR controllers
    """
    plt.figure(figsize=(15, 10))

    # Trajectory comparison
    plt.subplot(2, 3, 1)
    plt.plot(ax, ay, "xk", label="waypoints", markersize=8)
    plt.plot(cx, cy, "-r", label="reference path", linewidth=2)
    plt.plot(x_d, y_d, "-b", label="discrete LQR", linewidth=1.5)
    plt.plot(x_c, y_c, "-g", label="continuous LQR", linewidth=1.5)
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.legend()
    plt.title("Path Tracking Comparison")

    # Tracking error comparison
    plt.subplot(2, 3, 2)
    plt.plot(t_d[: len(error_d)], error_d, "-b", label="discrete LQR")
    plt.plot(t_c[: len(error_c)], error_c, "-g", label="continuous LQR")
    plt.grid(True)
    plt.xlabel("time [s]")
    plt.ylabel("tracking error [m]")
    plt.legend()
    plt.title("Tracking Error vs Time")

    # Velocity comparison
    plt.subplot(2, 3, 3)
    plt.plot(t_d, [iv * 3.6 for iv in v_d], "-b", label="discrete LQR")
    plt.plot(t_c, [iv * 3.6 for iv in v_c], "-g", label="continuous LQR")
    plt.grid(True)
    plt.xlabel("time [s]")
    plt.ylabel("velocity [km/h]")
    plt.legend()
    plt.title("Velocity Profile")

    # Yaw angle comparison
    plt.subplot(2, 3, 4)
    plt.plot(s, [np.rad2deg(iyaw) for iyaw in cyaw], "-r", label="reference", linewidth=2)
    if len(yaw_d) > len(s):
        plt.plot(
            t_d[: len(s)],
            [np.rad2deg(iyaw) for iyaw in yaw_d[: len(s)]],
            "-b",
            label="discrete LQR",
        )
    else:
        plt.plot(t_d, [np.rad2deg(iyaw) for iyaw in yaw_d], "-b", label="discrete LQR")

    if len(yaw_c) > len(s):
        plt.plot(
            t_c[: len(s)],
            [np.rad2deg(iyaw) for iyaw in yaw_c[: len(s)]],
            "-g",
            label="continuous LQR",
        )
    else:
        plt.plot(t_c, [np.rad2deg(iyaw) for iyaw in yaw_c], "-g", label="continuous LQR")

    plt.grid(True)
    plt.xlabel("time [s]")
    plt.ylabel("yaw angle [deg]")
    plt.legend()
    plt.title("Yaw Angle Tracking")

    # Path curvature
    plt.subplot(2, 3, 5)
    plt.plot(s, ck, "-r", label="curvature", linewidth=2)
    plt.grid(True)
    plt.xlabel("path length [m]")
    plt.ylabel("curvature [1/m]")
    plt.legend()
    plt.title("Reference Path Curvature")

    # Error statistics
    plt.subplot(2, 3, 6)
    methods = ["Discrete LQR", "Continuous LQR"]
    mean_errors = [np.mean(error_d), np.mean(error_c)]
    max_errors = [np.max(error_d), np.max(error_c)]

    x_pos = np.arange(len(methods))
    width = 0.35

    plt.bar(
        x_pos - width / 2,
        mean_errors,
        width,
        label="Mean Error",
        color=["blue", "green"],
        alpha=0.7,
    )
    plt.bar(
        x_pos + width / 2,
        max_errors,
        width,
        label="Max Error",
        color=["blue", "green"],
        alpha=0.4,
    )

    plt.xlabel("Controller Type")
    plt.ylabel("Error [m]")
    plt.title("Error Statistics")
    plt.xticks(x_pos, methods)
    plt.legend()
    plt.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()


def print_statistics(error_d, error_c, t_d, t_c):
    """
    Print comparison statistics
    """
    print("\n" + "=" * 50)
    print("PERFORMANCE COMPARISON RESULTS")
    print("=" * 50)

    print(f"\nDiscrete LQR Controller:")
    print(f"  Mean tracking error: {np.mean(error_d):.4f} m")
    print(f"  Max tracking error:  {np.max(error_d):.4f} m")
    print(f"  RMS tracking error:  {np.sqrt(np.mean(np.array(error_d)**2)):.4f} m")
    print(f"  Simulation time:     {t_d[-1]:.2f} s")

    print(f"\nContinuous LQR Controller:")
    print(f"  Mean tracking error: {np.mean(error_c):.4f} m")
    print(f"  Max tracking error:  {np.max(error_c):.4f} m")
    print(f"  RMS tracking error:  {np.sqrt(np.mean(np.array(error_c)**2)):.4f} m")
    print(f"  Simulation time:     {t_c[-1]:.2f} s")

    # Performance comparison
    print(f"\nPerformance Improvement (Continuous vs Discrete):")
    mean_improvement = (np.mean(error_d) - np.mean(error_c)) / np.mean(error_d) * 100
    max_improvement = (np.max(error_d) - np.max(error_c)) / np.max(error_d) * 100
    rms_improvement = (
        (np.sqrt(np.mean(np.array(error_d) ** 2)) - np.sqrt(np.mean(np.array(error_c) ** 2)))
        / np.sqrt(np.mean(np.array(error_d) ** 2))
        * 100
    )

    print(f"  Mean error improvement: {mean_improvement:+.2f}%")
    print(f"  Max error improvement:  {max_improvement:+.2f}%")
    print(f"  RMS error improvement:  {rms_improvement:+.2f}%")

    print("\n" + "=" * 50)


if __name__ == "__main__":
    run_comparison()
