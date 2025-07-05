"""
Vehicle Model Example with Plotting Tests

This example demonstrates the enhanced vehicle model with:
- Rate-based control vs Direct control
- Effects of actuator delays
- Forward and reverse driving scenarios
- Comprehensive plotting for analysis

Author: Assistant
"""

import math
import numpy as np
import matplotlib.pyplot as plt
from PathTracking.vehicle_model import VehicleModel, VehicleState, simulate_vehicle_motion
from PathTracking.config import load_config, VehicleConfig, SimulationConfig


def run_forward_driving_test():
    """
    Test forward driving scenarios with different control methods and delays

    Returns:
        dict: Dictionary containing trajectory data for plotting
    """
    print("=== Forward Driving Test ===")

    # Load configuration and create variants for different tests
    config = load_config()

    # Configuration for vehicle without delay
    config_no_delay = VehicleConfig()
    config_no_delay.steering_delay = 0.0
    config_no_delay.acceleration_delay = 0.0

    # Configuration for vehicle with delay
    config_with_delay = VehicleConfig()
    config_with_delay.steering_delay = 0.3
    config_with_delay.acceleration_delay = 0.2

    # Simulation configuration
    sim_config = SimulationConfig()
    sim_config.time_step = 0.1

    # Create vehicle models for comparison
    vehicle_no_delay = VehicleModel(config=config_no_delay)
    vehicle_with_delay = VehicleModel(config=config_with_delay)
    vehicle_direct_control = VehicleModel(config=config_with_delay)
    vehicle_direct_no_delay = VehicleModel(config=config_no_delay)

    # Set initial state [position_x, position_y, yaw_angle, velocity, steering_angle]
    initial_state_array = [0.0, 0.0, 0.0, 10.0, 0.0]
    initial_state = VehicleState.from_array(initial_state_array)

    # Simulation parameters
    time_step = sim_config.time_step
    simulation_time = 10.0
    time_steps = int(simulation_time / time_step)

    # Control sequence: turning maneuver
    control_sequence = []
    for step_index in range(time_steps):
        if step_index < time_steps // 3:
            # Turn left
            steering_rate = np.deg2rad(10)  # steering angle velocity
            acceleration = 0.0  # no acceleration
        elif step_index < 2 * time_steps // 3:
            # Straight
            steering_rate = np.deg2rad(-5)  # return to straight
            acceleration = 1.0  # accelerate
        else:
            # Turn right
            steering_rate = np.deg2rad(-15)
            acceleration = -1.0  # decelerate

        control_sequence.append([steering_rate, acceleration])

    # Simulate vehicle motion for rate-based control
    trajectory_no_delay = simulate_vehicle_motion(initial_state, control_sequence, config_no_delay, sim_config)
    trajectory_with_delay = simulate_vehicle_motion(initial_state, control_sequence, config_with_delay, sim_config)

    # Convert VehicleState objects to arrays for backward compatibility with plotting
    trajectory_no_delay = np.array([state.to_array() for state in trajectory_no_delay])
    trajectory_with_delay = np.array([state.to_array() for state in trajectory_with_delay])

    # Demonstrate direct control methods
    vehicle_direct_control.set_state(initial_state_array)
    vehicle_direct_no_delay.set_state(initial_state_array)
    trajectory_direct_control = [vehicle_direct_control.get_state_array()]
    trajectory_direct_no_delay = [vehicle_direct_no_delay.get_state_array()]

    for step_index in range(time_steps):
        # Direct control: specify target steering angle and velocity directly
        if step_index < time_steps // 3:
            # Turn left with specific steering angle and velocity
            target_steering = np.deg2rad(15)  # target steering angle
            target_velocity = 10.0  # target velocity
        elif step_index < 2 * time_steps // 3:
            # Straight with higher velocity
            target_steering = np.deg2rad(0)
            target_velocity = 15.0
        else:
            # Turn right with reduced velocity
            target_steering = np.deg2rad(-20)
            target_velocity = 8.0

        state = vehicle_direct_control.update_with_direct_control([target_steering, target_velocity], time_step)
        trajectory_direct_control.append(state.to_array())

        state = vehicle_direct_no_delay.update_with_direct_control([target_steering, target_velocity], time_step)
        trajectory_direct_no_delay.append(state.to_array())

    trajectory_direct_control = np.array(trajectory_direct_control)
    trajectory_direct_no_delay = np.array(trajectory_direct_no_delay)

    return {
        "time_step": time_step,
        "time_steps": time_steps,
        "trajectory_no_delay": trajectory_no_delay,
        "trajectory_with_delay": trajectory_with_delay,
        "trajectory_direct_control": trajectory_direct_control,
        "trajectory_direct_no_delay": trajectory_direct_no_delay,
    }


def run_reverse_driving_test():
    """
    Test reverse driving scenarios including parking maneuvers

    Returns:
        dict: Dictionary containing reverse trajectory data for plotting
    """
    print("=== Reverse Driving Test ===")

    # Create vehicle configurations for reverse driving comparison
    config_reverse_no_delay = VehicleConfig()
    config_reverse_no_delay.min_velocity = -15.0
    config_reverse_no_delay.max_velocity = 50.0
    config_reverse_no_delay.steering_delay = 0.0
    config_reverse_no_delay.acceleration_delay = 0.0

    config_reverse_with_delay = VehicleConfig()
    config_reverse_with_delay.min_velocity = -15.0
    config_reverse_with_delay.max_velocity = 50.0
    config_reverse_with_delay.steering_delay = 0.2
    config_reverse_with_delay.acceleration_delay = 0.15

    # Simulation configuration
    sim_config = SimulationConfig()
    sim_config.time_step = 0.1

    # Create vehicle models for reverse driving comparison
    vehicle_reverse_no_delay = VehicleModel(config=config_reverse_no_delay)
    vehicle_reverse_with_delay = VehicleModel(config=config_reverse_with_delay)

    # Set initial state for reverse driving [x, y, yaw, velocity, steering]
    # Start facing forward but will reverse
    initial_state_reverse_array = [0.0, 0.0, 0.0, 0.0, 0.0]
    initial_state_reverse = VehicleState.from_array(initial_state_reverse_array)

    # Simulation parameters
    time_step = sim_config.time_step
    simulation_time = 15.0  # Longer simulation for complex maneuver
    time_steps = int(simulation_time / time_step)

    # Control sequence for reverse parking maneuver
    reverse_control_sequence = []
    for step_index in range(time_steps):
        if step_index < time_steps // 5:
            # Phase 1: Start reversing straight
            steering_rate = 0.0
            acceleration = -2.0  # negative acceleration for reverse
        elif step_index < 2 * time_steps // 5:
            # Phase 2: Reverse while turning left
            steering_rate = np.deg2rad(20)  # turn steering wheel left
            acceleration = 0.0  # maintain reverse speed
        elif step_index < 3 * time_steps // 5:
            # Phase 3: Continue reversing, straighten out
            steering_rate = np.deg2rad(-15)  # straighten wheel
            acceleration = 0.0
        elif step_index < 4 * time_steps // 5:
            # Phase 4: Final adjustment - turn right while reversing
            steering_rate = np.deg2rad(-25)
            acceleration = 0.5  # slight deceleration
        else:
            # Phase 5: Stop
            steering_rate = 0.0
            acceleration = 2.0  # positive acceleration to stop

        reverse_control_sequence.append([steering_rate, acceleration])

    # Simulate reverse motion
    trajectory_reverse_no_delay = simulate_vehicle_motion(
        initial_state_reverse, reverse_control_sequence, config_reverse_no_delay, sim_config
    )
    trajectory_reverse_with_delay = simulate_vehicle_motion(
        initial_state_reverse, reverse_control_sequence, config_reverse_with_delay, sim_config
    )

    # Convert VehicleState objects to arrays for backward compatibility with plotting
    trajectory_reverse_no_delay = np.array([state.to_array() for state in trajectory_reverse_no_delay])
    trajectory_reverse_with_delay = np.array([state.to_array() for state in trajectory_reverse_with_delay])

    # Test direct control in reverse using VehicleModel
    vehicle_reverse_direct = VehicleModel(config=config_reverse_with_delay)
    vehicle_reverse_direct.set_state(initial_state_reverse_array)
    trajectory_reverse_direct = [vehicle_reverse_direct.get_state_array()]

    for step_index in range(time_steps):
        # Direct control for reverse parking
        if step_index < time_steps // 4:
            # Reverse straight
            target_steering = np.deg2rad(0)
            target_velocity = -8.0  # negative velocity for reverse
        elif step_index < time_steps // 2:
            # Reverse while turning
            target_steering = np.deg2rad(25)
            target_velocity = -6.0
        elif step_index < 3 * time_steps // 4:
            # Straighten while reversing
            target_steering = np.deg2rad(-10)
            target_velocity = -4.0
        else:
            # Final positioning
            target_steering = np.deg2rad(0)
            target_velocity = 0.0  # stop

        state = vehicle_reverse_direct.update_with_direct_control([target_steering, target_velocity], time_step)
        trajectory_reverse_direct.append(state.to_array())

    trajectory_reverse_direct = np.array(trajectory_reverse_direct)

    return {
        "time_step": time_step,
        "time_steps": time_steps,
        "trajectory_reverse_no_delay": trajectory_reverse_no_delay,
        "trajectory_reverse_with_delay": trajectory_reverse_with_delay,
        "trajectory_reverse_direct": trajectory_reverse_direct,
    }


def run_combined_forward_reverse_test():
    """
    Test combined forward and reverse driving in a single maneuver

    Returns:
        dict: Dictionary containing combined trajectory data
    """
    print("=== Combined Forward/Reverse Test ===")

    # Create vehicle configuration for combined maneuver
    config_combined = VehicleConfig()
    config_combined.min_velocity = -12.0
    config_combined.max_velocity = 30.0
    config_combined.steering_delay = 0.1
    config_combined.acceleration_delay = 0.1

    vehicle = VehicleModel(config=config_combined)

    # Initial state
    initial_state = [0.0, 0.0, 0.0, 0.0, 0.0]
    vehicle.set_state(initial_state)

    time_step = 0.1
    simulation_time = 20.0
    time_steps = int(simulation_time / time_step)

    trajectory_combined = [vehicle.get_state_array()]

    for step_index in range(time_steps):
        progress = step_index / time_steps

        if progress < 0.25:
            # Phase 1: Forward driving with turn
            target_steering = np.deg2rad(15) * math.sin(progress * 8 * math.pi)
            target_velocity = 15.0
        elif progress < 0.4:
            # Phase 2: Slow down and stop
            target_steering = np.deg2rad(0)
            target_velocity = 15.0 * (1 - (progress - 0.25) / 0.15)
        elif progress < 0.6:
            # Phase 3: Reverse with S-curve
            target_steering = np.deg2rad(-20) * math.sin((progress - 0.4) * 10 * math.pi)
            target_velocity = -8.0
        elif progress < 0.75:
            # Phase 4: Stop and change to forward
            target_steering = np.deg2rad(0)
            target_velocity = -8.0 * (1 - (progress - 0.6) / 0.15)
        else:
            # Phase 5: Forward driving to finish
            target_steering = np.deg2rad(10)
            target_velocity = 12.0

        state = vehicle.update_with_direct_control([target_steering, target_velocity], time_step)
        trajectory_combined.append(state.to_array())

    trajectory_combined = np.array(trajectory_combined)

    return {"time_step": time_step, "time_steps": time_steps, "trajectory_combined": trajectory_combined}


def plot_forward_driving_results(forward_data):
    """Plot results from forward driving test"""
    trajectory_no_delay = forward_data["trajectory_no_delay"]
    trajectory_with_delay = forward_data["trajectory_with_delay"]
    trajectory_direct_control = forward_data["trajectory_direct_control"]
    trajectory_direct_no_delay = forward_data["trajectory_direct_no_delay"]
    time_step = forward_data["time_step"]

    plt.figure(figsize=(15, 10))

    # Plot trajectory comparison
    plt.subplot(2, 3, 1)
    plt.plot(
        trajectory_no_delay[:, 0],
        trajectory_no_delay[:, 1],
        "b-",
        linewidth=2,
        label="Rate Control (No Delay)",
    )
    plt.plot(
        trajectory_with_delay[:, 0],
        trajectory_with_delay[:, 1],
        "r--",
        linewidth=2,
        label="Rate Control (With Delay)",
    )
    plt.plot(
        trajectory_direct_control[:, 0],
        trajectory_direct_control[:, 1],
        "g-.",
        linewidth=2,
        label="Direct Control (With Delay)",
    )
    plt.plot(
        trajectory_direct_no_delay[:, 0],
        trajectory_direct_no_delay[:, 1],
        "m:",
        linewidth=2,
        label="Direct Control (No Delay)",
    )
    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("Forward Driving: Vehicle Trajectory Comparison")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")

    # Plot velocity comparison
    plt.subplot(2, 3, 2)
    time_array = np.arange(len(trajectory_no_delay)) * time_step
    plt.plot(
        time_array,
        trajectory_no_delay[:, 3],
        "b-",
        linewidth=2,
        label="Rate Control (No Delay)",
    )
    plt.plot(
        time_array,
        trajectory_with_delay[:, 3],
        "r--",
        linewidth=2,
        label="Rate Control (With Delay)",
    )
    plt.plot(
        time_array,
        trajectory_direct_control[:, 3],
        "g-.",
        linewidth=2,
        label="Direct Control (With Delay)",
    )
    plt.plot(
        time_array,
        trajectory_direct_no_delay[:, 3],
        "m:",
        linewidth=2,
        label="Direct Control (No Delay)",
    )
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity [m/s]")
    plt.title("Forward Driving: Vehicle Velocity Comparison")
    plt.legend()
    plt.grid(True)

    # Plot yaw angle comparison
    plt.subplot(2, 3, 3)
    plt.plot(
        time_array,
        np.rad2deg(trajectory_no_delay[:, 2]),
        "b-",
        linewidth=2,
        label="Rate Control (No Delay)",
    )
    plt.plot(
        time_array,
        np.rad2deg(trajectory_with_delay[:, 2]),
        "r--",
        linewidth=2,
        label="Rate Control (With Delay)",
    )
    plt.plot(
        time_array,
        np.rad2deg(trajectory_direct_control[:, 2]),
        "g-.",
        linewidth=2,
        label="Direct Control (With Delay)",
    )
    plt.plot(
        time_array,
        np.rad2deg(trajectory_direct_no_delay[:, 2]),
        "m:",
        linewidth=2,
        label="Direct Control (No Delay)",
    )
    plt.xlabel("Time [s]")
    plt.ylabel("Yaw Angle [deg]")
    plt.title("Forward Driving: Vehicle Orientation Comparison")
    plt.legend()
    plt.grid(True)

    # Plot steering angle comparison
    plt.subplot(2, 3, 4)
    plt.plot(
        time_array,
        np.rad2deg(trajectory_no_delay[:, 4]),
        "b-",
        linewidth=2,
        label="Rate Control (No Delay)",
    )
    plt.plot(
        time_array,
        np.rad2deg(trajectory_with_delay[:, 4]),
        "r--",
        linewidth=2,
        label="Rate Control (With Delay)",
    )
    plt.plot(
        time_array,
        np.rad2deg(trajectory_direct_control[:, 4]),
        "g-.",
        linewidth=2,
        label="Direct Control (With Delay)",
    )
    plt.plot(
        time_array,
        np.rad2deg(trajectory_direct_no_delay[:, 4]),
        "m:",
        linewidth=2,
        label="Direct Control (No Delay)",
    )
    plt.xlabel("Time [s]")
    plt.ylabel("Steering Angle [deg]")
    plt.title("Forward Driving: Steering Angle Comparison")
    plt.legend()
    plt.grid(True)

    # Plot architecture info
    plt.subplot(2, 3, 5)
    plt.text(
        0.1,
        0.9,
        "Enhanced Architecture:",
        transform=plt.gca().transAxes,
        fontsize=12,
        weight="bold",
    )
    plt.text(
        0.1,
        0.8,
        "1. VehicleState:",
        transform=plt.gca().transAxes,
        fontsize=11,
        weight="bold",
    )
    plt.text(
        0.15,
        0.72,
        "• Structured state representation",
        transform=plt.gca().transAxes,
        fontsize=9,
    )
    plt.text(
        0.15,
        0.64,
        "• Type-safe with clear interface",
        transform=plt.gca().transAxes,
        fontsize=9,
    )
    plt.text(
        0.1,
        0.54,
        "2. DelayBuffer:",
        transform=plt.gca().transAxes,
        fontsize=11,
        weight="bold",
    )
    plt.text(
        0.15,
        0.46,
        "• Independent delay handling",
        transform=plt.gca().transAxes,
        fontsize=9,
    )
    plt.text(
        0.1,
        0.36,
        "3. BicycleKinematicModel:",
        transform=plt.gca().transAxes,
        fontsize=11,
        weight="bold",
    )
    plt.text(
        0.15,
        0.28,
        "• Pure kinematics with VehicleState",
        transform=plt.gca().transAxes,
        fontsize=9,
    )
    plt.text(
        0.1,
        0.18,
        "4. VehicleModel:",
        transform=plt.gca().transAxes,
        fontsize=11,
        weight="bold",
    )
    plt.text(
        0.15,
        0.1,
        "• Complete integrated system",
        transform=plt.gca().transAxes,
        fontsize=9,
    )
    plt.text(0.15, 0.02, "• Backward compatible", transform=plt.gca().transAxes, fontsize=9)
    plt.xlim(0, 1)
    plt.ylim(0, 1)
    plt.axis("off")
    plt.title("Modular Architecture")

    # Position error plot
    plt.subplot(2, 3, 6)
    position_error = np.sqrt(
        (trajectory_no_delay[:, 0] - trajectory_with_delay[:, 0]) ** 2
        + (trajectory_no_delay[:, 1] - trajectory_with_delay[:, 1]) ** 2
    )
    plt.plot(time_array, position_error, "g-", linewidth=2)
    plt.xlabel("Time [s]")
    plt.ylabel("Position Error [m]")
    plt.title("Forward Driving: Position Error due to Delays")
    plt.grid(True)

    plt.tight_layout()
    plt.suptitle("Forward Driving Test Results", fontsize=16, y=0.98)
    plt.show()


def plot_reverse_driving_results(reverse_data):
    """Plot results from reverse driving test"""
    trajectory_reverse_no_delay = reverse_data["trajectory_reverse_no_delay"]
    trajectory_reverse_with_delay = reverse_data["trajectory_reverse_with_delay"]
    trajectory_reverse_direct = reverse_data["trajectory_reverse_direct"]
    time_step = reverse_data["time_step"]

    plt.figure(figsize=(15, 10))

    # Plot reverse trajectory comparison
    plt.subplot(2, 3, 1)
    plt.plot(
        trajectory_reverse_no_delay[:, 0],
        trajectory_reverse_no_delay[:, 1],
        "b-",
        linewidth=2,
        label="Rate Control (No Delay)",
        marker="o",
        markersize=3,
        markevery=10,
    )
    plt.plot(
        trajectory_reverse_with_delay[:, 0],
        trajectory_reverse_with_delay[:, 1],
        "r--",
        linewidth=2,
        label="Rate Control (With Delay)",
        marker="s",
        markersize=3,
        markevery=10,
    )
    plt.plot(
        trajectory_reverse_direct[:, 0],
        trajectory_reverse_direct[:, 1],
        "g-.",
        linewidth=2,
        label="Direct Control (With Delay)",
        marker="^",
        markersize=3,
        markevery=10,
    )

    # Mark start and end points
    plt.plot(0, 0, "ko", markersize=8, label="Start")
    plt.plot(
        trajectory_reverse_no_delay[-1, 0],
        trajectory_reverse_no_delay[-1, 1],
        "ks",
        markersize=8,
        label="End (No Delay)",
    )

    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("Reverse Driving: Parking Maneuver Comparison")
    plt.legend()
    plt.grid(True)
    plt.axis("equal")

    # Plot reverse velocity comparison
    plt.subplot(2, 3, 2)
    time_array = np.arange(len(trajectory_reverse_no_delay)) * time_step
    plt.plot(
        time_array,
        trajectory_reverse_no_delay[:, 3],
        "b-",
        linewidth=2,
        label="Rate Control (No Delay)",
    )
    plt.plot(
        time_array,
        trajectory_reverse_with_delay[:, 3],
        "r--",
        linewidth=2,
        label="Rate Control (With Delay)",
    )
    plt.plot(
        time_array,
        trajectory_reverse_direct[:, 3],
        "g-.",
        linewidth=2,
        label="Direct Control (With Delay)",
    )
    plt.axhline(y=0, color="k", linestyle=":", alpha=0.5, label="Zero Velocity")
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity [m/s]")
    plt.title("Reverse Driving: Velocity Profile")
    plt.legend()
    plt.grid(True)

    # Plot reverse yaw angle comparison
    plt.subplot(2, 3, 3)
    plt.plot(
        time_array,
        np.rad2deg(trajectory_reverse_no_delay[:, 2]),
        "b-",
        linewidth=2,
        label="Rate Control (No Delay)",
    )
    plt.plot(
        time_array,
        np.rad2deg(trajectory_reverse_with_delay[:, 2]),
        "r--",
        linewidth=2,
        label="Rate Control (With Delay)",
    )
    plt.plot(
        time_array,
        np.rad2deg(trajectory_reverse_direct[:, 2]),
        "g-.",
        linewidth=2,
        label="Direct Control (With Delay)",
    )
    plt.xlabel("Time [s]")
    plt.ylabel("Yaw Angle [deg]")
    plt.title("Reverse Driving: Orientation Change")
    plt.legend()
    plt.grid(True)

    # Plot reverse steering angle comparison
    plt.subplot(2, 3, 4)
    plt.plot(
        time_array,
        np.rad2deg(trajectory_reverse_no_delay[:, 4]),
        "b-",
        linewidth=2,
        label="Rate Control (No Delay)",
    )
    plt.plot(
        time_array,
        np.rad2deg(trajectory_reverse_with_delay[:, 4]),
        "r--",
        linewidth=2,
        label="Rate Control (With Delay)",
    )
    plt.plot(
        time_array,
        np.rad2deg(trajectory_reverse_direct[:, 4]),
        "g-.",
        linewidth=2,
        label="Direct Control (With Delay)",
    )
    plt.xlabel("Time [s]")
    plt.ylabel("Steering Angle [deg]")
    plt.title("Reverse Driving: Steering Profile")
    plt.legend()
    plt.grid(True)

    # Plot parking phases info
    plt.subplot(2, 3, 5)
    plt.text(
        0.1,
        0.9,
        "Reverse Parking Phases:",
        transform=plt.gca().transAxes,
        fontsize=12,
        weight="bold",
    )
    plt.text(
        0.1,
        0.8,
        "Phase 1: Reverse Straight",
        transform=plt.gca().transAxes,
        fontsize=11,
        weight="bold",
    )
    plt.text(
        0.15,
        0.72,
        "• Acceleration: -2.0 m/s²",
        transform=plt.gca().transAxes,
        fontsize=9,
    )
    plt.text(
        0.15,
        0.64,
        "• Steering: 0° (straight)",
        transform=plt.gca().transAxes,
        fontsize=9,
    )
    plt.text(
        0.1,
        0.54,
        "Phase 2: Reverse + Turn Left",
        transform=plt.gca().transAxes,
        fontsize=11,
        weight="bold",
    )
    plt.text(
        0.15,
        0.46,
        "• Steering rate: 20°/s",
        transform=plt.gca().transAxes,
        fontsize=9,
    )
    plt.text(
        0.1,
        0.36,
        "Phase 3: Straighten Out",
        transform=plt.gca().transAxes,
        fontsize=11,
        weight="bold",
    )
    plt.text(
        0.15,
        0.28,
        "• Steering rate: -15°/s",
        transform=plt.gca().transAxes,
        fontsize=9,
    )
    plt.text(
        0.1,
        0.18,
        "Phase 4: Final Adjustment",
        transform=plt.gca().transAxes,
        fontsize=11,
        weight="bold",
    )
    plt.text(
        0.15,
        0.1,
        "• Turn right: -25°/s",
        transform=plt.gca().transAxes,
        fontsize=9,
    )
    plt.text(0.15, 0.02, "Phase 5: Stop", transform=plt.gca().transAxes, fontsize=11, weight="bold")
    plt.xlim(0, 1)
    plt.ylim(0, 1)
    plt.axis("off")
    plt.title("Parking Maneuver Strategy")

    # Reverse position error plot
    plt.subplot(2, 3, 6)
    position_error_reverse = np.sqrt(
        (trajectory_reverse_no_delay[:, 0] - trajectory_reverse_with_delay[:, 0]) ** 2
        + (trajectory_reverse_no_delay[:, 1] - trajectory_reverse_with_delay[:, 1]) ** 2
    )
    plt.plot(time_array, position_error_reverse, "r-", linewidth=2)
    plt.xlabel("Time [s]")
    plt.ylabel("Position Error [m]")
    plt.title("Reverse Driving: Position Error due to Delays")
    plt.grid(True)

    plt.tight_layout()
    plt.suptitle("Reverse Driving Test Results", fontsize=16, y=0.98)
    plt.show()


def plot_combined_results(combined_data):
    """Plot results from combined forward/reverse test"""
    trajectory_combined = combined_data["trajectory_combined"]
    time_step = combined_data["time_step"]

    plt.figure(figsize=(15, 8))

    # Plot combined trajectory
    plt.subplot(2, 3, 1)

    # Color code the trajectory by velocity (forward=blue, reverse=red, stop=green)
    for i in range(len(trajectory_combined) - 1):
        velocity = trajectory_combined[i, 3]
        if velocity > 0.5:
            color = "blue"
            alpha = 0.8
        elif velocity < -0.5:
            color = "red"
            alpha = 0.8
        else:
            color = "green"
            alpha = 0.6

        plt.plot(
            trajectory_combined[i : i + 2, 0], trajectory_combined[i : i + 2, 1], color=color, alpha=alpha, linewidth=2
        )

    # Mark key points
    plt.plot(trajectory_combined[0, 0], trajectory_combined[0, 1], "ko", markersize=8, label="Start")
    plt.plot(trajectory_combined[-1, 0], trajectory_combined[-1, 1], "ks", markersize=8, label="End")

    # Add custom legend
    import matplotlib.lines as mlines

    forward_line = mlines.Line2D([], [], color="blue", linewidth=2, label="Forward Motion")
    reverse_line = mlines.Line2D([], [], color="red", linewidth=2, label="Reverse Motion")
    stop_line = mlines.Line2D([], [], color="green", linewidth=2, label="Near Stop")

    plt.xlabel("X [m]")
    plt.ylabel("Y [m]")
    plt.title("Combined Forward/Reverse Maneuver")
    plt.legend(handles=[forward_line, reverse_line, stop_line])
    plt.grid(True)
    plt.axis("equal")

    # Plot velocity profile
    plt.subplot(2, 3, 2)
    time_array = np.arange(len(trajectory_combined)) * time_step
    plt.plot(time_array, trajectory_combined[:, 3], "b-", linewidth=2)
    plt.axhline(y=0, color="k", linestyle=":", alpha=0.5, label="Zero Velocity")
    plt.xlabel("Time [s]")
    plt.ylabel("Velocity [m/s]")
    plt.title("Combined: Velocity Profile")
    plt.grid(True)
    plt.legend()

    # Plot yaw angle
    plt.subplot(2, 3, 3)
    plt.plot(time_array, np.rad2deg(trajectory_combined[:, 2]), "g-", linewidth=2)
    plt.xlabel("Time [s]")
    plt.ylabel("Yaw Angle [deg]")
    plt.title("Combined: Orientation Change")
    plt.grid(True)

    # Plot steering angle
    plt.subplot(2, 3, 4)
    plt.plot(time_array, np.rad2deg(trajectory_combined[:, 4]), "m-", linewidth=2)
    plt.xlabel("Time [s]")
    plt.ylabel("Steering Angle [deg]")
    plt.title("Combined: Steering Profile")
    plt.grid(True)

    # Plot speed vs steering correlation
    plt.subplot(2, 3, 5)
    plt.scatter(
        np.abs(trajectory_combined[:, 3]),
        np.abs(np.rad2deg(trajectory_combined[:, 4])),
        c=time_array,
        cmap="viridis",
        alpha=0.6,
    )
    plt.colorbar(label="Time [s]")
    plt.xlabel("Speed [m/s]")
    plt.ylabel("Abs Steering Angle [deg]")
    plt.title("Speed vs Steering Correlation")
    plt.grid(True)

    # Maneuver phases info
    plt.subplot(2, 3, 6)
    plt.text(
        0.1,
        0.9,
        "Combined Maneuver Phases:",
        transform=plt.gca().transAxes,
        fontsize=12,
        weight="bold",
    )
    plt.text(
        0.1,
        0.8,
        "Phase 1 (0-25%): Forward + Curves",
        transform=plt.gca().transAxes,
        fontsize=10,
        weight="bold",
        color="blue",
    )
    plt.text(
        0.15,
        0.72,
        "• Sinusoidal steering pattern",
        transform=plt.gca().transAxes,
        fontsize=9,
    )
    plt.text(
        0.1,
        0.62,
        "Phase 2 (25-40%): Decelerate",
        transform=plt.gca().transAxes,
        fontsize=10,
        weight="bold",
        color="orange",
    )
    plt.text(
        0.15,
        0.54,
        "• Gradual speed reduction to stop",
        transform=plt.gca().transAxes,
        fontsize=9,
    )
    plt.text(
        0.1,
        0.44,
        "Phase 3 (40-60%): Reverse S-curve",
        transform=plt.gca().transAxes,
        fontsize=10,
        weight="bold",
        color="red",
    )
    plt.text(
        0.15,
        0.36,
        "• Complex reverse maneuver",
        transform=plt.gca().transAxes,
        fontsize=9,
    )
    plt.text(
        0.1,
        0.26,
        "Phase 4 (60-75%): Stop Again",
        transform=plt.gca().transAxes,
        fontsize=10,
        weight="bold",
        color="green",
    )
    plt.text(
        0.15,
        0.18,
        "• Transition back to forward",
        transform=plt.gca().transAxes,
        fontsize=9,
    )
    plt.text(
        0.1,
        0.08,
        "Phase 5 (75-100%): Forward Finish",
        transform=plt.gca().transAxes,
        fontsize=10,
        weight="bold",
        color="blue",
    )
    plt.xlim(0, 1)
    plt.ylim(0, 1)
    plt.axis("off")
    plt.title("Maneuver Description")

    plt.tight_layout()
    plt.suptitle("Combined Forward/Reverse Test Results", fontsize=16, y=0.98)
    plt.show()


def print_summary_results(forward_data, reverse_data, combined_data):
    """Print summary of all test results"""
    print("\n" + "=" * 60)
    print("=== VEHICLE MODEL TEST SUMMARY ===")
    print("=" * 60)

    # Forward driving summary
    trajectory_no_delay = forward_data["trajectory_no_delay"]
    trajectory_with_delay = forward_data["trajectory_with_delay"]
    position_error = np.sqrt(
        (trajectory_no_delay[-1, 0] - trajectory_with_delay[-1, 0]) ** 2
        + (trajectory_no_delay[-1, 1] - trajectory_with_delay[-1, 1]) ** 2
    )

    print("\n--- Forward Driving Test Results ---")
    print(
        f"Rate Control (No delay) - Final Position: ({trajectory_no_delay[-1, 0]:.2f}, {trajectory_no_delay[-1, 1]:.2f}) m"
    )
    print(
        f"Rate Control (With delay) - Final Position: ({trajectory_with_delay[-1, 0]:.2f}, {trajectory_with_delay[-1, 1]:.2f}) m"
    )
    print(f"Position error due to delays: {position_error:.2f} m")
    print(f"Final yaw angle difference: {np.rad2deg(trajectory_no_delay[-1, 2] - trajectory_with_delay[-1, 2]):.2f}°")

    # Reverse driving summary
    trajectory_reverse = reverse_data["trajectory_reverse_no_delay"]
    print(f"\n--- Reverse Driving Test Results ---")
    print(f"Total reverse distance: {np.sqrt(trajectory_reverse[-1, 0]**2 + trajectory_reverse[-1, 1]**2):.2f} m")
    print(f"Final reverse position: ({trajectory_reverse[-1, 0]:.2f}, {trajectory_reverse[-1, 1]:.2f}) m")
    print(f"Final reverse orientation: {np.rad2deg(trajectory_reverse[-1, 2]):.1f}°")
    print(f"Minimum velocity reached: {np.min(trajectory_reverse[:, 3]):.1f} m/s")

    # Combined maneuver summary
    trajectory_combined = combined_data["trajectory_combined"]
    forward_distance = 0
    reverse_distance = 0
    for i in range(len(trajectory_combined) - 1):
        dx = trajectory_combined[i + 1, 0] - trajectory_combined[i, 0]
        dy = trajectory_combined[i + 1, 1] - trajectory_combined[i, 1]
        distance = np.sqrt(dx**2 + dy**2)
        if trajectory_combined[i, 3] >= 0:
            forward_distance += distance
        else:
            reverse_distance += distance

    print(f"\n--- Combined Maneuver Test Results ---")
    print(f"Total forward distance: {forward_distance:.2f} m")
    print(f"Total reverse distance: {reverse_distance:.2f} m")
    print(f"Total maneuver distance: {forward_distance + reverse_distance:.2f} m")
    print(f"Final combined position: ({trajectory_combined[-1, 0]:.2f}, {trajectory_combined[-1, 1]:.2f}) m")
    print(f"Maximum forward velocity: {np.max(trajectory_combined[:, 3]):.1f} m/s")
    print(f"Maximum reverse velocity: {np.min(trajectory_combined[:, 3]):.1f} m/s")

    print(f"\n--- Vehicle Model Architecture Benefits ---")
    print("✓ VehicleState provides structured, type-safe state management")
    print("✓ DelayBuffer enables realistic actuator delay simulation")
    print("✓ BicycleKinematicModel separates pure kinematics from control")
    print("✓ VehicleModel integrates all components with dual control methods")
    print("✓ Support for both forward and reverse driving scenarios")
    print("✓ Backward compatibility with array-based interfaces")
    print("✓ Comprehensive testing framework for validation")


def main():
    """
    Main function to run all vehicle model tests with plotting
    """
    print("Starting Vehicle Model Comprehensive Testing...")
    print("This will demonstrate forward driving, reverse driving, and combined maneuvers.")

    # Run all tests
    forward_data = run_forward_driving_test()
    reverse_data = run_reverse_driving_test()
    combined_data = run_combined_forward_reverse_test()

    # Plot all results
    plot_forward_driving_results(forward_data)
    plot_reverse_driving_results(reverse_data)
    plot_combined_results(combined_data)

    # Print comprehensive summary
    print_summary_results(forward_data, reverse_data, combined_data)

    print("\n" + "=" * 60)
    print("Testing completed! Check the plots for detailed analysis.")
    print("=" * 60)


if __name__ == "__main__":
    main()
