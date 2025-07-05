"""
Path tracking simulation with iterative linear model predictive control for speed and steer control

This module implements Model Predictive Control (MPC) for autonomous vehicle path tracking.
The controller optimizes both speed and steering inputs to follow a reference trajectory.

Author: Atsushi Sakai (@Atsushi_twi)
Refactored for better readability and maintainability
"""

import math
import pathlib
import sys
import time

import cvxpy
import matplotlib.pyplot as plt
import numpy as np

sys.path.append(str(pathlib.Path(__file__).parent.parent))

from CubicSpline import cubic_spline_planner
from utils.angle import angle_mod

# State vector dimensions
STATE_DIMENSION = 4  # [x, y, velocity, yaw]
CONTROL_DIMENSION = 2  # [acceleration, steering_angle]
PREDICTION_HORIZON = 5  # Number of prediction steps

# MPC cost matrices
INPUT_COST_MATRIX = np.diag([0.01, 0.01])  # Cost for acceleration and steering
INPUT_RATE_COST_MATRIX = np.diag([0.01, 1.0])  # Cost for input changes
STATE_COST_MATRIX = np.diag([1.0, 1.0, 0.5, 0.5])  # Cost for state deviation
TERMINAL_STATE_COST_MATRIX = STATE_COST_MATRIX  # Terminal state cost

# Control parameters
GOAL_TOLERANCE_DISTANCE = 1.5  # [m] Distance threshold to consider goal reached
STOP_SPEED_THRESHOLD = 0.5 / 3.6  # [m/s] Speed threshold to consider vehicle stopped
MAX_SIMULATION_TIME = 500.0  # [s] Maximum simulation duration

# Iterative MPC parameters
MAX_ITERATIONS = 3  # Maximum iterations for iterative linearization
CONVERGENCE_THRESHOLD = 0.1  # Threshold for iteration convergence

# Vehicle target parameters
TARGET_SPEED = 10.0 / 3.6  # [m/s] Desired cruising speed
NEAREST_INDEX_SEARCH_COUNT = 10  # Number of points to search for nearest index

# Simulation parameters
TIME_STEP = 0.2  # [s] Simulation time step

# Vehicle physical parameters
VEHICLE_LENGTH = 4.5  # [m]
VEHICLE_WIDTH = 2.0  # [m]
REAR_TO_WHEEL_DISTANCE = 1.0  # [m]
WHEEL_LENGTH = 0.3  # [m]
WHEEL_WIDTH = 0.2  # [m]
WHEEL_TREAD = 0.7  # [m]
WHEELBASE = 2.5  # [m] Distance between front and rear axles

# Vehicle constraints
MAX_STEERING_ANGLE = np.deg2rad(45.0)  # [rad] Maximum steering angle
MAX_STEERING_RATE = np.deg2rad(30.0)  # [rad/s] Maximum steering rate
MAX_VELOCITY = 55.0 / 3.6  # [m/s] Maximum forward velocity
MIN_VELOCITY = -20.0 / 3.6  # [m/s] Maximum reverse velocity
MAX_ACCELERATION = 1.0  # [m/s²] Maximum acceleration/deceleration

# Visualization flag
SHOW_ANIMATION = True


class VehicleState:
    """
    Represents the current state of a vehicle in 2D space.

    Attributes:
        x (float): X coordinate position [m]
        y (float): Y coordinate position [m]
        yaw (float): Vehicle heading angle [rad]
        velocity (float): Vehicle speed [m/s]
        previous_steering_angle (float): Previous steering input for continuity
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, velocity=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.velocity = velocity
        self.previous_steering_angle = None


def normalize_angle(angle):
    """Normalize angle to [-pi, pi] range."""
    return angle_mod(angle)


def get_linearized_model_matrices(velocity, yaw_angle, steering_angle):
    """
    Calculate linearized discrete-time state space model matrices.

    The vehicle model is: x_{k+1} = A*x_k + B*u_k + C
    where x = [x, y, velocity, yaw] and u = [acceleration, steering_angle]

    Args:
        velocity (float): Current vehicle velocity [m/s]
        yaw_angle (float): Current vehicle yaw angle [rad]
        steering_angle (float): Current steering angle [rad]

    Returns:
        tuple: (A_matrix, B_matrix, C_vector) - Linearized model matrices
    """
    # State transition matrix A
    A_matrix = np.zeros((STATE_DIMENSION, STATE_DIMENSION))
    A_matrix[0, 0] = 1.0  # x position
    A_matrix[1, 1] = 1.0  # y position
    A_matrix[2, 2] = 1.0  # velocity
    A_matrix[3, 3] = 1.0  # yaw angle

    # Position derivatives based on current state
    A_matrix[0, 2] = TIME_STEP * math.cos(yaw_angle)  # dx/dv
    A_matrix[0, 3] = -TIME_STEP * velocity * math.sin(yaw_angle)  # dx/dyaw
    A_matrix[1, 2] = TIME_STEP * math.sin(yaw_angle)  # dy/dv
    A_matrix[1, 3] = TIME_STEP * velocity * math.cos(yaw_angle)  # dy/dyaw
    A_matrix[3, 2] = TIME_STEP * math.tan(steering_angle) / WHEELBASE  # dyaw/dv

    # Control input matrix B
    B_matrix = np.zeros((STATE_DIMENSION, CONTROL_DIMENSION))
    B_matrix[2, 0] = TIME_STEP  # velocity response to acceleration
    B_matrix[3, 1] = TIME_STEP * velocity / (WHEELBASE * math.cos(steering_angle) ** 2)  # yaw response to steering

    # Constant offset vector C (linearization offset)
    C_vector = np.zeros(STATE_DIMENSION)
    C_vector[0] = TIME_STEP * velocity * math.sin(yaw_angle) * yaw_angle
    C_vector[1] = -TIME_STEP * velocity * math.cos(yaw_angle) * yaw_angle
    C_vector[3] = -TIME_STEP * velocity * steering_angle / (WHEELBASE * math.cos(steering_angle) ** 2)

    return A_matrix, B_matrix, C_vector


def plot_vehicle(x, y, yaw, steering_angle=0.0, body_color="-r", wheel_color="-k"):
    """
    Plot vehicle shape with wheels at given position and orientation.

    Args:
        x (float): Vehicle x position [m]
        y (float): Vehicle y position [m]
        yaw (float): Vehicle yaw angle [rad]
        steering_angle (float): Front wheel steering angle [rad]
        body_color (str): Color specification for vehicle body
        wheel_color (str): Color specification for wheels
    """
    # Vehicle body outline
    vehicle_outline = np.array(
        [
            [
                -REAR_TO_WHEEL_DISTANCE,
                (VEHICLE_LENGTH - REAR_TO_WHEEL_DISTANCE),
                (VEHICLE_LENGTH - REAR_TO_WHEEL_DISTANCE),
                -REAR_TO_WHEEL_DISTANCE,
                -REAR_TO_WHEEL_DISTANCE,
            ],
            [
                VEHICLE_WIDTH / 2,
                VEHICLE_WIDTH / 2,
                -VEHICLE_WIDTH / 2,
                -VEHICLE_WIDTH / 2,
                VEHICLE_WIDTH / 2,
            ],
        ]
    )

    # Front right wheel
    front_right_wheel = np.array(
        [
            [WHEEL_LENGTH, -WHEEL_LENGTH, -WHEEL_LENGTH, WHEEL_LENGTH, WHEEL_LENGTH],
            [
                -WHEEL_WIDTH - WHEEL_TREAD,
                -WHEEL_WIDTH - WHEEL_TREAD,
                WHEEL_WIDTH - WHEEL_TREAD,
                WHEEL_WIDTH - WHEEL_TREAD,
                -WHEEL_WIDTH - WHEEL_TREAD,
            ],
        ]
    )

    # Copy wheel shapes for other wheels
    rear_right_wheel = np.copy(front_right_wheel)
    front_left_wheel = np.copy(front_right_wheel)
    front_left_wheel[1, :] *= -1  # Mirror for left side
    rear_left_wheel = np.copy(rear_right_wheel)
    rear_left_wheel[1, :] *= -1  # Mirror for left side

    # Rotation matrices
    vehicle_rotation = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
    steering_rotation = np.array(
        [
            [math.cos(steering_angle), math.sin(steering_angle)],
            [-math.sin(steering_angle), math.cos(steering_angle)],
        ]
    )

    # Apply steering rotation to front wheels
    front_right_wheel = (front_right_wheel.T.dot(steering_rotation)).T
    front_left_wheel = (front_left_wheel.T.dot(steering_rotation)).T

    # Move front wheels to wheelbase position
    front_right_wheel[0, :] += WHEELBASE
    front_left_wheel[0, :] += WHEELBASE

    # Apply vehicle rotation to all components
    front_right_wheel = (front_right_wheel.T.dot(vehicle_rotation)).T
    front_left_wheel = (front_left_wheel.T.dot(vehicle_rotation)).T
    vehicle_outline = (vehicle_outline.T.dot(vehicle_rotation)).T
    rear_right_wheel = (rear_right_wheel.T.dot(vehicle_rotation)).T
    rear_left_wheel = (rear_left_wheel.T.dot(vehicle_rotation)).T

    # Translate to vehicle position
    for component in [
        vehicle_outline,
        front_right_wheel,
        front_left_wheel,
        rear_right_wheel,
        rear_left_wheel,
    ]:
        component[0, :] += x
        component[1, :] += y

    # Plot all components
    plt.plot(
        np.array(vehicle_outline[0, :]).flatten(),
        np.array(vehicle_outline[1, :]).flatten(),
        body_color,
    )
    plt.plot(
        np.array(front_right_wheel[0, :]).flatten(),
        np.array(front_right_wheel[1, :]).flatten(),
        wheel_color,
    )
    plt.plot(
        np.array(rear_right_wheel[0, :]).flatten(),
        np.array(rear_right_wheel[1, :]).flatten(),
        wheel_color,
    )
    plt.plot(
        np.array(front_left_wheel[0, :]).flatten(),
        np.array(front_left_wheel[1, :]).flatten(),
        wheel_color,
    )
    plt.plot(
        np.array(rear_left_wheel[0, :]).flatten(),
        np.array(rear_left_wheel[1, :]).flatten(),
        wheel_color,
    )
    plt.plot(x, y, "*")


def update_vehicle_state(state, acceleration, steering_angle):
    """
    Update vehicle state using bicycle model dynamics.

    Args:
        state (VehicleState): Current vehicle state
        acceleration (float): Control acceleration input [m/s²]
        steering_angle (float): Control steering input [rad]

    Returns:
        VehicleState: Updated vehicle state
    """
    # Apply steering angle constraints
    steering_angle = max(-MAX_STEERING_ANGLE, min(MAX_STEERING_ANGLE, steering_angle))

    # Update state using bicycle model
    state.x += state.velocity * math.cos(state.yaw) * TIME_STEP
    state.y += state.velocity * math.sin(state.yaw) * TIME_STEP
    state.yaw += state.velocity / WHEELBASE * math.tan(steering_angle) * TIME_STEP
    state.velocity += acceleration * TIME_STEP

    # Apply velocity constraints
    state.velocity = max(MIN_VELOCITY, min(MAX_VELOCITY, state.velocity))

    return state


def convert_matrix_to_array(matrix):
    """Convert cvxpy matrix to numpy array."""
    return np.array(matrix).flatten()


def find_nearest_trajectory_index(current_state, trajectory_x, trajectory_y, trajectory_yaw, previous_index):
    """
    Find the nearest point on the reference trajectory to the current vehicle position.

    Args:
        current_state (VehicleState): Current vehicle state
        trajectory_x (list): Reference trajectory x coordinates
        trajectory_y (list): Reference trajectory y coordinates
        trajectory_yaw (list): Reference trajectory yaw angles
        previous_index (int): Previous nearest index for efficient search

    Returns:
        tuple: (nearest_index, cross_track_error)
    """
    # Search in a local window around previous index
    search_start = previous_index
    search_end = min(previous_index + NEAREST_INDEX_SEARCH_COUNT, len(trajectory_x))

    # Calculate distances to trajectory points
    distances_x = [current_state.x - trajectory_x[i] for i in range(search_start, search_end)]
    distances_y = [current_state.y - trajectory_y[i] for i in range(search_start, search_end)]
    squared_distances = [dx**2 + dy**2 for dx, dy in zip(distances_x, distances_y)]

    # Find minimum distance and corresponding index
    min_distance = min(squared_distances)
    nearest_index = squared_distances.index(min_distance) + search_start
    min_distance = math.sqrt(min_distance)

    # Calculate cross-track error (positive for left deviation, negative for right)
    dx_to_path = trajectory_x[nearest_index] - current_state.x
    dy_to_path = trajectory_y[nearest_index] - current_state.y
    angle_to_path = normalize_angle(trajectory_yaw[nearest_index] - math.atan2(dy_to_path, dx_to_path))

    cross_track_error = min_distance if angle_to_path < 0 else -min_distance

    return nearest_index, cross_track_error


def predict_vehicle_motion(initial_state, acceleration_sequence, steering_sequence, reference_trajectory):
    """
    Predict vehicle motion over the prediction horizon.

    Args:
        initial_state (list): Initial state [x, y, velocity, yaw]
        acceleration_sequence (list): Sequence of acceleration inputs
        steering_sequence (list): Sequence of steering inputs
        reference_trajectory (np.array): Reference trajectory for initialization

    Returns:
        np.array: Predicted state trajectory
    """
    predicted_states = reference_trajectory * 0.0  # Initialize with correct shape

    # Set initial state
    for i in range(len(initial_state)):
        predicted_states[i, 0] = initial_state[i]

    # Simulate forward
    current_state = VehicleState(
        x=initial_state[0],
        y=initial_state[1],
        yaw=initial_state[3],
        velocity=initial_state[2],
    )

    for step, (accel, steer) in enumerate(zip(acceleration_sequence, steering_sequence), 1):
        current_state = update_vehicle_state(current_state, accel, steer)
        predicted_states[0, step] = current_state.x
        predicted_states[1, step] = current_state.y
        predicted_states[2, step] = current_state.velocity
        predicted_states[3, step] = current_state.yaw

    return predicted_states


def iterative_linear_mpc_control(
    reference_trajectory,
    initial_state,
    reference_steering,
    previous_acceleration,
    previous_steering,
):
    """
    Iterative linear MPC controller that updates linearization point.

    Args:
        reference_trajectory (np.array): Reference state trajectory
        initial_state (list): Current vehicle state
        reference_steering (np.array): Reference steering sequence
        previous_acceleration (list): Previous acceleration sequence
        previous_steering (list): Previous steering sequence

    Returns:
        tuple: (acceleration_sequence, steering_sequence, predicted_x, predicted_y,
                predicted_yaw, predicted_velocity)
    """
    predicted_x = predicted_y = predicted_yaw = predicted_velocity = None

    # Initialize control sequences if not provided
    if previous_acceleration is None or previous_steering is None:
        previous_acceleration = [0.0] * PREDICTION_HORIZON
        previous_steering = [0.0] * PREDICTION_HORIZON

    # Iterative linearization
    for iteration in range(MAX_ITERATIONS):
        # Predict motion with current control sequence
        predicted_trajectory = predict_vehicle_motion(
            initial_state,
            previous_acceleration,
            previous_steering,
            reference_trajectory,
        )

        # Store previous control for convergence check
        old_acceleration = previous_acceleration[:]
        old_steering = previous_steering[:]

        # Solve linear MPC with updated linearization point
        (
            previous_acceleration,
            previous_steering,
            predicted_x,
            predicted_y,
            predicted_yaw,
            predicted_velocity,
        ) = solve_linear_mpc(
            reference_trajectory,
            predicted_trajectory,
            initial_state,
            reference_steering,
        )

        # Check convergence
        control_change = sum(abs(np.array(previous_acceleration) - np.array(old_acceleration))) + sum(
            abs(np.array(previous_steering) - np.array(old_steering))
        )

        if control_change <= CONVERGENCE_THRESHOLD:
            break
    else:
        print("Warning: MPC iterations reached maximum without convergence")

    return (
        previous_acceleration,
        previous_steering,
        predicted_x,
        predicted_y,
        predicted_yaw,
        predicted_velocity,
    )


def solve_linear_mpc(reference_trajectory, linearization_trajectory, initial_state, reference_steering):
    """
    Solve linear MPC optimization problem.

    Args:
        reference_trajectory (np.array): Reference state trajectory to track
        linearization_trajectory (np.array): Trajectory around which to linearize
        initial_state (list): Initial state constraint
        reference_steering (np.array): Reference steering sequence

    Returns:
        tuple: (acceleration_sequence, steering_sequence, predicted_x, predicted_y,
                predicted_yaw, predicted_velocity)
    """
    # Decision variables
    state_variables = cvxpy.Variable((STATE_DIMENSION, PREDICTION_HORIZON + 1))
    control_variables = cvxpy.Variable((CONTROL_DIMENSION, PREDICTION_HORIZON))

    # Initialize cost and constraints
    total_cost = 0.0
    constraints = []

    # Build cost function and constraints over prediction horizon
    for time_step in range(PREDICTION_HORIZON):
        # Control input cost
        total_cost += cvxpy.quad_form(control_variables[:, time_step], INPUT_COST_MATRIX)

        # State tracking cost (skip first step)
        if time_step != 0:
            state_error = reference_trajectory[:, time_step] - state_variables[:, time_step]
            total_cost += cvxpy.quad_form(state_error, STATE_COST_MATRIX)

        # Get linearized model matrices
        velocity = linearization_trajectory[2, time_step]
        yaw_angle = linearization_trajectory[3, time_step]
        steering_angle = reference_steering[0, time_step]
        A_matrix, B_matrix, C_vector = get_linearized_model_matrices(velocity, yaw_angle, steering_angle)

        # System dynamics constraint
        next_state = A_matrix @ state_variables[:, time_step] + B_matrix @ control_variables[:, time_step] + C_vector
        constraints += [state_variables[:, time_step + 1] == next_state]

        # Control rate constraints
        if time_step < (PREDICTION_HORIZON - 1):
            control_rate = control_variables[:, time_step + 1] - control_variables[:, time_step]
            total_cost += cvxpy.quad_form(control_rate, INPUT_RATE_COST_MATRIX)

            # Steering rate limit
            steering_rate_limit = MAX_STEERING_RATE * TIME_STEP
            constraints += [
                cvxpy.abs(control_variables[1, time_step + 1] - control_variables[1, time_step]) <= steering_rate_limit
            ]

    # Terminal cost
    terminal_state_error = reference_trajectory[:, PREDICTION_HORIZON] - state_variables[:, PREDICTION_HORIZON]
    total_cost += cvxpy.quad_form(terminal_state_error, TERMINAL_STATE_COST_MATRIX)

    # State and control constraints
    constraints += [state_variables[:, 0] == initial_state]  # Initial state
    constraints += [state_variables[2, :] <= MAX_VELOCITY]  # Maximum velocity
    constraints += [state_variables[2, :] >= MIN_VELOCITY]  # Minimum velocity
    constraints += [cvxpy.abs(control_variables[0, :]) <= MAX_ACCELERATION]  # Acceleration limits
    constraints += [cvxpy.abs(control_variables[1, :]) <= MAX_STEERING_ANGLE]  # Steering limits

    # Solve optimization problem
    optimization_problem = cvxpy.Problem(cvxpy.Minimize(total_cost), constraints)
    optimization_problem.solve(solver=cvxpy.CLARABEL, verbose=False)

    # Extract solution
    if optimization_problem.status in [cvxpy.OPTIMAL, cvxpy.OPTIMAL_INACCURATE]:
        predicted_x = convert_matrix_to_array(state_variables.value[0, :])
        predicted_y = convert_matrix_to_array(state_variables.value[1, :])
        predicted_velocity = convert_matrix_to_array(state_variables.value[2, :])
        predicted_yaw = convert_matrix_to_array(state_variables.value[3, :])
        acceleration_sequence = convert_matrix_to_array(control_variables.value[0, :])
        steering_sequence = convert_matrix_to_array(control_variables.value[1, :])
    else:
        print("Error: Cannot solve MPC optimization problem")
        acceleration_sequence = steering_sequence = None
        predicted_x = predicted_y = predicted_yaw = predicted_velocity = None

    return (
        acceleration_sequence,
        steering_sequence,
        predicted_x,
        predicted_y,
        predicted_yaw,
        predicted_velocity,
    )


def calculate_reference_trajectory(
    current_state,
    trajectory_x,
    trajectory_y,
    trajectory_yaw,
    trajectory_curvature,
    speed_profile,
    path_resolution,
    previous_target_index,
):
    """
    Calculate reference trajectory for MPC horizon.

    Args:
        current_state (VehicleState): Current vehicle state
        trajectory_x (list): Reference path x coordinates
        trajectory_y (list): Reference path y coordinates
        trajectory_yaw (list): Reference path yaw angles
        trajectory_curvature (list): Reference path curvature
        speed_profile (list): Desired speed along path
        path_resolution (float): Distance resolution of path
        previous_target_index (int): Previous target index

    Returns:
        tuple: (reference_trajectory, target_index, reference_steering)
    """
    reference_trajectory = np.zeros((STATE_DIMENSION, PREDICTION_HORIZON + 1))
    reference_steering = np.zeros((1, PREDICTION_HORIZON + 1))
    total_points = len(trajectory_x)

    # Find nearest point on trajectory
    nearest_index, _ = find_nearest_trajectory_index(
        current_state, trajectory_x, trajectory_y, trajectory_yaw, previous_target_index
    )

    # Ensure we don't go backwards
    target_index = max(nearest_index, previous_target_index)

    # Set initial reference point
    reference_trajectory[0, 0] = trajectory_x[target_index]
    reference_trajectory[1, 0] = trajectory_y[target_index]
    reference_trajectory[2, 0] = speed_profile[target_index]
    reference_trajectory[3, 0] = trajectory_yaw[target_index]
    reference_steering[0, 0] = 0.0  # Steering reference is typically zero

    # Calculate reference points for prediction horizon
    traveled_distance = 0.0
    for step in range(PREDICTION_HORIZON + 1):
        traveled_distance += abs(current_state.velocity) * TIME_STEP
        distance_index_offset = int(round(traveled_distance / path_resolution))

        future_index = target_index + distance_index_offset

        if future_index < total_points:
            reference_trajectory[0, step] = trajectory_x[future_index]
            reference_trajectory[1, step] = trajectory_y[future_index]
            reference_trajectory[2, step] = speed_profile[future_index]
            reference_trajectory[3, step] = trajectory_yaw[future_index]
            reference_steering[0, step] = 0.0
        else:
            # Use last point if we've reached the end
            reference_trajectory[0, step] = trajectory_x[total_points - 1]
            reference_trajectory[1, step] = trajectory_y[total_points - 1]
            reference_trajectory[2, step] = speed_profile[total_points - 1]
            reference_trajectory[3, step] = trajectory_yaw[total_points - 1]
            reference_steering[0, step] = 0.0

    return reference_trajectory, target_index, reference_steering


def check_goal_reached(current_state, goal_position, target_index, trajectory_length):
    """
    Check if vehicle has reached the goal.

    Args:
        current_state (VehicleState): Current vehicle state
        goal_position (list): Goal position [x, y]
        target_index (int): Current target index on trajectory
        trajectory_length (int): Total length of trajectory

    Returns:
        bool: True if goal is reached
    """
    # Calculate distance to goal
    distance_to_goal = math.hypot(current_state.x - goal_position[0], current_state.y - goal_position[1])

    # Check if close to goal
    is_near_goal = distance_to_goal <= GOAL_TOLERANCE_DISTANCE

    # Check if we're near the end of the trajectory
    is_near_end = abs(target_index - trajectory_length) < 5

    # Check if vehicle has stopped
    is_stopped = abs(current_state.velocity) <= STOP_SPEED_THRESHOLD

    return is_near_goal and is_near_end and is_stopped


def run_mpc_simulation(
    trajectory_x,
    trajectory_y,
    trajectory_yaw,
    trajectory_curvature,
    speed_profile,
    path_resolution,
    initial_state,
):
    """
    Main simulation loop for MPC path tracking.

    Args:
        trajectory_x (list): Reference trajectory x coordinates
        trajectory_y (list): Reference trajectory y coordinates
        trajectory_yaw (list): Reference trajectory yaw angles
        trajectory_curvature (list): Reference trajectory curvature
        speed_profile (list): Desired speed profile
        path_resolution (float): Path resolution [m]
        initial_state (VehicleState): Initial vehicle state

    Returns:
        tuple: (time_history, x_history, y_history, yaw_history, velocity_history,
                steering_history, acceleration_history)
    """
    goal_position = [trajectory_x[-1], trajectory_y[-1]]
    current_state = initial_state

    # Compensate initial yaw angle
    initial_yaw_error = current_state.yaw - trajectory_yaw[0]
    if initial_yaw_error >= math.pi:
        current_state.yaw -= 2.0 * math.pi
    elif initial_yaw_error <= -math.pi:
        current_state.yaw += 2.0 * math.pi

    # Initialize simulation history
    simulation_time = 0.0
    time_history = [0.0]
    x_history = [current_state.x]
    y_history = [current_state.y]
    yaw_history = [current_state.yaw]
    velocity_history = [current_state.velocity]
    steering_history = [0.0]
    acceleration_history = [0.0]

    # Find initial target index
    target_index, _ = find_nearest_trajectory_index(current_state, trajectory_x, trajectory_y, trajectory_yaw, 0)

    # Initialize control sequences
    previous_steering_sequence = None
    previous_acceleration_sequence = None

    # Smooth yaw angles to avoid discontinuities
    trajectory_yaw = smooth_trajectory_yaw(trajectory_yaw)

    # Main simulation loop
    while simulation_time <= MAX_SIMULATION_TIME:
        # Calculate reference trajectory for current horizon
        reference_trajectory, target_index, reference_steering = calculate_reference_trajectory(
            current_state,
            trajectory_x,
            trajectory_y,
            trajectory_yaw,
            trajectory_curvature,
            speed_profile,
            path_resolution,
            target_index,
        )

        # Current state vector
        current_state_vector = [
            current_state.x,
            current_state.y,
            current_state.velocity,
            current_state.yaw,
        ]

        # Solve MPC optimization
        (
            acceleration_sequence,
            steering_sequence,
            predicted_x,
            predicted_y,
            predicted_yaw,
            predicted_velocity,
        ) = iterative_linear_mpc_control(
            reference_trajectory,
            current_state_vector,
            reference_steering,
            previous_acceleration_sequence,
            previous_steering_sequence,
        )

        # Apply first control input
        applied_acceleration = 0.0
        applied_steering = 0.0
        if steering_sequence is not None:
            applied_steering = steering_sequence[0]
            applied_acceleration = acceleration_sequence[0]
            current_state = update_vehicle_state(current_state, applied_acceleration, applied_steering)

        # Update time and history
        simulation_time += TIME_STEP
        time_history.append(simulation_time)
        x_history.append(current_state.x)
        y_history.append(current_state.y)
        yaw_history.append(current_state.yaw)
        velocity_history.append(current_state.velocity)
        steering_history.append(applied_steering)
        acceleration_history.append(applied_acceleration)

        # Check if goal is reached
        if check_goal_reached(current_state, goal_position, target_index, len(trajectory_x)):
            print("Goal reached successfully!")
            break

        # Visualization
        if SHOW_ANIMATION:
            plt.cla()
            plt.gcf().canvas.mpl_connect(
                "key_release_event",
                lambda event: [exit(0) if event.key == "escape" else None],
            )

            if predicted_x is not None:
                plt.plot(predicted_x, predicted_y, "xr", label="MPC prediction")
            plt.plot(trajectory_x, trajectory_y, "-r", label="Reference path")
            plt.plot(x_history, y_history, "ob", label="Vehicle trajectory")
            plt.plot(
                reference_trajectory[0, :],
                reference_trajectory[1, :],
                "xk",
                label="Reference",
            )
            plt.plot(
                trajectory_x[target_index],
                trajectory_y[target_index],
                "xg",
                label="Target",
            )

            plot_vehicle(
                current_state.x,
                current_state.y,
                current_state.yaw,
                steering_angle=applied_steering,
            )

            plt.axis("equal")
            plt.grid(True)
            plt.title(f"Time: {simulation_time:.2f}s, Speed: {current_state.velocity * 3.6:.2f} km/h")
            plt.pause(0.0001)

        # Store control sequences for next iteration
        previous_acceleration_sequence = acceleration_sequence
        previous_steering_sequence = steering_sequence

    return (
        time_history,
        x_history,
        y_history,
        yaw_history,
        velocity_history,
        steering_history,
        acceleration_history,
    )


def calculate_speed_profile(trajectory_x, trajectory_y, trajectory_yaw, target_speed):
    """
    Calculate speed profile considering direction changes.

    Args:
        trajectory_x (list): Trajectory x coordinates
        trajectory_y (list): Trajectory y coordinates
        trajectory_yaw (list): Trajectory yaw angles
        target_speed (float): Desired forward speed

    Returns:
        list: Speed profile along trajectory
    """
    speed_profile = [target_speed] * len(trajectory_x)
    current_direction = 1.0  # 1.0 for forward, -1.0 for reverse

    # Determine direction at each point
    for i in range(len(trajectory_x) - 1):
        delta_x = trajectory_x[i + 1] - trajectory_x[i]
        delta_y = trajectory_y[i + 1] - trajectory_y[i]

        if delta_x != 0.0 or delta_y != 0.0:
            movement_direction = math.atan2(delta_y, delta_x)
            direction_error = abs(normalize_angle(movement_direction - trajectory_yaw[i]))

            # Large direction error indicates reverse motion
            if direction_error >= math.pi / 4.0:
                current_direction = -1.0
            else:
                current_direction = 1.0

        speed_profile[i] = current_direction * target_speed

    # Stop at the end
    speed_profile[-1] = 0.0
    return speed_profile


def smooth_trajectory_yaw(yaw_angles):
    """
    Smooth yaw angle sequence to avoid large jumps.

    Args:
        yaw_angles (list): Raw yaw angles

    Returns:
        list: Smoothed yaw angles
    """
    smoothed_yaw = yaw_angles[:]

    for i in range(len(smoothed_yaw) - 1):
        yaw_difference = smoothed_yaw[i + 1] - smoothed_yaw[i]

        # Adjust for angle wrapping
        while yaw_difference >= math.pi / 2.0:
            smoothed_yaw[i + 1] -= 2.0 * math.pi
            yaw_difference = smoothed_yaw[i + 1] - smoothed_yaw[i]

        while yaw_difference <= -math.pi / 2.0:
            smoothed_yaw[i + 1] += 2.0 * math.pi
            yaw_difference = smoothed_yaw[i + 1] - smoothed_yaw[i]

    return smoothed_yaw


def create_straight_path(path_resolution):
    """Create a simple straight line trajectory."""
    waypoints_x = [0.0, 5.0, 10.0, 20.0, 30.0, 40.0, 50.0]
    waypoints_y = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

    trajectory_x, trajectory_y, trajectory_yaw, trajectory_curvature, _ = cubic_spline_planner.calc_spline_course(
        waypoints_x, waypoints_y, ds=path_resolution
    )

    return trajectory_x, trajectory_y, trajectory_yaw, trajectory_curvature


def create_curved_path(path_resolution):
    """Create a curved trajectory."""
    waypoints_x = [0.0, -10.0, -20.0, -40.0, -50.0, -60.0, -70.0]
    waypoints_y = [0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0]

    trajectory_x, trajectory_y, trajectory_yaw, trajectory_curvature, _ = cubic_spline_planner.calc_spline_course(
        waypoints_x, waypoints_y, ds=path_resolution
    )

    return trajectory_x, trajectory_y, trajectory_yaw, trajectory_curvature


def create_reverse_path(path_resolution):
    """Create a path requiring reverse motion."""
    waypoints_x = [0.0, -10.0, -20.0, -40.0, -50.0, -60.0, -70.0]
    waypoints_y = [0.0, -1.0, 1.0, 0.0, -1.0, 1.0, 0.0]

    trajectory_x, trajectory_y, trajectory_yaw, trajectory_curvature, _ = cubic_spline_planner.calc_spline_course(
        waypoints_x, waypoints_y, ds=path_resolution
    )

    # Adjust yaw for reverse motion
    trajectory_yaw = [angle - math.pi for angle in trajectory_yaw]

    return trajectory_x, trajectory_y, trajectory_yaw, trajectory_curvature


def create_complex_path(path_resolution):
    """Create a complex trajectory with curves."""
    waypoints_x = [0.0, 60.0, 125.0, 50.0, 75.0, 30.0, -10.0]
    waypoints_y = [0.0, 0.0, 50.0, 65.0, 30.0, 50.0, -20.0]

    trajectory_x, trajectory_y, trajectory_yaw, trajectory_curvature, _ = cubic_spline_planner.calc_spline_course(
        waypoints_x, waypoints_y, ds=path_resolution
    )

    return trajectory_x, trajectory_y, trajectory_yaw, trajectory_curvature


def create_parking_path(path_resolution):
    """Create a parking maneuver path with forward and reverse sections."""
    # Forward section
    forward_waypoints_x = [0.0, 30.0, 6.0, 20.0, 35.0]
    forward_waypoints_y = [0.0, 0.0, 20.0, 35.0, 20.0]

    forward_x, forward_y, forward_yaw, forward_curvature, _ = cubic_spline_planner.calc_spline_course(
        forward_waypoints_x, forward_waypoints_y, ds=path_resolution
    )

    # Reverse section
    reverse_waypoints_x = [35.0, 10.0, 0.0, 0.0]
    reverse_waypoints_y = [20.0, 30.0, 5.0, 0.0]

    reverse_x, reverse_y, reverse_yaw, reverse_curvature, _ = cubic_spline_planner.calc_spline_course(
        reverse_waypoints_x, reverse_waypoints_y, ds=path_resolution
    )

    # Adjust reverse section yaw angles
    reverse_yaw = [angle - math.pi for angle in reverse_yaw]

    # Combine trajectories
    trajectory_x = forward_x + reverse_x
    trajectory_y = forward_y + reverse_y
    trajectory_yaw = forward_yaw + reverse_yaw
    trajectory_curvature = forward_curvature + reverse_curvature

    return trajectory_x, trajectory_y, trajectory_yaw, trajectory_curvature


def main():
    """Main function demonstrating MPC path tracking."""
    print(f"{__file__} starting MPC simulation...")
    start_time = time.time()

    # Simulation parameters
    path_resolution = 1.0  # [m] Distance between path points

    # Select trajectory type (uncomment desired trajectory)
    # trajectory_x, trajectory_y, trajectory_yaw, trajectory_curvature = create_straight_path(path_resolution)
    # trajectory_x, trajectory_y, trajectory_yaw, trajectory_curvature = create_curved_path(path_resolution)
    # trajectory_x, trajectory_y, trajectory_yaw, trajectory_curvature = create_reverse_path(path_resolution)
    # trajectory_x, trajectory_y, trajectory_yaw, trajectory_curvature = create_complex_path(path_resolution)
    trajectory_x, trajectory_y, trajectory_yaw, trajectory_curvature = create_parking_path(path_resolution)

    # Calculate speed profile
    speed_profile = calculate_speed_profile(trajectory_x, trajectory_y, trajectory_yaw, TARGET_SPEED)

    # Set initial vehicle state
    initial_state = VehicleState(x=trajectory_x[0], y=trajectory_y[0], yaw=trajectory_yaw[0], velocity=0.0)

    # Run simulation
    (
        time_history,
        x_history,
        y_history,
        yaw_history,
        velocity_history,
        steering_history,
        acceleration_history,
    ) = run_mpc_simulation(
        trajectory_x,
        trajectory_y,
        trajectory_yaw,
        trajectory_curvature,
        speed_profile,
        path_resolution,
        initial_state,
    )

    # Print timing results
    elapsed_time = time.time() - start_time
    print(f"Simulation completed in {elapsed_time:.6f} seconds")

    # Plot results
    if SHOW_ANIMATION:
        plt.close("all")

        # Plot trajectory comparison
        plt.figure(figsize=(12, 8))
        plt.subplot(2, 2, 1)
        plt.plot(trajectory_x, trajectory_y, "-r", label="Reference path", linewidth=2)
        plt.plot(x_history, y_history, "-g", label="Vehicle trajectory", linewidth=2)
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("X position [m]")
        plt.ylabel("Y position [m]")
        plt.legend()
        plt.title("Path Tracking Performance")

        # Plot speed profile
        plt.subplot(2, 2, 2)
        plt.plot(
            time_history,
            [v * 3.6 for v in velocity_history],
            "-r",
            label="Vehicle speed",
            linewidth=2,
        )
        plt.grid(True)
        plt.xlabel("Time [s]")
        plt.ylabel("Speed [km/h]")
        plt.legend()
        plt.title("Speed Profile")

        # Plot steering angle
        plt.subplot(2, 2, 3)
        plt.plot(
            time_history[1:],
            [math.degrees(s) for s in steering_history[1:]],
            "-b",
            label="Steering angle",
            linewidth=2,
        )
        plt.grid(True)
        plt.xlabel("Time [s]")
        plt.ylabel("Steering angle [deg]")
        plt.legend()
        plt.title("Steering Control")

        # Plot acceleration
        plt.subplot(2, 2, 4)
        plt.plot(
            time_history[1:],
            acceleration_history[1:],
            "-m",
            label="Acceleration",
            linewidth=2,
        )
        plt.grid(True)
        plt.xlabel("Time [s]")
        plt.ylabel("Acceleration [m/s²]")
        plt.legend()
        plt.title("Acceleration Control")

        plt.tight_layout()
        plt.show()


def main_reverse_test():
    """Test function for reverse motion."""
    print(f"{__file__} starting reverse motion test...")
    start_time = time.time()

    path_resolution = 1.0
    trajectory_x, trajectory_y, trajectory_yaw, trajectory_curvature = create_reverse_path(path_resolution)
    speed_profile = calculate_speed_profile(trajectory_x, trajectory_y, trajectory_yaw, TARGET_SPEED)

    # Start with zero yaw to test yaw compensation
    initial_state = VehicleState(x=trajectory_x[0], y=trajectory_y[0], yaw=0.0, velocity=0.0)

    (
        time_history,
        x_history,
        y_history,
        yaw_history,
        velocity_history,
        steering_history,
        acceleration_history,
    ) = run_mpc_simulation(
        trajectory_x,
        trajectory_y,
        trajectory_yaw,
        trajectory_curvature,
        speed_profile,
        path_resolution,
        initial_state,
    )

    elapsed_time = time.time() - start_time
    print(f"Reverse test completed in {elapsed_time:.6f} seconds")

    if SHOW_ANIMATION:
        plt.close("all")
        plt.figure(figsize=(10, 6))
        plt.subplot(1, 2, 1)
        plt.plot(trajectory_x, trajectory_y, "-r", label="Reference path", linewidth=2)
        plt.plot(x_history, y_history, "-g", label="Vehicle trajectory", linewidth=2)
        plt.grid(True)
        plt.axis("equal")
        plt.xlabel("X position [m]")
        plt.ylabel("Y position [m]")
        plt.legend()
        plt.title("Reverse Motion Test")

        plt.subplot(1, 2, 2)
        plt.plot(time_history, [v * 3.6 for v in velocity_history], "-r", linewidth=2)
        plt.grid(True)
        plt.xlabel("Time [s]")
        plt.ylabel("Speed [km/h]")
        plt.title("Speed Profile (Reverse)")
        plt.show()


if __name__ == "__main__":
    main()
    # main_reverse_test()
