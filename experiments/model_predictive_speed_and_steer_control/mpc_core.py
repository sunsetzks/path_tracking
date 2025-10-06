"""
Core MPC class for path tracking control.

This module contains the essential MPC optimization class extracted from the main simulation.
"""

import math
import numpy as np
import cvxpy
from utils.angle import angle_mod


class MPCSolver:
    """
    Model Predictive Control solver for vehicle path tracking.
    
    This class provides MPC optimization functionality for autonomous vehicle control,
    including both speed and steering control with constraints.
    """
    
    def __init__(self, 
                 prediction_horizon=5,
                 time_step=0.2,
                 wheelbase=2.5,
                 max_velocity=55.0/3.6,
                 min_velocity=-20.0/3.6,
                 max_acceleration=1.0,
                 max_steering_angle=np.deg2rad(45.0),
                 max_steering_rate=np.deg2rad(30.0),
                 input_cost_weights=[0.01, 0.01],
                 input_rate_cost_weights=[0.01, 1.0],
                 state_cost_weights=[1.0, 1.0, 0.5, 0.5],
                 terminal_cost_weights=None):
        """
        Initialize MPC solver with configuration parameters.
        
        Args:
            prediction_horizon (int): Number of prediction steps
            time_step (float): Time step duration [s]
            wheelbase (float): Vehicle wheelbase [m]
            max_velocity (float): Maximum forward velocity [m/s]
            min_velocity (float): Maximum reverse velocity [m/s]
            max_acceleration (float): Maximum acceleration/deceleration [m/s²]
            max_steering_angle (float): Maximum steering angle [rad]
            max_steering_rate (float): Maximum steering rate [rad/s]
            input_cost_weights (list): Cost weights for control inputs [acceleration, steering]
            input_rate_cost_weights (list): Cost weights for control input changes
            state_cost_weights (list): Cost weights for state deviations [x, y, velocity, yaw]
            terminal_cost_weights (list): Cost weights for terminal state (defaults to state_cost_weights)
        """
        # State and control dimensions
        self.state_dimension = 4  # [x, y, velocity, yaw]
        self.control_dimension = 2  # [acceleration, steering_angle]
        
        # MPC parameters
        self.prediction_horizon = prediction_horizon
        self.time_step = time_step
        self.wheelbase = wheelbase
        
        # Vehicle constraints
        self.max_velocity = max_velocity
        self.min_velocity = min_velocity
        self.max_acceleration = max_acceleration
        self.max_steering_angle = max_steering_angle
        self.max_steering_rate = max_steering_rate
        
        # Cost matrices
        self.input_cost_matrix = np.diag(input_cost_weights)
        self.input_rate_cost_matrix = np.diag(input_rate_cost_weights)
        self.state_cost_matrix = np.diag(state_cost_weights)
        self.terminal_cost_matrix = np.diag(terminal_cost_weights) if terminal_cost_weights else np.diag(state_cost_weights)
        
        # Solver status
        self.last_solve_status = None
        self.last_solve_time = None
    
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi] range."""
        return angle_mod(angle)
    
    def get_linearized_model_matrices(self, velocity, yaw_angle, steering_angle):
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
        A_matrix = np.zeros((self.state_dimension, self.state_dimension))
        A_matrix[0, 0] = 1.0  # x position
        A_matrix[1, 1] = 1.0  # y position
        A_matrix[2, 2] = 1.0  # velocity
        A_matrix[3, 3] = 1.0  # yaw angle

        # Position derivatives based on current state
        A_matrix[0, 2] = self.time_step * math.cos(yaw_angle)  # dx/dv
        A_matrix[0, 3] = -self.time_step * velocity * math.sin(yaw_angle)  # dx/dyaw
        A_matrix[1, 2] = self.time_step * math.sin(yaw_angle)  # dy/dv
        A_matrix[1, 3] = self.time_step * velocity * math.cos(yaw_angle)  # dy/dyaw
        A_matrix[3, 2] = self.time_step * math.tan(steering_angle) / self.wheelbase  # dyaw/dv

        # Control input matrix B
        B_matrix = np.zeros((self.state_dimension, self.control_dimension))
        B_matrix[2, 0] = self.time_step  # velocity response to acceleration
        B_matrix[3, 1] = self.time_step * velocity / (self.wheelbase * math.cos(steering_angle) ** 2)  # yaw response to steering

        # Constant offset vector C (linearization offset)
        C_vector = np.zeros(self.state_dimension)
        C_vector[0] = self.time_step * velocity * math.sin(yaw_angle) * yaw_angle
        C_vector[1] = -self.time_step * velocity * math.cos(yaw_angle) * yaw_angle
        C_vector[3] = -self.time_step * velocity * steering_angle / (self.wheelbase * math.cos(steering_angle) ** 2)

        return A_matrix, B_matrix, C_vector
    
    def convert_matrix_to_array(self, matrix):
        """Convert cvxpy matrix to numpy array."""
        return np.array(matrix).flatten()
    
    def predict_kinematic_motion(self, initial_state, control_sequence):
        """
        Predict vehicle motion using the true kinematic model (bicycle model).
        
        This provides a more accurate prediction compared to the linearized model
        used in the MPC optimization.
        
        Args:
            initial_state (list): Initial state [x, y, velocity, yaw]
            control_sequence (list): Sequence of control inputs [acceleration, steering_angle]
            
        Returns:
            np.array: Predicted state trajectory using kinematic model
        """
        prediction_horizon = len(control_sequence)
        predicted_states = np.zeros((self.state_dimension, prediction_horizon + 1))
        
        # Set initial state
        predicted_states[:, 0] = initial_state
        
        # Current state for simulation
        current_state = np.array(initial_state)
        
        # Simulate forward using kinematic model
        for i in range(prediction_horizon):
            if isinstance(control_sequence[i], (list, tuple, np.ndarray)) and len(control_sequence[i]) >= 2:
                acceleration = float(control_sequence[i][0])
                steering_angle = float(control_sequence[i][1])
            else:
                acceleration = float(control_sequence[i])
                steering_angle = 0.0
            
            # Apply steering angle constraints
            steering_angle = max(-self.max_steering_angle, min(self.max_steering_angle, steering_angle))
            
            # Kinematic model (bicycle model)
            x = current_state[0]
            y = current_state[1]
            velocity = current_state[2]
            yaw = current_state[3]
            
            # Update state using bicycle model
            new_x = x + velocity * math.cos(yaw) * self.time_step
            new_y = y + velocity * math.sin(yaw) * self.time_step
            new_yaw = yaw + velocity / self.wheelbase * math.tan(steering_angle) * self.time_step
            new_velocity = velocity + acceleration * self.time_step
            
            # Apply velocity constraints
            new_velocity = max(self.min_velocity, min(self.max_velocity, new_velocity))
            
            # Normalize yaw angle
            new_yaw = self.normalize_angle(new_yaw)
            
            # Update current state
            current_state = np.array([new_x, new_y, new_velocity, new_yaw])
            predicted_states[:, i + 1] = current_state
        
        return predicted_states
    
    def solve_with_kinematic_comparison(self, reference_trajectory, linearization_trajectory, initial_state, reference_steering):
        """
        Solve MPC and return both linearized and kinematic predictions for comparison.
        
        Args:
            reference_trajectory (np.array): Reference state trajectory to track
            linearization_trajectory (np.array): Trajectory around which to linearize
            initial_state (list): Initial state constraint
            reference_steering (np.array): Reference steering sequence
            
        Returns:
            tuple: (acceleration_sequence, steering_sequence, predicted_x, predicted_y,
                    predicted_yaw, predicted_velocity, kinematic_states)
        """
        # Solve MPC using linearized model
        result = self.solve(reference_trajectory, linearization_trajectory, initial_state, reference_steering)
        
        acceleration_sequence, steering_sequence, predicted_x, predicted_y, predicted_yaw, predicted_velocity = result
        
        # If MPC solved successfully, compute kinematic prediction
        if acceleration_sequence is not None and steering_sequence is not None:
            # Create control sequence for kinematic prediction
            control_sequence = list(zip(acceleration_sequence, steering_sequence))
            
            # Predict using kinematic model
            kinematic_states = self.predict_kinematic_motion(initial_state, control_sequence)
            
            # Extract kinematic predictions
            kinematic_x = kinematic_states[0, :]
            kinematic_y = kinematic_states[1, :]
            kinematic_velocity = kinematic_states[2, :]
            kinematic_yaw = kinematic_states[3, :]
            
            return (
                acceleration_sequence,
                steering_sequence,
                predicted_x,
                predicted_y,
                predicted_yaw,
                predicted_velocity,
                {
                    'x': kinematic_x,
                    'y': kinematic_y,
                    'velocity': kinematic_velocity,
                    'yaw': kinematic_yaw,
                    'states': kinematic_states
                }
            )
        else:
            return (
                acceleration_sequence,
                steering_sequence,
                predicted_x,
                predicted_y,
                predicted_yaw,
                predicted_velocity,
                None
            )
    
    def compare_predictions(self, linearized_prediction, kinematic_prediction, title="Prediction Comparison", acceleration_sequence=None, steering_sequence=None):
        """
        Compare linearized and kinematic predictions and visualize the differences.
        
        Args:
            linearized_prediction (dict): Linearized model prediction results
            kinematic_prediction (dict): Kinematic model prediction results
            title (str): Title for the comparison plot
            acceleration_sequence (array): MPC acceleration sequence
            steering_sequence (array): MPC steering sequence
        """
        import matplotlib.pyplot as plt
        
        if kinematic_prediction is None:
            print("No kinematic prediction available for comparison")
            return
        
        # Extract data
        lin_x = linearized_prediction['x']
        lin_y = linearized_prediction['y']
        lin_velocity = linearized_prediction['velocity']
        lin_yaw = linearized_prediction['yaw']
        
        kin_x = kinematic_prediction['x']
        kin_y = kinematic_prediction['y']
        kin_velocity = kinematic_prediction['velocity']
        kin_yaw = kinematic_prediction['yaw']
        
        # Create comparison plots - 2x3 layout to include control sequences
        fig, axes = plt.subplots(2, 3, figsize=(18, 10))
        fig.suptitle(title, fontsize=14)
        
        # Plot 1: Trajectory comparison
        ax1 = axes[0, 0]
        ax1.plot(lin_x, lin_y, 'b-o', label='Linearized Model', markersize=4, linewidth=2, alpha=0.3)
        ax1.plot(kin_x, kin_y, 'r-s', label='Kinematic Model', markersize=4, linewidth=2, alpha=0.3)
        ax1.plot(lin_x[0], lin_y[0], 'go', label='Start', markersize=8, alpha=0.3)
        ax1.plot(lin_x[-1], lin_y[-1], 'ro', label='End', markersize=8, alpha=0.3)
        ax1.set_xlabel('X position [m]')
        ax1.set_ylabel('Y position [m]')
        ax1.legend()
        ax1.grid(True)
        ax1.axis('equal')
        ax1.set_title('Trajectory Comparison')
        
        # Plot 2: Velocity comparison
        ax2 = axes[0, 1]
        time_steps = np.arange(len(lin_velocity)) * self.time_step
        ax2.plot(time_steps, lin_velocity, 'b-o', label='Linearized Model', markersize=4, linewidth=2, alpha=0.3)
        ax2.plot(time_steps, kin_velocity, 'r-s', label='Kinematic Model', markersize=4, linewidth=2, alpha=0.3)
        ax2.set_xlabel('Time [s]')
        ax2.set_ylabel('Velocity [m/s]')
        ax2.legend()
        ax2.grid(True)
        ax2.set_title('Velocity Comparison')
        
        # Plot 3: Yaw angle comparison
        ax3 = axes[0, 2]
        ax3.plot(time_steps, [math.degrees(y) for y in lin_yaw], 'b-o', 
                label='Linearized Model', markersize=4, linewidth=2, alpha=0.3)
        ax3.plot(time_steps, [math.degrees(y) for y in kin_yaw], 'r-s', 
                label='Kinematic Model', markersize=4, linewidth=2, alpha=0.3)
        ax3.set_xlabel('Time [s]')
        ax3.set_ylabel('Yaw angle [deg]')
        ax3.legend()
        ax3.grid(True)
        ax3.set_title('Yaw Angle Comparison')
        
        # Plot 4: Position error
        ax4 = axes[1, 0]
        position_error = np.sqrt((lin_x - kin_x)**2 + (lin_y - kin_y)**2)
        ax4.plot(time_steps, position_error, 'g-s', markersize=4, linewidth=2, alpha=0.3)
        ax4.set_xlabel('Time [s]')
        ax4.set_ylabel('Position Error [m]')
        ax4.grid(True)
        ax4.set_title('Position Error (Linearized vs Kinematic)')
        
        # Plot 5: Acceleration sequence
        ax5 = axes[1, 1]
        if acceleration_sequence is not None:
            control_time_steps = np.arange(len(acceleration_sequence)) * self.time_step
            ax5.plot(control_time_steps, acceleration_sequence, 'm-o', 
                    label='Acceleration', markersize=6, linewidth=2, alpha=0.8)
            ax5.axhline(y=0, color='k', linestyle='--', alpha=0.5)
            ax5.set_xlabel('Time [s]')
            ax5.set_ylabel('Acceleration [m/s²]')
            ax5.legend()
            ax5.grid(True)
            ax5.set_title('MPC Acceleration Control')
        else:
            ax5.text(0.5, 0.5, 'No acceleration data', ha='center', va='center', transform=ax5.transAxes)
            ax5.set_title('MPC Acceleration Control')
        
        # Plot 6: Steering sequence
        ax6 = axes[1, 2]
        if steering_sequence is not None:
            control_time_steps = np.arange(len(steering_sequence)) * self.time_step
            ax6.plot(control_time_steps, [math.degrees(s) for s in steering_sequence], 'c-s', 
                    label='Steering', markersize=6, linewidth=2, alpha=0.8)
            ax6.axhline(y=0, color='k', linestyle='--', alpha=0.5)
            ax6.set_xlabel('Time [s]')
            ax6.set_ylabel('Steering Angle [deg]')
            ax6.legend()
            ax6.grid(True)
            ax6.set_title('MPC Steering Control')
        else:
            ax6.text(0.5, 0.5, 'No steering data', ha='center', va='center', transform=ax6.transAxes)
            ax6.set_title('MPC Steering Control')
        
        plt.tight_layout()
        plt.show()
        
        # Print statistics
        max_position_error = np.max(position_error)
        mean_position_error = np.mean(position_error)
        max_velocity_error = np.max(np.abs(lin_velocity - kin_velocity))
        max_yaw_error = np.max(np.abs(lin_yaw - kin_yaw))
        
        print(f"\n=== Prediction Comparison Statistics ===")
        print(f"Max position error: {max_position_error:.4f} m")
        print(f"Mean position error: {mean_position_error:.4f} m")
        print(f"Max velocity error: {max_velocity_error:.4f} m/s")
        print(f"Max yaw error: {math.degrees(max_yaw_error):.4f} deg")
        
        return {
            'max_position_error': max_position_error,
            'mean_position_error': mean_position_error,
            'max_velocity_error': max_velocity_error,
            'max_yaw_error': max_yaw_error,
            'position_error': position_error
        }
    
    def solve(self, reference_trajectory, linearization_trajectory, initial_state, reference_steering):
        """
        Solve linear MPC optimization problem.

        Args:
            reference_trajectory (np.array): Reference state trajectory to track
                - Shape: (4, prediction_horizon + 1)
                - Order: rows are [x, y, velocity, yaw], columns are time steps 0..N
                - Units: x/y in meters, velocity in m/s, yaw in radians
                - Used in: state tracking cost at steps 1..N-1 and terminal cost at step N
            
            linearization_trajectory (np.array): Trajectory around which to linearize
                - Shape: (4, prediction_horizon)
                - Order: rows [x, y, velocity, yaw], columns 0..N-1
                - Used in: computing A, B, C matrices via get_linearized_model_matrices()
                - Provide reasonable nominal trajectory (e.g., last solve's prediction)
            
            initial_state (list): Initial state constraint
                - Length: 4 -> [x0, y0, v0, yaw0]
                - Used in: state_variables[:, 0] == initial_state
                - Units: meters, m/s, radians
            
            reference_steering (np.array): Reference steering sequence for linearization
                - Shape: (1, prediction_horizon)
                - Used in: steering_angle = reference_steering[0, t] for linearization
                - Note: This is not a hard constraint, only affects linearization point

        Returns:
            tuple: (acceleration_sequence, steering_sequence, predicted_x, predicted_y,
                    predicted_yaw, predicted_velocity)
                - acceleration_sequence: shape (prediction_horizon,), units m/s²
                - steering_sequence: shape (prediction_horizon,), units radians
                - predicted_x, predicted_y, predicted_yaw, predicted_velocity: 
                  each shape (prediction_horizon + 1,)
        """
        import time
        start_time = time.time()
        
        # Decision variables
        state_variables = cvxpy.Variable((self.state_dimension, self.prediction_horizon + 1))
        control_variables = cvxpy.Variable((self.control_dimension, self.prediction_horizon))

        # Initialize cost and constraints
        total_cost = 0.0
        constraints = []

        # Build cost function and constraints over prediction horizon
        for time_step in range(self.prediction_horizon):
            # Control input cost
            total_cost += cvxpy.quad_form(control_variables[:, time_step], self.input_cost_matrix)

            # State tracking cost (skip first step)
            if time_step != 0:
                state_error = reference_trajectory[:, time_step] - state_variables[:, time_step]
                total_cost += cvxpy.quad_form(state_error, self.state_cost_matrix)

            # Get linearized model matrices
            velocity = linearization_trajectory[2, time_step]
            yaw_angle = linearization_trajectory[3, time_step]
            steering_angle = reference_steering[0, time_step]
            A_matrix, B_matrix, C_vector = self.get_linearized_model_matrices(velocity, yaw_angle, steering_angle)

            # System dynamics constraint
            next_state = A_matrix @ state_variables[:, time_step] + B_matrix @ control_variables[:, time_step] + C_vector
            constraints += [state_variables[:, time_step + 1] == next_state]

            # Control rate constraints
            if time_step < (self.prediction_horizon - 1):
                control_rate = control_variables[:, time_step + 1] - control_variables[:, time_step]
                total_cost += cvxpy.quad_form(control_rate, self.input_rate_cost_matrix)

                # Steering rate limit
                steering_rate_limit = self.max_steering_rate * self.time_step
                constraints += [
                    cvxpy.abs(control_variables[1, time_step + 1] - control_variables[1, time_step]) <= steering_rate_limit
                ]

        # Terminal cost
        terminal_state_error = reference_trajectory[:, self.prediction_horizon] - state_variables[:, self.prediction_horizon]
        total_cost += cvxpy.quad_form(terminal_state_error, self.terminal_cost_matrix)

        # State and control constraints
        constraints += [state_variables[:, 0] == initial_state]  # Initial state
        constraints += [state_variables[2, :] <= self.max_velocity]  # Maximum velocity
        constraints += [state_variables[2, :] >= self.min_velocity]  # Minimum velocity
        constraints += [cvxpy.abs(control_variables[0, :]) <= self.max_acceleration]  # Acceleration limits
        constraints += [cvxpy.abs(control_variables[1, :]) <= self.max_steering_angle]  # Steering limits

        # Solve optimization problem
        optimization_problem = cvxpy.Problem(cvxpy.Minimize(total_cost), constraints)
        optimization_problem.solve(solver=cvxpy.CLARABEL, verbose=False)
        
        # Record solve status and time
        self.last_solve_status = optimization_problem.status
        self.last_solve_time = time.time() - start_time

        # Extract solution
        if optimization_problem.status in [cvxpy.OPTIMAL, cvxpy.OPTIMAL_INACCURATE] and state_variables.value is not None and control_variables.value is not None:
            predicted_x = self.convert_matrix_to_array(state_variables.value[0, :])
            predicted_y = self.convert_matrix_to_array(state_variables.value[1, :])
            predicted_velocity = self.convert_matrix_to_array(state_variables.value[2, :])
            predicted_yaw = self.convert_matrix_to_array(state_variables.value[3, :])
            acceleration_sequence = self.convert_matrix_to_array(control_variables.value[0, :])
            steering_sequence = self.convert_matrix_to_array(control_variables.value[1, :])
        else:
            print(f"Error: Cannot solve MPC optimization problem. Status: {optimization_problem.status}")
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
    
    def get_solver_info(self):
        """Get information about the last solve."""
        return {
            'status': self.last_solve_status,
            'solve_time': self.last_solve_time,
            'prediction_horizon': self.prediction_horizon,
            'time_step': self.time_step
        }
    
    def update_parameters(self, **kwargs):
        """Update MPC parameters dynamically."""
        for key, value in kwargs.items():
            if hasattr(self, key):
                setattr(self, key, value)
                # Update cost matrices if weights changed
                if key in ['input_cost_weights', 'input_rate_cost_weights', 'state_cost_weights', 'terminal_cost_weights']:
                    if key == 'input_cost_weights':
                        self.input_cost_matrix = np.diag(value)
                    elif key == 'input_rate_cost_weights':
                        self.input_rate_cost_matrix = np.diag(value)
                    elif key == 'state_cost_weights':
                        self.state_cost_matrix = np.diag(value)
                    elif key == 'terminal_cost_weights':
                        self.terminal_cost_matrix = np.diag(value)
            else:
                print(f"Warning: Unknown parameter '{key}'")


