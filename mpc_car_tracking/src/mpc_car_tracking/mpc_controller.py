"""
Model Predictive Control (MPC) controller for car tracking.

This module implements an MPC controller that solves an optimization problem
to track a reference trajectory while respecting vehicle dynamics and constraints.
"""

import numpy as np
import cvxpy as cp
from typing import List, Tuple, Optional
from dataclasses import dataclass

from .vehicle_model import VehicleState, VehicleControl, BicycleModel, VehicleParameters


@dataclass
class MPCParameters:
    """MPC controller parameters.
    
    Attributes:
        prediction_horizon: Number of prediction steps
        control_horizon: Number of control steps (usually <= prediction_horizon)
        dt: Time step (s)
        Q: State cost matrix weights [x, y, yaw, v]
        R: Control cost matrix weights [acceleration, steering_angle]
        Q_terminal: Terminal state cost matrix weights
        max_iterations: Maximum solver iterations
        solver_verbose: Whether to print solver output
    """
    prediction_horizon: int = 20
    control_horizon: int = 20
    dt: float = 0.1
    Q: np.ndarray = None
    R: np.ndarray = None
    Q_terminal: np.ndarray = None
    max_iterations: int = 1000
    solver_verbose: bool = False
    
    def __post_init__(self):
        """Set default cost matrices if not provided."""
        if self.Q is None:
            # State cost weights: [x_error, y_error, yaw_error, v_error]
            self.Q = np.diag([10.0, 10.0, 1.0, 1.0])
        
        if self.R is None:
            # Control cost weights: [acceleration, steering_angle]
            self.R = np.diag([0.1, 1.0])
        
        if self.Q_terminal is None:
            # Terminal cost (higher weights for final state)
            self.Q_terminal = self.Q * 2.0


@dataclass
class MPCResult:
    """Result from MPC optimization.
    
    Attributes:
        success: Whether optimization was successful
        optimal_control: Optimal control sequence
        predicted_states: Predicted state trajectory
        cost: Optimal cost value
        solve_time: Time taken to solve (s)
        status: Solver status string
    """
    success: bool
    optimal_control: Optional[List[VehicleControl]] = None
    predicted_states: Optional[List[VehicleState]] = None
    cost: Optional[float] = None
    solve_time: Optional[float] = None
    status: Optional[str] = None


class MPCController:
    """Model Predictive Control controller for vehicle tracking."""
    
    def __init__(self, 
                 vehicle_model: BicycleModel,
                 mpc_params: Optional[MPCParameters] = None):
        """Initialize MPC controller.
        
        Args:
            vehicle_model: Vehicle dynamics model
            mpc_params: MPC parameters. If None, default parameters are used.
        """
        self.vehicle_model = vehicle_model
        self.params = mpc_params or MPCParameters()
        
        # CVXPY variables and parameters
        self._setup_optimization_problem()
    
    def _setup_optimization_problem(self):
        """Set up the CVXPY optimization problem."""
        N = self.params.prediction_horizon
        
        # Decision variables
        # States: [x, y, yaw, v] for each time step
        self.x_var = cp.Variable((4, N + 1))
        # Controls: [acceleration, steering_angle] for each time step
        self.u_var = cp.Variable((2, N))
        
        # Parameters (will be updated at each solve)
        self.x0_param = cp.Parameter(4)  # Initial state
        self.x_ref_param = cp.Parameter((4, N + 1))  # Reference trajectory
        
        # Cost matrices as parameters for flexibility
        self.Q_param = cp.Parameter((4, 4), PSD=True)
        self.R_param = cp.Parameter((2, 2), PSD=True)
        self.Q_terminal_param = cp.Parameter((4, 4), PSD=True)
        
        # Linearization matrices (updated at each solve)
        self.A_matrices = [cp.Parameter((4, 4)) for _ in range(N)]
        self.B_matrices = [cp.Parameter((4, 2)) for _ in range(N)]
        
        # Get constraint bounds
        state_lb, state_ub = self.vehicle_model.get_state_bounds()
        control_lb, control_ub = self.vehicle_model.get_control_bounds()
        
        # Build constraints
        constraints = []
        
        # Initial state constraint
        constraints.append(self.x_var[:, 0] == self.x0_param)
        
        # Dynamics constraints (linearized)
        for k in range(N):
            constraints.append(
                self.x_var[:, k + 1] == self.A_matrices[k] @ self.x_var[:, k] + 
                                        self.B_matrices[k] @ self.u_var[:, k]
            )
        
        # State constraints
        for k in range(N + 1):
            # Velocity bounds
            constraints.append(self.x_var[3, k] >= state_lb[3])
            constraints.append(self.x_var[3, k] <= state_ub[3])
            # Yaw angle bounds (periodic, handled in solver)
            constraints.append(self.x_var[2, k] >= state_lb[2])
            constraints.append(self.x_var[2, k] <= state_ub[2])
        
        # Control constraints
        for k in range(N):
            constraints.append(self.u_var[:, k] >= control_lb)
            constraints.append(self.u_var[:, k] <= control_ub)
        
        # Build objective function
        cost = 0
        
        # Stage costs
        for k in range(N):
            state_error = self.x_var[:, k] - self.x_ref_param[:, k]
            cost += cp.quad_form(state_error, self.Q_param)
            cost += cp.quad_form(self.u_var[:, k], self.R_param)
        
        # Terminal cost
        terminal_error = self.x_var[:, N] - self.x_ref_param[:, N]
        cost += cp.quad_form(terminal_error, self.Q_terminal_param)
        
        # Create problem
        self.problem = cp.Problem(cp.Minimize(cost), constraints)
    
    def solve(self, 
              current_state: VehicleState,
              reference_trajectory: List[VehicleState],
              previous_control: Optional[VehicleControl] = None) -> MPCResult:
        """Solve MPC optimization problem.
        
        Args:
            current_state: Current vehicle state
            reference_trajectory: Reference trajectory to track
            previous_control: Previous control input (for warm start)
            
        Returns:
            MPC optimization result
        """
        import time
        
        start_time = time.time()
        
        # Validate reference trajectory length
        N = self.params.prediction_horizon
        if len(reference_trajectory) < N + 1:
            return MPCResult(
                success=False,
                status="Reference trajectory too short"
            )
        
        # Set initial state
        self.x0_param.value = current_state.to_array()
        
        # Set reference trajectory
        x_ref = np.zeros((4, N + 1))
        for k in range(N + 1):
            x_ref[:, k] = reference_trajectory[k].to_array()
        self.x_ref_param.value = x_ref
        
        # Set cost matrices
        self.Q_param.value = self.params.Q
        self.R_param.value = self.params.R
        self.Q_terminal_param.value = self.params.Q_terminal
        
        # Linearize dynamics around reference trajectory
        self._linearize_dynamics(reference_trajectory, previous_control)
        
        # Solve optimization problem
        try:
            self.problem.solve(
                solver=cp.OSQP,
                verbose=self.params.solver_verbose,
                max_iter=self.params.max_iterations,
                eps_abs=1e-6,
                eps_rel=1e-6
            )
            
            solve_time = time.time() - start_time
            
            if self.problem.status in [cp.OPTIMAL, cp.OPTIMAL_INACCURATE]:
                # Extract optimal solution
                optimal_controls = []
                predicted_states = []
                
                # Get predicted states
                for k in range(N + 1):
                    state_array = self.x_var.value[:, k]
                    predicted_states.append(VehicleState.from_array(state_array))
                
                # Get optimal controls
                for k in range(N):
                    control_array = self.u_var.value[:, k]
                    optimal_controls.append(VehicleControl(
                        acceleration=control_array[0],
                        steering_angle=control_array[1]
                    ))
                
                return MPCResult(
                    success=True,
                    optimal_control=optimal_controls,
                    predicted_states=predicted_states,
                    cost=self.problem.value,
                    solve_time=solve_time,
                    status=self.problem.status
                )
            
            else:
                return MPCResult(
                    success=False,
                    cost=None,
                    solve_time=solve_time,
                    status=self.problem.status
                )
                
        except Exception as e:
            return MPCResult(
                success=False,
                status=f"Solver error: {str(e)}"
            )
    
    def _linearize_dynamics(self, 
                          reference_trajectory: List[VehicleState],
                          previous_control: Optional[VehicleControl]):
        """Linearize vehicle dynamics around reference trajectory."""
        N = self.params.prediction_horizon
        
        # Default control if none provided
        if previous_control is None:
            previous_control = VehicleControl()
        
        # Linearize at each prediction step
        for k in range(N):
            # Use reference state and zero control for linearization
            ref_state = reference_trajectory[k]
            ref_control = VehicleControl()  # Assume zero control for linearization
            
            # Get linearized matrices
            A_k, B_k = self.vehicle_model.linearize(
                ref_state, ref_control, self.params.dt
            )
            
            # Set parameter values
            self.A_matrices[k].value = A_k
            self.B_matrices[k].value = B_k
    
    def get_control(self, 
                   current_state: VehicleState,
                   reference_trajectory: List[VehicleState],
                   previous_control: Optional[VehicleControl] = None) -> Tuple[VehicleControl, MPCResult]:
        """Get control input from MPC controller.
        
        Args:
            current_state: Current vehicle state
            reference_trajectory: Reference trajectory to track
            previous_control: Previous control input
            
        Returns:
            Tuple of (control_input, mpc_result)
        """
        result = self.solve(current_state, reference_trajectory, previous_control)
        
        if result.success and result.optimal_control:
            # Return first control input (receding horizon)
            return result.optimal_control[0], result
        else:
            # Return zero control if optimization failed
            return VehicleControl(), result
    
    def update_parameters(self, new_params: MPCParameters):
        """Update MPC parameters and rebuild optimization problem if needed."""
        old_horizon = self.params.prediction_horizon
        self.params = new_params
        
        # Rebuild optimization problem if horizon changed
        if new_params.prediction_horizon != old_horizon:
            self._setup_optimization_problem()