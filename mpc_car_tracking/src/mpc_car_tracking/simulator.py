"""
Simulation environment for testing MPC car tracking.

This module provides a simulation framework to test the MPC controller
with various scenarios and disturbances.
"""

import numpy as np
import time
from typing import List, Dict, Any, Optional, Callable, Tuple
from dataclasses import dataclass, field
from enum import Enum

from .vehicle_model import VehicleState, VehicleControl, BicycleModel, VehicleParameters
from .mpc_controller import MPCController, MPCParameters, MPCResult
from .trajectory_planner import TrajectoryPlanner, TrajectoryPoint


class SimulationStatus(Enum):
    """Simulation status enumeration."""
    READY = "ready"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"


@dataclass
class DisturbanceModel:
    """Model for simulation disturbances.
    
    Attributes:
        wind_force: Constant wind force [fx, fy] in N
        wind_noise_std: Standard deviation of wind noise
        measurement_noise_std: Standard deviation of state measurement noise
        actuator_delay: Actuator delay in seconds
        actuator_noise_std: Standard deviation of actuator noise
    """
    wind_force: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0]))
    wind_noise_std: float = 0.0
    measurement_noise_std: float = 0.0
    actuator_delay: float = 0.0
    actuator_noise_std: float = 0.0


@dataclass 
class SimulationConfig:
    """Configuration for simulation.
    
    Attributes:
        dt: Simulation time step
        total_time: Total simulation time
        real_time_factor: Real-time simulation factor (1.0 = real-time)
        save_data: Whether to save simulation data
        disturbance: Disturbance model
        vehicle_params: Vehicle parameters
        mpc_params: MPC parameters
    """
    dt: float = 0.1
    total_time: float = 30.0
    real_time_factor: float = 1.0
    save_data: bool = True
    disturbance: DisturbanceModel = field(default_factory=DisturbanceModel)
    vehicle_params: VehicleParameters = field(default_factory=VehicleParameters)
    mpc_params: MPCParameters = field(default_factory=MPCParameters)


@dataclass
class SimulationData:
    """Container for simulation data.
    
    Attributes:
        time: Time vector
        states: Vehicle states over time
        controls: Control inputs over time
        reference_states: Reference trajectory states
        mpc_results: MPC solver results
        tracking_errors: Tracking errors over time
        computational_times: MPC solve times
    """
    time: List[float] = field(default_factory=list)
    states: List[VehicleState] = field(default_factory=list)
    controls: List[VehicleControl] = field(default_factory=list)
    reference_states: List[VehicleState] = field(default_factory=list)
    mpc_results: List[MPCResult] = field(default_factory=list)
    tracking_errors: List[Dict[str, float]] = field(default_factory=list)
    computational_times: List[float] = field(default_factory=list)


class MPCSimulator:
    """Simulation environment for MPC car tracking."""
    
    def __init__(self, config: Optional[SimulationConfig] = None):
        """Initialize simulator.
        
        Args:
            config: Simulation configuration
        """
        self.config = config or SimulationConfig()
        
        # Initialize components
        self.vehicle_model = BicycleModel(self.config.vehicle_params)
        self.mpc_controller = MPCController(self.vehicle_model, self.config.mpc_params)
        self.trajectory_planner = TrajectoryPlanner(self.config.vehicle_params)
        
        # Simulation state
        self.status = SimulationStatus.READY
        self.data = SimulationData()
        
        # Control delay buffer for actuator delay simulation
        self.control_buffer: List[VehicleControl] = []
        
        # Random number generator for reproducible results
        self.rng = np.random.RandomState(42)
    
    def run_simulation(self,
                      initial_state: VehicleState,
                      reference_trajectory: List[VehicleState],
                      progress_callback: Optional[Callable[[float], None]] = None) -> SimulationData:
        """Run simulation with MPC controller.
        
        Args:
            initial_state: Initial vehicle state
            reference_trajectory: Reference trajectory to track
            progress_callback: Optional callback for progress updates
            
        Returns:
            Simulation data
        """
        if self.status == SimulationStatus.RUNNING:
            raise RuntimeError("Simulation already running")
        
        # Reset simulation
        self._reset_simulation()
        self.status = SimulationStatus.RUNNING
        
        try:
            # Initialize state
            current_state = initial_state
            current_control = VehicleControl()
            
            # Time parameters
            time_steps = int(self.config.total_time / self.config.dt)
            
            # Extend reference trajectory if needed
            if len(reference_trajectory) < time_steps + self.config.mpc_params.prediction_horizon:
                reference_trajectory = self._extend_trajectory(reference_trajectory, time_steps)
            
            # Initialize control buffer for delays
            delay_steps = int(self.config.disturbance.actuator_delay / self.config.dt)
            self.control_buffer = [VehicleControl() for _ in range(delay_steps + 1)]
            
            # Simulation loop
            for step in range(time_steps):
                current_time = step * self.config.dt
                
                # Update progress
                if progress_callback:
                    progress = step / time_steps
                    progress_callback(progress)
                
                # Get reference trajectory segment for MPC
                ref_start_idx = step
                ref_end_idx = min(step + self.config.mpc_params.prediction_horizon + 1, 
                                len(reference_trajectory))
                ref_segment = reference_trajectory[ref_start_idx:ref_end_idx]
                
                # Pad reference if needed
                while len(ref_segment) < self.config.mpc_params.prediction_horizon + 1:
                    ref_segment.append(reference_trajectory[-1])
                
                # Add measurement noise
                noisy_state = self._add_measurement_noise(current_state)
                
                # Solve MPC
                start_solve_time = time.time()
                control, mpc_result = self.mpc_controller.get_control(
                    noisy_state, ref_segment, current_control
                )
                solve_time = time.time() - start_solve_time
                
                # Add actuator noise and delay
                control = self._apply_actuator_effects(control)
                
                # Apply disturbances
                disturbed_control = self._apply_disturbances(control, current_state)
                
                # Simulate vehicle dynamics
                next_state = self.vehicle_model.step(current_state, disturbed_control, self.config.dt)
                
                # Calculate tracking error
                ref_state = reference_trajectory[step] if step < len(reference_trajectory) else reference_trajectory[-1]
                error = self._calculate_tracking_error(current_state, ref_state)
                
                # Store data
                if self.config.save_data:
                    self.data.time.append(current_time)
                    self.data.states.append(current_state)
                    self.data.controls.append(control)
                    self.data.reference_states.append(ref_state)
                    self.data.mpc_results.append(mpc_result)
                    self.data.tracking_errors.append(error)
                    self.data.computational_times.append(solve_time)
                
                # Update for next iteration
                current_state = next_state
                current_control = control
                
                # Real-time simulation delay
                if self.config.real_time_factor > 0:
                    time.sleep(self.config.dt / self.config.real_time_factor)
            
            self.status = SimulationStatus.COMPLETED
            
        except Exception as e:
            self.status = SimulationStatus.FAILED
            raise RuntimeError(f"Simulation failed: {str(e)}")
        
        return self.data
    
    def run_scenario(self, scenario_name: str, **kwargs) -> SimulationData:
        """Run predefined simulation scenario.
        
        Args:
            scenario_name: Name of the scenario
            **kwargs: Additional scenario parameters
            
        Returns:
            Simulation data
        """
        scenarios = {
            'straight_line': self._scenario_straight_line,
            'circular_track': self._scenario_circular_track,
            'figure_eight': self._scenario_figure_eight,
            'lane_change': self._scenario_lane_change,
            'slalom': self._scenario_slalom
        }
        
        if scenario_name not in scenarios:
            raise ValueError(f"Unknown scenario: {scenario_name}")
        
        initial_state, reference_trajectory = scenarios[scenario_name](**kwargs)
        return self.run_simulation(initial_state, reference_trajectory)
    
    def _reset_simulation(self):
        """Reset simulation data."""
        self.data = SimulationData()
        self.status = SimulationStatus.READY
    
    def _extend_trajectory(self, trajectory: List[VehicleState], required_length: int) -> List[VehicleState]:
        """Extend trajectory by repeating the last state."""
        extended = trajectory.copy()
        while len(extended) < required_length:
            extended.append(trajectory[-1])
        return extended
    
    def _add_measurement_noise(self, state: VehicleState) -> VehicleState:
        """Add measurement noise to state."""
        if self.config.disturbance.measurement_noise_std <= 0:
            return state
        
        noise = self.rng.normal(0, self.config.disturbance.measurement_noise_std, 4)
        
        return VehicleState(
            x=state.x + noise[0],
            y=state.y + noise[1],
            yaw=state.yaw + noise[2],
            v=max(0, state.v + noise[3])  # Ensure non-negative velocity
        )
    
    def _apply_actuator_effects(self, control: VehicleControl) -> VehicleControl:
        """Apply actuator delay and noise."""
        # Add to buffer
        self.control_buffer.append(control)
        
        # Get delayed control
        delayed_control = self.control_buffer.pop(0)
        
        # Add actuator noise
        if self.config.disturbance.actuator_noise_std > 0:
            noise = self.rng.normal(0, self.config.disturbance.actuator_noise_std, 2)
            delayed_control = VehicleControl(
                acceleration=delayed_control.acceleration + noise[0],
                steering_angle=delayed_control.steering_angle + noise[1]
            )
        
        return delayed_control
    
    def _apply_disturbances(self, control: VehicleControl, state: VehicleState) -> VehicleControl:
        """Apply external disturbances."""
        # For simplicity, wind force is modeled as additional acceleration
        if np.any(self.config.disturbance.wind_force != 0) or self.config.disturbance.wind_noise_std > 0:
            # Convert wind force to acceleration (assuming unit mass)
            wind_accel = self.config.disturbance.wind_force.copy()
            
            # Add wind noise
            if self.config.disturbance.wind_noise_std > 0:
                wind_noise = self.rng.normal(0, self.config.disturbance.wind_noise_std, 2)
                wind_accel += wind_noise
            
            # Project wind acceleration to vehicle frame
            cos_yaw = np.cos(state.yaw)
            sin_yaw = np.sin(state.yaw)
            
            # Longitudinal wind component
            wind_long = wind_accel[0] * cos_yaw + wind_accel[1] * sin_yaw
            
            # Add to control acceleration
            disturbed_control = VehicleControl(
                acceleration=control.acceleration + wind_long,
                steering_angle=control.steering_angle
            )
            
            return disturbed_control
        
        return control
    
    def _calculate_tracking_error(self, current_state: VehicleState, ref_state: VehicleState) -> Dict[str, float]:
        """Calculate tracking errors."""
        position_error = np.sqrt((current_state.x - ref_state.x)**2 + 
                               (current_state.y - ref_state.y)**2)
        
        yaw_error = current_state.yaw - ref_state.yaw
        # Normalize yaw error to [-pi, pi]
        while yaw_error > np.pi:
            yaw_error -= 2 * np.pi
        while yaw_error < -np.pi:
            yaw_error += 2 * np.pi
        
        return {
            'position_error': position_error,
            'x_error': current_state.x - ref_state.x,
            'y_error': current_state.y - ref_state.y,
            'yaw_error': yaw_error,
            'velocity_error': current_state.v - ref_state.v
        }
    
    # Predefined scenarios
    def _scenario_straight_line(self, distance: float = 100.0, speed: float = 15.0) -> Tuple[VehicleState, List[VehicleState]]:
        """Straight line tracking scenario."""
        initial_state = VehicleState(x=0, y=0, yaw=0, v=speed)
        end_state = VehicleState(x=distance, y=0, yaw=0, v=speed)
        
        trajectory_points = self.trajectory_planner.straight_line(
            initial_state, end_state, num_points=int(distance/speed/self.config.dt), target_speed=speed
        )
        
        reference_trajectory = [tp.state for tp in trajectory_points]
        return initial_state, reference_trajectory
    
    def _scenario_circular_track(self, radius: float = 30.0, speed: float = 10.0) -> Tuple[VehicleState, List[VehicleState]]:
        """Circular track scenario."""
        initial_state = VehicleState(x=radius, y=0, yaw=np.pi/2, v=speed)
        
        trajectory_points = self.trajectory_planner.circular_path(
            center=(0, 0), radius=radius, start_angle=0, end_angle=2*np.pi,
            num_points=int(2*np.pi*radius/speed/self.config.dt), target_speed=speed
        )
        
        reference_trajectory = [tp.state for tp in trajectory_points]
        return initial_state, reference_trajectory
    
    def _scenario_figure_eight(self, width: float = 40.0, height: float = 20.0, speed: float = 8.0) -> Tuple[VehicleState, List[VehicleState]]:
        """Figure-8 tracking scenario."""
        initial_state = VehicleState(x=width/4, y=0, yaw=0, v=speed)
        
        trajectory_points = self.trajectory_planner.figure_eight(
            center=(0, 0), width=width, height=height,
            num_points=int(self.config.total_time/self.config.dt), target_speed=speed
        )
        
        reference_trajectory = [tp.state for tp in trajectory_points]
        return initial_state, reference_trajectory
    
    def _scenario_lane_change(self, lane_width: float = 3.5, speed: float = 20.0) -> Tuple[VehicleState, List[VehicleState]]:
        """Lane change scenario."""
        initial_state = VehicleState(x=0, y=0, yaw=0, v=speed)
        
        trajectory_points = self.trajectory_planner.lane_change(
            initial_state, lane_offset=lane_width, lane_change_distance=50.0,
            target_speed=speed, num_points=int(self.config.total_time/self.config.dt)
        )
        
        reference_trajectory = [tp.state for tp in trajectory_points]
        return initial_state, reference_trajectory
    
    def _scenario_slalom(self, cone_spacing: float = 20.0, cone_offset: float = 5.0, speed: float = 12.0) -> Tuple[VehicleState, List[VehicleState]]:
        """Slalom course scenario."""
        # Create waypoints for slalom
        waypoints = [(0, 0)]
        num_cones = int(self.config.total_time * speed / cone_spacing)
        
        for i in range(1, num_cones + 1):
            x = i * cone_spacing
            y = cone_offset * (-1)**i  # Alternate left and right
            waypoints.append((x, y))
        
        initial_state = VehicleState(x=0, y=0, yaw=0, v=speed)
        
        trajectory_points = self.trajectory_planner.spline_path(
            waypoints, target_speed=speed, 
            num_points=int(self.config.total_time/self.config.dt)
        )
        
        reference_trajectory = [tp.state for tp in trajectory_points]
        return initial_state, reference_trajectory
    
    def get_performance_metrics(self) -> Dict[str, float]:
        """Calculate performance metrics from simulation data."""
        if not self.data.tracking_errors:
            return {}
        
        # Extract error data
        position_errors = [err['position_error'] for err in self.data.tracking_errors]
        yaw_errors = [abs(err['yaw_error']) for err in self.data.tracking_errors]
        velocity_errors = [abs(err['velocity_error']) for err in self.data.tracking_errors]
        solve_times = self.data.computational_times
        
        # Calculate metrics
        metrics = {
            'max_position_error': max(position_errors),
            'mean_position_error': np.mean(position_errors),
            'rms_position_error': np.sqrt(np.mean(np.array(position_errors)**2)),
            'max_yaw_error': max(yaw_errors),
            'mean_yaw_error': np.mean(yaw_errors),
            'max_velocity_error': max(velocity_errors),
            'mean_velocity_error': np.mean(velocity_errors),
            'max_solve_time': max(solve_times),
            'mean_solve_time': np.mean(solve_times),
            'solve_success_rate': np.mean([r.success for r in self.data.mpc_results if r is not None])
        }
        
        return metrics