"""
Simulation Environment

A comprehensive simulation environment for testing Stanley controller performance
with obstacles, different scenarios, and evaluation metrics.
"""

import numpy as np
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass
import math

from .stanley_controller import StanleyController, ControlParams
from .vehicle_dynamics import VehicleDynamics, VehicleParameters, SimpleVehicleDynamics
from .utils.se2 import SE2


@dataclass
class Obstacle:
    """Obstacle representation"""
    x: float
    y: float
    radius: float
    
    def distance_to_point(self, x: float, y: float) -> float:
        """Calculate distance from obstacle center to a point."""
        return np.sqrt((x - self.x)**2 + (y - self.y)**2)
    
    def is_collision(self, x: float, y: float, safety_margin: float = 0.0) -> bool:
        """Check if a point collides with this obstacle."""
        return self.distance_to_point(x, y) < (self.radius + safety_margin)


@dataclass
class SimulationConfig:
    """Simulation configuration parameters"""
    dt: float = 0.1  # Time step [s]
    max_time: float = 100.0  # Maximum simulation time [s]
    collision_check: bool = True  # Enable collision checking
    collision_safety_margin: float = 0.5  # Safety margin for collision [m]
    goal_tolerance: float = 1.0  # Goal tolerance [m]
    max_iterations: int = 10000  # Maximum simulation steps


class SimulationEnvironment:
    """
    Simulation environment for testing Stanley controller performance.
    
    Features:
    - Multiple vehicle dynamics models
    - Obstacle avoidance
    - Path tracking evaluation
    - Real-time simulation
    - Performance metrics
    """
    
    def __init__(self, config: SimulationConfig):
        """
        Initialize simulation environment.
        
        Args:
            config: Simulation configuration
        """
        self.config = config
        self.obstacles: List[Obstacle] = []
        self.trajectory_history: List[SE2] = []
        self.control_history: List[Dict[str, float]] = []
        self.error_history: List[Dict[str, float]] = []
        self.collision_detected = False
        self.simulation_complete = False
        
    def add_obstacle(self, x: float, y: float, radius: float):
        """Add a circular obstacle to the environment."""
        self.obstacles.append(Obstacle(x, y, radius))
        
    def add_obstacles_from_list(self, obstacles: List[Tuple[float, float, float]]):
        """Add multiple obstacles from a list of (x, y, radius) tuples."""
        for obs in obstacles:
            self.add_obstacle(*obs)
            
    def clear_obstacles(self):
        """Remove all obstacles from the environment."""
        self.obstacles.clear()
        
    def check_collision(self, x: float, y: float) -> bool:
        """Check if a position collides with any obstacle."""
        if not self.config.collision_check:
            return False
            
        for obstacle in self.obstacles:
            if obstacle.is_collision(x, y, self.config.collision_safety_margin):
                return True
        return False
    
    def simulate(self,
                controller: StanleyController,
                vehicle_dynamics: VehicleDynamics,
                initial_state: SE2,
                path_points: np.ndarray,
                path_yaw: np.ndarray,
                target_speed: float) -> Dict[str, Any]:
        """
        Run a complete simulation.
        
        Args:
            controller: Stanley controller instance
            vehicle_dynamics: Vehicle dynamics model
            initial_state: Initial vehicle state
            path_points: Reference path points
            path_yaw: Reference path yaw angles
            target_speed: Target speed
            
        Returns:
            Simulation results dictionary
        """
        # Reset simulation state
        self.trajectory_history.clear()
        self.control_history.clear()
        self.error_history.clear()
        self.collision_detected = False
        self.simulation_complete = False
        vehicle_dynamics.reset()
        
        # Initialize state
        state = SE2(float(initial_state.x), float(initial_state.y), float(initial_state.theta))
        time = 0.0
        step = 0
        
        # Main simulation loop
        while (time < self.config.max_time and 
               step < self.config.max_iterations and 
               not self.simulation_complete and
               not self.collision_detected):
            
            # Store current state
            self.trajectory_history.append(SE2(float(state.x), float(state.y), float(state.theta)))
            
            # Compute control
            steering, acceleration, target_idx = controller.compute_control(
                state, path_points, path_yaw, target_speed)
            
            # Store control inputs
            self.control_history.append({
                'steering': float(steering),
                'acceleration': float(acceleration),
                'time': time
            })
            
            # Update vehicle dynamics
            vehicle_dynamics.update(self.config.dt, steering, acceleration, 0.0)
            state = vehicle_dynamics.get_state()
            
            # Calculate errors
            if target_idx < len(path_points):
                target_point = path_points[target_idx]
                distance_error = np.linalg.norm([state.x - target_point[0], state.y - target_point[1]])
                heading_error = abs(state.theta - path_yaw[target_idx])
                
                self.error_history.append({
                    'distance_error': float(distance_error),
                    'heading_error': float(heading_error),
                    'time': time
                })
            
            # Check for collisions
            if self.check_collision(state.x, state.y):
                self.collision_detected = True
                break
            
            # Check if goal is reached
            if target_idx >= len(path_points) - 1:
                goal_point = path_points[-1]
                distance_to_goal = np.linalg.norm([state.x - goal_point[0], state.y - goal_point[1]])
                if distance_to_goal < self.config.goal_tolerance:
                    self.simulation_complete = True
                    break
            
            # Update time
            time += self.config.dt
            step += 1
        
        # Calculate performance metrics
        metrics = self._calculate_metrics(path_points, target_speed)
        
        return {
            'success': self.simulation_complete,
            'collision': self.collision_detected,
            'time': time,
            'steps': step,
            'trajectory': np.array([[s.x, s.y, s.theta] for s in self.trajectory_history]),
            'controls': self.control_history,
            'errors': self.error_history,
            'metrics': metrics
        }
    
    def _calculate_metrics(self, path_points: np.ndarray, target_speed: float) -> Dict[str, float]:
        """Calculate performance metrics."""
        if not self.trajectory_history:
            return {}
        
        # Final state
        final_state = self.trajectory_history[-1]
        
        # Distance to final goal
        goal_point = path_points[-1]
        final_distance = np.linalg.norm([final_state.x - goal_point[0], final_state.y - goal_point[1]])
        
        # Average tracking error
        if self.error_history:
            avg_distance_error = np.mean([e['distance_error'] for e in self.error_history])
            avg_heading_error = np.mean([e['heading_error'] for e in self.error_history])
            max_distance_error = np.max([e['distance_error'] for e in self.error_history])
        else:
            avg_distance_error = 0.0
            avg_heading_error = 0.0
            max_distance_error = 0.0
        
        # Average speed
        avg_speed = np.mean([self.control_history[i]['time'] - self.control_history[i-1]['time'] 
                           for i in range(1, len(self.control_history))]) if len(self.control_history) > 1 else 0.0
        
        # Control smoothness (steering rate)
        if len(self.control_history) > 1:
            steering_rates = []
            for i in range(1, len(self.control_history)):
                dt = self.control_history[i]['time'] - self.control_history[i-1]['time']
                if dt > 0:
                    steering_rate = abs(self.control_history[i]['steering'] - self.control_history[i-1]['steering']) / dt
                    steering_rates.append(steering_rate)
            avg_steering_rate = np.mean(steering_rates) if steering_rates else 0.0
            max_steering_rate = np.max(steering_rates) if steering_rates else 0.0
        else:
            avg_steering_rate = 0.0
            max_steering_rate = 0.0
        
        return {
            'final_distance_to_goal': float(final_distance),
            'average_distance_error': float(avg_distance_error),
            'average_heading_error': float(avg_heading_error),
            'max_distance_error': float(max_distance_error),
            'average_speed': float(avg_speed),
            'target_speed': float(target_speed),
            'speed_error': float(avg_speed - target_speed),
            'average_steering_rate': float(avg_steering_rate),
            'max_steering_rate': float(max_steering_rate),
            'simulation_time': float(self.trajectory_history[-1].theta if self.trajectory_history else 0.0)
        }


class ScenarioGenerator:
    """Generate different test scenarios for simulation."""
    
    @staticmethod
    def straight_line_scenario(length: float = 100.0) -> Tuple[np.ndarray, np.ndarray]:
        """Generate a straight line scenario."""
        x = np.linspace(0, length, int(length / 0.5))
        y = np.zeros_like(x)
        yaw = np.zeros_like(x)
        return np.column_stack([x, y]), yaw
    
    @staticmethod
    def circular_scenario(radius: float = 20.0, num_points: int = 200) -> Tuple[np.ndarray, np.ndarray]:
        """Generate a circular scenario."""
        theta = np.linspace(0, 2 * np.pi, num_points)
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        yaw = theta + np.pi/2  # Tangent to circle
        return np.column_stack([x, y]), yaw
    
    @staticmethod
    def figure_eight_scenario(scale: float = 20.0, num_points: int = 400) -> Tuple[np.ndarray, np.ndarray]:
        """Generate a figure-eight scenario."""
        t = np.linspace(0, 2 * np.pi, num_points)
        x = scale * np.sin(t)
        y = scale * np.sin(t) * np.cos(t)
        yaw = np.arctan2(np.gradient(y), np.gradient(x))
        return np.column_stack([x, y]), yaw
    
    @staticmethod
    def highway_scenario(length: float = 200.0, lanes: int = 3, lane_width: float = 3.5) -> Tuple[np.ndarray, np.ndarray]:
        """Generate a highway scenario with multiple lanes."""
        x = np.linspace(0, length, int(length / 0.5))
        
        # Create a curved highway
        y_center = lane_width * lanes / 2
        y = y_center + 5 * np.sin(x / 30.0)  # Gentle sine wave
        
        # Calculate yaw (heading)
        yaw = np.arctan2(5 * np.cos(x / 30.0) / 30.0, 1.0)
        
        return np.column_stack([x, y]), yaw
    
    @staticmethod
    def parking_scenario() -> Tuple[np.ndarray, np.ndarray]:
        """Generate a parking scenario."""
        # Create a parking space
        x = [0, 2, 2, 0, 0]
        y = [0, 0, -5, -5, 0]
        
        # Create approach path
        approach_x = np.linspace(-10, 0, 50)
        approach_y = np.linspace(2.5, 0, 50)
        
        # Combine approach and parking
        x_full = np.concatenate([approach_x, x])
        y_full = np.concatenate([approach_y, y])
        
        # Calculate yaw
        yaw = np.arctan2(np.gradient(y_full), np.gradient(x_full))
        
        return np.column_stack([x_full, y_full]), yaw


class BatchEvaluator:
    """Evaluate controller performance across multiple scenarios."""
    
    def __init__(self, config: SimulationConfig):
        """
        Initialize batch evaluator.
        
        Args:
            config: Simulation configuration
        """
        self.config = config
        self.results = []
        
    def evaluate_scenarios(self,
                          controller: StanleyController,
                          scenarios: List[Tuple[str, Tuple[np.ndarray, np.ndarray]]],
                          target_speeds: List[float],
                          initial_states: List[SE2]) -> List[Dict[str, Any]]:
        """
        Evaluate controller across multiple scenarios.
        
        Args:
            controller: Stanley controller instance
            scenarios: List of (scenario_name, (path_points, path_yaw)) tuples
            target_speeds: List of target speeds to test
            initial_states: List of initial states
            
        Returns:
            List of evaluation results
        """
        self.results.clear()
        
        for scenario_name, (path_points, path_yaw) in scenarios:
            for target_speed in target_speeds:
                for initial_state in initial_states:
                    # Create simulation environment
                    sim_env = SimulationEnvironment(self.config)
                    
                    # Create simple vehicle dynamics for evaluation
                    vehicle_dynamics = VehicleDynamics(VehicleParameters())
                    
                    # Run simulation
                    result = sim_env.simulate(
                        controller, vehicle_dynamics, initial_state,
                        path_points, path_yaw, target_speed
                    )
                    
                    # Add scenario information
                    result['scenario'] = scenario_name
                    result['target_speed'] = target_speed
                    result['initial_state'] = {
                        'x': initial_state.x,
                        'y': initial_state.y,
                        'theta': initial_state.theta
                    }
                    
                    self.results.append(result)
        
        return self.results
    
    def summarize_results(self) -> Dict[str, Any]:
        """Summarize evaluation results."""
        if not self.results:
            return {}
        
        summary = {
            'total_scenarios': len(self.results),
            'successful_scenarios': sum(1 for r in self.results if r['success']),
            'collisions': sum(1 for r in self.results if r['collision']),
            'average_time': np.mean([r['time'] for r in self.results]),
            'average_final_distance': np.mean([r['metrics']['final_distance_to_goal'] for r in self.results]),
            'average_tracking_error': np.mean([r['metrics']['average_distance_error'] for r in self.results]),
        }
        
        # Group by scenario
        scenario_summary = {}
        for result in self.results:
            scenario = result['scenario']
            if scenario not in scenario_summary:
                scenario_summary[scenario] = {
                    'count': 0,
                    'success_rate': 0.0,
                    'avg_time': 0.0,
                    'avg_error': 0.0
                }
            
            scenario_summary[scenario]['count'] += 1
            scenario_summary[scenario]['avg_time'] += result['time']
            scenario_summary[scenario]['avg_error'] += result['metrics']['average_distance_error']
        
        for scenario, data in scenario_summary.items():
            data['success_rate'] = sum(1 for r in self.results if r['scenario'] == scenario and r['success']) / data['count']
            data['avg_time'] /= data['count']
            data['avg_error'] /= data['count']
        
        summary['scenario_summary'] = scenario_summary
        
        return summary