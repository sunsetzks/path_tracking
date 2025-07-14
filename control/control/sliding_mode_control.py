"""
Sliding Mode Control (SMC) for Vehicle Position and Velocity Control to Origin
==============================================================================

This module implements a sliding mode controller to bring a vehicle from an initial
state (p0, v0) to the origin (0, 0) with zero velocity. The controller is designed
to be robust against bounded disturbances.

Theory:
- System dynamics: ṗ(t) = v(t), v̇(t) = a(t) + d(t)
- Sliding surface: s(t) = p(t) + λv(t)
- Control law: a(t) = -(1/λ)v(t) - K·sat(s(t)/Φ)

Author: Generated for path tracking project
Date: 2024
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, Optional, List, Callable
import logging

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class SlidingModeController:
    """
    Sliding Mode Controller for position and velocity control to origin.
    
    The controller uses a sliding surface s(t) = p(t) + λv(t) to ensure
    exponential convergence to the origin while maintaining robustness
    against bounded disturbances.
    """
    
    def __init__(self, 
                 lambda_param: float = 1.0,
                 K: float = 2.0,
                 phi: float = 0.1,
                 a_min: float = -5.0,
                 a_max: float = 5.0,
                 disturbance_bound: float = 1.0):
        """
        Initialize the sliding mode controller.
        
        Args:
            lambda_param: Sliding surface slope parameter (λ > 0)
            K: Switching gain (K > disturbance_bound for stability)
            phi: Boundary layer thickness (Φ > 0)
            a_min: Minimum acceleration limit
            a_max: Maximum acceleration limit
            disturbance_bound: Upper bound of disturbance |d(t)| ≤ D
        """
        self.lambda_param = lambda_param
        self.K = K
        self.phi = phi
        self.a_min = a_min
        self.a_max = a_max
        self.disturbance_bound = disturbance_bound
        
        # Validate parameters
        if lambda_param <= 0:
            raise ValueError("λ must be positive")
        if K <= disturbance_bound:
            raise ValueError("K must be greater than disturbance bound for stability")
        if phi <= 0:
            raise ValueError("Φ must be positive")
            
        logger.info(f"SMC initialized with λ={lambda_param}, K={K}, Φ={phi}")
    
    def compute_control(self, position: float, velocity: float) -> Tuple[float, dict]:
        """
        Compute the control input (acceleration) based on current state.
        
        Args:
            position: Current position p(t)
            velocity: Current velocity v(t)
            
        Returns:
            Tuple of (control_input, debug_info)
        """
        # Compute sliding surface
        sliding_surface = position + self.lambda_param * velocity
        
        # Equivalent control (maintains sliding when no disturbance)
        a_eq = -(1.0 / self.lambda_param) * velocity
        
        # Switching control (robustness against disturbances)
        # Using tanh instead of sign function to reduce chattering
        a_sw = -self.K * np.tanh(sliding_surface / self.phi)
        
        # Total control input
        a_total = a_eq + a_sw
        
        # Apply actuator constraints
        a_cmd = np.clip(a_total, self.a_min, self.a_max)
        
        # Debug information
        debug_info = {
            'sliding_surface': sliding_surface,
            'a_equivalent': a_eq,
            'a_switching': a_sw,
            'a_total': a_total,
            'a_commanded': a_cmd,
            'saturated': a_cmd != a_total
        }
        
        return a_cmd, debug_info
    
    def lyapunov_function(self, sliding_surface: float) -> float:
        """
        Compute the Lyapunov function V(t) = 0.5 * s(t)^2.
        
        Args:
            sliding_surface: Current sliding surface value s(t)
            
        Returns:
            Lyapunov function value
        """
        return 0.5 * sliding_surface**2


class VehicleSystem:
    """
    Simple vehicle system with double integrator dynamics.
    
    System equations:
    ṗ(t) = v(t)
    v̇(t) = a(t) + d(t)
    """
    
    def __init__(self, 
                 initial_position: float = 10.0,
                 initial_velocity: float = 5.0,
                 disturbance_function: Optional[Callable[[float], float]] = None):
        """
        Initialize the vehicle system.
        
        Args:
            initial_position: Initial position p(0)
            initial_velocity: Initial velocity v(0)
            disturbance_function: Function d(t) returning disturbance at time t
        """
        self.position = initial_position
        self.velocity = initial_velocity
        self.initial_position = initial_position
        self.initial_velocity = initial_velocity
        self.disturbance_function = disturbance_function or (lambda t: 0.0)
        
    def update(self, acceleration: float, dt: float, time: float) -> None:
        """
        Update system state using Euler integration.
        
        Args:
            acceleration: Control input a(t)
            dt: Time step
            time: Current time (for disturbance calculation)
        """
        # Get disturbance
        disturbance = self.disturbance_function(time)
        
        # Update velocity: v̇(t) = a(t) + d(t)
        self.velocity += (acceleration + disturbance) * dt
        
        # Update position: ṗ(t) = v(t)
        self.position += self.velocity * dt
    
    def get_state(self) -> Tuple[float, float]:
        """Get current state (position, velocity)."""
        return self.position, self.velocity
    
    def reset(self) -> None:
        """Reset to initial conditions."""
        self.position = self.initial_position
        self.velocity = self.initial_velocity


class SMCSimulator:
    """
    Simulator for sliding mode control system.
    """
    
    def __init__(self, 
                 controller: SlidingModeController,
                 vehicle: VehicleSystem,
                 dt: float = 0.01,
                 max_time: float = 20.0,
                 convergence_threshold: float = 1e-3):
        """
        Initialize the simulator.
        
        Args:
            controller: SMC controller instance
            vehicle: Vehicle system instance
            dt: Simulation time step
            max_time: Maximum simulation time
            convergence_threshold: Threshold for convergence detection
        """
        self.controller = controller
        self.vehicle = vehicle
        self.dt = dt
        self.max_time = max_time
        self.convergence_threshold = convergence_threshold
        
    def run_simulation(self) -> dict:
        """
        Run the complete simulation.
        
        Returns:
            Dictionary containing simulation results
        """
        # Initialize storage
        time_history = []
        position_history = []
        velocity_history = []
        acceleration_history = []
        sliding_surface_history = []
        lyapunov_history = []
        
        # Reset vehicle to initial conditions
        self.vehicle.reset()
        
        # Simulation loop
        time = 0.0
        step = 0
        
        logger.info("Starting SMC simulation...")
        
        while time < self.max_time:
            # Get current state
            position, velocity = self.vehicle.get_state()
            
            # Check convergence
            if abs(position) < self.convergence_threshold and abs(velocity) < self.convergence_threshold:
                logger.info(f"Converged at t={time:.2f}s")
                break
            
            # Compute control
            acceleration, debug_info = self.controller.compute_control(position, velocity)
            
            # Update vehicle
            self.vehicle.update(acceleration, self.dt, time)
            
            # Store data
            time_history.append(time)
            position_history.append(position)
            velocity_history.append(velocity)
            acceleration_history.append(acceleration)
            sliding_surface_history.append(debug_info['sliding_surface'])
            lyapunov_history.append(self.controller.lyapunov_function(debug_info['sliding_surface']))
            
            # Update time
            time += self.dt
            step += 1
            
            # Progress logging
            if step % 100 == 0:
                logger.info(f"t={time:.2f}s, p={position:.3f}, v={velocity:.3f}, s={debug_info['sliding_surface']:.3f}")
        
        logger.info(f"Simulation completed in {step} steps")
        
        return {
            'time': np.array(time_history),
            'position': np.array(position_history),
            'velocity': np.array(velocity_history),
            'acceleration': np.array(acceleration_history),
            'sliding_surface': np.array(sliding_surface_history),
            'lyapunov': np.array(lyapunov_history),
            'converged': abs(position) < self.convergence_threshold and abs(velocity) < self.convergence_threshold,
            'final_time': time
        }


def create_disturbance_functions():
    """
    Create various disturbance functions for testing.
    
    Returns:
        Dictionary of disturbance functions
    """
    return {
        'none': lambda t: 0.0,
        'constant': lambda t: 0.5,
        'sinusoidal': lambda t: 0.8 * np.sin(2 * np.pi * 0.5 * t),
        'step': lambda t: 0.7 if 5 < t < 10 else 0.0,
        'random': lambda t: np.random.uniform(-0.5, 0.5)
    }


def plot_simulation_results(results: dict, title_suffix: str = "") -> None:
    """
    Plot comprehensive simulation results.
    
    Args:
        results: Simulation results dictionary
        title_suffix: Additional text for plot titles
    """
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    fig.suptitle(f'Sliding Mode Control Simulation Results{title_suffix}', fontsize=16)
    
    # Position vs Time
    axes[0, 0].plot(results['time'], results['position'], 'b-', linewidth=2)
    axes[0, 0].axhline(y=0, color='r', linestyle='--', alpha=0.7)
    axes[0, 0].set_xlabel('Time (s)')
    axes[0, 0].set_ylabel('Position (m)')
    axes[0, 0].set_title('Position vs Time')
    axes[0, 0].grid(True, alpha=0.3)
    
    # Velocity vs Time
    axes[0, 1].plot(results['time'], results['velocity'], 'g-', linewidth=2)
    axes[0, 1].axhline(y=0, color='r', linestyle='--', alpha=0.7)
    axes[0, 1].set_xlabel('Time (s)')
    axes[0, 1].set_ylabel('Velocity (m/s)')
    axes[0, 1].set_title('Velocity vs Time')
    axes[0, 1].grid(True, alpha=0.3)
    
    # Phase Portrait
    axes[0, 2].plot(results['position'], results['velocity'], 'purple', linewidth=2)
    axes[0, 2].plot(results['position'][0], results['velocity'][0], 'go', markersize=8, label='Start')
    axes[0, 2].plot(0, 0, 'ro', markersize=8, label='Target')
    axes[0, 2].set_xlabel('Position (m)')
    axes[0, 2].set_ylabel('Velocity (m/s)')
    axes[0, 2].set_title('Phase Portrait')
    axes[0, 2].legend()
    axes[0, 2].grid(True, alpha=0.3)
    
    # Control Input (Acceleration)
    axes[1, 0].plot(results['time'], results['acceleration'], 'orange', linewidth=2)
    axes[1, 0].set_xlabel('Time (s)')
    axes[1, 0].set_ylabel('Acceleration (m/s²)')
    axes[1, 0].set_title('Control Input')
    axes[1, 0].grid(True, alpha=0.3)
    
    # Sliding Surface
    axes[1, 1].plot(results['time'], results['sliding_surface'], 'red', linewidth=2)
    axes[1, 1].axhline(y=0, color='k', linestyle='--', alpha=0.7)
    axes[1, 1].set_xlabel('Time (s)')
    axes[1, 1].set_ylabel('Sliding Surface s(t)')
    axes[1, 1].set_title('Sliding Surface')
    axes[1, 1].grid(True, alpha=0.3)
    
    # Lyapunov Function
    axes[1, 2].semilogy(results['time'], results['lyapunov'], 'brown', linewidth=2)
    axes[1, 2].set_xlabel('Time (s)')
    axes[1, 2].set_ylabel('Lyapunov Function V(t)')
    axes[1, 2].set_title('Lyapunov Function (log scale)')
    axes[1, 2].grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()


def compare_parameters():
    """
    Compare different parameter settings for SMC.
    """
    # Different parameter sets
    param_sets = [
        {'lambda_param': 0.5, 'K': 2.0, 'phi': 0.1, 'name': 'λ=0.5, K=2.0'},
        {'lambda_param': 1.0, 'K': 2.0, 'phi': 0.1, 'name': 'λ=1.0, K=2.0'},
        {'lambda_param': 2.0, 'K': 2.0, 'phi': 0.1, 'name': 'λ=2.0, K=2.0'},
        {'lambda_param': 1.0, 'K': 1.5, 'phi': 0.1, 'name': 'λ=1.0, K=1.5'},
        {'lambda_param': 1.0, 'K': 3.0, 'phi': 0.1, 'name': 'λ=1.0, K=3.0'},
    ]
    
    plt.figure(figsize=(15, 10))
    
    for i, params in enumerate(param_sets):
        # Create controller and vehicle
        controller = SlidingModeController(
            lambda_param=params['lambda_param'],
            K=params['K'],
            phi=params['phi']
        )
        vehicle = VehicleSystem(initial_position=10.0, initial_velocity=5.0)
        simulator = SMCSimulator(controller, vehicle)
        
        # Run simulation
        results = simulator.run_simulation()
        
        # Plot position
        plt.subplot(2, 3, 1)
        plt.plot(results['time'], results['position'], label=params['name'])
        plt.xlabel('Time (s)')
        plt.ylabel('Position (m)')
        plt.title('Position Comparison')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Plot velocity
        plt.subplot(2, 3, 2)
        plt.plot(results['time'], results['velocity'], label=params['name'])
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity (m/s)')
        plt.title('Velocity Comparison')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Plot phase portrait
        plt.subplot(2, 3, 3)
        plt.plot(results['position'], results['velocity'], label=params['name'])
        plt.xlabel('Position (m)')
        plt.ylabel('Velocity (m/s)')
        plt.title('Phase Portrait Comparison')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Plot control input
        plt.subplot(2, 3, 4)
        plt.plot(results['time'], results['acceleration'], label=params['name'])
        plt.xlabel('Time (s)')
        plt.ylabel('Acceleration (m/s²)')
        plt.title('Control Input Comparison')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Plot sliding surface
        plt.subplot(2, 3, 5)
        plt.plot(results['time'], results['sliding_surface'], label=params['name'])
        plt.xlabel('Time (s)')
        plt.ylabel('Sliding Surface')
        plt.title('Sliding Surface Comparison')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Plot Lyapunov function
        plt.subplot(2, 3, 6)
        plt.semilogy(results['time'], results['lyapunov'], label=params['name'])
        plt.xlabel('Time (s)')
        plt.ylabel('Lyapunov Function')
        plt.title('Lyapunov Function Comparison')
        plt.legend()
        plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()


def main():
    """
    Main function to run SMC simulations and demonstrations.
    """
    print("=" * 80)
    print("SLIDING MODE CONTROL SIMULATION")
    print("=" * 80)
    print()
    
    # 1. Basic simulation without disturbance
    print("1. Basic SMC Simulation (No Disturbance)")
    print("-" * 40)
    
    controller = SlidingModeController(lambda_param=1.0, K=2.0, phi=0.1)
    vehicle = VehicleSystem(initial_position=10.0, initial_velocity=5.0)
    simulator = SMCSimulator(controller, vehicle)
    
    results = simulator.run_simulation()
    print(f"Converged: {results['converged']}")
    print(f"Final time: {results['final_time']:.2f}s")
    print(f"Final position: {results['position'][-1]:.6f}m")
    print(f"Final velocity: {results['velocity'][-1]:.6f}m/s")
    print()
    
    plot_simulation_results(results, " - No Disturbance")
    
    # 2. Simulation with sinusoidal disturbance
    print("2. SMC Simulation with Sinusoidal Disturbance")
    print("-" * 45)
    
    disturbance_funcs = create_disturbance_functions()
    vehicle_disturbed = VehicleSystem(
        initial_position=10.0, 
        initial_velocity=5.0,
        disturbance_function=disturbance_funcs['sinusoidal']
    )
    simulator_disturbed = SMCSimulator(controller, vehicle_disturbed)
    
    results_disturbed = simulator_disturbed.run_simulation()
    print(f"Converged: {results_disturbed['converged']}")
    print(f"Final time: {results_disturbed['final_time']:.2f}s")
    print(f"Final position: {results_disturbed['position'][-1]:.6f}m")
    print(f"Final velocity: {results_disturbed['velocity'][-1]:.6f}m/s")
    print()
    
    plot_simulation_results(results_disturbed, " - Sinusoidal Disturbance")
    
    # 3. Parameter comparison
    print("3. Parameter Comparison")
    print("-" * 20)
    compare_parameters()
    
    # 4. Robustness test with different disturbances
    print("4. Robustness Test with Different Disturbances")
    print("-" * 48)
    
    disturbance_funcs = create_disturbance_functions()
    
    plt.figure(figsize=(15, 8))
    
    for i, (name, dist_func) in enumerate(disturbance_funcs.items()):
        vehicle_test = VehicleSystem(
            initial_position=10.0,
            initial_velocity=5.0,
            disturbance_function=dist_func
        )
        simulator_test = SMCSimulator(controller, vehicle_test)
        results_test = simulator_test.run_simulation()
        
        plt.subplot(2, 3, i + 1)
        plt.plot(results_test['time'], results_test['position'], 'b-', label='Position')
        plt.plot(results_test['time'], results_test['velocity'], 'g-', label='Velocity')
        plt.axhline(y=0, color='r', linestyle='--', alpha=0.7)
        plt.xlabel('Time (s)')
        plt.ylabel('State')
        plt.title(f'Disturbance: {name}')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        print(f"  {name}: Converged={results_test['converged']}, "
              f"Time={results_test['final_time']:.2f}s")
    
    plt.tight_layout()
    plt.show()
    
    print("\nSimulation completed successfully!")


if __name__ == "__main__":
    main() 