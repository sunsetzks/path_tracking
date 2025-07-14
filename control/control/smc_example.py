"""
Simple example demonstrating Sliding Mode Control (SMC) for vehicle control to origin.

This script shows how to use the SMC implementation with different scenarios:
1. Basic control without disturbance
2. Control with sinusoidal disturbance
3. Parameter comparison

Run this script to see the SMC in action.
"""

import numpy as np
import matplotlib.pyplot as plt
from .sliding_mode_control import SlidingModeController, VehicleSystem, SMCSimulator


def basic_smc_example():
    """
    Basic SMC example without disturbance.
    """
    print("Running basic SMC example...")
    
    # Create controller with default parameters
    controller = SlidingModeController(
        lambda_param=1.0,  # Sliding surface slope
        K=2.0,            # Switching gain
        phi=0.1           # Boundary layer thickness
    )
    
    # Create vehicle system with initial conditions
    vehicle = VehicleSystem(
        initial_position=10.0,  # Start at 10m
        initial_velocity=5.0    # Start with 5m/s velocity
    )
    
    # Create and run simulator
    simulator = SMCSimulator(controller, vehicle)
    results = simulator.run_simulation()
    
    # Print results
    print(f"Converged: {results['converged']}")
    print(f"Final time: {results['final_time']:.2f}s")
    print(f"Final position: {results['position'][-1]:.6f}m")
    print(f"Final velocity: {results['velocity'][-1]:.6f}m/s")
    
    # Plot results
    plt.figure(figsize=(12, 8))
    
    # Position and velocity
    plt.subplot(2, 2, 1)
    plt.plot(results['time'], results['position'], 'b-', linewidth=2, label='Position')
    plt.plot(results['time'], results['velocity'], 'g-', linewidth=2, label='Velocity')
    plt.axhline(y=0, color='r', linestyle='--', alpha=0.7, label='Target')
    plt.xlabel('Time (s)')
    plt.ylabel('State')
    plt.title('Position and Velocity vs Time')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Phase portrait
    plt.subplot(2, 2, 2)
    plt.plot(results['position'], results['velocity'], 'purple', linewidth=2)
    plt.plot(results['position'][0], results['velocity'][0], 'go', markersize=8, label='Start')
    plt.plot(0, 0, 'ro', markersize=8, label='Target')
    plt.xlabel('Position (m)')
    plt.ylabel('Velocity (m/s)')
    plt.title('Phase Portrait')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Control input
    plt.subplot(2, 2, 3)
    plt.plot(results['time'], results['acceleration'], 'orange', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (m/s²)')
    plt.title('Control Input')
    plt.grid(True, alpha=0.3)
    
    # Sliding surface
    plt.subplot(2, 2, 4)
    plt.plot(results['time'], results['sliding_surface'], 'red', linewidth=2)
    plt.axhline(y=0, color='k', linestyle='--', alpha=0.7)
    plt.xlabel('Time (s)')
    plt.ylabel('Sliding Surface s(t)')
    plt.title('Sliding Surface')
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    return results


def disturbed_smc_example():
    """
    SMC example with sinusoidal disturbance.
    """
    print("\nRunning SMC example with sinusoidal disturbance...")
    
    # Create controller
    controller = SlidingModeController(
        lambda_param=1.0,
        K=2.5,  # Slightly higher gain to handle disturbance
        phi=0.1
    )
    
    # Create vehicle with sinusoidal disturbance
    disturbance_func = lambda t: 0.8 * np.sin(2 * np.pi * 0.5 * t)
    vehicle = VehicleSystem(
        initial_position=10.0,
        initial_velocity=5.0,
        disturbance_function=disturbance_func
    )
    
    # Run simulation
    simulator = SMCSimulator(controller, vehicle)
    results = simulator.run_simulation()
    
    print(f"Converged: {results['converged']}")
    print(f"Final time: {results['final_time']:.2f}s")
    print(f"Final position: {results['position'][-1]:.6f}m")
    print(f"Final velocity: {results['velocity'][-1]:.6f}m/s")
    
    # Plot disturbance effect
    plt.figure(figsize=(12, 6))
    
    plt.subplot(1, 2, 1)
    plt.plot(results['time'], results['position'], 'b-', linewidth=2, label='Position')
    plt.plot(results['time'], results['velocity'], 'g-', linewidth=2, label='Velocity')
    plt.axhline(y=0, color='r', linestyle='--', alpha=0.7, label='Target')
    plt.xlabel('Time (s)')
    plt.ylabel('State')
    plt.title('States with Sinusoidal Disturbance')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.subplot(1, 2, 2)
    # Plot disturbance
    time_dist = np.linspace(0, results['final_time'], len(results['time']))
    disturbance_values = [disturbance_func(t) for t in time_dist]
    plt.plot(time_dist, disturbance_values, 'r--', linewidth=2, label='Disturbance')
    plt.plot(results['time'], results['acceleration'], 'orange', linewidth=2, label='Control Input')
    plt.xlabel('Time (s)')
    plt.ylabel('Acceleration (m/s²)')
    plt.title('Control Input vs Disturbance')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show()
    
    return results


def parameter_comparison_example():
    """
    Compare different parameter settings.
    """
    print("\nRunning parameter comparison example...")
    
    # Different lambda values
    lambda_values = [0.5, 1.0, 2.0]
    
    plt.figure(figsize=(15, 5))
    
    for i, lambda_val in enumerate(lambda_values):
        controller = SlidingModeController(
            lambda_param=lambda_val,
            K=2.0,
            phi=0.1
        )
        
        vehicle = VehicleSystem(initial_position=10.0, initial_velocity=5.0)
        simulator = SMCSimulator(controller, vehicle)
        results = simulator.run_simulation()
        
        plt.subplot(1, 3, i + 1)
        plt.plot(results['position'], results['velocity'], 'purple', linewidth=2)
        plt.plot(results['position'][0], results['velocity'][0], 'go', markersize=8, label='Start')
        plt.plot(0, 0, 'ro', markersize=8, label='Target')
        
        # Plot sliding surface line
        pos_range = np.linspace(min(results['position']), max(results['position']), 100)
        sliding_line = -pos_range / lambda_val
        plt.plot(pos_range, sliding_line, 'k--', alpha=0.5, label=f'Sliding line (λ={lambda_val})')
        
        plt.xlabel('Position (m)')
        plt.ylabel('Velocity (m/s)')
        plt.title(f'Phase Portrait (λ={lambda_val})')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        print(f"λ={lambda_val}: Converged={results['converged']}, Time={results['final_time']:.2f}s")
    
    plt.tight_layout()
    plt.show()


def main():
    """
    Run all SMC examples.
    """
    print("=" * 60)
    print("SLIDING MODE CONTROL EXAMPLES")
    print("=" * 60)
    
    # Run examples
    basic_smc_example()
    disturbed_smc_example()
    parameter_comparison_example()
    
    print("\n" + "=" * 60)
    print("All examples completed successfully!")
    print("=" * 60)


if __name__ == "__main__":
    main() 