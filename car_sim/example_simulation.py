"""
Example usage of the vehicle dynamics simulation
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import sys
import os

# Add the car_sim directory to the path so we can import our modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from vehicle_dynamics import VehicleSimulation, VehicleParameters, simulate_trajectory
from visualization import plot_vehicle_trajectory, plot_state_variables, animate_vehicle_simulation


def sine_wave_control(t: float, state) -> tuple:
    """
    Control function that creates a sine wave trajectory
    """
    # Acceleration profile: accelerate to target speed, then maintain
    target_speed = 10.0  # m/s
    acceleration = 1.0 if state.v < target_speed else 0.0
    
    # Steering profile: create sine wave pattern
    steering_rate = 0.5 * np.sin(0.5 * t)  # Vary steering rate with time
    
    return acceleration, steering_rate


def step_response_control(t: float, state) -> tuple:
    """
    Control function for steering step response
    """
    # Constant acceleration
    acceleration = 1.0
    
    # Step change in steering rate at t=5s
    if t < 5.0:
        steering_rate = 0.0
    else:
        steering_rate = 0.3
    
    return acceleration, steering_rate


def figure_eight_control(t: float, state) -> tuple:
    """
    Control function for figure-eight trajectory
    """
    # Constant speed
    target_speed = 8.0
    acceleration = 0.5 if state.v < target_speed else 0.0
    
    # Create figure-eight pattern with steering
    steering_rate = 0.4 * np.sin(0.8 * t)
    
    return acceleration, steering_rate


def main():
    """Main function to run different simulation examples"""
    print("Vehicle Dynamics Simulation Examples")
    print("=" * 40)
    
    # Example 1: Sine wave trajectory
    print("\n1. Sine Wave Trajectory")
    params = VehicleParameters()
    sim1 = VehicleSimulation(params)
    sim1 = simulate_trajectory(sim1, 20.0, sine_wave_control)
    
    # Plot results
    fig1 = plot_vehicle_trajectory(sim1)
    fig2 = plot_state_variables(sim1)
    fig2.suptitle('Sine Wave Trajectory - State Variables')
    
    # Example 2: Step response
    print("\n2. Steering Step Response")
    sim2 = VehicleSimulation(params)
    sim2 = simulate_trajectory(sim2, 15.0, step_response_control)
    
    fig3 = plot_vehicle_trajectory(sim2)
    fig3.suptitle('Steering Step Response')
    fig4 = plot_state_variables(sim2)
    fig4.suptitle('Steering Step Response - State Variables')
    
    # Example 3: Figure-eight
    print("\n3. Figure-Eight Trajectory")
    sim3 = VehicleSimulation(params)
    sim3 = simulate_trajectory(sim3, 25.0, figure_eight_control)
    
    fig5 = plot_vehicle_trajectory(sim3)
    fig5.suptitle('Figure-Eight Trajectory')
    fig6 = plot_state_variables(sim3)
    fig6.suptitle('Figure-Eight Trajectory - State Variables')
    
    # Example 4: Animation
    print("\n4. Creating Animation...")
    fig7, ax7, update_fn = animate_vehicle_simulation(sim3, interval=50)
    
    # Create animation
    anim = FuncAnimation(fig7, update_fn, frames=len(sim3.history['t']),
                        interval=50, blit=False, repeat=True)
    
    plt.show()
    
    print("\nSimulation examples completed!")


if __name__ == "__main__":
    main()