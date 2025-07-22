"""
Visualization tools for vehicle dynamics simulation
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from typing import Optional, Tuple, Callable
from matplotlib.figure import Figure
from matplotlib.axes import Axes
import vehicle_dynamics


def plot_vehicle_trajectory(sim: vehicle_dynamics.VehicleSimulation,
                          ax: Optional[Axes] = None,
                          show_vehicle: bool = True,
                          vehicle_scale: float = 1.0) -> Figure:
    """
    Plot vehicle trajectory with optional vehicle representation at intervals
    
    Args:
        sim: Vehicle simulation instance
        ax: Matplotlib axes to plot on (if None, creates new figure)
        show_vehicle: Whether to show vehicle representation along trajectory
        vehicle_scale: Scale factor for vehicle representation
        
    Returns:
        Matplotlib figure
    """
    if ax is None:
        fig, ax = plt.subplots(1, 1, figsize=(12, 8))
    else:
        fig = ax.get_figure()
    
    # Plot trajectory
    ax.plot(sim.history['x'], sim.history['y'], 'b-', linewidth=2, label='Trajectory')
    ax.plot(sim.history['x'][0], sim.history['y'][0], 'go', markersize=8, label='Start')
    ax.plot(sim.history['x'][-1], sim.history['y'][-1], 'ro', markersize=8, label='End')
    
    # Add vehicle representations at intervals
    if show_vehicle:
        params = sim.params
        interval = max(1, len(sim.history['x']) // 20)  # Show at most 20 vehicles
        
        for i in range(0, len(sim.history['x']), interval):
            x = sim.history['x'][i]
            y = sim.history['y'][i]
            yaw = sim.history['yaw'][i]
            steer = sim.history['steer'][i]
            
            # Vehicle body
            vehicle_length = params.length * vehicle_scale
            vehicle_width = params.width * vehicle_scale
            
            # Calculate rear axle position (center of rotation)
            rear_axle_x = x - (params.wheelbase / 2) * np.cos(yaw)
            rear_axle_y = y - (params.wheelbase / 2) * np.sin(yaw)
            
            # Create rectangle for vehicle body
            bottom_left_x = rear_axle_x - (vehicle_length / 2) * np.cos(yaw) + (vehicle_width / 2) * np.sin(yaw)
            bottom_left_y = rear_axle_y - (vehicle_length / 2) * np.sin(yaw) - (vehicle_width / 2) * np.cos(yaw)
            
            vehicle_rect = Rectangle((bottom_left_x, bottom_left_y), 
                                   vehicle_length, vehicle_width,
                                   angle=np.degrees(yaw), 
                                   fill=True, facecolor='red', 
                                   edgecolor='black', alpha=0.7,
                                   zorder=10)
            ax.add_patch(vehicle_rect)
            
            # Front wheels (show steering angle)
            wheel_length = 0.2 * vehicle_scale
            wheel_width = 0.1 * vehicle_scale
            
            # Front left wheel
            front_left_x = x + (params.wheelbase / 2) * np.cos(yaw) - (vehicle_width / 2) * np.sin(yaw)
            front_left_y = y + (params.wheelbase / 2) * np.sin(yaw) + (vehicle_width / 2) * np.cos(yaw)
            
            wheel_rect = Rectangle((front_left_x - wheel_length/2, front_left_y - wheel_width/2),
                                 wheel_length, wheel_width,
                                 angle=np.degrees(yaw + steer),
                                 fill=True, facecolor='black',
                                 edgecolor='black', zorder=11)
            ax.add_patch(wheel_rect)
            
            # Front right wheel
            front_right_x = x + (params.wheelbase / 2) * np.cos(yaw) + (vehicle_width / 2) * np.sin(yaw)
            front_right_y = y + (params.wheelbase / 2) * np.sin(yaw) - (vehicle_width / 2) * np.cos(yaw)
            
            wheel_rect = Rectangle((front_right_x - wheel_length/2, front_right_y - wheel_width/2),
                                 wheel_length, wheel_width,
                                 angle=np.degrees(yaw + steer),
                                 fill=True, facecolor='black',
                                 edgecolor='black', zorder=11)
            ax.add_patch(wheel_rect)
    
    ax.set_xlabel('X Position [m]')
    ax.set_ylabel('Y Position [m]')
    ax.set_title('Vehicle Trajectory')
    ax.legend()
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    if fig is None:
        fig = plt.gcf()
    # Ensure we return a Figure instance
    if hasattr(fig, 'get_figure'):
        fig = fig.get_figure()
    return fig  # type: ignore


def plot_state_variables(sim: vehicle_dynamics.VehicleSimulation) -> Figure:
    """
    Plot vehicle state variables over time
    
    Args:
        sim: Vehicle simulation instance
        
    Returns:
        Matplotlib figure
    """
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
    
    t = sim.history['t']
    
    # Velocity
    ax1.plot(t, sim.history['v'], 'b-', linewidth=2)
    ax1.set_xlabel('Time [s]')
    ax1.set_ylabel('Velocity [m/s]')
    ax1.set_title('Velocity vs Time')
    ax1.grid(True, alpha=0.3)
    
    # Steering angle
    ax2.plot(t, np.degrees(sim.history['steer']), 'r-', linewidth=2)
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Steering Angle [deg]')
    ax2.set_title('Steering Angle vs Time')
    ax2.grid(True, alpha=0.3)
    
    # Yaw angle
    ax3.plot(t, np.degrees(sim.history['yaw']), 'g-', linewidth=2)
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Yaw Angle [deg]')
    ax3.set_title('Yaw Angle vs Time')
    ax3.grid(True, alpha=0.3)
    
    # Acceleration (from control inputs)
    ax4.plot(t, sim.history['a'], 'm-', linewidth=2)
    ax4.set_xlabel('Time [s]')
    ax4.set_ylabel('Acceleration [m/s²]')
    ax4.set_title('Acceleration vs Time')
    ax4.grid(True, alpha=0.3)
    
    plt.tight_layout()
    return fig


def animate_vehicle_simulation(sim: vehicle_dynamics.VehicleSimulation,
                              interval: int = 50) -> Tuple[Figure, Axes, Callable]:
    """
    Create animation of vehicle simulation
    
    Args:
        sim: Vehicle simulation instance
        interval: Animation interval in milliseconds
        
    Returns:
        Tuple of (figure, axes)
    """
    fig, ax = plt.subplots(1, 1, figsize=(12, 8))
    
    # Plot full trajectory as reference
    ax.plot(sim.history['x'], sim.history['y'], 'b-', alpha=0.3, linewidth=1)
    ax.plot(sim.history['x'][0], sim.history['y'][0], 'go', markersize=8)
    ax.plot(sim.history['x'][-1], sim.history['y'][-1], 'ro', markersize=8)
    
    # Initialize vehicle representation
    params = sim.params
    vehicle_length = params.length
    vehicle_width = params.width
    
    # Vehicle body
    vehicle_rect = Rectangle((0, 0), vehicle_length, vehicle_width,
                           angle=0, fill=True, facecolor='red', 
                           edgecolor='black', alpha=0.7)
    ax.add_patch(vehicle_rect)
    
    # Front wheels
    wheel_length = 0.2
    wheel_width = 0.1
    
    front_left_wheel = Rectangle((0, 0), wheel_length, wheel_width,
                               angle=0, fill=True, facecolor='black',
                               edgecolor='black')
    front_right_wheel = Rectangle((0, 0), wheel_length, wheel_width,
                                angle=0, fill=True, facecolor='black',
                                edgecolor='black')
    ax.add_patch(front_left_wheel)
    ax.add_patch(front_right_wheel)
    
    # Text for time and speed
    time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes, fontsize=12,
                       bbox=dict(boxstyle="round", fc="w"))
    
    vehicle_patches = []
    
    def update(frame):
        # Remove previous vehicle representation
        for patch in vehicle_patches:
            patch.remove()
        vehicle_patches.clear()
        
        # Get current state
        x = sim.history['x'][frame]
        y = sim.history['y'][frame]
        yaw = sim.history['yaw'][frame]
        steer = sim.history['steer'][frame]
        t = sim.history['t'][frame]
        v = sim.history['v'][frame]
        
        # Update vehicle body
        rear_axle_x = x - (params.wheelbase / 2) * np.cos(yaw)
        rear_axle_y = y - (params.wheelbase / 2) * np.sin(yaw)
        
        bottom_left_x = rear_axle_x - (vehicle_length / 2) * np.cos(yaw) + (vehicle_width / 2) * np.sin(yaw)
        bottom_left_y = rear_axle_y - (vehicle_length / 2) * np.sin(yaw) - (vehicle_width / 2) * np.cos(yaw)
        
        vehicle_rect = Rectangle((bottom_left_x, bottom_left_y),
                               vehicle_length, vehicle_width,
                               angle=np.degrees(yaw),
                               fill=True, facecolor='red',
                               edgecolor='black', alpha=0.7,
                               zorder=10)
        ax.add_patch(vehicle_rect)
        vehicle_patches.append(vehicle_rect)
        
        # Update front wheels
        front_left_x = x + (params.wheelbase / 2) * np.cos(yaw) - (vehicle_width / 2) * np.sin(yaw)
        front_left_y = y + (params.wheelbase / 2) * np.sin(yaw) + (vehicle_width / 2) * np.cos(yaw)
        
        front_left_wheel = Rectangle((front_left_x - wheel_length/2, front_left_y - wheel_width/2),
                                   wheel_length, wheel_width,
                                   angle=np.degrees(yaw + steer),
                                   fill=True, facecolor='black',
                                   edgecolor='black', zorder=11)
        ax.add_patch(front_left_wheel)
        vehicle_patches.append(front_left_wheel)
        
        front_right_x = x + (params.wheelbase / 2) * np.cos(yaw) + (vehicle_width / 2) * np.sin(yaw)
        front_right_y = y + (params.wheelbase / 2) * np.sin(yaw) - (vehicle_width / 2) * np.cos(yaw)
        
        front_right_wheel = Rectangle((front_right_x - wheel_length/2, front_right_y - wheel_width/2),
                                     wheel_length, wheel_width,
                                     angle=np.degrees(yaw + steer),
                                     fill=True, facecolor='black',
                                     edgecolor='black', zorder=11)
        ax.add_patch(front_right_wheel)
        vehicle_patches.append(front_right_wheel)
        
        # Update text
        time_text.set_text(f'Time: {t:.1f}s\nSpeed: {v:.1f} m/s')
        
        return [vehicle_rect, front_left_wheel, front_right_wheel, time_text]
    
    ax.set_xlim(min(sim.history['x']) - 5, max(sim.history['x']) + 5)
    ax.set_ylim(min(sim.history['y']) - 5, max(sim.history['y']) + 5)
    ax.set_xlabel('X Position [m]')
    ax.set_ylabel('Y Position [m]')
    ax.set_title('Vehicle Simulation Animation')
    ax.grid(True, alpha=0.3)
    ax.axis('equal')
    
    return fig, ax, update