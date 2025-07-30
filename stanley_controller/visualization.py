"""
Visualization Module

Comprehensive visualization tools for Stanley controller simulation results,
including real-time plotting, trajectory analysis, and performance metrics.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle
from typing import List, Tuple, Optional, Dict, Any
import math

from .utils.se2 import SE2
from .utils.angle import angle_mod
from .vehicle_dynamics import VehicleParameters


class Visualizer:
    """
    Comprehensive visualization class for Stanley controller simulations.
    
    Features:
    - Real-time animation
    - Trajectory plotting
    - Vehicle visualization
    - Obstacle display
    - Performance metrics
    - Multi-panel plots
    """
    
    def __init__(self, figsize: Tuple[int, int] = (15, 10)):
        """
        Initialize visualizer.
        
        Args:
            figsize: Figure size (width, height)
        """
        self.figsize = figsize
        self.colors = {
            'trajectory': 'blue',
            'reference': 'red',
            'obstacle': 'gray',
            'vehicle': 'black',
            'target': 'green',
            'collision': 'red'
        }
        
    def plot_trajectory_comparison(self, 
                                 results: List[Dict[str, Any]], 
                                 scenarios: List[str],
                                 save_path: Optional[str] = None):
        """
        Plot trajectory comparison across different scenarios.
        
        Args:
            results: List of simulation results
            scenarios: List of scenario names
            save_path: Optional path to save the plot
        """
        fig, axes = plt.subplots(2, 3, figsize=self.figsize)
        fig.suptitle('Stanley Controller Performance Across Scenarios', fontsize=16, fontweight='bold')
        
        for i, scenario in enumerate(scenarios):
            if i >= 6:  # Limit to 6 subplots
                break
                
            ax = axes[i // 3, i % 3]
            
            # Filter results for this scenario
            scenario_results = [r for r in results if r['scenario'] == scenario]
            
            if not scenario_results:
                continue
                
            # Plot reference path
            ref_result = scenario_results[0]
            if 'reference_path' in ref_result:
                path_points = ref_result['reference_path']
                ax.plot(path_points[:, 0], path_points[:, 1], 
                       '--', color=self.colors['reference'], linewidth=2, 
                       label='Reference Path', alpha=0.7)
            
            # Plot trajectories
            for j, result in enumerate(scenario_results[:3]):  # Show max 3 trajectories
                if 'trajectory' in result:
                    traj = result['trajectory']
                    alpha = 0.8 - j * 0.2
                    ax.plot(traj[:, 0], traj[:, 1], 
                           color=self.colors['trajectory'], linewidth=2, 
                           alpha=alpha, label=f'Run {j+1}')
            
            # Plot obstacles if any
            if 'obstacles' in ref_result:
                for obs in ref_result['obstacles']:
                    circle = Circle((obs['x'], obs['y']), obs['radius'],
                                  color=self.colors['obstacle'], alpha=0.5)
                    ax.add_patch(circle)
            
            ax.set_title(f'{scenario}\nSuccess: {sum(1 for r in scenario_results if r["success"])}/{len(scenario_results)}')
            ax.axis('equal')
            ax.grid(True, alpha=0.3)
            ax.legend()
            
            # Set axis limits
            if 'trajectory' in ref_result:
                traj = ref_result['trajectory']
                margin = 5
                ax.set_xlim(np.min(traj[:, 0]) - margin, np.max(traj[:, 0]) + margin)
                ax.set_ylim(np.min(traj[:, 1]) - margin, np.max(traj[:, 1]) + margin)
        
        # Remove empty subplots
        for i in range(len(scenarios), 6):
            fig.delaxes(axes[i // 3, i % 3])
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        
        plt.show()
    
    def plot_performance_metrics(self, results: List[Dict[str, Any]], save_path: Optional[str] = None):
        """
        Plot performance metrics across different scenarios.
        
        Args:
            results: List of simulation results
            save_path: Optional path to save the plot
        """
        fig, axes = plt.subplots(2, 2, figsize=self.figsize)
        fig.suptitle('Stanley Controller Performance Metrics', fontsize=16, fontweight='bold')
        
        # Extract metrics
        scenarios = list(set(r['scenario'] for r in results))
        success_rates = []
        avg_errors = []
        avg_times = []
        collision_rates = []
        
        for scenario in scenarios:
            scenario_results = [r for r in results if r['scenario'] == scenario]
            success_rates.append(sum(1 for r in scenario_results if r['success']) / len(scenario_results))
            avg_errors.append(np.mean([r['metrics']['average_distance_error'] for r in scenario_results]))
            avg_times.append(np.mean([r['time'] for r in scenario_results]))
            collision_rates.append(sum(1 for r in scenario_results if r['collision']) / len(scenario_results))
        
        # Success rate
        axes[0, 0].bar(scenarios, success_rates, color='green', alpha=0.7)
        axes[0, 0].set_title('Success Rate')
        axes[0, 0].set_ylabel('Success Rate')
        axes[0, 0].set_ylim(0, 1)
        axes[0, 0].tick_params(axis='x', rotation=45)
        
        # Average tracking error
        axes[0, 1].bar(scenarios, avg_errors, color='orange', alpha=0.7)
        axes[0, 1].set_title('Average Tracking Error')
        axes[0, 1].set_ylabel('Error (m)')
        axes[0, 1].tick_params(axis='x', rotation=45)
        
        # Average simulation time
        axes[1, 0].bar(scenarios, avg_times, color='blue', alpha=0.7)
        axes[1, 0].set_title('Average Simulation Time')
        axes[1, 0].set_ylabel('Time (s)')
        axes[1, 0].tick_params(axis='x', rotation=45)
        
        # Collision rate
        axes[1, 1].bar(scenarios, collision_rates, color='red', alpha=0.7)
        axes[1, 1].set_title('Collision Rate')
        axes[1, 1].set_ylabel('Collision Rate')
        axes[1, 1].set_ylim(0, 1)
        axes[1, 1].tick_params(axis='x', rotation=45)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        
        plt.show()
    
    def animate_simulation(self, 
                          result: Dict[str, Any], 
                          reference_path: Optional[np.ndarray] = None,
                          save_path: Optional[str] = None,
                          interval: int = 50):
        """
        Create an animated visualization of a simulation.
        
        Args:
            result: Simulation result dictionary
            reference_path: Optional reference path
            save_path: Optional path to save the animation
            interval: Animation interval in milliseconds
        """
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=self.figsize)
        fig.suptitle('Stanley Controller Simulation Animation', fontsize=16, fontweight='bold')
        
        # Extract trajectory
        trajectory = result['trajectory']
        controls = result['controls']
        
        # Setup main plot (trajectory)
        ax1.set_title('Vehicle Trajectory')
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # Plot reference path if available
        if reference_path is not None:
            ax1.plot(reference_path[:, 0], reference_path[:, 1], 
                    '--', color=self.colors['reference'], linewidth=2, 
                    label='Reference Path', alpha=0.7)
        
        # Plot obstacles if available
        if 'obstacles' in result:
            for obs in result['obstacles']:
                circle = Circle((obs['x'], obs['y']), obs['radius'],
                              color=self.colors['obstacle'], alpha=0.5)
                ax1.add_patch(circle)
        
        # Initialize trajectory line
        traj_line, = ax1.plot([], [], color=self.colors['trajectory'], linewidth=2, label='Trajectory')
        
        # Initialize vehicle
        vehicle_patch = patches.Rectangle((0, 0), 4.5, 2.0, angle=0, 
                                        facecolor=self.colors['vehicle'], alpha=0.7)
        ax1.add_patch(vehicle_patch)
        
        # Setup control plot
        ax2.set_title('Control Inputs')
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Value')
        ax2.grid(True, alpha=0.3)
        
        # Initialize control lines
        time_data = [c['time'] for c in controls]
        steering_data = [c['steering'] for c in controls]
        accel_data = [c['acceleration'] for c in controls]
        
        steering_line, = ax2.plot([], [], 'b-', label='Steering (rad)')
        accel_line, = ax2.plot([], [], 'r-', label='Acceleration (m/s²)')
        
        ax2.legend()
        ax2.set_xlim(0, max(time_data) if time_data else 1)
        ax2.set_ylim(min(min(steering_data), min(accel_data)) - 0.5, 
                    max(max(steering_data), max(accel_data)) + 0.5)
        
        def animate(frame):
            if frame >= len(trajectory):
                return traj_line, vehicle_patch, steering_line, accel_line
            
            # Update trajectory
            traj_line.set_data(trajectory[:frame+1, 0], trajectory[:frame+1, 1])
            
            # Update vehicle
            x, y, theta = trajectory[frame]
            vehicle_patch.set_xy((x - 2.25, y - 1.0))  # Center the rectangle
            vehicle_patch.angle = np.degrees(theta)
            
            # Update control plots
            current_time = time_data[:frame+1]
            steering_line.set_data(current_time, steering_data[:frame+1])
            accel_line.set_data(current_time, accel_data[:frame+1])
            
            return traj_line, vehicle_patch, steering_line, accel_line
        
        # Create animation
        anim = FuncAnimation(fig, animate, frames=len(trajectory), 
                           interval=interval, blit=True, repeat=True)
        
        # Add legend to main plot
        ax1.legend()
        
        plt.tight_layout()
        
        if save_path:
            anim.save(save_path, writer='pillow', fps=20)
        
        plt.show()
        
        return anim
    
    def plot_vehicle_animation_frame(self, ax, x: float, y: float, yaw: float, 
                                   steering: float = 0.0, scale: float = 1.0):
        """
        Plot a single vehicle frame for animation.
        
        Args:
            ax: Matplotlib axes
            x: Vehicle x position
            y: Vehicle y position
            yaw: Vehicle yaw angle
            steering: Steering angle
            scale: Scale factor for vehicle size
        """
        # Vehicle dimensions
        length = 4.5 * scale
        width = 2.0 * scale
        wheelbase = 2.9 * scale
        
        # Vehicle corners in vehicle frame
        corners = np.array([
            [-length/2, -width/2],
            [length/2, -width/2],
            [length/2, width/2],
            [-length/2, width/2],
            [-length/2, -width/2]
        ])
        
        # Rotation matrix
        cos_yaw = np.cos(yaw)
        sin_yaw = np.sin(yaw)
        rotation_matrix = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
        
        # Transform to world frame
        world_corners = corners @ rotation_matrix.T
        world_corners[:, 0] += x
        world_corners[:, 1] += y
        
        # Plot vehicle body
        vehicle_patch = patches.Polygon(world_corners[:-1], 
                                      facecolor='blue', alpha=0.7, edgecolor='black')
        ax.add_patch(vehicle_patch)
        
        # Plot wheels
        wheel_length = 0.4 * scale
        wheel_width = 0.2 * scale
        
        # Front wheels (with steering)
        front_wheel_positions = [
            [wheelbase/2, -width/4],
            [wheelbase/2, width/4]
        ]
        
        for wheel_pos in front_wheel_positions:
            wheel_corners = np.array([
                [-wheel_length/2, -wheel_width/2],
                [wheel_length/2, -wheel_width/2],
                [wheel_length/2, wheel_width/2],
                [-wheel_length/2, wheel_width/2],
                [-wheel_length/2, -wheel_width/2]
            ])
            
            # Apply steering rotation
            cos_steer = np.cos(steering)
            sin_steer = np.sin(steering)
            steer_rotation = np.array([[cos_steer, -sin_steer], [sin_steer, cos_steer]])
            
            # Apply vehicle rotation
            wheel_corners = wheel_corners @ steer_rotation.T @ rotation_matrix.T
            wheel_corners[:, 0] += x + wheel_pos[0] * cos_yaw - wheel_pos[1] * sin_yaw
            wheel_corners[:, 1] += y + wheel_pos[0] * sin_yaw + wheel_pos[1] * cos_yaw
            
            wheel_patch = patches.Polygon(wheel_corners[:-1], 
                                        facecolor='red', alpha=0.8)
            ax.add_patch(wheel_patch)
        
        # Rear wheels
        rear_wheel_positions = [
            [-wheelbase/2, -width/4],
            [-wheelbase/2, width/4]
        ]
        
        for wheel_pos in rear_wheel_positions:
            wheel_corners = np.array([
                [-wheel_length/2, -wheel_width/2],
                [wheel_length/2, -wheel_width/2],
                [wheel_length/2, wheel_width/2],
                [-wheel_length/2, wheel_width/2],
                [-wheel_length/2, -wheel_width/2]
            ])
            
            # Apply vehicle rotation only
            wheel_corners = wheel_corners @ rotation_matrix.T
            wheel_corners[:, 0] += x + wheel_pos[0] * cos_yaw - wheel_pos[1] * sin_yaw
            wheel_corners[:, 1] += y + wheel_pos[0] * sin_yaw + wheel_pos[1] * cos_yaw
            
            wheel_patch = patches.Polygon(wheel_corners[:-1], 
                                        facecolor='black', alpha=0.8)
            ax.add_patch(wheel_patch)
    
    def plot_error_analysis(self, result: Dict[str, Any], save_path: Optional[str] = None):
        """
        Plot detailed error analysis.
        
        Args:
            result: Simulation result dictionary
            save_path: Optional path to save the plot
        """
        if 'errors' not in result or not result['errors']:
            print("No error data available for analysis")
            return
        
        fig, axes = plt.subplots(2, 2, figsize=self.figsize)
        fig.suptitle('Error Analysis', fontsize=16, fontweight='bold')
        
        errors = result['errors']
        times = [e['time'] for e in errors]
        distance_errors = [e['distance_error'] for e in errors]
        heading_errors = [e['heading_error'] for e in errors]
        
        # Distance error over time
        axes[0, 0].plot(times, distance_errors, 'b-', linewidth=2)
        axes[0, 0].set_title('Distance Error vs Time')
        axes[0, 0].set_xlabel('Time (s)')
        axes[0, 0].set_ylabel('Distance Error (m)')
        axes[0, 0].grid(True, alpha=0.3)
        
        # Heading error over time
        axes[0, 1].plot(times, heading_errors, 'r-', linewidth=2)
        axes[0, 1].set_title('Heading Error vs Time')
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Heading Error (rad)')
        axes[0, 1].grid(True, alpha=0.3)
        
        # Error histogram
        axes[1, 0].hist(distance_errors, bins=30, alpha=0.7, color='blue', edgecolor='black')
        axes[1, 0].set_title('Distance Error Distribution')
        axes[1, 0].set_xlabel('Distance Error (m)')
        axes[1, 0].set_ylabel('Frequency')
        axes[1, 0].grid(True, alpha=0.3)
        
        # Control inputs
        if 'controls' in result:
            controls = result['controls']
            control_times = [c['time'] for c in controls]
            steerings = [c['steering'] for c in controls]
            accelerations = [c['acceleration'] for c in controls]
            
            # Steering angle
            ax_twin = axes[1, 1].twinx()
            line1 = axes[1, 1].plot(control_times, steerings, 'b-', linewidth=2, label='Steering')
            line2 = ax_twin.plot(control_times, accelerations, 'r-', linewidth=2, label='Acceleration')
            
            axes[1, 1].set_title('Control Inputs')
            axes[1, 1].set_xlabel('Time (s)')
            axes[1, 1].set_ylabel('Steering Angle (rad)', color='b')
            ax_twin.set_ylabel('Acceleration (m/s²)', color='r')
            axes[1, 1].tick_params(axis='y', labelcolor='b')
            ax_twin.tick_params(axis='y', labelcolor='r')
            
            # Combine legends
            lines = line1 + line2
            labels = [l.get_label() for l in lines]
            axes[1, 1].legend(lines, labels, loc='upper right')
        
        axes[1, 1].grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        
        plt.show()
    
    def create_dashboard(self, results: List[Dict[str, Any]], save_path: Optional[str] = None):
        """
        Create a comprehensive dashboard with multiple visualizations.
        
        Args:
            results: List of simulation results
            save_path: Optional path to save the dashboard
        """
        fig = plt.figure(figsize=(20, 15))
        fig.suptitle('Stanley Controller Performance Dashboard', fontsize=20, fontweight='bold')
        
        # Create grid layout
        gs = fig.add_gridspec(3, 4, hspace=0.3, wspace=0.3)
        
        # 1. Overall success rate
        ax1 = fig.add_subplot(gs[0, 0])
        total_scenarios = len(results)
        successful_scenarios = sum(1 for r in results if r['success'])
        collision_scenarios = sum(1 for r in results if r['collision'])
        
        categories = ['Successful', 'Failed', 'Collision']
        values = [successful_scenarios, total_scenarios - successful_scenarios - collision_scenarios, collision_scenarios]
        colors = ['green', 'orange', 'red']
        
        ax1.pie(values, labels=categories, colors=colors, autopct='%1.1f%%', startangle=90)
        ax1.set_title('Overall Performance')
        
        # 2. Average tracking error by scenario
        ax2 = fig.add_subplot(gs[0, 1])
        scenarios = list(set(r['scenario'] for r in results))
        scenario_errors = {}
        
        for scenario in scenarios:
            scenario_results = [r for r in results if r['scenario'] == scenario]
            if scenario_results:
                scenario_errors[scenario] = np.mean([r['metrics']['average_distance_error'] for r in scenario_results])
        
        if scenario_errors:
            scenarios_list = list(scenario_errors.keys())
            errors_list = list(scenario_errors.values())
            ax2.bar(scenarios_list, errors_list, color='orange', alpha=0.7)
            ax2.set_title('Average Tracking Error by Scenario')
            ax2.set_ylabel('Error (m)')
            ax2.tick_params(axis='x', rotation=45)
        
        # 3. Simulation time distribution
        ax3 = fig.add_subplot(gs[0, 2])
        times = [r['time'] for r in results]
        ax3.hist(times, bins=20, alpha=0.7, color='blue', edgecolor='black')
        ax3.set_title('Simulation Time Distribution')
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Frequency')
        
        # 4. Control input analysis
        ax4 = fig.add_subplot(gs[0, 3])
        all_steering = []
        all_acceleration = []
        
        for result in results:
            if 'controls' in result:
                controls = result['controls']
                all_steering.extend([c['steering'] for c in controls])
                all_acceleration.extend([c['acceleration'] for c in controls])
        
        if all_steering and all_acceleration:
            ax4.hist(all_steering, bins=30, alpha=0.5, color='blue', label='Steering')
            ax4.hist(all_acceleration, bins=30, alpha=0.5, color='red', label='Acceleration')
            ax4.set_title('Control Input Distribution')
            ax4.set_xlabel('Value')
            ax4.set_ylabel('Frequency')
            ax4.legend()
        
        # 5. Trajectory examples (show first 3 scenarios)
        ax5 = fig.add_subplot(gs[1:, :2])
        plotted_scenarios = 0
        
        for result in results:
            if plotted_scenarios >= 3:
                break
                
            if 'trajectory' in result:
                traj = result['trajectory']
                ax5.plot(traj[:, 0], traj[:, 1], alpha=0.7, linewidth=2, 
                        label=f"{result['scenario']} (Success: {result['success']})")
                plotted_scenarios += 1
        
        ax5.set_title('Example Trajectories')
        ax5.set_xlabel('X (m)')
        ax5.set_ylabel('Y (m)')
        ax5.legend()
        ax5.grid(True, alpha=0.3)
        ax5.axis('equal')
        
        # 6. Performance metrics table
        ax6 = fig.add_subplot(gs[1:, 2:])
        ax6.axis('tight')
        ax6.axis('off')
        
        # Create table data
        table_data = []
        headers = ['Scenario', 'Success', 'Time (s)', 'Avg Error (m)', 'Collisions']
        
        for scenario in scenarios:
            scenario_results = [r for r in results if r['scenario'] == scenario]
            if scenario_results:
                success_rate = sum(1 for r in scenario_results if r['success']) / len(scenario_results)
                avg_time = np.mean([r['time'] for r in scenario_results])
                avg_error = np.mean([r['metrics']['average_distance_error'] for r in scenario_results])
                collision_rate = sum(1 for r in scenario_results if r['collision']) / len(scenario_results)
                
                table_data.append([
                    scenario,
                    f"{success_rate:.2f}",
                    f"{avg_time:.2f}",
                    f"{avg_error:.2f}",
                    f"{collision_rate:.2f}"
                ])
        
        if table_data:
            table = ax6.table(cellText=table_data, colLabels=headers, 
                            cellLoc='center', loc='center')
            table.auto_set_font_size(False)
            table.set_fontsize(10)
            table.scale(1.2, 1.5)
            ax6.set_title('Performance Summary')
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        
        plt.show()