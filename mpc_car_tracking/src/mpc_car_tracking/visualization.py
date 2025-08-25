"""
Visualization tools for MPC car tracking.

This module provides plotting functions for simulation results,
trajectory visualization, and performance analysis.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from typing import List, Dict, Optional, Tuple, Any
import matplotlib.gridspec as gridspec

from .vehicle_model import VehicleState, VehicleControl, VehicleParameters
from .simulator import SimulationData, MPCSimulator
from .trajectory_planner import TrajectoryPoint


class MPCVisualizer:
    """Visualization tools for MPC simulation results."""
    
    def __init__(self, figsize: Tuple[int, int] = (12, 8)):
        """Initialize visualizer.
        
        Args:
            figsize: Default figure size
        """
        self.figsize = figsize
        plt.style.use('seaborn-v0_8' if 'seaborn-v0_8' in plt.style.available else 'default')
    
    def plot_trajectory_tracking(self, 
                               simulation_data: SimulationData,
                               title: str = "MPC Trajectory Tracking",
                               save_path: Optional[str] = None,
                               show_prediction: bool = False) -> plt.Figure:
        """Plot vehicle trajectory tracking results.
        
        Args:
            simulation_data: Simulation data
            title: Plot title
            save_path: Path to save figure
            show_prediction: Whether to show MPC predictions
            
        Returns:
            Matplotlib figure
        """
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=self.figsize)
        
        # Extract data
        actual_x = [state.x for state in simulation_data.states]
        actual_y = [state.y for state in simulation_data.states]
        ref_x = [state.x for state in simulation_data.reference_states]
        ref_y = [state.y for state in simulation_data.reference_states]
        
        # Plot 1: Trajectory in XY plane
        ax1.plot(ref_x, ref_y, 'b--', linewidth=2, label='Reference', alpha=0.7)
        ax1.plot(actual_x, actual_y, 'r-', linewidth=2, label='Actual')
        
        # Mark start and end points
        ax1.plot(actual_x[0], actual_y[0], 'go', markersize=8, label='Start')
        ax1.plot(actual_x[-1], actual_y[-1], 'ro', markersize=8, label='End')
        
        # Show MPC predictions at selected time steps
        if show_prediction and simulation_data.mpc_results:
            prediction_steps = range(0, len(simulation_data.mpc_results), 
                                   max(1, len(simulation_data.mpc_results) // 5))
            
            for step in prediction_steps:
                mpc_result = simulation_data.mpc_results[step]
                if mpc_result.success and mpc_result.predicted_states:
                    pred_x = [state.x for state in mpc_result.predicted_states]
                    pred_y = [state.y for state in mpc_result.predicted_states]
                    ax1.plot(pred_x, pred_y, 'g:', alpha=0.3, linewidth=1)
        
        ax1.set_xlabel('X Position (m)')
        ax1.set_ylabel('Y Position (m)')
        ax1.set_title('Vehicle Trajectory')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        ax1.axis('equal')
        
        # Plot 2: Tracking errors over time
        time = simulation_data.time
        position_errors = [err['position_error'] for err in simulation_data.tracking_errors]
        yaw_errors = [abs(err['yaw_error']) * 180/np.pi for err in simulation_data.tracking_errors]
        
        ax2_twin = ax2.twinx()
        
        line1 = ax2.plot(time, position_errors, 'b-', linewidth=2, label='Position Error')
        line2 = ax2_twin.plot(time, yaw_errors, 'r-', linewidth=2, label='Yaw Error')
        
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Position Error (m)', color='b')
        ax2_twin.set_ylabel('Yaw Error (deg)', color='r')
        ax2.tick_params(axis='y', labelcolor='b')
        ax2_twin.tick_params(axis='y', labelcolor='r')
        ax2.set_title('Tracking Errors')
        ax2.grid(True, alpha=0.3)
        
        # Combine legends
        lines = line1 + line2
        labels = [l.get_label() for l in lines]
        ax2.legend(lines, labels, loc='upper right')
        
        plt.suptitle(title, fontsize=14, fontweight='bold')
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        
        return fig
    
    def plot_control_inputs(self,
                           simulation_data: SimulationData,
                           title: str = "MPC Control Inputs",
                           save_path: Optional[str] = None) -> plt.Figure:
        """Plot control input history.
        
        Args:
            simulation_data: Simulation data
            title: Plot title
            save_path: Path to save figure
            
        Returns:
            Matplotlib figure
        """
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=self.figsize, sharex=True)
        
        time = simulation_data.time
        accelerations = [control.acceleration for control in simulation_data.controls]
        steering_angles = [control.steering_angle * 180/np.pi for control in simulation_data.controls]
        
        # Plot acceleration
        ax1.plot(time, accelerations, 'b-', linewidth=2)
        ax1.set_ylabel('Acceleration (m/s²)')
        ax1.set_title('Longitudinal Control')
        ax1.grid(True, alpha=0.3)
        ax1.axhline(y=0, color='k', linestyle='--', alpha=0.5)
        
        # Plot steering angle
        ax2.plot(time, steering_angles, 'r-', linewidth=2)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Steering Angle (deg)')
        ax2.set_title('Lateral Control')
        ax2.grid(True, alpha=0.3)
        ax2.axhline(y=0, color='k', linestyle='--', alpha=0.5)
        
        plt.suptitle(title, fontsize=14, fontweight='bold')
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        
        return fig
    
    def plot_vehicle_states(self,
                          simulation_data: SimulationData,
                          title: str = "Vehicle States",
                          save_path: Optional[str] = None) -> plt.Figure:
        """Plot vehicle state variables over time.
        
        Args:
            simulation_data: Simulation data
            title: Plot title
            save_path: Path to save figure
            
        Returns:
            Matplotlib figure
        """
        fig, axes = plt.subplots(2, 2, figsize=self.figsize, sharex=True)
        
        time = simulation_data.time
        
        # Extract state data
        actual_states = {
            'x': [state.x for state in simulation_data.states],
            'y': [state.y for state in simulation_data.states],
            'yaw': [state.yaw * 180/np.pi for state in simulation_data.states],
            'v': [state.v for state in simulation_data.states]
        }
        
        ref_states = {
            'x': [state.x for state in simulation_data.reference_states],
            'y': [state.y for state in simulation_data.reference_states],
            'yaw': [state.yaw * 180/np.pi for state in simulation_data.reference_states],
            'v': [state.v for state in simulation_data.reference_states]
        }
        
        # Plot each state
        state_labels = ['X Position (m)', 'Y Position (m)', 'Yaw Angle (deg)', 'Velocity (m/s)']
        state_keys = ['x', 'y', 'yaw', 'v']
        
        for i, (ax, label, key) in enumerate(zip(axes.flat, state_labels, state_keys)):
            ax.plot(time, ref_states[key], 'b--', linewidth=2, label='Reference', alpha=0.7)
            ax.plot(time, actual_states[key], 'r-', linewidth=2, label='Actual')
            ax.set_ylabel(label)
            ax.grid(True, alpha=0.3)
            
            if i == 0:  # Add legend to first subplot
                ax.legend()
        
        # Set x-label for bottom plots
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 1].set_xlabel('Time (s)')
        
        plt.suptitle(title, fontsize=14, fontweight='bold')
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        
        return fig
    
    def plot_performance_metrics(self,
                               simulation_data: SimulationData,
                               title: str = "Performance Metrics",
                               save_path: Optional[str] = None) -> plt.Figure:
        """Plot MPC performance metrics.
        
        Args:
            simulation_data: Simulation data
            title: Plot title
            save_path: Path to save figure
            
        Returns:
            Matplotlib figure
        """
        fig = plt.figure(figsize=self.figsize)
        gs = gridspec.GridSpec(2, 3, figure=fig)
        
        time = simulation_data.time
        
        # Tracking errors
        ax1 = fig.add_subplot(gs[0, 0])
        position_errors = [err['position_error'] for err in simulation_data.tracking_errors]
        ax1.plot(time, position_errors, 'b-', linewidth=2)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Position Error (m)')
        ax1.set_title('Position Tracking Error')
        ax1.grid(True, alpha=0.3)
        
        # Solve times
        ax2 = fig.add_subplot(gs[0, 1])
        solve_times = [t * 1000 for t in simulation_data.computational_times]  # Convert to ms
        ax2.plot(time, solve_times, 'g-', linewidth=2)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Solve Time (ms)')
        ax2.set_title('MPC Computational Time')
        ax2.grid(True, alpha=0.3)
        
        # Success rate
        ax3 = fig.add_subplot(gs[0, 2])
        success_rate = np.cumsum([1 if r.success else 0 for r in simulation_data.mpc_results]) / np.arange(1, len(simulation_data.mpc_results) + 1)
        ax3.plot(time, success_rate * 100, 'r-', linewidth=2)
        ax3.set_xlabel('Time (s)')
        ax3.set_ylabel('Success Rate (%)')
        ax3.set_title('MPC Solve Success Rate')
        ax3.set_ylim([0, 105])
        ax3.grid(True, alpha=0.3)
        
        # Error distribution
        ax4 = fig.add_subplot(gs[1, :2])
        x_errors = [err['x_error'] for err in simulation_data.tracking_errors]
        y_errors = [err['y_error'] for err in simulation_data.tracking_errors]
        
        ax4.scatter(x_errors, y_errors, c=time, cmap='viridis', alpha=0.6, s=20)
        ax4.set_xlabel('X Error (m)')
        ax4.set_ylabel('Y Error (m)')
        ax4.set_title('Error Distribution')
        ax4.grid(True, alpha=0.3)
        ax4.axis('equal')
        
        # Add colorbar
        scatter = ax4.collections[0]
        cbar = plt.colorbar(scatter, ax=ax4)
        cbar.set_label('Time (s)')
        
        # Statistics table
        ax5 = fig.add_subplot(gs[1, 2])
        ax5.axis('off')
        
        # Calculate statistics
        stats = {
            'Max Pos Error': f"{max(position_errors):.3f} m",
            'Mean Pos Error': f"{np.mean(position_errors):.3f} m",
            'RMS Pos Error': f"{np.sqrt(np.mean(np.array(position_errors)**2)):.3f} m",
            'Max Solve Time': f"{max(solve_times):.1f} ms",
            'Mean Solve Time': f"{np.mean(solve_times):.1f} ms",
            'Final Success Rate': f"{success_rate[-1]*100:.1f} %"
        }
        
        table_data = [[key, value] for key, value in stats.items()]
        table = ax5.table(cellText=table_data, 
                         colLabels=['Metric', 'Value'],
                         cellLoc='left',
                         loc='center',
                         colWidths=[0.6, 0.4])
        table.auto_set_font_size(False)
        table.set_fontsize(9)
        table.scale(1, 1.5)
        ax5.set_title('Performance Summary')
        
        plt.suptitle(title, fontsize=14, fontweight='bold')
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        
        return fig
    
    def animate_simulation(self,
                         simulation_data: SimulationData,
                         vehicle_params: VehicleParameters,
                         interval: int = 50,
                         save_path: Optional[str] = None) -> FuncAnimation:
        """Create animation of vehicle tracking simulation.
        
        Args:
            simulation_data: Simulation data
            vehicle_params: Vehicle parameters for drawing
            interval: Animation interval in milliseconds
            save_path: Path to save animation (GIF or MP4)
            
        Returns:
            Matplotlib animation
        """
        fig, ax = plt.subplots(figsize=(10, 8))
        
        # Extract trajectory data
        actual_x = [state.x for state in simulation_data.states]
        actual_y = [state.y for state in simulation_data.states]
        ref_x = [state.x for state in simulation_data.reference_states]
        ref_y = [state.y for state in simulation_data.reference_states]
        actual_yaw = [state.yaw for state in simulation_data.states]
        
        # Plot reference trajectory
        ax.plot(ref_x, ref_y, 'b--', linewidth=2, alpha=0.5, label='Reference')
        
        # Initialize plots
        vehicle_patch = None
        actual_line, = ax.plot([], [], 'r-', linewidth=2, label='Actual')
        
        # Set up axis
        margin = 5
        ax.set_xlim(min(min(actual_x), min(ref_x)) - margin, 
                   max(max(actual_x), max(ref_x)) + margin)
        ax.set_ylim(min(min(actual_y), min(ref_y)) - margin,
                   max(max(actual_y), max(ref_y)) + margin)
        ax.set_xlabel('X Position (m)')
        ax.set_ylabel('Y Position (m)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        
        def animate(frame):
            nonlocal vehicle_patch
            
            if frame >= len(simulation_data.states):
                frame = len(simulation_data.states) - 1
            
            # Update actual trajectory
            actual_line.set_data(actual_x[:frame+1], actual_y[:frame+1])
            
            # Remove old vehicle patch
            if vehicle_patch:
                vehicle_patch.remove()
            
            # Draw vehicle as rectangle
            vehicle_length = vehicle_params.wheelbase * 1.5
            vehicle_width = vehicle_params.wheelbase * 0.6
            
            x, y, yaw = actual_x[frame], actual_y[frame], actual_yaw[frame]
            
            # Vehicle corners in local frame
            corners_local = np.array([
                [-vehicle_length/2, -vehicle_width/2],
                [vehicle_length/2, -vehicle_width/2],
                [vehicle_length/2, vehicle_width/2],
                [-vehicle_length/2, vehicle_width/2]
            ])
            
            # Rotate and translate to global frame
            cos_yaw, sin_yaw = np.cos(yaw), np.sin(yaw)
            rotation_matrix = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])
            corners_global = corners_local @ rotation_matrix.T
            corners_global[:, 0] += x
            corners_global[:, 1] += y
            
            # Create vehicle patch
            vehicle_patch = patches.Polygon(corners_global, closed=True, 
                                          facecolor='red', alpha=0.7, edgecolor='black')
            ax.add_patch(vehicle_patch)
            
            # Add direction arrow
            arrow_length = vehicle_length * 0.7
            arrow_x = x + arrow_length * cos_yaw
            arrow_y = y + arrow_length * sin_yaw
            ax.annotate('', xy=(arrow_x, arrow_y), xytext=(x, y),
                       arrowprops=dict(arrowstyle='->', color='blue', lw=2))
            
            # Update title with current time and error
            current_error = simulation_data.tracking_errors[frame]['position_error']
            ax.set_title(f'MPC Vehicle Tracking - Time: {simulation_data.time[frame]:.1f}s, Error: {current_error:.2f}m')
            
            return [actual_line, vehicle_patch]
        
        # Create animation
        anim = FuncAnimation(fig, animate, frames=len(simulation_data.states),
                           interval=interval, blit=False, repeat=True)
        
        if save_path:
            if save_path.endswith('.gif'):
                anim.save(save_path, writer='pillow', fps=1000//interval)
            else:
                anim.save(save_path, writer='ffmpeg', fps=1000//interval)
        
        return anim
    
    def compare_scenarios(self,
                        scenario_results: Dict[str, SimulationData],
                        title: str = "Scenario Comparison",
                        save_path: Optional[str] = None) -> plt.Figure:
        """Compare results from multiple scenarios.
        
        Args:
            scenario_results: Dictionary of scenario name to simulation data
            title: Plot title
            save_path: Path to save figure
            
        Returns:
            Matplotlib figure
        """
        fig, axes = plt.subplots(2, 2, figsize=self.figsize)
        
        colors = plt.cm.tab10(np.linspace(0, 1, len(scenario_results)))
        
        for i, (scenario_name, sim_data) in enumerate(scenario_results.items()):
            color = colors[i]
            
            # Trajectory plot
            actual_x = [state.x for state in sim_data.states]
            actual_y = [state.y for state in sim_data.states]
            axes[0, 0].plot(actual_x, actual_y, color=color, linewidth=2, label=scenario_name)
            
            # Position error
            time = sim_data.time
            position_errors = [err['position_error'] for err in sim_data.tracking_errors]
            axes[0, 1].plot(time, position_errors, color=color, linewidth=2, label=scenario_name)
            
            # Solve times
            solve_times = [t * 1000 for t in sim_data.computational_times]
            axes[1, 0].plot(time, solve_times, color=color, linewidth=2, label=scenario_name)
            
            # Control effort (acceleration)
            accelerations = [abs(control.acceleration) for control in sim_data.controls]
            axes[1, 1].plot(time, accelerations, color=color, linewidth=2, label=scenario_name)
        
        # Configure subplots
        axes[0, 0].set_xlabel('X Position (m)')
        axes[0, 0].set_ylabel('Y Position (m)')
        axes[0, 0].set_title('Trajectories')
        axes[0, 0].legend()
        axes[0, 0].grid(True, alpha=0.3)
        axes[0, 0].set_aspect('equal')
        
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Position Error (m)')
        axes[0, 1].set_title('Tracking Errors')
        axes[0, 1].legend()
        axes[0, 1].grid(True, alpha=0.3)
        
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('Solve Time (ms)')
        axes[1, 0].set_title('Computational Time')
        axes[1, 0].legend()
        axes[1, 0].grid(True, alpha=0.3)
        
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].set_ylabel('|Acceleration| (m/s²)')
        axes[1, 1].set_title('Control Effort')
        axes[1, 1].legend()
        axes[1, 1].grid(True, alpha=0.3)
        
        plt.suptitle(title, fontsize=14, fontweight='bold')
        plt.tight_layout()
        
        if save_path:
            plt.savefig(save_path, dpi=300, bbox_inches='tight')
        
        return fig