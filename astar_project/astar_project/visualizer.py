"""
Hybrid A* Path Planning Visualization Module

This module contains all visualization functionality for the Hybrid A* algorithm.
Separated from the core algorithm to improve modularity and reduce dependencies.

Author: Your Name
Date: 2025-07-22
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.colors import Normalize
from matplotlib.widgets import Button, CheckButtons
from matplotlib.figure import Figure
from matplotlib.axes import Axes
from typing import List, Optional, Dict, Any, TYPE_CHECKING

if TYPE_CHECKING:
    from .hybrid_astar import HybridAStar

# Optional scipy import for smooth path interpolation
try:
    from scipy.interpolate import interp1d
    HAS_SCIPY = True
except ImportError:
    HAS_SCIPY = False

try:
    from .hybrid_astar import State, DirectionMode, Node
except ImportError:
    from astar_project.hybrid_astar import State, DirectionMode, Node


class HybridAStarVisualizer:
    """Visualization class for Hybrid A* path planning results"""
    
    def __init__(self) -> None:
        """Initialize the visualizer"""
        # Interactive control states
        self.show_exploration = True
        self.show_trajectories = True
        self.show_path = True
        self.show_vehicle_arrows = True
        self.show_steering_arrows = True
        self.show_waypoint_numbers = True
        self.show_cost_panel = False  # 默认隐藏成本面板
        
        # Store data for redrawing
        self.current_data: Optional[Dict[str, Any]] = None
        self.current_ax: Optional[Axes] = None
        self.main_ax: Optional[Axes] = None
        self.cost_ax: Optional[Axes] = None
        self.control_ax: Optional[Axes] = None
        self.fig: Optional[Figure] = None
        self.checkboxes: Optional[CheckButtons] = None
        
        # Colorbar management
        self._current_colorbars = []
    
    def visualize_path(self, path: List[State], start: State, goal: State,
                      explored_nodes: Optional[List[Node]] = None,
                      simulation_trajectories: Optional[List[Any]] = None,
                      obstacle_map: Optional[np.ndarray] = None,
                      map_origin_x: float = 0, map_origin_y: float = 0,
                      grid_resolution: float = 1.0,
                      vehicle_model: Optional[Any] = None,
                      show_exploration: bool = True, 
                      show_trajectories: bool = True, 
                      show_costs: bool = False) -> None:  # 默认不显示成本面板
        """Enhanced visualization of the planned path with interactive controls"""
        if not path:
            print("No path to visualize")
            return
        
        # Store data for interactive updates
        self.current_data = {
            'path': path,
            'start': start,
            'goal': goal,
            'explored_nodes': explored_nodes,
            'simulation_trajectories': simulation_trajectories,
            'obstacle_map': obstacle_map,
            'map_origin_x': map_origin_x,
            'map_origin_y': map_origin_y,
            'grid_resolution': grid_resolution,
            'vehicle_model': vehicle_model
        }
        
        self._create_visualization()
    
    def visualize_node_path(self, path_nodes: List[Node], start: State, goal: State,
                           planner_instance: "HybridAStar",
                           explored_nodes: Optional[List[Node]] = None,
                           simulation_trajectories: Optional[List[Any]] = None,
                           obstacle_map: Optional[np.ndarray] = None,
                           map_origin_x: float = 0, map_origin_y: float = 0,
                           grid_resolution: float = 1.0,
                           vehicle_model: Optional[Any] = None,
                           show_exploration: bool = True, 
                           show_trajectories: bool = True, 
                           show_costs: bool = False) -> None:
        """Visualize path from Node list using detailed simulation trajectories
        
        Args:
            path_nodes: List of nodes from path planning
            planner_instance: Instance of HybridAStar planner to extract detailed path
            Other parameters same as visualize_path
        """
        if not path_nodes:
            print("No path to visualize")
            return
        
        # Extract detailed path using simulation trajectories
        detailed_path = planner_instance.extract_detailed_path(path_nodes)
        
        # Use the standard visualization with the detailed path
        self.visualize_path(
            path=detailed_path,
            start=start,
            goal=goal,
            explored_nodes=explored_nodes,
            simulation_trajectories=simulation_trajectories,
            obstacle_map=obstacle_map,
            map_origin_x=map_origin_x,
            map_origin_y=map_origin_y,
            grid_resolution=grid_resolution,
            vehicle_model=vehicle_model,
            show_exploration=show_exploration,
            show_trajectories=show_trajectories,
            show_costs=show_costs
        )
    
    def _create_visualization(self) -> None:
        """Create the main visualization layout and controls"""
        # Initialize display states
        self.show_exploration = True
        self.show_trajectories = True
        
        # Create figure with space for controls - balanced size for good visibility
        self.fig = plt.figure(figsize=(18, 10))
        
        # Create layout: main plot + control panel - balanced ratios for both functionality and space
        if self.show_cost_panel:
            # With cost panel: [main plot | cost plot | controls] - balanced layout
            from matplotlib.gridspec import GridSpec
            gs = GridSpec(1, 3, width_ratios=[4.5, 1, 1.2], figure=self.fig, 
                         left=0.05, right=0.98, top=0.95, bottom=0.08, wspace=0.12)
            self.main_ax = self.fig.add_subplot(gs[0])
            self.cost_ax = self.fig.add_subplot(gs[1])
            self.control_ax = self.fig.add_subplot(gs[2])
        else:
            # Without cost panel: [main plot | controls] - give adequate space to controls
            from matplotlib.gridspec import GridSpec
            gs = GridSpec(1, 2, width_ratios=[5, 1.2], figure=self.fig,
                         left=0.05, right=0.98, top=0.95, bottom=0.08, wspace=0.08)
            self.main_ax = self.fig.add_subplot(gs[0])
            self.cost_ax = None
            self.control_ax = self.fig.add_subplot(gs[1])
        
        # Store current axis reference
        self.current_ax = self.main_ax
        
        # Initialize stored limits to None to ensure proper initial view
        self._stored_limits = None
        
        self._create_interactive_controls()
        
        # Initial plot
        self._update_visualization()
        
        plt.show()
        
        # Print detailed path statistics
        if self.current_data:
            path = self.current_data['path']
            explored_nodes = self.current_data['explored_nodes']
            simulation_trajectories = self.current_data['simulation_trajectories']
            vehicle_model = self.current_data['vehicle_model']
            self._print_path_statistics(path, explored_nodes, simulation_trajectories, vehicle_model)
    
    def _create_interactive_controls(self) -> None:
        """Create interactive control buttons and checkboxes"""
        if not self.control_ax:
            return
            
        # Clear the control axis
        self.control_ax.clear()
        self.control_ax.set_xlim(0, 1)
        self.control_ax.set_ylim(0, 1)
        self.control_ax.axis('off')
        
        # Control panel title - proper size and positioning
        self.control_ax.text(0.5, 0.95, 'Display Controls', 
                           ha='center', va='top', fontsize=11, fontweight='bold',
                           transform=self.control_ax.transAxes)
        
        # Create checkboxes for different display elements - more concise labels
        checkbox_labels = [
            'Exploration Nodes',
            'Trajectories', 
            'Final Path',
            'Vehicle Arrows',
            'Steering Arrows',
            'Waypoint Numbers',
            'Cost Panel'
        ]
        
        # Initial states
        checkbox_states = [
            self.show_exploration,
            self.show_trajectories,
            self.show_path,
            self.show_vehicle_arrows,
            self.show_steering_arrows,
            self.show_waypoint_numbers,
            self.show_cost_panel
        ]
        
        # Create checkbox widget - proper positioning with adequate space
        if self.fig:
            bbox = self.control_ax.get_position()
            # Calculate proper positioning to ensure checkboxes are visible
            checkbox_width = max(0.15, bbox.width * 0.9)  # Use 90% of available width
            checkbox_height = 0.65  # Fixed height that works well
            checkbox_x = bbox.x0 + (bbox.width - checkbox_width) / 2  # Center horizontally
            checkbox_y = bbox.y0 + 0.15  # Start from a good vertical position
            
            # Debug: print positioning info
            print(f"Control panel bbox: {bbox}")
            print(f"Checkbox position: x={checkbox_x:.3f}, y={checkbox_y:.3f}, w={checkbox_width:.3f}, h={checkbox_height:.3f}")
            
            checkbox_ax = self.fig.add_axes((checkbox_x, checkbox_y, checkbox_width, checkbox_height))
            self.checkboxes = CheckButtons(checkbox_ax, checkbox_labels, checkbox_states)
            
            # Customize checkbox appearance for better visibility
            if self.checkboxes:
                for i, label in enumerate(self.checkboxes.labels):
                    label.set_fontsize(9)  # Good readable size
                
                # Connect checkbox events
                self.checkboxes.on_clicked(self._on_checkbox_clicked)
        
        # Add instruction text - clear and positioned well
        self.control_ax.text(0.5, 0.05, 'Click checkboxes to toggle', 
                           ha='center', va='bottom', fontsize=8, style='italic',
                           transform=self.control_ax.transAxes)
    
    def _on_checkbox_clicked(self, label: Optional[str]) -> None:
        """Handle checkbox click events"""
        if label is None:
            return
            
        # Update corresponding state - match labels exactly
        if label == 'Exploration Nodes':
            self.show_exploration = not self.show_exploration
        elif label == 'Trajectories':
            self.show_trajectories = not self.show_trajectories
            self.show_trajectories = False
        elif label == 'Final Path':
            self.show_path = not self.show_path
        elif label == 'Vehicle Arrows':
            self.show_vehicle_arrows = not self.show_vehicle_arrows
        elif label == 'Steering Arrows':
            self.show_steering_arrows = not self.show_steering_arrows
        elif label == 'Waypoint Numbers':
            self.show_waypoint_numbers = not self.show_waypoint_numbers
        elif label == 'Cost Panel':
            self.show_cost_panel = not self.show_cost_panel
            # Need to recreate layout when cost panel is toggled
            self._recreate_layout()
            return
        
        # Update visualization
        self._update_visualization()
    
    def _recreate_layout(self) -> None:
        """Recreate the figure layout when cost panel is toggled"""
        if not self.fig:
            return
        
        # Store current axis limits before clearing
        xlim = self.main_ax.get_xlim() if self.main_ax else None
        ylim = self.main_ax.get_ylim() if self.main_ax else None
        
        # Remove existing colorbars
        self._remove_existing_colorbars()
            
        # Clear current figure
        self.fig.clear()
        
        # Recreate layout - use balanced ratios for proper control visibility
        if self.show_cost_panel:
            # With cost panel: [main plot | cost plot | controls] - balanced layout
            from matplotlib.gridspec import GridSpec
            gs = GridSpec(1, 3, width_ratios=[4.5, 1, 1.2], figure=self.fig,
                         left=0.05, right=0.98, top=0.95, bottom=0.08, wspace=0.12)
            self.main_ax = self.fig.add_subplot(gs[0])
            self.cost_ax = self.fig.add_subplot(gs[1])
            self.control_ax = self.fig.add_subplot(gs[2])
        else:
            # Without cost panel: [main plot | controls] - give adequate space to controls
            from matplotlib.gridspec import GridSpec
            gs = GridSpec(1, 2, width_ratios=[5, 1.2], figure=self.fig,
                         left=0.05, right=0.98, top=0.95, bottom=0.08, wspace=0.08)
            self.main_ax = self.fig.add_subplot(gs[0])
            self.cost_ax = None
            self.control_ax = self.fig.add_subplot(gs[1])
        
        # Recreate controls and update visualization
        self._create_interactive_controls()
        self._update_visualization()
        
        # Restore axis limits if they existed
        if xlim and ylim and self.main_ax:
            self.main_ax.set_xlim(xlim)
            self.main_ax.set_ylim(ylim)
        
        if self.fig.canvas:
            self.fig.canvas.draw()
    
    def _update_visualization(self) -> None:
        """Update the main visualization based on current control states"""
        if not self.current_data or not self.main_ax:
            return
        
        # Store the current axis limits to prevent shrinking - improved logic
        xlim = None
        ylim = None
        use_stored_limits = False
        
        try:
            current_xlim = self.main_ax.get_xlim()
            current_ylim = self.main_ax.get_ylim()
            
            # Check if limits are meaningful and not matplotlib defaults
            xlim_range = abs(current_xlim[1] - current_xlim[0])
            ylim_range = abs(current_ylim[1] - current_ylim[0])
            
            # Use stored limits if they're reasonable and not just matplotlib's auto-scaling
            if (xlim_range > 1.0 and ylim_range > 1.0 and 
                not (current_xlim == (0.0, 1.0) and current_ylim == (0.0, 1.0))):
                xlim = current_xlim
                ylim = current_ylim
                use_stored_limits = True
            # Fallback to previously stored limits if available
            elif hasattr(self, '_stored_limits') and self._stored_limits:
                xlim = self._stored_limits[0]
                ylim = self._stored_limits[1] 
                use_stored_limits = True
        except Exception as e:
            print(f"Warning: Could not preserve axis limits: {e}")
            use_stored_limits = False
        
        # Remove existing colorbars first to prevent duplicates
        self._remove_existing_colorbars()
        
        # Clear main axis
        self.main_ax.clear()
        
        # Plot main visualization with current settings
        self._plot_main_visualization(
            self.main_ax, 
            self.current_data['path'], 
            self.current_data['start'], 
            self.current_data['goal'],
            self.current_data['explored_nodes'], 
            self.current_data['simulation_trajectories'],
            self.current_data['obstacle_map'], 
            self.current_data['map_origin_x'], 
            self.current_data['map_origin_y'],
            self.current_data['grid_resolution'], 
            self.current_data['vehicle_model'],
            self.show_exploration, 
            self.show_trajectories
        )
        
        # Restore axis limits if they were meaningful, otherwise let matplotlib auto-scale once
        if use_stored_limits and xlim and ylim:
            try:
                self.main_ax.set_xlim(xlim)
                self.main_ax.set_ylim(ylim)
            except Exception as e:
                print(f"Warning: Could not restore axis limits: {e}")
        
        # Always store current limits for future updates (after potential auto-scaling)
        try:
            self._stored_limits = (self.main_ax.get_xlim(), self.main_ax.get_ylim())
        except:
            pass
        
        # Update cost panel if visible
        if self.show_cost_panel and self.cost_ax:
            self._plot_cost_analysis(self.cost_ax, self.current_data['path'])
        
        # Redraw canvas
        if self.fig and self.fig.canvas:
            self.fig.canvas.draw()
    
    def _remove_existing_colorbars(self) -> None:
        """Remove existing colorbars to prevent duplicates"""
        if hasattr(self, '_current_colorbars'):
            for cbar in self._current_colorbars:
                try:
                    cbar.remove()
                except:
                    pass
            self._current_colorbars.clear()
        else:
            self._current_colorbars = []
    
    def _plot_main_visualization(self, ax: Axes, path: List[State], start: State, goal: State, 
                               explored_nodes: Optional[List[Node]], simulation_trajectories: Optional[List[Any]],
                               obstacle_map: Optional[np.ndarray], map_origin_x: float, map_origin_y: float, 
                               grid_resolution: float, vehicle_model: Optional[Any],
                               show_exploration: bool, show_trajectories: bool) -> None:
        """Plot main path visualization with exploration details"""
        
        # Plot obstacle map if available (black = obstacles, white = free space)
        if obstacle_map is not None:
            map_height, map_width = obstacle_map.shape
            extent = (map_origin_x, 
                     map_origin_x + map_width * grid_resolution,
                     map_origin_y,
                     map_origin_y + map_height * grid_resolution)
            # Invert the colormap so black = obstacles (1), white = free space (0)
            ax.imshow(obstacle_map, extent=extent, origin='lower', 
                     cmap='gray_r', alpha=0.8, vmin=0, vmax=1)
        
        # Plot exploration nodes with enhanced visibility (if enabled)
        if self.show_exploration and explored_nodes:
            # Plot parent-child connections using forward simulation trajectories
            print(f"Plotting {len(explored_nodes)} exploration connections with simulation trajectories...")
            connection_count = 0
            for node in explored_nodes:
                if node.parent is not None:
                    # Use trajectory states if available, otherwise fall back to straight line
                    if hasattr(node, 'trajectory_states') and node.trajectory_states:
                        # Plot the forward simulation trajectory from parent to this node
                        traj_x = [node.parent.state.x] + [s.x for s in node.trajectory_states]
                        traj_y = [node.parent.state.y] + [s.y for s in node.trajectory_states]
                        
                        # Color by cost for better visualization
                        cost_normalized = min(1.0, node.f_cost / 100.0)  # Normalize cost
                        color = cm.get_cmap('plasma')(cost_normalized)
                        
                        # Plot trajectory as connected line segments
                        ax.plot(traj_x, traj_y, color=color, linewidth=1.0, alpha=0.5, zorder=1)
                        
                        # Add small arrows along trajectory to show direction
                        if len(traj_x) > 2:
                            mid_idx = len(traj_x) // 2
                            if mid_idx > 0 and mid_idx < len(traj_x) - 1:
                                dx = traj_x[mid_idx + 1] - traj_x[mid_idx - 1]
                                dy = traj_y[mid_idx + 1] - traj_y[mid_idx - 1]
                                length = np.sqrt(dx*dx + dy*dy)
                                if length > 0.1:
                                    ax.arrow(traj_x[mid_idx], traj_y[mid_idx], 
                                           dx/length * 0.2, dy/length * 0.2,
                                           head_width=0.08, head_length=0.08,
                                           fc=color, ec=color, alpha=0.7, zorder=2)
                    else:
                        # Fallback to straight line connection
                        ax.plot([node.parent.state.x, node.state.x], 
                               [node.parent.state.y, node.state.y], 
                               'cyan', linewidth=0.8, alpha=0.4, zorder=1)
                    connection_count += 1
            
            # Plot exploration nodes with better visibility
            explored_x = [node.state.x for node in explored_nodes]
            explored_y = [node.state.y for node in explored_nodes]
            
            # Use different colors based on cost for better insight
            f_costs = [node.f_cost for node in explored_nodes]
            if f_costs:
                # Normalize costs for color mapping
                min_cost = min(f_costs)
                max_cost = max(f_costs)
                if max_cost > min_cost:
                    normalized_costs = [(cost - min_cost) / (max_cost - min_cost) for cost in f_costs]
                else:
                    normalized_costs = [0.5] * len(f_costs)
                
                scatter = ax.scatter(explored_x, explored_y, c=normalized_costs, 
                                   cmap='viridis', s=20, alpha=0.7, zorder=2,
                                   edgecolors='white', linewidths=0.5,
                                   label=f'Explored Nodes ({len(explored_nodes)}, {connection_count} connections)')
                
                # Add colorbar for cost visualization (only if not already present)
                if len(explored_nodes) > 10:  # Only add colorbar if meaningful
                    cbar = plt.colorbar(scatter, ax=ax, shrink=0.3, pad=0.02)
                    cbar.set_label('F-cost (normalized)', fontsize=8)
                    # Track colorbar for proper cleanup
                    if not hasattr(self, '_current_colorbars'):
                        self._current_colorbars = []
                    self._current_colorbars.append(cbar)
            else:
                ax.scatter(explored_x, explored_y, c='lightblue', s=20, alpha=0.7, 
                          edgecolors='white', linewidths=0.5, zorder=2,
                          label=f'Explored Nodes ({len(explored_nodes)})')
        
        # Plot simulation trajectories (if enabled) with enhanced visualization
        if self.show_trajectories and simulation_trajectories:
            # Sample trajectories to avoid overcrowding
            sample_rate = max(1, len(simulation_trajectories) // 50)
            sampled_trajectories = simulation_trajectories[::sample_rate]
            
            print(f"Showing {len(sampled_trajectories)} sampled forward simulation trajectories from {len(simulation_trajectories)} total")
            
            for i, traj in enumerate(sampled_trajectories):
                states = traj['states']
                direction = traj['direction']
                
                x_coords = [s.x for s in states]
                y_coords = [s.y for s in states]
                
                # Different colors and styles for forward/backward motion
                if direction == DirectionMode.FORWARD:
                    color = 'lightgreen'
                    marker = '>'  # Right arrow = forward motion
                    alpha = 0.4
                    edge_color = 'darkgreen'
                else:
                    color = 'lightcoral'
                    marker = '<'  # Left arrow = backward motion
                    alpha = 0.4
                    edge_color = 'darkred'
                
                # Plot trajectory as a line with reduced opacity
                ax.plot(x_coords, y_coords, color=color, alpha=alpha, linewidth=1.5)
                
                # Add directional markers at regular intervals to show motion direction
                marker_step = max(1, len(states) // 3)
                for j in range(0, len(states), marker_step):
                    if j < len(states):
                        ax.scatter(x_coords[j], y_coords[j], c=color, marker=marker, 
                                 s=30, alpha=alpha + 0.3, edgecolors=edge_color,
                                 linewidths=0.5, zorder=2)
                
                # Add arrow at the end of trajectory to show final vehicle orientation
                if len(states) > 1:
                    final_state = states[-1]
                    arrow_length = 0.4
                    dx = arrow_length * np.cos(final_state.yaw)
                    dy = arrow_length * np.sin(final_state.yaw)
                    ax.arrow(final_state.x, final_state.y, dx, dy,
                           head_width=0.1, head_length=0.1,
                           fc=color, ec=edge_color,
                           alpha=alpha + 0.4, linewidth=1.5, zorder=2)
            
            # Add legend entries for trajectories (only once)
            if sampled_trajectories:
                # Create invisible plot elements for legend with detailed explanations
                forward_count = len([t for t in simulation_trajectories if t["direction"] == DirectionMode.FORWARD])
                backward_count = len([t for t in simulation_trajectories if t["direction"] != DirectionMode.FORWARD])
                
                ax.plot([], [], color='lightgreen', linewidth=2, alpha=0.7, 
                       label=f'Forward Sim. Trajectories ({forward_count}) - ">" indicates forward motion')
                ax.plot([], [], color='lightcoral', linewidth=2, alpha=0.7, 
                       label=f'Backward Sim. Trajectories ({backward_count}) - "<" indicates backward motion')
                
                # Add explanation for arrows
                ax.plot([], [], 'k-', alpha=0, label='Arrow Meanings:')
                ax.plot([], [], 'k-', alpha=0, label='  • > = Forward motion')
                ax.plot([], [], 'k-', alpha=0, label='  • < = Backward motion')
                ax.plot([], [], 'k-', alpha=0, label='  • Long arrows = Vehicle orientation')
        
        # Plot final path with enhanced details - smooth curves (if enabled)
        if self.show_path:
            self._plot_path_with_steering_colors(ax, path, vehicle_model)
        
        # Plot vehicle orientations along path (if enabled)
        if self.show_vehicle_arrows or self.show_steering_arrows:
            self._plot_vehicle_orientations(ax, path)
        
        # Enhanced start and goal visualization
        self._plot_start_goal(ax, start, goal)
        
        # Waypoint numbers (every 10th point) (if enabled)
        if self.show_waypoint_numbers:
            waypoint_step = max(1, len(path) // 15)
            for i in range(0, len(path), waypoint_step):
                ax.annotate(f'{i}', (path[i].x, path[i].y), 
                           xytext=(5, 5), textcoords='offset points',
                           fontsize=8, alpha=0.7, 
                           bbox=dict(boxstyle='round,pad=0.2', facecolor='yellow', alpha=0.7))
        
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title('Hybrid A* Path Planning - Detailed Visualization\n' + 
                    'Blue curves=Search tree (forward simulation), Green/Red=Exploration, Red line=Final path', 
                    fontsize=12)
        
        # Add a text box with legend explanations
        legend_text = ('Visualization Guide:\n'
                      '• Cyan curves: Search tree connections (forward simulation)\n'
                      '• Green traj >: Forward exploration\n' 
                      '• Red traj <: Backward exploration\n'
                      '• Black rect: Vehicle outline\n'
                      '• Blue arrow: Vehicle heading\n'
                      '• Red arrow: Front wheel steering')
        
        ax.text(0.02, 0.02, legend_text, transform=ax.transAxes, 
               fontsize=8, verticalalignment='bottom',
               bbox=dict(boxstyle='round,pad=0.5', facecolor='lightyellow', alpha=0.9))
        
        ax.legend(fontsize=9, loc='upper right')
        ax.grid(True, alpha=0.3)
        
        # Set equal aspect but allow adjustable limits to avoid conflicts
        ax.set_aspect('equal', adjustable='box')
        
        # Add color bar for steering angle if vehicle model available
        if vehicle_model:
            sm = cm.ScalarMappable(cmap=cm.get_cmap('RdBu_r'), 
                                   norm=Normalize(vmin=-vehicle_model.max_steer, 
                                                 vmax=vehicle_model.max_steer))
            sm.set_array([])
            cbar = plt.colorbar(sm, ax=ax, shrink=0.6)
            cbar.set_label('Steering Angle (rad)', fontsize=10)
            # Track colorbar for proper cleanup
            if not hasattr(self, '_current_colorbars'):
                self._current_colorbars = []
            self._current_colorbars.append(cbar)
    
    def _plot_path_with_steering_colors(self, ax: Axes, path: List[State], vehicle_model: Optional[Any]) -> None:
        """Plot path with steering angle color coding"""
        x_coords = np.array([state.x for state in path])
        y_coords = np.array([state.y for state in path])
        steer_angles = [state.steer for state in path]
        
        if not vehicle_model:
            # Simple path without steering colors
            ax.plot(x_coords, y_coords, 'red', linewidth=3, alpha=0.8, label='Path')
            return
        
        # Create smooth interpolated path for better visualization
        if len(path) > 3 and HAS_SCIPY:
            try:
                # Parameter for interpolation (arc length approximation)
                t = np.linspace(0, 1, len(path))
                
                # Create smooth interpolation
                f_x = interp1d(t, x_coords, kind='cubic', assume_sorted=True)
                f_y = interp1d(t, y_coords, kind='cubic', assume_sorted=True)
                
                # Generate more points for smooth curve
                t_smooth = np.linspace(0, 1, len(path) * 5)  # 5x more points
                x_smooth = f_x(t_smooth)
                y_smooth = f_y(t_smooth)
                
                # Interpolate steering angles as well
                f_steer = interp1d(t, steer_angles, kind='linear', assume_sorted=True)
                steer_smooth = f_steer(t_smooth)
                
                # Create segments for colored smooth path
                for i in range(len(x_smooth)-1):
                    # Color based on steering angle
                    steer_normalized = abs(steer_smooth[i]) / vehicle_model.max_steer
                    color_intensity = steer_normalized
                    
                    if steer_smooth[i] > 0:
                        color = cm.get_cmap('Reds')(0.3 + 0.7 * color_intensity)
                    elif steer_smooth[i] < 0:
                        color = cm.get_cmap('Blues')(0.3 + 0.7 * color_intensity)
                    else:
                        color = 'green'
                    
                    ax.plot([x_smooth[i], x_smooth[i+1]], [y_smooth[i], y_smooth[i+1]], 
                           color=color, linewidth=3, alpha=0.8)
                
            except Exception as e:
                print(f"Warning: Smooth interpolation failed ({e}), using linear path visualization")
                self._plot_linear_path_segments(ax, path, vehicle_model)
        else:
            self._plot_linear_path_segments(ax, path, vehicle_model)
    
    def _plot_linear_path_segments(self, ax: Axes, path: List[State], vehicle_model: Optional[Any]) -> None:
        """Plot path as linear segments with steering colors"""
        x_coords = [state.x for state in path]
        y_coords = [state.y for state in path]
        steer_angles = [state.steer for state in path]
        
        if not vehicle_model or not hasattr(vehicle_model, 'max_steer'):
            # Fallback to simple plotting without steering colors
            ax.plot(x_coords, y_coords, 'r-', linewidth=3, label='Path')
            return
        
        for i in range(len(path)-1):
            steer_normalized = abs(steer_angles[i]) / vehicle_model.max_steer
            color_intensity = steer_normalized
            
            if steer_angles[i] > 0:
                color = cm.get_cmap('Reds')(0.3 + 0.7 * color_intensity)
            elif steer_angles[i] < 0:
                color = cm.get_cmap('Blues')(0.3 + 0.7 * color_intensity)
            else:
                color = 'green'
            
            ax.plot([x_coords[i], x_coords[i+1]], [y_coords[i], y_coords[i+1]], 
                   color=color, linewidth=3, alpha=0.8)
    
    def _plot_vehicle_orientations(self, ax: Axes, path: List[State]) -> None:
        """Plot vehicle orientations along path with detailed visual indicators
        
        Visual elements explained:
        - Black rectangle: Vehicle body outline
        - Blue arrow: Vehicle heading direction (yaw angle)
        - Red arrow: Front wheel steering direction (when steering > 0.1 rad)
        """
        orientation_step = max(1, len(path) // 30)
        for i in range(0, len(path), orientation_step):
            state = path[i]
            
            # Vehicle body representation (always show)
            vehicle_length = 0.8
            vehicle_width = 0.4
            
            # Calculate vehicle corners
            cos_yaw = np.cos(state.yaw)
            sin_yaw = np.sin(state.yaw)
            
            # Vehicle outline (black rectangle = vehicle body)
            front_x = state.x + vehicle_length/2 * cos_yaw
            front_y = state.y + vehicle_length/2 * sin_yaw
            rear_x = state.x - vehicle_length/2 * cos_yaw
            rear_y = state.y - vehicle_length/2 * sin_yaw
            
            # Draw vehicle as rectangle
            vehicle_corners_x = [
                rear_x - vehicle_width/2 * sin_yaw,
                rear_x + vehicle_width/2 * sin_yaw,
                front_x + vehicle_width/2 * sin_yaw,
                front_x - vehicle_width/2 * sin_yaw,
                rear_x - vehicle_width/2 * sin_yaw
            ]
            vehicle_corners_y = [
                rear_y + vehicle_width/2 * cos_yaw,
                rear_y - vehicle_width/2 * cos_yaw,
                front_y - vehicle_width/2 * cos_yaw,
                front_y + vehicle_width/2 * cos_yaw,
                rear_y + vehicle_width/2 * cos_yaw
            ]
            
            ax.plot(vehicle_corners_x, vehicle_corners_y, 'k-', linewidth=1, alpha=0.7)
            
            # Direction arrow (blue = vehicle heading direction) - controlled by show_vehicle_arrows
            if self.show_vehicle_arrows:
                arrow_length = 0.6
                dx = arrow_length * cos_yaw
                dy = arrow_length * sin_yaw
                ax.arrow(state.x, state.y, dx, dy, head_width=0.15, 
                        head_length=0.15, fc='darkblue', ec='darkblue', alpha=0.8)
            
            # Steering angle visualization (red = front wheel direction when steering) - controlled by show_steering_arrows
            if self.show_steering_arrows and abs(state.steer) > 0.01:  # Only show if significant steering
                # Front wheel position
                front_wheel_x = state.x + vehicle_length/2 * cos_yaw
                front_wheel_y = state.y + vehicle_length/2 * sin_yaw
                
                # Steered wheel direction - CORRECTED: add steer angle
                # For bicycle model: positive steer = left turn, so front wheel points left relative to vehicle
                wheel_yaw = state.yaw + state.steer  # FIXED: should be + not - for correct steering visualization
                wheel_dx = 0.3 * np.cos(wheel_yaw)
                wheel_dy = 0.3 * np.sin(wheel_yaw)
                
                ax.arrow(front_wheel_x, front_wheel_y, wheel_dx, wheel_dy,
                        head_width=0.1, head_length=0.1, 
                        fc='red', ec='red', alpha=0.8)
    
    def _plot_start_goal(self, ax: Axes, start: State, goal: State) -> None:
        """Plot start and goal with direction arrows"""
        # Start
        ax.plot(start.x, start.y, 'go', markersize=12, label='Start', zorder=10)
        start_dx = 1.2 * np.cos(start.yaw)
        start_dy = 1.2 * np.sin(start.yaw)
        ax.arrow(start.x, start.y, start_dx, start_dy, 
                head_width=0.4, head_length=0.4, fc='green', ec='darkgreen', 
                linewidth=2, zorder=10)
        
        # Goal
        ax.plot(goal.x, goal.y, 'ro', markersize=12, label='Goal', zorder=10)
        goal_dx = 1.2 * np.cos(goal.yaw)
        goal_dy = 1.2 * np.sin(goal.yaw)
        ax.arrow(goal.x, goal.y, goal_dx, goal_dy,
                head_width=0.4, head_length=0.4, fc='red', ec='darkred',
                linewidth=2, zorder=10)
    
    def _plot_cost_analysis(self, ax: Axes, path: List[State]) -> None:
        """Plot cost analysis charts
        
        路径成本分析图表包含三个关键指标：
        1. Steering Cost (转向成本) - 红色：衡量转向角度大小，反映转向剧烈程度
        2. Turn Rate (转向速率) - 蓝色：衡量角度变化率，反映转向的急剧程度  
        3. Curvature (曲率) - 绿色：衡量路径弯曲程度，数值越大路径越弯曲
        
        这些指标帮助评估路径的平滑性和车辆跟踪的难易程度
        """
        if len(path) < 2:
            return
            
        # Calculate costs along path
        path_indices = list(range(len(path)))
        steer_costs = []
        turn_costs = []
        curvatures = []
        
        for i, state in enumerate(path):
            # Steering cost
            steer_cost = abs(state.steer) / 0.785  # Approximate max steer
            steer_costs.append(steer_cost)
            
            # Turn cost (yaw change rate)
            if i > 0:
                yaw_change = abs(self._normalize_angle(state.yaw - path[i-1].yaw)) / 0.1  # Approximate dt
                turn_costs.append(yaw_change)
            else:
                turn_costs.append(0)
            
            # Curvature approximation
            if i > 0 and i < len(path) - 1:
                p1 = np.array([path[i-1].x, path[i-1].y])
                p2 = np.array([state.x, state.y])
                p3 = np.array([path[i+1].x, path[i+1].y])
                
                # Calculate curvature
                a = np.linalg.norm(p2 - p1)
                b = np.linalg.norm(p3 - p2)
                c = np.linalg.norm(p3 - p1)
                
                if a > 1e-6 and b > 1e-6:
                    area = 0.5 * abs(np.cross(p2 - p1, p3 - p1))
                    curvature = 4 * area / (a * b * c) if c > 1e-6 else 0
                else:
                    curvature = 0
                curvatures.append(curvature)
            else:
                curvatures.append(0)
        
        # Create subplots for different cost components
        ax.clear()
        
        # Multi-axis plot
        ax2 = ax.twinx()
        ax3 = ax.twinx()
        ax3.spines['right'].set_position(('outward', 60))
        
        # Plot different cost components
        line1 = ax.plot(path_indices, steer_costs, 'r-', linewidth=2, 
                       label='Steering Cost', alpha=0.8)
        line2 = ax2.plot(path_indices, turn_costs, 'b-', linewidth=2, 
                        label='Turn Rate (rad/s)', alpha=0.8)
        line3 = ax3.plot(path_indices, curvatures, 'g-', linewidth=2, 
                        label='Curvature', alpha=0.8)
        
        # Styling
        ax.set_xlabel('Path Index', fontsize=10)
        ax.set_ylabel('Steering Cost', color='r', fontsize=10)
        ax2.set_ylabel('Turn Rate (rad/s)', color='b', fontsize=10)
        ax3.set_ylabel('Curvature', color='g', fontsize=10)
        
        ax.tick_params(axis='y', labelcolor='r', labelsize=9)
        ax2.tick_params(axis='y', labelcolor='b', labelsize=9)
        ax3.tick_params(axis='y', labelcolor='g', labelsize=9)
        ax.tick_params(axis='x', labelsize=9)
        
        ax.set_title('Path Cost Analysis\n(Steering, Turn Rate, Curvature)', fontsize=11)
        ax.grid(True, alpha=0.3)
        
        # Combined legend with smaller font
        lines = line1 + line2 + line3
        labels: List[str] = [str(l.get_label()) for l in lines]
        ax.legend(lines, labels, loc='upper right', fontsize=8)
        
        # Add explanation text
        explanation_text = ('Red: Steering Cost\nBlue: Turn Rate\nGreen: Curvature\n\nHigher values\nindicate more\ndemanding motion')
        ax.text(0.02, 0.02, explanation_text, transform=ax.transAxes, 
               fontsize=7, verticalalignment='bottom',
               bbox=dict(boxstyle='round,pad=0.3', facecolor='lightcyan', alpha=0.8))
    
    @staticmethod
    def _normalize_angle(angle: float) -> float:
        """Normalize angle to [-pi, pi]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def _print_path_statistics(self, path: List[State], explored_nodes: Optional[List[Node]], 
                              simulation_trajectories: Optional[List[Any]], vehicle_model: Optional[Any]) -> None:
        """Print detailed path statistics"""
        if not path:
            return
            
        print(f"\n{'='*60}")
        print(f"DETAILED PATH STATISTICS")
        print(f"{'='*60}")
        
        # Basic statistics
        total_distance = sum(np.sqrt((path[i+1].x - path[i].x)**2 + 
                                   (path[i+1].y - path[i].y)**2) 
                           for i in range(len(path)-1))
        
        print(f"Path length: {len(path)} waypoints")
        print(f"Total distance: {total_distance:.2f} m")
        print(f"Average waypoint spacing: {total_distance/(len(path)-1):.2f} m")
        
        # Steering statistics
        steer_angles = [state.steer for state in path]
        max_steer = max(abs(s) for s in steer_angles)
        avg_steer = np.mean([abs(s) for s in steer_angles])
        
        print(f"\nSTEERING ANALYSIS:")
        print(f"Maximum steering angle: {np.degrees(max_steer):.1f}° ({max_steer:.3f} rad)")
        print(f"Average |steering|: {np.degrees(avg_steer):.1f}° ({avg_steer:.3f} rad)")
        
        if vehicle_model:
            print(f"Steering utilization: {max_steer/vehicle_model.max_steer*100:.1f}%")
        
        # Direction changes
        direction_changes = 0
        for i in range(1, len(path)):
            if path[i].direction != path[i-1].direction:
                direction_changes += 1
        
        print(f"Direction changes (cusps): {direction_changes}")
        
        # Exploration statistics
        if explored_nodes:
            print(f"\nSEARCH STATISTICS:")
            print(f"Nodes explored: {len(explored_nodes)}")
            
        if simulation_trajectories:
            print(f"Trajectories simulated: {len(simulation_trajectories)}")
        
        print(f"{'='*60}")
    
    def visualize_detailed_search_tree(self, path: List[State], start: State, goal: State,
                                     explored_nodes: List[Node],
                                     obstacle_map: Optional[np.ndarray] = None,
                                     map_origin_x: float = 0, map_origin_y: float = 0,
                                     grid_resolution: float = 1.0,
                                     max_connections: int = 1000, 
                                     node_spacing_filter: float = 0.5):
        """Detailed visualization of search tree with all connections clearly visible"""
        if not explored_nodes:
            print("No exploration data available")
            return
            
        fig, ax = plt.subplots(1, 1, figsize=(16, 12))
        
        # Plot obstacle map with higher contrast (black = obstacles, white = free)
        if obstacle_map is not None:
            map_height, map_width = obstacle_map.shape
            extent = (map_origin_x, 
                     map_origin_x + map_width * grid_resolution,
                     map_origin_y,
                     map_origin_y + map_height * grid_resolution)
            # Use inverted grayscale for clear obstacle visualization
            ax.imshow(obstacle_map, extent=extent, origin='lower', 
                     cmap='gray_r', alpha=0.9, vmin=0, vmax=1, zorder=0)
        
        # Filter nodes to reduce visual clutter while keeping structure
        filtered_nodes = self._filter_nodes_for_visualization(explored_nodes, goal, node_spacing_filter)
        print(f"Showing {len(filtered_nodes)} filtered nodes from {len(explored_nodes)} total")
        
        # Draw parent-child connections with enhanced visibility
        connection_count = self._plot_search_tree_connections(ax, filtered_nodes, max_connections)
        
        # Draw nodes with enhanced visibility
        self._plot_search_tree_nodes(ax, filtered_nodes, goal)
        
        # Highlight final path with thick line
        if path:
            self._plot_final_path_overlay(ax, path)
        
        # Enhanced start and goal markers
        self._plot_start_goal_markers(ax, start, goal)
        
        # Add search statistics as text
        self._add_search_statistics_text(ax, explored_nodes, filtered_nodes, connection_count, path)
        
        ax.set_xlabel('X (m)', fontsize=14, fontweight='bold')
        ax.set_ylabel('Y (m)', fontsize=14, fontweight='bold')
        ax.set_title('Detailed Search Tree Visualization\n(Parent-Child Connections & Node Costs)', 
                    fontsize=16, fontweight='bold')
        ax.legend(fontsize=12, loc='upper right')
        ax.grid(True, alpha=0.4, linestyle='--')
        ax.axis('equal')
        
        # Set better axis limits
        self._set_axis_limits(ax, filtered_nodes, start, goal)
        
        plt.tight_layout()
        plt.show()
        
        print(f"Displayed search tree with {len(filtered_nodes)} nodes and {connection_count} connections")
    
    def _filter_nodes_for_visualization(self, explored_nodes, goal, node_spacing_filter):
        """Filter nodes to reduce visual clutter while keeping structure"""
        filtered_nodes = []
        for i, node in enumerate(explored_nodes):
            # Keep every N-th node or nodes with special significance
            if (i % 3 == 0 or  # Every 3rd node
                node.parent is None or  # Root nodes
                abs(node.state.x - goal.x) + abs(node.state.y - goal.y) < 3.0):  # Near goal
                
                # Check spacing with already filtered nodes
                too_close = False
                for fn in filtered_nodes[-20:]:  # Check last 20 for performance
                    if (abs(node.state.x - fn.state.x) < node_spacing_filter and 
                        abs(node.state.y - fn.state.y) < node_spacing_filter):
                        too_close = True
                        break
                
                if not too_close:
                    filtered_nodes.append(node)
        
        return filtered_nodes
    
    def _plot_search_tree_connections(self, ax, filtered_nodes, max_connections):
        """Plot parent-child connections in search tree"""
        connection_count = 0
        
        for node in filtered_nodes:
            if node.parent is not None and connection_count < max_connections:
                # Find parent in filtered list
                parent_node = self._find_parent_in_filtered(node, filtered_nodes)
                
                if parent_node:
                    # Color by search depth/generation
                    depth_color = min(1.0, connection_count / max_connections)
                    color = cm.get_cmap('plasma')(depth_color)
                    
                    # Draw connection with gradient effect
                    ax.plot([parent_node.state.x, node.state.x], 
                           [parent_node.state.y, node.state.y], 
                           color=color, linewidth=1.5, alpha=0.7, zorder=1,
                           solid_capstyle='round')
                    
                    # Add small arrow to show direction
                    if connection_count % 5 == 0:  # Only for some connections
                        self._add_connection_arrow(ax, parent_node, node, color)
                    
                    connection_count += 1
        
        return connection_count
    
    def _find_parent_in_filtered(self, node, filtered_nodes):
        """Find parent node in filtered list"""
        for fn in filtered_nodes:
            if (abs(fn.state.x - node.parent.state.x) < 0.1 and 
                abs(fn.state.y - node.parent.state.y) < 0.1):
                return fn
        return None
    
    def _add_connection_arrow(self, ax, parent_node, node, color):
        """Add directional arrow on connection"""
        mid_x = (parent_node.state.x + node.state.x) / 2
        mid_y = (parent_node.state.y + node.state.y) / 2
        dx = node.state.x - parent_node.state.x
        dy = node.state.y - parent_node.state.y
        length = np.sqrt(dx*dx + dy*dy)
        if length > 0.1:
            dx_norm = dx / length * 0.3
            dy_norm = dy / length * 0.3
            ax.arrow(mid_x - dx_norm/2, mid_y - dy_norm/2, 
                   dx_norm, dy_norm,
                   head_width=0.15, head_length=0.15,
                   fc=color, ec=color, alpha=0.8, zorder=2)
    
    def _plot_search_tree_nodes(self, ax, filtered_nodes, goal):
        """Plot nodes in search tree with variable sizes and colors"""
        node_x = []
        node_y = []
        node_costs = []
        node_sizes = []
        node_colors = []
        
        for i, node in enumerate(filtered_nodes):
            node_x.append(node.state.x)
            node_y.append(node.state.y)
            node_costs.append(node.f_cost)
            
            # Variable size based on importance
            if node.parent is None:  # Start node
                size = 120
                node_colors.append('green')
            elif abs(node.state.x - goal.x) + abs(node.state.y - goal.y) < 2.0:  # Near goal
                size = 80
                node_colors.append('orange')
            else:
                size = max(20, 60 - min(40, node.f_cost))
                node_colors.append('lightblue')
            
            node_sizes.append(size)
        
        # Plot nodes
        for i in range(len(node_x)):
            ax.scatter(node_x[i], node_y[i], 
                      c=node_colors[i], s=node_sizes[i], 
                      alpha=0.9, zorder=3,
                      edgecolors='black', linewidths=1)
            
            # Add cost labels for important nodes
            if node_sizes[i] > 50:
                ax.annotate(f'{node_costs[i]:.1f}', 
                           (node_x[i], node_y[i]), 
                           xytext=(3, 3), textcoords='offset points',
                           fontsize=8, fontweight='bold',
                           bbox=dict(boxstyle='round,pad=0.2', 
                                   facecolor='white', alpha=0.8))
    
    def _plot_final_path_overlay(self, ax, path):
        """Plot final path as overlay on search tree"""
        path_x = [state.x for state in path]
        path_y = [state.y for state in path]
        ax.plot(path_x, path_y, 'red', linewidth=6, alpha=0.9, 
               label='Final Path', zorder=5, solid_capstyle='round')
        
        # Mark path nodes
        ax.scatter(path_x, path_y, c='red', s=100, zorder=6, 
                  edgecolors='white', linewidths=2, marker='D')
    
    def _plot_start_goal_markers(self, ax, start, goal):
        """Plot enhanced start and goal markers"""
        ax.scatter(start.x, start.y, c='darkgreen', s=200, 
                  marker='s', label='Start', zorder=10,
                  edgecolors='white', linewidths=3)
        ax.scatter(goal.x, goal.y, c='darkred', s=200, 
                  marker='*', label='Goal', zorder=10,
                  edgecolors='white', linewidths=3)
    
    def _add_search_statistics_text(self, ax, explored_nodes, filtered_nodes, connection_count, path):
        """Add search statistics text to plot"""
        stats_text = f"""Search Statistics:
Total Nodes: {len(explored_nodes)}
Shown Nodes: {len(filtered_nodes)}
Connections: {connection_count}
Path Length: {len(path) if path else 0}"""
        
        ax.text(0.02, 0.98, stats_text, transform=ax.transAxes, 
               fontsize=10, verticalalignment='top',
               bbox=dict(boxstyle='round,pad=0.5', facecolor='lightyellow', alpha=0.9))
    
    def _set_axis_limits(self, ax, filtered_nodes, start, goal):
        """Set appropriate axis limits"""
        if filtered_nodes:
            all_x = [n.state.x for n in filtered_nodes] + [start.x, goal.x]
            all_y = [n.state.y for n in filtered_nodes] + [start.y, goal.y]
            margin = 2.0
            ax.set_xlim(min(all_x) - margin, max(all_x) + margin)
            ax.set_ylim(min(all_y) - margin, max(all_y) + margin)
    
    def visualize_search_progress(self, path: List[State], start: State, goal: State,
                                explored_nodes: List[Node],
                                obstacle_map: Optional[np.ndarray] = None,
                                map_origin_x: float = 0, map_origin_y: float = 0,
                                grid_resolution: float = 1.0,
                                max_nodes_to_show: int = 500):
        """Enhanced visualization of the search progress with clear connections"""
        if not explored_nodes:
            print("No exploration data available")
            return
            
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 10))
        
        # Left plot: Search tree with connections
        self._plot_search_tree(ax1, path, start, goal, explored_nodes, 
                              obstacle_map, map_origin_x, map_origin_y, 
                              grid_resolution, max_nodes_to_show)
        
        # Right plot: Search progression over time
        self._plot_search_progression(ax2, path, start, goal, explored_nodes,
                                    obstacle_map, map_origin_x, map_origin_y,
                                    grid_resolution, max_nodes_to_show)
        
        plt.tight_layout()
        plt.show()
    
    def _plot_search_tree(self, ax, path, start, goal, explored_nodes,
                         obstacle_map, map_origin_x, map_origin_y, 
                         grid_resolution, max_nodes_to_show):
        """Plot search tree with clear parent-child connections"""
        # Plot obstacle map
        if obstacle_map is not None:
            map_height, map_width = obstacle_map.shape
            extent = (map_origin_x, 
                     map_origin_x + map_width * grid_resolution,
                     map_origin_y,
                     map_origin_y + map_height * grid_resolution)
            ax.imshow(obstacle_map, extent=extent, origin='lower', 
                     cmap='gray_r', alpha=0.8, vmin=0, vmax=1)
        
        # Limit nodes to show for performance
        nodes_to_show = min(len(explored_nodes), max_nodes_to_show)
        step = max(1, len(explored_nodes) // nodes_to_show)
        selected_nodes = explored_nodes[::step]
        
        # First pass: draw all parent-child connections
        print(f"Drawing search tree with {len(selected_nodes)} nodes...")
        connection_count = 0
        for node in selected_nodes:
            if node.parent is not None:
                # Check if parent is also in selected nodes (for cleaner visualization)
                parent_in_selection = any(abs(n.state.x - node.parent.state.x) < 0.1 and 
                                        abs(n.state.y - node.parent.state.y) < 0.1 
                                        for n in selected_nodes)
                
                if parent_in_selection or connection_count < 200:  # Limit connections for clarity
                    # Color connections by search depth/cost
                    depth_color = min(1.0, node.g_cost / 50.0)  # Normalize roughly
                    color = cm.get_cmap('plasma')(depth_color)
                    
                    ax.plot([node.parent.state.x, node.state.x], 
                           [node.parent.state.y, node.state.y], 
                           color=color, linewidth=1.2, alpha=0.6, zorder=1)
                    connection_count += 1
        
        # Second pass: draw nodes
        exploration_x = []
        exploration_y = []
        exploration_costs = []
        node_sizes = []
        
        for node in selected_nodes:
            exploration_x.append(node.state.x)
            exploration_y.append(node.state.y)
            exploration_costs.append(node.f_cost)
            # Larger nodes for important nodes (low f-cost)
            size = max(15, 50 - min(40, node.f_cost * 2))
            node_sizes.append(size)
        
        # Plot nodes with cost-based coloring
        if exploration_costs:
            scatter = ax.scatter(exploration_x, exploration_y, 
                               c=exploration_costs, cmap='viridis_r',  # Reverse so low cost = bright
                               s=node_sizes, alpha=0.8, zorder=3,
                               edgecolors='white', linewidths=1)
            
            # Colorbar
            cbar = plt.colorbar(scatter, ax=ax, shrink=0.6)
            cbar.set_label('F-cost (Lower=Better)', fontsize=10)
        
        # Highlight path nodes if available
        if path:
            path_x = [state.x for state in path]
            path_y = [state.y for state in path]
            ax.plot(path_x, path_y, 'red', linewidth=4, alpha=0.9, 
                   label='Final Path', zorder=4)
            ax.scatter(path_x, path_y, c='red', s=80, zorder=5, 
                      edgecolors='white', linewidths=2)
        
        # Start and goal
        ax.plot(start.x, start.y, 'go', markersize=18, label='Start', 
               zorder=10, markeredgecolor='white', markeredgewidth=2)
        ax.plot(goal.x, goal.y, 'ro', markersize=18, label='Goal', 
               zorder=10, markeredgecolor='white', markeredgewidth=2)
        
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title(f'Search Tree Structure\n({len(selected_nodes)} nodes, {connection_count} connections)', 
                    fontsize=12)
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
    
    def _plot_search_progression(self, ax, path, start, goal, explored_nodes,
                               obstacle_map, map_origin_x, map_origin_y,
                               grid_resolution, max_nodes_to_show):
        """Plot search progression with temporal coloring"""
        # Plot obstacle map (black = obstacles, white = free space)
        if obstacle_map is not None:
            map_height, map_width = obstacle_map.shape
            extent = (map_origin_x, 
                     map_origin_x + map_width * grid_resolution,
                     map_origin_y,
                     map_origin_y + map_height * grid_resolution)
            ax.imshow(obstacle_map, extent=extent, origin='lower', 
                     cmap='gray_r', alpha=0.8, vmin=0, vmax=1)
        
        # Show exploration progression with temporal color gradient
        nodes_to_show = min(len(explored_nodes), max_nodes_to_show)
        step = max(1, len(explored_nodes) // nodes_to_show)
        
        exploration_x = []
        exploration_y = []
        exploration_order = []
        
        for i in range(0, len(explored_nodes), step):
            node = explored_nodes[i]
            exploration_x.append(node.state.x)
            exploration_y.append(node.state.y)
            exploration_order.append(i / len(explored_nodes))  # Normalized time
        
        # Color by exploration order (early = blue, late = red)
        scatter = ax.scatter(exploration_x, exploration_y, 
                           c=exploration_order, cmap='coolwarm', 
                           s=30, alpha=0.8, zorder=2,
                           edgecolors='white', linewidths=0.5)
        
        # Add arrows showing search direction for some nodes
        arrow_step = max(1, len(exploration_x) // 20)
        for i in range(arrow_step, len(exploration_x), arrow_step):
            if i > 0:
                dx = exploration_x[i] - exploration_x[i-arrow_step]
                dy = exploration_y[i] - exploration_y[i-arrow_step]
                if abs(dx) > 0.1 or abs(dy) > 0.1:  # Only if meaningful movement
                    ax.arrow(exploration_x[i-arrow_step], exploration_y[i-arrow_step], 
                           dx * 0.7, dy * 0.7, 
                           head_width=0.3, head_length=0.3, 
                           fc='orange', ec='darkorange', alpha=0.7, zorder=3)
        
        # Plot final path
        if path:
            x_coords = [state.x for state in path]
            y_coords = [state.y for state in path]
            ax.plot(x_coords, y_coords, 'lime', linewidth=4, 
                   label='Final Path', zorder=4, alpha=0.9)
        
        # Start and goal
        ax.plot(start.x, start.y, 'go', markersize=18, label='Start', 
               zorder=10, markeredgecolor='white', markeredgewidth=2)
        ax.plot(goal.x, goal.y, 'ro', markersize=18, label='Goal', 
               zorder=10, markeredgecolor='white', markeredgewidth=2)
        
        ax.set_xlabel('X (m)', fontsize=12)
        ax.set_ylabel('Y (m)', fontsize=12)
        ax.set_title(f'Search Progression Over Time\n({len(exploration_x)} nodes shown)', fontsize=12)
        ax.legend(fontsize=10)
        ax.grid(True, alpha=0.3)
        ax.axis('equal')
        
        # Add colorbar for temporal progression
        cbar = plt.colorbar(scatter, ax=ax, shrink=0.6)
        cbar.set_label('Search Time (Blue=Early, Red=Late)', fontsize=10)
