#!/usr/bin/env python3
"""
Example demonstrating the simplified gradient-based path smoother.
Uses distance-based interpolation followed by optimization smoothing.
Includes interactive mode where you can drag points to modify the path.
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import os

# Add the parent directory to the path so we can import from src
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(parent_dir, 'src'))

from path_smoothing.gradient_based_smoother import GradientPathSmoother

class InteractivePathSmoother:
    """Interactive path smoother with draggable points."""
    
    def __init__(self, initial_path, smoother_params=None):
        """
        Initialize the interactive path smoother.
        
        Args:
            initial_path: List of (x, y) tuples for the initial path
            smoother_params: Dict of parameters for the GradientPathSmoother
        """
        self.original_path = list(initial_path)
        self.current_path = list(initial_path)
        
        # Default smoother parameters
        default_params = {
            'alpha': 0.3,
            'beta': 0.5,
            'target_distance': 0.1,
            'max_iterations': 300
        }
        if smoother_params:
            default_params.update(smoother_params)
        
        self.smoother = GradientPathSmoother(**default_params)
        
        # Interactive state
        self.selected_point = None
        self.is_dragging = False
        self.smoothed_path = None
        self.update_counter = 0
        self.update_frequency = 3  # Update smoothed path every N mouse movements
        
        # Setup the plot
        self.setup_plot()
        self.update_smoothed_path()
        self.update_plot()
        
    def setup_plot(self):
        """Setup the interactive matplotlib plot."""
        self.fig, self.ax = plt.subplots(figsize=(14, 10))
        self.fig.suptitle('Interactive Path Smoother - Drag points to modify path', fontsize=14)
        
        # Create empty line objects for the different paths
        self.original_line, = self.ax.plot([], [], 'ro-', label='Original Path', 
                                         linewidth=2, markersize=8, picker=True, pickradius=10)
        self.smoothed_line, = self.ax.plot([], [], 'g-', label='Smoothed Path', 
                                         linewidth=3, alpha=0.8)
        
        self.ax.set_xlabel('X', fontsize=12)
        self.ax.set_ylabel('Y', fontsize=12)
        self.ax.legend(fontsize=11)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_aspect('equal')
        
        # Connect event handlers
        self.fig.canvas.mpl_connect('pick_event', self.on_pick)
        self.fig.canvas.mpl_connect('button_release_event', self.on_release)
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_motion)
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        
        # Add instructions
        instructions = ('Instructions:\n'
                       '• Click and drag red points to modify path\n'
                       '• Path will re-smooth automatically\n'
                       '• Press "r" to reset to original path\n'
                       '• Press "q" to quit\n'
                       f'• Smoothing params: α={self.smoother.alpha}, β={self.smoother.beta}, '
                       f'distance={self.smoother.target_distance}')
        
        self.ax.text(0.02, 0.98, instructions, 
                    transform=self.ax.transAxes, fontsize=9, verticalalignment='top',
                    bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.8))
    
    def update_smoothed_path(self):
        """Update the smoothed path based on current path."""
        try:
            self.smoothed_path = self.smoother.smooth_path(self.current_path)
            print(f"Smoothing completed - {len(self.smoothed_path)} points generated")
        except Exception as e:
            print(f"Smoothing failed: {e}")
            self.smoothed_path = self.current_path
    
    def update_plot(self):
        """Update the plot with current paths."""
        # Update original path
        if self.current_path:
            path_array = np.array(self.current_path)
            self.original_line.set_data(path_array[:, 0], path_array[:, 1])
        
        # Update smoothed path
        if self.smoothed_path:
            smoothed_array = np.array(self.smoothed_path)
            self.smoothed_line.set_data(smoothed_array[:, 0], smoothed_array[:, 1])
        
        # Auto-scale the plot
        self.ax.relim()
        self.ax.autoscale()
        
        # Redraw
        self.fig.canvas.draw_idle()
    
    def on_pick(self, event):
        """Handle point selection."""
        if event.artist == self.original_line:
            # Find which point was clicked
            ind = event.ind[0] if event.ind else None
            if ind is not None and ind < len(self.current_path):
                self.selected_point = ind
                self.is_dragging = True
                print(f"Selected point {ind}: ({self.current_path[ind][0]:.2f}, {self.current_path[ind][1]:.2f})")
    
    def on_release(self, event):
        """Handle mouse release."""
        if self.is_dragging:
            self.is_dragging = False
            # Final update when dragging ends
            self.update_smoothed_path()
            self.update_plot()
            self.selected_point = None
            print("Point released - final smoothing completed")
    
    def on_motion(self, event):
        """Handle mouse motion for dragging."""
        if self.is_dragging and self.selected_point is not None and event.inaxes == self.ax:
            # Update the selected point position
            self.current_path[self.selected_point] = (event.xdata, event.ydata)
            
            # Update counter for reduced frequency updates
            self.update_counter += 1
            
            # Update smoothed path only every N movements for better performance
            if self.update_counter % self.update_frequency == 0:
                self.update_smoothed_path()
            
            # Always update the visual display
            self.update_plot()
    
    def on_key_press(self, event):
        """Handle keyboard events."""
        if event.key == 'r':
            print("Resetting to original path...")
            self.reset_path()
        elif event.key == 'q':
            print("Quitting...")
            plt.close(self.fig)
    
    def reset_path(self):
        """Reset to original path."""
        self.current_path = list(self.original_path)
        self.update_smoothed_path()
        self.update_plot()
        print("Path reset to original")
    
    def get_current_path(self):
        """Get the current modified path."""
        return list(self.current_path)
    
    def get_smoothed_path(self):
        """Get the current smoothed path."""
        return list(self.smoothed_path) if self.smoothed_path else []

def create_demo_path():
    """Create a demo path with sharp turns that benefit from smoothing."""
    x_points = [0.0, 2.0, 4.0, 6.0, 8.0, 10.0, 12.0]
    y_points = [0.0, 3.0, 1.0, 4.0, 0.0, 2.0, 1.0]
    return list(zip(x_points, y_points))

def create_circular_path():
    """Create a circular path with some irregularities."""
    angles = np.linspace(0, 2*np.pi, 8, endpoint=False)
    radius = 4
    x_points = radius * np.cos(angles)
    y_points = radius * np.sin(angles)
    
    # Add some irregularities to make it more interesting
    x_points += np.array([0, 1, -0.5, 1.5, -1, 0.5, -1.5, 0.8])
    y_points += np.array([0, 0.5, 1, -0.5, 1.2, -1, 0.3, -0.8])
    
    return list(zip(x_points, y_points))

def run_interactive_demo():
    """Run the interactive path smoothing demo."""
    print("=== Interactive Path Smoother Demo ===")
    print("Unified mode: Distance-based interpolation + optimization smoothing")
    print("You can drag the red points to modify the path\n")
    
    # Let user choose path type
    print("Choose path type:")
    print("1. Simple demo path (lines and corners)")
    print("2. Circular path with irregularities")
    
    try:
        path_choice = input("Please select path type (1/2, default is 2): ").strip()
        if not path_choice:
            path_choice = "2"
    except:
        path_choice = "2"
    
    if path_choice == "1":
        demo_path = create_demo_path()
    else:
        demo_path = create_circular_path()
    
    # Setup smoother parameters
    smoother_params = {
        'alpha': 0.3,           # Smoothness weight
        'beta': 0.3,            # Similarity weight (reduced for more smoothing)
        'target_distance': 0.15, # Distance between interpolated points
        'max_iterations': 300   # Optimization iterations
    }
    
    # Create interactive smoother
    interactive_smoother = InteractivePathSmoother(demo_path, smoother_params)
    
    # Show the plot
    plt.show()
    
    # After closing, print final results
    print("\n=== Final Results ===")
    final_path = interactive_smoother.get_current_path()
    smoothed_path = interactive_smoother.get_smoothed_path()
    
    print(f"Modified path point count: {len(final_path)}")
    print(f"Smoothed path point count: {len(smoothed_path)}")
    
    # Calculate some statistics
    if len(final_path) > 1:
        final_array = np.array(final_path)
        distances = np.sqrt(np.sum(np.diff(final_array, axis=0)**2, axis=1))
        print(f"Total path length: {np.sum(distances):.2f}")
        print(f"Average segment length: {np.mean(distances):.2f}")
    
    if len(smoothed_path) > 1:
        smooth_array = np.array(smoothed_path)
        smooth_distances = np.sqrt(np.sum(np.diff(smooth_array, axis=0)**2, axis=1))
        print(f"Smoothed path total length: {np.sum(smooth_distances):.2f}")
        print(f"Smoothed path average segment length: {np.mean(smooth_distances):.2f}")

def run_static_demo():
    """Run a static demonstration of the path smoother."""
    print("=== Static Path Smoother Demo ===")
    print("Demonstrating the unified distance-based smoothing approach\n")
    
    # Create test paths
    demo_path = create_demo_path()
    circular_path = create_circular_path()
    
    # Test different parameter combinations
    test_configs = [
        {'alpha': 0.3, 'beta': 0.0, 'target_distance': 0.05, 'name': 'High Detail'},
        {'alpha': 0.5, 'beta': 0.0, 'target_distance': 0.05, 'name': 'Smooth & Fast'},
        {'alpha': 0.2, 'beta': 0.0, 'target_distance': 0.05, 'name': 'Conservative'},
    ]
    
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    fig.suptitle('Gradient Path Smoother - Different Parameter Settings', fontsize=14)
    
    for i, config in enumerate(test_configs):
        for j, (path, path_name) in enumerate([(demo_path, 'Demo Path'), (circular_path, 'Circular Path')]):
            ax = axes[j, i]
            
            # Create smoother with current config
            smoother = GradientPathSmoother(
                alpha=config['alpha'],
                beta=config['beta'], 
                target_distance=config['target_distance'],
                max_iterations=500
            )
            
            # Smooth the path
            smoothed_path = smoother.smooth_path(path)
            
            # Plot original and smoothed paths
            orig_arr = np.array(path)
            smooth_arr = np.array(smoothed_path)
            
            ax.plot(orig_arr[:, 0], orig_arr[:, 1], 'ro-', label='Original', 
                   linewidth=2, markersize=6)
            ax.plot(smooth_arr[:, 0], smooth_arr[:, 1], 'g-', label='Smoothed', 
                   linewidth=2)
            
            ax.set_title(f'{config["name"]}\n{path_name}')
            ax.set_xlabel('X')
            ax.set_ylabel('Y')
            ax.legend()
            ax.grid(True, alpha=0.3)
            ax.set_aspect('equal')
            
            # Print statistics
            print(f"{config['name']} - {path_name}:")
            print(f"  Original points: {len(path)}, Smoothed points: {len(smoothed_path)}")
            print(f"  Alpha: {config['alpha']}, Beta: {config['beta']}, Distance: {config['target_distance']}")
    
    plt.tight_layout()
    plt.show()

def main():
    """Main function with option to choose demo mode."""
    print("Simplified Gradient Path Smoother - Unified Mode")
    print("Uses distance-based interpolation followed by optimization smoothing\n")
    
    print("Choose demo mode:")
    print("1. Interactive demo (draggable points)")
    print("2. Static comparison demo")
    print("3. Run both demos")
    
    try:
        choice = input("Please enter your choice (1/2/3, default is 1): ").strip()
        if not choice:
            choice = "1"
    except:
        choice = "1"
    
    if choice == "1":
        run_interactive_demo()
    elif choice == "2":
        run_static_demo()
    elif choice == "3":
        print("\nFirst running interactive demo...")
        run_interactive_demo()
        print("\nNext running static comparison demo...")
        run_static_demo()
    else:
        print("Invalid choice, running interactive demo...")
        run_interactive_demo()

if __name__ == "__main__":
    main() 