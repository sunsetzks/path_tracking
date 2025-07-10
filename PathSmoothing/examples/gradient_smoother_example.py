#!/usr/bin/env python3
"""
Example demonstrating the gradient-based path smoother with interpolation.
This example shows how interpolation before scipy optimization improves smoothing results.
Now includes interactive mode where you can drag points to modify the path.
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
            'distance_based': True,
            'target_distance': 0.1,
            'method': 'L-BFGS-B',
            'max_iterations': 500
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
                       f'• Smoothing params: α={self.smoother.alpha}, β={self.smoother.beta}')
        
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
        self.fig.canvas.draw_idle()  # Use draw_idle for better performance
    
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

def create_zigzag_path():
    """Create a simple zigzag path for testing."""
    x_points = [0.0, 1.0, 2.0, 3.0, 4.0, 5.0]
    y_points = [0.0, 1.0, 0.0, 1.0, 0.0, 1.0]
    return list(zip(x_points, y_points))

def create_curved_path():
    """Create a curved path with varying point density."""
    # Create a path with non-uniform spacing
    angles = np.array([0, 0.5, 1.2, 2.0, 3.0, 4.5, 6.0])
    x_points = 5 * np.cos(angles)
    y_points = 5 * np.sin(angles)
    return list(zip(x_points, y_points))

def create_demo_path():
    """Create a demo path that's good for interactive editing."""
    # Create a path with some sharp turns that benefit from smoothing
    x_points = [0, 2, 4, 6, 8, 10, 12]
    y_points = [0, 3, 1, 4, 0, 2, 1]
    return list(zip(x_points, y_points))

def create_interesting_path():
    """Create a more interesting path with curves and corners."""
    # Create a path that resembles a track or circuit
    angles = np.linspace(0, 2*np.pi, 8, endpoint=False)
    radius = 4
    x_points = radius * np.cos(angles)
    y_points = radius * np.sin(angles)
    
    # Add some irregularities to make it more interesting
    x_points += np.array([0, 1, -0.5, 1.5, -1, 0.5, -1.5, 0.8])
    y_points += np.array([0, 0.5, 1, -0.5, 1.2, -1, 0.3, -0.8])
    
    return list(zip(x_points, y_points))

def plot_comparison(original_path, interpolated_path, smoothed_path, title):
    """Plot original, interpolated, and smoothed paths for comparison."""
    plt.figure(figsize=(12, 8))
    
    # Convert paths to arrays for plotting
    orig_arr = np.array(original_path)
    interp_arr = np.array(interpolated_path)
    smooth_arr = np.array(smoothed_path)
    
    plt.plot(orig_arr[:, 0], orig_arr[:, 1], 'ro-', label='Original Path', linewidth=2, markersize=8)
    plt.plot(interp_arr[:, 0], interp_arr[:, 1], 'b.-', label='Interpolated Path', linewidth=1, markersize=4, alpha=0.7)
    plt.plot(smooth_arr[:, 0], smooth_arr[:, 1], 'g-', label='Smoothed Path', linewidth=2)
    
    plt.title(title)
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')

def run_interactive_demo():
    """Run the interactive path smoothing demo."""
    print("=== 交互式路径平滑器演示 ===")
    print("你可以拖动红色点来修改路径，系统会自动重新平滑")
    print("关闭窗口结束演示\n")
    
    # Let user choose path type
    print("选择路径类型:")
    print("1. 简单演示路径 (直线和转角)")
    print("2. 有趣的环形路径")
    
    try:
        path_choice = input("请选择路径类型 (1/2, 默认为2): ").strip()
        if not path_choice:
            path_choice = "2"
    except:
        path_choice = "2"
    
    if path_choice == "1":
        demo_path = create_demo_path()
    else:
        demo_path = create_interesting_path()
    
    # Setup smoother parameters
    smoother_params = {
        'alpha': 0.3,
        'beta': 0.3,  # Reduced beta for more smoothing
        'distance_based': True,
        'target_distance': 0.15,  # Smaller for more detail
        'method': 'L-BFGS-B',
        'max_iterations': 300  # Reduced for faster interactive response
    }
    
    # Create interactive smoother
    interactive_smoother = InteractivePathSmoother(demo_path, smoother_params)
    
    # Show the plot
    plt.show()
    
    # After closing, print final paths
    print("\n=== 最终结果 ===")
    final_path = interactive_smoother.get_current_path()
    smoothed_path = interactive_smoother.get_smoothed_path()
    
    print(f"修改后的路径点数: {len(final_path)}")
    print(f"平滑后的路径点数: {len(smoothed_path)}")
    
    # Calculate some statistics
    if len(final_path) > 1:
        final_array = np.array(final_path)
        distances = np.sqrt(np.sum(np.diff(final_array, axis=0)**2, axis=1))
        print(f"路径总长度: {np.sum(distances):.2f}")
        print(f"平均段长度: {np.mean(distances):.2f}")
    
    if len(smoothed_path) > 1:
        smooth_array = np.array(smoothed_path)
        smooth_distances = np.sqrt(np.sum(np.diff(smooth_array, axis=0)**2, axis=1))
        print(f"平滑路径总长度: {np.sum(smooth_distances):.2f}")
        print(f"平滑路径平均段长度: {np.mean(smooth_distances):.2f}")

def run_static_demos():
    """Run the original static demonstration."""
    # Create test paths
    zigzag_path = create_zigzag_path()
    curved_path = create_curved_path()
    
    print("=== 梯度路径平滑器示例 ===")
    print("演示基于距离的插值功能\n")
    
    # Test 1: Factor-based interpolation (original method)
    print("1. 使用插值因子的方法 (interpolation_factor=3)")
    smoother1 = GradientPathSmoother(
        alpha=0.3,
        beta=0.5,
        interpolation_factor=10,
        distance_based=False,
        method='L-BFGS-B',
        max_iterations=500
    )
    
    interpolated1 = smoother1.get_interpolated_path(zigzag_path)
    smoothed1 = smoother1.smooth_path(zigzag_path)
    
    print(f"原始路径点数: {len(zigzag_path)}")
    print(f"插值后点数: {len(interpolated1)}")
    print(f"平滑后点数: {len(smoothed1)}")
    
    # Test 2: Distance-based interpolation with fixed distance
    print("\n2. 使用固定距离间隔的插值 (target_distance=0.3)")
    smoother2 = GradientPathSmoother(
        alpha=0.3,
        beta=0.0,
        distance_based=True,
        target_distance=0.05,
        method='L-BFGS-B',
        max_iterations=5000
    )
    
    interpolated2 = smoother2.get_interpolated_path(zigzag_path)
    smoothed2 = smoother2.smooth_path(zigzag_path)
    
    print(f"原始路径点数: {len(zigzag_path)}")
    print(f"插值后点数: {len(interpolated2)}")
    print(f"平滑后点数: {len(smoothed2)}")
    
    # Test 3: Distance-based interpolation with auto distance
    print("\n3. 使用自动计算距离间隔的插值 (distance_based=True, auto)")
    smoother3 = GradientPathSmoother(
        alpha=0.3,
        beta=0.5,
        distance_based=True,
        interpolation_factor=4,  # Used for auto distance calculation
        method='L-BFGS-B',
        max_iterations=500
    )
    
    interpolated3 = smoother3.get_interpolated_path(curved_path)
    smoothed3 = smoother3.smooth_path(curved_path)
    
    # Calculate average distance for reference
    curved_arr = np.array(curved_path)
    distances = np.sqrt(np.sum(np.diff(curved_arr, axis=0)**2, axis=1))
    avg_distance = np.mean(distances)
    auto_distance = avg_distance / 4
    
    print(f"原始路径点数: {len(curved_path)}")
    print(f"平均距离: {avg_distance:.3f}")
    print(f"自动计算的目标距离: {auto_distance:.3f}")
    print(f"插值后点数: {len(interpolated3)}")
    print(f"平滑后点数: {len(smoothed3)}")
    
    # Plotting
    plot_comparison(zigzag_path, interpolated1, smoothed1, 
                   "Factor-based Interpolation (factor=3)")
    
    plot_comparison(zigzag_path, interpolated2, smoothed2, 
                   "Distance-based Interpolation (distance=0.3)")
    
    plot_comparison(curved_path, interpolated3, smoothed3, 
                   "Auto Distance-based Interpolation")
    
    plt.show()
    
    # Distance analysis
    print("\n=== 距离分析 ===")
    for i, (name, path) in enumerate([
        ("Factor-based", interpolated1),
        ("Fixed distance", interpolated2), 
        ("Auto distance", interpolated3)
    ]):
        path_arr = np.array(path)
        distances = np.sqrt(np.sum(np.diff(path_arr, axis=0)**2, axis=1))
        print(f"{name}: 平均距离={np.mean(distances):.3f}, "
              f"标准差={np.std(distances):.3f}, "
              f"最小距离={np.min(distances):.3f}, "
              f"最大距离={np.max(distances):.3f}")

def main():
    """Main function with option to choose demo mode."""
    print("选择演示模式:")
    print("1. 交互式演示 (可拖动点)")
    print("2. 静态对比演示")
    print("3. 同时运行两种演示")
    
    try:
        choice = input("请输入选择 (1/2/3, 默认为1): ").strip()
        if not choice:
            choice = "1"
    except:
        choice = "1"
    
    if choice == "1":
        run_interactive_demo()
    elif choice == "2":
        run_static_demos()
    elif choice == "3":
        print("\n首先运行交互式演示...")
        run_interactive_demo()
        print("\n接下来运行静态对比演示...")
        run_static_demos()
    else:
        print("无效选择，运行交互式演示...")
        run_interactive_demo()

if __name__ == "__main__":
    main() 