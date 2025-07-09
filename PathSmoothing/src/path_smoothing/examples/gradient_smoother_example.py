import numpy as np
import matplotlib.pyplot as plt
from path_smoothing import GradientPathSmoother

def generate_sample_path():
    """Generate a sample path with some sharp turns."""
    t = np.linspace(0, 2*np.pi, 100)
    x = t + 0.5 * np.sin(2*t)
    y = np.sin(t) + 0.2 * np.random.randn(len(t))  # Add some noise
    return list(zip(x, y))

def main():
    # Generate a sample path
    original_path = generate_sample_path()
    
    # Create path smoother instance
    smoother = GradientPathSmoother(
        alpha=0.1,  # Weight for smoothness
        beta=0.2,   # Weight for similarity to original path
        learning_rate=0.01,
        max_iterations=1000
    )
    
    # Smooth the path
    smoothed_path = smoother.smooth_path(original_path)
    
    # Calculate curvature
    original_curvature = smoother.get_path_curvature(original_path)
    smoothed_curvature = smoother.get_path_curvature(smoothed_path)
    
    # Plotting
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 10))
    
    # Plot paths
    original_path = np.array(original_path)
    smoothed_path = np.array(smoothed_path)
    
    ax1.plot(original_path[:, 0], original_path[:, 1], 'b-', label='Original Path')
    ax1.plot(smoothed_path[:, 0], smoothed_path[:, 1], 'r-', label='Smoothed Path')
    ax1.set_title('Path Comparison')
    ax1.legend()
    ax1.grid(True)
    
    # Plot curvature
    ax2.plot(original_curvature, 'b-', label='Original Curvature')
    ax2.plot(smoothed_curvature, 'r-', label='Smoothed Curvature')
    ax2.set_title('Curvature Comparison')
    ax2.legend()
    ax2.grid(True)
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main() 