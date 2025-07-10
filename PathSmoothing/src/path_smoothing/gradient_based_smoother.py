import numpy as np
from typing import List, Tuple
from scipy.interpolate import interp1d
from scipy.optimize import minimize
import warnings

class GradientPathSmoother:
    """
    A simplified path smoother that uses distance-based interpolation followed by 
    scipy optimization to minimize a cost function balancing path smoothness 
    and deviation from the original path.
    """
    
    def __init__(self, 
                 alpha: float = 0.3,      # Weight for path smoothness
                 beta: float = 0.5,       # Weight for path similarity
                 target_distance: float = 0.1,  # Fixed distance interval for interpolation
                 max_iterations: int = 500):
        """
        Initialize the path smoother.
        
        Args:
            alpha: Weight for path smoothness term (higher = smoother)
            beta: Weight for path similarity term (higher = closer to original)
            target_distance: Fixed distance interval for interpolation
            max_iterations: Maximum number of optimization iterations
        """
        self.alpha = alpha
        self.beta = beta
        self.target_distance = target_distance
        self.max_iterations = max_iterations

    def _interpolate_path(self, path: np.ndarray) -> np.ndarray:
        """
        Interpolate the path to increase point density using distance-based interpolation.
        
        Args:
            path: Original path as numpy array of shape (n, 2)
            
        Returns:
            Interpolated path with uniform distance spacing
        """
        if len(path) < 2:
            return path
            
        # Calculate cumulative distance along the path
        distances = np.sqrt(np.sum(np.diff(path, axis=0)**2, axis=1))
        cumulative_distances = np.concatenate([[0], np.cumsum(distances)])
        total_distance = cumulative_distances[-1]
        
        # Create new parameter values based on fixed distance intervals
        num_new_points = int(np.ceil(total_distance / self.target_distance)) + 1
        new_params = np.linspace(0, total_distance, num_new_points)
        
        # Interpolate x and y coordinates using linear interpolation
        interp_x = interp1d(cumulative_distances, path[:, 0], kind='linear')
        interp_y = interp1d(cumulative_distances, path[:, 1], kind='linear')
        
        new_x = interp_x(new_params)
        new_y = interp_y(new_params)
        
        return np.column_stack([new_x, new_y])

    def _objective_function(self, path_flat: np.ndarray, original_path: np.ndarray) -> float:
        """
        Compute the objective function to minimize.
        
        Args:
            path_flat: Flattened path array (optimization variable)
            original_path: Original interpolated path for similarity term
            
        Returns:
            Objective function value
        """
        # Reshape flat path back to 2D
        path = path_flat.reshape(-1, 2)
        
        # Smoothness term: sum of squared second derivatives
        if len(path) >= 3:
            second_derivatives = path[2:] - 2 * path[1:-1] + path[:-2]
            smoothness_cost = np.sum(np.square(second_derivatives))
        else:
            smoothness_cost = 0.0
        
        # Similarity term: sum of squared deviations from original path
        similarity_cost = np.sum(np.square(path - original_path))
        
        # Combined objective
        return self.alpha * smoothness_cost + self.beta * similarity_cost

    def smooth_path(self, original_path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Smooth the input path using distance-based interpolation followed by optimization.
        
        Args:
            original_path: List of (x, y) coordinates representing the original path
            
        Returns:
            Smoothed path as a list of (x, y) coordinates
        """
        # Convert input path to numpy array
        original_path_array = np.array(original_path)
        
        # Step 1: Interpolate the path to increase density using distance-based method
        interpolated_path = self._interpolate_path(original_path_array)
        
        # Store start and end points to keep them fixed
        start_point = interpolated_path[0].copy()
        end_point = interpolated_path[-1].copy()

        initial_path = interpolated_path
        
        # Flatten path for optimization
        initial_path_flat = initial_path.flatten()
        
        # Create equality constraints to fix start and end points
        def start_x_constraint(x): return x[0] - start_point[0]
        def start_y_constraint(x): return x[1] - start_point[1]
        def end_x_constraint(x): return x[-2] - end_point[0]
        def end_y_constraint(x): return x[-1] - end_point[1]
        
        constraints = [
            {'type': 'eq', 'fun': start_x_constraint},
            {'type': 'eq', 'fun': start_y_constraint},
            {'type': 'eq', 'fun': end_x_constraint},
            {'type': 'eq', 'fun': end_y_constraint}
        ]
        
        # Optimization options with improved precision
        options = {
            'maxiter': self.max_iterations,
            'ftol': 1e-12,  # Much stricter convergence tolerance
            'disp': False
        }
        
        # Run optimization with suppressed warnings
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            
            # Use SLSQP method which properly supports equality constraints
            result = minimize(
                fun=self._objective_function,
                x0=initial_path_flat,
                args=(interpolated_path,),
                method='SLSQP',
                constraints=constraints,
                options=options
            )
        
        # Print convergence information
        if result.success:
            print(f"Path smoothing converged in {result.nit} iterations")
        else:
            print(f"Path smoothing warning: {result.message}")
        
        # Reshape optimized path back to 2D
        optimized_path = result.x.reshape(-1, 2)
        
        # Ensure endpoints are exactly fixed
        optimized_path[0] = start_point
        optimized_path[-1] = end_point
        
        return [(float(point[0]), float(point[1])) for point in optimized_path]

    def get_path_curvature(self, path: List[Tuple[float, float]]) -> List[float]:
        """
        Calculate the curvature at each point in the path.
        
        Args:
            path: List of (x, y) coordinates
            
        Returns:
            List of curvature values
        """
        path_array = np.array(path)
        dx = np.gradient(path_array[:, 0])
        dy = np.gradient(path_array[:, 1])
        ddx = np.gradient(dx)
        ddy = np.gradient(dy)
        
        # Avoid division by zero
        denominator = (dx * dx + dy * dy) ** 1.5
        denominator = np.where(denominator == 0, 1e-10, denominator)
        
        curvature = np.abs(dx * ddy - dy * ddx) / denominator
        return curvature.tolist() 