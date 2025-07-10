import numpy as np
from typing import List, Tuple, Optional
from scipy.interpolate import interp1d
from scipy.optimize import minimize
import warnings

class GradientPathSmoother:
    """
    A path smoother using scipy optimization to minimize a cost function that balances
    path smoothness and deviation from the original path.
    The start and end points remain fixed during optimization.
    Includes interpolation to increase path density before optimization.
    """
    def __init__(self, 
                 alpha: float = 0.1,    # Weight for path smoothness
                 beta: float = 0.2,     # Weight for path similarity
                 method: str = 'L-BFGS-B',  # Optimization method
                 max_iterations: int = 1000,
                 tolerance: float = 1e-6,
                 fix_endpoints: bool = True,
                 interpolation_factor: int = 3,  # Number of points to interpolate between each pair
                 interpolation_method: str = 'linear',  # Interpolation method
                 distance_based: bool = False,  # Whether to use fixed distance intervals
                 target_distance: Optional[float] = None):  # Fixed distance interval for distance-based interpolation
        """
        Initialize the path smoother.
        
        Args:
            alpha: Weight for path smoothness term
            beta: Weight for path similarity term
            method: Optimization method ('L-BFGS-B', 'BFGS', 'CG', 'TNC', etc.)
            max_iterations: Maximum number of iterations
            tolerance: Tolerance for optimization convergence
            fix_endpoints: Whether to keep start and end points fixed
            interpolation_factor: Number of interpolated points between each original point pair (when distance_based=False)
            interpolation_method: Interpolation method ('linear', 'cubic', 'quadratic')
            distance_based: If True, use fixed distance intervals; if False, use interpolation_factor
            target_distance: Fixed distance interval for interpolation (only used when distance_based=True)
        """
        self.alpha = alpha
        self.beta = beta
        self.method = method
        self.max_iterations = max_iterations
        self.tolerance = tolerance
        self.fix_endpoints = fix_endpoints
        self.interpolation_factor = interpolation_factor
        self.interpolation_method = interpolation_method
        self.distance_based = distance_based
        self.target_distance = target_distance

    def _calculate_average_distance(self, path: np.ndarray) -> float:
        """
        Calculate the average distance between consecutive points in the path.
        
        Args:
            path: Path as numpy array of shape (n, 2)
            
        Returns:
            Average distance between consecutive points
        """
        if len(path) < 2:
            return 0.0
        
        distances = np.sqrt(np.sum(np.diff(path, axis=0)**2, axis=1))
        return np.mean(distances)

    def _interpolate_path(self, path: np.ndarray) -> np.ndarray:
        """
        Interpolate the path to increase point density using distance-based interpolation.
        
        Args:
            path: Original path as numpy array of shape (n, 2)
            
        Returns:
            Interpolated path with increased density
        """
        if len(path) < 2:
            return path
            
        # Calculate cumulative distance along the path
        distances = np.sqrt(np.sum(np.diff(path, axis=0)**2, axis=1))
        cumulative_distances = np.concatenate([[0], np.cumsum(distances)])
        total_distance = cumulative_distances[-1]
        
        # Create new parameter values for interpolation based on the mode
        if self.distance_based:
            # Use fixed distance intervals
            if self.target_distance is not None:
                target_dist = self.target_distance
            else:
                # Auto-calculate target distance as average distance divided by interpolation factor
                avg_distance = self._calculate_average_distance(path)
                target_dist = avg_distance / self.interpolation_factor
            
            num_new_points = int(np.ceil(total_distance / target_dist)) + 1
            new_params = np.linspace(0, total_distance, num_new_points)
        else:
            # Use interpolation factor (original behavior)
            num_original_points = len(path)
            num_new_points = (num_original_points - 1) * self.interpolation_factor + 1
            new_params = np.linspace(0, total_distance, num_new_points)
        
        # Interpolate x and y coordinates separately
        try:
            # Try cubic interpolation first
            if self.interpolation_method == 'cubic' and len(path) >= 4:
                interp_x = interp1d(cumulative_distances, path[:, 0], kind='cubic')
                interp_y = interp1d(cumulative_distances, path[:, 1], kind='cubic')
            elif self.interpolation_method == 'quadratic' and len(path) >= 3:
                interp_x = interp1d(cumulative_distances, path[:, 0], kind='quadratic')
                interp_y = interp1d(cumulative_distances, path[:, 1], kind='quadratic')
            else:
                # Fall back to linear interpolation
                interp_x = interp1d(cumulative_distances, path[:, 0], kind='linear')
                interp_y = interp1d(cumulative_distances, path[:, 1], kind='linear')
                
            new_x = interp_x(new_params)
            new_y = interp_y(new_params)
            
            return np.column_stack([new_x, new_y])
            
        except Exception:
            # If interpolation fails, fall back to linear
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

    def _create_constraints(self, path_shape: Tuple[int, int]) -> List[dict]:
        """
        Create constraints to fix endpoints if required.
        
        Args:
            path_shape: Shape of the path array (n_points, 2)
            
        Returns:
            List of constraint dictionaries for scipy.optimize
        """
        constraints = []
        
        if self.fix_endpoints:
            n_points, n_dims = path_shape
            
            # Fix first point (start)
            def start_x_constraint(x): return x[0]  # x[0] should be original start x
            def start_y_constraint(x): return x[1]  # x[1] should be original start y
            
            # Fix last point (end)
            def end_x_constraint(x): return x[-2]  # x[-2] should be original end x
            def end_y_constraint(x): return x[-1]  # x[-1] should be original end y
            
            constraints.extend([
                {'type': 'eq', 'fun': start_x_constraint},
                {'type': 'eq', 'fun': start_y_constraint},
                {'type': 'eq', 'fun': end_x_constraint},
                {'type': 'eq', 'fun': end_y_constraint}
            ])
        
        return constraints

    def smooth_path(self, 
                   original_path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Smooth the input path using interpolation followed by scipy optimization.
        
        Args:
            original_path: List of (x, y) coordinates representing the original path
            
        Returns:
            Smoothed path as a list of (x, y) coordinates
        """
        # Convert input path to numpy array for easier computation
        original_path_array = np.array(original_path)
        
        # Step 1: Interpolate the path to increase density
        interpolated_path = self._interpolate_path(original_path_array)
        
        # Store original start and end points for constraints
        start_point = interpolated_path[0].copy()
        end_point = interpolated_path[-1].copy()
        
        # Flatten path for optimization (scipy expects 1D array)
        initial_path_flat = interpolated_path.flatten()
        
        # Create constraints if endpoints should be fixed
        if self.fix_endpoints:
            constraints = []
            n_points = len(interpolated_path)
            
            # Create equality constraints for start and end points
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
        else:
            constraints = []
        
        # Set up optimization options
        options = {
            'maxiter': self.max_iterations,
            'ftol': self.tolerance,
            'disp': False
        }
        
        # Suppress optimization warnings for cleaner output
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            
            # Run optimization
            result = minimize(
                fun=self._objective_function,
                x0=initial_path_flat,
                args=(interpolated_path,),
                method=self.method,
                constraints=constraints,
                options=options
            )
        
        # Print convergence information
        if result.success:
            print(f"Optimization converged successfully in {result.nit} iterations")
        else:
            print(f"Optimization did not converge: {result.message}")
        
        # Reshape optimized path back to 2D
        optimized_path = result.x.reshape(-1, 2)
        
        # Ensure endpoints are exactly fixed (numerical precision)
        if self.fix_endpoints:
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

    def get_interpolated_path(self, original_path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Get the interpolated path without smoothing (for debugging/visualization).
        
        Args:
            original_path: List of (x, y) coordinates representing the original path
            
        Returns:
            Interpolated path as a list of (x, y) coordinates
        """
        original_path_array = np.array(original_path)
        interpolated_path = self._interpolate_path(original_path_array)
        return [(float(point[0]), float(point[1])) for point in interpolated_path] 