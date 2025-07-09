import numpy as np
from typing import List, Tuple, Optional
from scipy.interpolate import interp1d

class GradientPathSmoother:
    """
    A path smoother using gradient descent to minimize a cost function that balances
    path smoothness and deviation from the original path.
    The start and end points remain fixed during optimization.
    Includes interpolation to increase path density before optimization.
    """
    def __init__(self, 
                 alpha: float = 0.1,    # Weight for path smoothness
                 beta: float = 0.2,     # Weight for path similarity
                 learning_rate: float = 0.01,
                 max_iterations: int = 1000,
                 convergence_threshold: float = 1e-6,
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
            learning_rate: Learning rate for gradient descent
            max_iterations: Maximum number of iterations
            convergence_threshold: Threshold for convergence check
            fix_endpoints: Whether to keep start and end points fixed
            interpolation_factor: Number of interpolated points between each original point pair (when distance_based=False)
            interpolation_method: Interpolation method ('linear', 'cubic', 'quadratic')
            distance_based: If True, use fixed distance intervals; if False, use interpolation_factor
            target_distance: Fixed distance interval for interpolation (only used when distance_based=True)
        """
        self.alpha = alpha
        self.beta = beta
        self.learning_rate = learning_rate
        self.max_iterations = max_iterations
        self.convergence_threshold = convergence_threshold
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

    def _compute_smoothness_gradient(self, path: np.ndarray) -> np.ndarray:
        """
        Compute the gradient of the smoothness term.
        The smoothness term penalizes sharp turns in the path.
        """
        # Calculate second derivatives (approximation)
        second_derivatives = path[2:] - 2 * path[1:-1] + path[:-2]
        
        # Initialize gradient array
        gradient = np.zeros_like(path)
        
        # Fill in the gradient for interior points
        gradient[2:] += second_derivatives
        gradient[1:-1] += -2 * second_derivatives
        gradient[:-2] += second_derivatives
        
        return gradient

    def _compute_similarity_gradient(self, 
                                   current_path: np.ndarray, 
                                   original_path: np.ndarray) -> np.ndarray:
        """
        Compute the gradient of the similarity term.
        The similarity term penalizes deviation from the original path.
        """
        return 2 * (current_path - original_path)

    def smooth_path(self, 
                   original_path: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Smooth the input path using interpolation followed by gradient descent.
        
        Args:
            original_path: List of (x, y) coordinates representing the original path
            
        Returns:
            Smoothed path as a list of (x, y) coordinates
        """
        # Convert input path to numpy array for easier computation
        original_path_array = np.array(original_path)
        
        # Step 1: Interpolate the path to increase density
        interpolated_path = self._interpolate_path(original_path_array)
        path = interpolated_path.copy()
        
        # Store original start and end points
        start_point = path[0].copy()
        end_point = path[-1].copy()
        
        prev_cost = float('inf')
        
        # Step 2: Apply gradient descent optimization
        for iteration in range(self.max_iterations):
            # Compute gradients
            smoothness_gradient = self._compute_smoothness_gradient(path)
            similarity_gradient = self._compute_similarity_gradient(path, interpolated_path)
            
            # Combine gradients with weights
            total_gradient = (self.alpha * smoothness_gradient + 
                            self.beta * similarity_gradient)
            
            # Zero out gradients for endpoints if they should be fixed
            if self.fix_endpoints:
                total_gradient[0] = 0.0
                total_gradient[-1] = 0.0
            
            # Update path
            path = path - self.learning_rate * total_gradient
            
            # Explicitly fix start and end points to ensure they don't drift due to numerical errors
            if self.fix_endpoints:
                path[0] = start_point
                path[-1] = end_point
            
            # Compute current cost
            smoothness_cost = np.sum(np.square(path[2:] - 2 * path[1:-1] + path[:-2]))
            similarity_cost = np.sum(np.square(path - interpolated_path))
            current_cost = self.alpha * smoothness_cost + self.beta * similarity_cost
            
            # Check convergence
            if abs(prev_cost - current_cost) < self.convergence_threshold:
                print(f"Convergence reached at iteration {iteration}")
                break
                
            prev_cost = current_cost
            
        return [(float(point[0]), float(point[1])) for point in path]

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