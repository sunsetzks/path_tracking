import numpy as np
from typing import List, Tuple

class GradientPathSmoother:
    """
    A path smoother using gradient descent to minimize a cost function that balances
    path smoothness and deviation from the original path.
    The start and end points remain fixed during optimization.
    """
    def __init__(self, 
                 alpha: float = 0.1,    # Weight for path smoothness
                 beta: float = 0.2,     # Weight for path similarity
                 learning_rate: float = 0.01,
                 max_iterations: int = 1000,
                 convergence_threshold: float = 1e-6,
                 fix_endpoints: bool = True):
        """
        Initialize the path smoother.
        
        Args:
            alpha: Weight for path smoothness term
            beta: Weight for path similarity term
            learning_rate: Learning rate for gradient descent
            max_iterations: Maximum number of iterations
            convergence_threshold: Threshold for convergence check
            fix_endpoints: Whether to keep start and end points fixed
        """
        self.alpha = alpha
        self.beta = beta
        self.learning_rate = learning_rate
        self.max_iterations = max_iterations
        self.convergence_threshold = convergence_threshold
        self.fix_endpoints = fix_endpoints

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
        Smooth the input path using gradient descent.
        
        Args:
            original_path: List of (x, y) coordinates representing the original path
            
        Returns:
            Smoothed path as a list of (x, y) coordinates
        """
        # Convert input path to numpy array for easier computation
        path = np.array(original_path)
        original_path_array = path.copy()
        
        # Store original start and end points
        start_point = path[0].copy()
        end_point = path[-1].copy()
        
        prev_cost = float('inf')
        
        for iteration in range(self.max_iterations):
            # Compute gradients
            smoothness_gradient = self._compute_smoothness_gradient(path)
            similarity_gradient = self._compute_similarity_gradient(path, original_path_array)
            
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
            similarity_cost = np.sum(np.square(path - original_path_array))
            current_cost = self.alpha * smoothness_cost + self.beta * similarity_cost
            
            # Check convergence
            if abs(prev_cost - current_cost) < self.convergence_threshold:
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