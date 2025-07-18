"""
Kalman Filter implementation for linear state estimation.

This module provides a standard Kalman Filter for estimating vehicle states
such as position, velocity, and acceleration from noisy sensor measurements.
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, Optional


class KalmanFilter:
    """
    Standard Kalman Filter for linear state estimation.
    
    The filter estimates the state vector x using a linear system model:
    x(k+1) = F * x(k) + B * u(k) + w(k)  # State prediction
    z(k) = H * x(k) + v(k)               # Measurement model
    
    where:
    - x: state vector
    - F: state transition matrix
    - B: control input matrix  
    - H: measurement matrix
    - w: process noise (covariance Q)
    - v: measurement noise (covariance R)
    """
    
    def __init__(self, F: np.ndarray, H: np.ndarray, Q: np.ndarray, R: np.ndarray, 
                 B: Optional[np.ndarray] = None):
        """
        Initialize Kalman Filter.
        
        Args:
            F: State transition matrix (n x n)
            H: Measurement matrix (m x n)
            Q: Process noise covariance matrix (n x n)
            R: Measurement noise covariance matrix (m x m)
            B: Control input matrix (n x p), optional
        """
        self.F = F  # State transition matrix
        self.H = H  # Measurement matrix
        self.Q = Q  # Process noise covariance
        self.R = R  # Measurement noise covariance
        self.B = B  # Control input matrix
        
        # State dimensions
        self.n = F.shape[0]  # Number of states
        self.m = H.shape[0]  # Number of measurements
        
        # Initialize state and covariance
        self.x = np.zeros(self.n)  # State estimate
        self.P = np.eye(self.n)    # Error covariance matrix
        
        # For storing history
        self.history = {
            'x': [],
            'P_diag': [],
            'innovation': [],
            'innovation_cov': []
        }
    
    def predict(self, u: Optional[np.ndarray] = None) -> None:
        """
        Prediction step of Kalman filter.
        
        Args:
            u: Control input vector (optional)
        """
        # State prediction
        self.x = self.F @ self.x
        if u is not None and self.B is not None:
            self.x += self.B @ u
            
        # Covariance prediction
        self.P = self.F @ self.P @ self.F.T + self.Q
    
    def update(self, z: np.ndarray) -> None:
        """
        Update step of Kalman filter.
        
        Args:
            z: Measurement vector
        """
        # Innovation (measurement residual)
        y = z - self.H @ self.x
        
        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R
        
        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # State update
        self.x = self.x + K @ y
        
        # Covariance update (Joseph form for numerical stability)
        I_KH = np.eye(self.n) - K @ self.H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T
        
        # Store history
        self.history['x'].append(self.x.copy())
        self.history['P_diag'].append(np.diag(self.P).copy())
        self.history['innovation'].append(y.copy())
        self.history['innovation_cov'].append(S.copy())
    
    def filter_step(self, z: np.ndarray, u: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Complete filter step: predict + update.
        
        Args:
            z: Measurement vector
            u: Control input vector (optional)
            
        Returns:
            Updated state estimate
        """
        self.predict(u)
        self.update(z)
        return self.x.copy()
    
    def set_initial_state(self, x0: np.ndarray, P0: np.ndarray) -> None:
        """
        Set initial state and covariance.
        
        Args:
            x0: Initial state vector
            P0: Initial covariance matrix
        """
        self.x = x0.copy()
        self.P = P0.copy()
    
    def get_state(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get current state estimate and covariance.
        
        Returns:
            Tuple of (state_estimate, covariance_matrix)
        """
        return self.x.copy(), self.P.copy()
    
    def reset_history(self) -> None:
        """Reset the filter history."""
        self.history = {
            'x': [],
            'P_diag': [],
            'innovation': [],
            'innovation_cov': []
        }


def create_vehicle_kalman_filter(dt: float) -> KalmanFilter:
    """
    Create a Kalman filter for 2D vehicle tracking.
    
    State vector: [x, y, vx, vy] (position and velocity in 2D)
    Measurement: [x, y] (position measurements from GPS/vision)
    
    Args:
        dt: Time step in seconds
        
    Returns:
        Configured Kalman filter
    """
    # State transition matrix (constant velocity model)
    F = np.array([
        [1, 0, dt, 0],
        [0, 1, 0, dt],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    
    # Measurement matrix (observe position only)
    H = np.array([
        [1, 0, 0, 0],
        [0, 1, 0, 0]
    ])
    
    # Process noise covariance (uncertainty in acceleration)
    q = 0.1  # Process noise standard deviation
    Q = q**2 * np.array([
        [dt**4/4, 0, dt**3/2, 0],
        [0, dt**4/4, 0, dt**3/2],
        [dt**3/2, 0, dt**2, 0],
        [0, dt**3/2, 0, dt**2]
    ])
    
    # Measurement noise covariance
    r = 0.5  # Measurement noise standard deviation (meters)
    R = r**2 * np.eye(2)
    
    return KalmanFilter(F, H, Q, R)


if __name__ == "__main__":
    # This will be moved to a separate test file
    print("Kalman Filter implementation ready for testing") 