"""
Particle Filter implementation for non-linear state estimation.

This module provides a particle filter for handling non-linear system dynamics
and non-Gaussian noise, commonly used in robotics and tracking applications.
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Callable, Tuple, Optional, List
from scipy.stats import multivariate_normal


class ParticleFilter:
    """
    Generic Particle Filter for non-linear state estimation.
    
    The particle filter represents the probability distribution of the state
    using a set of weighted particles (samples). It's particularly useful for
    non-linear systems and non-Gaussian noise distributions.
    """
    
    def __init__(self, num_particles: int, state_dim: int, 
                 process_noise_cov: np.ndarray, measurement_noise_cov: np.ndarray):
        """
        Initialize particle filter.
        
        Args:
            num_particles: Number of particles to use
            state_dim: Dimension of the state vector
            process_noise_cov: Process noise covariance matrix
            measurement_noise_cov: Measurement noise covariance matrix
        """
        self.num_particles = num_particles
        self.state_dim = state_dim
        self.process_noise_cov = process_noise_cov
        self.measurement_noise_cov = measurement_noise_cov
        
        # Initialize particles and weights
        self.particles = np.zeros((num_particles, state_dim))
        self.weights = np.ones(num_particles) / num_particles
        
        # For resampling
        self.effective_sample_threshold = num_particles / 2
        
        # For storing history
        self.history = {
            'particles': [],
            'weights': [],
            'estimated_state': [],
            'time': []
        }
        self.time_step = 0
    
    def initialize_particles(self, initial_state: np.ndarray, initial_cov: np.ndarray) -> None:
        """
        Initialize particles around initial state estimate.
        
        Args:
            initial_state: Initial state estimate
            initial_cov: Initial covariance matrix
        """
        self.particles = np.random.multivariate_normal(
            initial_state, initial_cov, self.num_particles
        )
        self.weights = np.ones(self.num_particles) / self.num_particles
    
    def predict(self, motion_model: Callable, control_input: Optional[np.ndarray] = None) -> None:
        """
        Prediction step: propagate particles through motion model.
        
        Args:
            motion_model: Function that takes (particle, control_input, dt) and returns next state
            control_input: Control input vector (optional)
        """
        for i in range(self.num_particles):
            # Apply motion model to each particle
            self.particles[i] = motion_model(self.particles[i], control_input)
            
            # Add process noise
            noise = np.random.multivariate_normal(
                np.zeros(self.state_dim), self.process_noise_cov
            )
            self.particles[i] += noise
    
    def update(self, measurement: np.ndarray, measurement_model: Callable) -> None:
        """
        Update step: update particle weights based on measurement likelihood.
        
        Args:
            measurement: Observed measurement vector
            measurement_model: Function that takes particle and returns expected measurement
        """
        for i in range(self.num_particles):
            # Predict measurement from particle
            predicted_measurement = measurement_model(self.particles[i])
            
            # Calculate likelihood of actual measurement
            innovation = measurement - predicted_measurement
            try:
                likelihood = multivariate_normal.pdf(
                    innovation, mean=np.zeros(len(innovation)), cov=self.measurement_noise_cov
                )
            except:
                # Fallback for numerical stability
                likelihood = np.exp(-0.5 * innovation.T @ np.linalg.inv(self.measurement_noise_cov) @ innovation)
            
            # Update weight
            self.weights[i] *= likelihood
        
        # Normalize weights
        self.weights += 1e-300  # Avoid division by zero
        self.weights /= np.sum(self.weights)
    
    def resample(self) -> None:
        """
        Resample particles based on weights (systematic resampling).
        """
        # Calculate effective sample size
        eff_sample_size = 1.0 / np.sum(self.weights ** 2)
        
        if eff_sample_size < self.effective_sample_threshold:
            # Systematic resampling
            cumulative_sum = np.cumsum(self.weights)
            cumulative_sum[-1] = 1.0  # Avoid numerical errors
            
            # Generate uniform random numbers
            u = (np.arange(self.num_particles) + np.random.random()) / self.num_particles
            
            # Resample particles
            new_particles = np.zeros_like(self.particles)
            j = 0
            for i in range(self.num_particles):
                while u[i] > cumulative_sum[j]:
                    j += 1
                new_particles[i] = self.particles[j]
            
            self.particles = new_particles
            self.weights = np.ones(self.num_particles) / self.num_particles
    
    def estimate_state(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Estimate state as weighted mean and covariance of particles.
        
        Returns:
            Tuple of (state_estimate, covariance_estimate)
        """
        # Weighted mean
        state_estimate = np.average(self.particles, weights=self.weights, axis=0)
        
        # Weighted covariance
        diff = self.particles - state_estimate
        covariance_estimate = np.average(
            np.array([np.outer(d, d) for d in diff]), weights=self.weights, axis=0
        )
        
        return state_estimate, covariance_estimate
    
    def filter_step(self, measurement: np.ndarray, motion_model: Callable, 
                   measurement_model: Callable, control_input: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Complete filter step: predict + update + resample.
        
        Args:
            measurement: Observed measurement vector
            motion_model: Motion model function
            measurement_model: Measurement model function
            control_input: Control input vector (optional)
            
        Returns:
            State estimate
        """
        self.predict(motion_model, control_input)
        self.update(measurement, measurement_model)
        self.resample()
        
        state_estimate, _ = self.estimate_state()
        
        # Store history
        self.history['particles'].append(self.particles.copy())
        self.history['weights'].append(self.weights.copy())
        self.history['estimated_state'].append(state_estimate.copy())
        self.history['time'].append(self.time_step)
        self.time_step += 1
        
        return state_estimate
    
    def get_particles(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get current particles and weights.
        
        Returns:
            Tuple of (particles, weights)
        """
        return self.particles.copy(), self.weights.copy()
    
    def reset(self) -> None:
        """Reset filter history."""
        self.time_step = 0
        self.history = {
            'particles': [],
            'weights': [],
            'estimated_state': [],
            'time': []
        }


class VehicleParticleFilter(ParticleFilter):
    """
    Specialized Particle Filter for 2D vehicle tracking.
    
    State vector: [x, y, vx, vy] (position and velocity)
    Measurement: [x, y] (position measurements from GPS/vision)
    """
    
    def __init__(self, num_particles: int = 1000, process_noise_std: float = 0.1, 
                 measurement_noise_std: float = 0.5):
        """
        Initialize vehicle particle filter.
        
        Args:
            num_particles: Number of particles
            process_noise_std: Standard deviation of process noise
            measurement_noise_std: Standard deviation of measurement noise
        """
        state_dim = 4  # [x, y, vx, vy]
        
        # Process noise covariance (acceleration uncertainty)
        process_noise_cov = process_noise_std**2 * np.eye(state_dim)
        
        # Measurement noise covariance (position measurement uncertainty)
        measurement_noise_cov = measurement_noise_std**2 * np.eye(2)
        
        super().__init__(num_particles, state_dim, process_noise_cov, measurement_noise_cov)
        
        self.dt = 0.1  # Default time step
    
    def set_time_step(self, dt: float) -> None:
        """Set time step for motion model."""
        self.dt = dt
    
    def motion_model(self, state: np.ndarray, control_input: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Constant velocity motion model.
        
        Args:
            state: Current state [x, y, vx, vy]
            control_input: Not used in this model
            
        Returns:
            Next state prediction
        """
        x, y, vx, vy = state
        
        # Constant velocity model
        next_state = np.array([
            x + vx * self.dt,
            y + vy * self.dt,
            vx,
            vy
        ])
        
        return next_state
    
    def measurement_model(self, state: np.ndarray) -> np.ndarray:
        """
        Position measurement model.
        
        Args:
            state: Current state [x, y, vx, vy]
            
        Returns:
            Expected measurement [x, y]
        """
        return state[:2]  # Observe position only


class NonLinearParticleFilter(ParticleFilter):
    """
    Example of particle filter for a non-linear system.
    
    Demonstrates tracking an object with non-linear motion (e.g., circular motion).
    """
    
    def __init__(self, num_particles: int = 1000, angular_velocity: float = 0.1):
        """
        Initialize non-linear particle filter.
        
        Args:
            num_particles: Number of particles
            angular_velocity: Angular velocity for circular motion
        """
        state_dim = 4  # [x, y, vx, vy]
        process_noise_cov = 0.01 * np.eye(state_dim)
        measurement_noise_cov = 0.25 * np.eye(2)
        
        super().__init__(num_particles, state_dim, process_noise_cov, measurement_noise_cov)
        
        self.omega = angular_velocity  # Angular velocity
        self.dt = 0.1
    
    def set_angular_velocity(self, omega: float) -> None:
        """Set angular velocity for circular motion."""
        self.omega = omega
    
    def motion_model(self, state: np.ndarray, control_input: Optional[np.ndarray] = None) -> np.ndarray:
        """
        Non-linear motion model (circular motion).
        
        Args:
            state: Current state [x, y, vx, vy]
            control_input: Not used
            
        Returns:
            Next state prediction
        """
        x, y, vx, vy = state
        
        # Circular motion model
        # Rotate velocity vector by angular velocity
        cos_omega_dt = np.cos(self.omega * self.dt)
        sin_omega_dt = np.sin(self.omega * self.dt)
        
        new_vx = vx * cos_omega_dt - vy * sin_omega_dt
        new_vy = vx * sin_omega_dt + vy * cos_omega_dt
        
        # Update position
        new_x = x + new_vx * self.dt
        new_y = y + new_vy * self.dt
        
        return np.array([new_x, new_y, new_vx, new_vy])
    
    def measurement_model(self, state: np.ndarray) -> np.ndarray:
        """
        Position measurement model.
        
        Args:
            state: Current state [x, y, vx, vy]
            
        Returns:
            Expected measurement [x, y]
        """
        return state[:2]


def create_tracking_scenario(duration: float = 10.0, dt: float = 0.1) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Create a test scenario for particle filter tracking.
    
    Args:
        duration: Simulation duration in seconds
        dt: Time step in seconds
        
    Returns:
        Tuple of (time_array, true_trajectory, noisy_measurements)
    """
    t = np.arange(0, duration, dt)
    
    # Generate true circular trajectory
    radius = 5.0
    omega = 0.2  # Angular velocity
    
    true_x = radius * np.cos(omega * t)
    true_y = radius * np.sin(omega * t)
    true_trajectory = np.column_stack([true_x, true_y])
    
    # Add measurement noise
    measurement_noise = 0.5 * np.random.randn(len(t), 2)
    noisy_measurements = true_trajectory + measurement_noise
    
    return t, true_trajectory, noisy_measurements


if __name__ == "__main__":
    print("Particle Filter implementation ready for testing") 