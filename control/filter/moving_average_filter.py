"""
Moving Average Filter implementation for signal smoothing.

This module provides different types of moving average filters for smoothing
signals and reducing noise in real-time applications.
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Deque, Union, List
from collections import deque


class MovingAverageFilter:
    """
    Simple Moving Average (SMA) filter for signal smoothing.
    
    Computes the average of the last N samples to smooth the input signal.
    Formula: y[n] = (1/N) * Σ(x[n-k]) for k = 0 to N-1
    """
    
    def __init__(self, window_size: int):
        """
        Initialize moving average filter.
        
        Args:
            window_size: Number of samples to average (window size)
        """
        self.window_size = window_size
        self.buffer: Deque[float] = deque(maxlen=window_size)
        self.sum = 0.0
        
        # For storing history
        self.history = {
            'input': [],
            'output': [],
            'time': []
        }
        self.time_step = 0
    
    def filter(self, input_value: float) -> float:
        """
        Apply moving average filter to input value.
        
        Args:
            input_value: Input signal value
            
        Returns:
            Filtered output value (moving average)
        """
        # If buffer is full, subtract the oldest value from sum
        if len(self.buffer) == self.window_size:
            self.sum -= self.buffer[0]
        
        # Add new value to buffer and sum
        self.buffer.append(input_value)
        self.sum += input_value
        
        # Calculate moving average
        output = self.sum / len(self.buffer)
        
        # Store history
        self.history['input'].append(input_value)
        self.history['output'].append(output)
        self.history['time'].append(self.time_step)
        self.time_step += 1
        
        return output
    
    def filter_batch(self, input_signal: np.ndarray) -> np.ndarray:
        """
        Apply filter to a batch of input values.
        
        Args:
            input_signal: Array of input values
            
        Returns:
            Array of filtered output values
        """
        output_signal = np.zeros_like(input_signal)
        
        for i, value in enumerate(input_signal):
            output_signal[i] = self.filter(value)
            
        return output_signal
    
    def reset(self) -> None:
        """Reset filter state."""
        self.buffer.clear()
        self.sum = 0.0
        self.time_step = 0
        self.history = {
            'input': [],
            'output': [],
            'time': []
        }
    
    def get_current_average(self) -> float:
        """Get current moving average without adding new data."""
        if len(self.buffer) == 0:
            return 0.0
        return self.sum / len(self.buffer)


class ExponentialMovingAverageFilter:
    """
    Exponential Moving Average (EMA) filter.
    
    Gives more weight to recent samples using an exponential decay.
    Formula: y[n] = α * x[n] + (1-α) * y[n-1]
    where α is the smoothing factor (0 < α < 1).
    """
    
    def __init__(self, alpha: float):
        """
        Initialize exponential moving average filter.
        
        Args:
            alpha: Smoothing factor (0 < α < 1)
                  Higher α = more responsive to recent changes
                  Lower α = more smoothing
        """
        if not 0 < alpha < 1:
            raise ValueError("Alpha must be between 0 and 1")
            
        self.alpha = alpha
        self.ema = 0.0
        self.is_initialized = False
        
        # For storing history
        self.history = {
            'input': [],
            'output': [],
            'time': []
        }
        self.time_step = 0
    
    def filter(self, input_value: float) -> float:
        """
        Apply exponential moving average filter to input value.
        
        Args:
            input_value: Input signal value
            
        Returns:
            Filtered output value (EMA)
        """
        if not self.is_initialized:
            # Initialize with first input value
            self.ema = input_value
            self.is_initialized = True
        else:
            # Apply EMA formula
            self.ema = self.alpha * input_value + (1 - self.alpha) * self.ema
        
        # Store history
        self.history['input'].append(input_value)
        self.history['output'].append(self.ema)
        self.history['time'].append(self.time_step)
        self.time_step += 1
        
        return self.ema
    
    def filter_batch(self, input_signal: np.ndarray) -> np.ndarray:
        """
        Apply filter to a batch of input values.
        
        Args:
            input_signal: Array of input values
            
        Returns:
            Array of filtered output values
        """
        output_signal = np.zeros_like(input_signal)
        
        for i, value in enumerate(input_signal):
            output_signal[i] = self.filter(value)
            
        return output_signal
    
    def reset(self) -> None:
        """Reset filter state."""
        self.ema = 0.0
        self.is_initialized = False
        self.time_step = 0
        self.history = {
            'input': [],
            'output': [],
            'time': []
        }
    
    def set_alpha(self, new_alpha: float) -> None:
        """
        Update smoothing factor.
        
        Args:
            new_alpha: New smoothing factor (0 < α < 1)
        """
        if not 0 < new_alpha < 1:
            raise ValueError("Alpha must be between 0 and 1")
        self.alpha = new_alpha


class WeightedMovingAverageFilter:
    """
    Weighted Moving Average filter with custom weights.
    
    Allows custom weighting of samples in the moving window.
    More recent samples can be given higher weights.
    """
    
    def __init__(self, weights: List[float]):
        """
        Initialize weighted moving average filter.
        
        Args:
            weights: List of weights (most recent sample first)
                    Weights will be normalized to sum to 1
        """
        self.weights = np.array(weights)
        self.weights = self.weights / np.sum(self.weights)  # Normalize
        self.window_size = len(weights)
        self.buffer: Deque[float] = deque(maxlen=self.window_size)
        
        # For storing history
        self.history = {
            'input': [],
            'output': [],
            'time': []
        }
        self.time_step = 0
    
    def filter(self, input_value: float) -> float:
        """
        Apply weighted moving average filter to input value.
        
        Args:
            input_value: Input signal value
            
        Returns:
            Filtered output value (weighted average)
        """
        self.buffer.append(input_value)
        
        # Calculate weighted average
        if len(self.buffer) < self.window_size:
            # Use available samples with corresponding weights
            available_weights = self.weights[-len(self.buffer):]
            available_weights = available_weights / np.sum(available_weights)
            output = np.dot(list(self.buffer), available_weights)
        else:
            # Use all samples with full weights (most recent first)
            output = np.dot(list(reversed(self.buffer)), self.weights)
        
        # Store history
        self.history['input'].append(input_value)
        self.history['output'].append(output)
        self.history['time'].append(self.time_step)
        self.time_step += 1
        
        return output
    
    def filter_batch(self, input_signal: np.ndarray) -> np.ndarray:
        """
        Apply filter to a batch of input values.
        
        Args:
            input_signal: Array of input values
            
        Returns:
            Array of filtered output values
        """
        output_signal = np.zeros_like(input_signal)
        
        for i, value in enumerate(input_signal):
            output_signal[i] = self.filter(value)
            
        return output_signal
    
    def reset(self) -> None:
        """Reset filter state."""
        self.buffer.clear()
        self.time_step = 0
        self.history = {
            'input': [],
            'output': [],
            'time': []
        }


def create_linear_weights(window_size: int) -> List[float]:
    """
    Create linearly decreasing weights (most recent sample has highest weight).
    
    Args:
        window_size: Size of the moving window
        
    Returns:
        List of weights with linear decay
    """
    return list(range(window_size, 0, -1))


def create_exponential_weights(window_size: int, decay_factor: float = 0.8) -> List[float]:
    """
    Create exponentially decreasing weights.
    
    Args:
        window_size: Size of the moving window
        decay_factor: Exponential decay factor (0 < decay < 1)
        
    Returns:
        List of weights with exponential decay
    """
    weights = [decay_factor ** i for i in range(window_size)]
    return weights


if __name__ == "__main__":
    print("Moving Average Filter implementation ready for testing") 