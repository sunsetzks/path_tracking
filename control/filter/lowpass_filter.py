"""
Low-pass Filter implementation for noise reduction.

This module provides digital low-pass filters for smoothing noisy signals
commonly found in sensor data and control systems.
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Union, List


class LowPassFilter:
    """
    Digital low-pass filter for noise reduction.
    
    Implements a first-order low-pass filter with configurable cutoff frequency.
    The filter equation is: y[n] = α * x[n] + (1-α) * y[n-1]
    where α is the smoothing factor related to the cutoff frequency.
    """
    
    def __init__(self, cutoff_freq: float, sample_rate: float):
        """
        Initialize low-pass filter.
        
        Args:
            cutoff_freq: Cutoff frequency in Hz
            sample_rate: Sampling frequency in Hz
        """
        self.cutoff_freq = cutoff_freq
        self.sample_rate = sample_rate
        
        # Calculate smoothing factor (alpha)
        dt = 1.0 / sample_rate
        rc = 1.0 / (2 * np.pi * cutoff_freq)
        self.alpha = dt / (rc + dt)
        
        # Filter state
        self.output: Union[float, np.ndarray] = 0.0
        self.is_initialized = False
        
        # For storing history
        self.history = {
            'input': [],
            'output': [],
            'time': []
        }
        self.time_step = 0
    
    def filter(self, input_value: Union[float, np.ndarray]) -> Union[float, np.ndarray]:
        """
        Apply low-pass filter to input value.
        
        Args:
            input_value: Input signal value(s)
            
        Returns:
            Filtered output value(s)
        """
        if not self.is_initialized:
            # Initialize with first input value
            self.output = input_value
            self.is_initialized = True
        else:
            # Apply low-pass filter equation
            if isinstance(input_value, np.ndarray):
                self.output = self.alpha * input_value + (1 - self.alpha) * self.output
            else:
                self.output = self.alpha * input_value + (1 - self.alpha) * self.output
        
        # Store history
        self.history['input'].append(input_value)
        self.history['output'].append(self.output)
        self.history['time'].append(self.time_step / self.sample_rate)
        self.time_step += 1
        
        return self.output
    
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
        self.output = 0.0
        self.is_initialized = False
        self.time_step = 0
        self.history = {
            'input': [],
            'output': [],
            'time': []
        }
    
    def set_cutoff_frequency(self, new_cutoff_freq: float) -> None:
        """
        Update cutoff frequency and recalculate alpha.
        
        Args:
            new_cutoff_freq: New cutoff frequency in Hz
        """
        self.cutoff_freq = new_cutoff_freq
        dt = 1.0 / self.sample_rate
        rc = 1.0 / (2 * np.pi * new_cutoff_freq)
        self.alpha = dt / (rc + dt)


class ButterworthLowPassFilter:
    """
    Second-order Butterworth low-pass filter.
    
    Provides better roll-off characteristics than first-order filter.
    Uses direct form II implementation for numerical stability.
    """
    
    def __init__(self, cutoff_freq: float, sample_rate: float):
        """
        Initialize Butterworth low-pass filter.
        
        Args:
            cutoff_freq: Cutoff frequency in Hz
            sample_rate: Sampling frequency in Hz
        """
        self.cutoff_freq = cutoff_freq
        self.sample_rate = sample_rate
        
        # Calculate filter coefficients
        self._calculate_coefficients()
        
        # Filter memory (for direct form II)
        self.w1 = 0.0  # w[n-1]
        self.w2 = 0.0  # w[n-2]
        
        # For storing history
        self.history = {
            'input': [],
            'output': [],
            'time': []
        }
        self.time_step = 0
    
    def _calculate_coefficients(self) -> None:
        """Calculate second-order Butterworth filter coefficients."""
        # Normalized frequency
        wc = 2 * np.pi * self.cutoff_freq / self.sample_rate
        
        # Butterworth filter design
        k = np.tan(wc / 2)
        norm = 1 + np.sqrt(2) * k + k**2
        
        # Numerator coefficients (b)
        self.b0 = k**2 / norm
        self.b1 = 2 * self.b0
        self.b2 = self.b0
        
        # Denominator coefficients (a)
        self.a1 = (2 * (k**2 - 1)) / norm
        self.a2 = (1 - np.sqrt(2) * k + k**2) / norm
    
    def filter(self, input_value: float) -> float:
        """
        Apply Butterworth low-pass filter to input value.
        
        Args:
            input_value: Input signal value
            
        Returns:
            Filtered output value
        """
        # Direct form II implementation
        w0 = input_value - self.a1 * self.w1 - self.a2 * self.w2
        output = self.b0 * w0 + self.b1 * self.w1 + self.b2 * self.w2
        
        # Update memory
        self.w2 = self.w1
        self.w1 = w0
        
        # Store history
        self.history['input'].append(input_value)
        self.history['output'].append(output)
        self.history['time'].append(self.time_step / self.sample_rate)
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
        self.w1 = 0.0
        self.w2 = 0.0
        self.time_step = 0
        self.history = {
            'input': [],
            'output': [],
            'time': []
        }


def create_noise_signal(duration: float, sample_rate: float, 
                       signal_freq: float = 1.0, noise_amplitude: float = 0.5) -> tuple:
    """
    Create a test signal with noise for filter testing.
    
    Args:
        duration: Signal duration in seconds
        sample_rate: Sampling frequency in Hz
        signal_freq: Frequency of the clean signal in Hz
        noise_amplitude: Amplitude of added white noise
        
    Returns:
        Tuple of (time_array, clean_signal, noisy_signal)
    """
    t = np.linspace(0, duration, int(duration * sample_rate), endpoint=False)
    
    # Clean sinusoidal signal
    clean_signal = np.sin(2 * np.pi * signal_freq * t)
    
    # Add white noise
    noise = noise_amplitude * np.random.randn(len(t))
    noisy_signal = clean_signal + noise
    
    return t, clean_signal, noisy_signal


if __name__ == "__main__":
    print("Low-pass Filter implementation ready for testing") 