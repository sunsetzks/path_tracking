"""
Complementary Filter implementation for sensor fusion.

This module provides complementary filters for fusing data from multiple sensors,
commonly used for combining accelerometer and gyroscope data in IMU applications.
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, Optional


class ComplementaryFilter:
    """
    Basic Complementary Filter for sensor fusion.
    
    Combines high-frequency data from one sensor with low-frequency data from another.
    Commonly used for attitude estimation using gyroscope and accelerometer data.
    
    The filter equation is:
    output = α * (output + gyro_data * dt) + (1-α) * accel_data
    
    where α is the filter coefficient (0 < α < 1):
    - α close to 1: trust gyroscope more (good short-term accuracy)
    - α close to 0: trust accelerometer more (good long-term stability)
    """
    
    def __init__(self, alpha: float = 0.98):
        """
        Initialize complementary filter.
        
        Args:
            alpha: Filter coefficient (0 < α < 1)
                  Typical values: 0.95 - 0.99
        """
        if not 0 < alpha < 1:
            raise ValueError("Alpha must be between 0 and 1")
            
        self.alpha = alpha
        self.angle = 0.0  # Current estimated angle
        self.is_initialized = False
        
        # For storing history
        self.history = {
            'gyro_input': [],
            'accel_input': [],
            'filtered_output': [],
            'time': []
        }
        self.time_step = 0
    
    def update(self, gyro_rate: float, accel_angle: float, dt: float) -> float:
        """
        Update filter with new sensor data.
        
        Args:
            gyro_rate: Angular velocity from gyroscope (rad/s or deg/s)
            accel_angle: Angle calculated from accelerometer (rad or deg)
            dt: Time step since last update (seconds)
            
        Returns:
            Filtered angle estimate
        """
        if not self.is_initialized:
            # Initialize with accelerometer reading
            self.angle = accel_angle
            self.is_initialized = True
        else:
            # Apply complementary filter
            # High-pass: integrate gyroscope (short-term accuracy)
            gyro_angle = self.angle + gyro_rate * dt
            
            # Low-pass: use accelerometer (long-term stability)
            # Complementary filter equation
            self.angle = self.alpha * gyro_angle + (1 - self.alpha) * accel_angle
        
        # Store history
        self.history['gyro_input'].append(gyro_rate)
        self.history['accel_input'].append(accel_angle)
        self.history['filtered_output'].append(self.angle)
        self.history['time'].append(self.time_step * dt)
        self.time_step += 1
        
        return self.angle
    
    def get_angle(self) -> float:
        """Get current angle estimate."""
        return self.angle
    
    def reset(self) -> None:
        """Reset filter state."""
        self.angle = 0.0
        self.is_initialized = False
        self.time_step = 0
        self.history = {
            'gyro_input': [],
            'accel_input': [],
            'filtered_output': [],
            'time': []
        }
    
    def set_alpha(self, new_alpha: float) -> None:
        """
        Update filter coefficient.
        
        Args:
            new_alpha: New filter coefficient (0 < α < 1)
        """
        if not 0 < new_alpha < 1:
            raise ValueError("Alpha must be between 0 and 1")
        self.alpha = new_alpha


class ComplementaryFilter2D:
    """
    2D Complementary Filter for attitude estimation.
    
    Estimates roll and pitch angles using 3-axis accelerometer and gyroscope data.
    """
    
    def __init__(self, alpha: float = 0.98):
        """
        Initialize 2D complementary filter.
        
        Args:
            alpha: Filter coefficient (0 < α < 1)
        """
        if not 0 < alpha < 1:
            raise ValueError("Alpha must be between 0 and 1")
            
        self.alpha = alpha
        self.roll = 0.0   # Roll angle (rotation about x-axis)
        self.pitch = 0.0  # Pitch angle (rotation about y-axis)
        self.is_initialized = False
        
        # For storing history
        self.history = {
            'gyro_x': [], 'gyro_y': [], 'gyro_z': [],
            'accel_x': [], 'accel_y': [], 'accel_z': [],
            'roll': [], 'pitch': [],
            'time': []
        }
        self.time_step = 0
    
    def update(self, gyro_xyz: np.ndarray, accel_xyz: np.ndarray, dt: float) -> Tuple[float, float]:
        """
        Update filter with 3-axis sensor data.
        
        Args:
            gyro_xyz: Gyroscope data [x, y, z] in rad/s
            accel_xyz: Accelerometer data [x, y, z] in m/s²
            dt: Time step since last update (seconds)
            
        Returns:
            Tuple of (roll, pitch) angles in radians
        """
        gx, gy, gz = gyro_xyz
        ax, ay, az = accel_xyz
        
        if not self.is_initialized:
            # Initialize with accelerometer readings
            self.roll = np.arctan2(ay, az)
            self.pitch = np.arctan2(-ax, np.sqrt(ay*ay + az*az))
            self.is_initialized = True
        else:
            # Calculate angles from accelerometer
            accel_roll = np.arctan2(ay, az)
            accel_pitch = np.arctan2(-ax, np.sqrt(ay*ay + az*az))
            
            # Integrate gyroscope data
            gyro_roll = self.roll + gx * dt
            gyro_pitch = self.pitch + gy * dt
            
            # Apply complementary filter
            self.roll = self.alpha * gyro_roll + (1 - self.alpha) * accel_roll
            self.pitch = self.alpha * gyro_pitch + (1 - self.alpha) * accel_pitch
        
        # Store history
        self.history['gyro_x'].append(gx)
        self.history['gyro_y'].append(gy)
        self.history['gyro_z'].append(gz)
        self.history['accel_x'].append(ax)
        self.history['accel_y'].append(ay)
        self.history['accel_z'].append(az)
        self.history['roll'].append(self.roll)
        self.history['pitch'].append(self.pitch)
        self.history['time'].append(self.time_step * dt)
        self.time_step += 1
        
        return self.roll, self.pitch
    
    def get_angles(self) -> Tuple[float, float]:
        """Get current roll and pitch estimates."""
        return self.roll, self.pitch
    
    def reset(self) -> None:
        """Reset filter state."""
        self.roll = 0.0
        self.pitch = 0.0
        self.is_initialized = False
        self.time_step = 0
        self.history = {
            'gyro_x': [], 'gyro_y': [], 'gyro_z': [],
            'accel_x': [], 'accel_y': [], 'accel_z': [],
            'roll': [], 'pitch': [],
            'time': []
        }
    
    def set_alpha(self, new_alpha: float) -> None:
        """
        Update filter coefficient.
        
        Args:
            new_alpha: New filter coefficient (0 < α < 1)
        """
        if not 0 < new_alpha < 1:
            raise ValueError("Alpha must be between 0 and 1")
        self.alpha = new_alpha


class AdaptiveComplementaryFilter:
    """
    Adaptive Complementary Filter with dynamic alpha adjustment.
    
    Automatically adjusts the filter coefficient based on acceleration magnitude
    to improve performance during dynamic movements.
    """
    
    def __init__(self, alpha_min: float = 0.90, alpha_max: float = 0.99, 
                 accel_threshold: float = 1.2):
        """
        Initialize adaptive complementary filter.
        
        Args:
            alpha_min: Minimum filter coefficient (during high acceleration)
            alpha_max: Maximum filter coefficient (during low acceleration)
            accel_threshold: Acceleration threshold for adaptation (in g's)
        """
        self.alpha_min = alpha_min
        self.alpha_max = alpha_max
        self.accel_threshold = accel_threshold
        self.current_alpha = alpha_max
        
        self.angle = 0.0
        self.is_initialized = False
        
        # For storing history
        self.history = {
            'gyro_input': [],
            'accel_input': [],
            'accel_magnitude': [],
            'alpha_used': [],
            'filtered_output': [],
            'time': []
        }
        self.time_step = 0
    
    def _calculate_adaptive_alpha(self, accel_magnitude: float) -> float:
        """
        Calculate adaptive alpha based on acceleration magnitude.
        
        Args:
            accel_magnitude: Current acceleration magnitude (in g's)
            
        Returns:
            Adaptive alpha value
        """
        # When acceleration is close to 1g (static), use high alpha (trust gyro)
        # When acceleration deviates from 1g (dynamic), use low alpha (trust accel less)
        deviation = abs(accel_magnitude - 1.0)
        
        if deviation < 0.1:  # Near static condition
            return self.alpha_max
        elif deviation > self.accel_threshold:  # High dynamic condition
            return self.alpha_min
        else:  # Interpolate between min and max
            factor = deviation / self.accel_threshold
            return self.alpha_max - factor * (self.alpha_max - self.alpha_min)
    
    def update(self, gyro_rate: float, accel_angle: float, accel_magnitude: float, 
               dt: float) -> float:
        """
        Update filter with new sensor data and adaptive alpha.
        
        Args:
            gyro_rate: Angular velocity from gyroscope (rad/s)
            accel_angle: Angle calculated from accelerometer (rad)
            accel_magnitude: Magnitude of acceleration vector (in g's)
            dt: Time step since last update (seconds)
            
        Returns:
            Filtered angle estimate
        """
        # Calculate adaptive alpha
        self.current_alpha = self._calculate_adaptive_alpha(accel_magnitude)
        
        if not self.is_initialized:
            self.angle = accel_angle
            self.is_initialized = True
        else:
            # Apply adaptive complementary filter
            gyro_angle = self.angle + gyro_rate * dt
            self.angle = self.current_alpha * gyro_angle + (1 - self.current_alpha) * accel_angle
        
        # Store history
        self.history['gyro_input'].append(gyro_rate)
        self.history['accel_input'].append(accel_angle)
        self.history['accel_magnitude'].append(accel_magnitude)
        self.history['alpha_used'].append(self.current_alpha)
        self.history['filtered_output'].append(self.angle)
        self.history['time'].append(self.time_step * dt)
        self.time_step += 1
        
        return self.angle
    
    def get_angle(self) -> float:
        """Get current angle estimate."""
        return self.angle
    
    def get_current_alpha(self) -> float:
        """Get current adaptive alpha value."""
        return self.current_alpha
    
    def reset(self) -> None:
        """Reset filter state."""
        self.angle = 0.0
        self.current_alpha = self.alpha_max
        self.is_initialized = False
        self.time_step = 0
        self.history = {
            'gyro_input': [],
            'accel_input': [],
            'accel_magnitude': [],
            'alpha_used': [],
            'filtered_output': [],
            'time': []
        }


def simulate_imu_data(duration: float, sample_rate: float, 
                     true_angle_func=None) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Generate simulated IMU data for testing complementary filters.
    
    Args:
        duration: Simulation duration in seconds
        sample_rate: Sampling frequency in Hz
        true_angle_func: Function to generate true angle vs time
        
    Returns:
        Tuple of (time_array, gyro_data, accel_angle_data)
    """
    dt = 1.0 / sample_rate
    t = np.arange(0, duration, dt)
    
    if true_angle_func is None:
        # Default: sinusoidal motion
        true_angle = 30 * np.sin(2 * np.pi * 0.5 * t) * np.pi / 180  # 30 deg amplitude, 0.5 Hz
    else:
        true_angle = true_angle_func(t)
    
    # Simulate gyroscope (derivative of angle + noise + bias)
    gyro_rate = np.gradient(true_angle, dt)
    gyro_bias = 0.02  # Constant bias (rad/s)
    gyro_noise = 0.01 * np.random.randn(len(t))
    gyro_data = gyro_rate + gyro_bias + gyro_noise
    
    # Simulate accelerometer (true angle + noise)
    accel_noise = 0.05 * np.random.randn(len(t))
    accel_angle_data = true_angle + accel_noise
    
    return t, gyro_data, accel_angle_data


if __name__ == "__main__":
    print("Complementary Filter implementation ready for testing") 