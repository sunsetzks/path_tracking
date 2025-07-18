"""
Simple demonstration of all filter implementations.

This script provides easy-to-run examples for each filter type.
Run this script to see the filters in action!
"""

import numpy as np
import matplotlib.pyplot as plt

from kalman_filter import create_vehicle_kalman_filter
from lowpass_filter import LowPassFilter, create_noise_signal
from moving_average_filter import MovingAverageFilter, ExponentialMovingAverageFilter
from complementary_filter import ComplementaryFilter, simulate_imu_data
from particle_filter import VehicleParticleFilter, create_tracking_scenario


def demo_kalman_filter():
    """Demonstrate Kalman Filter for vehicle tracking."""
    print("=" * 50)
    print("KALMAN FILTER DEMO - Vehicle Position Tracking")
    print("=" * 50)
    
    # Parameters
    dt = 0.1
    duration = 5.0
    t = np.arange(0, duration, dt)
    
    # Create Kalman filter
    kf = create_vehicle_kalman_filter(dt)
    
    # Generate true circular trajectory
    radius = 3.0
    omega = 0.5
    true_x = radius * np.cos(omega * t)
    true_y = radius * np.sin(omega * t)
    
    # Add measurement noise
    noise_std = 0.3
    meas_x = true_x + noise_std * np.random.randn(len(t))
    meas_y = true_y + noise_std * np.random.randn(len(t))
    
    # Initialize filter
    kf.set_initial_state(np.array([true_x[0], true_y[0], 0, 0]), np.eye(4))
    
    # Run filter
    estimates = []
    for i in range(len(t)):
        measurement = np.array([meas_x[i], meas_y[i]])
        estimate = kf.filter_step(measurement)
        estimates.append(estimate)
    
    estimates = np.array(estimates)
    
    # Plot results
    plt.figure(figsize=(10, 6))
    
    plt.subplot(1, 2, 1)
    plt.plot(true_x, true_y, 'g-', linewidth=3, label='True Path')
    plt.scatter(meas_x, meas_y, c='r', s=20, alpha=0.6, label='Noisy Measurements')
    plt.plot(estimates[:, 0], estimates[:, 1], 'b--', linewidth=2, label='Kalman Estimate')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Vehicle Position Tracking')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    
    plt.subplot(1, 2, 2)
    position_error = np.sqrt((estimates[:, 0] - true_x)**2 + (estimates[:, 1] - true_y)**2)
    plt.plot(t, position_error, 'r-', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Position Error (m)')
    plt.title('Tracking Error')
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()
    
    print(f"Mean position error: {np.mean(position_error):.3f} m")
    print(f"Max position error: {np.max(position_error):.3f} m")


def demo_lowpass_filter():
    """Demonstrate Low-pass Filter for noise reduction."""
    print("\n" + "=" * 50)
    print("LOW-PASS FILTER DEMO - Noise Reduction")
    print("=" * 50)
    
    # Generate noisy signal
    duration = 3.0
    sample_rate = 100.0
    signal_freq = 2.0
    noise_amplitude = 0.4
    
    t, clean_signal, noisy_signal = create_noise_signal(
        duration, sample_rate, signal_freq, noise_amplitude
    )
    
    # Create low-pass filter
    cutoff_freq = 5.0  # Hz
    lpf = LowPassFilter(cutoff_freq, sample_rate)
    
    # Filter the signal
    filtered_signal = lpf.filter_batch(noisy_signal)
    
    # Plot results
    plt.figure(figsize=(12, 4))
    
    plt.plot(t, clean_signal, 'g-', linewidth=3, label='Clean Signal')
    plt.plot(t, noisy_signal, 'r-', alpha=0.7, label='Noisy Signal')
    plt.plot(t, filtered_signal, 'b--', linewidth=2, label='Filtered Signal')
    
    plt.xlabel('Time (s)')
    plt.ylabel('Amplitude')
    plt.title(f'Low-pass Filter (Cutoff: {cutoff_freq} Hz)')
    plt.legend()
    plt.grid(True)
    plt.show()
    
    # Calculate improvement
    noise_power_before = np.var(noisy_signal - clean_signal)
    noise_power_after = np.var(filtered_signal - clean_signal)
    improvement_db = 10 * np.log10(noise_power_before / noise_power_after)
    
    print(f"Noise reduction: {improvement_db:.1f} dB")


def demo_moving_average_filter():
    """Demonstrate Moving Average Filter for smoothing."""
    print("\n" + "=" * 50)
    print("MOVING AVERAGE FILTER DEMO - Signal Smoothing")
    print("=" * 50)
    
    # Generate step signal with noise
    samples = 300
    step_position = 100
    
    signal = np.zeros(samples)
    signal[step_position:] = 1.0
    noise = 0.3 * np.random.randn(samples)
    noisy_signal = signal + noise
    
    # Test different filters
    ma_simple = MovingAverageFilter(window_size=20)
    ma_exp_fast = ExponentialMovingAverageFilter(alpha=0.3)
    ma_exp_slow = ExponentialMovingAverageFilter(alpha=0.1)
    
    # Apply filters
    simple_result = []
    exp_fast_result = []
    exp_slow_result = []
    
    for value in noisy_signal:
        simple_result.append(ma_simple.filter(value))
        exp_fast_result.append(ma_exp_fast.filter(value))
        exp_slow_result.append(ma_exp_slow.filter(value))
    
    # Plot results
    plt.figure(figsize=(12, 6))
    
    plt.plot(signal, 'g-', linewidth=3, label='True Signal')
    plt.plot(noisy_signal, 'r-', alpha=0.5, label='Noisy Signal')
    plt.plot(simple_result, 'b--', linewidth=2, label='Simple MA (N=20)')
    plt.plot(exp_fast_result, 'm--', linewidth=2, label='Exponential MA (α=0.3)')
    plt.plot(exp_slow_result, 'c--', linewidth=2, label='Exponential MA (α=0.1)')
    
    plt.axvline(x=step_position, color='k', linestyle=':', alpha=0.7, label='Step Input')
    plt.xlabel('Sample')
    plt.ylabel('Amplitude')
    plt.title('Moving Average Filter Comparison')
    plt.legend()
    plt.grid(True)
    plt.show()
    
    print("Notice how different alpha values affect responsiveness vs smoothness")


def demo_complementary_filter():
    """Demonstrate Complementary Filter for sensor fusion."""
    print("\n" + "=" * 50)
    print("COMPLEMENTARY FILTER DEMO - IMU Sensor Fusion")
    print("=" * 50)
    
    # Generate IMU data
    duration = 8.0
    sample_rate = 50.0
    dt = 1.0 / sample_rate
    
    t, gyro_data, accel_angle_data = simulate_imu_data(duration, sample_rate)
    
    # Create complementary filter
    alpha = 0.98
    cf = ComplementaryFilter(alpha)
    
    # Apply filter
    filtered_angles = []
    for i in range(len(t)):
        angle = cf.update(gyro_data[i], accel_angle_data[i], dt)
        filtered_angles.append(angle)
    
    # Generate true angle for comparison
    true_angle = 30 * np.sin(2 * np.pi * 0.5 * t) * np.pi / 180
    
    # Plot results
    plt.figure(figsize=(12, 8))
    
    plt.subplot(2, 1, 1)
    plt.plot(t, np.degrees(true_angle), 'g-', linewidth=3, label='True Angle')
    plt.plot(t, np.degrees(accel_angle_data), 'r:', alpha=0.7, label='Accelerometer')
    plt.plot(t, np.degrees(filtered_angles), 'b--', linewidth=2, label=f'Complementary Filter (α={alpha})')
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (degrees)')
    plt.title('Attitude Estimation')
    plt.legend()
    plt.grid(True)
    
    plt.subplot(2, 1, 2)
    plt.plot(t, np.degrees(gyro_data), 'b-', alpha=0.7, label='Gyroscope Rate')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Rate (deg/s)')
    plt.title('Gyroscope Input')
    plt.legend()
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()
    
    # Calculate accuracy
    error = np.abs(np.array(filtered_angles) - true_angle)
    print(f"Mean angle error: {np.degrees(np.mean(error)):.2f} degrees")
    print(f"Max angle error: {np.degrees(np.max(error)):.2f} degrees")


def demo_particle_filter():
    """Demonstrate Particle Filter for non-linear tracking."""
    print("\n" + "=" * 50)
    print("PARTICLE FILTER DEMO - Non-linear Object Tracking")
    print("=" * 50)
    
    # Generate circular trajectory with noise
    duration = 8.0
    dt = 0.1
    t, true_trajectory, noisy_measurements = create_tracking_scenario(duration, dt)
    
    # Create particle filter
    num_particles = 500
    pf = VehicleParticleFilter(num_particles=num_particles)
    pf.set_time_step(dt)
    
    # Initialize particles around first measurement
    initial_state = np.array([true_trajectory[0, 0], true_trajectory[0, 1], 0, 0])
    initial_cov = np.diag([1.0, 1.0, 1.0, 1.0])
    pf.initialize_particles(initial_state, initial_cov)
    
    # Track the object
    estimated_positions = []
    for measurement in noisy_measurements:
        state_est = pf.filter_step(measurement, pf.motion_model, pf.measurement_model)
        estimated_positions.append(state_est[:2])
    
    estimated_positions = np.array(estimated_positions)
    
    # Plot results
    plt.figure(figsize=(10, 8))
    
    plt.plot(true_trajectory[:, 0], true_trajectory[:, 1], 'g-', linewidth=3, label='True Trajectory')
    plt.scatter(noisy_measurements[:, 0], noisy_measurements[:, 1], c='r', s=15, alpha=0.6, label='Noisy Measurements')
    plt.plot(estimated_positions[:, 0], estimated_positions[:, 1], 'b--', linewidth=2, label=f'Particle Filter ({num_particles} particles)')
    
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.title('Particle Filter Object Tracking')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.show()
    
    # Calculate tracking error
    position_errors = np.linalg.norm(estimated_positions - true_trajectory, axis=1)
    print(f"Mean tracking error: {np.mean(position_errors):.3f} m")
    print(f"Max tracking error: {np.max(position_errors):.3f} m")


def main():
    """Run all filter demonstrations."""
    print("FILTER DEMONSTRATION SUITE")
    print("This script demonstrates 5 different types of filters")
    print("Close each plot window to proceed to the next demo")
    
    # Set random seed for reproducible results
    np.random.seed(42)
    
    # Run demonstrations
    demo_kalman_filter()
    demo_lowpass_filter() 
    demo_moving_average_filter()
    demo_complementary_filter()
    demo_particle_filter()
    
    print("\n" + "=" * 60)
    print("ALL FILTER DEMONSTRATIONS COMPLETED!")
    print("=" * 60)
    print("\nSummary of demonstrated filters:")
    print("1. Kalman Filter - Optimal for linear systems with Gaussian noise")
    print("2. Low-pass Filter - Simple noise reduction for signals")
    print("3. Moving Average Filter - Signal smoothing and trend following")
    print("4. Complementary Filter - Sensor fusion for IMU applications")
    print("5. Particle Filter - Non-linear tracking with non-Gaussian noise")
    print("\nAll filters are ready for use in your applications!")


if __name__ == "__main__":
    main() 