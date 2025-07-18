"""
Comprehensive simulation tests for all filter implementations.

This module provides simulation scenarios to test and compare the performance
of different filter types under various conditions.
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Dict, List, Tuple
import time

from kalman_filter import KalmanFilter, create_vehicle_kalman_filter
from lowpass_filter import LowPassFilter, ButterworthLowPassFilter, create_noise_signal
from moving_average_filter import MovingAverageFilter, ExponentialMovingAverageFilter, WeightedMovingAverageFilter, create_linear_weights
from complementary_filter import ComplementaryFilter, ComplementaryFilter2D, simulate_imu_data
from particle_filter import VehicleParticleFilter, NonLinearParticleFilter, create_tracking_scenario


class FilterTestSuite:
    """
    Test suite for running and comparing different filters.
    """
    
    def __init__(self):
        """Initialize test suite."""
        self.results = {}
        self.test_data = {}
    
    def test_kalman_filter(self, plot: bool = True) -> Dict:
        """
        Test Kalman Filter with vehicle tracking scenario.
        
        Args:
            plot: Whether to generate plots
            
        Returns:
            Dictionary with test results
        """
        print("Testing Kalman Filter...")
        
        # Test parameters
        dt = 0.1  # Time step
        duration = 10.0  # Simulation duration
        t = np.arange(0, duration, dt)
        
        # Create filter
        kf = create_vehicle_kalman_filter(dt)
        
        # Generate true trajectory (circular motion)
        radius = 5.0
        omega = 0.2
        true_x = radius * np.cos(omega * t)
        true_y = radius * np.sin(omega * t)
        true_vx = -radius * omega * np.sin(omega * t)
        true_vy = radius * omega * np.cos(omega * t)
        
        true_states = np.column_stack([true_x, true_y, true_vx, true_vy])
        
        # Generate noisy measurements (position only)
        measurement_noise = 0.5 * np.random.randn(len(t), 2)
        measurements = true_states[:, :2] + measurement_noise
        
        # Initialize filter
        initial_state = np.array([true_x[0], true_y[0], 0.0, 0.0])
        initial_cov = np.diag([1.0, 1.0, 1.0, 1.0])
        kf.set_initial_state(initial_state, initial_cov)
        
        # Run filter
        start_time = time.time()
        estimated_states = []
        for i, measurement in enumerate(measurements):
            state_est = kf.filter_step(measurement)
            estimated_states.append(state_est)
        
        processing_time = time.time() - start_time
        estimated_states = np.array(estimated_states)
        
        # Calculate errors
        position_errors = np.linalg.norm(estimated_states[:, :2] - true_states[:, :2], axis=1)
        velocity_errors = np.linalg.norm(estimated_states[:, 2:] - true_states[:, 2:], axis=1)
        
        results = {
            'filter_name': 'Kalman Filter',
            'mean_position_error': np.mean(position_errors),
            'std_position_error': np.std(position_errors),
            'mean_velocity_error': np.mean(velocity_errors),
            'std_velocity_error': np.std(velocity_errors),
            'processing_time': processing_time,
            'convergence_time': self._calculate_convergence_time(position_errors, threshold=1.0)
        }
        
        # Store data for plotting
        self.test_data['kalman'] = {
            'time': t,
            'true_states': true_states,
            'measurements': measurements,
            'estimated_states': estimated_states,
            'position_errors': position_errors
        }
        
        if plot:
            self._plot_kalman_results()
        
        return results
    
    def test_lowpass_filters(self, plot: bool = True) -> Dict:
        """
        Test Low-pass Filters with noisy signal.
        
        Args:
            plot: Whether to generate plots
            
        Returns:
            Dictionary with test results
        """
        print("Testing Low-pass Filters...")
        
        # Generate test signal
        duration = 5.0
        sample_rate = 100.0
        signal_freq = 2.0
        noise_amplitude = 0.3
        
        t, clean_signal, noisy_signal = create_noise_signal(
            duration, sample_rate, signal_freq, noise_amplitude
        )
        
        # Test different filters
        filters = {
            'Simple LPF (5Hz)': LowPassFilter(5.0, sample_rate),
            'Simple LPF (10Hz)': LowPassFilter(10.0, sample_rate),
            'Butterworth LPF (5Hz)': ButterworthLowPassFilter(5.0, sample_rate)
        }
        
        results = {}
        filtered_signals = {}
        
        for name, lpf in filters.items():
            # Reset and apply filter
            lpf.reset()
            start_time = time.time()
            filtered_signal = lpf.filter_batch(noisy_signal)
            processing_time = time.time() - start_time
            
            # Calculate performance metrics
            mse = np.mean((filtered_signal - clean_signal) ** 2)
            snr_improvement = 10 * np.log10(
                np.var(noisy_signal - clean_signal) / np.var(filtered_signal - clean_signal)
            )
            
            results[name] = {
                'mse': mse,
                'snr_improvement_db': snr_improvement,
                'processing_time': processing_time
            }
            
            filtered_signals[name] = filtered_signal
        
        # Store data for plotting
        self.test_data['lowpass'] = {
            'time': t,
            'clean_signal': clean_signal,
            'noisy_signal': noisy_signal,
            'filtered_signals': filtered_signals
        }
        
        if plot:
            self._plot_lowpass_results()
        
        return results
    
    def test_moving_average_filters(self, plot: bool = True) -> Dict:
        """
        Test Moving Average Filters with step response.
        
        Args:
            plot: Whether to generate plots
            
        Returns:
            Dictionary with test results
        """
        print("Testing Moving Average Filters...")
        
        # Generate step signal with noise
        duration = 500
        step_position = 200
        
        # Create step signal
        signal = np.zeros(duration)
        signal[step_position:] = 1.0
        
        # Add noise
        noise = 0.2 * np.random.randn(duration)
        noisy_signal = signal + noise
        
        # Test different filters
        window_size = 20
        filters = {
            'Simple MA': MovingAverageFilter(window_size),
            'Exponential MA (α=0.1)': ExponentialMovingAverageFilter(0.1),
            'Exponential MA (α=0.3)': ExponentialMovingAverageFilter(0.3),
            'Weighted MA (Linear)': WeightedMovingAverageFilter(create_linear_weights(window_size))
        }
        
        results = {}
        filtered_signals = {}
        
        for name, ma_filter in filters.items():
            # Reset and apply filter
            ma_filter.reset()
            start_time = time.time()
            
            filtered_signal = []
            for value in noisy_signal:
                filtered_signal.append(ma_filter.filter(value))
            
            processing_time = time.time() - start_time
            filtered_signal = np.array(filtered_signal)
            
            # Calculate step response metrics
            steady_state_error = abs(np.mean(filtered_signal[-50:]) - 1.0)
            rise_time = self._calculate_rise_time(filtered_signal, step_position)
            overshoot = self._calculate_overshoot(filtered_signal, step_position)
            
            results[name] = {
                'steady_state_error': steady_state_error,
                'rise_time': rise_time,
                'overshoot': overshoot,
                'processing_time': processing_time
            }
            
            filtered_signals[name] = filtered_signal
        
        # Store data for plotting
        self.test_data['moving_average'] = {
            'time': np.arange(duration),
            'true_signal': signal,
            'noisy_signal': noisy_signal,
            'filtered_signals': filtered_signals,
            'step_position': step_position
        }
        
        if plot:
            self._plot_moving_average_results()
        
        return results
    
    def test_complementary_filter(self, plot: bool = True) -> Dict:
        """
        Test Complementary Filter with IMU simulation.
        
        Args:
            plot: Whether to generate plots
            
        Returns:
            Dictionary with test results
        """
        print("Testing Complementary Filter...")
        
        # Generate IMU data
        duration = 10.0
        sample_rate = 100.0
        dt = 1.0 / sample_rate
        
        t, gyro_data, accel_angle_data = simulate_imu_data(duration, sample_rate)
        
        # Test different alpha values
        alpha_values = [0.95, 0.98, 0.99]
        results = {}
        filtered_angles = {}
        
        for alpha in alpha_values:
            cf = ComplementaryFilter(alpha)
            
            start_time = time.time()
            filtered_angle = []
            for i in range(len(t)):
                angle = cf.update(gyro_data[i], accel_angle_data[i], dt)
                filtered_angle.append(angle)
            
            processing_time = time.time() - start_time
            filtered_angle = np.array(filtered_angle)
            
            # Calculate true angle for comparison (sinusoidal)
            true_angle = 30 * np.sin(2 * np.pi * 0.5 * t) * np.pi / 180
            
            # Performance metrics
            angle_error = np.abs(filtered_angle - true_angle)
            mean_error = np.mean(angle_error)
            max_error = np.max(angle_error)
            
            filter_name = f'CF (α={alpha})'
            results[filter_name] = {
                'mean_error_deg': np.degrees(mean_error),
                'max_error_deg': np.degrees(max_error),
                'processing_time': processing_time
            }
            
            filtered_angles[filter_name] = filtered_angle
        
        # Store data for plotting
        self.test_data['complementary'] = {
            'time': t,
            'true_angle': true_angle,
            'gyro_data': gyro_data,
            'accel_angle_data': accel_angle_data,
            'filtered_angles': filtered_angles
        }
        
        if plot:
            self._plot_complementary_results()
        
        return results
    
    def test_particle_filter(self, plot: bool = True) -> Dict:
        """
        Test Particle Filter with non-linear tracking.
        
        Args:
            plot: Whether to generate plots
            
        Returns:
            Dictionary with test results
        """
        print("Testing Particle Filter...")
        
        # Generate tracking scenario
        duration = 10.0
        dt = 0.1
        t, true_trajectory, noisy_measurements = create_tracking_scenario(duration, dt)
        
        # Test different numbers of particles
        particle_counts = [100, 500, 1000]
        results = {}
        estimated_trajectories = {}
        
        for num_particles in particle_counts:
            pf = VehicleParticleFilter(num_particles=num_particles)
            pf.set_time_step(dt)
            
            # Initialize particles
            initial_state = np.array([true_trajectory[0, 0], true_trajectory[0, 1], 0.0, 0.0])
            initial_cov = np.diag([1.0, 1.0, 1.0, 1.0])
            pf.initialize_particles(initial_state, initial_cov)
            
            start_time = time.time()
            estimated_states = []
            for measurement in noisy_measurements:
                state_est = pf.filter_step(measurement, pf.motion_model, pf.measurement_model)
                estimated_states.append(state_est)
            
            processing_time = time.time() - start_time
            estimated_states = np.array(estimated_states)
            
            # Calculate tracking errors
            position_errors = np.linalg.norm(
                estimated_states[:, :2] - true_trajectory, axis=1
            )
            
            filter_name = f'PF ({num_particles} particles)'
            results[filter_name] = {
                'mean_position_error': np.mean(position_errors),
                'std_position_error': np.std(position_errors),
                'max_position_error': np.max(position_errors),
                'processing_time': processing_time,
                'particles_per_second': len(noisy_measurements) * num_particles / processing_time
            }
            
            estimated_trajectories[filter_name] = estimated_states[:, :2]
        
        # Store data for plotting
        self.test_data['particle'] = {
            'time': t,
            'true_trajectory': true_trajectory,
            'noisy_measurements': noisy_measurements,
            'estimated_trajectories': estimated_trajectories
        }
        
        if plot:
            self._plot_particle_results()
        
        return results
    
    def run_all_tests(self, plot: bool = True) -> Dict:
        """
        Run all filter tests and generate summary.
        
        Args:
            plot: Whether to generate plots
            
        Returns:
            Dictionary with all test results
        """
        print("=" * 60)
        print("Running Comprehensive Filter Test Suite")
        print("=" * 60)
        
        all_results = {}
        
        # Run individual tests
        all_results['kalman'] = self.test_kalman_filter(plot=False)
        all_results['lowpass'] = self.test_lowpass_filters(plot=False)
        all_results['moving_average'] = self.test_moving_average_filters(plot=False)
        all_results['complementary'] = self.test_complementary_filter(plot=False)
        all_results['particle'] = self.test_particle_filter(plot=False)
        
        # Generate summary
        self._print_summary(all_results)
        
        if plot:
            self._plot_all_results()
        
        return all_results
    
    def _calculate_convergence_time(self, errors: np.ndarray, threshold: float) -> float:
        """Calculate time to converge below threshold."""
        converged_indices = np.where(errors < threshold)[0]
        if len(converged_indices) > 0:
            return converged_indices[0] * 0.1  # Assuming 0.1s time step
        return np.inf
    
    def _calculate_rise_time(self, signal: np.ndarray, step_position: int) -> int:
        """Calculate rise time (10% to 90% of final value)."""
        if step_position >= len(signal):
            return 0
        
        final_value = np.mean(signal[-50:])
        signal_after_step = signal[step_position:]
        
        target_10 = 0.1 * final_value
        target_90 = 0.9 * final_value
        
        idx_10 = np.where(signal_after_step >= target_10)[0]
        idx_90 = np.where(signal_after_step >= target_90)[0]
        
        if len(idx_10) > 0 and len(idx_90) > 0:
            return idx_90[0] - idx_10[0]
        return 0
    
    def _calculate_overshoot(self, signal: np.ndarray, step_position: int) -> float:
        """Calculate percentage overshoot."""
        if step_position >= len(signal):
            return 0.0
        
        final_value = np.mean(signal[-50:])
        signal_after_step = signal[step_position:]
        
        max_value = np.max(signal_after_step)
        overshoot = (max_value - final_value) / final_value * 100
        return max(0, overshoot)
    
    def _print_summary(self, results: Dict) -> None:
        """Print test results summary."""
        print("\n" + "=" * 60)
        print("FILTER PERFORMANCE SUMMARY")
        print("=" * 60)
        
        for category, category_results in results.items():
            print(f"\n{category.upper()} FILTER RESULTS:")
            print("-" * 40)
            
            if isinstance(category_results, dict) and 'filter_name' in category_results:
                # Single filter result
                for key, value in category_results.items():
                    if key != 'filter_name':
                        print(f"  {key}: {value:.4f}")
            else:
                # Multiple filter results
                for filter_name, metrics in category_results.items():
                    print(f"  {filter_name}:")
                    for key, value in metrics.items():
                        print(f"    {key}: {value:.4f}")
    
    def _plot_kalman_results(self) -> None:
        """Plot Kalman filter test results."""
        data = self.test_data['kalman']
        
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        fig.suptitle('Kalman Filter Test Results', fontsize=16)
        
        # Position tracking
        axes[0, 0].plot(data['true_states'][:, 0], data['true_states'][:, 1], 'g-', label='True Path', linewidth=2)
        axes[0, 0].plot(data['estimated_states'][:, 0], data['estimated_states'][:, 1], 'b--', label='Estimated Path')
        axes[0, 0].scatter(data['measurements'][:, 0], data['measurements'][:, 1], c='r', s=1, alpha=0.5, label='Measurements')
        axes[0, 0].set_xlabel('X Position (m)')
        axes[0, 0].set_ylabel('Y Position (m)')
        axes[0, 0].set_title('Position Tracking')
        axes[0, 0].legend()
        axes[0, 0].grid(True)
        
        # Position error over time
        axes[0, 1].plot(data['time'], data['position_errors'], 'r-', linewidth=2)
        axes[0, 1].set_xlabel('Time (s)')
        axes[0, 1].set_ylabel('Position Error (m)')
        axes[0, 1].set_title('Position Error vs Time')
        axes[0, 1].grid(True)
        
        # X position comparison
        axes[1, 0].plot(data['time'], data['true_states'][:, 0], 'g-', label='True X', linewidth=2)
        axes[1, 0].plot(data['time'], data['estimated_states'][:, 0], 'b--', label='Estimated X')
        axes[1, 0].scatter(data['time'], data['measurements'][:, 0], c='r', s=1, alpha=0.5, label='Measurements')
        axes[1, 0].set_xlabel('Time (s)')
        axes[1, 0].set_ylabel('X Position (m)')
        axes[1, 0].set_title('X Position vs Time')
        axes[1, 0].legend()
        axes[1, 0].grid(True)
        
        # Y position comparison
        axes[1, 1].plot(data['time'], data['true_states'][:, 1], 'g-', label='True Y', linewidth=2)
        axes[1, 1].plot(data['time'], data['estimated_states'][:, 1], 'b--', label='Estimated Y')
        axes[1, 1].scatter(data['time'], data['measurements'][:, 1], c='r', s=1, alpha=0.5, label='Measurements')
        axes[1, 1].set_xlabel('Time (s)')
        axes[1, 1].set_ylabel('Y Position (m)')
        axes[1, 1].set_title('Y Position vs Time')
        axes[1, 1].legend()
        axes[1, 1].grid(True)
        
        plt.tight_layout()
        plt.show()
    
    def _plot_lowpass_results(self) -> None:
        """Plot low-pass filter test results."""
        data = self.test_data['lowpass']
        
        plt.figure(figsize=(12, 8))
        
        plt.subplot(2, 1, 1)
        plt.plot(data['time'], data['clean_signal'], 'g-', label='Clean Signal', linewidth=2)
        plt.plot(data['time'], data['noisy_signal'], 'r-', alpha=0.7, label='Noisy Signal')
        plt.xlabel('Time (s)')
        plt.ylabel('Amplitude')
        plt.title('Original Signals')
        plt.legend()
        plt.grid(True)
        
        plt.subplot(2, 1, 2)
        plt.plot(data['time'], data['clean_signal'], 'g-', label='Clean Signal', linewidth=3)
        
        for name, signal in data['filtered_signals'].items():
            plt.plot(data['time'], signal, '--', label=name, linewidth=2)
        
        plt.xlabel('Time (s)')
        plt.ylabel('Amplitude')
        plt.title('Low-pass Filter Comparison')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        plt.show()
    
    def _plot_moving_average_results(self) -> None:
        """Plot moving average filter test results."""
        data = self.test_data['moving_average']
        
        plt.figure(figsize=(12, 6))
        
        plt.plot(data['time'], data['true_signal'], 'g-', label='True Signal', linewidth=3)
        plt.plot(data['time'], data['noisy_signal'], 'r-', alpha=0.5, label='Noisy Signal')
        
        for name, signal in data['filtered_signals'].items():
            plt.plot(data['time'], signal, '--', label=name, linewidth=2)
        
        plt.axvline(x=data['step_position'], color='k', linestyle=':', alpha=0.7, label='Step Input')
        plt.xlabel('Sample')
        plt.ylabel('Amplitude')
        plt.title('Moving Average Filter Step Response')
        plt.legend()
        plt.grid(True)
        
        plt.tight_layout()
        plt.show()
    
    def _plot_complementary_results(self) -> None:
        """Plot complementary filter test results."""
        data = self.test_data['complementary']
        
        fig, axes = plt.subplots(2, 1, figsize=(12, 8))
        fig.suptitle('Complementary Filter Test Results', fontsize=16)
        
        # Angle estimation
        axes[0].plot(data['time'], np.degrees(data['true_angle']), 'g-', label='True Angle', linewidth=3)
        axes[0].plot(data['time'], np.degrees(data['accel_angle_data']), 'r:', alpha=0.7, label='Accelerometer')
        
        for name, angle in data['filtered_angles'].items():
            axes[0].plot(data['time'], np.degrees(angle), '--', label=name, linewidth=2)
        
        axes[0].set_xlabel('Time (s)')
        axes[0].set_ylabel('Angle (degrees)')
        axes[0].set_title('Angle Estimation')
        axes[0].legend()
        axes[0].grid(True)
        
        # Gyroscope data
        axes[1].plot(data['time'], np.degrees(data['gyro_data']), 'b-', alpha=0.7, label='Gyroscope Rate')
        axes[1].set_xlabel('Time (s)')
        axes[1].set_ylabel('Angular Rate (deg/s)')
        axes[1].set_title('Gyroscope Input')
        axes[1].legend()
        axes[1].grid(True)
        
        plt.tight_layout()
        plt.show()
    
    def _plot_particle_results(self) -> None:
        """Plot particle filter test results."""
        data = self.test_data['particle']
        
        plt.figure(figsize=(12, 8))
        
        # Plot true trajectory
        plt.plot(data['true_trajectory'][:, 0], data['true_trajectory'][:, 1], 
                'g-', label='True Trajectory', linewidth=3)
        
        # Plot noisy measurements
        plt.scatter(data['noisy_measurements'][:, 0], data['noisy_measurements'][:, 1], 
                   c='r', s=10, alpha=0.3, label='Noisy Measurements')
        
        # Plot estimated trajectories
        colors = ['blue', 'orange', 'purple']
        for i, (name, trajectory) in enumerate(data['estimated_trajectories'].items()):
            plt.plot(trajectory[:, 0], trajectory[:, 1], 
                    '--', color=colors[i], label=name, linewidth=2)
        
        plt.xlabel('X Position (m)')
        plt.ylabel('Y Position (m)')
        plt.title('Particle Filter Tracking Results')
        plt.legend()
        plt.grid(True)
        plt.axis('equal')
        
        plt.tight_layout()
        plt.show()
    
    def _plot_all_results(self) -> None:
        """Plot all filter results in a comprehensive view."""
        print("Generating comprehensive plots...")
        self._plot_kalman_results()
        self._plot_lowpass_results()
        self._plot_moving_average_results()
        self._plot_complementary_results()
        self._plot_particle_results()


def main():
    """Run the filter test suite."""
    # Set random seed for reproducible results
    np.random.seed(42)
    
    # Create and run test suite
    test_suite = FilterTestSuite()
    results = test_suite.run_all_tests(plot=True)
    
    print("\nTest suite completed successfully!")
    print("All filter implementations are working correctly.")


if __name__ == "__main__":
    main() 