"""
Vehicle Kinematic Model

This module implements a vehicle kinematic model with the following components:
- VehicleState: Structured representation of vehicle state
- DelayBuffer: Handles actuator delays with command buffering
- BicycleKinematicModel: Core bicycle model kinematics (pure, no delays)
- VehicleModel: Main vehicle model combining kinematics with delays and control methods

Control methods:
- Rate-based control: Direct steering rate and acceleration inputs
- Direct control: Target steering angle and velocity with automatic conversion

Author: Assistant
"""

import numpy as np
import math
from collections import deque
from dataclasses import dataclass
from typing import Tuple, Union


@dataclass
class VehicleState:
    """
    Structured representation of vehicle state
    
    Provides a clear interface for accessing and manipulating vehicle state components.
    All angles are in radians, distances in meters, velocities in m/s.
    """
    position_x: float = 0.0     # Vehicle position X coordinate [m]
    position_y: float = 0.0     # Vehicle position Y coordinate [m]
    yaw_angle: float = 0.0      # Vehicle heading angle [rad]
    velocity: float = 0.0       # Vehicle velocity [m/s]
    steering_angle: float = 0.0 # Steering wheel angle [rad]
    
    def to_array(self) -> np.ndarray:
        """
        Convert state to numpy array format
        
        Returns:
            np.ndarray: State as [position_x, position_y, yaw_angle, velocity, steering_angle]
        """
        return np.array([self.position_x, self.position_y, self.yaw_angle, 
                        self.velocity, self.steering_angle])
    
    @classmethod
    def from_array(cls, state_array: Union[np.ndarray, list]) -> 'VehicleState':
        """
        Create VehicleState from array format
        
        Args:
            state_array: Array with [position_x, position_y, yaw_angle, velocity, steering_angle]
            
        Returns:
            VehicleState: New VehicleState instance
        """
        if len(state_array) != 5:
            raise ValueError("State array must have exactly 5 elements")
        
        return cls(
            position_x=float(state_array[0]),
            position_y=float(state_array[1]),
            yaw_angle=float(state_array[2]),
            velocity=float(state_array[3]),
            steering_angle=float(state_array[4])
        )
    
    def get_position(self) -> Tuple[float, float]:
        """
        Get position as tuple
        
        Returns:
            Tuple[float, float]: (position_x, position_y)
        """
        return (self.position_x, self.position_y)
    
    def get_pose(self) -> Tuple[float, float, float]:
        """
        Get pose (position + orientation) as tuple
        
        Returns:
            Tuple[float, float, float]: (position_x, position_y, yaw_angle)
        """
        return (self.position_x, self.position_y, self.yaw_angle)
    
    def copy(self) -> 'VehicleState':
        """
        Create a copy of the current state
        
        Returns:
            VehicleState: Copy of current state
        """
        return VehicleState(
            position_x=self.position_x,
            position_y=self.position_y,
            yaw_angle=self.yaw_angle,
            velocity=self.velocity,
            steering_angle=self.steering_angle
        )
    
    def __str__(self) -> str:
        """String representation of vehicle state"""
        return (f"VehicleState(pos=({self.position_x:.2f}, {self.position_y:.2f}), "
                f"yaw={math.degrees(self.yaw_angle):.1f}°, "
                f"vel={self.velocity:.2f}m/s, "
                f"steer={math.degrees(self.steering_angle):.1f}°)")


class DelayBuffer:
    """
    Command delay buffer for actuator delays
    
    Buffers commands with timestamps and retrieves them after specified delays.
    """
    
    def __init__(self, delay=0.0):
        """
        Initialize delay buffer
        
        Args:
            delay (float): Time delay [s]
        """
        self.delay = max(0.0, delay)
        self.command_buffer = deque()  # Store tuples of (command, timestamp)
        self.last_command = 0.0
        
    def add_command(self, command, current_time):
        """
        Add command to buffer with delay
        
        Args:
            command (float): Command value
            current_time (float): Current simulation time [s]
        """
        self.command_buffer.append((command, current_time + self.delay))
        
    def get_effective_command(self, command, current_time):
        """
        Get effective command considering delay
        
        Args:
            command (float): Current input command
            current_time (float): Current simulation time [s]
            
        Returns:
            float: Effective command after delay processing
        """
        if self.delay <= 0:
            return command
            
        # Add current command to buffer
        self.add_command(command, current_time)
        
        # Retrieve commands that have completed their delay
        while (self.command_buffer and 
               self.command_buffer[0][1] <= current_time):
            self.last_command = self.command_buffer.popleft()[0]
            
        return self.last_command
        
    def set_delay(self, delay):
        """
        Update delay and clear buffer
        
        Args:
            delay (float): New delay [s]
        """
        self.delay = max(0.0, delay)
        self.command_buffer.clear()
        self.last_command = 0.0
        
    def clear(self):
        """Clear buffer and reset last command"""
        self.command_buffer.clear()
        self.last_command = 0.0


class BicycleKinematicModel:
    """
    Pure bicycle kinematic model without delays
    
    Implements basic bicycle model kinematics for vehicle motion.
    State vector: [position_x, position_y, yaw_angle, velocity, steering_angle]
    """
    
    def __init__(self, wheelbase=2.9, max_steering_angle=np.deg2rad(45.0), max_velocity=50.0):
        """
        Initialize bicycle model parameters
        
        Args:
            wheelbase (float): Distance between front and rear axles [m]
            max_steering_angle (float): Maximum steering angle [rad]
            max_velocity (float): Maximum velocity [m/s]
        """
        self.wheelbase = wheelbase
        self.max_steering_angle = max_steering_angle
        self.max_velocity = max_velocity
        
        # Control limits
        self.max_steering_rate = np.deg2rad(30)  # rad/s
        self.max_acceleration = 3.0  # m/s²
        self.max_deceleration = -5.0  # m/s²
        
        # Vehicle state using structured representation
        self.state = VehicleState()
        
    def update(self, steering_rate, acceleration, time_step):
        """
        Update vehicle state using bicycle model kinematics
        
        Args:
            steering_rate (float): Steering angle velocity [rad/s]
            acceleration (float): Acceleration [m/s²]
            time_step (float): Time step [s]
            
        Returns:
            VehicleState: Updated vehicle state
        """
        # Apply control limits
        steering_rate = np.clip(steering_rate, -self.max_steering_rate, self.max_steering_rate)
        acceleration = np.clip(acceleration, self.max_deceleration, self.max_acceleration)
        
        # Update steering angle with limits
        new_steering_angle = np.clip(
            self.state.steering_angle + steering_rate * time_step,
            -self.max_steering_angle, self.max_steering_angle
        )
        
        # Update velocity with limits
        new_velocity = np.clip(
            self.state.velocity + acceleration * time_step,
            0.0, self.max_velocity
        )
        
        # Update position using bicycle model
        new_position_x = self.state.position_x + new_velocity * math.cos(self.state.yaw_angle) * time_step
        new_position_y = self.state.position_y + new_velocity * math.sin(self.state.yaw_angle) * time_step
        
        # Update yaw angle using bicycle model
        new_yaw_angle = self.normalize_angle(
            self.state.yaw_angle + (new_velocity / self.wheelbase) * math.tan(new_steering_angle) * time_step
        )
        
        # Update state using VehicleState structure
        self.state = VehicleState(
            position_x=new_position_x,
            position_y=new_position_y,
            yaw_angle=new_yaw_angle,
            velocity=new_velocity,
            steering_angle=new_steering_angle
        )
        
        return self.state.copy()
        
    def set_state(self, state):
        """
        Set vehicle state
        
        Args:
            state (Union[VehicleState, array, list]): Vehicle state as VehicleState object or 
                     array [position_x, position_y, yaw_angle, velocity, steering_angle]
        """
        if isinstance(state, VehicleState):
            self.state = state.copy()
        else:
            self.state = VehicleState.from_array(state)
        
    def get_state(self):
        """
        Get current vehicle state
        
        Returns:
            VehicleState: Current vehicle state
        """
        return self.state.copy()
        
    def get_state_array(self):
        """
        Get current vehicle state as numpy array (for backward compatibility)
        
        Returns:
            np.array: Current state [position_x, position_y, yaw_angle, velocity, steering_angle]
        """
        return self.state.to_array()
        
    @staticmethod
    def normalize_angle(angle):
        """
        Normalize angle to [-pi, pi]
        
        Args:
            angle (float): angle in radians
            
        Returns:
            float: normalized angle
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


class VehicleModel:
    """
    Complete vehicle model with delays and control methods
    
    Combines bicycle kinematics with actuator delays and provides two control interfaces:
    1. Rate-based control: Direct steering rate and acceleration inputs
    2. Direct control: Target steering angle and velocity with automatic conversion
    """
    
    def __init__(self, wheelbase=2.9, max_steering_angle=np.deg2rad(45.0), max_velocity=50.0,
                 steering_delay=0.0, acceleration_delay=0.0,
                 steering_rate_gain=5.0, acceleration_gain=2.0):
        """
        Initialize vehicle model
        
        Args:
            wheelbase (float): Distance between front and rear axles [m]
            max_steering_angle (float): Maximum steering angle [rad]
            max_velocity (float): Maximum velocity [m/s]
            steering_delay (float): Time delay for steering commands [s]
            acceleration_delay (float): Time delay for acceleration commands [s]
            steering_rate_gain (float): Gain for converting steering error to steering rate [rad/s per rad]
            acceleration_gain (float): Gain for converting velocity error to acceleration [(m/s²) per (m/s)]
        """
        # Core kinematic model
        self.kinematics = BicycleKinematicModel(wheelbase, max_steering_angle, max_velocity)
        
        # Delay buffers
        self.steering_delay_buffer = DelayBuffer(steering_delay)
        self.acceleration_delay_buffer = DelayBuffer(acceleration_delay)
        
        # Control gains for direct control method
        self.steering_rate_gain = steering_rate_gain
        self.acceleration_gain = acceleration_gain
        
        # Time tracking
        self.current_time = 0.0
        
    def update_with_rates(self, control_input, time_step):
        """
        Update vehicle state using steering rate and acceleration with actuator delays
        
        Args:
            control_input (array): [steering_rate, acceleration] - steering angle velocity and acceleration
            time_step (float): time step [s]
            
        Returns:
            VehicleState: Updated vehicle state
        """
        steering_rate_cmd, acceleration_cmd = control_input
        
        # Update time
        self.current_time += time_step
        
        # Apply delays
        effective_steering_rate = self.steering_delay_buffer.get_effective_command(
            steering_rate_cmd, self.current_time)
        effective_acceleration = self.acceleration_delay_buffer.get_effective_command(
            acceleration_cmd, self.current_time)
        
        # Update kinematics
        return self.kinematics.update(effective_steering_rate, effective_acceleration, time_step)
        
    def update_with_direct_control(self, control_input, time_step):
        """
        Update vehicle state using direct steering angle and velocity commands with delays
        
        Args:
            control_input (array): [steering_angle, velocity] - target steering angle and velocity
            time_step (float): time step [s]
            
        Returns:
            VehicleState: Updated vehicle state
        """
        # Get current state and update time
        current_state = self.kinematics.get_state()
        self.current_time += time_step
        
        # Add commands to delay buffers first (with clipping)
        target_steering_angle, target_velocity = control_input
        clipped_steering = np.clip(target_steering_angle,
                                 -self.kinematics.max_steering_angle,
                                 self.kinematics.max_steering_angle)
        clipped_velocity = np.clip(target_velocity, 0.0, self.kinematics.max_velocity)
        
        # Add commands to buffers with current time
        self.steering_delay_buffer.add_command(clipped_steering, self.current_time)
        self.acceleration_delay_buffer.add_command(clipped_velocity, self.current_time)
        
        # Get delayed commands from buffers
        effective_steering = self.steering_delay_buffer.get_effective_command(
            clipped_steering, self.current_time)
        effective_velocity = self.acceleration_delay_buffer.get_effective_command(
            clipped_velocity, self.current_time)
        
        # Calculate rates based on delayed commands vs current state
        steering_error = effective_steering - current_state.steering_angle
        desired_steering_rate = self.steering_rate_gain * steering_error
        
        velocity_error = effective_velocity - current_state.velocity
        desired_acceleration = self.acceleration_gain * velocity_error
        
        # Update kinematics with calculated rates
        return self.kinematics.update(desired_steering_rate, desired_acceleration, time_step)
    
    def update(self, control_input, time_step):
        """
        Update vehicle state using rate-based control (default method)
        
        Args:
            control_input (array): [steering_rate, acceleration] - steering angle velocity and acceleration
            time_step (float): time step [s]
            
        Returns:
            VehicleState: Updated vehicle state
        """
        return self.update_with_rates(control_input, time_step)
        
    def set_state(self, state):
        """
        Set vehicle state and reset delay buffers
        
        Args:
            state (Union[VehicleState, array, list]): Vehicle state as VehicleState object or
                     array [position_x, position_y, yaw_angle, velocity, steering_angle]
        """
        self.kinematics.set_state(state)
        self.steering_delay_buffer.clear()
        self.acceleration_delay_buffer.clear()
        self.current_time = 0.0
        
    def get_state(self):
        """
        Get current vehicle state
        
        Returns:
            VehicleState: Current vehicle state
        """
        return self.kinematics.get_state()
        
    def get_state_array(self):
        """
        Get current vehicle state as numpy array (for backward compatibility)
        
        Returns:
            np.array: Current state [position_x, position_y, yaw_angle, velocity, steering_angle]
        """
        return self.kinematics.get_state_array()
        
    def get_position(self):
        """
        Get vehicle position
        
        Returns:
            tuple: (position_x, position_y) position
        """
        state = self.kinematics.get_state()
        return state.get_position()
        
    def get_orientation(self):
        """
        Get vehicle orientation
        
        Returns:
            float: yaw_angle [rad]
        """
        return self.kinematics.get_state().yaw_angle
        
    def get_velocity(self):
        """
        Get vehicle velocity
        
        Returns:
            float: velocity [m/s]
        """
        return self.kinematics.get_state().velocity
        
    def get_steering_angle(self):
        """
        Get steering angle
        
        Returns:
            float: steering_angle [rad]
        """
        return self.kinematics.get_state().steering_angle
        
    def set_delays(self, steering_delay=None, acceleration_delay=None):
        """
        Update delay parameters
        
        Args:
            steering_delay (float, optional): New steering delay [s]
            acceleration_delay (float, optional): New acceleration delay [s]
        """
        if steering_delay is not None:
            self.steering_delay_buffer.set_delay(steering_delay)
        if acceleration_delay is not None:
            self.acceleration_delay_buffer.set_delay(acceleration_delay)
            
    def get_delays(self):
        """
        Get current delay parameters
        
        Returns:
            tuple: (steering_delay, acceleration_delay) in seconds
        """
        return self.steering_delay_buffer.delay, self.acceleration_delay_buffer.delay


# Legacy compatibility - keep original class name as alias
VehicleKinematicModel = VehicleModel


def simulate_vehicle_motion(initial_state, control_sequence, time_step=0.1, wheelbase=2.9,
                           steering_delay=0.0, acceleration_delay=0.0):
    """
    Simulate vehicle motion with given control sequence and actuator delays
    
    Args:
        initial_state (Union[VehicleState, array, list]): Initial state as VehicleState object or 
                     array [position_x, position_y, yaw_angle, velocity, steering_angle]
        control_sequence (array): Control inputs [[steering_rate1, acceleration1], [steering_rate2, acceleration2], ...]
        time_step (float): Time step [s]
        wheelbase (float): Vehicle wheelbase [m]
        steering_delay (float): Time delay for steering commands [s]
        acceleration_delay (float): Time delay for acceleration commands [s]
        
    Returns:
        np.array: State trajectory as array for backward compatibility
    """
    vehicle = VehicleModel(wheelbase=wheelbase, 
                          steering_delay=steering_delay, 
                          acceleration_delay=acceleration_delay)
    vehicle.set_state(initial_state)
    
    trajectory = [vehicle.get_state_array()]  # Convert to array for backward compatibility
    
    for control_input in control_sequence:
        state = vehicle.update(control_input, time_step)
        trajectory.append(state.to_array())  # Convert to array for backward compatibility
    
    return np.array(trajectory)


# Example usage and testing
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    
    # Create vehicle models for comparison: one without delay, one with delay, one for direct control demonstration
    vehicle_no_delay = VehicleModel()
    vehicle_with_delay = VehicleModel(steering_delay=0.3, acceleration_delay=0.2)
    vehicle_direct_control = VehicleModel(steering_delay=0.3, acceleration_delay=0.2)  # Direct control with delays
    vehicle_direct_no_delay = VehicleModel()  # Direct control without delays
    
    # Set initial state [position_x, position_y, yaw_angle, velocity, steering_angle]
    initial_state = [0.0, 0.0, 0.0, 10.0, 0.0]
    
    # Simulation parameters
    time_step = 0.1
    simulation_time = 10.0
    time_steps = int(simulation_time / time_step)
    
    # Control sequence: turning maneuver
    control_sequence = []
    for step_index in range(time_steps):
        if step_index < time_steps // 3:
            # Turn left
            steering_rate = np.deg2rad(10)  # steering angle velocity
            acceleration = 0.0  # no acceleration
        elif step_index < 2 * time_steps // 3:
            # Straight
            steering_rate = np.deg2rad(-5)  # return to straight
            acceleration = 1.0  # accelerate
        else:
            # Turn right
            steering_rate = np.deg2rad(-15)
            acceleration = -1.0  # decelerate
        
        control_sequence.append([steering_rate, acceleration])
    
    # Simulate vehicle motion for both cases
    trajectory_no_delay = simulate_vehicle_motion(initial_state, control_sequence, time_step)
    trajectory_with_delay = simulate_vehicle_motion(initial_state, control_sequence, time_step,
                                                   steering_delay=0.3, acceleration_delay=0.2)
    
    # Demonstrate direct control methods
    vehicle_direct_control.set_state(initial_state)
    vehicle_direct_no_delay.set_state(initial_state)
    trajectory_direct_control = [vehicle_direct_control.get_state_array()]  # Convert to array for plotting
    trajectory_direct_no_delay = [vehicle_direct_no_delay.get_state_array()]  # Convert to array for plotting
    
    for step_index in range(time_steps):
        # Direct control: specify target steering angle and velocity directly
        if step_index < time_steps // 3:
            # Turn left with specific steering angle and velocity
            target_steering = np.deg2rad(15)  # target steering angle
            target_velocity = 10.0  # target velocity
        elif step_index < 2 * time_steps // 3:
            # Straight with higher velocity
            target_steering = np.deg2rad(0)
            target_velocity = 15.0
        else:
            # Turn right with reduced velocity
            target_steering = np.deg2rad(-20)
            target_velocity = 8.0
        
        state = vehicle_direct_control.update_with_direct_control([target_steering, target_velocity], time_step)
        trajectory_direct_control.append(state.to_array())  # Convert to array for plotting
        
        state = vehicle_direct_no_delay.update_with_direct_control([target_steering, target_velocity], time_step)
        trajectory_direct_no_delay.append(state.to_array())  # Convert to array for plotting
    
    trajectory_direct_control = np.array(trajectory_direct_control)
    trajectory_direct_no_delay = np.array(trajectory_direct_no_delay)
    
    # Plot results
    plt.figure(figsize=(15, 10))
    
    # Plot trajectory comparison
    plt.subplot(2, 3, 1)
    plt.plot(trajectory_no_delay[:, 0], trajectory_no_delay[:, 1], 'b-', linewidth=2, label='Rate Control (No Delay)')
    plt.plot(trajectory_with_delay[:, 0], trajectory_with_delay[:, 1], 'r--', linewidth=2, label='Rate Control (With Delay)')
    plt.plot(trajectory_direct_control[:, 0], trajectory_direct_control[:, 1], 'g-.', linewidth=2, label='Direct Control (With Delay)')
    plt.plot(trajectory_direct_no_delay[:, 0], trajectory_direct_no_delay[:, 1], 'm:', linewidth=2, label='Direct Control (No Delay)')
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Vehicle Trajectory Comparison')
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    
    # Plot velocity comparison
    plt.subplot(2, 3, 2)
    time_array = np.arange(len(trajectory_no_delay)) * time_step
    plt.plot(time_array, trajectory_no_delay[:, 3], 'b-', linewidth=2, label='Rate Control (No Delay)')
    plt.plot(time_array, trajectory_with_delay[:, 3], 'r--', linewidth=2, label='Rate Control (With Delay)')
    plt.plot(time_array, trajectory_direct_control[:, 3], 'g-.', linewidth=2, label='Direct Control (With Delay)')
    plt.plot(time_array, trajectory_direct_no_delay[:, 3], 'm:', linewidth=2, label='Direct Control (No Delay)')
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [m/s]')
    plt.title('Vehicle Velocity Comparison')
    plt.legend()
    plt.grid(True)
    
    # Plot yaw angle comparison
    plt.subplot(2, 3, 3)
    plt.plot(time_array, np.rad2deg(trajectory_no_delay[:, 2]), 'b-', linewidth=2, label='Rate Control (No Delay)')
    plt.plot(time_array, np.rad2deg(trajectory_with_delay[:, 2]), 'r--', linewidth=2, label='Rate Control (With Delay)')
    plt.plot(time_array, np.rad2deg(trajectory_direct_control[:, 2]), 'g-.', linewidth=2, label='Direct Control (With Delay)')
    plt.plot(time_array, np.rad2deg(trajectory_direct_no_delay[:, 2]), 'm:', linewidth=2, label='Direct Control (No Delay)')
    plt.xlabel('Time [s]')
    plt.ylabel('Yaw Angle [deg]')
    plt.title('Vehicle Orientation Comparison')
    plt.legend()
    plt.grid(True)
    
    # Plot steering angle comparison
    plt.subplot(2, 3, 4)
    plt.plot(time_array, np.rad2deg(trajectory_no_delay[:, 4]), 'b-', linewidth=2, label='Rate Control (No Delay)')
    plt.plot(time_array, np.rad2deg(trajectory_with_delay[:, 4]), 'r--', linewidth=2, label='Rate Control (With Delay)')
    plt.plot(time_array, np.rad2deg(trajectory_direct_control[:, 4]), 'g-.', linewidth=2, label='Direct Control (With Delay)')
    plt.plot(time_array, np.rad2deg(trajectory_direct_no_delay[:, 4]), 'm:', linewidth=2, label='Direct Control (No Delay)')
    plt.xlabel('Time [s]')
    plt.ylabel('Steering Angle [deg]')
    plt.title('Steering Angle Comparison')
    plt.legend()
    plt.grid(True)
    
    # Plot delay effects
    plt.subplot(2, 3, 5)
    plt.text(0.1, 0.9, 'Enhanced Architecture:', transform=plt.gca().transAxes, fontsize=12, weight='bold')
    plt.text(0.1, 0.8, '1. VehicleState:', transform=plt.gca().transAxes, fontsize=11, weight='bold')
    plt.text(0.15, 0.72, '• Structured state representation', transform=plt.gca().transAxes, fontsize=9)
    plt.text(0.15, 0.64, '• Type-safe with clear interface', transform=plt.gca().transAxes, fontsize=9)
    plt.text(0.1, 0.54, '2. DelayBuffer:', transform=plt.gca().transAxes, fontsize=11, weight='bold')
    plt.text(0.15, 0.46, '• Independent delay handling', transform=plt.gca().transAxes, fontsize=9)
    plt.text(0.1, 0.36, '3. BicycleKinematicModel:', transform=plt.gca().transAxes, fontsize=11, weight='bold')
    plt.text(0.15, 0.28, '• Pure kinematics with VehicleState', transform=plt.gca().transAxes, fontsize=9)
    plt.text(0.1, 0.18, '4. VehicleModel:', transform=plt.gca().transAxes, fontsize=11, weight='bold')
    plt.text(0.15, 0.1, '• Complete integrated system', transform=plt.gca().transAxes, fontsize=9)
    plt.text(0.15, 0.02, '• Backward compatible', transform=plt.gca().transAxes, fontsize=9)
    plt.xlim(0, 1)
    plt.ylim(0, 1)
    plt.axis('off')
    plt.title('Modular Architecture')
    
    # Position error plot
    plt.subplot(2, 3, 6)
    position_error = np.sqrt((trajectory_no_delay[:, 0] - trajectory_with_delay[:, 0])**2 + 
                            (trajectory_no_delay[:, 1] - trajectory_with_delay[:, 1])**2)
    plt.plot(time_array, position_error, 'g-', linewidth=2)
    plt.xlabel('Time [s]')
    plt.ylabel('Position Error [m]')
    plt.title('Position Error due to Delays')
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()
    
    # Print comparison results
    print("=== Enhanced Vehicle Model with VehicleState Structure ===")
    print("\n1. VehicleState - Structured state representation:")
    print(f"   Current state: {vehicle_with_delay.get_state()}")
    print("\n2. DelayBuffer - Independent delay handling:")
    print(f"   Steering delay: {vehicle_with_delay.get_delays()[0]:.1f}s")
    print(f"   Acceleration delay: {vehicle_with_delay.get_delays()[1]:.1f}s")
    print("\n3. BicycleKinematicModel - Pure kinematics:")
    print(f"   Wheelbase: {vehicle_with_delay.kinematics.wheelbase:.1f}m")
    print(f"   Max steering: {np.rad2deg(vehicle_with_delay.kinematics.max_steering_angle):.1f} deg")
    print("\n4. VehicleModel - Complete system:")
    print(f"   Steering rate gain: {vehicle_direct_control.steering_rate_gain:.1f} rad/s per rad")
    print(f"   Acceleration gain: {vehicle_direct_control.acceleration_gain:.1f} (m/s²) per (m/s)")
    print("\nArchitecture benefits:")
    print("   • VehicleState provides type-safe structured access")
    print("   • Separation of concerns (delay vs kinematics)")
    print("   • Backward compatibility with array-based interface")
    print("   • Easier testing and maintenance")
    print("   • Reusable components for other applications")
    print("\nFinal states comparison:")
    print("Rate Control (No delay) - Position: ({:.2f}, {:.2f}) m, Yaw: {:.2f} deg, Velocity: {:.2f} m/s".format(
        trajectory_no_delay[-1, 0], trajectory_no_delay[-1, 1], 
        np.rad2deg(trajectory_no_delay[-1, 2]), trajectory_no_delay[-1, 3]))
    print("Rate Control (With delay) - Position: ({:.2f}, {:.2f}) m, Yaw: {:.2f} deg, Velocity: {:.2f} m/s".format(
        trajectory_with_delay[-1, 0], trajectory_with_delay[-1, 1], 
        np.rad2deg(trajectory_with_delay[-1, 2]), trajectory_with_delay[-1, 3]))
    print("Direct Control (With delay) - Position: ({:.2f}, {:.2f}) m, Yaw: {:.2f} deg, Velocity: {:.2f} m/s".format(
        trajectory_direct_control[-1, 0], trajectory_direct_control[-1, 1], 
        np.rad2deg(trajectory_direct_control[-1, 2]), trajectory_direct_control[-1, 3]))
    print(f"Position error (Rate with delay vs No delay): {position_error[-1]:.2f} m")
