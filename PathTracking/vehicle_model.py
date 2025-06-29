"""
Vehicle Kinematic Model

This module implements a vehicle kinematic model with the following:
- Control inputs: steering angle velocity (delta_dot) and acceleration (a)
- State outputs: position (x, y), orientation (yaw), velocity (v), and steering angle (delta)

Author: Assistant
"""

import numpy as np
import math


class VehicleKinematicModel:
    """
    Vehicle kinematic model with bicycle model approximation
    
    State vector: [x, y, yaw, v, delta]
    - x, y: position coordinates
    - yaw: vehicle orientation angle
    - v: velocity
    - delta: steering angle
    
    Control inputs: [delta_dot, a]
    - delta_dot: steering angle velocity
    - a: acceleration
    """
    
    def __init__(self, wheelbase=2.9, max_steer=np.deg2rad(45.0), max_speed=50.0):
        """
        Initialize vehicle model parameters
        
        Args:
            wheelbase (float): Distance between front and rear axles [m]
            max_steer (float): Maximum steering angle [rad]
            max_speed (float): Maximum velocity [m/s]
        """
        self.L = wheelbase  # wheelbase length
        self.max_steer = max_steer
        self.max_speed = max_speed
        
        # State vector: [x, y, yaw, v, delta]
        self.state = np.zeros(5)
        
    def update(self, control_input, dt):
        """
        Update vehicle state using kinematic model
        
        Args:
            control_input (array): [delta_dot, a] - steering angle velocity and acceleration
            dt (float): time step [s]
            
        Returns:
            np.array: Updated state vector [x, y, yaw, v, delta]
        """
        delta_dot, a = control_input
        x, y, yaw, v, delta = self.state
        
        # Apply control limits
        delta_dot = np.clip(delta_dot, -np.deg2rad(30), np.deg2rad(30))  # steering rate limit
        a = np.clip(a, -5.0, 3.0)  # acceleration limits
        
        # Update steering angle
        delta_new = delta + delta_dot * dt
        delta_new = np.clip(delta_new, -self.max_steer, self.max_steer)
        
        # Update velocity
        v_new = v + a * dt
        v_new = np.clip(v_new, 0.0, self.max_speed)
        
        # Kinematic model equations (bicycle model)
        if abs(v_new) > 0.01:  # avoid division by zero
            # Update position and orientation
            x_new = x + v_new * math.cos(yaw) * dt
            y_new = y + v_new * math.sin(yaw) * dt
            yaw_new = yaw + (v_new / self.L) * math.tan(delta_new) * dt
            
            # Normalize yaw angle to [-pi, pi]
            yaw_new = self.normalize_angle(yaw_new)
        else:
            # Vehicle is stationary
            x_new = x
            y_new = y
            yaw_new = yaw
        
        # Update state
        self.state = np.array([x_new, y_new, yaw_new, v_new, delta_new])
        
        return self.state.copy()
    
    def set_state(self, state):
        """
        Set vehicle state
        
        Args:
            state (array): [x, y, yaw, v, delta]
        """
        self.state = np.array(state)
        
    def get_state(self):
        """
        Get current vehicle state
        
        Returns:
            np.array: Current state [x, y, yaw, v, delta]
        """
        return self.state.copy()
    
    def get_position(self):
        """
        Get vehicle position
        
        Returns:
            tuple: (x, y) position
        """
        return self.state[0], self.state[1]
    
    def get_orientation(self):
        """
        Get vehicle orientation
        
        Returns:
            float: yaw angle [rad]
        """
        return self.state[2]
    
    def get_velocity(self):
        """
        Get vehicle velocity
        
        Returns:
            float: velocity [m/s]
        """
        return self.state[3]
    
    def get_steering_angle(self):
        """
        Get steering angle
        
        Returns:
            float: steering angle [rad]
        """
        return self.state[4]
    
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


def simulate_vehicle_motion(initial_state, control_sequence, dt=0.1, wheelbase=2.9):
    """
    Simulate vehicle motion with given control sequence
    
    Args:
        initial_state (array): Initial state [x, y, yaw, v, delta]
        control_sequence (array): Control inputs [[delta_dot1, a1], [delta_dot2, a2], ...]
        dt (float): Time step [s]
        wheelbase (float): Vehicle wheelbase [m]
        
    Returns:
        np.array: State trajectory
    """
    vehicle = VehicleKinematicModel(wheelbase=wheelbase)
    vehicle.set_state(initial_state)
    
    trajectory = [vehicle.get_state()]
    
    for control in control_sequence:
        state = vehicle.update(control, dt)
        trajectory.append(state)
    
    return np.array(trajectory)


# Example usage and testing
if __name__ == "__main__":
    import matplotlib.pyplot as plt
    
    # Create vehicle model
    vehicle = VehicleKinematicModel()
    
    # Set initial state [x, y, yaw, v, delta]
    initial_state = [0.0, 0.0, 0.0, 10.0, 0.0]
    vehicle.set_state(initial_state)
    
    # Simulation parameters
    dt = 0.1
    simulation_time = 10.0
    steps = int(simulation_time / dt)
    
    # Control sequence: turning maneuver
    control_sequence = []
    for i in range(steps):
        if i < steps // 3:
            # Turn left
            delta_dot = np.deg2rad(10)  # steering angle velocity
            a = 0.0  # no acceleration
        elif i < 2 * steps // 3:
            # Straight
            delta_dot = np.deg2rad(-5)  # return to straight
            a = 1.0  # accelerate
        else:
            # Turn right
            delta_dot = np.deg2rad(-15)
            a = -1.0  # decelerate
        
        control_sequence.append([delta_dot, a])
    
    # Simulate vehicle motion
    trajectory = simulate_vehicle_motion(initial_state, control_sequence, dt)
    
    # Plot results
    plt.figure(figsize=(12, 8))
    
    # Plot trajectory
    plt.subplot(2, 2, 1)
    plt.plot(trajectory[:, 0], trajectory[:, 1], 'b-', linewidth=2)
    plt.xlabel('X [m]')
    plt.ylabel('Y [m]')
    plt.title('Vehicle Trajectory')
    plt.grid(True)
    plt.axis('equal')
    
    # Plot velocity
    plt.subplot(2, 2, 2)
    time = np.arange(len(trajectory)) * dt
    plt.plot(time, trajectory[:, 3], 'r-', linewidth=2)
    plt.xlabel('Time [s]')
    plt.ylabel('Velocity [m/s]')
    plt.title('Vehicle Velocity')
    plt.grid(True)
    
    # Plot yaw angle
    plt.subplot(2, 2, 3)
    plt.plot(time, np.rad2deg(trajectory[:, 2]), 'g-', linewidth=2)
    plt.xlabel('Time [s]')
    plt.ylabel('Yaw Angle [deg]')
    plt.title('Vehicle Orientation')
    plt.grid(True)
    
    # Plot steering angle
    plt.subplot(2, 2, 4)
    plt.plot(time, np.rad2deg(trajectory[:, 4]), 'm-', linewidth=2)
    plt.xlabel('Time [s]')
    plt.ylabel('Steering Angle [deg]')
    plt.title('Steering Angle')
    plt.grid(True)
    
    plt.tight_layout()
    plt.show()
    
    # Print final state
    print("Final vehicle state:")
    print(f"Position: ({trajectory[-1, 0]:.2f}, {trajectory[-1, 1]:.2f}) m")
    print(f"Yaw: {np.rad2deg(trajectory[-1, 2]):.2f} deg")
    print(f"Velocity: {trajectory[-1, 3]:.2f} m/s")
    print(f"Steering angle: {np.rad2deg(trajectory[-1, 4]):.2f} deg")
