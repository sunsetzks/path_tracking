"""
Vehicle Lateral Dynamics Simulation with Center of Mass Reference
Vehicle Lateral Dynamics Simulation - Center of Mass Reference

This simulation demonstrates the lateral dynamics equations:
1. Lateral force balance: m(v_y_dot + v_x * psi_dot) = F_yf * cos(delta) + F_yr
2. Yaw moment balance: I_z * psi_dot_dot = l_f * F_yf * cos(delta) - l_r * F_yr

Key Variables:
- v_x, v_y: longitudinal/lateral velocity at center of mass
- psi_dot: yaw rate (rotation around center of mass)
- F_yf, F_yr: front/rear tire lateral forces
- delta: steering angle
"""

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation

class VehicleLateralDynamics:
    """Vehicle lateral dynamics model based on center of mass reference"""
    
    def __init__(self):
        # Vehicle parameters (typical passenger car)
        self.m = 1500.0      # Vehicle mass [kg]
        self.I_z = 2500.0    # Yaw moment of inertia [kg*m^2]
        self.l_f = 1.2       # Distance from CG to front axle [m]
        self.l_r = 1.6       # Distance from CG to rear axle [m]
        self.L = self.l_f + self.l_r  # Wheelbase [m]
        
        # Tire parameters (linear tire model)
        self.C_f = 80000.0   # Front tire cornering stiffness [N/rad]
        self.C_r = 80000.0   # Rear tire cornering stiffness [N/rad]
        
        # Simulation state: [v_y, psi_dot, psi, x, y]
        self.state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        
    def tire_model(self, slip_angle):
        """
        Linear tire model: F_y = -C * alpha
        slip_angle: tire slip angle [rad]
        Returns: lateral force [N]
        """
        return -slip_angle  # Normalized for visualization
    
    def calculate_slip_angles(self, v_x, v_y, psi_dot, delta):
        """
        Calculate front and rear tire slip angles
        
        Args:
            v_x: longitudinal velocity [m/s]
            v_y: lateral velocity [m/s]
            psi_dot: yaw rate [rad/s]
            delta: steering angle [rad]
            
        Returns:
            alpha_f, alpha_r: front and rear slip angles [rad]
        """
        # Front tire slip angle
        if abs(v_x) > 0.1:  # Avoid division by zero
            alpha_f = delta - np.arctan2(v_y + self.l_f * psi_dot, v_x)
        else:
            alpha_f = 0.0
            
        # Rear tire slip angle
        if abs(v_x) > 0.1:
            alpha_r = -np.arctan2(v_y - self.l_r * psi_dot, v_x)
        else:
            alpha_r = 0.0
            
        return alpha_f, alpha_r
    
    def dynamics(self, t, state, v_x, delta):
        """
        Vehicle lateral dynamics equations
        
        State: [v_y, psi_dot, psi, x, y]
        
        Returns: state derivatives
        """
        v_y, psi_dot, psi, x, y = state
        
        # Calculate tire slip angles
        alpha_f, alpha_r = self.calculate_slip_angles(v_x, v_y, psi_dot, delta)
        
        # Calculate tire forces (linear tire model)
        F_yf = self.C_f * alpha_f
        F_yr = self.C_r * alpha_r
        
        # Lateral dynamics equations
        # Equation 1: Lateral force balance
        # m(v_y_dot + v_x * psi_dot) = F_yf * cos(delta) + F_yr
        v_y_dot = (F_yf * np.cos(delta) + F_yr) / self.m - v_x * psi_dot
        
        # Equation 2: Yaw moment balance
        # I_z * psi_dot_dot = l_f * F_yf * cos(delta) - l_r * F_yr
        psi_dot_dot = (self.l_f * F_yf * np.cos(delta) - self.l_r * F_yr) / self.I_z
        
        # Position dynamics (global coordinates)
        x_dot = v_x * np.cos(psi) - v_y * np.sin(psi)
        y_dot = v_x * np.sin(psi) + v_y * np.cos(psi)
        
        return np.array([v_y_dot, psi_dot_dot, psi_dot, x_dot, y_dot])
    
    def simulate(self, t_span, v_x_profile, delta_profile):
        """
        Simulate vehicle lateral dynamics
        
        Args:
            t_span: time span [start, end]
            v_x_profile: function that returns v_x given time
            delta_profile: function that returns steering angle given time
            
        Returns:
            t: time array
            states: state history
            forces: tire force history
        """
        def dynamics_wrapper(t, state):
            v_x = v_x_profile(t)
            delta = delta_profile(t)
            return self.dynamics(t, state, v_x, delta)
        
        # Solve ODE
        sol = solve_ivp(dynamics_wrapper, t_span, self.state, 
                       dense_output=True, rtol=1e-6)
        
        # Extract results
        t = np.linspace(t_span[0], t_span[1], 1000)
        states = sol.sol(t).T
        
        # Calculate forces for each time step
        forces = []
        for i, time in enumerate(t):
            v_x = v_x_profile(time)
            delta = delta_profile(time)
            v_y, psi_dot = states[i, 0], states[i, 1]
            
            alpha_f, alpha_r = self.calculate_slip_angles(v_x, v_y, psi_dot, delta)
            F_yf = self.C_f * alpha_f
            F_yr = self.C_r * alpha_r
            
            forces.append([F_yf, F_yr, alpha_f, alpha_r])
        
        forces = np.array(forces)
        
        return t, states, forces

def create_simulation_scenarios():
    """Create different simulation scenarios to demonstrate the dynamics"""
    
    # Scenario 1: Step steering input
    def step_steering_scenario():
        def v_x_profile(t):
            return 20.0  # Constant speed 20 m/s
        
        def delta_profile(t):
            if t < 1.0:
                return 0.0
            elif t < 5.0:
                return 0.1  # 0.1 rad = ~5.7 degrees
            else:
                return 0.0
        
        return v_x_profile, delta_profile, "Step Steering Input"
    
    # Scenario 2: Sinusoidal steering (slalom)
    def sinusoidal_steering_scenario():
        def v_x_profile(t):
            return 15.0  # Constant speed 15 m/s
        
        def delta_profile(t):
            return 0.05 * np.sin(0.5 * t)  # Sinusoidal steering
        
        return v_x_profile, delta_profile, "Sinusoidal Steering (Slalom)"
    
    # Scenario 3: Increasing speed with constant steering
    def speed_variation_scenario():
        def v_x_profile(t):
            return 10.0 + 2.0 * t  # Increasing speed
        
        def delta_profile(t):
            return 0.05  # Constant steering angle
        
        return v_x_profile, delta_profile, "Increasing Speed with Constant Steering"
    
    return [step_steering_scenario(), sinusoidal_steering_scenario(), speed_variation_scenario()]

def plot_simulation_results(t, states, forces, scenario_name):
    """Plot comprehensive simulation results"""
    
    fig, axes = plt.subplots(3, 3, figsize=(15, 12))
    fig.suptitle(f'Vehicle Lateral Dynamics Simulation: {scenario_name}', fontsize=16)
    
    v_y, psi_dot, psi, x, y = states.T
    F_yf, F_yr, alpha_f, alpha_r = forces.T
    
    # Row 1: Vehicle motion
    axes[0, 0].plot(t, v_y)
    axes[0, 0].set_title('Lateral Velocity v_y')
    axes[0, 0].set_xlabel('Time [s]')
    axes[0, 0].set_ylabel('v_y [m/s]')
    axes[0, 0].grid(True)
    
    axes[0, 1].plot(t, psi_dot * 180 / np.pi)
    axes[0, 1].set_title('Yaw Rate ψ̇')
    axes[0, 1].set_xlabel('Time [s]')
    axes[0, 1].set_ylabel('ψ̇ [deg/s]')
    axes[0, 1].grid(True)
    
    axes[0, 2].plot(x, y)
    axes[0, 2].set_title('Vehicle Trajectory')
    axes[0, 2].set_xlabel('X [m]')
    axes[0, 2].set_ylabel('Y [m]')
    axes[0, 2].grid(True)
    axes[0, 2].axis('equal')
    
    # Row 2: Tire forces
    axes[1, 0].plot(t, F_yf / 1000, label='Front')
    axes[1, 0].plot(t, F_yr / 1000, label='Rear')
    axes[1, 0].set_title('Tire Lateral Forces')
    axes[1, 0].set_xlabel('Time [s]')
    axes[1, 0].set_ylabel('Force [kN]')
    axes[1, 0].legend()
    axes[1, 0].grid(True)
    
    axes[1, 1].plot(t, alpha_f * 180 / np.pi, label='Front')
    axes[1, 1].plot(t, alpha_r * 180 / np.pi, label='Rear')
    axes[1, 1].set_title('Tire Slip Angles')
    axes[1, 1].set_xlabel('Time [s]')
    axes[1, 1].set_ylabel('Slip Angle [deg]')
    axes[1, 1].legend()
    axes[1, 1].grid(True)
    
    # Phase plot: lateral velocity vs yaw rate
    axes[1, 2].plot(v_y, psi_dot * 180 / np.pi)
    axes[1, 2].set_title('Phase Plot: v_y vs ψ̇')
    axes[1, 2].set_xlabel('v_y [m/s]')
    axes[1, 2].set_ylabel('ψ̇ [deg/s]')
    axes[1, 2].grid(True)
    
    # Row 3: Analysis
    # Lateral acceleration
    a_y = np.gradient(v_y, t) + 20.0 * psi_dot  # Assuming constant v_x for simplicity
    axes[2, 0].plot(t, a_y)
    axes[2, 0].set_title('Lateral Acceleration')
    axes[2, 0].set_xlabel('Time [s]')
    axes[2, 0].set_ylabel('a_y [m/s²]')
    axes[2, 0].grid(True)
    
    # Understeer gradient analysis
    beta = np.arctan2(v_y, 20.0) * 180 / np.pi  # Sideslip angle
    axes[2, 1].plot(t, beta)
    axes[2, 1].set_title('Vehicle Sideslip Angle β')
    axes[2, 1].set_xlabel('Time [s]')
    axes[2, 1].set_ylabel('β [deg]')
    axes[2, 1].grid(True)
    
    # Force distribution
    total_force = np.abs(F_yf) + np.abs(F_yr)
    front_ratio = np.abs(F_yf) / (total_force + 1e-6)
    axes[2, 2].plot(t, front_ratio)
    axes[2, 2].set_title('Front Force Ratio')
    axes[2, 2].set_xlabel('Time [s]')
    axes[2, 2].set_ylabel('F_yf / (F_yf + F_yr)')
    axes[2, 2].grid(True)
    
    plt.tight_layout()
    return fig

def create_vehicle_animation(t, states, scenario_name):
    """Create animated visualization of vehicle motion"""
    
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_xlim(-50, 200)
    ax.set_ylim(-100, 100)
    ax.set_aspect('equal')
    ax.grid(True)
    ax.set_title(f'Vehicle Motion Animation: {scenario_name}')
    
    # Vehicle representation
    vehicle_length = 4.0
    vehicle_width = 2.0
    
    # Initialize vehicle patch
    vehicle = patches.Rectangle((0, 0), vehicle_length, vehicle_width, 
                               angle=0, facecolor='blue', alpha=0.7)
    ax.add_patch(vehicle)
    
    # Trajectory line
    trajectory_line, = ax.plot([], [], 'r-', alpha=0.5, linewidth=2)
    
    # Velocity vectors
    velocity_arrow = patches.FancyArrowPatch((0, 0), (0, 0), 
                                           arrowstyle='->', mutation_scale=20, 
                                           color='green', linewidth=2)
    ax.add_patch(velocity_arrow)
    
    def animate(frame):
        if frame >= len(t):
            return vehicle, trajectory_line, velocity_arrow
        
        # Get current state
        v_y, psi_dot, psi, x, y = states[frame]
        
        # Update vehicle position and orientation
        vehicle.set_xy((x - vehicle_length/2, y - vehicle_width/2))
        vehicle.angle = psi * 180 / np.pi
        
        # Update trajectory
        trajectory_line.set_data(states[:frame+1, 3], states[:frame+1, 4])
        
        # Update velocity vector
        v_scale = 5.0
        v_x_vis = 20.0  # Assume constant for visualization
        velocity_arrow.set_positions((x, y), 
                                   (x + v_x_vis * v_scale * np.cos(psi) - v_y * v_scale * np.sin(psi),
                                    y + v_x_vis * v_scale * np.sin(psi) + v_y * v_scale * np.cos(psi)))
        
        return vehicle, trajectory_line, velocity_arrow
    
    anim = FuncAnimation(fig, animate, frames=len(t), interval=50, blit=True)
    return fig, anim

def main():
    """Main simulation function"""
    print("=== Vehicle Lateral Dynamics Simulation ===")
    print("Demonstrating the center of mass reference lateral dynamics equations:")
    print("1. m(v̇_y + v_x ψ̇) = F_yf cos(δ) + F_yr")
    print("2. I_z ψ̈ = l_f F_yf cos(δ) - l_r F_yr")
    print()
    
    # Create vehicle model
    vehicle = VehicleLateralDynamics()
    
    # Get simulation scenarios
    scenarios = create_simulation_scenarios()
    
    # Run simulations
    for i, (v_x_profile, delta_profile, scenario_name) in enumerate(scenarios):
        print(f"Running Scenario {i+1}: {scenario_name}")
        
        # Simulate
        t_span = [0, 10]
        t, states, forces = vehicle.simulate(t_span, v_x_profile, delta_profile)
        
        # Plot results
        fig = plot_simulation_results(t, states, forces, scenario_name)
        plt.show()
        
        # Create animation
        fig_anim, anim = create_vehicle_animation(t, states, scenario_name)
        plt.show()
        
        print(f"Scenario {i+1} completed.\n")
    
    print("=== Key Insights from the Simulation ===")
    print("1. Lateral force balance equation shows how lateral forces create lateral acceleration")
    print("2. Yaw moment balance equation shows how tire forces create yaw motion")
    print("3. The coupling between v_y and ψ̇ creates complex vehicle dynamics")
    print("4. Tire slip angles determine the lateral forces generated")
    print("5. Vehicle stability depends on the balance between front and rear forces")

if __name__ == "__main__":
    main() 