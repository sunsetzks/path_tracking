"""
Simple Demo: Vehicle Lateral Dynamics Equations
简单演示：车辆横向动力学方程

This script demonstrates the key concepts behind the lateral dynamics equations:
1. m(v̇_y + v_x ψ̇) = F_yf cos(δ) + F_yr  (横向力平衡)
2. I_z ψ̈ = l_f F_yf cos(δ) - l_r F_yr    (横摆力矩平衡)
"""

import math
import numpy as np
import matplotlib.pyplot as plt

def demonstrate_lateral_dynamics():
    """Demonstrate the physical meaning of lateral dynamics equations"""
    
    print("=== Lateral Dynamics Equations Physical Meaning Demo ===\n")
    
    # Vehicle parameters
    m = 1500.0      # 车辆质量 [kg]
    I_z = 2500.0    # 横摆转动惯量 [kg*m^2]
    l_f = 1.2       # 质心到前轴距离 [m]
    l_r = 1.6       # 质心到后轴距离 [m]
    
    print(f"Vehicle Parameters:")
    print(f"  Mass m = {m} kg")
    print(f"  Yaw moment of inertia I_z = {I_z} kg*m²")
    print(f"  Front axle distance l_f = {l_f} m")
    print(f"  Rear axle distance l_r = {l_r} m\n")
    
    # Scenario: Constant speed turn
    v_x = 20.0      # 纵向速度 [m/s]
    delta = 0.1     # 转向角 [rad] ≈ 5.7°
    
    print(f"Scenario: Constant Speed Turn")
    print(f"  Longitudinal speed v_x = {v_x} m/s")
    print(f"  Steering angle δ = {delta:.3f} rad = {np.degrees(delta):.1f}°\n")
    
    # Simulate steady-state conditions
    print("=== Steady-State Analysis ===")
    
    # For steady-state circular motion, we can calculate required forces
    # Assume steady-state: v̇_y = 0, ψ̈ = 0
    # This gives us: v_x * ψ̇ = (F_yf + F_yr) / m
    
    # For a simple analysis, assume equal front and rear forces
    R = 50.0  # Turning radius [m]
    psi_dot = v_x / R  # Yaw rate
    
    # Required lateral acceleration for circular motion
    a_y_required = v_x * psi_dot
    F_total_required = m * a_y_required
    
    print(f"Steady-State Circular Motion Analysis:")
    print(f"  Turning radius R = {R} m")
    print(f"  Yaw rate ψ̇ = {psi_dot:.3f} rad/s = {np.degrees(psi_dot):.1f}°/s")
    print(f"  Required lateral acceleration a_y = {a_y_required:.2f} m/s²")
    print(f"  Required total lateral force F_total = {F_total_required:.0f} N\n")
    
    # Demonstrate force distribution
    print("=== Force Distribution Analysis ===")
    
    # Simple force distribution (assuming no slip angles for now)
    F_yf = F_total_required * 0.6  # Front wheels take 60%
    F_yr = F_total_required * 0.4  # Rear wheels take 40%
    
    print(f"Assumed Force Distribution:")
    print(f"  Front lateral force F_yf = {F_yf:.0f} N")
    print(f"  Rear lateral force F_yr = {F_yr:.0f} N")
    
    # Check equation 1: m(v̇_y + v_x ψ̇) = F_yf cos(δ) + F_yr
    left_side = m * (0 + v_x * psi_dot)  # v̇_y = 0 for steady state
    right_side = F_yf * np.cos(delta) + F_yr
    
    print(f"\nEquation 1 Verification: m(v̇_y + v_x ψ̇) = F_yf cos(δ) + F_yr")
    print(f"  Left side: {left_side:.0f} N")
    print(f"  Right side: {right_side:.0f} N")
    print(f"  Error: {abs(left_side - right_side):.0f} N")
    
    # Check equation 2: I_z ψ̈ = l_f F_yf cos(δ) - l_r F_yr
    left_side_2 = I_z * 0  # ψ̈ = 0 for steady state
    right_side_2 = l_f * F_yf * np.cos(delta) - l_r * F_yr
    
    print(f"\nEquation 2 Verification: I_z ψ̈ = l_f F_yf cos(δ) - l_r F_yr")
    print(f"  Left side: {left_side_2:.0f} N*m")
    print(f"  Right side: {right_side_2:.0f} N*m")
    print(f"  Error: {abs(left_side_2 - right_side_2):.0f} N*m")
    
    return v_x, delta, psi_dot, F_yf, F_yr

def create_force_diagram():
    """Create a visual diagram showing forces and moments"""
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(15, 6))
    
    # Vehicle parameters
    l_f = 1.2
    l_r = 1.6
    
    # Diagram 1: Force diagram
    ax1.set_xlim(-3, 3)
    ax1.set_ylim(-1, 4)
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    ax1.set_title('Vehicle Force Diagram (Top View)', fontsize=14)
    
    # Vehicle outline
    vehicle_x = [-l_r, l_f, l_f, -l_r, -l_r]
    vehicle_y = [-0.8, -0.8, 0.8, 0.8, -0.8]
    ax1.plot(vehicle_x, vehicle_y, 'k-', linewidth=2, label='Vehicle Body')
    
    # Center of mass
    ax1.plot(0, 0, 'ro', markersize=8, label='Center of Mass')
    ax1.text(0.1, 0.1, 'CG', fontsize=12)
    
    # Axles
    ax1.plot([l_f, l_f], [-1, 1], 'b-', linewidth=3, label='Front Axle')
    ax1.plot([-l_r, -l_r], [-1, 1], 'g-', linewidth=3, label='Rear Axle')
    
    # Forces
    F_scale = 0.002
    F_yf = 3000
    F_yr = 2000
    
    # Front tire forces
    ax1.arrow(l_f, -0.5, 0, F_yf * F_scale, head_width=0.1, head_length=0.1, 
              fc='blue', ec='blue', linewidth=2)
    ax1.arrow(l_f, 0.5, 0, F_yf * F_scale, head_width=0.1, head_length=0.1, 
              fc='blue', ec='blue', linewidth=2)
    ax1.text(l_f + 0.2, 0, f'F_yf = {F_yf}N', fontsize=10, color='blue')
    
    # Rear tire forces
    ax1.arrow(-l_r, -0.5, 0, F_yr * F_scale, head_width=0.1, head_length=0.1, 
              fc='green', ec='green', linewidth=2)
    ax1.arrow(-l_r, 0.5, 0, F_yr * F_scale, head_width=0.1, head_length=0.1, 
              fc='green', ec='green', linewidth=2)
    ax1.text(-l_r - 0.8, 0, f'F_yr = {F_yr}N', fontsize=10, color='green')
    
    # Velocity vector
    ax1.arrow(0, 0, 2, 0, head_width=0.1, head_length=0.1, 
              fc='red', ec='red', linewidth=2)
    ax1.text(1, -0.3, 'v_x', fontsize=12, color='red')
    
    # Dimensions
    ax1.annotate('', xy=(0, -1.5), xytext=(l_f, -1.5),
                arrowprops=dict(arrowstyle='<->', color='black'))
    ax1.text(l_f/2, -1.7, f'l_f = {l_f}m', ha='center', fontsize=10)
    
    ax1.annotate('', xy=(-l_r, -1.5), xytext=(0, -1.5),
                arrowprops=dict(arrowstyle='<->', color='black'))
    ax1.text(-l_r/2, -1.7, f'l_r = {l_r}m', ha='center', fontsize=10)
    
    ax1.legend(loc='upper right')
    
    # Diagram 2: Moment diagram
    ax2.set_xlim(-3, 3)
    ax2.set_ylim(-2, 2)
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)
    ax2.set_title('Yaw Moment Diagram', fontsize=14)
    
    # Vehicle outline (simplified)
    ax2.plot(vehicle_x, vehicle_y, 'k-', linewidth=2)
    ax2.plot(0, 0, 'ro', markersize=8)
    
    # Moment arms
    ax2.plot([0, l_f], [0, 0], 'b--', linewidth=2, alpha=0.7)
    ax2.plot([0, -l_r], [0, 0], 'g--', linewidth=2, alpha=0.7)
    
    # Moment arrows (curved)
    from matplotlib.patches import FancyArrowPatch
    from matplotlib.patches import ConnectionPatch
    
    # Front moment (clockwise)
    moment_f = FancyArrowPatch((l_f-0.3, 0.3), (l_f+0.3, 0.3),
                               arrowstyle='->', mutation_scale=20, 
                               color='blue', linewidth=2)
    ax2.add_patch(moment_f)
    ax2.text(l_f, 0.6, f'M_f = l_f × F_yf\n= {l_f * F_yf:.0f} N·m', 
             ha='center', fontsize=10, color='blue')
    
    # Rear moment (counter-clockwise)
    moment_r = FancyArrowPatch((-l_r+0.3, -0.3), (-l_r-0.3, -0.3),
                               arrowstyle='->', mutation_scale=20, 
                               color='green', linewidth=2)
    ax2.add_patch(moment_r)
    ax2.text(-l_r, -0.8, f'M_r = l_r × F_yr\n= {l_r * F_yr:.0f} N·m', 
             ha='center', fontsize=10, color='green')
    
    # Net moment
    net_moment = l_f * F_yf - l_r * F_yr
    ax2.text(0, 1.5, f'Net Moment = M_f - M_r = {net_moment:.0f} N·m',
             ha='center', fontsize=12, bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow"))
    
    plt.tight_layout()
    return fig

def demonstrate_dynamic_response():
    """Demonstrate how the vehicle responds to steering input"""
    
    print("\n=== Dynamic Response Demonstration ===")
    
    # Time simulation
    dt = 0.00001
    t_end = 10.0
    time = np.arange(0, t_end, dt)
    
    # Vehicle parameters
    m = 1500.0
    I_z = 2500.0
    l_f = 1.2
    l_r = 1.6
    C_f = 80000.0  # Front tire cornering stiffness
    C_r = 80000.0  # Rear tire cornering stiffness
    
    # Initial conditions
    v_x = 0.1
    v_y = 0.0
    psi_dot = 0.0
    
    # Steering input (step input)
    delta = np.zeros_like(time)
    # Fast change in steering angle instead of step input
    transition_time = 3  # 100ms transition time
    transition_start = 1.0
    transition_end = transition_start + transition_time
    max_steering_angle = 0.05  # Maximum steering angle in radians
    
    # Create smooth transition using sigmoid-like function
    for i, t in enumerate(time):
        if t >= transition_end:
            delta[i] = max_steering_angle
        elif t >= transition_start:
            # Smooth transition from 0 to max_steering_angle
            progress = (t - transition_start) / transition_time
            # Use sigmoid-like function for smooth transition
            smooth_progress = 0.5 * (1 + np.tanh(6 * (progress - 0.5)))
            delta[i] = max_steering_angle * smooth_progress
        else:
            delta[i] = 0.0
    
    # Arrays to store results
    v_y_history = np.zeros_like(time)
    psi_dot_history = np.zeros_like(time)
    F_yf_history = np.zeros_like(time)
    F_yr_history = np.zeros_like(time)
    alpha_f_history = np.zeros_like(time)
    alpha_r_history = np.zeros_like(time)
    slip_angle_history = np.zeros_like(time)
    
    # Simple integration (Euler method)
    for i, t in enumerate(time):
        # Calculate slip angles
        if abs(v_x) > 0.01:
            alpha_f = delta[i] - np.arctan2(v_y + l_f * psi_dot, v_x)
            alpha_r = -np.arctan2(v_y - l_r * psi_dot, v_x)
        else:
            alpha_f = alpha_r = 0.0
        
        # Calculate slip angle (simplified)
        slip_angle = np.degrees(np.arctan2(abs(v_y), abs(v_x) + 0.001))  # Convert to degrees
        
        # Calculate tire forces
        F_yf = C_f * alpha_f
        F_yr = C_r * alpha_r
        
        # Store values
        v_y_history[i] = v_y
        psi_dot_history[i] = psi_dot
        F_yf_history[i] = F_yf
        F_yr_history[i] = F_yr
        alpha_f_history[i] = alpha_f
        alpha_r_history[i] = alpha_r
        slip_angle_history[i] = slip_angle
        
        # Calculate derivatives using the lateral dynamics equations
        v_y_dot = (F_yf * np.cos(delta[i]) + F_yr) / m - v_x * psi_dot
        psi_dot_dot = (l_f * F_yf * np.cos(delta[i]) - l_r * F_yr) / I_z
        
        # Update states
        if i < len(time) - 1:
            v_y += v_y_dot * dt
            psi_dot += psi_dot_dot * dt
    
    # Print key results
    print(f"\nKey Simulation Results:")
    print(f"  Steering input: {delta[time >= 1.0][0]:.3f} rad = {np.degrees(delta[time >= 1.0][0]):.1f}°")
    print(f"  Maximum lateral velocity: {np.max(np.abs(v_y_history)):.2f} m/s")
    print(f"  Maximum yaw rate: {np.max(np.abs(psi_dot_history)):.3f} rad/s = {np.degrees(np.max(np.abs(psi_dot_history))):.1f}°/s")
    print(f"  Maximum front slip angle: {np.degrees(np.max(np.abs(alpha_f_history))):.2f}°")
    print(f"  Maximum rear slip angle: {np.degrees(np.max(np.abs(alpha_r_history))):.2f}°")
    print(f"  Maximum slip angle: {np.max(slip_angle_history):.3f}")
    print(f"  Maximum front tire force: {np.max(np.abs(F_yf_history))/1000:.1f} kN")
    print(f"  Maximum rear tire force: {np.max(np.abs(F_yr_history))/1000:.1f} kN")
    
    # Calculate steady-state values
    steady_state_start = int(0.8 * len(time))  # Last 20% of simulation
    v_y_ss = np.mean(v_y_history[steady_state_start:])
    psi_dot_ss = np.mean(psi_dot_history[steady_state_start:])
    alpha_f_ss = np.mean(alpha_f_history[steady_state_start:])
    alpha_r_ss = np.mean(alpha_r_history[steady_state_start:])
    
    print(f"\nSteady-State Values (t > {time[steady_state_start]:.1f}s):")
    print(f"  Lateral velocity: {v_y_ss:.2f} m/s")
    print(f"  Yaw rate: {psi_dot_ss:.3f} rad/s = {np.degrees(psi_dot_ss):.1f}°/s")
    print(f"  Front slip angle: {np.degrees(alpha_f_ss):.2f}°")
    print(f"  Rear slip angle: {np.degrees(alpha_r_ss):.2f}°")
    
    # Calculate vehicle response characteristics
    settling_time_idx = np.where(np.abs(v_y_history - v_y_ss) < 0.02 * np.max(np.abs(v_y_history)))[0]
    if len(settling_time_idx) > 0:
        settling_time = time[settling_time_idx[0]]
        print(f"  Lateral velocity settling time: {settling_time:.2f} s")
    
    # Plot results
    fig, axes = plt.subplots(3, 2, figsize=(15, 12))
    fig.suptitle('Vehicle Lateral Dynamics Response - Enhanced Analysis', fontsize=16)
    
    # Steering input
    axes[0, 0].plot(time, delta * 180 / np.pi, 'k-', linewidth=2)
    axes[0, 0].axvline(x=1.0, color='r', linestyle='--', alpha=0.7, label='Step input')
    axes[0, 0].set_title('Steering Angle Input')
    axes[0, 0].set_xlabel('Time [s]')
    axes[0, 0].set_ylabel('Steering Angle [deg]')
    axes[0, 0].grid(True)
    axes[0, 0].legend()
    
    # Lateral velocity response
    axes[0, 1].plot(time, v_y_history, 'b-', linewidth=2, label='v_y')
    axes[0, 1].axhline(y=v_y_ss, color='r', linestyle='--', alpha=0.7, label='Steady-state')
    axes[0, 1].set_title('Lateral Velocity Response')
    axes[0, 1].set_xlabel('Time [s]')
    axes[0, 1].set_ylabel('v_y [m/s]')
    axes[0, 1].grid(True)
    axes[0, 1].legend()
    
    # Yaw rate response
    axes[1, 0].plot(time, psi_dot_history * 180 / np.pi, 'r-', linewidth=2, label='ψ̇')
    axes[1, 0].axhline(y=np.degrees(psi_dot_ss), color='r', linestyle='--', alpha=0.7, label='Steady-state')
    axes[1, 0].set_title('Yaw Rate Response')
    axes[1, 0].set_xlabel('Time [s]')
    axes[1, 0].set_ylabel('Yaw Rate [deg/s]')
    axes[1, 0].grid(True)
    axes[1, 0].legend()
    
    # Slip angles
    axes[1, 1].plot(time, np.degrees(alpha_f_history), 'b-', linewidth=2, label='Front (α_f)')
    axes[1, 1].plot(time, np.degrees(alpha_r_history), 'g-', linewidth=2, label='Rear (α_r)')
    axes[1, 1].axhline(y=np.degrees(alpha_f_ss), color='b', linestyle='--', alpha=0.7)
    axes[1, 1].axhline(y=np.degrees(alpha_r_ss), color='g', linestyle='--', alpha=0.7)
    axes[1, 1].set_title('Tire Slip Angles')
    axes[1, 1].set_xlabel('Time [s]')
    axes[1, 1].set_ylabel('Slip Angle [deg]')
    axes[1, 1].grid(True)
    axes[1, 1].legend()
    
    # Tire forces
    axes[2, 0].plot(time, F_yf_history / 1000, 'b-', linewidth=2, label='Front')
    axes[2, 0].plot(time, F_yr_history / 1000, 'g-', linewidth=2, label='Rear')
    axes[2, 0].set_title('Tire Lateral Forces')
    axes[2, 0].set_xlabel('Time [s]')
    axes[2, 0].set_ylabel('Force [kN]')
    axes[2, 0].grid(True)
    axes[2, 0].legend()
    
    # Slip ratio and vehicle dynamics
    ax_slip = axes[2, 1]
    ax_slip.plot(time, slip_angle_history, 'm-', linewidth=2, label='Slip Angle')
    ax_slip.set_xlabel('Time [s]')
    ax_slip.set_ylabel('Slip Ratio', color='m')
    ax_slip.tick_params(axis='y', labelcolor='m')
    ax_slip.grid(True, alpha=0.3)
    
    # Add vehicle yaw angle on secondary y-axis
    yaw_angle = np.cumsum(psi_dot_history) * dt
    ax_yaw = ax_slip.twinx()
    ax_yaw.plot(time, np.degrees(yaw_angle), 'c-', linewidth=2, label='Yaw Angle')
    ax_yaw.set_ylabel('Yaw Angle [deg]', color='c')
    ax_yaw.tick_params(axis='y', labelcolor='c')
    
    axes[2, 1].set_title('Slip Angle [°] and Vehicle Yaw')
    axes[2, 1].legend(loc='upper left')
    ax_yaw.legend(loc='upper right')
    
    plt.tight_layout()
    return fig

def main():
    """Main demonstration function"""
    
    # Basic physics demonstration
    v_x, delta, psi_dot, F_yf, F_yr = demonstrate_lateral_dynamics()
    
    # Create visual diagrams
    # fig1 = create_force_diagram()
    # plt.show()
    
    # Dynamic response demonstration
    fig2 = demonstrate_dynamic_response()
    
    # Maximize the figure window using a simple approach
    try:
        # Set a large figure size to simulate maximization
        fig2.set_size_inches(16, 10)  # Large size that should fill most screens
        
        # Try to maximize the window using platform-specific methods
        try:
            manager = plt.get_current_fig_manager()
            if manager:
                # Try different maximization methods
                try:
                    # Windows
                    manager.window.state('zoomed')  # type: ignore
                except:
                    try:
                        # Qt
                        manager.window.showMaximized()  # type: ignore
                    except:
                        try:
                            # Linux
                            manager.window.attributes('-zoomed', True)  # type: ignore
                        except:
                            pass
        except:
            # If any error occurs during maximization, just continue
            pass
            
    except:
        # If maximization fails, continue without error
        pass
    
    plt.show()
    
    print("\n=== Summary: Role of Lateral Dynamics Equations ===")
    print("1. Equation 1 (Lateral Force Balance): Describes how lateral forces create lateral acceleration")
    print("   - Left side: m(v̇_y + v_x ψ̇) is the lateral inertial force")
    print("   - Right side: F_yf cos(δ) + F_yr is the lateral force provided by tires")
    print("   - Physical meaning: Tire forces must balance inertial forces to maintain motion")
    print()
    print("2. Equation 2 (Yaw Moment Balance): Describes how tire forces create yaw motion")
    print("   - Left side: I_z ψ̈ is the inertial moment from yaw angular acceleration")
    print("   - Right side: l_f F_yf cos(δ) - l_r F_yr is the moment difference between front and rear")
    print("   - Physical meaning: Front-rear moment difference determines vehicle rotation tendency")
    print()
    print("3. Practical Applications:")
    print("   - Vehicle Stability Control (ESP/VSC)")
    print("   - Steering system design")
    print("   - Tire performance analysis")
    print("   - Autonomous vehicle path tracking")
    print("   - Vehicle dynamics simulation")

if __name__ == "__main__":
    main() 