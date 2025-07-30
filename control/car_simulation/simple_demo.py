"""
Simple Demo: Vehicle Lateral Dynamics Equations
简单演示：车辆横向动力学方程

This script demonstrates the key concepts behind the lateral dynamics equations:
1. m(v̇_y + v_x ψ̇) = F_yf cos(δ) + F_yr  (横向力平衡)
2. I_z ψ̈ = l_f F_yf cos(δ) - l_r F_yr    (横摆力矩平衡)
"""

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
    dt = 0.01
    t_end = 5.0
    time = np.arange(0, t_end, dt)
    
    # Vehicle parameters
    m = 1500.0
    I_z = 2500.0
    l_f = 1.2
    l_r = 1.6
    C_f = 80000.0  # Front tire cornering stiffness
    C_r = 80000.0  # Rear tire cornering stiffness
    
    # Initial conditions
    v_x = 20.0
    v_y = 0.0
    psi_dot = 0.0
    
    # Steering input (step input)
    delta = np.zeros_like(time)
    delta[time >= 1.0] = 0.05  # 0.05 rad step at t=1s
    
    # Arrays to store results
    v_y_history = np.zeros_like(time)
    psi_dot_history = np.zeros_like(time)
    F_yf_history = np.zeros_like(time)
    F_yr_history = np.zeros_like(time)
    
    # Simple integration (Euler method)
    for i, t in enumerate(time):
        # Calculate slip angles (simplified)
        if abs(v_x) > 0.1:
            alpha_f = delta[i] - np.arctan2(v_y + l_f * psi_dot, v_x)
            alpha_r = -np.arctan2(v_y - l_r * psi_dot, v_x)
        else:
            alpha_f = alpha_r = 0.0
        
        # Calculate tire forces
        F_yf = C_f * alpha_f
        F_yr = C_r * alpha_r
        
        # Store values
        v_y_history[i] = v_y
        psi_dot_history[i] = psi_dot
        F_yf_history[i] = F_yf
        F_yr_history[i] = F_yr
        
        # Calculate derivatives using the lateral dynamics equations
        v_y_dot = (F_yf * np.cos(delta[i]) + F_yr) / m - v_x * psi_dot
        psi_dot_dot = (l_f * F_yf * np.cos(delta[i]) - l_r * F_yr) / I_z
        
        # Update states
        if i < len(time) - 1:
            v_y += v_y_dot * dt
            psi_dot += psi_dot_dot * dt
    
    # Plot results
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle('Vehicle Lateral Dynamics Response', fontsize=16)
    
    # Steering input
    axes[0, 0].plot(time, delta * 180 / np.pi, 'k-', linewidth=2)
    axes[0, 0].set_title('Steering Angle Input')
    axes[0, 0].set_xlabel('Time [s]')
    axes[0, 0].set_ylabel('Steering Angle [deg]')
    axes[0, 0].grid(True)
    
    # Lateral velocity response
    axes[0, 1].plot(time, v_y_history, 'b-', linewidth=2)
    axes[0, 1].set_title('Lateral Velocity Response')
    axes[0, 1].set_xlabel('Time [s]')
    axes[0, 1].set_ylabel('v_y [m/s]')
    axes[0, 1].grid(True)
    
    # Yaw rate response
    axes[1, 0].plot(time, psi_dot_history * 180 / np.pi, 'r-', linewidth=2)
    axes[1, 0].set_title('Yaw Rate Response')
    axes[1, 0].set_xlabel('Time [s]')
    axes[1, 0].set_ylabel('ψ̇ [deg/s]')
    axes[1, 0].grid(True)
    
    # Tire forces
    axes[1, 1].plot(time, F_yf_history / 1000, 'b-', linewidth=2, label='Front')
    axes[1, 1].plot(time, F_yr_history / 1000, 'g-', linewidth=2, label='Rear')
    axes[1, 1].set_title('Tire Lateral Forces')
    axes[1, 1].set_xlabel('Time [s]')
    axes[1, 1].set_ylabel('Force [kN]')
    axes[1, 1].legend()
    axes[1, 1].grid(True)
    
    plt.tight_layout()
    return fig

def main():
    """Main demonstration function"""
    
    # Basic physics demonstration
    v_x, delta, psi_dot, F_yf, F_yr = demonstrate_lateral_dynamics()
    
    # Create visual diagrams
    fig1 = create_force_diagram()
    plt.show()
    
    # Dynamic response demonstration
    fig2 = demonstrate_dynamic_response()
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