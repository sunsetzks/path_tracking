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
    
    print("=== 横向动力学方程物理意义演示 ===\n")
    
    # Vehicle parameters
    m = 1500.0      # 车辆质量 [kg]
    I_z = 2500.0    # 横摆转动惯量 [kg*m^2]
    l_f = 1.2       # 质心到前轴距离 [m]
    l_r = 1.6       # 质心到后轴距离 [m]
    
    print(f"车辆参数:")
    print(f"  质量 m = {m} kg")
    print(f"  横摆转动惯量 I_z = {I_z} kg*m²")
    print(f"  前轴距离 l_f = {l_f} m")
    print(f"  后轴距离 l_r = {l_r} m\n")
    
    # Scenario: Constant speed turn
    v_x = 20.0      # 纵向速度 [m/s]
    delta = 0.1     # 转向角 [rad] ≈ 5.7°
    
    print(f"场景：恒速转弯")
    print(f"  纵向速度 v_x = {v_x} m/s")
    print(f"  转向角 δ = {delta:.3f} rad = {np.degrees(delta):.1f}°\n")
    
    # Simulate steady-state conditions
    print("=== 稳态分析 ===")
    
    # For steady-state circular motion, we can calculate required forces
    # Assume steady-state: v̇_y = 0, ψ̈ = 0
    # This gives us: v_x * ψ̇ = (F_yf + F_yr) / m
    
    # For a simple analysis, assume equal front and rear forces
    R = 50.0  # 转弯半径 [m]
    psi_dot = v_x / R  # 横摆角速度
    
    # Required lateral acceleration for circular motion
    a_y_required = v_x * psi_dot
    F_total_required = m * a_y_required
    
    print(f"稳态圆周运动分析:")
    print(f"  转弯半径 R = {R} m")
    print(f"  横摆角速度 ψ̇ = {psi_dot:.3f} rad/s = {np.degrees(psi_dot):.1f}°/s")
    print(f"  所需横向加速度 a_y = {a_y_required:.2f} m/s²")
    print(f"  所需总横向力 F_total = {F_total_required:.0f} N\n")
    
    # Demonstrate force distribution
    print("=== 力分布分析 ===")
    
    # Simple force distribution (assuming no slip angles for now)
    F_yf = F_total_required * 0.6  # 前轮承担60%
    F_yr = F_total_required * 0.4  # 后轮承担40%
    
    print(f"假设力分布:")
    print(f"  前轮侧向力 F_yf = {F_yf:.0f} N")
    print(f"  后轮侧向力 F_yr = {F_yr:.0f} N")
    
    # Check equation 1: m(v̇_y + v_x ψ̇) = F_yf cos(δ) + F_yr
    left_side = m * (0 + v_x * psi_dot)  # v̇_y = 0 for steady state
    right_side = F_yf * np.cos(delta) + F_yr
    
    print(f"\n方程1验证: m(v̇_y + v_x ψ̇) = F_yf cos(δ) + F_yr")
    print(f"  左边: {left_side:.0f} N")
    print(f"  右边: {right_side:.0f} N")
    print(f"  误差: {abs(left_side - right_side):.0f} N")
    
    # Check equation 2: I_z ψ̈ = l_f F_yf cos(δ) - l_r F_yr
    left_side_2 = I_z * 0  # ψ̈ = 0 for steady state
    right_side_2 = l_f * F_yf * np.cos(delta) - l_r * F_yr
    
    print(f"\n方程2验证: I_z ψ̈ = l_f F_yf cos(δ) - l_r F_yr")
    print(f"  左边: {left_side_2:.0f} N*m")
    print(f"  右边: {right_side_2:.0f} N*m")
    print(f"  误差: {abs(left_side_2 - right_side_2):.0f} N*m")
    
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
    ax1.set_title('车辆受力图 (俯视图)', fontsize=14)
    
    # Vehicle outline
    vehicle_x = [-l_r, l_f, l_f, -l_r, -l_r]
    vehicle_y = [-0.8, -0.8, 0.8, 0.8, -0.8]
    ax1.plot(vehicle_x, vehicle_y, 'k-', linewidth=2, label='车身')
    
    # Center of mass
    ax1.plot(0, 0, 'ro', markersize=8, label='质心')
    ax1.text(0.1, 0.1, 'CG', fontsize=12)
    
    # Axles
    ax1.plot([l_f, l_f], [-1, 1], 'b-', linewidth=3, label='前轴')
    ax1.plot([-l_r, -l_r], [-1, 1], 'g-', linewidth=3, label='后轴')
    
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
    ax2.set_title('横摆力矩图', fontsize=14)
    
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
    ax2.text(0, 1.5, f'净力矩 = M_f - M_r = {net_moment:.0f} N·m', 
             ha='center', fontsize=12, bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow"))
    
    plt.tight_layout()
    return fig

def demonstrate_dynamic_response():
    """Demonstrate how the vehicle responds to steering input"""
    
    print("\n=== 动态响应演示 ===")
    
    # Time simulation
    dt = 0.01
    t_end = 5.0
    time = np.arange(0, t_end, dt)
    
    # Vehicle parameters
    m = 1500.0
    I_z = 2500.0
    l_f = 1.2
    l_r = 1.6
    C_f = 80000.0  # 前轮侧偏刚度
    C_r = 80000.0  # 后轮侧偏刚度
    
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
    fig.suptitle('车辆横向动力学响应', fontsize=16)
    
    # Steering input
    axes[0, 0].plot(time, delta * 180 / np.pi, 'k-', linewidth=2)
    axes[0, 0].set_title('转向角输入')
    axes[0, 0].set_xlabel('时间 [s]')
    axes[0, 0].set_ylabel('转向角 [deg]')
    axes[0, 0].grid(True)
    
    # Lateral velocity response
    axes[0, 1].plot(time, v_y_history, 'b-', linewidth=2)
    axes[0, 1].set_title('横向速度响应')
    axes[0, 1].set_xlabel('时间 [s]')
    axes[0, 1].set_ylabel('v_y [m/s]')
    axes[0, 1].grid(True)
    
    # Yaw rate response
    axes[1, 0].plot(time, psi_dot_history * 180 / np.pi, 'r-', linewidth=2)
    axes[1, 0].set_title('横摆角速度响应')
    axes[1, 0].set_xlabel('时间 [s]')
    axes[1, 0].set_ylabel('ψ̇ [deg/s]')
    axes[1, 0].grid(True)
    
    # Tire forces
    axes[1, 1].plot(time, F_yf_history / 1000, 'b-', linewidth=2, label='前轮')
    axes[1, 1].plot(time, F_yr_history / 1000, 'g-', linewidth=2, label='后轮')
    axes[1, 1].set_title('轮胎侧向力')
    axes[1, 1].set_xlabel('时间 [s]')
    axes[1, 1].set_ylabel('力 [kN]')
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
    
    print("\n=== 总结：横向动力学方程的作用 ===")
    print("1. 方程1 (横向力平衡): 描述横向力如何产生横向加速度")
    print("   - 左边: m(v̇_y + v_x ψ̇) 是横向惯性力")
    print("   - 右边: F_yf cos(δ) + F_yr 是轮胎提供的横向力")
    print("   - 物理意义: 轮胎力必须平衡惯性力才能维持运动")
    print()
    print("2. 方程2 (横摆力矩平衡): 描述轮胎力如何产生横摆运动")
    print("   - 左边: I_z ψ̈ 是横摆角加速度产生的惯性力矩")
    print("   - 右边: l_f F_yf cos(δ) - l_r F_yr 是前后轮力矩差")
    print("   - 物理意义: 前后轮力矩差决定车辆的旋转趋势")
    print()
    print("3. 实际应用:")
    print("   - 车辆稳定性控制 (ESP/VSC)")
    print("   - 转向系统设计")
    print("   - 轮胎性能分析")
    print("   - 自动驾驶路径跟踪")
    print("   - 车辆动力学仿真")

if __name__ == "__main__":
    main() 