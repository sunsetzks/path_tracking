"""
Lateral Dynamics Equations Explanation
横向动力学方程物理意义解释

This script provides a clear explanation of what the lateral dynamics equations mean
and why they are important in vehicle dynamics simulation.
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrowPatch, Circle
import matplotlib.patches as mpatches

def explain_equations():
    """Explain the physical meaning of the lateral dynamics equations"""
    
    print("=" * 60)
    print("横向动力学方程详解")
    print("=" * 60)
    
    print("\n### 方程1: 横向力平衡方程")
    print("m(v̇_y + v_x ψ̇) = F_yf cos(δ) + F_yr")
    print()
    print("物理意义:")
    print("- 左边: m(v̇_y + v_x ψ̇) 是车辆质心处的横向惯性力")
    print("  - v̇_y: 横向速度变化率 (横向加速度的一部分)")
    print("  - v_x ψ̇: 由于横摆运动产生的向心加速度")
    print("- 右边: F_yf cos(δ) + F_yr 是轮胎提供的总横向力")
    print("  - F_yf: 前轮横向力")
    print("  - F_yr: 后轮横向力")
    print("  - cos(δ): 前轮转向角的余弦值")
    
    print("\n### 方程2: 横摆力矩平衡方程")
    print("I_z ψ̈ = l_f F_yf cos(δ) - l_r F_yr")
    print()
    print("物理意义:")
    print("- 左边: I_z ψ̈ 是绕质心的横摆角加速度产生的惯性力矩")
    print("  - I_z: 绕质心的横摆转动惯量")
    print("  - ψ̈: 横摆角加速度")
    print("- 右边: l_f F_yf cos(δ) - l_r F_yr 是前后轮力矩的净值")
    print("  - l_f F_yf cos(δ): 前轮对质心产生的力矩")
    print("  - l_r F_yr: 后轮对质心产生的力矩")
    
    print("\n### 为什么这些方程很重要？")
    print("1. 预测车辆行为: 给定转向输入，预测车辆的运动轨迹")
    print("2. 控制系统设计: 设计ESP、VSC等稳定性控制系统")
    print("3. 轮胎性能分析: 分析轮胎在不同工况下的性能")
    print("4. 自动驾驶: 路径跟踪和车辆控制的理论基础")
    print("5. 车辆设计: 优化车辆的稳定性和操控性")

def create_detailed_force_diagram():
    """Create a detailed force diagram with annotations"""
    
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(16, 12))
    fig.suptitle('Vehicle Lateral Dynamics - Detailed Analysis', fontsize=16)
    
    # Vehicle parameters
    l_f = 1.2
    l_r = 1.6
    vehicle_width = 1.8
    
    # Diagram 1: Force Analysis
    ax1.set_xlim(-3, 3)
    ax1.set_ylim(-2, 3)
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    ax1.set_title('Force Analysis (Top View)', fontsize=14)
    
    # Vehicle body
    vehicle = Rectangle((-l_r, -vehicle_width/2), l_f+l_r, vehicle_width, 
                       fill=False, linewidth=2, edgecolor='black')
    ax1.add_patch(vehicle)
    
    # Center of mass
    ax1.plot(0, 0, 'ro', markersize=10, label='Center of Mass')
    ax1.text(0.1, 0.1, 'CG', fontsize=12, fontweight='bold')
    
    # Axles
    ax1.plot([l_f, l_f], [-1.2, 1.2], 'b-', linewidth=4, label='Front Axle')
    ax1.plot([-l_r, -l_r], [-1.2, 1.2], 'g-', linewidth=4, label='Rear Axle')
    
    # Forces
    F_yf = 4000  # N
    F_yr = 3000  # N
    F_scale = 0.0008
    
    # Front forces
    ax1.arrow(l_f, -0.6, 0, F_yf * F_scale, head_width=0.1, head_length=0.1, 
              fc='blue', ec='blue', linewidth=3)
    ax1.arrow(l_f, 0.6, 0, F_yf * F_scale, head_width=0.1, head_length=0.1, 
              fc='blue', ec='blue', linewidth=3)
    ax1.text(l_f + 0.3, 0, f'F_yf = {F_yf}N', fontsize=12, color='blue', fontweight='bold')
    
    # Rear forces
    ax1.arrow(-l_r, -0.6, 0, F_yr * F_scale, head_width=0.1, head_length=0.1, 
              fc='green', ec='green', linewidth=3)
    ax1.arrow(-l_r, 0.6, 0, F_yr * F_scale, head_width=0.1, head_length=0.1, 
              fc='green', ec='green', linewidth=3)
    ax1.text(-l_r - 0.8, 0, f'F_yr = {F_yr}N', fontsize=12, color='green', fontweight='bold')
    
    # Velocity vector
    ax1.arrow(0, 0, 2.5, 0, head_width=0.1, head_length=0.1, 
              fc='red', ec='red', linewidth=3)
    ax1.text(1.3, -0.3, 'v_x', fontsize=14, color='red', fontweight='bold')
    
    # Dimensions
    ax1.annotate('', xy=(0, -1.8), xytext=(l_f, -1.8),
                arrowprops=dict(arrowstyle='<->', color='black', lw=2))
    ax1.text(l_f/2, -2.0, f'l_f = {l_f}m', ha='center', fontsize=11, fontweight='bold')
    
    ax1.annotate('', xy=(-l_r, -1.8), xytext=(0, -1.8),
                arrowprops=dict(arrowstyle='<->', color='black', lw=2))
    ax1.text(-l_r/2, -2.0, f'l_r = {l_r}m', ha='center', fontsize=11, fontweight='bold')
    
    ax1.legend(loc='upper right')
    
    # Diagram 2: Moment Analysis
    ax2.set_xlim(-3, 3)
    ax2.set_ylim(-2.5, 2.5)
    ax2.set_aspect('equal')
    ax2.grid(True, alpha=0.3)
    ax2.set_title('Moment Analysis', fontsize=14)
    
    # Vehicle outline
    vehicle2 = Rectangle((-l_r, -vehicle_width/2), l_f+l_r, vehicle_width, 
                        fill=False, linewidth=2, edgecolor='black')
    ax2.add_patch(vehicle2)
    ax2.plot(0, 0, 'ro', markersize=10)
    
    # Moment arms
    ax2.plot([0, l_f], [0, 0], 'b--', linewidth=3, alpha=0.8, label='Front moment arm')
    ax2.plot([0, -l_r], [0, 0], 'g--', linewidth=3, alpha=0.8, label='Rear moment arm')
    
    # Curved arrows for moments
    from matplotlib.patches import Arc
    
    # Front moment (positive direction)
    arc1 = Arc((l_f, 0), 0.8, 0.8, angle=0, theta1=0, theta2=180, 
               linewidth=3, color='blue')
    ax2.add_patch(arc1)
    ax2.arrow(l_f - 0.4, 0.35, 0.1, 0, head_width=0.05, head_length=0.05, 
              fc='blue', ec='blue', linewidth=2)
    ax2.text(l_f, 1.2, f'M_f = l_f × F_yf\n= {l_f * F_yf:.0f} N·m', 
             ha='center', fontsize=11, color='blue', fontweight='bold',
             bbox=dict(boxstyle="round,pad=0.3", facecolor="lightblue", alpha=0.7))
    
    # Rear moment (negative direction)
    arc2 = Arc((-l_r, 0), 0.8, 0.8, angle=0, theta1=180, theta2=360, 
               linewidth=3, color='green')
    ax2.add_patch(arc2)
    ax2.arrow(-l_r + 0.4, -0.35, -0.1, 0, head_width=0.05, head_length=0.05, 
              fc='green', ec='green', linewidth=2)
    ax2.text(-l_r, -1.4, f'M_r = l_r × F_yr\n= {l_r * F_yr:.0f} N·m', 
             ha='center', fontsize=11, color='green', fontweight='bold',
             bbox=dict(boxstyle="round,pad=0.3", facecolor="lightgreen", alpha=0.7))
    
    # Net moment
    net_moment = l_f * F_yf - l_r * F_yr
    ax2.text(0, 2.0, f'Net Moment = M_f - M_r = {net_moment:.0f} N·m', 
             ha='center', fontsize=12, fontweight='bold',
             bbox=dict(boxstyle="round,pad=0.5", facecolor="yellow", alpha=0.8))
    
    # Diagram 3: Velocity Components
    ax3.set_xlim(-3, 3)
    ax3.set_ylim(-2, 3)
    ax3.set_aspect('equal')
    ax3.grid(True, alpha=0.3)
    ax3.set_title('Velocity Components at Center of Mass', fontsize=14)
    
    # Vehicle outline
    vehicle3 = Rectangle((-l_r, -vehicle_width/2), l_f+l_r, vehicle_width, 
                        fill=False, linewidth=2, edgecolor='black')
    ax3.add_patch(vehicle3)
    ax3.plot(0, 0, 'ro', markersize=10)
    
    # Velocity components
    v_x = 2.0  # Visual scale
    v_y = 0.8  # Visual scale
    
    # Longitudinal velocity
    ax3.arrow(0, 0, v_x, 0, head_width=0.1, head_length=0.1, 
              fc='red', ec='red', linewidth=3, label='v_x (longitudinal)')
    ax3.text(v_x/2, -0.3, 'v_x', fontsize=14, color='red', fontweight='bold', ha='center')
    
    # Lateral velocity
    ax3.arrow(0, 0, 0, v_y, head_width=0.1, head_length=0.1, 
              fc='blue', ec='blue', linewidth=3, label='v_y (lateral)')
    ax3.text(-0.3, v_y/2, 'v_y', fontsize=14, color='blue', fontweight='bold', ha='center')
    
    # Resultant velocity
    ax3.arrow(0, 0, v_x, v_y, head_width=0.1, head_length=0.1, 
              fc='purple', ec='purple', linewidth=3, linestyle='--', label='Resultant velocity')
    
    # Sideslip angle
    angle = np.arctan2(v_y, v_x)
    arc3 = Arc((0, 0), 0.8, 0.8, angle=0, theta1=0, theta2=np.degrees(angle), 
               linewidth=2, color='purple')
    ax3.add_patch(arc3)
    ax3.text(0.6, 0.2, 'β (sideslip)', fontsize=12, color='purple', fontweight='bold')
    
    # Yaw rate visualization
    yaw_circle = Circle((0, 0), 1.5, fill=False, linestyle=':', linewidth=2, color='orange')
    ax3.add_patch(yaw_circle)
    ax3.text(1.1, 1.1, 'ψ̇ (yaw rate)', fontsize=12, color='orange', fontweight='bold')
    
    ax3.legend(loc='upper right')
    
    # Diagram 4: Equation Summary
    ax4.set_xlim(0, 10)
    ax4.set_ylim(0, 10)
    ax4.axis('off')
    ax4.set_title('Equation Summary', fontsize=14)
    
    # Equation 1
    ax4.text(5, 8.5, 'Equation 1: Lateral Force Balance', 
             fontsize=14, fontweight='bold', ha='center',
             bbox=dict(boxstyle="round,pad=0.5", facecolor="lightblue", alpha=0.8))
    
    ax4.text(5, 7.5, 'm(v̇_y + v_x ψ̇) = F_yf cos(δ) + F_yr', 
             fontsize=16, ha='center', fontweight='bold',
             bbox=dict(boxstyle="round,pad=0.3", facecolor="white", edgecolor="blue"))
    
    ax4.text(1, 6.5, 'Inertial Force', fontsize=12, ha='left', color='blue')
    ax4.text(9, 6.5, 'Tire Forces', fontsize=12, ha='right', color='blue')
    
    # Equation 2
    ax4.text(5, 5, 'Equation 2: Yaw Moment Balance', 
             fontsize=14, fontweight='bold', ha='center',
             bbox=dict(boxstyle="round,pad=0.5", facecolor="lightgreen", alpha=0.8))
    
    ax4.text(5, 4, 'I_z ψ̈ = l_f F_yf cos(δ) - l_r F_yr', 
             fontsize=16, ha='center', fontweight='bold',
             bbox=dict(boxstyle="round,pad=0.3", facecolor="white", edgecolor="green"))
    
    ax4.text(1, 3, 'Inertial Moment', fontsize=12, ha='left', color='green')
    ax4.text(9, 3, 'Tire Moments', fontsize=12, ha='right', color='green')
    
    # Applications
    ax4.text(5, 1.5, 'Key Applications:', fontsize=14, fontweight='bold', ha='center')
    applications = [
        '• Vehicle Stability Control (ESP/VSC)',
        '• Autonomous Vehicle Path Tracking',
        '• Tire Performance Analysis',
        '• Vehicle Design Optimization'
    ]
    
    for i, app in enumerate(applications):
        ax4.text(5, 1 - i*0.3, app, fontsize=11, ha='center')
    
    plt.tight_layout()
    return fig

def demonstrate_parameter_effects():
    """Demonstrate how different parameters affect vehicle behavior"""
    
    print("\n" + "="*60)
    print("参数影响分析")
    print("="*60)
    
    # Base parameters
    m = 1500.0
    I_z = 2500.0
    l_f = 1.2
    l_r = 1.6
    C_f = 80000.0
    C_r = 80000.0
    
    # Scenario parameters
    v_x = 20.0  # m/s
    delta = 0.05  # rad
    
    print(f"\n基准参数:")
    print(f"  车辆质量: {m} kg")
    print(f"  转动惯量: {I_z} kg·m²")
    print(f"  前轴距离: {l_f} m")
    print(f"  后轴距离: {l_r} m")
    print(f"  前轮刚度: {C_f} N/rad")
    print(f"  后轮刚度: {C_r} N/rad")
    print(f"  行驶速度: {v_x} m/s")
    print(f"  转向角: {delta} rad = {np.degrees(delta):.1f}°")
    
    # Calculate steady-state response for different parameters
    scenarios = [
        ("Base Case", m, I_z, l_f, l_r, C_f, C_r),
        ("Heavy Vehicle (+50% mass)", m*1.5, I_z*1.5, l_f, l_r, C_f, C_r),
        ("Front-biased CG", m, I_z, l_f*0.8, l_r*1.2, C_f, C_r),
        ("Rear-biased CG", m, I_z, l_f*1.2, l_r*0.8, C_f, C_r),
        ("Soft Front Tires", m, I_z, l_f, l_r, C_f*0.7, C_r),
        ("Soft Rear Tires", m, I_z, l_f, l_r, C_f, C_r*0.7),
    ]
    
    results = []
    
    for name, m_i, I_z_i, l_f_i, l_r_i, C_f_i, C_r_i in scenarios:
        # Simplified steady-state calculation
        # Assume small angles and linear tire model
        
        # Vehicle characteristic parameters
        L = l_f_i + l_r_i
        K_us = (m_i * l_f_i) / (L * C_r_i) - (m_i * l_r_i) / (L * C_f_i)  # Understeer gradient
        
        # Steady-state yaw rate
        psi_dot_ss = (v_x * delta) / (L + K_us * v_x**2)
        
        # Steady-state lateral acceleration
        a_y_ss = v_x * psi_dot_ss
        
        # Steady-state sideslip angle
        beta_ss = (l_r_i * delta) / (L + K_us * v_x**2)
        
        results.append({
            'name': name,
            'psi_dot': psi_dot_ss,
            'a_y': a_y_ss,
            'beta': beta_ss,
            'K_us': K_us
        })
    
    # Display results
    print(f"\n{'Scenario':<20} {'Yaw Rate':<12} {'Lat. Accel':<12} {'Sideslip':<12} {'Understeer':<12}")
    print(f"{'':20} {'[deg/s]':<12} {'[m/s²]':<12} {'[deg]':<12} {'Gradient':<12}")
    print("-" * 80)
    
    for result in results:
        print(f"{result['name']:<20} "
              f"{result['psi_dot']*180/np.pi:<12.2f} "
              f"{result['a_y']:<12.2f} "
              f"{result['beta']*180/np.pi:<12.2f} "
              f"{result['K_us']:<12.4f}")
    
    # Plot comparison
    fig, ((ax1, ax2), (ax3, ax4)) = plt.subplots(2, 2, figsize=(15, 10))
    fig.suptitle('Parameter Effects on Vehicle Response', fontsize=16)
    
    names = [r['name'] for r in results]
    psi_dots = [r['psi_dot'] * 180/np.pi for r in results]
    a_ys = [r['a_y'] for r in results]
    betas = [r['beta'] * 180/np.pi for r in results]
    K_us_values = [r['K_us'] for r in results]
    
    # Yaw rate comparison
    bars1 = ax1.bar(range(len(names)), psi_dots, color=['red' if i==0 else 'lightblue' for i in range(len(names))])
    ax1.set_title('Steady-State Yaw Rate')
    ax1.set_ylabel('Yaw Rate [deg/s]')
    ax1.set_xticks(range(len(names)))
    ax1.set_xticklabels(names, rotation=45, ha='right')
    ax1.grid(True, alpha=0.3)
    
    # Lateral acceleration comparison
    bars2 = ax2.bar(range(len(names)), a_ys, color=['red' if i==0 else 'lightgreen' for i in range(len(names))])
    ax2.set_title('Steady-State Lateral Acceleration')
    ax2.set_ylabel('Lateral Acceleration [m/s²]')
    ax2.set_xticks(range(len(names)))
    ax2.set_xticklabels(names, rotation=45, ha='right')
    ax2.grid(True, alpha=0.3)
    
    # Sideslip angle comparison
    bars3 = ax3.bar(range(len(names)), betas, color=['red' if i==0 else 'lightyellow' for i in range(len(names))])
    ax3.set_title('Steady-State Sideslip Angle')
    ax3.set_ylabel('Sideslip Angle [deg]')
    ax3.set_xticks(range(len(names)))
    ax3.set_xticklabels(names, rotation=45, ha='right')
    ax3.grid(True, alpha=0.3)
    
    # Understeer gradient comparison
    colors = ['red' if i==0 else ('orange' if K_us_values[i] > 0 else 'lightcoral') for i in range(len(names))]
    bars4 = ax4.bar(range(len(names)), K_us_values, color=colors)
    ax4.set_title('Understeer Gradient')
    ax4.set_ylabel('Understeer Gradient [s²/m]')
    ax4.set_xticks(range(len(names)))
    ax4.set_xticklabels(names, rotation=45, ha='right')
    ax4.grid(True, alpha=0.3)
    ax4.axhline(y=0, color='black', linestyle='--', alpha=0.5)
    
    plt.tight_layout()
    return fig

def main():
    """Main function to run all explanations"""
    
    # Text explanation
    explain_equations()
    
    # Visual diagrams
    fig1 = create_detailed_force_diagram()
    plt.show()
    
    # Parameter analysis
    fig2 = demonstrate_parameter_effects()
    plt.show()
    
    print("\n" + "="*60)
    print("结论")
    print("="*60)
    print("横向动力学方程是车辆动力学的核心，它们:")
    print("1. 描述了轮胎力与车辆运动之间的关系")
    print("2. 是车辆稳定性控制系统的理论基础")
    print("3. 帮助工程师设计更安全、更稳定的车辆")
    print("4. 在自动驾驶中用于精确的路径跟踪控制")
    print("5. 通过参数调整可以改变车辆的操控特性")

if __name__ == "__main__":
    main() 