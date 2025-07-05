#!/usr/bin/env python3
"""
Simple Odometry vs True Trajectory Comparison

This script clearly demonstrates the difference between true trajectory 
and odometry estimation with noise.

Author: Assistant
"""

import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path
import sys

# Add the parent directory to the path to import PathTracking modules
sys.path.append(str(Path(__file__).parent))

from PathTracking.config import load_config
from PathTracking.vehicle_model import VehicleModel, VehicleState


def compare_trajectories():
    """Compare true trajectory vs odometry trajectory with clear visualization"""
    
    print("=" * 60)
    print("真实轨迹 vs 里程计轨迹对比")
    print("=" * 60)
    
    # Load config with enhanced noise for clear demonstration
    config = load_config('config.yaml')
    
    # Use the updated noise parameters from config.yaml
    print("使用的噪声参数:")
    print(f"  位置噪声标准差: {config.vehicle.odometry_position_noise_std} m")
    print(f"  角度噪声标准差: {config.vehicle.odometry_yaw_noise_std} rad ({np.degrees(config.vehicle.odometry_yaw_noise_std):.1f}°)")
    print(f"  速度噪声标准差: {config.vehicle.odometry_velocity_noise_std} m/s")
    print(f"  随机种子: {config.vehicle.noise_seed}")
    print()
    
    # Create vehicle model
    vehicle = VehicleModel(config.vehicle)
    
    # Create trajectory data
    true_trajectory = []
    odometry_trajectory = []
    times = []
    
    # Simulation parameters
    dt = 0.1
    total_time = 20.0
    steps = int(total_time / dt)
    
    print("运行仿真...")
    
    for i in range(steps):
        t = i * dt
        times.append(t)
        
        # Create varied motion pattern
        if t < 3.0:
            # Accelerate forward
            control = (0.0, 1.0)
        elif t < 8.0:
            # Turn left
            control = (0.3, 0.2)
        elif t < 13.0:
            # Turn right
            control = (-0.4, 0.0)
        elif t < 18.0:
            # Gentle curve
            control = (0.1, 0.0)
        else:
            # Straight
            control = (0.0, 0.0)
        
        # Update vehicle
        vehicle.update_with_rates(control, dt)
        
        # Get both state types
        true_state = vehicle.get_true_state()
        odometry_state = vehicle.get_odometry_state()
        
        # Store trajectories
        true_trajectory.append([true_state.position_x, true_state.position_y])
        odometry_trajectory.append([odometry_state.position_x, odometry_state.position_y])
    
    # Convert to numpy arrays
    true_traj = np.array(true_trajectory)
    odom_traj = np.array(odometry_trajectory)
    
    # Calculate statistics
    position_errors = np.sqrt((true_traj[:, 0] - odom_traj[:, 0])**2 + 
                             (true_traj[:, 1] - odom_traj[:, 1])**2)
    
    final_error = position_errors[-1]
    mean_error = np.mean(position_errors)
    max_error = np.max(position_errors)
    
    print(f"轨迹统计:")
    print(f"  最终位置误差: {final_error:.3f} m")
    print(f"  平均位置误差: {mean_error:.3f} m")
    print(f"  最大位置误差: {max_error:.3f} m")
    print()
    
    # Print some specific points for comparison
    print("轨迹对比 (每5秒一个点):")
    print("时间    真实位置              里程计位置            误差")
    print("-" * 65)
    for i in range(0, len(times), 50):  # Every 5 seconds
        t = times[i]
        true_pos = true_traj[i]
        odom_pos = odom_traj[i]
        error = position_errors[i]
        print(f"{t:4.1f}s   ({true_pos[0]:6.2f}, {true_pos[1]:6.2f})   ({odom_pos[0]:6.2f}, {odom_pos[1]:6.2f})   {error:.3f}m")
    
    # Create visualization
    plt.figure(figsize=(15, 10))
    
    # Plot 1: Trajectory comparison
    plt.subplot(2, 2, 1)
    plt.plot(true_traj[:, 0], true_traj[:, 1], 'b-', linewidth=3, label='True Trajectory', alpha=0.8)
    plt.plot(odom_traj[:, 0], odom_traj[:, 1], 'r--', linewidth=2, label='Odometry Trajectory', alpha=0.8)
    
    # Mark start and end points
    plt.plot(true_traj[0, 0], true_traj[0, 1], 'go', markersize=10, label='Start')
    plt.plot(true_traj[-1, 0], true_traj[-1, 1], 'ro', markersize=10, label='End (True)')
    plt.plot(odom_traj[-1, 0], odom_traj[-1, 1], 'r^', markersize=10, label='End (Odometry)')
    
    plt.xlabel('X Position [m]')
    plt.ylabel('Y Position [m]')
    plt.title('Trajectory Comparison')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    
    # Plot 2: Position error over time
    plt.subplot(2, 2, 2)
    plt.plot(times, position_errors, 'g-', linewidth=2)
    plt.xlabel('Time [s]')
    plt.ylabel('Position Error [m]')
    plt.title('Position Error Over Time')
    plt.grid(True, alpha=0.3)
    
    # Plot 3: X position comparison
    plt.subplot(2, 2, 3)
    plt.plot(times, true_traj[:, 0], 'b-', linewidth=2, label='True X')
    plt.plot(times, odom_traj[:, 0], 'r--', linewidth=2, label='Odometry X')
    plt.xlabel('Time [s]')
    plt.ylabel('X Position [m]')
    plt.title('X Position Comparison')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    # Plot 4: Y position comparison
    plt.subplot(2, 2, 4)
    plt.plot(times, true_traj[:, 1], 'b-', linewidth=2, label='True Y')
    plt.plot(times, odom_traj[:, 1], 'r--', linewidth=2, label='Odometry Y')
    plt.xlabel('Time [s]')
    plt.ylabel('Y Position [m]')
    plt.title('Y Position Comparison')
    plt.legend()
    plt.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.savefig('trajectory_comparison.png', dpi=150, bbox_inches='tight')
    plt.show()
    
    print(f"\n图表已保存为 'trajectory_comparison.png'")
    
    # Show clear difference
    if final_error > 0.1:
        print("✅ 里程计和真实轨迹有明显差异！")
    elif final_error > 0.01:
        print("⚠️  里程计和真实轨迹有小幅差异。")
    else:
        print("❌ 里程计和真实轨迹差异很小，可能需要增大噪声参数。")


def demonstrate_state_access():
    """Demonstrate correct way to access different state types"""
    
    print("\n" + "=" * 60)
    print("正确访问不同状态类型的方法")
    print("=" * 60)
    
    config = load_config('config.yaml')
    vehicle = VehicleModel(config.vehicle)
    
    # Run a few steps
    for i in range(3):
        vehicle.update_with_rates((0.1, 0.5), 0.1)
    
    print("获取不同状态类型的方法:")
    print()
    
    # Method 1: Explicit state type
    true_state = vehicle.get_true_state()
    odometry_state = vehicle.get_odometry_state()
    global_state = vehicle.get_global_state()
    
    print("方法1 - 显式调用特定状态类型:")
    print(f"  vehicle.get_true_state():     位置 = ({true_state.position_x:.3f}, {true_state.position_y:.3f})")
    print(f"  vehicle.get_odometry_state(): 位置 = ({odometry_state.position_x:.3f}, {odometry_state.position_y:.3f})")
    print(f"  vehicle.get_global_state():   位置 = ({global_state.position_x:.3f}, {global_state.position_y:.3f})")
    print()
    
    # Method 2: Using get_state() with parameter
    true_state2 = vehicle.get_state("true")
    odometry_state2 = vehicle.get_state("odometry")
    global_state2 = vehicle.get_state("global")
    
    print("方法2 - 使用 get_state() 参数:")
    print(f"  vehicle.get_state('true'):     位置 = ({true_state2.position_x:.3f}, {true_state2.position_y:.3f})")
    print(f"  vehicle.get_state('odometry'): 位置 = ({odometry_state2.position_x:.3f}, {odometry_state2.position_y:.3f})")
    print(f"  vehicle.get_state('global'):   位置 = ({global_state2.position_x:.3f}, {global_state2.position_y:.3f})")
    print()
    
    # Method 3: Default state type
    default_state = vehicle.get_state()  # Uses config.vehicle.default_state_type
    print("方法3 - 默认状态类型:")
    print(f"  vehicle.get_state():          位置 = ({default_state.position_x:.3f}, {default_state.position_y:.3f})")
    print(f"  默认状态类型: '{config.vehicle.default_state_type}'")
    print()
    
    print("⚠️  注意:")
    print("  - 如果您想要里程计状态，请使用 get_odometry_state() 或 get_state('odometry')")
    print("  - 如果您想要真实状态，请使用 get_true_state() 或 get_state('true')")
    print("  - get_state() 默认返回真实状态，除非在配置中修改了 default_state_type")


if __name__ == "__main__":
    # Run the comparison
    compare_trajectories()
    
    # Demonstrate correct state access
    demonstrate_state_access()
    
    print("\n" + "=" * 60)
    print("示例完成！")
    print("=" * 60) 