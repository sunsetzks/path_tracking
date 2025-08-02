"""
调试Stanley控制器
"""

import numpy as np
import sys
import os

# Add stanley_controller to path
sys.path.append('/Users/kuisongzheng/ws/path_tracking')

from stanley_controller import StanleyController, SimulationEnvironment
from stanley_controller.vehicle_dynamics import SimpleVehicleDynamics
from stanley_controller.utils.se2 import SE2
from stanley_controller.simulation import SimulationConfig, ScenarioGenerator
from stanley_controller.stanley_controller import ControlParams

def debug_stanley():
    """调试Stanley控制器的基本操作"""
    
    # 创建控制器
    control_params = ControlParams(
        k_cross_track=0.5,
        k_heading=1.0,
        max_steer_angle=np.radians(30.0),
        wheelbase=2.9,
        lookahead_distance=5.0
    )
    controller = StanleyController(control_params)
    
    # 创建简单的直线路径进行测试
    path_points = np.array([[0, 0], [10, 0], [20, 0], [30, 0], [40, 0]])
    path_yaw = np.array([0, 0, 0, 0, 0])
    
    # 车辆初始状态
    state = SE2(x=0.0, y=0.0, theta=0.0)
    target_speed = 5.0
    current_speed = 0.0
    
    print("=== 调试Stanley控制器 ===")
    print(f"初始状态: x={state.x:.2f}, y={state.y:.2f}, theta={np.degrees(state.theta):.1f}°")
    print(f"目标速度: {target_speed} m/s")
    print(f"当前速度: {current_speed} m/s")
    
    # 计算控制命令
    steering, acceleration, target_idx = controller.compute_control(
        state, path_points, path_yaw, target_speed, current_speed
    )
    
    print(f"控制输出:")
    print(f"  转向角: {np.degrees(steering):.2f}°")
    print(f"  加速度: {acceleration:.2f} m/s²")
    print(f"  目标索引: {target_idx}")
    
    # 测试车辆动力学
    vehicle = SimpleVehicleDynamics(wheelbase=2.9)
    print(f"\n=== 测试车辆动力学 ===")
    print(f"初始状态: x={vehicle.x:.2f}, y={vehicle.y:.2f}, yaw={np.degrees(vehicle.yaw):.1f}°, speed={vehicle.speed:.2f}")
    
    # 更新车辆状态
    dt = 0.1
    for i in range(10):
        vehicle.update(dt, steering, acceleration)
        print(f"步骤 {i+1}: x={vehicle.x:.2f}, y={vehicle.y:.2f}, yaw={np.degrees(vehicle.yaw):.1f}°, speed={vehicle.speed:.2f}")
        
        if i == 4:  # 中途改变加速度为0，看看是否保持速度
            acceleration = 0.0
            print("  -> 加速度设为0")

if __name__ == "__main__":
    debug_stanley()
