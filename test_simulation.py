"""
测试完整仿真循环
"""

import numpy as np
import sys
sys.path.append('/Users/kuisongzheng/ws/path_tracking')

from stanley_controller import StanleyController, SimulationEnvironment
from stanley_controller.vehicle_dynamics import SimpleVehicleDynamics
from stanley_controller.utils.se2 import SE2
from stanley_controller.simulation import SimulationConfig, ScenarioGenerator
from stanley_controller.stanley_controller import ControlParams

def test_simulation():
    """测试仿真循环"""
    
    # 配置
    config = SimulationConfig(dt=0.1, max_time=10.0)
    control_params = ControlParams()
    controller = StanleyController(control_params)
    vehicle_dynamics = SimpleVehicleDynamics(wheelbase=2.9)
    
    # 创建简单直线路径
    path_points = np.array([[0, 0], [50, 0], [100, 0]])
    path_yaw = np.array([0, 0, 0])
    initial_state = SE2(x=0.0, y=0.0, theta=0.0)
    target_speed = 5.0
    
    # 仿真环境
    sim_env = SimulationEnvironment(config)
    
    print("开始仿真...")
    
    # 手动仿真几步
    vehicle_dynamics.reset()
    state = SE2(initial_state.x, initial_state.y, initial_state.theta)
    
    for i in range(50):  # 5秒仿真
        current_speed = vehicle_dynamics.get_speed()
        
        # 计算控制
        steering, acceleration, target_idx = controller.compute_control(
            state, path_points, path_yaw, target_speed, current_speed)
        
        # 更新车辆
        vehicle_dynamics.update(config.dt, steering, acceleration)
        state = vehicle_dynamics.get_state()
        
        if i % 10 == 0:  # 每秒打印一次
            print(f"步骤 {i}: x={state.x:.2f}, speed={current_speed:.2f}, accel={acceleration:.2f}, target_idx={target_idx}")
    
    print("手动仿真完成")
    
    # 现在测试完整仿真环境
    print("\n测试完整仿真环境...")
    result = sim_env.simulate(controller, vehicle_dynamics, initial_state, path_points, path_yaw, target_speed)
    
    print(f"仿真结果:")
    print(f"  成功: {result['success']}")
    print(f"  时间: {result['time']:.2f}s")
    print(f"  步数: {result['steps']}")
    if result['metrics']:
        print(f"  平均速度: {result['metrics']['average_speed']:.2f} m/s")

if __name__ == "__main__":
    test_simulation()
