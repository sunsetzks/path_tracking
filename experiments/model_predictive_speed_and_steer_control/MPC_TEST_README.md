# MPC Solver Test Suite

这个测试套件专门用于测试 `solve_linear_mpc` 函数的功能和性能。

## 文件结构

- `mpc_core.py` - 提取的核心 MPC 函数
- `test_mpc_solver.py` - 完整的测试套件
- `run_mpc_test.py` - 简化的运行脚本
- `model_predictive_speed_and_steer_control.py` - 原始完整仿真代码

## 核心功能

### mpc_core.py
包含从原始代码中提取的核心 MPC 函数：
- `solve_linear_mpc()` - 主要的 MPC 优化求解器
- `get_linearized_model_matrices()` - 线性化模型矩阵计算
- `normalize_angle()` - 角度归一化
- `convert_matrix_to_array()` - 矩阵转换工具

### test_mpc_solver.py
提供全面的测试功能：

#### 测试用例类型
1. **直线轨迹** - 简单的直线参考轨迹
2. **曲线轨迹** - 抛物线形参考轨迹
3. **圆形轨迹** - 圆形参考轨迹
4. **随机轨迹** - 随机生成的参考轨迹

#### 初始状态测试
- 原点起始
- 位置偏移
- 速度起始
- 角度偏移

#### 约束测试
- 速度约束验证
- 转向角约束验证
- 加速度约束验证

## 使用方法

### 运行完整测试套件
```bash
cd /Users/kuisongzheng/ws/path_tracking/experiments/model_predictive_speed_and_steer_control
python run_mpc_test.py
```

### 运行特定测试
```python
from test_mpc_solver import test_mpc_solver, create_straight_line_reference

# 创建测试数据
reference_trajectory, reference_steering = create_straight_line_reference()
initial_state = [0.0, 0.0, 0.0, 0.0]

# 运行测试
success, result = test_mpc_solver(
    "My Test",
    reference_trajectory,
    reference_steering,
    initial_state
)
```

### 自定义测试用例
```python
import numpy as np
from mpc_core import solve_linear_mpc

# 创建自定义参考轨迹
reference_trajectory = np.zeros((4, 6))  # 4 states, 6 time steps
reference_steering = np.zeros((1, 6))

# 设置参考轨迹
for i in range(6):
    reference_trajectory[0, i] = i * 1.0  # x position
    reference_trajectory[1, i] = 0.0      # y position
    reference_trajectory[2, i] = 3.0      # velocity
    reference_trajectory[3, i] = 0.0      # yaw angle

# 设置初始状态
initial_state = [0.0, 0.0, 0.0, 0.0]

# 创建线性化轨迹
linearization_trajectory = reference_trajectory.copy()

# 求解 MPC
result = solve_linear_mpc(
    reference_trajectory,
    linearization_trajectory,
    initial_state,
    reference_steering
)
```

## 测试输出

测试会显示：
- 每个测试用例的成功/失败状态
- 求解的控制序列（加速度和转向角）
- 预测的轨迹状态
- 约束验证结果
- 可视化图表（轨迹对比、速度曲线、控制输入等）

## 参数说明

### MPC 参数
- `PREDICTION_HORIZON = 5` - 预测时域
- `TIME_STEP = 0.2` - 时间步长
- `STATE_DIMENSION = 4` - 状态维度 [x, y, velocity, yaw]
- `CONTROL_DIMENSION = 2` - 控制维度 [acceleration, steering]

### 约束参数
- `MAX_VELOCITY = 55.0/3.6` m/s - 最大速度
- `MIN_VELOCITY = -20.0/3.6` m/s - 最小速度
- `MAX_ACCELERATION = 1.0` m/s² - 最大加速度
- `MAX_STEERING_ANGLE = 45°` - 最大转向角
- `MAX_STEERING_RATE = 30°/s` - 最大转向速率

## 故障排除

如果测试失败，检查：
1. 依赖包是否正确安装（cvxpy, numpy, matplotlib）
2. 参考轨迹是否合理
3. 初始状态是否在约束范围内
4. 线性化轨迹是否合理

## 扩展功能

可以轻松添加新的测试用例：
1. 在 `test_mpc_solver.py` 中添加新的轨迹生成函数
2. 在 `run_comprehensive_tests()` 中添加新的测试用例
3. 自定义初始状态和约束条件
