# PathTracking 配置管理系统

PathTracking 项目已经成功实现了一个完整的全局配置管理系统，提供了灵活而强大的参数管理功能。

## 🎯 系统特性

### ✅ 已实现功能

1. **集中化配置管理** - 所有组件参数统一管理
2. **YAML 配置文件支持** - 标准配置文件格式
3. **预设配置方案** - 针对不同场景优化的参数组合
4. **环境变量支持** - 运行时参数覆盖
5. **混合参数模式** - 支持部分自定义参数
6. **向后兼容性** - 现有代码无需修改即可使用

## 📁 配置文件结构

```yaml
# config.yaml - 主配置文件
vehicle:           # 车辆模型参数
  wheelbase: 2.9
  max_steering_angle: 45.0
  max_velocity: 50.0
  # ... 更多参数

pure_pursuit:      # Pure Pursuit 控制器参数
  min_lookahead: 1.0
  k_gain: 10.0
  # ... 更多参数

velocity_controller: # 速度控制器参数
  max_forward_velocity: 5.0
  max_acceleration: 1.0
  # ... 更多参数

trajectory:        # 轨迹相关参数
  discretization_distance: 0.1
  # ... 更多参数

simulation:        # 仿真和可视化参数
  time_step: 0.1
  plot_margin: 5.0
  # ... 更多参数
```

## 🚀 使用方式

### 1. 默认配置（自动加载）
```python
from PathTracking.vehicle_model import VehicleModel
from PathTracking.pure_pursuit import PurePursuitController

# 组件自动使用全局配置
vehicle = VehicleModel()
controller = PurePursuitController(wheelbase=2.9)
```

### 2. 预设配置方案
```python
from PathTracking.config import load_preset_config

# 停车场景配置 - 低速精确控制
load_preset_config('parking')

# 高速公路配置 - 高速稳定控制  
load_preset_config('high_speed')

# 前进驾驶配置 - 平衡性能
load_preset_config('forward')

# 倒车配置 - 谨慎操作
load_preset_config('reverse')
```

### 3. 自定义配置文件
```python
from PathTracking.config import ConfigManager

# 从文件加载配置
config_manager = ConfigManager('my_custom_config.yaml')
```

### 4. 环境变量配置
```bash
export PT_VEHICLE_WHEELBASE=3.0
export PT_PUREPURSUIT_MIN_LOOKAHEAD=2.0
python your_script.py  # 自动使用环境变量
```

### 5. 混合参数模式
```python
# 部分参数使用配置文件，部分自定义
vehicle = VehicleModel(
    wheelbase=3.5,        # 自定义轴距
    max_velocity=60.0,    # 自定义最大速度
    # 其他参数自动从配置加载
)
```

## 📊 预设配置对比

| 参数 | Default | Forward | Reverse | Parking | High Speed |
|------|---------|---------|---------|---------|------------|
| 最大速度 (m/s) | 50.0 | 15.0 | 8.0 | **5.0** | **30.0** |
| 前进速度限制 (m/s) | 5.0 | **6.0** | 3.0 | **2.0** | **15.0** |
| 最小前瞻距离 (m) | 1.0 | **2.5** | 1.5 | **1.0** | **5.0** |
| 目标容差 (m) | 0.5 | 1.0 | 0.8 | **0.3** | **2.0** |
| 最大加速度 (m/s²) | 1.0 | **2.0** | 1.0 | **0.8** | **3.0** |

## 🔧 核心组件配置

### VehicleConfig - 车辆模型参数
- **物理参数**: wheelbase, vehicle_length, vehicle_width
- **运动学限制**: max_steering_angle, max_velocity, max_acceleration  
- **时延参数**: steering_delay, acceleration_delay
- **控制增益**: steering_rate_gain, acceleration_gain

### PurePursuitConfig - Pure Pursuit 控制器参数
- **前瞻距离**: min_lookahead, k_gain
- **控制限制**: max_steering_angle
- **目标检测**: goal_tolerance, velocity_tolerance

### VelocityControllerConfig - 速度控制器参数  
- **速度限制**: max_forward_velocity, max_backward_velocity
- **加速度限制**: max_acceleration, max_deceleration
- **安全参数**: conservative_braking_factor, min_velocity

### TrajectoryConfig - 轨迹参数
- **离散化**: discretization_distance
- **采样**: default_sample_count

### SimulationConfig - 仿真参数
- **时间步长**: time_step, max_time
- **可视化**: plot_margin, figure_size, animation_interval

## 🌟 系统优势

1. **零配置启动** - 所有组件都有合理的默认值
2. **场景优化** - 预设配置针对特定使用场景调优
3. **灵活性** - 支持完全自定义到部分覆盖的各种使用模式
4. **类型安全** - 完整的类型注解和运行时验证
5. **环境适应** - 支持环境变量进行运行时配置
6. **向后兼容** - 现有代码无需任何修改

## 📝 使用示例

查看 `config_example.py` 文件获取完整的使用示例，包括：
- 基本配置使用
- 从文件加载配置
- 预设配置切换  
- 环境变量配置
- 混合参数模式
- 配置保存和加载

## 🎮 快速开始

```python
# 1. 使用默认配置
from PathTracking.vehicle_model import VehicleModel
vehicle = VehicleModel()

# 2. 切换到停车模式
from PathTracking.config import load_preset_config
load_preset_config('parking')
# 现在所有组件都将使用停车优化参数

# 3. 验证配置
from PathTracking.config import get_vehicle_config
config = get_vehicle_config()
print(f"当前最大速度: {config.max_velocity}m/s")  # 输出: 5.0m/s
```

配置系统已经完全集成到所有核心组件中，提供了现代化、灵活且易用的参数管理解决方案。 