# Hybrid A* Path Planning with eCAL Integration

这是一个完整的Hybrid A*路径规划算法实现，支持eCAL中间件和Foxglove Studio可视化。

## 功能特性

- ✅ 完整的Hybrid A*路径规划算法
- ✅ **灵活的碰撞检测接口** - 支持网格地图、几何形状和自定义检测器
- ✅ eCAL中间件集成支持
- ✅ Foxglove Studio可视化
- ✅ Protobuf消息协议
- ✅ 实时规划状态发布
- ✅ 完整的可视化数据流

## 新特性：模块化碰撞检测

从v2.0开始，我们引入了灵活的碰撞检测接口，替代了硬编码的障碍物地图方式：

### 快速开始
```cpp
#include "hybrid_astar.hpp"
#include "collision_detector.hpp"

// 创建规划器
HybridAStar planner(config);

// 方式1：网格地图碰撞检测
auto grid_detector = std::make_shared<GridCollisionDetector>(
    obstacle_map, grid_resolution, origin_x, origin_y);
planner.set_collision_detector(grid_detector);

// 方式2：几何形状碰撞检测
auto geometric_detector = std::make_shared<GeometricCollisionDetector>(vehicle_radius);
geometric_detector->add_circle_obstacle(x, y, radius);
geometric_detector->add_rectangle_obstacle(x_min, y_min, x_max, y_max);
planner.set_collision_detector(geometric_detector);

// 方式3：自定义碰撞检测
class MyDetector : public CollisionDetector {
    bool is_collision_free(const State& state) const override {
        // 你的碰撞检测逻辑
        return true;
    }
};
auto custom_detector = std::make_shared<MyDetector>();
planner.set_collision_detector(custom_detector);
```

📖 详细文档：[碰撞检测接口](COLLISION_DETECTION.md) | [迁移指南](MIGRATION_GUIDE.md)

## 系统架构

```
┌─────────────────────┐    ┌──────────────────┐    ┌─────────────────────┐
│   Hybrid A* Core    │    │  Visualization   │    │   Foxglove Studio   │
│                     │────│   Publisher      │────│                     │
│ - Path Planning     │    │                  │    │ - Real-time Display │
│ - Obstacle Avoidance│    │ - eCAL Integration│    │ - 3D Visualization  │
│ - Vehicle Dynamics  │    │ - Protobuf Msgs  │    │ - Data Analysis     │
└─────────────────────┘    └──────────────────┘    └─────────────────────┘
```

## 依赖项

### 必需依赖
- CMake 3.16+
- C++17 编译器
- pthread

### 可选依赖（用于完整功能）
- eCAL (Eclipse Cyclone DDS)
- Protocol Buffers 3.12+
- Foxglove Studio

## 编译构建

### 基础版本（无eCAL）
```bash
mkdir build && cd build
cmake -DECAL_FOUND=OFF -DProtobuf_FOUND=OFF ..
make
```

### 完整版本（with eCAL）
```bash
# 安装eCAL (Ubuntu/Debian)
sudo apt update
sudo apt install ecal-dev

# 构建
mkdir build && cd build
cmake ..
make
```

## 使用方法

### 1. 运行基础Demo
```bash
./hybrid_astar_demo
```

### 2. 运行eCAL集成Demo
```bash
# 启动eCAL系统
ecal_mon &

# 运行demo
./hybrid_astar_ecal_demo

# 查看topic列表
ecal_info
```

### 3. Foxglove Studio可视化
1. 下载并安装 [Foxglove Studio](https://foxglove.dev/download)
2. 启动Foxglove Studio
3. 连接到eCAL数据源
4. 添加可视化面板：
   - Grid (障碍物地图)
   - Path (规划路径)
   - MarkerArray (探索节点和车辆)

## 消息协议

### eCAL Topics

| Topic | Type | Description |
|-------|------|-------------|
| `planning_result` | `hybrid_astar::PlanningResult` | 完整规划结果 |
| `planning_status` | `hybrid_astar::PlanningStatus` | 实时规划状态 |
| `obstacle_map` | `foxglove::Grid` | 障碍物地图 |
| `planned_path` | `foxglove::Path` | 规划路径 |
| `planning_markers` | `foxglove::MarkerArray` | 可视化标记 |

### 消息结构

#### PlanningResult
```protobuf
message PlanningResult {
  foxglove.Time timestamp = 1;
  string frame_id = 2;
  VehicleState start_state = 3;
  VehicleState goal_state = 4;
  repeated PlanningNode path_nodes = 5;
  repeated PlanningNode explored_nodes = 6;
  repeated VehicleState detailed_path = 7;
  PlanningStatistics statistics = 8;
  foxglove.Grid obstacle_map = 9;
  foxglove.Path planned_path = 10;
  foxglove.MarkerArray exploration_markers = 11;
  foxglove.MarkerArray vehicle_markers = 12;
}
```

## API使用示例

```cpp
#include "hybrid_astar.hpp"
#include "visualization_publisher.hpp"

// 创建配置
PlanningConfig config;
config.max_steer = 0.6;
config.grid_resolution = 0.5;

// 创建规划器
HybridAStar planner(config);

// 设置障碍物地图
std::vector<std::vector<int>> obstacle_map = ...;
planner.set_obstacle_map(obstacle_map, origin_x, origin_y);

// 定义起点和终点
State start{-3.0, -3.0, 0.0, DirectionMode::FORWARD};
State goal{3.0, 3.0, 0.0, DirectionMode::FORWARD};

// 规划路径
auto path = planner.plan_path(start, goal);

// 发布可视化
VisualizationPublisher viz_pub;
viz_pub.initialize();
viz_pub.publish_planning_result(start, goal, path, ...);
```

## 项目结构

```
hybrid_astar_ecal/
├── CMakeLists.txt              # 构建配置
├── README.md                   # 项目说明
├── include/                    # 头文件
│   ├── common_types.hpp        # 通用类型定义
│   ├── hybrid_astar.hpp        # 核心算法
│   ├── obstacle_map.hpp        # 障碍物地图
│   ├── vehicle_model.hpp       # 车辆模型
│   └── visualization_publisher.hpp # 可视化发布器
├── src/                        # 源文件
│   ├── common_types.cpp
│   ├── hybrid_astar.cpp
│   ├── obstacle_map.cpp
│   ├── vehicle_model.cpp
│   └── visualization_publisher.cpp
├── proto/                      # Protocol Buffers定义
│   ├── hybrid_astar.proto      # 自定义消息
│   └── foxglove/               # Foxglove标准消息
│       ├── Grid.proto
│       ├── Path.proto
│       ├── MarkerArray.proto
│       └── ...
└── examples/                   # 示例程序
    ├── demo.cpp                # 基础示例
    └── ecal_demo.cpp           # eCAL集成示例
```

## 配置参数

```cpp
struct PlanningConfig {
    // 车辆参数
    double wheelbase = 2.5;           // 轴距 (m)
    double max_steer = 0.6;           // 最大转向角 (rad)
    
    // 离散化参数
    double grid_resolution = 1.0;     // 网格分辨率 (m)
    double angle_resolution = 0.1;    // 角度分辨率 (rad)
    double steer_resolution = 0.1;    // 转向分辨率 (rad)
    
    // 运动参数
    double velocity = 2.0;            // 规划速度 (m/s)
    double simulation_time = 1.0;     // 每步仿真时间 (s)
    
    // 代价权重
    double w_steer = 1.0;             // 转向代价权重
    double w_turn = 1.0;              // 转弯代价权重
    double w_cusp = 2.0;              // 方向变化代价权重
    
    // 目标容差
    double position_tolerance = 1.0;   // 位置容差 (m)
    double angle_tolerance = 0.2;      // 角度容差 (rad)
    
    // 算法限制
    int max_iterations = 10000;       // 最大迭代次数
    bool debug_enabled = false;       // 启用调试可视化
};
```

## 故障排除

### 编译错误
- 确保C++17支持
- 检查依赖项安装
- 使用正确的CMake选项

### eCAL连接问题
- 检查eCAL服务是否运行
- 验证网络配置
- 使用 `ecal_mon` 查看topics

### Foxglove可视化问题
- 确保eCAL数据源正确配置
- 检查消息格式兼容性
- 验证topic名称

## 贡献

欢迎提交Issue和Pull Request来改进这个项目。

## 许可证

MIT License - 详见LICENSE文件

## 相关链接

- [eCAL官方文档](https://continental.github.io/ecal/)
- [Foxglove Studio](https://foxglove.dev/)
- [Protocol Buffers](https://developers.google.com/protocol-buffers)
- [Hybrid A*算法论文](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf)
