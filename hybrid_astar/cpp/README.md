# Hybrid A* Core

这是 Hybrid A* 路径规划算法的核心实现，不包含任何可视化或中间件依赖。

## 功能特性

- ✅ 完整的 Hybrid A* 路径规划算法
- ✅ 灵活的碰撞检测接口
- ✅ 支持网格地图、几何形状和自定义检测器
- ✅ 纯 C++ 实现，无外部中间件依赖
- ✅ 模块化设计，易于集成

## 组件

### 核心算法
- `HybridAStar` - 主要算法类
- `VehicleModel` - 车辆运动学模型
- `CommonTypes` - 基础数据类型

### 碰撞检测
- `CollisionDetector` - 碰撞检测抽象接口
- `GridCollisionDetector` - 网格地图碰撞检测
- `ObstacleMap` - 障碍物地图管理

## 快速开始

### 编译

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
```

### 基本使用

```cpp
#include "hybrid_astar.hpp"
#include "collision_detector.hpp"

// 创建规划器
HybridAStarConfig config;
config.xy_resolution = 1.0;
config.yaw_resolution = 15.0; // degrees
config.max_steer_angle = 30.0; // degrees

HybridAStar planner(config);

// 设置碰撞检测器
auto grid_detector = std::make_shared<GridCollisionDetector>(
    obstacle_map, grid_resolution, origin_x, origin_y);
planner.set_collision_detector(grid_detector);

// 规划路径
Pose2D start{0.0, 0.0, 0.0};
Pose2D goal{10.0, 10.0, 0.0};

auto result = planner.plan(start, goal);
if (result.success) {
    // 使用路径
    for (const auto& pose : result.path) {
        std::cout << "(" << pose.x << ", " << pose.y << ", " << pose.theta << ")" << std::endl;
    }
}
```

## 架构设计

这个核心包专注于算法实现，不包含：
- 可视化组件
- eCAL 集成
- Protobuf 消息
- 网络通信

如需这些功能，请使用 `hybrid_astar_ecal` 包。

## API 文档

### HybridAStar 类

主要的路径规划算法实现。

#### 构造函数
```cpp
HybridAStar(const HybridAStarConfig& config)
```

#### 主要方法
```cpp
PlanningResult plan(const Pose2D& start, const Pose2D& goal)
void set_collision_detector(std::shared_ptr<CollisionDetector> detector)
```

### CollisionDetector 接口

碰撞检测的抽象基类。

```cpp
class CollisionDetector {
public:
    virtual bool is_collision(const Pose2D& pose) const = 0;
    virtual bool is_collision(const std::vector<Pose2D>& footprint) const = 0;
};
```

## 示例

查看 `examples/` 目录获取完整的使用示例：

- `basic_demo.cpp` - 基础使用示例
- `collision_detector_example.cpp` - 碰撞检测示例

## 测试

```bash
# 运行测试
cd build
ctest --verbose
```

## 集成到其他项目

### 使用 CMake

```cmake
find_package(hybrid_astar_core REQUIRED)
target_link_libraries(your_target hybrid_astar_core::hybrid_astar_core)
```

### 使用 pkg-config

```bash
pkg-config --cflags --libs hybrid_astar_core
```
