# Hybrid A* 包重构说明

本项目已重构为两个独立的包，实现了核心算法与可视化集成的清晰分离。

## 包结构

### 1. hybrid_astar_core - 核心算法包

**位置**: `hybrid_astar_core/`

**功能**: 纯 C++ 实现的 Hybrid A* 路径规划算法核心

**特性**:
- ✅ 完整的 Hybrid A* 路径规划算法
- ✅ 灵活的碰撞检测接口（网格地图、几何形状、自定义检测器）
- ✅ 车辆运动学模型
- ✅ 无外部中间件依赖
- ✅ 模块化设计，易于集成

**依赖**: 仅标准 C++ 库和线程库

### 2. hybrid_astar_ecal - eCAL 集成包

**位置**: `hybrid_astar_ecal/`

**功能**: eCAL 中间件集成和 Foxglove Studio 可视化

**特性**:
- ✅ eCAL 中间件集成
- ✅ Foxglove Studio 实时可视化
- ✅ Protobuf 消息协议
- ✅ 实时规划状态发布
- ✅ 分布式系统架构支持

**依赖**: 
- `hybrid_astar_core` (核心算法)
- eCAL (中间件)
- Protobuf (消息)
- Foxglove 相关库

## 构建指南

### 1. 构建核心包

```bash
cd hybrid_astar_core
./build.sh
```

### 2. 构建 eCAL 集成包

```bash
# 确保核心包已构建
cd hybrid_astar_ecal
./build.sh
```

## 使用指南

### 核心算法使用

```cpp
#include <hybrid_astar_core/hybrid_astar.hpp>
#include <hybrid_astar_core/collision_detector.hpp>

// 创建规划器
HybridAStarConfig config;
HybridAStar planner(config);

// 设置碰撞检测器
auto detector = std::make_shared<GridCollisionDetector>(/*...*/);
planner.set_collision_detector(detector);

// 规划路径
State start(0.0, 0.0, 0.0);
State goal(10.0, 10.0, 0.0);
auto result = planner.plan_path(start, goal);
```

### eCAL 集成使用

```cpp
#include <hybrid_astar_core/hybrid_astar.hpp>
#include "visualization_publisher.hpp"

// 创建可视化发布器
VisualizationPublisher viz_pub("hybrid_astar_demo");

// 规划后发布可视化
if (result.has_value()) {
    viz_pub.publish_planning_result(/*...*/);
}
```

## 重构优势

### 1. 清晰的职责分离
- **核心包**: 专注算法实现，无可视化依赖
- **eCAL 包**: 专注可视化和中间件集成

### 2. 降低耦合度
- 核心算法可独立使用
- 可视化功能可选加载
- 更容易维护和测试

### 3. 便于集成
- 其他项目可仅依赖核心包
- 支持多种可视化方案
- 模块化架构便于扩展

### 4. 构建优化
- 核心包构建快速，无重依赖
- eCAL 包仅在需要可视化时构建
- 减少不必要的依赖传播

## 迁移指南

### 从原 hybrid_astar_ecal 迁移

**之前**:
```cpp
#include "hybrid_astar.hpp"
// 所有功能都在一个包中
```

**现在**:
```cpp
// 仅使用核心算法
#include <hybrid_astar_core/hybrid_astar.hpp>

// 或者同时使用可视化
#include <hybrid_astar_core/hybrid_astar.hpp>
#include "visualization_publisher.hpp"
```

**API 变化**:
- `planner.plan()` → `planner.plan_path()`
- 返回类型从自定义结构体变为 `std::optional<std::vector<std::shared_ptr<Node>>>`

## 示例

### 核心包示例

参见 `hybrid_astar_core/examples/`:
- `basic_demo.cpp` - 基础算法使用
- `collision_detector_example.cpp` - 碰撞检测示例

### eCAL 集成示例

参见 `hybrid_astar_ecal/examples/`:
- `ecal_demo.cpp` - 完整的 eCAL 集成示例

## 测试

### 核心包测试

```bash
cd hybrid_astar_core/build
ctest --verbose
```

### eCAL 包测试

```bash
cd hybrid_astar_ecal/build
# 运行示例以验证集成
./examples/ecal_demo
```

## 开发建议

1. **新功能开发**: 优先在核心包中实现算法逻辑
2. **可视化功能**: 在 eCAL 包中扩展
3. **测试策略**: 核心算法单独测试，集成功能端到端测试
4. **版本管理**: 两个包可独立版本管理

## 未来扩展

这种架构支持：
- 添加其他可视化后端（ROS、直接 OpenGL 等）
- 集成其他中间件（ROS2、DDS 等）
- 支持多种编程语言绑定（Python、Java 等）
- 独立的性能优化和算法改进
