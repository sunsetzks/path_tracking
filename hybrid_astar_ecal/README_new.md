# Hybrid A* eCAL Integration

这是 Hybrid A* 路径规划算法的 eCAL 中间件集成包，提供实时可视化和分布式系统支持。

## 功能特性

- ✅ eCAL 中间件集成支持
- ✅ Foxglove Studio 实时可视化
- ✅ Protobuf 消息协议
- ✅ 实时规划状态发布
- ✅ 完整的可视化数据流
- ✅ 分布式系统架构支持

## 依赖关系

这个包依赖于 `hybrid_astar_core` 核心算法包：

```
hybrid_astar_ecal
├── hybrid_astar_core (核心算法)
├── eCAL (中间件)
├── Protobuf (消息)
└── Foxglove (可视化)
```

## 安装依赖

### 1. 安装 eCAL

```bash
# Ubuntu/Debian
sudo apt update
sudo apt install ecal-dev

# 或者从源码编译
git clone https://github.com/eclipse-ecal/ecal.git
cd ecal
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
```

### 2. 安装 Protobuf

```bash
sudo apt install protobuf-compiler libprotobuf-dev
```

### 3. 构建核心包

```bash
cd ../hybrid_astar_core
./build.sh
```

## 编译

```bash
# 确保核心包已经构建
cd hybrid_astar_ecal
./build.sh
```

## 使用示例

### 基本 eCAL 集成

```cpp
#include <hybrid_astar_core/hybrid_astar.hpp>
#include <hybrid_astar_core/collision_detector.hpp>
#include "visualization_publisher.hpp"

// 创建规划器
HybridAStarConfig config;
HybridAStar planner(config);

// 创建可视化发布器
VisualizationPublisher viz_pub("hybrid_astar_demo");

// 设置碰撞检测器
auto detector = std::make_shared<GridCollisionDetector>(/*...*/);
planner.set_collision_detector(detector);

// 规划和发布
Pose2D start{0.0, 0.0, 0.0};
Pose2D goal{10.0, 10.0, 0.0};

auto result = planner.plan(start, goal);
if (result.success) {
    // 发布可视化数据
    viz_pub.publish_planning_result(
        start, goal, 
        result.path_nodes,
        result.explored_nodes,
        result.path,
        result.simulation_trajectories
    );
}
```

## 架构设计

### 包结构

```
hybrid_astar_ecal/
├── include/
│   └── visualization_publisher.hpp    # eCAL 可视化发布器
├── src/
│   └── visualization_publisher.cpp    # 实现
├── proto/                             # Protobuf 消息定义
├── examples/
│   └── ecal_demo.cpp                  # eCAL 集成示例
├── config/                            # 配置文件
└── CMakeLists.txt
```

### 消息流

```
HybridAStar -> VisualizationPublisher -> eCAL -> Foxglove Studio
     ↑              ↓
CollisionDetector  JSON统计
```

## eCAL 话题

| 话题名称 | 消息类型 | 描述 |
|---------|---------|------|
| `/hybrid_astar/scene_update` | `foxglove.SceneUpdate` | 可视化场景更新 |
| `/hybrid_astar/obstacles` | `foxglove.SceneUpdate` | 障碍物显示 |
| `/hybrid_astar/path_update` | `foxglove.SceneUpdate` | 路径更新 |
| `/hybrid_astar/statistics` | `std::string` | JSON 格式统计信息 |

## Foxglove Studio 设置

1. 启动 Foxglove Studio
2. 连接到 eCAL 数据源
3. 添加 3D 面板
4. 订阅相关话题：
   - `/hybrid_astar/scene_update`
   - `/hybrid_astar/obstacles` 
   - `/hybrid_astar/path_update`

## 示例运行

```bash
# 终端 1：启动 eCAL 监控
ecal_mon

# 终端 2：运行 Foxglove Studio
foxglove-studio

# 终端 3：运行示例
cd build/examples
./ecal_demo
```

## 配置

可以通过配置文件或环境变量自定义：

- eCAL 网络设置
- 话题名称
- 发布频率
- 可视化样式

参见 `config/` 目录下的配置文件。

## 与核心包的集成

这个包设计为核心算法的可视化层：

- 核心算法在 `hybrid_astar_core` 中实现
- 本包仅处理可视化和通信
- 完全解耦的设计便于维护和测试

## 开发指南

### 添加新的可视化功能

1. 在 `visualization_publisher.hpp` 中添加新方法
2. 实现相应的 Protobuf 消息创建
3. 更新示例代码
4. 添加相应的测试

### 扩展消息类型

1. 在 `proto/` 目录添加新的 .proto 文件
2. 更新 `proto/CMakeLists.txt`
3. 在可视化发布器中集成新消息
4. 更新文档
