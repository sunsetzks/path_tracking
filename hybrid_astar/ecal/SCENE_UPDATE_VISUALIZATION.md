# Foxglove SceneUpdate Visualization for Hybrid A*

这个目录包含使用 Foxglove SceneUpdate proto 消息的 Hybrid A* 路径规划可视化系统。该系统设计为参考 Python `FoxgloveHybridAStarVisualizer` 类，提供实时的3D可视化能力。

## 架构概述

### 类设计

`VisualizationPublisher` 类基于 Python 版本的架构，使用分离的频道来发布不同的可视化组件：

```cpp
class VisualizationPublisher {
public:
    // 初始化和配置
    bool initialize();
    void shutdown();
    
    // 主要可视化方法
    void visualize_path_planning(...);
    
    // 设置配置
    void update_settings(const VisualizationSettings& settings);
    
private:
    // 分离的eCAL发布器
    std::unique_ptr<eCAL::protobuf::CPublisher<foxglove::SceneUpdate>> scene_pub_;
    std::unique_ptr<eCAL::protobuf::CPublisher<foxglove::SceneUpdate>> path_pub_;
    std::unique_ptr<eCAL::protobuf::CPublisher<foxglove::SceneUpdate>> exploration_pub_;
    std::unique_ptr<eCAL::protobuf::CPublisher<foxglove::SceneUpdate>> start_goal_pub_;
    std::unique_ptr<eCAL::protobuf::CPublisher<std::string>> statistics_pub_;
};
```

### 可视化频道

系统创建以下 eCAL 频道：

| 频道 | 类型 | 内容 |
|------|------|------|
| `/hybrid_astar/visualization/scene` | `foxglove::SceneUpdate` | 障碍物（立方体） |
| `/hybrid_astar/visualization/path` | `foxglove::SceneUpdate` | 最终路径（线条和箭头） |
| `/hybrid_astar/visualization/exploration` | `foxglove::SceneUpdate` | 搜索节点（球体）和模拟轨迹（线条） |
| `/hybrid_astar/visualization/start_goal` | `foxglove::SceneUpdate` | 起点和终点位置（球体和箭头） |
| `/hybrid_astar/visualization/statistics` | `std::string` | JSON格式的统计数据 |

### Foxglove 基元

系统使用以下 Foxglove 3D 基元：

- **CubePrimitive**: 表示障碍物
- **SpherePrimitive**: 表示搜索节点、起点和终点
- **LinePrimitive**: 表示路径和轨迹
- **ArrowPrimitive**: 表示车辆方向

## 使用方法

### 基本用法

```cpp
#include "visualization_publisher.hpp"

// 创建可视化发布器
VisualizationPublisher visualizer("my_planner");

// 初始化
if (!visualizer.initialize()) {
    std::cerr << "Failed to initialize visualizer" << std::endl;
    return -1;
}

// 配置设置
VisualizationPublisher::VisualizationSettings settings;
settings.path_line_thickness = 0.08;
settings.show_final_path_arrows = true;
visualizer.update_settings(settings);

// 发布完整的可视化
visualizer.visualize_path_planning(
    start_state, goal_state,
    path_nodes,           // 可选的规划结果节点
    explored_nodes,       // 搜索过程中探索的节点
    detailed_path,        // 详细的状态序列
    simulation_trajectories, // 前向模拟轨迹
    obstacle_map,         // 2D障碍物地图
    map_origin_x, map_origin_y, grid_resolution,
    planning_time_ms
);

// 关闭
visualizer.shutdown();
```

### 可视化设置

```cpp
struct VisualizationSettings {
    double path_line_thickness = 0.05;      // 路径线条粗细
    double path_alpha = 0.5;                 // 路径透明度
    int max_exploration_nodes = 100000;     // 最大搜索节点数
    double exploration_sphere_size = 0.03;   // 搜索节点球体大小
    double exploration_line_thickness = 0.01; // 轨迹线条粗细
    bool show_final_path_arrows = false;     // 是否显示路径方向箭头
    bool show_node_forward_trajectories = true; // 是否显示节点前向仿真轨迹
};
```

### 颜色编码方案

系统使用方向相关的颜色编码来区分不同的运动模式：

#### 最终路径 (Final Path)
- **绿色线条**: 前向行驶段 (DirectionMode::FORWARD)
- **红色线条**: 倒退行驶段 (DirectionMode::BACKWARD)  
- **洋红色线条**: 未知方向段 (DirectionMode::NONE)

#### 节点前向仿真轨迹 (Node Forward Trajectories)
- **绿色线条**: 前向仿真轨迹，透明度 0.3
- **红色线条**: 倒退仿真轨迹，透明度 0.3
- **灰色线条**: 未知方向轨迹，透明度 0.3

**优化说明**: 由于每条前向仿真轨迹只有一个方向，系统只检查轨迹第一个点的方向来确定整条轨迹的颜色，提高了性能和代码简洁性。

#### 方向箭头 (Direction Arrows)
- **绿色箭头**: 前向行驶方向
- **红色箭头**: 倒退行驶方向
- **橙色箭头**: 未知方向 (备用颜色)

## 示例程序

运行示例程序：

```bash
cd build
./examples/scene_update_visualization_example
```

该示例演示了：
- 创建模拟的障碍物地图
- 生成示例路径和搜索节点
- 发布完整的可视化数据
- 在 Foxglove Studio 中查看结果

## 在 Foxglove Studio 中查看

1. **启动 eCAL 系统**: 确保 eCAL 系统正在运行
2. **运行可视化程序**: 启动你的路径规划程序或示例
3. **打开 Foxglove Studio**: 连接到 eCAL 数据源
4. **添加 3D 面板**: 为每个可视化频道创建 3D 面板
5. **订阅主题**: 订阅以下主题：
   - `/hybrid_astar/visualization/scene`
   - `/hybrid_astar/visualization/path`
   - `/hybrid_astar/visualization/exploration`
   - `/hybrid_astar/visualization/start_goal`
6. **添加统计面板**: 为 JSON 统计数据添加原始消息面板

## 颜色编码

- **起点**: 绿色球体和箭头
- **终点**: 红色球体和箭头
- **障碍物**: 灰色立方体
- **最终路径**: 洋红色线条，橙色方向箭头（可选）
- **搜索节点**: 按成本着色的球体（蓝色=低成本，红色=高成本）
- **模拟轨迹**: 半透明蓝色线条

## 与 Python 版本的对比

这个 C++ 实现基于 Python `FoxgloveHybridAStarVisualizer` 类的架构：

### 相似之处
- 分离的可视化频道
- 相同的 Foxglove 基元类型
- 类似的颜色编码方案
- 统计数据的 JSON 格式

### 差异
- 使用 eCAL 而不是 WebSocket 进行传输
- C++ 类型系统和 protobuf 消息
- 内置的探索节点数量限制以提高性能
- 更严格的错误处理

## 构建要求

- eCAL with protobuf support
- Foxglove protobuf schemas
- C++17 compiler
- CMake 3.15+

## 性能注意事项

- 搜索节点数量限制为 `max_exploration_nodes` 以避免可视化过载
- 模拟轨迹与探索节点数量相匹配
- 使用高效的 protobuf 序列化
- 分离的频道允许选择性订阅

## 故障排除

### 常见问题

1. **eCAL 初始化失败**: 确保 eCAL 系统正在运行
2. **没有可视化数据**: 检查 Foxglove Studio 是否正确连接到 eCAL
3. **性能问题**: 减少 `max_exploration_nodes` 设置
4. **编译错误**: 确保所有 Foxglove protobuf 模式都可用

### 调试

启用详细日志输出来诊断问题：

```cpp
// 在初始化之前设置eCAL日志级别
eCAL::Logging::SetLogLevel(eCAL_Logging_eLogLevel::log_level_debug1);
```
