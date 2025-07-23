# 交互式路径规划可视化

该增强版本的 Hybrid A* 可视化工具提供了交互式控制功能，让用户可以动态地显示和隐藏不同的可视化元素。

## 新增功能

### 交互式控制面板
- **Show Exploration Nodes**: 显示/隐藏搜索树中的探索节点
- **Show Trajectories**: 显示/隐藏前向仿真轨迹
- **Show Final Path**: 显示/隐藏最终规划路径
- **Show Vehicle Arrows**: 显示/隐藏车辆朝向箭头（蓝色）
- **Show Steering Arrows**: 显示/隐藏转向箭头（红色）
- **Show Waypoint Numbers**: 显示/隐藏路径点编号
- **Show Cost Panel**: 显示/隐藏成本分析面板

### 使用方法

```python
from astar_project.visualizer import HybridAStarVisualizer

# 创建可视化器
visualizer = HybridAStarVisualizer()

# 调用可视化方法
visualizer.visualize_path(
    path=path,
    start=start,
    goal=goal,
    explored_nodes=explored_nodes,
    simulation_trajectories=simulation_trajectories,
    obstacle_map=obstacle_map,
    show_costs=False  # 默认隐藏成本面板
)
```

### 控制说明

1. **实时切换**: 点击右侧复选框即可实时切换显示元素
2. **成本面板**: 勾选"Show Cost Panel"会重新布局界面，显示路径成本分析
3. **分离控制**: 车辆箭头和转向箭头可以分别控制显示
4. **节点过滤**: 探索节点会自动过滤以提高可视化性能

### 可视化元素说明

- **青色曲线**: 搜索树连接（前向仿真）
- **绿色轨迹 >**: 前向探索轨迹
- **红色轨迹 <**: 后向探索轨迹  
- **黑色矩形**: 车辆轮廓
- **蓝色箭头**: 车辆朝向方向
- **红色箭头**: 前轮转向方向

### 技术实现

- 使用 matplotlib 的 CheckButtons 小部件
- 实时重绘机制，保持交互性能
- 模块化设计，易于扩展新的控制功能
- 智能布局管理，支持成本面板的动态显示/隐藏

### 性能优化

- 轨迹采样以减少视觉混乱
- 节点过滤以提高渲染性能
- 按需重绘机制，只更新必要的元素
