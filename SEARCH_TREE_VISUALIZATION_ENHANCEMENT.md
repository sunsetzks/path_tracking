# Hybrid A* 搜索树可视化增强

## 问题描述
原始代码中的中间搜索节点连接边看不清楚，用户无法清楚地看到搜索算法的探索过程。

## 解决方案

### 1. 增强主要可视化函数 (`_plot_main_visualization`)
- **父子连接可视化**: 添加了青色连线显示搜索树中节点之间的父子关系
- **节点可视化增强**: 
  - 使用基于F-cost的颜色映射
  - 节点边框使用白色，提高可见性
  - 增加节点大小以更好地显示
- **连接统计**: 显示绘制的连接数量

### 2. 新增详细搜索树可视化 (`visualize_detailed_search_tree`)
- **智能节点过滤**: 
  - 过滤过于密集的节点以减少视觉混乱
  - 保留重要节点（根节点、目标附近节点）
  - 可调节的节点间距参数 `node_spacing_filter`
- **增强连接显示**:
  - 彩色渐变连线显示搜索深度
  - 方向箭头显示搜索方向
  - 可控制的最大连接数 `max_connections`
- **节点重要性可视化**:
  - 变尺寸节点表示重要性
  - 颜色编码区分不同类型节点
  - 成本标签显示重要节点的F-cost值

### 3. 改进搜索进度可视化 (`visualize_search_progress`)
- **双面板显示**:
  - 左侧: 搜索树结构图
  - 右侧: 时间进度图
- **时间色彩映射**: 早期搜索=蓝色，后期搜索=红色
- **搜索方向箭头**: 显示算法的搜索方向

## 主要改进特性

### 连接可视化
```python
# 绘制父子连接
for node in self.explored_nodes:
    if node.parent is not None:
        ax.plot([node.parent.state.x, node.state.x], 
               [node.parent.state.y, node.state.y], 
               'cyan', linewidth=0.8, alpha=0.4, zorder=1)
```

### 节点重要性显示
```python
# 基于重要性的节点大小和颜色
if node.parent is None:  # 起始节点
    size = 120
    color = 'green'
elif distance_to_goal < 2.0:  # 目标附近
    size = 80
    color = 'orange'
else:  # 普通节点
    size = max(20, 60 - min(40, node.f_cost))
    color = 'lightblue'
```

### 智能过滤
```python
# 避免节点过于密集
too_close = False
for existing_node in filtered_nodes:
    if distance(node, existing_node) < node_spacing_filter:
        too_close = True
        break
```

## 使用方法

### 基本使用
```python
# 显示增强的主要可视化
planner.visualize_path(path, start, goal, 
                      show_exploration=True, 
                      show_trajectories=False, 
                      show_costs=True)

# 显示详细搜索树
planner.visualize_detailed_search_tree(path, start, goal,
                                      max_connections=800,
                                      node_spacing_filter=0.5)
```

### 参数调整建议
- **复杂场景**: `max_connections=500`, `node_spacing_filter=0.8`
- **简单场景**: `max_connections=1000`, `node_spacing_filter=0.3` 
- **调试模式**: `max_connections=1500`, `node_spacing_filter=0.2`

## 可视化效果

### 颜色编码
- 🔵 **青色连线**: 父子节点连接
- 🟢 **绿色节点**: 起始节点
- 🔴 **红色节点**: 目标节点
- 🟠 **橙色节点**: 目标附近的重要节点
- 🔵 **蓝色节点**: 普通搜索节点

### 大小编码
- **大节点**: 重要节点（起始、目标附近、低成本）
- **中等节点**: 普通搜索节点
- **小节点**: 高成本节点

### 透明度和层次
- **背景层** (zorder=0): 障碍物地图
- **连接层** (zorder=1): 父子连接线
- **节点层** (zorder=3): 搜索节点
- **路径层** (zorder=5): 最终路径
- **标记层** (zorder=10): 起始和目标标记

## 测试文件
- `test_enhanced_search_visualization.py`: 完整功能测试
- `interactive_visualization_demo.py`: 交互式参数调整演示

## 性能优化
- 节点过滤减少计算量
- 连接数量限制避免过度绘制
- 智能采样保持结构完整性

通过这些增强，用户现在可以清楚地看到：
1. 搜索算法如何从起始点扩展
2. 哪些区域被更深入地探索
3. 搜索树的分支结构
4. 算法的搜索策略和偏好方向
