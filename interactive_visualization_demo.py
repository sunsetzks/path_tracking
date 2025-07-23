#!/usr/bin/env python3
"""
Interactive visualization demo with adjustable parameters
用户可以调整可视化参数以获得最佳显示效果
"""

import numpy as np
import sys
import matplotlib.pyplot as plt

# Add the astar_project module to path
sys.path.append('/home/zks/ws/path_tracking/astar_project')

from astar_project.hybrid_astar import HybridAStar, VehicleModel, State, DirectionMode

def create_demo_planner_with_solution():
    """创建一个已经有解的规划器实例"""
    
    # Create vehicle model
    vehicle = VehicleModel(wheelbase=2.5, max_steer=np.pi/4)
    
    # Create planner
    planner = HybridAStar(
        vehicle_model=vehicle,
        grid_resolution=1.0,
        angle_resolution=np.pi/8,
        velocity=2.0,
        simulation_time=0.6,
        dt=0.1
    )
    
    # Create moderate obstacle map
    map_size = 25
    obstacle_map = np.zeros((map_size, map_size))
    
    # Add some interesting obstacles
    obstacle_map[10:15, 8:12] = 1   # Rectangle
    obstacle_map[18:22, 15:20] = 1  # Another rectangle
    obstacle_map[5:8, 18:22] = 1    # Third rectangle
    
    planner.set_obstacle_map(obstacle_map, origin_x=0, origin_y=0)
    
    # Define states
    start = State(x=2.0, y=2.0, yaw=np.pi/4, direction=DirectionMode.FORWARD)
    goal = State(x=20.0, y=18.0, yaw=np.pi/3, direction=DirectionMode.FORWARD)
    
    # Plan path
    path = planner.plan_path(start, goal, max_iterations=2000)
    
    return planner, path, start, goal

def demo_visualization_options():
    """演示不同的可视化选项"""
    
    print("="*60)
    print("INTERACTIVE HYBRID A* VISUALIZATION DEMO")
    print("="*60)
    print("正在生成示例场景...")
    
    # Get demo scenario
    planner, path, start, goal = create_demo_planner_with_solution()
    
    if not path:
        print("路径规划失败，无法演示")
        return
    
    print(f"路径规划成功！找到路径包含 {len(path)} 个航点")
    print(f"搜索了 {len(planner.explored_nodes)} 个节点")
    
    # Demo different visualization options
    options = {
        "1": ("标准可视化 - 显示所有连接", {
            'max_connections': 1000,
            'node_spacing_filter': 0.3
        }),
        "2": ("清晰可视化 - 过滤密集连接", {
            'max_connections': 300, 
            'node_spacing_filter': 0.8
        }),
        "3": ("详细可视化 - 显示更多连接", {
            'max_connections': 1500,
            'node_spacing_filter': 0.2
        }),
        "4": ("简洁可视化 - 最少连接", {
            'max_connections': 150,
            'node_spacing_filter': 1.2
        })
    }
    
    print("\n可用的可视化选项:")
    for key, (desc, _) in options.items():
        print(f"  {key}. {desc}")
    
    print("\n正在依次显示所有选项...")
    
    for key, (description, params) in options.items():
        print(f"\n显示选项 {key}: {description}")
        print(f"参数: max_connections={params['max_connections']}, node_spacing_filter={params['node_spacing_filter']}")
        
        # Show visualization with current parameters
        planner.visualize_detailed_search_tree(
            path, start, goal,
            max_connections=params['max_connections'],
            node_spacing_filter=params['node_spacing_filter']
        )
        
        # Wait for user
        input("按回车键继续到下一个可视化选项...")

def demo_search_progression():
    """演示搜索进度的可视化"""
    
    print("\n" + "="*50)
    print("搜索进度可视化演示")
    print("="*50)
    
    planner, path, start, goal = create_demo_planner_with_solution()
    
    if path:
        print("显示搜索进度的双面板可视化...")
        planner.visualize_search_progress(path, start, goal, max_nodes_to_show=400)
    
def demo_main_visualization():
    """演示主要路径可视化"""
    
    print("\n" + "="*50)
    print("主要路径可视化演示")
    print("="*50)
    
    planner, path, start, goal = create_demo_planner_with_solution()
    
    if path:
        print("显示带有增强搜索连接的主要可视化...")
        planner.visualize_path(path, start, goal,
                              show_exploration=True,
                              show_trajectories=False,
                              show_costs=True)

def compare_before_after():
    """对比增强前后的效果"""
    
    print("\n" + "="*50)
    print("可视化增强对比")
    print("="*50)
    
    planner, path, start, goal = create_demo_planner_with_solution()
    
    if not path:
        return
    
    print("创建对比可视化...")
    
    # Create side-by-side comparison
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(20, 8))
    
    # Left: Original style (minimal connections)
    planner._plot_search_tree(ax1, path, start, goal, max_nodes_to_show=200)
    ax1.set_title("原始可视化\n(连接较少，节点较小)", fontsize=14)
    
    # Right: Enhanced style (more connections and better visibility)
    # Temporarily modify the planner for enhanced visualization
    filtered_nodes = planner.explored_nodes[::2]  # Every 2nd node
    
    # Plot enhanced version
    if planner.obstacle_map is not None:
        extent = (planner.map_origin_x, 
                 planner.map_origin_x + planner.map_width * planner.grid_resolution,
                 planner.map_origin_y,
                 planner.map_origin_y + planner.map_height * planner.grid_resolution)
        ax2.imshow(planner.obstacle_map, extent=extent, origin='lower', 
                  cmap='gray_r', alpha=0.7)
    
    # Draw enhanced connections
    connection_count = 0
    for node in filtered_nodes[:300]:  # Limit for clarity
        if node.parent is not None and connection_count < 200:
            # Enhanced connection style
            depth_color = min(1.0, connection_count / 200.0)
            color = plt.cm.get_cmap('plasma')(depth_color)
            ax2.plot([node.parent.state.x, node.state.x], 
                    [node.parent.state.y, node.state.y], 
                    color=color, linewidth=2, alpha=0.8)
            connection_count += 1
    
    # Enhanced nodes
    node_x = [n.state.x for n in filtered_nodes]
    node_y = [n.state.y for n in filtered_nodes] 
    node_costs = [n.f_cost for n in filtered_nodes]
    
    scatter = ax2.scatter(node_x, node_y, c=node_costs, cmap='viridis_r',
                         s=40, alpha=0.9, edgecolors='white', linewidths=1)
    
    # Path and markers
    if path:
        path_x = [s.x for s in path]
        path_y = [s.y for s in path]
        ax2.plot(path_x, path_y, 'red', linewidth=4, alpha=0.9)
    
    ax2.plot(start.x, start.y, 'go', markersize=15, markeredgecolor='white', markeredgewidth=2)
    ax2.plot(goal.x, goal.y, 'ro', markersize=15, markeredgecolor='white', markeredgewidth=2)
    
    ax2.set_title("增强可视化\n(更多连接，更清晰的节点)", fontsize=14)
    ax2.grid(True, alpha=0.3)
    ax2.axis('equal')
    
    plt.tight_layout()
    plt.show()
    
    print("对比显示完成！左侧是原始效果，右侧是增强效果。")

def print_usage_tips():
    """打印使用建议"""
    
    print("\n" + "="*60)
    print("可视化使用建议")
    print("="*60)
    
    tips = [
        "1. max_connections 参数控制显示的连接数量",
        "   - 较大值(>800): 显示更完整的搜索树",
        "   - 较小值(<300): 减少视觉混乱",
        "",
        "2. node_spacing_filter 参数控制节点密度",
        "   - 较大值(>0.8): 只显示间距较远的节点", 
        "   - 较小值(<0.3): 显示更密集的节点",
        "",
        "3. 建议的参数组合:",
        "   - 复杂场景: max_connections=500, node_spacing_filter=0.8",
        "   - 简单场景: max_connections=1000, node_spacing_filter=0.3",
        "   - 调试模式: max_connections=1500, node_spacing_filter=0.2",
        "",
        "4. 可视化特性:",
        "   - 青色线条: 父子节点连接",
        "   - 颜色渐变: 搜索深度或成本",
        "   - 节点大小: 重要性或成本",
        "   - 箭头: 搜索方向"
    ]
    
    for tip in tips:
        print(f"  {tip}")

if __name__ == "__main__":
    try:
        # Run main demo
        demo_visualization_options()
        
        # Show search progression
        demo_search_progression() 
        
        # Show main visualization
        demo_main_visualization()
        
        # Show before/after comparison
        compare_before_after()
        
        # Print usage tips
        print_usage_tips()
        
        print("\n🎯 交互式可视化演示完成!")
        print("现在您可以看到清晰的搜索树连接和中间节点了！")
        
    except KeyboardInterrupt:
        print("\n演示被用户中断")
    except Exception as e:
        print(f"\n演示出现错误: {e}")
        import traceback
        traceback.print_exc()
