import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt

# 设置中文字体 - 尝试使用系统可用的中文字体
plt.rcParams['font.sans-serif'] = ['Heiti TC', 'Heiti SC', 'PingFang SC', 'Arial Unicode MS', 'DejaVu Sans']
plt.rcParams['axes.unicode_minus'] = False

# 定义变量
x = cp.Variable(2)
y = cp.Variable(2)

# 目标函数 - 修改为DCP兼容的形式
objective = 0.5 * (cp.sum_squares(x) + cp.sum_squares(y)) + cp.quad_form(x, np.eye(2)) * 0.1

# 约束条件
constraints = [
    cp.sum(x) <= 2,
    cp.sum(y) <= 2, 
    cp.abs(x - y) <= 2  # 元素级的绝对值约束
]

# 求解
problem = cp.Problem(cp.Minimize(objective), constraints)
result = problem.solve()

# 输出结果
print("=" * 50)
print("凸优化问题求解结果")
print("=" * 50)
print(f"状态: {problem.status}")
print(f"最优值: {problem.value:.6f}")
print(f"最优解:")
print(f"x = [{x.value[0]:.6f}, {x.value[1]:.6f}]")
print(f"y = [{y.value[0]:.6f}, {y.value[1]:.6f}]")

# 可视化
plt.figure(figsize=(10, 8))

# 绘制可行区域
xx, yy = np.meshgrid(np.linspace(-1, 3, 100), np.linspace(-1, 3, 100))
feasible = (xx + yy <= 2)
plt.contourf(xx, yy, feasible, levels=[0.5, 1], alpha=0.2, colors='blue')

# 绘制最优解
plt.plot(x.value[0], x.value[1], 'ro', markersize=10, label=f'x* = ({x.value[0]:.2f}, {x.value[1]:.2f})')
plt.plot(y.value[0], y.value[1], 'go', markersize=10, label=f'y* = ({y.value[0]:.2f}, {y.value[1]:.2f})')

# 连接两点
plt.plot([x.value[0], y.value[0]], [x.value[1], y.value[1]], 'k--', alpha=0.5, label='连接线')

# 绘制约束边界
boundary_x = np.linspace(-1, 3, 100)
boundary_y = 2 - boundary_x
plt.plot(boundary_x, boundary_y, 'b-', linewidth=2, label='约束边界 x1+x2=2')

plt.xlim(-1, 3)
plt.ylim(-1, 3)
plt.grid(True, alpha=0.3)
plt.axhline(0, color='black', linewidth=0.5)
plt.axvline(0, color='black', linewidth=0.5)
plt.xlabel('x1 / y1')
plt.ylabel('x2 / y2')
plt.title('Convex Optimization Problem Solution Visualization')
plt.legend()
plt.axis('equal')
plt.savefig('optimization_result.png', dpi=150, bbox_inches='tight')
print("Plot saved as 'optimization_result.png'")

# 约束满足情况检查
print("\n约束满足检查:")
print(f"x1 + x2 = {x.value[0] + x.value[1]:.6f} <= 2: {x.value[0] + x.value[1] <= 2}")
print(f"y1 + y2 = {y.value[0] + y.value[1]:.6f} <= 2: {y.value[0] + y.value[1] <= 2}")
print(f"|x1 - y1| = {abs(x.value[0] - y.value[0]):.6f} <= 2: {abs(x.value[0] - y.value[0]) <= 2}")
print(f"|x2 - y2| = {abs(x.value[1] - y.value[1]):.6f} <= 2: {abs(x.value[1] - y.value[1]) <= 2}")