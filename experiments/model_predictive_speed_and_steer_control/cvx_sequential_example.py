import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt
import warnings

# 忽略各种警告
warnings.filterwarnings('ignore', message='Unable to import Axes3D')
warnings.filterwarnings('ignore', message='.*3D.*')
warnings.filterwarnings('ignore', category=RuntimeWarning)
warnings.filterwarnings('ignore', category=UserWarning)
warnings.filterwarnings('ignore', message='Glyph.*missing from font')
warnings.filterwarnings('ignore', module='matplotlib.projections')

# 设置中文字体
plt.rcParams['font.sans-serif'] = ['Noto Sans CJK SC', 'AR PL UMing CN', 'Droid Sans Fallback']
plt.rcParams['axes.unicode_minus'] = False

# 定义参数
n_steps = 8  # 时间步数
initial_x0 = np.array([1.0, 1.0])  # 初始状态 x0

# 状态转移矩阵 A 和 B，以及偏置 b
A = np.eye(2)  # 状态转移矩阵
B = np.eye(2)  # 控制输入矩阵
b = np.array([0.2, 0.1])  # 偏置项


# 定义变量序列 x[0], x[1], ..., x[n_steps] 和控制输入 u[0], u[1], ..., u[n_steps-1]
x = cp.Variable((n_steps + 1, 2))  # 包含初始状态，共 n_steps+1 个状态
u = cp.Variable((n_steps, 2))  # 控制输入序列，共 n_steps 个控制量

# 目标函数 - 最小化所有状态的二次型 + 控制输入的代价
objective = 0
for i in range(n_steps + 1):
    # 状态代价项
    objective += 0.5 * cp.sum_squares(x[i] - np.array([2.0, 1.5]))  # 目标是达到 (2.0, 1.5)

# 控制输入代价项
for i in range(n_steps):
    objective += 1 * cp.sum_squares(u[i])  # 控制输入代价，权重较小

# 约束条件
constraints = []

# 初始状态约束
constraints.append(x[0] == initial_x0)

# 状态转移约束 x[i+1] = A * x[i] + B * u[i] + b
for i in range(n_steps):
    constraints.append(x[i+1] == A @ x[i] + B @ u[i] + b)

# 状态边界约束
for i in range(n_steps + 1):
    constraints.append(x[i] >= 0)  # 状态非负
    constraints.append(x[i] <= 5)  # 状态上界

# 控制输入边界约束
for i in range(n_steps):
    constraints.append(u[i] >= -2.0)  # 控制输入下界
    constraints.append(u[i] <= 2.0)  # 控制输入上界

# 求解
problem = cp.Problem(cp.Minimize(objective), constraints)
result = problem.solve()

# 检查求解是否成功
if problem.status not in ['optimal', 'optimal_inaccurate']:
    print(f"求解失败！状态: {problem.status}")
    exit(1)

# 输出结果
print("=" * 80)
print("带有控制输入的连续线性约束凸优化问题求解结果")
print("=" * 80)
print(f"状态: {problem.status}")
print(f"最优值: {problem.value:.6f}")
print(f"初始状态: x[0] = [{initial_x0[0]:.2f}, {initial_x0[1]:.2f}]")
print(f"状态转移矩阵 A:\n{A}")
print(f"控制输入矩阵 B:\n{B}")
print(f"偏置项 b: {b}")

# 获取结果轨迹和控制输入
trajectory = x.value
control_inputs = u.value
if trajectory is None or control_inputs is None:
    print("无法获取结果！")
    exit(1)

print("\n📍 状态轨迹:")
for i in range(n_steps + 1):
    print(f"x[{i}] = [{trajectory[i][0]:.4f}, {trajectory[i][1]:.4f}]")

print("\n🎮 控制输入:")
for i in range(n_steps):
    print(f"u[{i}] = [{control_inputs[i][0]:.4f}, {control_inputs[i][1]:.4f}]")

# 可视化
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 7))

# 左图：状态空间轨迹
ax1.set_xlim(-0.5, 5)
ax1.set_ylim(-0.5, 3)
ax1.grid(True, alpha=0.3)
ax1.set_xlabel('x1')
ax1.set_ylabel('x2')
ax1.set_title('状态空间轨迹')

# 绘制状态转移轨迹
trajectory = x.value

# 绘制转移箭头
for i in range(n_steps):
    start = trajectory[i]
    end = trajectory[i+1]
    ax1.arrow(start[0], start[1], end[0]-start[0], end[1]-start[1],
              head_width=0.08, head_length=0.06, fc='blue', ec='blue', alpha=0.6)

# 绘制状态点
ax1.plot(trajectory[:, 0], trajectory[:, 1], 'o', linewidth=2, markersize=8,
         color='blue', alpha=0.7, label='状态点')

# 特别标记初始和最终状态
ax1.plot(trajectory[0, 0], trajectory[0, 1], 'go', markersize=12,
         label=f'初始状态 x[0] = ({trajectory[0,0]:.2f}, {trajectory[0,1]:.2f})')
ax1.plot(trajectory[-1, 0], trajectory[-1, 1], 'ro', markersize=12,
         label=f'最终状态 x[{n_steps}] = ({trajectory[-1,0]:.2f}, {trajectory[-1,1]:.2f})')

# 绘制目标点
ax1.plot(2.0, 1.5, 'r*', markersize=20, label=f'目标状态 (2.0, 1.5)')

# 添加状态标注
for i in range(len(trajectory)):
    offset_x, offset_y = (0.15, 0.15) if i % 2 == 0 else (-0.25, 0.15)
    # ax1.annotate(f'x[{i}]', (trajectory[i, 0], trajectory[i, 1]),
  #               xytext=(offset_x, offset_y), textcoords='offset points',
  #               fontsize=9, fontweight='bold',
  #               # bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7)
  #               )

# 绘制约束边界
ax1.axhline(y=0, color='red', linestyle='--', alpha=0.5, linewidth=1)
ax1.axhline(y=5, color='red', linestyle='--', alpha=0.5, linewidth=1)
ax1.axvline(x=0, color='red', linestyle='--', alpha=0.5, linewidth=1)
ax1.axvline(x=5, color='red', linestyle='--', alpha=0.5, linewidth=1)

# 添加约束区域填充
ax1.fill_between([0, 5], [0, 0], [5, 5], alpha=0.1, color='gray', label='可行域')

ax1.legend(loc='upper left', fontsize=9)
ax1.axis('equal')

# 右图：控制输入时间序列图
time_points_control = np.arange(n_steps)
ax2.plot(time_points_control, control_inputs[:, 0], 'g-^', linewidth=2, markersize=6, label='u1 分量')
ax2.plot(time_points_control, control_inputs[:, 1], 'm-d', linewidth=2, markersize=6, label='u2 分量')
ax2.axhline(y=0, color='black', linestyle='-', alpha=0.3, linewidth=0.5)
ax2.axhline(y=2.0, color='red', linestyle='--', alpha=0.5, label='控制上界')
ax2.axhline(y=-2.0, color='red', linestyle='--', alpha=0.5, label='控制下界')
ax2.set_xlabel('时间步')
ax2.set_ylabel('控制输入值')
ax2.set_title('控制输入随时间变化')
ax2.grid(True, alpha=0.3)
ax2.legend()
ax2.set_xlim(-0.5, n_steps - 0.5)

# 添加第三个图：状态转移向量可视化
fig3, ax3 = plt.subplots(figsize=(10, 8))

# 绘制状态转移向量
for i in range(n_steps):
    start = trajectory[i]
    end = trajectory[i+1]

    # 绘制向量箭头
    ax3.arrow(start[0], start[1], end[0]-start[0], end[1]-start[1],
              head_width=0.08, head_length=0.06, fc='blue', ec='blue',
              alpha=0.6, linewidth=2,
              label=f'转移 {i}→{i+1}' if i == 0 else '')

# 绘制所有状态点
for i in range(n_steps + 1):
    if i == 0:
        color, marker, size = 'green', 'o', 15
        label = f'x[0] (初始)'
    elif i == n_steps:
        color, marker, size = 'red', 's', 15
        label = f'x[{n_steps}] (最终)'
    else:
        color, marker, size = 'blue', 'o', 10
        label = f'x[{i}]' if i <= 3 else None

    ax3.plot(trajectory[i, 0], trajectory[i, 1], marker=marker,
             markersize=size, color=color, label=label)

# 绘制目标点
ax3.plot(2.0, 1.5, 'r*', markersize=20, label='目标 (2.0, 1.5)')

# 添加状态标注（带背景框）
for i in range(n_steps + 1):
    offset_x, offset_y = (0.2, 0.2) if i % 2 == 0 else (-0.4, 0.2)
    # ax3.annotate(f'x[{i}]\n({trajectory[i,0]:.2f}, {trajectory[i,1]:.2f})',
  #               (trajectory[i, 0], trajectory[i, 1]),
  #               xytext=(offset_x, offset_y), textcoords='offset points',
  #               fontsize=9, fontweight='bold',
  #               # bbox=dict(boxstyle='round,pad=0.3', facecolor='yellow', alpha=0.7),
  #               arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'))

# 绘制约束边界
ax3.axhline(y=0, color='red', linestyle='--', alpha=0.5, linewidth=1, label='约束边界')
ax3.axhline(y=5, color='red', linestyle='--', alpha=0.5, linewidth=1)
ax3.axvline(x=0, color='red', linestyle='--', alpha=0.5, linewidth=1)
ax3.axvline(x=5, color='red', linestyle='--', alpha=0.5, linewidth=1)

# 添加约束区域填充
ax3.fill_between([0, 5], [0, 0], [5, 5], alpha=0.1, color='gray')

ax3.set_xlim(-0.5, 5)
ax3.set_ylim(-0.5, 3)
ax3.grid(True, alpha=0.3)
ax3.set_xlabel('x1', fontsize=12)
ax3.set_ylabel('x2', fontsize=12)
ax3.set_title('状态转移向量图', fontsize=14, fontweight='bold')
ax3.legend(loc='upper left', fontsize=9)
ax3.axis('equal')

# 保存第三张图
plt.savefig('sequential_optimization_vectors.png', dpi=150, bbox_inches='tight')

# 添加第四个图：代价分解分析
target_state = np.array([2.0, 1.5])
fig4, (ax4_1, ax4_2) = plt.subplots(2, 1, figsize=(12, 10))

# 上图：累积代价分解
time_points = np.arange(n_steps + 1)
cumulative_state_costs = []
cumulative_control_costs = []
cumulative_state = 0.0
cumulative_control = 0.0

# 初始状态代价
if trajectory is not None and len(trajectory) > 0:
    state_error = trajectory[0] - target_state
    cumulative_state = 0.5 * np.dot(state_error, state_error)
    cumulative_state_costs.append(cumulative_state)
    cumulative_control_costs.append(cumulative_control)

    for i in range(n_steps):
        # 状态代价
        if i+1 < len(trajectory):
            state_error = trajectory[i+1] - target_state
            cumulative_state += 0.5 * np.dot(state_error, state_error)
            cumulative_state_costs.append(cumulative_state)

            # 控制代价
            if control_inputs is not None and i < len(control_inputs):
                control = control_inputs[i]
                cumulative_control += 0.1 * np.dot(control, control)
                cumulative_control_costs.append(cumulative_control)

    ax4_1.fill_between(time_points, 0, cumulative_state_costs, alpha=0.7, color='blue', label='累积状态代价')
    ax4_1.fill_between(time_points, cumulative_state_costs,
                        [sc + cc for sc, cc in zip(cumulative_state_costs, cumulative_control_costs)],
                        alpha=0.7, color='green', label='累积控制代价')
    ax4_1.plot(time_points, [sc + cc for sc, cc in zip(cumulative_state_costs, cumulative_control_costs)],
              'r-', linewidth=2, label='总累积代价')
    ax4_1.set_xlabel('时间步')
    ax4_1.set_ylabel('累积代价')
    ax4_1.set_title('累积代价分解')
    ax4_1.legend()
    ax4_1.grid(True, alpha=0.3)

    # 下图：状态距离和控制量
    state_distances = [np.linalg.norm(trajectory[i] - target_state) for i in range(len(trajectory))]
    control_norms = [np.linalg.norm(control_inputs[i]) for i in range(len(control_inputs))] if control_inputs is not None else []

    ax4_2.plot(time_points, state_distances, 'b-o', linewidth=2, markersize=6, label='状态距离')
    ax4_2.plot(np.arange(len(control_norms)), control_norms, 'r-^', linewidth=2, markersize=6, label='控制量大小')
    ax4_2.set_xlabel('时间步')
    ax4_2.set_ylabel('距离 / 控制量')
    ax4_2.set_title('状态距离和控制量随时间变化')
    ax4_2.legend()
    ax4_2.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('sequential_optimization_comprehensive.png', dpi=150, bbox_inches='tight')

plt.tight_layout()
plt.savefig('sequential_optimization_result.png', dpi=150, bbox_inches='tight')
print("\n图表已保存为 'sequential_optimization_result.png', 'sequential_optimization_vectors.png' 和 'sequential_optimization_comprehensive.png'")

# 验证状态转移约束
print("\n🔍 状态转移约束验证:")
print("步骤 | 实际x[i+1] | 预测A*x[i]+B*u[i]+b | 误差 | 控制量u[i]")
print("-" * 70)
max_error = 0
for i in range(n_steps):
    expected = A @ trajectory[i] + B @ control_inputs[i] + b
    actual = trajectory[i+1]
    error = np.linalg.norm(actual - expected)
    max_error = max(max_error, error)

    print(f"{i:2d}→{i+1:2d} | [{actual[0]:6.3f},{actual[1]:6.3f}] | "
          f"[{expected[0]:6.3f},{expected[1]:6.3f}] | {error:8.6f} | "
          f"[{control_inputs[i][0]:6.3f},{control_inputs[i][1]:6.3f}]")

print("-" * 70)
print(f"最大误差: {max_error:.10f}")
if max_error > 1e-6:
    print("⚠️  注意: 存在较大的约束误差!")
else:
    print("✅ 所有状态转移约束都得到满足")

# 计算并显示状态转移的详细信息
print("\n🔄 状态转移详细信息:")
print("=" * 90)
print("步骤 | 当前状态      | 下一状态      | 控制量u[i]    | Bu贡献       | 状态代价 | 控制代价 | 目标距离")
print("-" * 90)

target_state = np.array([2.0, 1.5])

for i in range(n_steps):
    current = trajectory[i]
    next_state = trajectory[i+1]
    current_control = control_inputs[i]
    bu_contribution = B @ current_control

    # 计算状态和控制代价
    current_state_error = current - target_state
    current_state_cost = 0.5 * np.dot(current_state_error, current_state_error)
    control_cost = 0.1 * np.dot(current_control, current_control)

    # 计算到目标的距离
    current_distance = np.linalg.norm(current_state_error)

    print(f" {i:2d}→{i+1:2d} | "
          f"[{current[0]:6.3f},{current[1]:6.3f}] | "
          f"[{next_state[0]:6.3f},{next_state[1]:6.3f}] | "
          f"[{current_control[0]:7.3f},{current_control[1]:7.3f}] | "
          f"[{bu_contribution[0]:7.3f},{bu_contribution[1]:7.3f}] | "
          f"{current_state_cost:8.4f} | "
          f"{control_cost:8.4f} | "
          f"{current_distance:7.4f}")

# 打印最终状态的详细信息
final_state = trajectory[n_steps]
final_state_error = final_state - target_state
final_state_cost = 0.5 * np.dot(final_state_error, final_state_error)
final_distance = np.linalg.norm(final_state_error)

print("-" * 90)
print(f"最终 | [{final_state[0]:6.3f},{final_state[1]:6.3f}] |        -        |        -        |        -        | {final_state_cost:8.4f} |        -        | {final_distance:7.4f}")
print("=" * 90)

print(f"\n📊 详细状态代价分析:")
print(f"目标状态: [{target_state[0]:.1f}, {target_state[1]:.1f}]")
print()

# 每个状态的详细分析
print("状态 | 坐标          | 与目标偏差     | 状态代价     | 距离目标")
print("-" * 60)
for i in range(n_steps + 1):
    if trajectory is not None and i < len(trajectory):
        state = trajectory[i]
        state_error = state - target_state
        state_cost = 0.5 * np.dot(state_error, state_error)
        distance = np.linalg.norm(state_error)

        print(f"x[{i:2d}] | [{state[0]:6.3f},{state[1]:6.3f}] | "
              f"[{state_error[0]:+7.3f},{state_error[1]:+7.3f}] | "
              f"{state_cost:12.6f} | {distance:9.4f}")
print("=" * 60)

# 累积代价分析
print(f"\n📈 累积代价分析:")
print("步骤 | 状态代价 | 控制代价 | 累积状态代价 | 累积控制代价 | 累积总代价")
print("-" * 75)

cumulative_state_cost = 0.0
cumulative_control_cost = 0.0

# 初始状态代价
state_error = trajectory[0] - target_state
state_cost_0 = 0.5 * np.dot(state_error, state_error)
cumulative_state_cost = state_cost_0
print(f" x[0] | {state_cost_0:8.6f} |   -      | {cumulative_state_cost:12.6f} |   0.000000 | {cumulative_state_cost:12.6f}")

# 每一步的累积代价
for i in range(n_steps):
    # 当前状态代价
    state_error = trajectory[i+1] - target_state
    current_state_cost = 0.5 * np.dot(state_error, state_error)
    cumulative_state_cost += current_state_cost

    # 控制代价
    current_control = control_inputs[i]
    current_control_cost = 0.1 * np.dot(current_control, current_control)
    cumulative_control_cost += current_control_cost

    cumulative_total = cumulative_state_cost + cumulative_control_cost

    print(f" {i:2d}→{i+1:2d} | {current_state_cost:8.6f} | {current_control_cost:8.6f} | {cumulative_state_cost:12.6f} | {cumulative_control_cost:11.6f} | {cumulative_total:12.6f}")

print("=" * 75)

# 总代价统计
total_state_cost = 0.0
total_control_cost = 0.0

for i in range(n_steps + 1):
    state_error = trajectory[i] - target_state
    total_state_cost += 0.5 * np.dot(state_error, state_error)

for i in range(n_steps):
    control = control_inputs[i]
    total_control_cost += 0.1 * np.dot(control, control)

total_cost = total_state_cost + total_control_cost

print(f"\n💰 最终代价统计:")
print(f"  总状态代价 = {total_state_cost:.6f}")
print(f"  总控制代价 = {total_control_cost:.6f}")
print(f"  总代价 = {total_cost:.6f}")
print(f"  平均每步状态代价 = {total_state_cost/(n_steps+1):.6f}")
print(f"  平均每步控制代价 = {total_control_cost/n_steps:.6f}" if n_steps > 0 else "  平均每步控制代价 = 0.000000")

# 控制输入统计分析
control_norms = [np.linalg.norm(control_inputs[i]) for i in range(n_steps)]
print(f"\n🎮 控制输入统计:")
print(f"  最大控制量: {max(control_norms):.6f}")
print(f"  最小控制量: {min(control_norms):.6f}")
print(f"  平均控制量: {np.mean(control_norms):.6f}")
print(f"  控制量标准差: {np.std(control_norms):.6f}")

# 控制能量分析
total_control_energy = sum(np.linalg.norm(control_inputs[i])**2 for i in range(n_steps))
print(f"  总控制能量: {total_control_energy:.6f}")

# 状态转移效率分析
if trajectory is not None and len(trajectory) > 0:
    print(f"\n🎯 优化目标分析:")
    print(f"  目标状态: [{target_state[0]:.1f}, {target_state[1]:.1f}]")
    initial_distance = np.linalg.norm(trajectory[0] - target_state)
    final_distance = np.linalg.norm(trajectory[-1] - target_state)
    print(f"  初始距离: {initial_distance:.4f}")
    print(f"  最终距离: {final_distance:.4f}")
    print(f"  距离改善: {initial_distance - final_distance:.4f}")
else:
    print(f"\n🎯 优化目标分析: 数据不可用")

# 系统特性分析
eigenvalues_A = np.linalg.eigvals(A)
eigenvalues_B = np.linalg.svd(B)[1]  # B的奇异值
print(f"\n🔬 系统特性分析:")
print(f"  状态转移矩阵 A 的特征值: {eigenvalues_A}")
print(f"  系统稳定性: {'稳定' if all(abs(val) < 1 for val in eigenvalues_A) else '不稳定'}")
print(f"  控制矩阵 B 的奇异值: {eigenvalues_B}")
print(f"  控制有效性: {'有效' if min(eigenvalues_B) > 0.1 else '较弱'}")
print(f"  偏置项 b: [{b[0]:.1f}, {b[1]:.1f}]")

# 可控性分析
print(f"\n🎛️  可控性分析:")
controllability_matrix = np.hstack([B, A @ B])
rank = np.linalg.matrix_rank(controllability_matrix)
print(f"  可控性矩阵 rank: {rank}")
print(f"  系统可控性: {'完全可控' if rank == 2 else '部分可控或不可控'}")

# 自由度分析
total_variables = 2 * (n_steps + 1) + 2 * n_steps  # 状态变量 + 控制变量
equality_constraints = 2 * n_steps + 2  # 状态转移约束 + 初始状态约束
degrees_of_freedom = total_variables - equality_constraints
print(f"  总变量数: {total_variables}")
print(f"  等式约束数: {equality_constraints}")
print(f"  优化自由度: {degrees_of_freedom}")

# 验证与求解器结果的一致性
if hasattr(problem, 'value') and problem.value is not None:
    try:
        # 检查是否为数值类型
        if isinstance(problem.value, (int, float, np.number)):
            solver_value = float(problem.value)
            if abs(total_cost - solver_value) > 1e-6:
                print(f"\n⚠️  代价一致性检查:")
                print(f"  计算的总代价: {total_cost:.6f}")
                print(f"  求解器结果: {solver_value:.6f}")
                print(f"  差异: {abs(total_cost - solver_value):.8f}")
            else:
                print(f"\n✅ 代价一致性检查: 通过")
                print(f"  手动计算 = 求解器结果 = {total_cost:.6f}")
        else:
            print(f"\n⚠️  求解器结果类型异常: {type(problem.value)}")
    except Exception as e:
        print(f"\n⚠️  处理求解器结果时出错: {e}")
else:
    print(f"\n⚠️  求解器结果不可用")