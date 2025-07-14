import numpy as np
from scipy.integrate import solve_ivp
import matplotlib.pyplot as plt

# Define the ODE
def exponential_decay(t, y):
    return -2 * y

# Solve
sol = solve_ivp(exponential_decay, [0, 5], [3], t_eval=np.linspace(0, 5, 100))

# Plot
plt.plot(sol.t, sol.y[0], label='Numerical solution')
t_exact = np.linspace(0, 5, 100)
y_exact = 3 * np.exp(-2 * t_exact)
plt.plot(t_exact, y_exact, '--', label='Exact solution')
plt.legend()
plt.xlabel('t')
plt.ylabel('y(t)')
plt.title('Solution of dy/dt = -2y, y(0)=3')
plt.show()

def lotka_volterra(t, z, alpha, beta, delta, gamma):
    x, y = z
    return [alpha*x - beta*x*y, delta*x*y - gamma*y]

sol = solve_ivp(
    lotka_volterra, 
    [0, 15], 
    [10, 5], 
    args=(0.1, 0.2, 0.1, 0.1),
    t_eval=np.linspace(0, 15, 300)
)

plt.plot(sol.t, sol.y[0], label='Prey')
plt.plot(sol.t, sol.y[1], label='Predator')
plt.legend()
plt.xlabel('Time')
plt.ylabel('Population')
plt.title('Lotka-Volterra Model')
plt.show()