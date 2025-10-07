import matplotlib.pyplot as plt
import numpy as np

# Define grid
x1 = np.linspace(-3, 3, 2000)
x2 = np.linspace(-3, 3, 2000)
X1, X2 = np.meshgrid(x1, x2)

# Compute function
Z = X1 * X2**2

# Plot 3D surface
fig = plt.figure(figsize=(7, 5))
ax = fig.add_subplot(111, projection='3d')
ax.plot_surface(X1, X2, Z, cmap='plasma')

ax.set_title(r"Surface of $f(x_1, x_2) = x_1 x_2^2$")
ax.set_xlabel("$x_1$")
ax.set_ylabel("$x_2$")
ax.set_zlabel("$f(x_1, x_2)$")

plt.show()
