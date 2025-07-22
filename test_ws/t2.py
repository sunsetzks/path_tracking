import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import BSpline

# Define the knot vector (length m+1)
knots = [0, 1, 2, 3, 4, 5, 6]

# Extend for B-spline requirements: need p+1 copies at ends for clamped,
# but since we want open uniform, just repeat first and last degree+1 times.
def make_knot_vector(degree, internal_knots):
    """Create an open knot vector by repeating first and last knots."""
    return (
        [internal_knots[0]] * (degree + 1) +
        internal_knots[1:-1] +
        [internal_knots[-1]] * (degree + 1)
    )

# Range of u values to evaluate
u_fine = np.linspace(0, 6, 1000)

fig, axes = plt.subplots(2, 2, figsize=(12, 8))
axes = axes.ravel()

degrees = [0, 1, 2, 3]

for idx, p in enumerate(degrees):
    ax = axes[idx]
    
    # Create open knot vector with repeated endpoints
    extended_knots = make_knot_vector(p, knots)
    
    # Number of control points: n + 1 = len(knots) + p - 1?
    # Actually: num_basis = len(extended_knots) - p - 1
    num_basis = len(extended_knots) - p - 1
    
    # Plot each basis function N_i,p(u)
    for i in range(num_basis):
        # Coefficients: all zero except one (to isolate basis function)
        coeffs = np.zeros(num_basis)
        coeffs[i] = 1
        
        # Construct B-spline (only the i-th basis function)
        b_spline = BSpline(extended_knots, coeffs, p)
        
        # Evaluate
        y = b_spline(u_fine)
        
        # Only plot where it's non-zero
        ax.plot(u_fine, y, label=f'$N_{{{i},{p}}}$', linewidth=2)

    ax.set_title(f'B-spline Basis Functions (Degree {p})')
    ax.set_xlabel('$u$')
    ax.set_ylabel('Value')
    ax.grid(True, alpha=0.3)
    ax.set_ylim(-0.1, 1.1)

plt.tight_layout()
plt.show()