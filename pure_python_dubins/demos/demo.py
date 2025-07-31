import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import pure_python_dubins as dubins
import matplotlib
import matplotlib.pyplot as plt
import numpy

matplotlib.rcParams['figure.figsize'] = 12, 9

qs = [
    (0.0, 0.0, 0.0),
    (0.0, 0.0, numpy.pi/4),
    (4.0, 4.0, numpy.pi/4),
    (4.0, 0.0, 0.0),
    (-4.0, 0.0, 0.0),
    (4.0, 4.0, 0.0),
    (4.0, -4.0, 0.0),
    (-4.0, 4.0, 0.0),
    (-4.0, -4.0, 0.0),
    (4.0, 4.0, numpy.pi),
    (4.0, -4.0, numpy.pi),
    (0.5, 0.0, numpy.pi),
]

items = [
    (0, 4),
    (0, 5),
    (0, 6),
    (0, 7),
    (0, 8),
    (0, 9),
    (0, 10),
    (0, 11),
    (1, 2),
    (2, 1)
]

def expand_axis(ax, scale, name):
    getter = getattr(ax, 'get_' + name)
    setter = getattr(ax, 'set_' + name)
    a, b = getter()
    mid = (a+b)/2.0
    diff = (b - mid)
    setter(mid - scale*diff, mid + scale*diff)

def expand_plot(ax, scale = 1.1):
    expand_axis(ax, scale, 'xlim')
    expand_axis(ax, scale, 'ylim')

def plot_dubins_path(q0, q1, r=1.0, step_size=0.5):
    qs, _ = dubins.path_sample(q0, q1, r, step_size)
    qs = numpy.array(qs)
    xs = qs[:, 0]
    ys = qs[:, 1]
    us = xs + numpy.cos(qs[:, 2])
    vs = ys + numpy.sin(qs[:, 2])
    plt.plot(xs, ys, 'b-')
    plt.plot(xs, ys, 'r.')
    # Plot start and goal direction as arrows
    plt.arrow(xs[0], ys[0], numpy.cos(qs[0, 2]), numpy.sin(qs[0, 2]),
              head_width=0.2, head_length=0.3, fc='green', ec='green')
    plt.arrow(xs[-1], ys[-1], numpy.cos(qs[-1, 2]), numpy.sin(qs[-1, 2]),
              head_width=0.2, head_length=0.3, fc='red', ec='red')
    ax = plt.gca()
    expand_plot(ax)
    ax.set_aspect('equal')

def plot_backward_dubins_path(q0, q1, r=1.0, step_size=0.5):
    """Plot a Dubins path for backward motion"""
    qs, _ = dubins.backward_path_sample(q0, q1, r, step_size)
    qs = numpy.array(qs)
    xs = qs[:, 0]
    ys = qs[:, 1]
    us = xs + numpy.cos(qs[:, 2])
    vs = ys + numpy.sin(qs[:, 2])
    plt.plot(xs, ys, 'm-', linewidth=2, label='Backward Path')
    plt.plot(xs, ys, 'c.')
    # Plot start and goal direction as arrows
    plt.arrow(xs[0], ys[0], numpy.cos(qs[0, 2]), numpy.sin(qs[0, 2]),
              head_width=0.2, head_length=0.3, fc='purple', ec='purple')
    plt.arrow(xs[-1], ys[-1], numpy.cos(qs[-1, 2]), numpy.sin(qs[-1, 2]),
              head_width=0.2, head_length=0.3, fc='orange', ec='orange')
    ax = plt.gca()
    expand_plot(ax)
    ax.set_aspect('equal')


def plot_forward_backward_comparison(q0, q1, r=1.0, step_size=0.5):
    """Plot both forward and backward Dubins paths for comparison"""
    # Plot forward path
    qs_forward, _ = dubins.path_sample(q0, q1, r, step_size)
    qs_forward = numpy.array(qs_forward)
    xs_forward = qs_forward[:, 0]
    ys_forward = qs_forward[:, 1]
    plt.plot(xs_forward, ys_forward, 'b-', linewidth=2, label='Forward Path')
    plt.plot(xs_forward, ys_forward, 'b.')
    
    # Plot backward path
    qs_backward, _ = dubins.backward_path_sample(q0, q1, r, step_size)
    qs_backward = numpy.array(qs_backward)
    xs_backward = qs_backward[:, 0]
    ys_backward = qs_backward[:, 1]
    plt.plot(xs_backward, ys_backward, 'm-', linewidth=2, label='Backward Path')
    plt.plot(xs_backward, ys_backward, 'm.')
    
    # Plot start and goal poses
    plt.plot(q0[0], q0[1], 'go', markersize=10, label='Start')
    plt.plot(q1[0], q1[1], 'ro', markersize=10, label='Goal')
    
    # Plot direction arrows for forward path
    plt.arrow(xs_forward[0], ys_forward[0], numpy.cos(qs_forward[0, 2]), numpy.sin(qs_forward[0, 2]),
              head_width=0.2, head_length=0.3, fc='green', ec='green')
    plt.arrow(xs_forward[-1], ys_forward[-1], numpy.cos(qs_forward[-1, 2]), numpy.sin(qs_forward[-1, 2]),
              head_width=0.2, head_length=0.3, fc='red', ec='red')
    
    # Plot direction arrows for backward path
    plt.arrow(xs_backward[0], ys_backward[0], numpy.cos(qs_backward[0, 2]), numpy.sin(qs_backward[0, 2]),
              head_width=0.2, head_length=0.3, fc='purple', ec='purple')
    plt.arrow(xs_backward[-1], ys_backward[-1], numpy.cos(qs_backward[-1, 2]), numpy.sin(qs_backward[-1, 2]),
              head_width=0.2, head_length=0.3, fc='orange', ec='orange')
    
    ax = plt.gca()
    expand_plot(ax)
    ax.set_aspect('equal')
    ax.legend()

def plot_dubins_table(cols, rho=1.0):
    rows = int((len(items) + cols - 1) / cols)
    for i, (a, b) in enumerate(items):
        plt.subplot(rows, cols, i+1)
        plot_dubins_path(qs[a], qs[b], r = rho)
    plt.savefig('samples.png')
    plt.show()

if __name__ == "__main__":
    # Test backward motion functionality
    print("Testing Dubins backward motion extension...")
    
    # Test case 1: Simple forward vs backward comparison
    print("\n=== Test Case 1: Forward vs Backward Path Comparison ===")
    q0_test = (0.0, 0.0, 0.0)
    q1_test = (4.0, 0.0, 0.0)
    
    plt.figure(figsize=(12, 8))
    plot_forward_backward_comparison(q0_test, q1_test, r=1.0, step_size=0.2)
    plt.title("Forward vs Backward Dubins Path")
    plt.grid(True)
    plt.savefig('forward_backward_comparison.png')
    plt.show()
    
    # Test case 2: Different orientations
    print("\n=== Test Case 2: Different Orientations ===")
    q0_test2 = (0.0, 0.0, numpy.pi/4)
    q1_test2 = (4.0, 3.0, -numpy.pi/4)
    
    plt.figure(figsize=(12, 8))
    plot_forward_backward_comparison(q0_test2, q1_test2, r=1.0, step_size=0.2)
    plt.title("Forward vs Backward Path - Different Orientations")
    plt.grid(True)
    plt.savefig('forward_backward_comparison_angles.png')
    plt.show()
    
    # Test case 3: Backward only path
    print("\n=== Test Case 3: Backward Only Path ===")
    q0_test3 = (4.0, 4.0, numpy.pi)
    q1_test3 = (0.0, 0.0, 0.0)
    
    plt.figure(figsize=(10, 8))
    plot_backward_dubins_path(q0_test3, q1_test3, r=1.0, step_size=0.2)
    plt.title("Backward Only Dubins Path")
    plt.grid(True)
    plt.savefig('backward_only_path.png')
    plt.show()
    
    # Original demo
    print("\n=== Original Demo ===")
    plot_dubins_table(3, 1.0)
    
    print("\nBackward motion extension test completed!")