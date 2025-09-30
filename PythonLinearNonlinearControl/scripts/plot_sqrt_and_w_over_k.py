#!/usr/bin/env python3

"""
Plot sqrt(a / k) and w / k for k in [1, 10], with a = 1 and w = 1.
The plot is displayed and also saved to the project's assets directory.
"""

from pathlib import Path
from typing import Tuple

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.figure import Figure


def compute_series(a_value: float, w_value: float, k_min: float = 0.1, k_max: float = 10.0, num_points: int = 100) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Compute k range and the two series sqrt(a / k) and w / k.

    Parameters
    ----------
    a_value: float
        Parameter a in sqrt(a / k)
    w_value: float
        Parameter w in w / k
    k_min: float
        Start of k range (inclusive)
    k_max: float
        End of k range (inclusive)
    num_points: int
        Number of points to generate in the range

    Returns
    -------
    Tuple[np.ndarray, np.ndarray, np.ndarray]
        k array, sqrt(a/k) array, w/k array
    """
    k_values = np.linspace(k_min, k_max, num_points)
    sqrt_series = np.sqrt(a_value / k_values)
    w_over_k_series = w_value / k_values
    return k_values, sqrt_series, w_over_k_series


def plot_series(k_values: np.ndarray, sqrt_series: np.ndarray, w_over_k_series: np.ndarray) -> Figure:
    """Create the plot for the two series.

    Returns the matplotlib Figure for further saving or display.
    """
    fig, ax = plt.subplots(figsize=(7, 4.5), constrained_layout=True)
    ax.plot(k_values, sqrt_series, linestyle="-", color="#1f77b4", label=r"$\sqrt{a/k}$ (a=1)")
    ax.plot(k_values, w_over_k_series, linestyle="--", color="#ff7f0e", label=r"$w/k$ (w=1)")

    ax.set_xlabel("k")
    ax.set_ylabel("Value")
    ax.set_title(r"Comparison of $\sqrt{a/k}$ and $w/k$ for k = 0.1..10, a = 1, w = 1")
    ax.grid(True, which="both", linestyle=":", linewidth=0.8, alpha=0.7)
    ax.legend()

    # Set x-axis ticks at log scale for better visualization
    ax.set_xscale('log')
    ax.set_xticks([0.1, 0.2, 0.5, 1, 2, 5, 10])
    ax.set_xticklabels(['0.1', '0.2', '0.5', '1', '2', '5', '10'])
    return fig


def main() -> None:
    # Parameters per request
    a_value = 1.0
    w_value = 1.0

    k_values, sqrt_series, w_over_k_series = compute_series(a_value, w_value, 0.1, 10.0, 200)
    fig = plot_series(k_values, sqrt_series, w_over_k_series)

    # Save to assets directory next to the project
    assets_dir = Path(__file__).resolve().parents[1] / "assets"
    assets_dir.mkdir(parents=True, exist_ok=True)
    output_path = assets_dir / "sqrt_and_w_over_k.png"
    fig.savefig(output_path, dpi=200)
    print(f"Saved figure to: {output_path}")

    # Also show the plot for interactive use
    plt.show()


if __name__ == "__main__":
    main()


