"""Simple example to exercise the mpc_car_tracking package.

This script runs a short, non-interactive simulation to verify the package
imports and main API work. It avoids showing plots by default so it can run
in CI or headless environments.

Usage:
    PYTHONPATH=src python examples/simple_simulation.py

"""
from __future__ import annotations

import os

from mpc_car_tracking import run_demo


def main():
    # Run a quick scenario without showing plots (fast, non-interactive)
    result = run_demo(scenario='straight_line', show_plots=True, save_plots=False)

    if result.get('success'):
        print("Example completed successfully.")
        metrics = result.get('metrics', {})
        if metrics:
            print("Metrics:")
            for k, v in metrics.items():
                print(f"  {k}: {v}")
    else:
        print("Example failed:", result.get('error'))


if __name__ == '__main__':
    main()
