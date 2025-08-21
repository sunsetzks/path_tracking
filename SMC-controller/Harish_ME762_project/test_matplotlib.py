#!/usr/bin/env python3
"""Test script to verify matplotlib works with the package."""

import sys
import os

# Add the current directory to Python path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import Python_SMC_Simulation.example_usage as ex
    print('Example usage module loaded successfully!')
    print('Available functions:', [f for f in dir(ex) if not f.startswith('_')])
except Exception as e:
    print(f'Error loading example usage: {e}')
    exit(1)
