#!/usr/bin/env python3
"""Test steering angle discretization"""

from astar_project.hybrid_astar import HybridAStar, VehicleModel, State
import numpy as np

def test_steer_discretization():
    # Create a simple test
    vehicle = VehicleModel()
    planner = HybridAStar(
        vehicle_model=vehicle,
        grid_resolution=1.0,
        angle_resolution=np.pi/4,
        steer_resolution=np.pi/16
    )

    # Test state with different steering angles
    state1 = State(x=1.5, y=2.5, yaw=np.pi/3, steer=0.0)
    state2 = State(x=1.5, y=2.5, yaw=np.pi/3, steer=np.pi/8)

    grid1 = planner.discretize_state(state1)
    grid2 = planner.discretize_state(state2)

    print("State 1 (steer=0.0):", grid1)
    print("State 2 (steer=π/8):", grid2)
    print("Different discretization:", grid1 != grid2)
    print("Expected steer grid values:", int(0.0 / (np.pi/16)), "vs", int((np.pi/8) / (np.pi/16)))

if __name__ == "__main__":
    test_steer_discretization()
