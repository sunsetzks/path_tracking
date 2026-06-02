#!/usr/bin/env python3
import hybrid_astar_cpp as ha
import numpy as np

print("Testing plan_path...")

vehicle = ha.VehicleModel(2.5, np.pi/4)
print(f"Vehicle created: {vehicle}")

planner = ha.HybridAStar(vehicle, 1.0, np.pi/8, np.pi/16, 2.0, 1.0, 0.2)
print(f"Planner created: {planner}")

obstacle_map = ha.create_obstacle_map(10, 10)
planner.set_obstacle_map(obstacle_map, 0.0, 0.0)
print("Obstacle map set")

start = ha.State(1.0, 1.0, 0.0, ha.DirectionMode.NONE, 0.0)
goal = ha.State(3.0, 3.0, 0.0, ha.DirectionMode.FORWARD, 0.0)
print(f"Start: {start}")
print(f"Goal: {goal}")

print("Calling plan_path...")
path = planner.plan_path(start, goal, 500)
print(f"Path result: {path}")
print(f"Path type: {type(path)}")

if path is not None:
    print(f"Path length: {len(path)}")
    if len(path) > 0:
        first_element = path[0]
        print(f"First element: {first_element}")
        print(f"First element type: {type(first_element)}")
        
        # Try to access attributes to see what it is
        try:
            print(f"First element has state: {hasattr(first_element, 'state')}")
            if hasattr(first_element, 'state'):
                print(f"First element state: {first_element.state}")
        except Exception as e:
            print(f"Error accessing state: {e}")
        
        # Check if it's a State object
        try:
            print(f"First element x: {first_element.x}")
            print(f"First element y: {first_element.y}")
        except Exception as e:
            print(f"Error accessing x,y: {e}")
            
    # Try statistics with the path
    try:
        stats = planner.get_statistics(path)
        print(f"Statistics: {stats}")
    except Exception as e:
        print(f"Error getting statistics: {e}")
else:
    print("No path found")

print("Test completed.")
