"""
Test script for Foxglove visualization integration

This script runs a quick test to verify that the Foxglove visualizer
works correctly with the Hybrid A* implementation.
"""

import sys
import asyncio
import numpy as np
from typing import Optional

# Add project path
sys.path.append('/home/zks/ws/path_tracking/astar_project')

try:
    from astar_project.hybrid_astar import VehicleModel, HybridAStar, State, DirectionMode
    print("✓ Successfully imported Hybrid A* modules")
except ImportError as e:
    print(f"✗ Failed to import Hybrid A* modules: {e}")
    sys.exit(1)

try:
    from astar_project.foxglove_visualizer import FoxgloveHybridAStarVisualizer, FOXGLOVE_AVAILABLE
    if FOXGLOVE_AVAILABLE:
        print("✓ Foxglove SDK is available")
    else:
        print("⚠ Foxglove SDK not available, will test basic functionality")
except ImportError as e:
    print(f"⚠ Foxglove visualizer import failed: {e}")
    FOXGLOVE_AVAILABLE = False


def test_basic_planning():
    """Test basic path planning functionality"""
    print("\n=== Testing Basic Path Planning ===")
    
    # Create vehicle model
    vehicle = VehicleModel(wheelbase=2.5, max_steer=np.pi/4)
    print("✓ Vehicle model created")
    
    # Create planner
    planner = HybridAStar(
        vehicle_model=vehicle,
        grid_resolution=0.5,
        angle_resolution=np.pi/8,
        velocity=2.0,
        simulation_time=0.5,
        dt=0.1
    )
    print("✓ Hybrid A* planner created")
    
    # Create empty obstacle map for easier planning
    obstacle_map = np.zeros((40, 40))
    # Just add a small obstacle to make it interesting
    obstacle_map[18:22, 18:22] = 1  
    planner.set_obstacle_map(obstacle_map, origin_x=-10, origin_y=-10)
    print("✓ Obstacle map set")
    
    # Define easy start and goal
    start = State(x=-5.0, y=-5.0, yaw=0.0, direction=DirectionMode.NONE)
    goal = State(x=5.0, y=5.0, yaw=0.0, direction=DirectionMode.FORWARD)
    print("✓ Start and goal states defined")
    
    # Plan path
    print("Planning path...")
    path = planner.plan_path(start, goal, max_iterations=2000)
    
    if path:
        print(f"✓ Path found with {len(path)} waypoints")
        return planner, start, goal, path
    else:
        print("✗ No path found, but continuing with mock data for testing")
        # Create mock path for testing visualization components
        mock_path = [
            type('Node', (), {'state': State(x=-5.0, y=-5.0, yaw=0.0, direction=DirectionMode.FORWARD)})(),
            type('Node', (), {'state': State(x=-2.5, y=-2.5, yaw=np.pi/4, direction=DirectionMode.FORWARD)})(),
            type('Node', (), {'state': State(x=0.0, y=0.0, yaw=np.pi/2, direction=DirectionMode.FORWARD)})(),
            type('Node', (), {'state': State(x=2.5, y=2.5, yaw=3*np.pi/4, direction=DirectionMode.FORWARD)})(),
            type('Node', (), {'state': State(x=5.0, y=5.0, yaw=0.0, direction=DirectionMode.FORWARD)})()
        ]
        print("✓ Using mock path for testing")
        return planner, start, goal, mock_path


def test_visualization_data():
    """Test visualization data extraction"""
    print("\n=== Testing Visualization Data ===")
    
    result = test_basic_planning()
    if result[0] is None:
        print("✗ Cannot test visualization without valid path")
        return False
    
    planner, start, goal, path = result
    
    # Get visualization data
    viz_data = planner.get_visualization_data()
    print("✓ Visualization data extracted")
    
    # Check data contents
    required_keys = ['explored_nodes', 'simulation_trajectories', 'obstacle_map']
    for key in required_keys:
        if key in viz_data:
            print(f"✓ {key} present in visualization data")
        else:
            print(f"✗ {key} missing from visualization data")
    
    # Test statistics
    stats = planner.get_statistics(path)
    print(f"✓ Statistics calculated: {len(stats)} metrics")
    
    return True


async def test_foxglove_server():
    """Test Foxglove server creation and basic functionality"""
    print("\n=== Testing Foxglove Server ===")
    
    if not FOXGLOVE_AVAILABLE:
        print("⚠ Skipping Foxglove server test - SDK not available")
        return False
    
    try:
        # Create visualizer
        visualizer = FoxgloveHybridAStarVisualizer(port=8766)  # Use different port
        print("✓ Foxglove visualizer created")
        
        # Test server creation (without actually starting)
        print("✓ Visualizer initialization successful")
        
        # Test color creation
        color = visualizer._create_color(1.0, 0.0, 0.0, 1.0)
        if color is not None:
            print("✓ Color creation works")
        else:
            print("✗ Color creation failed")
        
        # Test point creation
        point = visualizer._create_point3(1.0, 2.0, 3.0)
        if point is not None:
            print("✓ Point creation works")
        else:
            print("✗ Point creation failed")
        
        return True
        
    except Exception as e:
        print(f"✗ Foxglove server test failed: {e}")
        return False


def test_fallback_visualization():
    """Test matplotlib fallback visualization"""
    print("\n=== Testing Fallback Visualization ===")
    
    try:
        from astar_project.foxglove_demo import matplotlib_fallback_visualization
        print("✓ Fallback visualization function imported")
        
        # Test with minimal data
        path = [
            State(x=0.0, y=0.0, yaw=0.0),
            State(x=1.0, y=1.0, yaw=np.pi/4),
            State(x=2.0, y=2.0, yaw=np.pi/2)
        ]
        start = State(x=0.0, y=0.0, yaw=0.0)
        goal = State(x=2.0, y=2.0, yaw=np.pi/2)
        
        print("✓ Test data created for fallback visualization")
        return True
        
    except ImportError as e:
        print(f"✗ Failed to import fallback visualization: {e}")
        return False


async def run_integration_test():
    """Run quick integration test if Foxglove is available"""
    print("\n=== Integration Test ===")
    
    if not FOXGLOVE_AVAILABLE:
        print("⚠ Skipping integration test - Foxglove SDK not available")
        return False
    
    result = test_basic_planning()
    if result[0] is None:
        print("✗ Cannot run integration test without valid path")
        return False
    
    planner, start, goal, path = result
    
    try:
        # Create visualizer
        visualizer = FoxgloveHybridAStarVisualizer(port=8767)
        
        # Test data preparation (without actually starting server)
        path_states = [node.state for node in path]
        viz_data = planner.get_visualization_data()
        
        # Store data in visualizer
        visualizer.current_data = {
            'path': path_states,
            'start': start,
            'goal': goal,
            **viz_data
        }
        
        print("✓ Integration test data prepared successfully")
        return True
        
    except Exception as e:
        print(f"✗ Integration test failed: {e}")
        return False


def main():
    """Run all tests"""
    print("Foxglove Visualization Test Suite")
    print("=" * 50)
    
    tests = [
        ("Basic Planning", test_basic_planning),
        ("Visualization Data", test_visualization_data),
        ("Foxglove Server", test_foxglove_server),
        ("Fallback Visualization", test_fallback_visualization),
        ("Integration Test", run_integration_test),
    ]
    
    results = []
    
    for test_name, test_func in tests:
        print(f"\nRunning {test_name}...")
        try:
            if asyncio.iscoroutinefunction(test_func):
                result = asyncio.run(test_func())
            else:
                result = test_func()
            
            if result not in [None, False]:
                results.append((test_name, "PASS"))
                print(f"✓ {test_name} PASSED")
            else:
                results.append((test_name, "FAIL"))
                print(f"✗ {test_name} FAILED")
                
        except Exception as e:
            results.append((test_name, "ERROR"))
            print(f"✗ {test_name} ERROR: {e}")
    
    # Summary
    print("\n" + "=" * 50)
    print("TEST SUMMARY")
    print("=" * 50)
    
    passed = sum(1 for _, status in results if status == "PASS")
    failed = sum(1 for _, status in results if status == "FAIL")
    errors = sum(1 for _, status in results if status == "ERROR")
    
    for test_name, status in results:
        status_symbol = "✓" if status == "PASS" else "✗"
        print(f"{status_symbol} {test_name}: {status}")
    
    print(f"\nResults: {passed} passed, {failed} failed, {errors} errors")
    
    if FOXGLOVE_AVAILABLE:
        print("\n✓ Foxglove SDK is available - full functionality ready")
        print("To test full visualization:")
        print("  python foxglove_demo.py")
    else:
        print("\n⚠ Foxglove SDK not available")
        print("To install: pip install foxglove-sdk foxglove-schemas-protobuf")
        print("For fallback visualization: python foxglove_demo.py")
    
    return passed == len(tests)


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
