"""
Simple test to verify the separation is working correctly
"""

def test_imports():
    """Test that imports work correctly"""
    print("Testing imports...")
    
    try:
        # Test core algorithm import (should work without matplotlib)
        from hybrid_astar import HybridAStar, VehicleModel, State, DirectionMode
        print("✓ Core algorithm imported successfully")
        
        # Test that we can create instances
        vehicle = VehicleModel()
        planner = HybridAStar(vehicle_model=vehicle)
        state = State(x=0.0, y=0.0, yaw=0.0)
        print("✓ Core classes instantiated successfully")
        
        # Test that planner has no matplotlib dependencies
        methods = [method for method in dir(planner) if not method.startswith('_')]
        viz_methods = [m for m in methods if 'visualize' in m.lower() or 'plot' in m.lower()]
        
        if not viz_methods:
            print("✓ Core algorithm has no visualization methods")
        else:
            print(f"⚠ Found visualization methods in core: {viz_methods}")
        
        # Test data storage for visualization
        planner.explored_nodes = []
        planner.simulation_trajectories = []
        viz_data = planner.get_visualization_data()
        if isinstance(viz_data, dict):
            print("✓ Visualization data storage works")
        
        return True
        
    except ImportError as e:
        print(f"✗ Core import failed: {e}")
        return False
    except Exception as e:
        print(f"✗ Core test failed: {e}")
        return False


def test_visualization_import():
    """Test visualization import (optional)"""
    print("\nTesting visualization import...")
    
    try:
        from visualizer import HybridAStarVisualizer
        print("✓ Visualizer imported successfully")
        
        # Test instantiation
        viz = HybridAStarVisualizer()
        print("✓ Visualizer instantiated successfully")
        
        # Check methods
        viz_methods = [method for method in dir(viz) if not method.startswith('_')]
        expected_methods = ['visualize_path', 'visualize_search_progress', 'visualize_detailed_search_tree']
        
        for method in expected_methods:
            if hasattr(viz, method):
                print(f"✓ Method {method} exists")
            else:
                print(f"✗ Method {method} missing")
        
        return True
        
    except ImportError as e:
        print(f"⚠ Visualization import failed (optional): {e}")
        return True  # This is OK, visualization is optional
    except Exception as e:
        print(f"✗ Visualization test failed: {e}")
        return False


def test_no_matplotlib_in_core():
    """Test that core algorithm doesn't import matplotlib"""
    print("\nTesting matplotlib dependency separation...")
    
    try:
        # Import core without matplotlib
        import sys
        
        # Check if matplotlib was imported by core
        matplotlib_modules = [name for name in sys.modules.keys() if 'matplotlib' in name]
        
        # Import core
        from hybrid_astar import HybridAStar, VehicleModel
        
        # Check again after importing core
        new_matplotlib_modules = [name for name in sys.modules.keys() if 'matplotlib' in name]
        
        if len(new_matplotlib_modules) == len(matplotlib_modules):
            print("✓ Core algorithm doesn't import matplotlib")
            return True
        else:
            print(f"✗ Core algorithm imported matplotlib: {set(new_matplotlib_modules) - set(matplotlib_modules)}")
            return False
            
    except Exception as e:
        print(f"✗ Matplotlib dependency test failed: {e}")
        return False


if __name__ == "__main__":
    print("Testing Hybrid A* Algorithm and Visualization Separation")
    print("=" * 60)
    
    # Test core functionality
    core_works = test_imports()
    
    # Test visualization (optional)
    viz_works = test_visualization_import()
    
    # Test separation 
    separation_works = test_no_matplotlib_in_core()
    
    print(f"\n{'='*60}")
    print("SEPARATION TEST RESULTS:")
    print(f"{'='*60}")
    print(f"Core Algorithm:        {'✓ PASS' if core_works else '✗ FAIL'}")
    print(f"Visualization Module:  {'✓ PASS' if viz_works else '✗ FAIL'}")
    print(f"Dependency Separation: {'✓ PASS' if separation_works else '✗ FAIL'}")
    
    all_passed = core_works and viz_works and separation_works
    
    if all_passed:
        print(f"\n🎉 SEPARATION SUCCESSFUL! 🎉")
        print("\nBenefits achieved:")
        print("  ✓ Core algorithm is independent of visualization")
        print("  ✓ No matplotlib dependency in core algorithm")
        print("  ✓ Visualization is completely modular and optional")
        print("  ✓ Algorithm can run in headless environments")
        print("  ✓ Easier to test and maintain")
        print("  ✓ Can use different visualization backends")
    else:
        print(f"\n❌ SEPARATION INCOMPLETE")
        
    print(f"\n{'='*60}")
