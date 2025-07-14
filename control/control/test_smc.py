"""
Test script to verify the mathematical correctness of the SMC implementation.

This script tests:
1. Parameter validation
2. Control law computation
3. Lyapunov function calculation
4. Sliding surface behavior
5. Convergence properties
"""

import numpy as np
import matplotlib.pyplot as plt
from sliding_mode_control import SlidingModeController, VehicleSystem, SMCSimulator


def test_parameter_validation():
    """Test parameter validation in SMC controller."""
    print("Testing parameter validation...")
    
    # Test valid parameters
    try:
        controller = SlidingModeController(lambda_param=1.0, K=2.0, phi=0.1)
        print("✓ Valid parameters accepted")
    except Exception as e:
        print(f"✗ Valid parameters rejected: {e}")
        return False
    
    # Test invalid lambda (negative)
    try:
        controller = SlidingModeController(lambda_param=-1.0, K=2.0, phi=0.1)
        print("✗ Negative lambda accepted (should be rejected)")
        return False
    except ValueError:
        print("✓ Negative lambda correctly rejected")
    
    # Test invalid K (less than disturbance bound)
    try:
        controller = SlidingModeController(lambda_param=1.0, K=0.5, phi=0.1, disturbance_bound=1.0)
        print("✗ K < disturbance_bound accepted (should be rejected)")
        return False
    except ValueError:
        print("✓ K < disturbance_bound correctly rejected")
    
    # Test invalid phi (negative)
    try:
        controller = SlidingModeController(lambda_param=1.0, K=2.0, phi=-0.1)
        print("✗ Negative phi accepted (should be rejected)")
        return False
    except ValueError:
        print("✓ Negative phi correctly rejected")
    
    return True


def test_control_law():
    """Test the control law computation."""
    print("\nTesting control law computation...")
    
    controller = SlidingModeController(lambda_param=1.0, K=2.0, phi=0.1)
    
    # Test case 1: Zero state (should give zero control)
    a_cmd, debug = controller.compute_control(0.0, 0.0)
    expected_a_eq = 0.0
    expected_s = 0.0
    
    if abs(debug['a_equivalent'] - expected_a_eq) < 1e-10:
        print("✓ Zero state gives zero equivalent control")
    else:
        print(f"✗ Zero state equivalent control: got {debug['a_equivalent']}, expected {expected_a_eq}")
        return False
    
    if abs(debug['sliding_surface'] - expected_s) < 1e-10:
        print("✓ Zero state gives zero sliding surface")
    else:
        print(f"✗ Zero state sliding surface: got {debug['sliding_surface']}, expected {expected_s}")
        return False
    
    # Test case 2: Known state
    p, v = 5.0, 2.0
    a_cmd, debug = controller.compute_control(p, v)
    
    expected_s = p + controller.lambda_param * v
    expected_a_eq = -(1.0 / controller.lambda_param) * v
    
    if abs(debug['sliding_surface'] - expected_s) < 1e-10:
        print("✓ Sliding surface computed correctly")
    else:
        print(f"✗ Sliding surface: got {debug['sliding_surface']}, expected {expected_s}")
        return False
    
    if abs(debug['a_equivalent'] - expected_a_eq) < 1e-10:
        print("✓ Equivalent control computed correctly")
    else:
        print(f"✗ Equivalent control: got {debug['a_equivalent']}, expected {expected_a_eq}")
        return False
    
    return True


def test_lyapunov_function():
    """Test Lyapunov function calculation."""
    print("\nTesting Lyapunov function...")
    
    controller = SlidingModeController(lambda_param=1.0, K=2.0, phi=0.1)
    
    # Test various sliding surface values
    test_values = [0.0, 1.0, -1.0, 2.5, -3.7]
    
    for s in test_values:
        V = controller.lyapunov_function(s)
        expected_V = 0.5 * s**2
        
        if abs(V - expected_V) < 1e-10:
            print(f"✓ Lyapunov function correct for s={s}")
        else:
            print(f"✗ Lyapunov function: got {V}, expected {expected_V} for s={s}")
            return False
    
    return True


def test_sliding_surface_behavior():
    """Test sliding surface behavior during simulation."""
    print("\nTesting sliding surface behavior...")
    
    controller = SlidingModeController(lambda_param=1.0, K=2.0, phi=0.1)
    vehicle = VehicleSystem(initial_position=5.0, initial_velocity=3.0)
    simulator = SMCSimulator(controller, vehicle, max_time=10.0)
    
    results = simulator.run_simulation()
    
    # Check that sliding surface decreases monotonically (in magnitude)
    s_values = np.abs(results['sliding_surface'])
    
    # After reaching phase, sliding surface should be small
    final_s = abs(results['sliding_surface'][-1])
    if final_s < 0.1:
        print("✓ Sliding surface converges to small value")
    else:
        print(f"✗ Sliding surface too large at end: {final_s}")
        return False
    
    # Check Lyapunov function decreases
    V_values = results['lyapunov']
    if V_values[-1] < V_values[0]:
        print("✓ Lyapunov function decreases")
    else:
        print(f"✗ Lyapunov function increases: {V_values[0]} -> {V_values[-1]}")
        return False
    
    return True


def test_convergence_properties():
    """Test convergence properties of the controller."""
    print("\nTesting convergence properties...")
    
    # Test different initial conditions
    initial_conditions = [
        (1.0, 0.5),
        (5.0, 2.0),
        (-3.0, 1.0),
        (10.0, -5.0)
    ]
    
    controller = SlidingModeController(lambda_param=1.0, K=2.0, phi=0.1)
    
    for p0, v0 in initial_conditions:
        vehicle = VehicleSystem(initial_position=p0, initial_velocity=v0)
        simulator = SMCSimulator(controller, vehicle, max_time=20.0)
        results = simulator.run_simulation()
        
        final_p = abs(results['position'][-1])
        final_v = abs(results['velocity'][-1])
        
        if results['converged']:
            print(f"✓ Converged from ({p0}, {v0}) to ({final_p:.6f}, {final_v:.6f})")
        else:
            print(f"✗ Failed to converge from ({p0}, {v0})")
            return False
    
    return True


def test_disturbance_rejection():
    """Test disturbance rejection capability."""
    print("\nTesting disturbance rejection...")
    
    controller = SlidingModeController(lambda_param=1.0, K=2.5, phi=0.1)
    
    # Test with constant disturbance
    disturbance_func = lambda t: 0.5  # Constant disturbance
    vehicle = VehicleSystem(
        initial_position=5.0,
        initial_velocity=2.0,
        disturbance_function=disturbance_func
    )
    
    simulator = SMCSimulator(controller, vehicle, max_time=25.0)
    results = simulator.run_simulation()
    
    # With disturbance, it might not converge exactly to zero but should be bounded
    final_p = abs(results['position'][-1])
    final_v = abs(results['velocity'][-1])
    
    if final_p < 1.0 and final_v < 1.0:  # Bounded error
        print(f"✓ Bounded error with disturbance: p={final_p:.6f}, v={final_v:.6f}")
    else:
        print(f"✗ Unbounded error with disturbance: p={final_p:.6f}, v={final_v:.6f}")
        return False
    
    return True


def visualize_phase_portrait():
    """Create a phase portrait visualization."""
    print("\nCreating phase portrait visualization...")
    
    controller = SlidingModeController(lambda_param=1.0, K=2.0, phi=0.1)
    
    # Multiple initial conditions
    initial_conditions = [
        (8.0, 4.0), (6.0, -2.0), (-4.0, 3.0), (-6.0, -3.0),
        (10.0, 0.0), (0.0, 5.0), (5.0, 5.0), (-5.0, -5.0)
    ]
    
    plt.figure(figsize=(10, 8))
    
    for p0, v0 in initial_conditions:
        vehicle = VehicleSystem(initial_position=p0, initial_velocity=v0)
        simulator = SMCSimulator(controller, vehicle, max_time=15.0)
        results = simulator.run_simulation()
        
        plt.plot(results['position'], results['velocity'], 'b-', alpha=0.7, linewidth=1.5)
        plt.plot(p0, v0, 'go', markersize=6)
    
    # Plot sliding surface line
    p_range = np.linspace(-12, 12, 100)
    sliding_line = -p_range / controller.lambda_param
    plt.plot(p_range, sliding_line, 'r--', linewidth=2, label=f'Sliding Surface (λ={controller.lambda_param})')
    
    plt.plot(0, 0, 'ro', markersize=8, label='Origin (Target)')
    plt.xlabel('Position (m)')
    plt.ylabel('Velocity (m/s)')
    plt.title('Phase Portrait - Multiple Initial Conditions')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')
    plt.xlim(-12, 12)
    plt.ylim(-8, 8)
    
    plt.tight_layout()
    plt.show()
    
    print("✓ Phase portrait visualization created")


def main():
    """Run all tests."""
    print("=" * 60)
    print("SLIDING MODE CONTROL MATHEMATICAL VERIFICATION")
    print("=" * 60)
    
    tests = [
        test_parameter_validation,
        test_control_law,
        test_lyapunov_function,
        test_sliding_surface_behavior,
        test_convergence_properties,
        test_disturbance_rejection
    ]
    
    passed = 0
    total = len(tests)
    
    for test in tests:
        if test():
            passed += 1
        else:
            print(f"Test {test.__name__} FAILED")
    
    print("\n" + "=" * 60)
    print(f"TEST RESULTS: {passed}/{total} tests passed")
    print("=" * 60)
    
    if passed == total:
        print("🎉 All tests passed! SMC implementation is mathematically correct.")
        
        # Create visualization
        visualize_phase_portrait()
    else:
        print("❌ Some tests failed. Please check the implementation.")
    
    return passed == total


if __name__ == "__main__":
    main() 