"""
Comprehensive Test Suite

Runs multiple scenarios to test the Stanley controller's performance across
different conditions including various speeds, paths, and obstacle configurations.
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import os
from matplotlib.patches import Circle

# Add parent directory to path for imports
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from stanley_controller import StanleyController, VehicleDynamics, SimulationEnvironment, Visualizer
from stanley_controller.utils.se2 import SE2
from stanley_controller.vehicle_dynamics import VehicleParameters
from stanley_controller.simulation import SimulationConfig, ScenarioGenerator, BatchEvaluator
from stanley_controller.stanley_controller import ControlParams


def run_comprehensive_test():
    """Run comprehensive test suite for Stanley controller."""
    print("Starting Comprehensive Stanley Controller Test Suite...")
    
    # Test configurations
    test_scenarios = [
        ("Straight Line", ScenarioGenerator.straight_line_scenario(100.0)),
        ("Circular Path", ScenarioGenerator.circular_scenario(20.0, 100)),
        ("Figure Eight", ScenarioGenerator.figure_eight_scenario(20.0, 200)),
        ("Highway", ScenarioGenerator.highway_scenario(150.0, 3, 3.5)),
    ]
    
    target_speeds = [3.0, 5.0, 8.0, 10.0]  # m/s
    initial_states = [
        SE2(x=0.0, y=0.0, theta=0.0),
        SE2(x=-10.0, y=5.0, theta=np.radians(30.0)),
        SE2(x=5.0, y=-5.0, theta=np.radians(-45.0)),
    ]
    
    # Create simulation configuration
    config = SimulationConfig(
        dt=0.1,
        max_time=60.0,
        collision_check=True,
        collision_safety_margin=0.5,
        goal_tolerance=2.0
    )
    
    # Create Stanley controller
    control_params = ControlParams(
        k_cross_track=0.5,
        k_heading=1.0,
        max_steer_angle=np.radians(30.0),
        wheelbase=2.9,
        lookahead_distance=5.0
    )
    controller = StanleyController(control_params)
    
    # Create batch evaluator
    evaluator = BatchEvaluator(config)
    
    # Run evaluation
    print(f"Running {len(test_scenarios)} scenarios with {len(target_speeds)} speeds and {len(initial_states)} initial states each...")
    print(f"Total simulations: {len(test_scenarios) * len(target_speeds) * len(initial_states)}")
    
    results = evaluator.evaluate_scenarios(
        controller, test_scenarios, target_speeds, initial_states
    )
    
    # Print summary
    print("\n" + "="*60)
    print("COMPREHENSIVE TEST RESULTS")
    print("="*60)
    
    summary = evaluator.summarize_results()
    print(f"Total simulations: {summary['total_scenarios']}")
    print(f"Successful scenarios: {summary['successful_scenarios']}")
    print(f"Collisions: {summary['collisions']}")
    print(f"Average simulation time: {summary['average_time']:.2f} s")
    print(f"Average final distance to goal: {summary['average_final_distance']:.2f} m")
    print(f"Average tracking error: {summary['average_tracking_error']:.2f} m")
    
    # Print scenario-specific results
    print("\nScenario Breakdown:")
    print("-" * 40)
    for scenario, data in summary['scenario_summary'].items():
        print(f"{scenario:15} | Success: {data['success_rate']:.2f} | "
              f"Avg Time: {data['avg_time']:.2f}s | Avg Error: {data['avg_error']:.2f}m")
    
    # Create visualizations
    visualizer = Visualizer()
    
    # Plot trajectory comparison
    print("\nGenerating trajectory comparison plots...")
    scenarios = [s[0] for s in test_scenarios]
    visualizer.plot_trajectory_comparison(results, scenarios, 
                                       save_path="stanley_controller/results/trajectory_comparison.png")
    
    # Plot performance metrics
    print("Generating performance metrics plots...")
    visualizer.plot_performance_metrics(results, 
                                     save_path="stanley_controller/results/performance_metrics.png")
    
    # Create comprehensive dashboard
    print("Generating comprehensive dashboard...")
    visualizer.create_dashboard(results, 
                             save_path="stanley_controller/results/comprehensive_dashboard.png")
    
    # Detailed analysis for each scenario
    print("\nGenerating detailed analysis for each scenario...")
    os.makedirs("stanley_controller/results/detailed_analysis", exist_ok=True)
    
    for scenario_name, (path_points, path_yaw) in test_scenarios:
        scenario_results = [r for r in results if r['scenario'] == scenario_name]
        
        if scenario_results:
            # Get best and worst performing runs
            best_result = min(scenario_results, key=lambda x: x['metrics']['average_distance_error'])
            worst_result = max(scenario_results, key=lambda x: x['metrics']['average_distance_error'])
            
            # Plot best trajectory
            plt.figure(figsize=(12, 8))
            
            # Plot reference path
            plt.plot(path_points[:, 0], path_points[:, 1], 'r--', linewidth=2, label='Reference Path')
            
            # Plot obstacles if any
            if 'obstacles' in best_result:
                for obs in best_result['obstacles']:
                    circle = Circle((obs['x'], obs['y']), obs['radius'], color='red', alpha=0.5)
                    plt.gca().add_patch(circle)
            
            # Plot best trajectory
            best_traj = best_result['trajectory']
            plt.plot(best_traj[:, 0], best_traj[:, 1], 'g-', linewidth=2, label='Best Trajectory')
            
            # Plot worst trajectory
            worst_traj = worst_result['trajectory']
            plt.plot(worst_traj[:, 0], worst_traj[:, 1], 'orange', linewidth=2, label='Worst Trajectory')
            
            plt.xlabel('X (m)')
            plt.ylabel('Y (m)')
            plt.title(f'{scenario_name} - Best vs Worst Performance')
            plt.legend()
            plt.grid(True, alpha=0.3)
            plt.axis('equal')
            plt.tight_layout()
            plt.savefig(f"stanley_controller/results/detailed_analysis/{scenario_name}_comparison.png", 
                       dpi=300, bbox_inches='tight')
            plt.show()
            
            # Plot error analysis for best result
            visualizer.plot_error_analysis(best_result, 
                                        save_path=f"stanley_controller/results/detailed_analysis/{scenario_name}_best_errors.png")
            
            # Create animation for best result
            visualizer.animate_simulation(best_result, reference_path=path_yaw,
                                       save_path=f"stanley_controller/results/detailed_analysis/{scenario_name}_animation.gif")
    
    # Speed analysis
    print("\nGenerating speed analysis...")
    speed_results = {}
    for speed in target_speeds:
        speed_results[speed] = [r for r in results if r['target_speed'] == speed]
    
    plt.figure(figsize=(12, 8))
    
    for i, (speed, speed_result_list) in enumerate(speed_results.items()):
        if speed_result_list:
            errors = [r['metrics']['average_distance_error'] for r in speed_result_list]
            success_rates = [sum(1 for r in speed_result_list if r['success']) / len(speed_result_list)]
            
            plt.subplot(2, 2, 1)
            plt.bar([speed], [np.mean(errors)], alpha=0.7, label=f'{speed} m/s')
            plt.xlabel('Target Speed (m/s)')
            plt.ylabel('Average Tracking Error (m)')
            plt.title('Tracking Error vs Speed')
            plt.grid(True, alpha=0.3)
            
            plt.subplot(2, 2, 2)
            plt.bar([speed], [success_rates[0]], alpha=0.7, label=f'{speed} m/s')
            plt.xlabel('Target Speed (m/s)')
            plt.ylabel('Success Rate')
            plt.title('Success Rate vs Speed')
            plt.grid(True, alpha=0.3)
            plt.ylim(0, 1)
    
    plt.tight_layout()
    plt.savefig("stanley_controller/results/speed_analysis.png", dpi=300, bbox_inches='tight')
    plt.show()
    
    # Controller parameter sensitivity analysis
    print("\nGenerating controller parameter sensitivity analysis...")
    
    # Test different cross-track error gains
    k_values = [0.1, 0.3, 0.5, 0.7, 1.0]
    k_errors = []
    
    for k in k_values:
        # Create controller with different k value
        test_params = ControlParams(
            k_cross_track=k,
            k_heading=1.0,
            max_steer_angle=np.radians(30.0),
            wheelbase=2.9,
            lookahead_distance=5.0
        )
        test_controller = StanleyController(test_params)
        
        # Run a simple test
        config = SimulationConfig(
            dt=0.1,
            max_time=30.0,
            collision_check=False,
            goal_tolerance=1.0
        )
        
        vehicle_params = VehicleParameters()
        vehicle_dynamics = VehicleDynamics(vehicle_params)
        sim_env = SimulationEnvironment(config)
        
        test_result = sim_env.simulate(
            test_controller, vehicle_dynamics, initial_states[0],
            test_scenarios[0][1][0], test_scenarios[0][1][1], target_speeds[0]
        )
        
        if test_result and 'metrics' in test_result:
            k_errors.append(test_result['metrics']['average_distance_error'])
        else:
            k_errors.append(float('inf'))
    
    plt.figure(figsize=(10, 6))
    plt.plot(k_values, k_errors, 'bo-', linewidth=2, markersize=8)
    plt.xlabel('Cross-Track Error Gain (k)')
    plt.ylabel('Average Tracking Error (m)')
    plt.title('Controller Parameter Sensitivity - Cross-Track Gain')
    plt.grid(True, alpha=0.3)
    plt.savefig("stanley_controller/results/parameter_sensitivity.png", dpi=300, bbox_inches='tight')
    plt.show()
    
    print("\n" + "="*60)
    print("COMPREHENSIVE TEST COMPLETED SUCCESSFULLY!")
    print("="*60)
    print(f"Results saved to: stanley_controller/results/")
    print(f"Total simulations run: {len(results)}")
    print(f"Overall success rate: {summary['successful_scenarios']/summary['total_scenarios']:.2%}")
    print(f"Average tracking error: {summary['average_tracking_error']:.2f} m")
    

def simulate_single_scenario(self, controller, path_points, path_yaw, initial_state, target_speed):
    """Helper method to simulate a single scenario."""
    config = SimulationConfig(
        dt=0.1,
        max_time=30.0,
        collision_check=False,
        goal_tolerance=1.0
    )
    
    vehicle_params = VehicleParameters()
    vehicle_dynamics = VehicleDynamics(vehicle_params)
    sim_env = SimulationEnvironment(config)
    
    return sim_env.simulate(controller, vehicle_dynamics, initial_state, 
                          path_points, path_yaw, target_speed)




if __name__ == "__main__":
    # Create results directory
    os.makedirs("stanley_controller/results", exist_ok=True)
    os.makedirs("stanley_controller/results/detailed_analysis", exist_ok=True)
    
    run_comprehensive_test()