"""MPC Car Tracking Package.

This package implements Model Predictive Control (MPC) for vehicle trajectory tracking.
It includes vehicle dynamics modeling, MPC controller, trajectory planning, simulation,
and visualization tools.
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Optional, Dict, Any

# Import main components
from .vehicle_model import VehicleState, VehicleControl, BicycleModel, VehicleParameters
from .mpc_controller import MPCController, MPCParameters, MPCResult
from .trajectory_planner import TrajectoryPlanner, TrajectoryPoint
from .simulator import MPCSimulator, SimulationConfig, DisturbanceModel
from .visualization import MPCVisualizer

# Package version
__version__ = "0.1.0"

# Expose main classes
__all__ = [
    'VehicleState', 'VehicleControl', 'BicycleModel', 'VehicleParameters',
    'MPCController', 'MPCParameters', 'MPCResult',
    'TrajectoryPlanner', 'TrajectoryPoint',
    'MPCSimulator', 'SimulationConfig', 'DisturbanceModel',
    'MPCVisualizer',
    'run_demo', 'main'
]


def run_demo(scenario: str = 'circular_track', 
             show_plots: bool = True,
             save_plots: bool = False,
             plot_dir: str = './plots') -> Dict[str, Any]:
    """Run MPC demo with specified scenario.
    
    Args:
        scenario: Scenario name ('straight_line', 'circular_track', 'figure_eight', 
                 'lane_change', 'slalom')
        show_plots: Whether to display plots
        save_plots: Whether to save plots to files
        plot_dir: Directory to save plots
        
    Returns:
        Dictionary containing simulation results and performance metrics
    """
    print(f"Running MPC Car Tracking Demo - Scenario: {scenario}")
    print("=" * 50)
    
    # Create simulation configuration
    config = SimulationConfig(
        dt=0.1,
        total_time=20.0,
        real_time_factor=0.0,  # Run as fast as possible
        save_data=True
    )
    
    # Initialize simulator
    simulator = MPCSimulator(config)
    
    # Run scenario
    print(f"Running {scenario} scenario...")
    try:
        simulation_data = simulator.run_scenario(scenario)
        
        if simulation_data.time:
            print(f"Simulation completed successfully!")
            print(f"Duration: {simulation_data.time[-1]:.1f} seconds")
            print(f"Time steps: {len(simulation_data.time)}")
            
            # Calculate performance metrics
            metrics = simulator.get_performance_metrics()
            
            print("\nPerformance Metrics:")
            print("-" * 30)
            for key, value in metrics.items():
                if 'time' in key.lower():
                    print(f"{key}: {value:.3f} s")
                elif 'error' in key.lower():
                    if 'yaw' in key.lower():
                        print(f"{key}: {value:.3f} rad ({value*180/np.pi:.1f} deg)")
                    else:
                        print(f"{key}: {value:.3f} m")
                elif 'rate' in key.lower():
                    print(f"{key}: {value:.1f}%")
                else:
                    print(f"{key}: {value:.3f}")
            
            # Create visualizations
            if show_plots or save_plots:
                visualizer = MPCVisualizer()
                
                print("\nGenerating visualizations...")
                
                # Trajectory tracking plot
                fig1 = visualizer.plot_trajectory_tracking(
                    simulation_data, 
                    title=f"MPC Tracking - {scenario.replace('_', ' ').title()}",
                    save_path=f"{plot_dir}/trajectory_{scenario}.png" if save_plots else None
                )
                
                # Control inputs plot
                fig2 = visualizer.plot_control_inputs(
                    simulation_data,
                    title=f"Control Inputs - {scenario.replace('_', ' ').title()}",
                    save_path=f"{plot_dir}/controls_{scenario}.png" if save_plots else None
                )
                
                # Vehicle states plot
                fig3 = visualizer.plot_vehicle_states(
                    simulation_data,
                    title=f"Vehicle States - {scenario.replace('_', ' ').title()}",
                    save_path=f"{plot_dir}/states_{scenario}.png" if save_plots else None
                )
                
                # Performance metrics plot
                fig4 = visualizer.plot_performance_metrics(
                    simulation_data,
                    title=f"Performance Metrics - {scenario.replace('_', ' ').title()}",
                    save_path=f"{plot_dir}/performance_{scenario}.png" if save_plots else None
                )
                
                if show_plots:
                    plt.show()
                
                print(f"Visualizations created successfully!")
                if save_plots:
                    print(f"Plots saved to {plot_dir}/")
            
            return {
                'simulation_data': simulation_data,
                'metrics': metrics,
                'success': True
            }
        
        else:
            print("Simulation failed - no data generated")
            return {'success': False, 'error': 'No simulation data'}
            
    except Exception as e:
        print(f"Simulation failed with error: {str(e)}")
        return {'success': False, 'error': str(e)}


def run_multiple_scenarios(scenarios: Optional[list] = None,
                          show_comparison: bool = True,
                          save_plots: bool = False,
                          plot_dir: str = './plots') -> Dict[str, Any]:
    """Run multiple scenarios and compare results.
    
    Args:
        scenarios: List of scenario names to run
        show_comparison: Whether to show comparison plots
        save_plots: Whether to save plots
        plot_dir: Directory to save plots
        
    Returns:
        Dictionary containing results for all scenarios
    """
    if scenarios is None:
        scenarios = ['straight_line', 'circular_track', 'figure_eight', 'lane_change']
    
    print("Running Multiple MPC Scenarios")
    print("=" * 40)
    
    results = {}
    scenario_data = {}
    
    # Run each scenario
    for scenario in scenarios:
        print(f"\n--- Running {scenario} ---")
        result = run_demo(scenario, show_plots=False, save_plots=False)
        results[scenario] = result
        
        if result['success']:
            scenario_data[scenario] = result['simulation_data']
            print(f"✓ {scenario} completed successfully")
        else:
            print(f"✗ {scenario} failed: {result.get('error', 'Unknown error')}")
    
    # Create comparison plot
    if show_comparison and scenario_data:
        print("\nGenerating comparison plots...")
        visualizer = MPCVisualizer(figsize=(14, 10))
        
        fig = visualizer.compare_scenarios(
            scenario_data,
            title="MPC Performance Comparison",
            save_path=f"{plot_dir}/scenario_comparison.png" if save_plots else None
        )
        
        if show_comparison:
            plt.show()
    
    return results


def main() -> None:
    """Main entry point for the package."""
    import argparse
    import os
    
    parser = argparse.ArgumentParser(description='MPC Car Tracking Demo')
    parser.add_argument('--scenario', type=str, default='circular_track',
                       choices=['straight_line', 'circular_track', 'figure_eight', 
                               'lane_change', 'slalom', 'all'],
                       help='Scenario to run')
    parser.add_argument('--no-plots', action='store_true',
                       help='Disable plot display')
    parser.add_argument('--save-plots', action='store_true',
                       help='Save plots to files')
    parser.add_argument('--plot-dir', type=str, default='./plots',
                       help='Directory to save plots')
    
    args = parser.parse_args()
    
    # Create plot directory if saving plots
    if args.save_plots:
        os.makedirs(args.plot_dir, exist_ok=True)
    
    # Run demo(s)
    if args.scenario == 'all':
        results = run_multiple_scenarios(
            show_comparison=not args.no_plots,
            save_plots=args.save_plots,
            plot_dir=args.plot_dir
        )
        
        # Print summary
        print("\n" + "=" * 50)
        print("SUMMARY")
        print("=" * 50)
        
        for scenario, result in results.items():
            if result['success']:
                metrics = result['metrics']
                print(f"{scenario:15s}: RMS Error = {metrics['rms_position_error']:.3f}m, "
                      f"Solve Time = {metrics['mean_solve_time']:.1f}ms")
            else:
                print(f"{scenario:15s}: FAILED")
    
    else:
        result = run_demo(
            scenario=args.scenario,
            show_plots=not args.no_plots,
            save_plots=args.save_plots,
            plot_dir=args.plot_dir
        )
        
        if not result['success']:
            exit(1)


if __name__ == "__main__":
    main()
