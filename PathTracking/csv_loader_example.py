#!/usr/bin/env python3
"""
CSV Loader Example for Performance Diagnostics

This script demonstrates how to load and display diagnostic data from CSV files.
It shows different ways to use the new CSV loading functionality.
"""

import os
import sys

# Add the parent directory to the path so we can import modules from PathTracking
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PathTracking.performance_diagnostics import PerformanceDiagnostics


def main():
    """
    Demonstrate CSV loading functionality with different usage patterns.
    """
    print("🔍 Performance Diagnostics CSV Loader Example")
    print("=" * 60)
    
    # Example CSV filename (you would replace this with your actual CSV file)
    csv_filename = "/Users/kuisongzheng/ws/path_tracking/final_diagnostic_data_328.csv"
    
    print(f"\n📁 Looking for CSV file: {csv_filename}")
    
    # Check if the file exists
    if not os.path.exists(csv_filename):
        print(f"❌ CSV file '{csv_filename}' not found.")
        print("\n💡 To use this example:")
        print("1. First run a simulation that exports diagnostic data")
        print("2. Make sure the CSV file exists in the current directory")
        print("3. Update the 'csv_filename' variable with your actual file path")
        print("\n📝 Example of how to create a CSV file:")
        print("```python")
        print("# In your simulation code:")
        print("diagnostics = PerformanceDiagnostics()")
        print("# ... run simulation and collect data ...")
        print("diagnostics.export_data_to_csv('my_simulation_data.csv')")
        print("```")
        return
    
    print("\n" + "="*60)
    print("METHOD 1: Using the class method (recommended)")
    print("="*60)
    
    # Method 1: Using the class method (one-liner approach)
    diagnostics = PerformanceDiagnostics.load_and_display(
        filename=csv_filename,
        show_summary=True,
        show_charts=True,
        save_chart_path="loaded_diagnostics_charts.png"  # Optional: save charts
    )
    
    if diagnostics:
        print("✅ Data loaded and displayed successfully using class method!")
    
    print("\n" + "="*60)
    print("METHOD 2: Using instance methods (step-by-step)")
    print("="*60)
    
    # Method 2: Using instance methods (step-by-step approach)
    diagnostics2 = PerformanceDiagnostics()
    
    # Load the data
    if diagnostics2.load_from_csv(csv_filename):
        print("✅ Data loaded successfully!")
        
        # Display basic info
        print(f"📊 Loaded {len(diagnostics2.history)} data points")
        
        # Show summary
        print("\n📋 Diagnostic Summary:")
        print(diagnostics2.get_diagnostic_summary())
        
        # Generate charts
        print("\n📈 Generating charts...")
        diagnostics2.plot_diagnostic_charts()
        
        # Optionally save charts
        diagnostics2.plot_diagnostic_charts(save_path="method2_charts.png")
        
    else:
        print("❌ Failed to load data using instance method")
    
    print("\n" + "="*60)
    print("METHOD 3: Load without displaying (data analysis only)")
    print("="*60)
    
    # Method 3: Load data for analysis without immediate display
    diagnostics3 = PerformanceDiagnostics()
    
    if diagnostics3.load_from_csv(csv_filename):
        print("✅ Data loaded for analysis!")
        
        # Access specific statistics
        stats = diagnostics3.stats
        print(f"\n📊 Key Performance Metrics:")
        print(f"   Average Velocity: {stats['average_velocity']:.2f} m/s")
        print(f"   Max Lateral Error: {stats['lateral_error']['max']:.3f} m")
        print(f"   Direction Conflicts: {stats['direction_conflicts']}")
        print(f"   Total Distance: {stats['total_distance']:.2f} m")
        
        # Access raw data for custom analysis
        if diagnostics3.history:
            print(f"\n🔍 Sample Data Point (first):")
            first_point = diagnostics3.history[0]
            print(f"   Time: {first_point.time:.2f}s")
            print(f"   Position: ({first_point.position_x:.2f}, {first_point.position_y:.2f})")
            print(f"   Velocity: {first_point.actual_velocity:.2f} m/s")
            print(f"   Lateral Error: {first_point.lateral_error:.3f} m")
        
        # Custom display with specific options
        diagnostics3.display_loaded_data(
            show_summary=False,  # Skip summary since we already showed custom stats
            show_charts=True,
            save_chart_path="analysis_charts.png"
        )
    
    print("\n" + "="*60)
    print("🎉 CSV Loading Demo Complete!")
    print("="*60)
    print("\n💡 Tips for using CSV loading:")
    print("• Use Method 1 for quick visualization of exported data")
    print("• Use Method 2 for more control over the display process")
    print("• Use Method 3 for custom analysis and processing")
    print("• All methods preserve the original data structure and statistics")
    print("• Charts can be saved to files for reports or documentation")


if __name__ == "__main__":
    main() 