#!/usr/bin/env python3
"""
Enhanced CSV Loader Example for Performance Diagnostics

This script demonstrates how to load and display diagnostic data from CSV files
stored in the log/csv directory. It lists available CSV files and allows users
to select which one to analyze.
"""

import os
import sys
import glob
from datetime import datetime
from typing import List, Optional

# Add the parent directory to the path so we can import modules from PathTracking
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from PathTracking.performance_diagnostics import PerformanceDiagnostics


def get_csv_files_info(csv_dir: str) -> List[dict]:
    """
    Get information about available CSV files in the directory.
    
    Args:
        csv_dir (str): Directory containing CSV files
        
    Returns:
        List[dict]: List of dictionaries containing file information
    """
    csv_files = []
    
    if not os.path.exists(csv_dir):
        return csv_files
    
    # Find all CSV files in the directory
    pattern = os.path.join(csv_dir, "*.csv")
    files = glob.glob(pattern)
    
    for file_path in files:
        try:
            # Get file stats
            stat = os.stat(file_path)
            file_size = stat.st_size
            mod_time = datetime.fromtimestamp(stat.st_mtime)
            
            # Try to get basic info about the CSV content
            data_points = 0
            duration = 0.0
            
            # Quick scan to get data points count and duration
            with open(file_path, 'r') as f:
                lines = f.readlines()
                data_points = len(lines) - 1  # Subtract header row
                
                if data_points > 0:
                    # Get first and last time values
                    try:
                        first_line = lines[1].split(',')
                        last_line = lines[-1].split(',')
                        start_time = float(first_line[0])
                        end_time = float(last_line[0])
                        duration = end_time - start_time
                    except (IndexError, ValueError):
                        duration = 0.0
            
            csv_files.append({
                'filename': os.path.basename(file_path),
                'full_path': file_path,
                'size': file_size,
                'modified': mod_time,
                'data_points': data_points,
                'duration': duration
            })
            
        except Exception as e:
            print(f"Warning: Could not read file {file_path}: {e}")
    
    # Sort by modification time (newest first)
    csv_files.sort(key=lambda x: x['modified'], reverse=True)
    
    return csv_files


def format_file_size(size_bytes: int) -> str:
    """Format file size in human readable format."""
    if size_bytes < 1024:
        return f"{size_bytes} B"
    elif size_bytes < 1024 * 1024:
        return f"{size_bytes / 1024:.1f} KB"
    else:
        return f"{size_bytes / (1024 * 1024):.1f} MB"


def display_csv_files_menu(csv_files: List[dict]) -> None:
    """
    Display a menu of available CSV files with their information.
    
    Args:
        csv_files (List[dict]): List of CSV file information
    """
    print("\n📋 Available CSV Files:")
    print("=" * 80)
    print(f"{'#':<3} {'Filename':<35} {'Size':<10} {'Points':<8} {'Duration':<10} {'Modified':<20}")
    print("-" * 80)
    
    for i, file_info in enumerate(csv_files, 1):
        print(f"{i:<3} {file_info['filename']:<35} "
              f"{format_file_size(file_info['size']):<10} "
              f"{file_info['data_points']:<8} "
              f"{file_info['duration']:.1f}s{'':<6} "
              f"{file_info['modified'].strftime('%Y-%m-%d %H:%M'):<20}")
    
    print("-" * 80)


def get_user_choice(csv_files: List[dict]) -> Optional[dict]:
    """
    Get user's choice of CSV file to analyze.
    
    Args:
        csv_files (List[dict]): List of available CSV files
        
    Returns:
        Optional[dict]: Selected file information or None if cancelled
    """
    while True:
        try:
            choice = input(f"\nEnter file number (1-{len(csv_files)}) or 'q' to quit: ").strip()
            
            if choice.lower() == 'q':
                return None
            
            file_index = int(choice) - 1
            if 0 <= file_index < len(csv_files):
                return csv_files[file_index]
            else:
                print(f"❌ Invalid choice. Please enter a number between 1 and {len(csv_files)}")
                
        except ValueError:
            print("❌ Invalid input. Please enter a number or 'q' to quit.")


def analyze_csv_file(file_info: dict) -> None:
    """
    Analyze and display the selected CSV file.
    
    Args:
        file_info (dict): Information about the selected CSV file
    """
    print(f"\n🔍 Analyzing: {file_info['filename']}")
    print("=" * 60)
    
    # Load and display the data
    diagnostics = PerformanceDiagnostics.load_and_display(
        filename=file_info['full_path'],
        show_summary=True,
        show_charts=True,
        save_chart_path=None  # Display interactively
    )
    
    if diagnostics:
        print(f"\n✅ Analysis complete for {file_info['filename']}")
        
        # Ask if user wants to save charts
        save_charts = input("\nSave diagnostic charts to file? (y/n): ").strip().lower()
        if save_charts == 'y':
            chart_filename = file_info['filename'].replace('.csv', '_charts.png')
            chart_path = os.path.join(os.path.dirname(file_info['full_path']), chart_filename)
            diagnostics.plot_diagnostic_charts(save_path=chart_path)
            print(f"📊 Charts saved to: {chart_path}")
    else:
        print(f"❌ Failed to analyze {file_info['filename']}")


def main():
    """
    Main function to run the enhanced CSV loader example.
    """
    print("🔍 Enhanced Performance Diagnostics CSV Loader")
    print("=" * 60)
    
    # Get the project root directory
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    csv_dir = os.path.join(project_root, 'log', 'csv')
    
    print(f"📁 Looking for CSV files in: {csv_dir}")
    
    # Get available CSV files
    csv_files = get_csv_files_info(csv_dir)
    
    if not csv_files:
        print("\n❌ No CSV files found in the log/csv directory.")
        print("\n💡 To create CSV files:")
        print("1. Run a simulation that exports diagnostic data")
        print("2. The CSV files will be automatically saved to log/csv/")
        print("\n📝 Example:")
        print("```python")
        print("# In your simulation code:")
        print("diagnostics = PerformanceDiagnostics()")
        print("# ... run simulation and collect data ...")
        print("diagnostics.export_data_to_csv('my_simulation_data.csv')  # Saves to log/csv/")
        print("```")
        
        # Check if there are CSV files in the project root
        root_csv_files = glob.glob(os.path.join(project_root, "*.csv"))
        if root_csv_files:
            print(f"\n📋 Found {len(root_csv_files)} CSV files in project root:")
            for csv_file in root_csv_files:
                print(f"   • {os.path.basename(csv_file)}")
            
            move_files = input("\nMove these files to log/csv/? (y/n): ").strip().lower()
            if move_files == 'y':
                os.makedirs(csv_dir, exist_ok=True)
                for csv_file in root_csv_files:
                    new_path = os.path.join(csv_dir, os.path.basename(csv_file))
                    os.rename(csv_file, new_path)
                    print(f"   Moved: {os.path.basename(csv_file)} → log/csv/")
                
                # Refresh the CSV files list
                csv_files = get_csv_files_info(csv_dir)
                if csv_files:
                    print(f"\n✅ Found {len(csv_files)} CSV files after moving!")
                else:
                    return
            else:
                return
        else:
            return
    
    print(f"\n✅ Found {len(csv_files)} CSV files")
    
    # Main interaction loop
    while True:
        # Display menu
        display_csv_files_menu(csv_files)
        
        # Get user choice
        selected_file = get_user_choice(csv_files)
        
        if selected_file is None:
            print("\n👋 Goodbye!")
            break
        
        # Analyze the selected file
        analyze_csv_file(selected_file)
        
        # Ask if user wants to analyze another file
        continue_analysis = input("\nAnalyze another file? (y/n): ").strip().lower()
        if continue_analysis != 'y':
            print("\n👋 Analysis session complete!")
            break


if __name__ == "__main__":
    main() 