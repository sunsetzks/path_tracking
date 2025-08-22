#!/usr/bin/env python3
"""
Custom Frame Example for DNA Helix Demo

This example shows how to customize the frame_id for the DNA helix visualization,
demonstrating the flexibility of the frame system in forglove-helper.

By default, PlotUtils uses 'map' as the frame_id, but you can customize this
to organize your visualizations better.

Author: Generated for path_tracking project
Date: 2025-01-27
"""

# Import the DNA helix demo
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))

from dna_helix_demo import DNAHelixDemo

class CustomFrameDNAHelixDemo(DNAHelixDemo):
    """
    Custom frame DNA helix demo that uses a different frame_id
    """

    def __init__(self):
        """Initialize with custom frame"""
        # Call parent constructor first
        super().__init__()

        # Override the frame_id with a custom value
        self.frame_id = "biology_lab"  # Custom frame for biology visualizations

        # You could also use other common frame names:
        # self.frame_id = "world"       # Global world frame
        # self.frame_id = "robot_base"  # Robot coordinate frame
        # self.frame_id = "sensor"      # Sensor coordinate frame
        # self.frame_id = "odom"        # Odometry frame

if __name__ == "__main__":
    # This would run the demo with the custom frame "biology_lab"
    # Instead of the default "dna_demo"
    demo = CustomFrameDNAHelixDemo()
    print(f"DNA Helix Demo will use frame_id: '{demo.frame_id}'")
    print("This allows you to organize different visualizations in separate coordinate frames")
    print("Run with: python3 examples/custom_frame_example.py")
