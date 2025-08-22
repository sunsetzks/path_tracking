#!/usr/bin/env python3
"""
Channel System Demo

This example demonstrates the new channel-based architecture for Foxglove visualization.
It shows how to use different channel types and the channel manager.

Features demonstrated:
- ChannelManager usage
- Different channel types (Scene, Data, TF, Grid, PointCloud, Laser)
- Utility functions for creating complex data
- Real-time updates and management

Run this example and connect Foxglove Studio to ws://localhost:8765 to see the visualization.

Author: Generated for path_tracking project
Date: 2025-01-27
"""

import asyncio
import math
import time
import numpy as np
from typing import List

# Import the new channel system
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from forglove_helper.channel_manager import ChannelManager, ChannelType
from forglove_helper.channel_utils import (
    TransformUtils, PrimitiveUtils, GridUtils, PointCloudUtils, LaserScanUtils
)
from forglove_helper.channels import SceneUpdateChannel

# Import Foxglove schemas
try:
    from foxglove.schemas import SceneEntity, SceneUpdate
    FOXGLOVE_AVAILABLE = True
except ImportError:
    print("Error: Foxglove SDK not available. Please install with: pip install foxglove-sdk")
    FOXGLOVE_AVAILABLE = False
    sys.exit(1)


class ChannelDemo:
    """
    Demonstration of the new channel-based architecture
    """
    
    def __init__(self, port: int = 8765):
        """Initialize the demo with a channel manager"""
        self.manager = ChannelManager(port=port, auto_start_server=False)
        self.demo_start_time = 0
        
    def setup_channels(self):
        """Set up various types of channels for the demo"""
        print("Setting up channels...")
        
        # 1. Scene channel for 3D visualization
        self.scene_channel = self.manager.create_scene_channel(
            "/demo/scene",
            description="Main 3D scene with animated objects"
        )
        
        # 2. Data channel for telemetry
        self.data_channel = self.manager.create_data_channel(
            "/demo/telemetry",
            description="Vehicle telemetry data"
        )
        
        # 3. Transform channel for coordinate frames
        self.tf_channel = self.manager.create_tf_channel(
            "/tf",
            description="Coordinate transformations"
        )
        
        # 4. Grid channel for occupancy map
        self.grid_channel = self.manager.create_grid_channel(
            "/demo/grid",
            description="Occupancy grid map"
        )
        
        # 5. Point cloud channel
        self.pointcloud_channel = self.manager.create_pointcloud_channel(
            "/demo/pointcloud",
            description="LiDAR point cloud data"
        )
        
        # 6. Laser scan channel
        self.laser_channel = self.manager.create_laser_channel(
            "/demo/scan",
            description="2D laser scan data"
        )
        
        # 7. Log channel for messages
        self.log_channel = self.manager.create_log_channel(
            "/demo/logs",
            description="Demo log messages"
        )
        
        print(f"✓ Created {len(self.manager)} channels")
        
    def publish_static_data(self):
        """Publish static data that doesn't change over time"""
        print("Publishing static data...")
        
        # Publish occupancy grid
        test_grid = GridUtils.create_simple_test_grid(size=100, resolution=0.1)
        self.grid_channel.publish(test_grid)
        
        # Publish coordinate transforms
        # Base to map transform
        base_to_map = TransformUtils.create_translation_transform(
            "map", "base_link", 0.0, 0.0, 0.0
        )
        self.tf_channel.publish(base_to_map)
        
        # Sensor transforms
        lidar_transform = TransformUtils.create_translation_transform(
            "base_link", "lidar", 0.0, 0.0, 0.5
        )
        self.tf_channel.publish(lidar_transform)
        
        camera_transform = TransformUtils.create_rotation_transform(
            "base_link", "camera", 0.0, 0.1, 0.0
        )
        self.tf_channel.publish(camera_transform)
        
        print("✓ Static data published")
        
    def update_dynamic_data(self, t: float):
        """Update dynamic data that changes over time"""
        
        # 1. Update 3D scene with animated objects
        entities = []
        
        # Moving sphere
        sphere = PrimitiveUtils.create_sphere(
            position=(3 * math.cos(t), 3 * math.sin(t), 0.5),
            radius=0.3,
            color=(1.0, 0.5, 0.0, 1.0)
        )
        entities.append(SceneEntity(id="moving_sphere", spheres=[sphere]))
        
        # Rotating arrow
        arrow = PrimitiveUtils.create_arrow(
            position=(0.0, 0.0, 0.3),
            direction=t,
            length=1.5,
            color=(0.0, 1.0, 1.0, 1.0)
        )
        entities.append(SceneEntity(id="rotating_arrow", arrows=[arrow]))
        
        # Oscillating cube
        cube = PrimitiveUtils.create_cube(
            position=(1.0, 0.0, 0.5 + 0.3 * math.sin(t * 2)),
            size=(0.5, 0.5, 0.5),
            color=(0.0, 1.0, 0.0, 1.0)
        )
        entities.append(SceneEntity(id="bouncing_cube", cubes=[cube]))
        
        # Trajectory line
        points = []
        for i in range(50):
            angle = t - i * 0.1
            points.append((3 * math.cos(angle), 3 * math.sin(angle), 0.1))
        
        line = PrimitiveUtils.create_line(
            points=points,
            thickness=0.02,
            color=(1.0, 1.0, 0.0, 0.8)
        )
        entities.append(SceneEntity(id="trajectory", lines=[line]))
        
        # Publish scene update
        self.scene_channel.publish(entities)
        
        # 2. Update telemetry data
        telemetry = {
            "timestamp": time.time(),
            "demo_time": t,
            "position": {
                "x": 3 * math.cos(t),
                "y": 3 * math.sin(t),
                "z": 0.5
            },
            "velocity": {
                "linear": 3.0,
                "angular": 1.0
            },
            "battery_level": max(0, 100 - t * 2),
            "temperature": 25 + 5 * math.sin(t * 0.5),
            "status": "running"
        }
        self.data_channel.publish(telemetry)
        
        # 3. Update robot position transform
        robot_transform = TransformUtils.create_translation_transform(
            "map", "base_link",
            3 * math.cos(t), 3 * math.sin(t), 0.0,
            timestamp=time.time()
        )
        self.tf_channel.publish(robot_transform)
        
        # 4. Update point cloud (every second)
        if int(t) % 2 == 0 and t - int(t) < 0.1:
            test_pointcloud = PointCloudUtils.create_test_point_cloud(
                n_points=2000, radius=8.0
            )
            self.pointcloud_channel.publish(test_pointcloud)
        
        # 5. Update laser scan (every 0.5 seconds)
        if int(t * 2) % 1 == 0 and (t * 2) - int(t * 2) < 0.05:
            test_laser = LaserScanUtils.create_test_laser_scan(
                n_rays=360, max_range=10.0
            )
            self.laser_channel.publish(test_laser)
        
        # 6. Log status occasionally
        if int(t) != int(t - 0.1):  # Once per second
            self.log_channel.publish(f"Demo running for {t:.1f} seconds")
    
    async def run_demo(self, duration: float = 60.0):
        """
        Run the demonstration
        
        Args:
            duration: How long to run the demo in seconds
        """
        print(f"Starting channel demo for {duration} seconds...")
        
        # Start the server with MCAP recording
        self.manager.start_server(enable_mcap=True)
        
        # Set up channels
        self.setup_channels()
        
        # Print status
        self.manager.print_status()
        
        # Publish static data
        self.publish_static_data()
        
        print(f"\n🚀 Demo running! Connect Foxglove Studio to ws://localhost:{self.manager.port}")
        print("Recommended panels:")
        print("  - 3D: Subscribe to /demo/scene")
        print("  - Raw Messages: Subscribe to /demo/telemetry")
        print("  - Map: Subscribe to /demo/grid")
        print("  - 3D: Subscribe to /demo/pointcloud")
        print("  - Plot: Subscribe to /demo/scan")
        print("  - Log: Subscribe to /demo/logs")
        print("\nPress Ctrl+C to stop the demo\n")
        
        # Main loop
        self.demo_start_time = time.time()
        try:
            while time.time() - self.demo_start_time < duration:
                t = time.time() - self.demo_start_time
                
                # Update dynamic content
                self.update_dynamic_data(t)
                
                # Print progress occasionally
                if int(t) % 10 == 0 and t - int(t) < 0.1:
                    print(f"Demo progress: {t:.1f}/{duration}s ({self.manager.total_messages_published} messages)")
                
                await asyncio.sleep(0.1)  # 10 Hz update rate
                
        except KeyboardInterrupt:
            print("\nDemo interrupted by user")
        
        print(f"\n✓ Demo completed! Published {self.manager.total_messages_published} messages")
        
        # Final status
        self.manager.print_status()
        
        # Stop the server
        self.manager.stop_server()


def main():
    """Main function to run the demo"""
    print("=== Foxglove Channel System Demo ===\n")
    
    if not FOXGLOVE_AVAILABLE:
        print("Error: Foxglove SDK not available")
        return
    
    # Create and run demo
    demo = ChannelDemo(port=8765)
    
    try:
        asyncio.run(demo.run_demo(duration=60.0))
    except KeyboardInterrupt:
        print("\nDemo interrupted")
    except Exception as e:
        print(f"Demo error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Ensure cleanup
        if demo.manager.is_running:
            demo.manager.stop_server()


if __name__ == "__main__":
    main()
