#!/usr/bin/env python3
"""
Channel System Demo

This example demonstrates the new channel-based architecture for Foxglove visualization.
It shows how to use different channel types and the channel manager.

Features demonstrated:
- ChannelManager usage
- Different channel types (Scene, Data, TF, Grid, PointCloud, Laser, Proto)
- Custom protobuf message channels (Person messages)
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
from foxglove.schemas import SceneEntity, SceneUpdate

# Import protobuf messages
try:
    from forglove_helper.protos.custom_person_pb2 import Person
    PROTOBUF_AVAILABLE = True
except ImportError:
    print("Warning: Custom protobuf messages not available")
    PROTOBUF_AVAILABLE = False


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
        
        # 3. Transform channels for coordinate frames
        # Dynamic transforms (robot movement)
        self.tf_channel = self.manager.create_tf_channel(
            "/tf",
            description="Dynamic coordinate transformations"
        )
        
        # Static transforms (sensor mounting positions)
        self.tf_static_channel = self.manager.create_tf_channel(
            "/tf_static", 
            description="Static coordinate transformations"
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

        # 8. Proto channel for custom protobuf messages (Person)
        if PROTOBUF_AVAILABLE:
            self.proto_channel = self.manager.create_proto_channel(
                "/demo/person",
                proto_class=Person,
                description="Custom Person protobuf messages"
            )
        else:
            print("⚠ Proto channel not created - protobuf messages not available")
            self.proto_channel = None

        print(f"✓ Created {len(self.manager)} channels")
        
    def publish_static_data(self):
        """Publish static data that doesn't change over time"""
        print("Publishing static data...")
        
        # Publish occupancy grid
        test_grid = GridUtils.create_simple_test_grid(size=100, resolution=0.1)
        self.grid_channel.publish(test_grid)
        
        # Publish static transforms (sensor mounting positions)
        self._publish_static_transforms(initial=True)
        
        print("✓ Static data published")
        
    def _publish_static_transforms(self, initial: bool = False):
        """Publish static transforms to /tf_static"""
        # Create a list of static transforms for all sensors
        transforms = [
            # LiDAR sensor transform (mounted on top of robot)
            TransformUtils.create_translation_transform(
                "base_link", "lidar", 0.0, 0.0, 0.5
            ),
            # Camera sensor transform (mounted in front of robot, slightly above base)
            TransformUtils.create_translation_transform(
                "base_link", "camera", 0.3, 0.0, 0.2
            ),
            # Laser scanner transform (2D laser at base level, front of robot)
            TransformUtils.create_translation_transform(
                "base_link", "laser", 0.2, 0.0, 0.1
            ),
        ]
        # Publish all transforms at once using a list
        self.tf_static_channel.publish(transforms)
        # self.tf_channel.publish(transforms)

        # Log republishing (avoid spam)
        if not initial:
            # Only log if enough time has passed since last log
            if hasattr(self, '_last_static_log_time'):
                if time.time() - self._last_static_log_time > 10:  # Log every 10 seconds max
                    print("🔄 Static transforms republished")
                    self._last_static_log_time = time.time()
            else:
                print("🔄 Static transforms republished")
                self._last_static_log_time = time.time()

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
        entities.append(SceneEntity(id="moving_sphere", frame_id="map", spheres=[sphere]))
        
        # Rotating arrow
        arrow = PrimitiveUtils.create_arrow(
            position=(0.0, 0.0, 0.3),
            direction=t,
            length=1.5,
            color=(0.0, 1.0, 1.0, 1.0)
        )
        entities.append(SceneEntity(id="rotating_arrow", frame_id="map", arrows=[arrow]))
        
        # Oscillating cube
        cube = PrimitiveUtils.create_cube(
            position=(1.0, 0.0, 0.5 + 0.3 * math.sin(t * 2)),
            size=(0.5, 0.5, 0.5),
            color=(0.0, 1.0, 0.0, 1.0)
        )
        entities.append(SceneEntity(id="bouncing_cube", frame_id="map", cubes=[cube]))
        
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
        entities.append(SceneEntity(id="trajectory", frame_id="map", lines=[line]))
        
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
        
        # 3. Update dynamic robot transform
        # Robot position follows circular path
        robot_x = 3 * math.cos(t)
        robot_y = 3 * math.sin(t)
        robot_z = 0.0
        
        # Robot orientation - facing the direction of movement (tangent to circle)
        robot_yaw = t + math.pi/2  # tangent direction to the circular path
        
        # Create dynamic robot transform (map -> base_link)
        qx, qy, qz, qw = TransformUtils.euler_to_quaternion(0.0, 0.0, robot_yaw)
        
        from foxglove.schemas import FrameTransform, Vector3, Quaternion, Timestamp
        robot_transform = FrameTransform(
            timestamp=Timestamp.now(),
            parent_frame_id="map",
            child_frame_id="base_link",
            translation=Vector3(x=robot_x, y=robot_y, z=robot_z),
            rotation=Quaternion(x=qx, y=qy, z=qz, w=qw)
        )
        
        # Publish only the dynamic transform to /tf
        self.tf_channel.publish(robot_transform)
        
        # Republish static transforms periodically (every 5 seconds)
        # This ensures newly connected Foxglove instances receive static transforms
        if int(t) % 5 == 0 and t - int(t) < 0.1:
            # self._publish_static_transforms()
            ...
        
        # 4. Update point cloud (every second) - published in lidar frame
        if int(t) % 2 == 0 and t - int(t) < 0.1:
            test_pointcloud = PointCloudUtils.create_test_point_cloud(
                n_points=2000, radius=8.0
            )
            # Note: frame_id is already set to "lidar" by the utility function
            self.pointcloud_channel.publish(test_pointcloud)
        
        # 5. Update laser scan (every 0.5 seconds) - published in laser frame
        if int(t * 2) % 1 == 0 and (t * 2) - int(t * 2) < 0.05:
            test_laser = LaserScanUtils.create_test_laser_scan(
                n_rays=360, max_range=10.0
            )
            # Note: frame_id is already set to "laser" by the utility function
            self.laser_channel.publish(test_laser)
        
        # 6. Log status occasionally
        if int(t) != int(t - 0.1):  # Once per second
            self.log_channel.publish(f"Demo running for {t:.1f} seconds")

        # 7. Update proto Person data (every 0.5 seconds)
        if int(t * 2) % 1 == 0 and (t * 2) - int(t * 2) < 0.05:
            if self.proto_channel is not None:
                # Create a dynamic Person message
                person_id = int(t) % 10 + 1  # Cycle through IDs 1-10
                person = Person()
                person.name = f"Demo Person {person_id}"  # type: ignore
                person.id = person_id  # type: ignore
                person.email = f"person{person_id}@demo.example.com"  # type: ignore

                self.proto_channel.publish(person)
    
    async def run_demo(self, duration: float = 10.0):
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

        time.sleep(3)
        
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
        print("  - Transforms: Subscribe to /tf (dynamic) and /tf_static (static)")
        print("    Note: Static transforms republish every 5s for late-connecting clients")
        if self.proto_channel is not None:
            print("  - Raw Messages: Subscribe to /demo/person (Person protobuf messages)")
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
    

    
    # Create and run demo
    demo = ChannelDemo(port=8765)
    
    try:
        asyncio.run(demo.run_demo(duration=1000.0))
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
