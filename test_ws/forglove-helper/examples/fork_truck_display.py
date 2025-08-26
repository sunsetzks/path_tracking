#!/usr/bin/env python3
"""
Fork Truck Display Demo for Foxglove

This example demonstrates how to use the Foxglove helper for fork truck visualization.
It shows a fork truck with configurable dimensions and forks, visualized in 3D using Foxglove Studio.

Features demonstrated:
- Fork truck with body, forks, and wheels
- Configurable robot dimensions and fork positions
- Real-time visualization in Foxglove Studio
- Integration with the forglove-helper package

Run this example and connect Foxglove Studio to ws://localhost:8765 to see the visualization.

Author: PathTracking
Date: 2025-08-26
"""

import asyncio
import math
import time
import numpy as np
import sys
import os

# Add the parent directory to the path to import forglove_helper
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import the forglove-helper modules
from forglove_helper.channel_manager import ChannelManager
from forglove_helper.channel_utils import PlotUtils, PrimitiveUtils

# Import Foxglove schemas
from foxglove.schemas import SceneEntity, SceneUpdate, Timestamp, Color, Vector3, Quaternion, Pose


class FoxgloveForkTruckDisplay:
    """
    Fork truck display class adapted for Foxglove visualization

    This class converts the fork truck model to Foxglove-compatible
    3D primitives that can be visualized in Foxglove Studio.
    """

    def __init__(self):
        """Initialize fork truck with default dimensions"""
        # Robot body dimensions (in meters)
        self.robot_x_min = -1.2
        self.robot_x_max = 1.5
        self.robot_y_min = -0.8
        self.robot_y_max = 0.8
        
        # Fork specifications
        self.fork_length = 1.0
        self.fork_width = 0.15
        self.fork_center_ys = [-0.3, 0.3]  # Y positions of fork centers
        
        # Padding for collision/inflation
        self.padding_front = 0.1
        self.padding_back = 0.1
        self.padding_left = 0.05
        self.padding_right = 0.05
        
        # Wheel parameters
        self.wheel_params = {
            'front_wheel_offset': 0.1,
            'front_wheel_x_size': 0.25,
            'front_wheel_y_size': 0.1
        }

    def get_robot_x_min(self):
        """Get minimum X coordinate with padding"""
        return self.robot_x_min - self.padding_back

    def get_robot_x_max(self):
        """Get maximum X coordinate with padding"""
        return self.robot_x_max + self.padding_front

    def get_robot_y_min(self):
        """Get minimum Y coordinate with padding"""
        return self.robot_y_min - self.padding_right

    def get_robot_y_max(self):
        """Get maximum Y coordinate with padding"""
        return self.robot_y_max + self.padding_left

    def get_robot_fork_length(self):
        """Get fork length with padding"""
        return self.fork_length + self.padding_back

    def get_robot_fork_width(self):
        """Get fork width"""
        return self.fork_width

    def get_robot_fork_center_ys(self):
        """Get fork center Y positions"""
        return self.fork_center_ys.copy()

    def get_robot_fork_center_xs(self):
        """Get fork center X positions"""
        fork_center_x = self.get_robot_x_min() + self.get_robot_fork_length() / 2
        return [fork_center_x] * len(self.get_robot_fork_center_ys())

    def get_robot_fork_centers(self):
        """Get fork center positions"""
        fork_centers_y = self.get_robot_fork_center_ys()
        fork_centers_x = self.get_robot_fork_center_xs()
        return list(zip(fork_centers_x, fork_centers_y))

    def create_fork_truck_entity(
        self,
        x: float,
        y: float,
        theta: float,
        kappa: float = 0.0,
        body_color: tuple = (0.5, 0.5, 0.8, 0.8),
        fork_color: tuple = (0.8, 0.6, 0.4, 0.8),
        wheel_color: tuple = (0.2, 0.2, 0.2, 1.0),
        front_wheel_color: tuple = (0.8, 0.2, 0.2, 1.0),
        include_center_box: bool = True,
        entity_id: str = "fork_truck",
    ) -> SceneEntity:
        """
        Create Foxglove SceneEntity for fork truck visualization

        Args:
            x (float): Robot X position [m]
            y (float): Robot Y position [m]
            theta (float): Robot heading angle [rad]
            kappa (float): Robot curvature [1/m]
            body_color (tuple): RGBA color for truck body
            fork_color (tuple): RGBA color for forks
            wheel_color (tuple): RGBA color for rear wheels
            front_wheel_color (tuple): RGBA color for front wheels
            include_center_box (bool): Whether to include center box
            entity_id (str): Unique identifier for the scene entity

        Returns:
            SceneEntity: Foxglove SceneEntity containing fork truck primitives
        """
        # Calculate wheelbase and steering angle
        wheelbase = self.get_robot_x_max()
        delta = math.atan(kappa * wheelbase) if kappa != 0 else 0.0

        # Create main body
        body_primitive = self._create_main_body_primitive(x, y, theta, body_color)

        # Create forks
        fork_primitives = self._create_fork_primitives(x, y, theta, fork_color)

        # Create wheels
        wheel_primitives = self._create_wheel_primitives(
            x, y, theta, delta, wheel_color, front_wheel_color
        )

        # Create center box if requested
        center_box_primitives = []
        if include_center_box:
            center_box_primitive = self._create_center_box_primitive(x, y, theta)
            center_box_primitives.append(center_box_primitive)

        # Collect all primitives
        cubes = [body_primitive] + fork_primitives + wheel_primitives + center_box_primitives

        # Create scene entity
        scene_entity = SceneEntity(
            id=entity_id,
            timestamp=Timestamp.now(),
            frame_id="map",
            lifetime=None,
            frame_locked=False,
            cubes=cubes,
        )

        return scene_entity

    def _create_main_body_primitive(self, x: float, y: float, theta: float, color: tuple):
        """Create main body primitive"""
        # Calculate body dimensions (excluding forks)
        body_x_min = self.get_robot_x_min() + self.get_robot_fork_length()
        body_x_max = self.get_robot_x_max()
        body_y_min = self.get_robot_y_min()
        body_y_max = self.get_robot_y_max()

        body_width = body_x_max - body_x_min
        body_height = body_y_max - body_y_min

        # Calculate body center in robot frame
        body_center_x_robot = (body_x_max + body_x_min) / 2.0
        body_center_y_robot = (body_y_max + body_y_min) / 2.0

        # Transform to world coordinates
        body_center_x = x + body_center_x_robot * math.cos(theta) - body_center_y_robot * math.sin(theta)
        body_center_y = y + body_center_x_robot * math.sin(theta) + body_center_y_robot * math.cos(theta)

        # Convert yaw to quaternion
        qw = math.cos(theta / 2)
        qz = math.sin(theta / 2)

        return PrimitiveUtils.create_cube(
            position=(body_center_x, body_center_y, 0.0),
            size=(body_width, body_height, 0.2),  # Slightly thicker body
            color=color,
            orientation=(0.0, 0.0, qz, qw)  # Rotation around Z-axis
        )

    def _create_fork_primitives(self, x: float, y: float, theta: float, color: tuple):
        """Create fork primitives"""
        primitives = []

        for fork_center_x_robot, fork_center_y_robot in self.get_robot_fork_centers():
            # Transform fork center to world coordinates
            fork_center_x = x + fork_center_x_robot * math.cos(theta) - fork_center_y_robot * math.sin(theta)
            fork_center_y = y + fork_center_x_robot * math.sin(theta) + fork_center_y_robot * math.cos(theta)

            # Convert yaw to quaternion
            qw = math.cos(theta / 2)
            qz = math.sin(theta / 2)

            # Create fork primitive
            fork_primitive = PrimitiveUtils.create_cube(
                position=(fork_center_x, fork_center_y, 0.0),
                size=(self.get_robot_fork_length(), self.get_robot_fork_width(), 0.05),  # Thin forks
                color=color,
                orientation=(0.0, 0.0, qz, qw)
            )
            primitives.append(fork_primitive)

        return primitives

    def _create_wheel_primitives(self, x: float, y: float, theta: float,
                                delta: float, wheel_color: tuple, front_wheel_color: tuple):
        """Create wheel primitives"""
        primitives = []

        # Front wheel position
        wheel_x_robot = self.get_robot_x_max() - self.wheel_params['front_wheel_offset'] - self.wheel_params['front_wheel_x_size'] / 2.0
        wheel_y_robot = 0.0

        # Transform front wheel to world coordinates
        wheel_x = x + wheel_x_robot * math.cos(theta) - wheel_y_robot * math.sin(theta)
        wheel_y = y + wheel_x_robot * math.sin(theta) + wheel_y_robot * math.cos(theta)

        # Front wheel orientation (includes steering)
        front_wheel_qw = math.cos((theta + delta) / 2)
        front_wheel_qz = math.sin((theta + delta) / 2)

        # Create front wheel
        front_wheel = PrimitiveUtils.create_cube(
            position=(wheel_x, wheel_y, 0.0),
            size=(self.wheel_params['front_wheel_x_size'], self.wheel_params['front_wheel_y_size'], 0.05),
            color=front_wheel_color,
            orientation=(0.0, 0.0, front_wheel_qz, front_wheel_qw)
        )
        primitives.append(front_wheel)

        # Create wheel direction indicator
        indicator_length = self.wheel_params['front_wheel_x_size'] / 2
        indicator_x = wheel_x + indicator_length * math.cos(theta + delta) / 2
        indicator_y = wheel_y + indicator_length * math.sin(theta + delta) / 2

        wheel_indicator = PrimitiveUtils.create_cube(
            position=(indicator_x, indicator_y, 0.01),
            size=(indicator_length, 0.002, 0.01),
            color=(0.0, 0.0, 0.0, 1.0),
            orientation=(0.0, 0.0, front_wheel_qz, front_wheel_qw)
        )
        primitives.append(wheel_indicator)

        return primitives

    def _create_center_box_primitive(self, x: float, y: float, theta: float):
        """Create center box primitive"""
        # Convert yaw to quaternion
        qw = math.cos(theta / 2)
        qz = math.sin(theta / 2)

        return PrimitiveUtils.create_cube(
            position=(x, y, 0.0),
            size=(0.1, 0.1, 0.1),
            color=(1.0, 0.0, 0.0, 1.0),  # Red center box
            orientation=(0.0, 0.0, qz, qw)
        )


class ForkTruckDemo:
    """
    Demonstration of fork truck with Foxglove visualization
    """

    def __init__(self):
        """Initialize the demo"""
        self.channel_manager = ChannelManager(port=8765)
        self.scene_channel = self.channel_manager.create_scene_channel("fork_truck_demo")
        self.fork_truck_display = FoxgloveForkTruckDisplay()
        self.running = True

    async def run_demo(self):
        """Run the fork truck demo"""
        print("Starting Fork Truck Demo...")
        print("Connect Foxglove Studio to ws://localhost:8765")
        print("Press Ctrl+C to stop")

        try:
            # Start the server
            self.channel_manager.start_server()

            # Demo parameters
            time_step = 0.0
            dt = 0.1

            while self.running:
                # Create different poses for continuous demonstration
                cycle_time = time_step % 12.0  # 12 second cycle
                
                if cycle_time < 3.0:
                    # Straight position
                    x, y, theta, kappa = 0.0, 0.0, 0.0, 0.0
                    title = "Fork Truck - Straight Position"
                elif cycle_time < 6.0:
                    # Turning position
                    x, y, theta, kappa = 3.0, 2.0, math.pi/4, 0.2
                    title = "Fork Truck - Turning Right"
                elif cycle_time < 9.0:
                    # 90 degree rotation
                    x, y, theta, kappa = 6.0, 0.0, math.pi/2, -0.1
                    title = "Fork Truck - 90° Rotation"
                else:
                    # Reversing position
                    x, y, theta, kappa = 3.0, -2.0, -math.pi/4, -0.15
                    title = "Fork Truck - Reversing"

                # Create fork truck entity
                fork_truck_entity = self.fork_truck_display.create_fork_truck_entity(
                    x, y, theta, kappa,
                    body_color=(0.2, 0.6, 0.8, 0.8),
                    fork_color=(0.8, 0.6, 0.4, 0.8),
                    wheel_color=(0.2, 0.2, 0.2, 1.0),
                    front_wheel_color=(0.8, 0.2, 0.2, 1.0),
                    include_center_box=True,
                    entity_id="demo_fork_truck"
                )

                # Create scene update
                scene_update = SceneUpdate(deletions=[], entities=[fork_truck_entity])

                # Publish the scene update
                self.scene_channel.publish(scene_update)

                # Print current state
                print(f"\r{title} - X: {x:.1f}, Y: {y:.1f}, θ: {math.degrees(theta):.1f}°, κ: {kappa:.2f}", end="")

                # Update time
                time_step += dt

                # Wait before next update
                await asyncio.sleep(dt)

        except KeyboardInterrupt:
            print("\nStopping demo...")
        except Exception as e:
            print(f"Error in demo: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.channel_manager.stop_server()

    def stop(self):
        """Stop the demo"""
        self.running = False


class AnimatedForkTruckDemo:
    """
    Animated demonstration showing fork truck movement
    """

    def __init__(self):
        """Initialize the demo"""
        self.channel_manager = ChannelManager(port=8765)
        self.scene_channel = self.channel_manager.create_scene_channel("animated_fork_truck")
        self.fork_truck_display = FoxgloveForkTruckDisplay()
        self.running = True

    async def run_demo(self):
        """Run the animated fork truck demo"""
        print("Starting Animated Fork Truck Demo...")
        print("Connect Foxglove Studio to ws://localhost:8765")
        print("Press Ctrl+C to stop")

        try:
            # Start the server
            self.channel_manager.start_server()

            # Animation parameters
            t = 0.0
            dt = 0.1
            speed = 0.5

            while self.running:
                # Create figure-8 motion for more interesting animation
                scale = 8.0
                x = scale * math.sin(t)
                y = scale * math.sin(t) * math.cos(t)
                
                # Calculate heading (tangent to the curve)
                dx_dt = scale * math.cos(t)
                dy_dt = scale * (math.cos(t) * math.cos(t) - math.sin(t) * math.sin(t))
                theta = math.atan2(dy_dt, dx_dt)
                
                # Calculate curvature for realistic steering
                velocity_magnitude = math.sqrt(dx_dt**2 + dy_dt**2)
                if velocity_magnitude > 0:
                    # Curvature calculation for parametric curve
                    d2x_dt2 = -scale * math.sin(t)
                    d2y_dt2 = scale * (-2 * math.sin(t) * math.cos(t))
                    
                    # Curvature formula: κ = |x'y'' - y'x''| / (x'^2 + y'^2)^(3/2)
                    numerator = abs(dx_dt * d2y_dt2 - dy_dt * d2x_dt2)
                    denominator = velocity_magnitude**3
                    kappa = numerator / denominator if denominator > 0 else 0.0
                    
                    # Limit maximum curvature for realistic steering
                    kappa = min(kappa, 0.5)
                else:
                    kappa = 0.0

                # Create fork truck entity with dynamic coloring based on position
                hue = (t / (4 * math.pi)) % 1.0
                body_color = (hue, 0.5, 0.8, 0.8)

                fork_truck_entity = self.fork_truck_display.create_fork_truck_entity(
                    x, y, theta, kappa,
                    body_color=body_color,
                    fork_color=(0.8, 0.6, 0.4, 0.8),
                    wheel_color=(0.2, 0.2, 0.2, 1.0),
                    front_wheel_color=(0.8, 0.2, 0.2, 1.0),
                    include_center_box=True,
                    entity_id="animated_fork_truck"
                )

                # Create scene update
                scene_update = SceneUpdate(deletions=[], entities=[fork_truck_entity])

                # Publish the scene update
                self.scene_channel.publish(scene_update)

                # Print current status
                print(f"\rFigure-8 Motion - X: {x:.1f}, Y: {y:.1f}, θ: {math.degrees(theta):.1f}°, κ: {kappa:.2f}", end="")

                # Update time (continuous animation)
                t += dt * speed

                # Wait before next update
                await asyncio.sleep(dt)

        except KeyboardInterrupt:
            print("\nStopping demo...")
        except Exception as e:
            print(f"Error in demo: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.channel_manager.stop_server()

    def stop(self):
        """Stop the demo"""
        self.running = False


async def main():
    """Main function - choose which demo to run"""
    print("Foxglove Fork Truck Display Demo")
    print("1. Static Pose Demo (cycles through different positions)")
    print("2. Animated Motion Demo (continuous figure-8 motion)")
    print("Choose demo (1 or 2, default=2): ", end="")

    # Default to animated demo if no input
    choice = "2"

    if choice == "1":
        demo = ForkTruckDemo()
        await demo.run_demo()
    else:
        demo = AnimatedForkTruckDemo()
        await demo.run_demo()


if __name__ == "__main__":
    asyncio.run(main())