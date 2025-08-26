#!/usr/bin/env python3
"""
Vehicle Display Demo for Foxglove

This example demonstrates how to use the vehicle display utilities with Foxglove Studio.
It shows a vehicle following a trajectory with realistic steering visualization.

Features demonstrated:
- Converting vehicle display to Foxglove-compatible 3D primitives
- Vehicle following a curved trajectory with steering angles
- Real-time visualization in Foxglove Studio
- Integration with the forglove-helper package

Run this example and connect Foxglove Studio to ws://localhost:8765 to see the visualization.

Author: PathTracking
Date: 2025-01-27
"""

import asyncio
import math
import time
import numpy as np
import sys
import os

# Add the parent directory to the path to import vehicle_display
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

# Import the forglove-helper modules
from forglove_helper.channel_manager import ChannelManager
from forglove_helper.channel_utils import PlotUtils, PrimitiveUtils

# Import Foxglove schemas
from foxglove.schemas import SceneEntity, SceneUpdate, Timestamp, Color, Vector3, Quaternion, Pose

# Import vehicle display module
import matplotlib.patches as patches
import matplotlib.pyplot as plt


class FoxgloveVehicleDisplay:
    """
    Vehicle display class adapted for Foxglove visualization

    This class converts the matplotlib-based vehicle display to Foxglove-compatible
    3D primitives that can be visualized in Foxglove Studio.
    """

    def __init__(
        self,
        vehicle_length: float = 4.5,
        vehicle_width: float = 2.0,
        wheelbase: float = 2.9,
        wheel_length: float = 0.4,
        wheel_width: float = 0.2,
        wheel_track: float = 1.6,
    ):
        """
        Initialize vehicle display parameters

        Args:
            vehicle_length (float): Total vehicle length [m]
            vehicle_width (float): Total vehicle width [m]
            wheelbase (float): Distance between front and rear axles [m]
            wheel_length (float): Wheel length [m]
            wheel_width (float): Wheel width [m]
            wheel_track (float): Distance between left and right wheels [m]
        """
        self.vehicle_length = vehicle_length
        self.vehicle_width = vehicle_width
        self.wheelbase = wheelbase
        self.wheel_length = wheel_length
        self.wheel_width = wheel_width
        self.wheel_track = wheel_track

        # Calculate distances for proper positioning
        self.rear_overhang = (vehicle_length - wheelbase) / 2
        self.front_overhang = self.rear_overhang

    def create_vehicle_entity(
        self,
        x: float,
        y: float,
        yaw: float,
        steering_angle: float = 0.0,
        body_color: tuple = (0.5, 0.5, 0.5, 0.8),
        wheel_color: tuple = (0.0, 0.0, 0.0, 1.0),
        front_wheel_color: tuple = (1.0, 0.0, 0.0, 1.0),
        include_arrows: bool = True,
        entity_id: str = "vehicle",
    ) -> SceneEntity:
        """
        Create Foxglove SceneEntity for vehicle visualization

        Args:
            x (float): Rear wheel center x position [m]
            y (float): Rear wheel center y position [m]
            yaw (float): Vehicle heading angle [rad]
            steering_angle (float): Front wheel steering angle [rad]
            body_color (tuple): RGBA color for vehicle body
            wheel_color (tuple): RGBA color for rear wheels
            front_wheel_color (tuple): RGBA color for front wheels
            include_arrows (bool): Whether to include direction arrows
            entity_id (str): Unique identifier for the scene entity

        Returns:
            SceneEntity: Foxglove SceneEntity containing vehicle primitives
        """
        # Create vehicle body (represented as a flat box)
        body_primitive = self._create_vehicle_body_primitive(x, y, yaw, body_color)

        # Create wheels
        wheel_primitives = self._create_wheel_primitives(
            x, y, yaw, steering_angle, wheel_color, front_wheel_color
        )

        # Create direction arrow if requested
        arrow_primitives = []
        if include_arrows:
            arrow_primitive = self._create_direction_arrow_primitive(x, y, yaw)
            arrow_primitives.append(arrow_primitive)

        # Collect cubes (body and wheels)
        cubes = [body_primitive] + wheel_primitives

        # Collect arrows
        arrows = arrow_primitives

        # Create scene entity
        scene_entity = SceneEntity(
            id=entity_id,
            timestamp=Timestamp.now(),
            frame_id="map",
            lifetime=None,
            frame_locked=False,
            cubes=cubes,
            arrows=arrows,
        )

        return scene_entity

    def _create_vehicle_body_primitive(self, x: float, y: float, yaw: float, color: tuple):
        """Create vehicle body primitive"""
        # Calculate vehicle center position from rear wheel position
        vehicle_center_x = x + (self.wheelbase / 2) * math.cos(yaw)
        vehicle_center_y = y + (self.wheelbase / 2) * math.sin(yaw)

        # Convert yaw to quaternion
        qw = math.cos(yaw / 2)
        qz = math.sin(yaw / 2)

        return PrimitiveUtils.create_cube(
            position=(vehicle_center_x, vehicle_center_y, 0.0),
            size=(self.vehicle_length, self.vehicle_width, 0.1),  # Flat box
            color=color,
            orientation=(0.0, 0.0, qz, qw)  # Rotation around Z-axis
        )

    def _create_wheel_primitives(self, x: float, y: float, yaw: float,
                                steering_angle: float, wheel_color: tuple, front_wheel_color: tuple):
        """Create wheel primitives"""
        primitives = []

        # Wheel positions relative to rear wheel center
        wheel_positions = {
            "front_left": np.array([self.wheelbase, self.wheel_track / 2]),
            "front_right": np.array([self.wheelbase, -self.wheel_track / 2]),
            "rear_left": np.array([0.0, self.wheel_track / 2]),
            "rear_right": np.array([0.0, -self.wheel_track / 2]),
        }

        # Create rotation matrices
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        vehicle_rotation = np.array([[cos_yaw, -sin_yaw], [sin_yaw, cos_yaw]])

        cos_steer = math.cos(steering_angle)
        sin_steer = math.sin(steering_angle)
        steering_rotation = np.array([[cos_steer, -sin_steer], [sin_steer, cos_steer]])

        for wheel_name, wheel_pos in wheel_positions.items():
            # Determine wheel color and rotation
            if "front" in wheel_name:
                color = front_wheel_color
                # Apply steering rotation to the wheel in its local frame, then vehicle rotation
                final_rotation = steering_rotation @ vehicle_rotation
            else:
                color = wheel_color
                # Only apply vehicle rotation for rear wheels
                final_rotation = vehicle_rotation

            # Transform wheel position to world coordinates (always use vehicle rotation for position)
            world_wheel_pos = wheel_pos @ vehicle_rotation.T
            wheel_x = x + world_wheel_pos[0]
            wheel_y = y + world_wheel_pos[1]

            # Convert rotation matrix to quaternion
            qw, qz = self._rotation_matrix_to_quaternion(final_rotation)

            # Create wheel primitive (thin box)
            wheel_primitive = PrimitiveUtils.create_cube(
                position=(wheel_x, wheel_y, 0.0),
                size=(self.wheel_length, self.wheel_width, 0.05),  # Thin wheel
                color=color,
                orientation=(0.0, 0.0, qz, qw)
            )
            primitives.append(wheel_primitive)

        return primitives

    def _create_direction_arrow_primitive(self, x: float, y: float, yaw: float):
        """Create direction arrow primitive"""
        # Position arrow at vehicle center
        arrow_x = x + (self.wheelbase / 2) * math.cos(yaw)
        arrow_y = y + (self.wheelbase / 2) * math.sin(yaw)

        return PrimitiveUtils.create_arrow(
            position=(arrow_x, arrow_y, 0.0),
            direction=yaw,
            length=self.vehicle_length * 0.6,
            thickness=0.05,
            color=(0.0, 1.0, 0.0, 1.0)  # Green arrow
        )

    def _rotation_matrix_to_quaternion(self, rotation_matrix: np.ndarray) -> tuple:
        """Convert 2D rotation matrix to quaternion"""
        # For 2D rotation, we only need Z-axis rotation
        cos_theta = rotation_matrix[0, 0]
        sin_theta = rotation_matrix[1, 0]

        qw = math.sqrt((1 + cos_theta) / 2)
        qz = sin_theta / (2 * qw) if qw > 0 else 0

        return qw, qz


class VehicleTrajectoryDemo:
    """
    Demonstration of vehicle following a trajectory with Foxglove visualization
    """

    def __init__(self):
        """Initialize the demo"""
        self.channel_manager = ChannelManager(port=8765)
        self.scene_channel = self.channel_manager.create_scene_channel("vehicle_demo")
        self.vehicle_display = FoxgloveVehicleDisplay()
        self.running = True

    def create_trajectory_entity(self) -> SceneEntity:
        """Create trajectory visualization entity"""
        # Create a figure-8 trajectory
        t = np.linspace(0, 4 * math.pi, 100)
        scale = 8.0

        # Figure-8 parametric equations
        x_traj = scale * np.sin(t)
        y_traj = scale * np.sin(t) * np.cos(t)

        # Create trajectory line
        points = [(float(x_traj[i]), float(y_traj[i]), 0.0) for i in range(len(x_traj))]
        trajectory_primitive = PrimitiveUtils.create_line(
            points=points,
            thickness=0.1,
            color=(0.5, 0.5, 0.5, 0.7)  # Gray trajectory
        )

        # Create scene entity for trajectory
        scene_entity = SceneEntity(
            id="trajectory",
            timestamp=Timestamp.now(),
            frame_id="map",
            lifetime=None,
            frame_locked=False,
            lines=[trajectory_primitive],
        )

        return scene_entity

    def create_vehicle_at_position(self, t: float) -> SceneEntity:
        """Create vehicle entity at a specific position along trajectory"""
        scale = 8.0

        # Figure-8 parametric equations
        x = scale * math.sin(t)
        y = scale * math.sin(t) * math.cos(t)

        # Calculate heading (tangent to the curve)
        dx_dt = scale * math.cos(t)
        dy_dt = scale * (math.cos(t) * math.cos(t) - math.sin(t) * math.sin(t))
        yaw = math.atan2(dy_dt, dx_dt)

        # Calculate steering angle for circular motion (approximate)
        # For figure-8, steering angle varies with position
        curvature = abs(dx_dt * dy_dt - dy_dt * dx_dt) / (dx_dt**2 + dy_dt**2)**(3/2)
        steering_angle = math.atan(curvature * self.vehicle_display.wheelbase)

        # Adjust steering sign based on turning direction
        if t > math.pi:
            steering_angle = -steering_angle

        # Create vehicle entity with dynamic coloring based on position
        hue = (t / (4 * math.pi))  # 0 to 1
        body_color = (hue, 0.5, 0.8, 0.8)  # Varying colors

        return self.vehicle_display.create_vehicle_entity(
            x, y, yaw, steering_angle,
            body_color=body_color,
            wheel_color=(0.0, 0.0, 0.0, 1.0),
            front_wheel_color=(1.0, 0.0, 0.0, 1.0),
            entity_id="trajectory_vehicle"
        )

    async def run_demo(self):
        """Run the vehicle trajectory demo"""
        print("Starting Vehicle Trajectory Demo...")
        print("Connect Foxglove Studio to ws://localhost:8765")
        print("Press Ctrl+C to stop")

        try:
            # Start the server
            self.channel_manager.start_server()

            # Create trajectory entity (static)
            trajectory_entity = self.create_trajectory_entity()

            # Animation parameters
            t = 0.0
            dt = 0.1  # Time step
            speed = 0.5  # Animation speed

            while self.running:
                # Create vehicle at current position
                vehicle_entity = self.create_vehicle_at_position(t)

                # Create scene update with both trajectory and vehicle entities
                scene_update = SceneUpdate(deletions=[], entities=[trajectory_entity, vehicle_entity])

                # Publish the scene update
                self.scene_channel.publish(scene_update)

                # Update time
                t = (t + dt * speed) % (4 * math.pi)  # Loop the animation

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


class SimpleVehicleDemo:
    """
    Simple demonstration showing a single vehicle with steering
    """

    def __init__(self):
        """Initialize the demo"""
        self.channel_manager = ChannelManager(port=8765)
        self.scene_channel = self.channel_manager.create_scene_channel("simple_vehicle")
        self.vehicle_display = FoxgloveVehicleDisplay()
        self.running = True

    async def run_demo(self):
        """Run the simple vehicle demo"""
        print("Starting Simple Vehicle Demo...")
        print("Connect Foxglove Studio to ws://localhost:8765")
        print("Press Ctrl+C to stop")

        try:
            # Start the server
            self.channel_manager.start_server()

            # Demo parameters
            x, y = 0.0, 0.0
            yaw = 0.0
            steering_angle = 0.0

            while self.running:
                # Create vehicle entity
                vehicle_entity = self.vehicle_display.create_vehicle_entity(
                    x, y, yaw, steering_angle,
                    body_color=(0.2, 0.6, 0.8, 0.8),
                    wheel_color=(0.0, 0.0, 0.0, 1.0),
                    front_wheel_color=(1.0, 0.2, 0.2, 1.0),
                    entity_id="simple_vehicle"
                )

                # Create scene update
                scene_update = SceneUpdate(deletions=[], entities=[vehicle_entity])

                # Publish the scene update
                self.scene_channel.publish(scene_update)

                # Animate steering
                steering_angle = 0.5 * math.sin(time.time())

                # Wait before next update
                await asyncio.sleep(0.1)

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
    print("Foxglove Vehicle Display Demo")
    print("1. Simple Vehicle Demo")
    print("2. Trajectory Following Demo")
    print("Choose demo (1 or 2): ", end="")

    # Default to trajectory demo if no input
    choice = "1"

    if choice == "1":
        demo = SimpleVehicleDemo()
        await demo.run_demo()
    else:
        demo = VehicleTrajectoryDemo()
        await demo.run_demo()


if __name__ == "__main__":
    asyncio.run(main())
