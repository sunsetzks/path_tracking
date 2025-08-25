#!/usr/bin/env python3
"""
Plot and Scatter Demo

This example demonstrates the new matplotlib-like plot and scatter utilities
for creating 2D and 3D visualizations in Foxglove Studio.

Features demonstrated:
- 2D line plots using plot()
- 2D scatter plots using scatter()
- 3D line plots and scatter plots
- Different marker styles and colors
- Axis creation for reference
- Integration with channel manager

Run this example and connect Foxglove Studio to ws://localhost:8765 to see the visualization.

Author: Generated for path_tracking project
Date: 2025-01-27
"""

import asyncio
import math
import time
import numpy as np
from typing import List

# Import the channel system
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from forglove_helper.channel_manager import ChannelManager, ChannelType
from forglove_helper.channel_utils import PlotUtils
from forglove_helper.channels import SceneUpdateChannel

# Import Foxglove schemas
from foxglove.schemas import SceneEntity, SceneUpdate, Timestamp


class PlotDemo:
    """
    Demonstration of the new matplotlib-like plot and scatter utilities
    """

    def __init__(self):
        """Initialize the plot demo"""
        self.channel_manager = ChannelManager(port=8765)
        self.scene_channel = self.channel_manager.create_scene_channel("plot_demo_scene")

        # Demo parameters
        self.running = True

    def create_sine_wave_demo(self) -> SceneEntity:
        """Create a simple sine wave plot"""
        x = np.linspace(-10, 10, 100)
        y = np.sin(x)
        entity = PlotUtils.plot(x, y, color=(0.0, 1.0, 0.0, 1.0), linewidth=0.05,
                              entity_id='sine_wave_demo')
        return entity

    def create_circle_scatter_demo(self) -> SceneEntity:
        """Create a circle scatter plot demo"""
        # Create random data
        np.random.seed(42)  # For reproducible results
        x = np.random.normal(0, 3, 25)
        y = np.random.normal(0, 3, 25)

        # Create scatter plot with circles
        entity = PlotUtils.scatter(
            x, y,
            marker='circle',
            c=(1.0, 0.0, 0.0, 1.0),
            s=30,
            entity_id='circle_scatter'
        )
        return entity

    def create_square_scatter_demo(self) -> SceneEntity:
        """Create a square scatter plot demo"""
        # Create random data
        np.random.seed(42)  # For reproducible results
        x = np.random.normal(0, 3, 25)
        y = np.random.normal(0, 3, 25)

        # Create scatter plot with squares
        entity = PlotUtils.scatter(
            x, y,
            marker='square',
            c=(0.0, 0.0, 1.0, 1.0),
            s=40,
            entity_id='square_scatter'
        )
        return entity

    def create_3d_spiral_demo(self) -> SceneEntity:
        """Create a 3D spiral plot"""
        t = np.linspace(0, 4*np.pi, 200)
        x = np.cos(t) * t * 0.5
        y = np.sin(t) * t * 0.5
        z = t * 0.3

        entity = PlotUtils.plot(x, y, z, color=(1.0, 0.5, 0.0, 1.0), linewidth=0.08,
                              entity_id='3d_spiral')
        return entity

    def create_3d_scatter_demo(self) -> SceneEntity:
        """Create a 3D scatter plot"""
        # Generate 3D random data
        np.random.seed(123)
        n_points = 100
        x = np.random.normal(0, 2, n_points)
        y = np.random.normal(0, 2, n_points)
        z = np.random.normal(0, 2, n_points)

        # Create colors based on z-coordinate
        colors = []
        for i in range(n_points):
            # Color from blue (low z) to red (high z)
            t = (z[i] - z.min()) / (z.max() - z.min())
            colors.append((t, 0.5, 1.0-t, 1.0))

        entity = PlotUtils.scatter(
            x, y, z,
            c=colors,
            marker='circle',
            s=25,
            entity_id='3d_scatter_demo'
        )
        return entity

    def create_sine_demo(self) -> SceneEntity:
        """Create sine wave plot"""
        x = np.linspace(-5, 5, 100)
        y1 = np.sin(x)
        entity = PlotUtils.plot(x, y1 + 3, color=(1.0, 0.0, 0.0, 1.0), linewidth=0.03,
                              entity_id='sine_function')
        return entity

    def create_cosine_demo(self) -> SceneEntity:
        """Create cosine wave plot"""
        x = np.linspace(-5, 5, 100)
        y2 = np.cos(x)
        entity = PlotUtils.plot(x, y2 + 1, color=(0.0, 1.0, 0.0, 1.0), linewidth=0.03,
                              entity_id='cosine_function')
        return entity

    def create_exp_demo(self) -> SceneEntity:
        """Create exponential plot"""
        x = np.linspace(-5, 5, 100)
        y3 = np.exp(-x**2 / 2) * 2 - 2
        entity = PlotUtils.plot(x, y3 - 1, color=(0.0, 0.0, 1.0, 1.0), linewidth=0.03,
                              entity_id='exp_function')
        return entity

    def create_coordinate_axes(self) -> SceneEntity:
        """Create coordinate axes for reference"""
        # Note: create_axes still returns primitives list, so we need to wrap it
        primitives = PlotUtils.create_axes(
            xlim=(-12, 12),
            ylim=(-12, 12),
            zlim=(-5, 8),
            color=(0.3, 0.3, 0.3, 0.8),
            linewidth=0.02
        )

        # Create SceneEntity from primitives
        lines = [p for p in primitives if hasattr(p, 'points')]

        return SceneEntity(
            id="axes",
            timestamp=Timestamp.now(),
            frame_id="plot_demo",
            lifetime=None,
            frame_locked=False,
            lines=lines,
            cubes=[],
            spheres=[]
        )

    def create_demo_entities(self) -> List[SceneEntity]:
        """Create all demo scene entities"""
        entities = []

        # Create coordinate axes
        axes_entity = self.create_coordinate_axes()
        entities.append(axes_entity)

        # Create sine wave
        sine_entity = self.create_sine_wave_demo()
        entities.append(sine_entity)

        # Create scatter plots
        circle_entity = self.create_circle_scatter_demo()
        entities.append(circle_entity)

        square_entity = self.create_square_scatter_demo()
        entities.append(square_entity)

        # Create 3D spiral
        spiral_entity = self.create_3d_spiral_demo()
        entities.append(spiral_entity)

        # Create 3D scatter
        scatter_3d_entity = self.create_3d_scatter_demo()
        entities.append(scatter_3d_entity)

        # Create function plots
        sine_func_entity = self.create_sine_demo()
        entities.append(sine_func_entity)

        cosine_func_entity = self.create_cosine_demo()
        entities.append(cosine_func_entity)

        exp_func_entity = self.create_exp_demo()
        entities.append(exp_func_entity)

        return entities

    async def run_demo(self):
        """Run the plot demonstration"""
        print("Starting Plot Demo...")
        print("Connect Foxglove Studio to ws://localhost:8765")
        print("Press Ctrl+C to stop")

        try:
            # Start the channel manager
            self.channel_manager.start_server()

            while self.running:
                # Create demo entities
                entities = self.create_demo_entities()

                # Create scene update
                scene_update = SceneUpdate(deletions=[], entities=entities)

                # Publish the scene update
                self.scene_channel.publish(scene_update)

                # Wait before next update
                await asyncio.sleep(2.0)

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
    """Main function"""
    demo = PlotDemo()
    try:
        await demo.run_demo()
    except KeyboardInterrupt:
        demo.stop()
        demo.channel_manager.stop_server()


if __name__ == "__main__":
    asyncio.run(main())
