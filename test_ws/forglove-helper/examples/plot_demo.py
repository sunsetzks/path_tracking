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
try:
    from foxglove.schemas import SceneEntity, SceneUpdate, Timestamp
    FOXGLOVE_AVAILABLE = True
except ImportError:
    print("Error: Foxglove SDK not available. Please install with: pip install foxglove-sdk")
    FOXGLOVE_AVAILABLE = False
    sys.exit(1)


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

    def create_sine_wave_demo(self) -> List:
        """Create a simple sine wave plot"""
        x = np.linspace(-10, 10, 100)
        y = np.sin(x)
        primitives = PlotUtils.plot(x, y, color=(0.0, 1.0, 0.0, 1.0), linewidth=0.05)
        return primitives

    def create_scatter_demo(self) -> List:
        """Create a scatter plot demo with different markers"""
        # Create random data
        np.random.seed(42)  # For reproducible results
        x = np.random.normal(0, 3, 50)
        y = np.random.normal(0, 3, 50)

        # Create scatter plot with circles
        circle_primitives = PlotUtils.scatter(
            x[:25], y[:25],
            marker='circle',
            c=(1.0, 0.0, 0.0, 1.0),
            s=30
        )

        # Create scatter plot with squares
        square_primitives = PlotUtils.scatter(
            x[25:], y[25:],
            marker='square',
            c=(0.0, 0.0, 1.0, 1.0),
            s=40
        )

        return circle_primitives + square_primitives

    def create_3d_spiral_demo(self) -> List:
        """Create a 3D spiral plot"""
        t = np.linspace(0, 4*np.pi, 200)
        x = np.cos(t) * t * 0.5
        y = np.sin(t) * t * 0.5
        z = t * 0.3

        primitives = PlotUtils.plot(x, y, z, color=(1.0, 0.5, 0.0, 1.0), linewidth=0.08)
        return primitives

    def create_3d_scatter_demo(self) -> List:
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

        primitives = PlotUtils.scatter(
            x, y, z,
            c=colors,
            marker='circle',
            s=25
        )
        return primitives

    def create_function_demo(self) -> List:
        """Create multiple function plots"""
        x = np.linspace(-5, 5, 100)

        primitives = []

        # Sine wave
        y1 = np.sin(x)
        sine_plot = PlotUtils.plot(x, y1 + 3, color=(1.0, 0.0, 0.0, 1.0), linewidth=0.03)
        primitives.extend(sine_plot)

        # Cosine wave
        y2 = np.cos(x)
        cosine_plot = PlotUtils.plot(x, y2 + 1, color=(0.0, 1.0, 0.0, 1.0), linewidth=0.03)
        primitives.extend(cosine_plot)

        # Exponential
        y3 = np.exp(-x**2 / 2) * 2 - 2
        exp_plot = PlotUtils.plot(x, y3 - 1, color=(0.0, 0.0, 1.0, 1.0), linewidth=0.03)
        primitives.extend(exp_plot)

        return primitives

    def create_coordinate_axes(self) -> List:
        """Create coordinate axes for reference"""
        return PlotUtils.create_axes(
            xlim=(-12, 12),
            ylim=(-12, 12),
            zlim=(-5, 8),
            color=(0.3, 0.3, 0.3, 0.8),
            linewidth=0.02
        )

    def create_demo_entities(self) -> List[SceneEntity]:
        """Create all demo scene entities"""
        entities = []

        # Create coordinate axes
        axes_primitives = self.create_coordinate_axes()
        if axes_primitives:
                    entities.append(SceneEntity(
            id="axes",
            timestamp=Timestamp(sec=int(time.time()), nsec=0),
            frame_id="plot_demo",
            lifetime=None,
            frame_locked=False,
            lines=axes_primitives
        ))

        # Create sine wave
        sine_primitives = self.create_sine_wave_demo()
        if sine_primitives:
            entities.append(SceneEntity(
                id="sine_wave",
                timestamp=Timestamp(sec=int(time.time()), nsec=0),
                frame_id="plot_demo",
                lifetime=None,
                frame_locked=False,
                lines=sine_primitives,
                cubes=[],
                spheres=[]
            ))

        # Create scatter plot
        scatter_primitives = self.create_scatter_demo()
        spheres = [p for p in scatter_primitives if hasattr(p, 'size') and hasattr(p, 'pose')]
        cubes = [p for p in scatter_primitives if hasattr(p, 'size') and hasattr(p, 'pose') and 'CubePrimitive' in str(type(p))]

        if spheres or cubes:
            entities.append(SceneEntity(
                id="scatter_plot",
                timestamp=Timestamp(sec=int(time.time()), nsec=0),
                frame_id="plot_demo",
                lifetime=None,
                frame_locked=False,
                lines=[],
                cubes=cubes,
                spheres=spheres
            ))

        # Create 3D spiral
        spiral_primitives = self.create_3d_spiral_demo()
        if spiral_primitives:
            entities.append(SceneEntity(
                id="3d_spiral",
                timestamp=Timestamp(sec=int(time.time()), nsec=0),
                frame_id="plot_demo",
                lifetime=None,
                frame_locked=False,
                lines=spiral_primitives,
                cubes=[],
                spheres=[]
            ))

        # Create 3D scatter
        scatter_3d_primitives = self.create_3d_scatter_demo()
        spheres_3d = [p for p in scatter_3d_primitives if hasattr(p, 'size') and hasattr(p, 'pose')]

        if spheres_3d:
            entities.append(SceneEntity(
                id="3d_scatter",
                timestamp=Timestamp(sec=int(time.time()), nsec=0),
                frame_id="plot_demo",
                lifetime=None,
                frame_locked=False,
                lines=[],
                cubes=[],
                spheres=spheres_3d
            ))

        # Create function comparison
        function_primitives = self.create_function_demo()
        if function_primitives:
            entities.append(SceneEntity(
                id="functions",
                timestamp=Timestamp(sec=int(time.time()), nsec=0),
                frame_id="plot_demo",
                lifetime=None,
                frame_locked=False,
                lines=function_primitives if function_primitives and hasattr(function_primitives[0], 'points') else [],
                cubes=[],
                spheres=[]
            ))

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
