#!/usr/bin/env python3
"""
Simple Plot Example

This is a minimal example showing how to use the matplotlib-like plot and scatter
utilities in the forglove-helper package.

Features demonstrated:
- Basic 2D plotting with plot()
- Basic 2D scatter plotting with scatter()
- Integration with Foxglove visualization

Run this example and connect Foxglove Studio to ws://localhost:8765 to see the visualization.

Author: Generated for path_tracking project
Date: 2025-01-27
"""

import asyncio
import math
import time
import numpy as np
from typing import List

# Import the forglove-helper modules
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from forglove_helper.channel_manager import ChannelManager
from forglove_helper.channel_utils import PlotUtils

# Import Foxglove schemas
from foxglove.schemas import SceneEntity, SceneUpdate, Timestamp


class SimplePlotExample:
    """
    Simple example of using PlotUtils for basic plotting
    """

    def __init__(self):
        """Initialize the example"""
        self.channel_manager = ChannelManager(port=8765)
        self.scene_channel = self.channel_manager.create_scene_channel("simple_plot")
        self.running = True

    def create_demo_entities(self) -> List[SceneEntity]:
        """Create demo scene entities"""
        entities = []

        # Create some sample data
        x = np.linspace(-5, 5, 100)
        y1 = np.sin(x)  # Sine wave
        y2 = np.cos(x)  # Cosine wave

        # Add sine wave plot
        sine_entity = PlotUtils.plot(x, y1, color=(1.0, 0.0, 0.0, 1.0), linewidth=0.05,
                                   entity_id='sine_wave') 
        entities.append(sine_entity)

        # Add cosine wave plot
        cosine_entity = PlotUtils.plot(x, y2, color=(0.0, 1.0, 0.0, 1.0), linewidth=0.05,
                                     entity_id='cosine_wave')
        entities.append(cosine_entity)

        # Add some scatter points
        scatter_x = np.random.uniform(-4, 4, 20)
        scatter_y = np.random.uniform(-1, 1, 20)
        scatter_entity = PlotUtils.scatter(scatter_x, scatter_y, c='blue', marker='circle', s=5,
                                         entity_id='scatter_points')
        entities.append(scatter_entity)

        return entities

    async def run_example(self):
        """Run the simple plot example"""
        print("Starting Simple Plot Example...")
        print("Connect Foxglove Studio to ws://localhost:8765")
        print("Press Ctrl+C to stop")

        try:
            # Start the server
            self.channel_manager.start_server()

            while self.running:
                # Create demo entities
                entities = self.create_demo_entities()

                # Create scene update
                scene_update = SceneUpdate(deletions=[], entities=entities)

                # Publish the scene update
                self.scene_channel.publish(scene_update)

                # Wait before next update
                await asyncio.sleep(1.0)

        except KeyboardInterrupt:
            print("\nStopping example...")
        except Exception as e:
            print(f"Error in example: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.channel_manager.stop_server()

    def stop(self):
        """Stop the example"""
        self.running = False


async def main():
    """Main function"""
    example = SimplePlotExample()
    try:
        await example.run_example()
    except KeyboardInterrupt:
        example.stop()
        example.channel_manager.stop_server()


if __name__ == "__main__":
    asyncio.run(main())
