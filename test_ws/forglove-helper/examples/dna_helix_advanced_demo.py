#!/usr/bin/env python3
"""
Advanced DNA Double Helix Visualization Demo

This example demonstrates an advanced 3D visualization of a DNA double helix
structure with biological details and animated features.

Features demonstrated:
- Realistic DNA double helix geometry with proper proportions
- Animated rotation and dynamic visualization
- Different nucleotide types (A-T, G-C base pairs)
- Color-coded nucleotides (Adenine, Thymine, Guanine, Cytosine)
- Phosphate backbone representation
- Integration with channel manager and real-time updates

The DNA structure includes:
- Two intertwined helical strands with realistic geometry
- Color-coded base pairs (A-T blue, G-C red)
- Phosphate backbone in yellow
- Nucleotide spheres with biological colors
- Smooth animation and rotation

Run this example and connect Foxglove Studio to ws://localhost:8765 to see the visualization.

Author: Generated for path_tracking project
Date: 2025-01-27
"""

import asyncio
import math
import time
import numpy as np
from typing import List, Tuple
import random

# Import the channel system
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'src'))

from forglove_helper.channel_manager import ChannelManager, ChannelType
from forglove_helper.channel_utils import PlotUtils
from forglove_helper.channels import SceneUpdateChannel

# Import Foxglove schemas
from foxglove.schemas import SceneEntity, SceneUpdate, Timestamp


class AdvancedDNAHelixDemo:
    """
    Advanced DNA Double Helix visualization with biological details and animation
    """

    def __init__(self):
        """Initialize the advanced DNA helix demo"""
        self.channel_manager = ChannelManager(port=8765)
        self.scene_channel = self.channel_manager.create_scene_channel("advanced_dna_helix_scene")

        # Realistic DNA parameters (in nanometers, scaled for visualization)
        self.major_groove_radius = 1.0      # Major groove radius
        self.minor_groove_radius = 0.6      # Minor groove radius
        self.height = 15.0                  # Total height of DNA segment
        self.base_pairs_per_turn = 10.4     # Base pairs per helical turn
        self.base_pair_distance = 0.34      # Distance between base pairs (nm)
        self.points_per_segment = 100       # Points for smooth curves

        # Animation parameters
        self.rotation_angle = 0.0
        self.rotation_speed = 0.5  # radians per second
        self.time_elapsed = 0.0

        # Demo parameters
        self.running = True

        # Frame configuration
        self.frame_id = "dna_demo"  # Custom frame for DNA visualization

    def get_nucleotide_color(self, nucleotide: str) -> Tuple[float, float, float, float]:
        """Get biological color for different nucleotides"""
        colors = {
            'A': (1.0, 0.6, 0.2, 1.0),  # Adenine - orange
            'T': (0.2, 0.6, 1.0, 1.0),  # Thymine - light blue
            'G': (0.8, 0.2, 0.8, 1.0),  # Guanine - magenta
            'C': (0.2, 0.8, 0.2, 1.0),  # Cytosine - green
        }
        return colors.get(nucleotide, (0.7, 0.7, 0.7, 1.0))

    def get_base_pair_color(self, pair: str) -> Tuple[float, float, float, float]:
        """Get color for base pair type"""
        if pair in ['AT', 'TA']:
            return (0.4, 0.7, 1.0, 0.8)  # A-T pairs - blue
        elif pair in ['GC', 'CG']:
            return (1.0, 0.4, 0.4, 0.8)  # G-C pairs - red
        else:
            return (0.6, 0.6, 0.6, 0.8)  # Unknown - gray

    def generate_dna_sequence(self, length: int) -> List[str]:
        """Generate a realistic DNA sequence"""
        nucleotides = ['A', 'T', 'G', 'C']
        sequence = []

        for i in range(length):
            if sequence and sequence[-1] == 'A':
                # A is usually paired with T
                sequence.append('T')
            elif sequence and sequence[-1] == 'T':
                # T is usually paired with A
                sequence.append('A')
            elif sequence and sequence[-1] == 'G':
                # G is usually paired with C
                sequence.append('C')
            elif sequence and sequence[-1] == 'C':
                # C is usually paired with G
                sequence.append('G')
            else:
                sequence.append(random.choice(nucleotides))

        return sequence

    def create_phosphate_backbone(self) -> List[SceneEntity]:
        """Create the phosphate backbone of DNA"""
        entities = []

        # Calculate helix parameters
        num_base_pairs = int(self.height / self.base_pair_distance)
        t = np.linspace(0, 2 * np.pi * (self.height / (10.4 * 0.34)), self.points_per_segment)

        # Pitch and radius
        pitch = self.base_pair_distance * self.base_pairs_per_turn
        angular_velocity = 2 * np.pi / pitch

        # Strand 1 backbone (major groove)
        x1 = self.major_groove_radius * np.cos(t)
        y1 = self.major_groove_radius * np.sin(t)
        z1 = (pitch / (2 * np.pi)) * t

        backbone1 = PlotUtils.plot(
            x1, y1, z1,
            color=(1.0, 0.9, 0.2, 0.9),  # Yellow phosphate backbone
            linewidth=0.06,
            frame_id=self.frame_id,
            entity_id='phosphate_backbone_1'
        )
        entities.append(backbone1)

        # Strand 2 backbone (minor groove with phase shift)
        x2 = self.minor_groove_radius * np.cos(t + np.pi)
        y2 = self.minor_groove_radius * np.sin(t + np.pi)
        z2 = (pitch / (2 * np.pi)) * t

        backbone2 = PlotUtils.plot(
            x2, y2, z2,
            color=(1.0, 0.9, 0.2, 0.9),  # Yellow phosphate backbone
            linewidth=0.06,
            frame_id=self.frame_id,
            entity_id='phosphate_backbone_2'
        )
        entities.append(backbone2)

        return entities

    def create_dna_strands(self) -> List[SceneEntity]:
        """Create the DNA nucleotide strands"""
        entities = []

        # Generate DNA sequence
        num_base_pairs = int(self.height / self.base_pair_distance)
        sequence = self.generate_dna_sequence(num_base_pairs)

        z_positions = np.linspace(0, self.height, num_base_pairs)

        # Create nucleotide entities
        strand1_x, strand1_y, strand1_z = [], [], []
        strand2_x, strand2_y, strand2_z = [], [], []

        for i, (nucleotide, z) in enumerate(zip(sequence, z_positions)):
            # Calculate angle at this z position
            angle = (2 * np.pi * z) / (self.base_pair_distance * self.base_pairs_per_turn)

            # Position on strand 1 (major groove)
            x1 = self.major_groove_radius * math.cos(angle)
            y1 = self.major_groove_radius * math.sin(angle)
            strand1_x.append(x1)
            strand1_y.append(y1)
            strand1_z.append(z)

            # Position on strand 2 (minor groove)
            x2 = self.minor_groove_radius * math.cos(angle + np.pi)
            y2 = self.minor_groove_radius * math.sin(angle + np.pi)
            strand2_x.append(x2)
            strand2_y.append(y2)
            strand2_z.append(z)

        # Create strand 1
        strand1_entity = PlotUtils.plot(
            strand1_x, strand1_y, strand1_z,
            color=(0.3, 0.6, 0.9, 0.7),
            linewidth=0.04,
            frame_id=self.frame_id,
            entity_id='dna_strand_1'
        )
        entities.append(strand1_entity)

        # Create strand 2
        strand2_entity = PlotUtils.plot(
            strand2_x, strand2_y, strand2_z,
            color=(0.9, 0.3, 0.3, 0.7),
            linewidth=0.04,
            frame_id=self.frame_id,
            entity_id='dna_strand_2'
        )
        entities.append(strand2_entity)

        return entities

    def create_base_pairs(self) -> List[SceneEntity]:
        """Create base pairs connecting the two strands"""
        entities = []

        # Generate DNA sequence
        num_base_pairs = int(self.height / self.base_pair_distance)
        sequence = self.generate_dna_sequence(num_base_pairs)
        z_positions = np.linspace(0, self.height, num_base_pairs)

        for i, (nucleotide, z) in enumerate(zip(sequence, z_positions)):
            # Calculate angle at this z position
            angle = (2 * np.pi * z) / (self.base_pair_distance * self.base_pairs_per_turn)

            # Position on strand 1
            x1 = self.major_groove_radius * math.cos(angle)
            y1 = self.major_groove_radius * math.sin(angle)

            # Position on strand 2
            x2 = self.minor_groove_radius * math.cos(angle + np.pi)
            y2 = self.minor_groove_radius * math.sin(angle + np.pi)

            # Determine complementary nucleotide
            if nucleotide == 'A':
                complement = 'T'
                pair = 'AT'
            elif nucleotide == 'T':
                complement = 'A'
                pair = 'TA'
            elif nucleotide == 'G':
                complement = 'C'
                pair = 'GC'
            else:  # C
                complement = 'G'
                pair = 'CG'

            # Create base pair connection
            pair_color = self.get_base_pair_color(pair)
            base_pair_entity = PlotUtils.plot(
                [x1, x2], [y1, y2], [z, z],
                color=pair_color,
                linewidth=0.03,
                frame_id=self.frame_id,
                entity_id=f'base_pair_{i}'
            )
            entities.append(base_pair_entity)

        return entities

    def create_nucleotides(self) -> List[SceneEntity]:
        """Create individual nucleotide spheres"""
        entities = []

        # Generate DNA sequence
        num_base_pairs = int(self.height / self.base_pair_distance)
        sequence = self.generate_dna_sequence(num_base_pairs)
        z_positions = np.linspace(0, self.height, num_base_pairs)

        for i, (nucleotide, z) in enumerate(zip(sequence, z_positions)):
            # Calculate angle at this z position
            angle = (2 * np.pi * z) / (self.base_pair_distance * self.base_pairs_per_turn)

            # Position on strand 1
            x1 = self.major_groove_radius * math.cos(angle)
            y1 = self.major_groove_radius * math.sin(angle)

            # Position on strand 2
            x2 = self.minor_groove_radius * math.cos(angle + np.pi)
            y2 = self.minor_groove_radius * math.sin(angle + np.pi)

            # Determine complementary nucleotide
            if nucleotide == 'A':
                complement = 'T'
            elif nucleotide == 'T':
                complement = 'A'
            elif nucleotide == 'G':
                complement = 'C'
            else:  # C
                complement = 'G'

            # Create nucleotide spheres
            nucleotide1 = PlotUtils.scatter(
                [x1], [y1], [z],
                marker='circle',
                c=self.get_nucleotide_color(nucleotide),
                s=12,
                frame_id=self.frame_id,
                entity_id=f'nucleotide_{nucleotide}_{i}'
            )
            entities.append(nucleotide1)

            nucleotide2 = PlotUtils.scatter(
                [x2], [y2], [z],
                marker='circle',
                c=self.get_nucleotide_color(complement),
                s=12,
                frame_id=self.frame_id,
                entity_id=f'nucleotide_{complement}_{i}'
            )
            entities.append(nucleotide2)

        return entities

    def create_coordinate_axes(self) -> SceneEntity:
        """Create coordinate axes for reference"""
        primitives = PlotUtils.create_axes(
            xlim=(-3, 3),
            ylim=(-3, 3),
            zlim=(-2, 17),
            color=(0.4, 0.4, 0.4, 0.5),
            linewidth=0.02
        )

        # Create SceneEntity from primitives
        lines = [p for p in primitives if hasattr(p, 'points')]

        return SceneEntity(
            id="dna_axes",
            timestamp=Timestamp.now(),
            frame_id=self.frame_id,
            lifetime=None,
            frame_locked=False,
            lines=lines,
            cubes=[],
            spheres=[]
        )

    def create_demo_entities(self) -> List[SceneEntity]:
        """Create all demo scene entities with rotation"""
        entities = []

        # Create coordinate axes
        axes_entity = self.create_coordinate_axes()
        entities.append(axes_entity)

        # Create phosphate backbone
        backbone_entities = self.create_phosphate_backbone()
        entities.extend(backbone_entities)

        # Create DNA strands
        strand_entities = self.create_dna_strands()
        entities.extend(strand_entities)

        # Create base pairs
        base_pair_entities = self.create_base_pairs()
        entities.extend(base_pair_entities)

        # Create nucleotides
        nucleotide_entities = self.create_nucleotides()
        entities.extend(nucleotide_entities)

        return entities

    async def run_demo(self):
        """Run the advanced DNA helix demonstration"""
        print("Starting Advanced DNA Double Helix Demo...")
        print("Connect Foxglove Studio to ws://localhost:8765")
        print("Features: Realistic DNA geometry, color-coded nucleotides, animated rotation")
        print("Press Ctrl+C to stop")

        try:
            # Start the channel manager
            self.channel_manager.start_server()

            start_time = time.time()

            while self.running:
                # Update animation
                current_time = time.time()
                self.time_elapsed = current_time - start_time
                self.rotation_angle = self.rotation_speed * self.time_elapsed

                # Create demo entities
                entities = self.create_demo_entities()

                # Create scene update
                scene_update = SceneUpdate(deletions=[], entities=entities)

                # Publish the scene update
                self.scene_channel.publish(scene_update)

                # Print status every 10 updates
                if int(self.time_elapsed) % 20 == 0 and self.time_elapsed > 0:
                    print(f"DNA Demo running... Time: {self.time_elapsed:.2f}s")
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
    """Main function"""
    demo = AdvancedDNAHelixDemo()
    try:
        await demo.run_demo()
    except KeyboardInterrupt:
        demo.stop()
        demo.channel_manager.stop_server()


if __name__ == "__main__":
    asyncio.run(main())
