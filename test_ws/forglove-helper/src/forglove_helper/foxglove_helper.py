"""
Foxglove Helper Class for 3D Visualization

A reusable helper class for creating 3D visualizations with Foxglove SDK.
This class provides an easy-to-use interface for creating WebSocket servers,
managing visualization channels, and creating 3D scenes.

Features:
- Easy WebSocket server setup
- Multiple visualization channels
- 3D primitive creation helpers
- MCAP recording support
- Scene update management
- New: Channel-based architecture support
- Backward compatibility with original API

Author: Generated from foxglove_basic_usage.py
Date: 2025-01-27
Updated: 2025-01-27 (Added channel-based architecture)
"""

import asyncio
import time
import math
from datetime import datetime
from pathlib import Path
from typing import Optional, Dict, Any, List, cast, Union

# Import Foxglove SDK components
try:
    import foxglove
    from foxglove import start_server, Channel, open_mcap
    from foxglove.mcap import MCAPWriter
    from foxglove.channels import SceneUpdateChannel
    from foxglove.schemas import (
        SceneUpdate, SceneEntity,
        CubePrimitive, SpherePrimitive, LinePrimitive, ArrowPrimitive,
        Color, Point3, Vector3, Pose, Quaternion, Timestamp
    )
    FOXGLOVE_AVAILABLE = True
except ImportError:
    print("Warning: Foxglove SDK not available. Please install with: pip install foxglove-sdk")
    FOXGLOVE_AVAILABLE = False

# Import new channel-based architecture
try:
    from .channel_manager import ChannelManager, ChannelType
    from .channels import BaseChannel
    from .channel_utils import PrimitiveUtils, TransformUtils, GridUtils, PointCloudUtils
    CHANNEL_SYSTEM_AVAILABLE = True
except ImportError:
    print("Warning: New channel system not available")
    CHANNEL_SYSTEM_AVAILABLE = False


class FoxgloveHelper:
    """
    Helper class for Foxglove 3D visualization

    This class simplifies the creation of Foxglove WebSocket servers and
    provides convenient methods for creating 3D scenes with various primitives.
    """

    @staticmethod
    def _generate_timestamped_filename(base_name: str = "foxglove", extension: str = "mcap") -> str:
        """
        Generate a timestamped filename for MCAP recordings.

        Args:
            base_name: Base name for the file (default: "foxglove")
            extension: File extension (default: "mcap")

        Returns:
            Timestamped filename in format: {base_name}_YYYY-MM-DD_HH-MM-SS.{extension}
        """
        timestamp = datetime.now().strftime("%m-%d_%H-%M")
        return f"logs/{base_name}_{timestamp}.{extension}"

    def __init__(self, port: int = 8765, mcap_output_path: Optional[str] = None):
        """
        Initialize the Foxglove helper

        Args:
            port: WebSocket server port
            mcap_output_path: Optional path to save MCAP file
        """
        if not FOXGLOVE_AVAILABLE:
            raise ImportError("Foxglove SDK is not available. Please install it first.")

        self.port = port
        self.mcap_output_path = mcap_output_path or self._generate_timestamped_filename()

        # Create log directory
        Path("logs").mkdir(exist_ok=True)

        self.server = None
        self.scene_channel = None
        self.data_channel = None
        self.channels: Dict[str, Channel] = {}

        # MCAP recording sink (if enabled)
        self.mcap_sink: Optional[MCAPWriter] = None
        self.mcap_enabled: bool = False

    def start_server(self, scene_topic: str = "/visualization/scene",
                    data_topic: str = "/visualization/data",
                    enable_mcap: bool = False) -> None:
        """
        Start the Foxglove WebSocket server with visualization channels

        Args:
            scene_topic: Topic name for 3D scene updates
            data_topic: Topic name for data messages
        """
        print(f"Starting Foxglove server on ws://localhost:{self.port}")

        # Start the WebSocket server
        self.server = start_server(port=self.port)

        # Optionally enable MCAP recording sink
        if enable_mcap or self.mcap_output_path:
            try:
                self.mcap_sink = open_mcap(self.mcap_output_path)
                self.mcap_enabled = True
                print(f"✓ MCAP recording enabled: {self.mcap_output_path}")
            except Exception as exc:  # pragma: no cover - best-effort
                print(f"Warning: failed to open MCAP sink: {exc}")

        # Create visualization channels
        self.scene_channel = SceneUpdateChannel(topic=scene_topic)
        self.data_channel = Channel(topic=data_topic)

        # Store channels for easy access
        self.channels['scene'] = self.scene_channel
        self.channels['data'] = self.data_channel

        print(f"✓ Foxglove server started on ws://localhost:{self.port}")
        print(f"→ Connect Foxglove Studio to view the visualization")
        print(f"→ Add a 3D panel and subscribe to '{scene_topic}'")
        print(f"→ Add a Raw Messages panel for '{data_topic}'")

    def stop_server(self) -> None:
        """Stop the server and cleanup resources"""
        print("Stopping Foxglove helper...")
        if self.server:
            self.server.stop()
        self.channels.clear()
        # Close MCAP sink if active
        if self.mcap_sink:
            try:
                self.mcap_sink.close()
            except Exception:
                pass
            self.mcap_sink = None
            self.mcap_enabled = False

    def close(self) -> None:
        """Alias for stop_server to match common resource APIs."""
        self.stop_server()

    def __enter__(self):
        """Context manager enter - start server with default topics."""
        self.start_server()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        """Context manager exit - ensure resources are cleaned up."""
        self.stop_server()

    def create_scene_update(self, entities: list[SceneEntity],
                          deletions: Optional[List[Any]] = None) -> SceneUpdate:
        """
        Create a SceneUpdate with the given entities

        Args:
            entities: List of SceneEntity objects
            deletions: Optional list of entity IDs to delete

        Returns:
            SceneUpdate object ready to be sent
        """
        # SceneUpdate deletions expect SceneEntityDeletion objects; when callers
        # pass a list of string IDs, convert them to dicts with an 'id' field.
        deletions_fixed: List[Dict[str, Any]] = []
        if deletions:
            for d in deletions:
                if isinstance(d, str):
                    deletions_fixed.append({"id": d})
                elif isinstance(d, dict):
                    deletions_fixed.append(d)

        # Cast to satisfy static typing expectations of the Foxglove schemas
        deletions_param = cast(List[Any], deletions_fixed)
        return SceneUpdate(
            deletions=deletions_param,
            entities=entities
        )

    def log_scene(self, scene_update: SceneUpdate) -> None:
        """Log a SceneUpdate to the scene channel"""
        if self.scene_channel:
            self.scene_channel.log(scene_update)

    def log_data(self, data: Dict[str, Any]) -> None:
        """Log data to the data channel"""
        if self.data_channel:
            self.data_channel.log(data)

    def create_channel(self, name: str, topic: str, schema: Optional[dict] = None, scene_update: bool = False) -> Channel:
        """Create and register a channel by name.

        Args:
            name: Local identifier for the channel
            topic: Topic string exposed to clients
            schema: Optional JSON schema for the channel
            scene_update: If True, create a SceneUpdateChannel

        Returns:
            Created Channel object
        """
        if scene_update:
            ch = SceneUpdateChannel(topic=topic)
        else:
            ch = Channel(topic=topic, schema=schema) if schema else Channel(topic=topic)

        self.channels[name] = ch
        return ch

    def publish(self, channel_name: str, message: Any) -> None:
        """Publish a message to a named channel (convenience wrapper)."""
        ch = self.channels.get(channel_name)
        if not ch:
            raise KeyError(f"Channel not found: {channel_name}")
        ch.log(message)

    def enable_mcap_recording(self, output_path: Optional[str] = None) -> None:
        """Enable MCAP recording. Can be called before or after server start."""
        if output_path:
            self.mcap_output_path = output_path
        elif not self.mcap_output_path:
            # Generate timestamped filename if no path is set
            self.mcap_output_path = self._generate_timestamped_filename()

        try:
            self.mcap_sink = open_mcap(self.mcap_output_path)
            self.mcap_enabled = True
            print(f"✓ MCAP recording started: {self.mcap_output_path}")
        except Exception as exc:  # pragma: no cover - best-effort
            print(f"Warning: failed to start MCAP recording: {exc}")

    def disable_mcap_recording(self) -> None:
        """Disable MCAP recording and close the sink."""
        if self.mcap_sink:
            try:
                self.mcap_sink.close()
            except Exception:
                pass
            self.mcap_sink = None
            self.mcap_enabled = False

    # Helper methods for creating 3D primitives

    def create_cube(self, position: tuple[float, float, float],
                   size: tuple[float, float, float] = (1.0, 1.0, 1.0),
                   color: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 1.0),
                   orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)) -> CubePrimitive:
        """
        Create a cube primitive

        Args:
            position: (x, y, z) position
            size: (width, height, depth) size
            color: (r, g, b, a) color
            orientation: (x, y, z, w) quaternion

        Returns:
            CubePrimitive object
        """
        return CubePrimitive(
            pose=Pose(
                position=Vector3(x=position[0], y=position[1], z=position[2]),
                orientation=Quaternion(x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])
            ),
            size=Vector3(x=size[0], y=size[1], z=size[2]),
            color=Color(r=color[0], g=color[1], b=color[2], a=color[3])
        )

    def create_sphere(self, position: tuple[float, float, float],
                     radius: float = 0.5,
                     color: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 1.0)) -> SpherePrimitive:
        """
        Create a sphere primitive

        Args:
            position: (x, y, z) position
            radius: Sphere radius
            color: (r, g, b, a) color

        Returns:
            SpherePrimitive object
        """
        return SpherePrimitive(
            pose=Pose(
                position=Vector3(x=position[0], y=position[1], z=position[2]),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            ),
            size=Vector3(x=radius*2, y=radius*2, z=radius*2),
            color=Color(r=color[0], g=color[1], b=color[2], a=color[3])
        )

    def create_line(self, points: list[tuple[float, float, float]],
                   thickness: float = 0.05,
                   color: tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0)) -> LinePrimitive:
        """
        Create a line primitive

        Args:
            points: List of (x, y, z) points
            thickness: Line thickness
            color: (r, g, b, a) color

        Returns:
            LinePrimitive object
        """
        point3_list = [Point3(x=p[0], y=p[1], z=p[2]) for p in points]
        return LinePrimitive(
            pose=Pose(
                position=Vector3(x=0.0, y=0.0, z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            ),
            thickness=thickness,
            scale_invariant=False,
            points=point3_list,
            color=Color(r=color[0], g=color[1], b=color[2], a=color[3])
        )

    def create_arrow(self, position: tuple[float, float, float],
                    direction: float,
                    length: float = 1.5,
                    thickness: float = 0.1,
                    color: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 1.0)) -> ArrowPrimitive:
        """
        Create an arrow primitive

        Args:
            position: (x, y, z) position
            direction: Direction in radians (yaw angle)
            length: Arrow shaft length
            thickness: Arrow thickness
            color: (r, g, b, a) color

        Returns:
            ArrowPrimitive object
        """
        quat_z = math.sin(direction / 2.0)
        quat_w = math.cos(direction / 2.0)

        return ArrowPrimitive(
            pose=Pose(
                position=Vector3(x=position[0], y=position[1], z=position[2]),
                orientation=Quaternion(x=0.0, y=0.0, z=quat_z, w=quat_w)
            ),
            shaft_length=length,
            shaft_diameter=thickness,
            head_length=length * 0.3,
            head_diameter=thickness * 2,
            color=Color(r=color[0], g=color[1], b=color[2], a=color[3])
        )

    def create_scene_entity(self, id: str, **kwargs) -> SceneEntity:
        """
        Create a SceneEntity with the given primitives

        Args:
            id: Entity ID
            **kwargs: Primitive lists (cubes, spheres, lines, arrows)

        Returns:
            SceneEntity object
        """
        return SceneEntity(id=id, **kwargs)

    # Convenience methods for common visualization tasks

    def clear_scene(self) -> None:
        """Clear all entities from the scene"""
        if self.scene_channel:
            clear_update = SceneUpdate(deletions=[], entities=[])
            self.scene_channel.log(clear_update)

    def update_entity(self, entity_id: str, **kwargs) -> None:
        """
        Update or create an entity in the scene

        Args:
            entity_id: ID of the entity to update
            **kwargs: Primitive data to update
        """
        if self.scene_channel:
            # First delete the old entity
            delete_update = self.create_scene_update([], deletions=[entity_id])
            self.scene_channel.log(delete_update)

            # Then add the new entity
            entity = SceneEntity(id=entity_id, **kwargs)
            update = self.create_scene_update([entity], deletions=None)
            self.scene_channel.log(update)

    async def run_demo(self, duration_seconds: float = 10.0) -> None:
        """
        Run a simple demo with animated 3D content

        Args:
            duration_seconds: How long to run the demo
        """
        if not self.server:
            raise RuntimeError("Server not started. Call start_server() first.")

        print("Running Foxglove demo...")
        start_time = time.time()

        # Use protoc-generated Python classes for the demo's Person messages.
        from forglove_helper.protos import custom_person_pb2  # type: ignore
        from forglove_helper.proto_helper import create_proto_channel, log_proto_message
        proto_channel = create_proto_channel("/visualization/person", custom_person_pb2.Person)
        self.channels['person_proto'] = proto_channel
        use_generated_person = True

        while time.time() - start_time < duration_seconds:
            # Create some animated content
            t = time.time() - start_time

            # Create animated sphere
            sphere = self.create_sphere(
                position=(2 * math.cos(t), 2 * math.sin(t), 0.5),
                radius=0.3,
                color=(1.0, 0.5, 0.0, 1.0)
            )

            # Create animated arrow
            arrow = self.create_arrow(
                position=(0.0, 0.0, 0.3),
                direction=t,
                length=1.0,
                color=(0.0, 1.0, 1.0, 1.0)
            )

            # Create scene update
            entities = [
                self.create_scene_entity("animated_sphere", spheres=[sphere]),
                self.create_scene_entity("rotating_arrow", arrows=[arrow])
            ]

            scene_update = self.create_scene_update(entities)
            self.log_scene(scene_update)

            # Send some demo data
            self.log_data({
                "timestamp": time.time(),
                "demo_time": t,
                "sphere_position": [2 * math.cos(t), 2 * math.sin(t), 0.5],
                "arrow_angle": t
            })

            # Publish a protobuf Person message on the custom channel if available
       
           
            # Create an instance of the protoc-generated Person class
            person = custom_person_pb2.Person()
            person.name = "Demo User" # type: ignore
            log_proto_message(self.channels['person_proto'], person, int(time.time() * 1e9))


            await asyncio.sleep(0.1)  # Update at 10Hz

        print("Demo completed!")


class EnhancedFoxgloveHelper:
    """
    Enhanced Foxglove helper using the new channel-based architecture
    
    This class provides the same interface as FoxgloveHelper but uses the new
    channel management system internally, offering more flexibility and features.
    
    Features:
    - All features of FoxgloveHelper
    - Advanced channel management
    - Multiple channel types (TF, Grid, PointCloud, etc.)
    - Better organization and extensibility
    - Utility functions for complex data types
    """
    
    def __init__(self, port: int = 8765, mcap_output_path: Optional[str] = None):
        """
        Initialize the enhanced helper
        
        Args:
            port: WebSocket server port
            mcap_output_path: Optional path to save MCAP file
        """
        if not CHANNEL_SYSTEM_AVAILABLE:
            raise ImportError("Enhanced helper requires the new channel system")
        
        self.manager = ChannelManager(port=port, mcap_output_path=mcap_output_path)
        self.scene_channel = None
        self.data_channel = None
        
        # Backward compatibility attributes
        self.port = port
        self.mcap_output_path = mcap_output_path
        self.server = None
        self.channels = {}
    
    def start_server(self, scene_topic: str = "/visualization/scene",
                    data_topic: str = "/visualization/data",
                    enable_mcap: bool = False) -> None:
        """
        Start the server with default channels (backward compatible)
        
        Args:
            scene_topic: Topic name for 3D scene updates
            data_topic: Topic name for data messages
            enable_mcap: Whether to enable MCAP recording
        """
        # Start the underlying channel manager
        self.manager.start_server(enable_mcap=enable_mcap)
        
        # Create default channels for backward compatibility
        self.scene_channel = self.manager.create_scene_channel("scene", scene_topic)
        self.data_channel = self.manager.create_data_channel("data", data_topic)
        
        # Set up backward compatibility attributes
        self.server = self.manager.server
        self.channels = {
            'scene': self.scene_channel,
            'data': self.data_channel
        }
        
        print(f"✓ Enhanced Foxglove helper started with {len(self.manager)} channels")
    
    def stop_server(self) -> None:
        """Stop the server and cleanup resources"""
        self.manager.stop_server()
        self.server = None
        self.channels.clear()
    
    def close(self) -> None:
        """Alias for stop_server"""
        self.stop_server()
    
    def __enter__(self):
        """Context manager enter"""
        self.start_server()
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        """Context manager exit"""
        self.stop_server()
    
    # Backward compatibility methods
    
    def create_scene_update(self, entities: list[SceneEntity],
                          deletions: Optional[List[Any]] = None) -> SceneUpdate:
        """Create a SceneUpdate (backward compatible)"""
        helper = FoxgloveHelper()
        return helper.create_scene_update(entities, deletions)
    
    def log_scene(self, scene_update: SceneUpdate) -> None:
        """Log a SceneUpdate to the scene channel"""
        if self.scene_channel:
            self.scene_channel.publish(scene_update)
    
    def log_data(self, data: Dict[str, Any]) -> None:
        """Log data to the data channel"""
        if self.data_channel:
            self.data_channel.publish(data)
    
    # Enhanced methods using new utilities
    
    def create_cube(self, position: tuple[float, float, float],
                   size: tuple[float, float, float] = (1.0, 1.0, 1.0),
                   color: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 1.0),
                   orientation: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)) -> CubePrimitive:
        """Create a cube primitive using utilities"""
        return PrimitiveUtils.create_cube(position, size, color, orientation)
    
    def create_sphere(self, position: tuple[float, float, float],
                     radius: float = 0.5,
                     color: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 1.0)) -> SpherePrimitive:
        """Create a sphere primitive using utilities"""
        return PrimitiveUtils.create_sphere(position, radius, color)
    
    def create_line(self, points: list[tuple[float, float, float]],
                   thickness: float = 0.05,
                   color: tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0)) -> LinePrimitive:
        """Create a line primitive using utilities"""
        return PrimitiveUtils.create_line(points, thickness, color)
    
    def create_arrow(self, position: tuple[float, float, float],
                    direction: float,
                    length: float = 1.5,
                    thickness: float = 0.1,
                    color: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 1.0)) -> ArrowPrimitive:
        """Create an arrow primitive using utilities"""
        return PrimitiveUtils.create_arrow(position, direction, length, thickness, color)
    
    def create_scene_entity(self, id: str, **kwargs) -> SceneEntity:
        """Create a SceneEntity with the given primitives"""
        return SceneEntity(id=id, **kwargs)
    
    # Enhanced channel management
    
    def create_channel(self, channel_type: Union[ChannelType, str], name: str, topic: str, 
                      **kwargs) -> BaseChannel:
        """
        Create a new channel of the specified type
        
        Args:
            channel_type: Type of channel (ChannelType enum or string)
            name: Channel name
            topic: Topic name
            **kwargs: Additional channel-specific parameters
            
        Returns:
            Created channel instance
        """
        if isinstance(channel_type, str):
            # Convert string to enum
            type_map = {
                'scene': ChannelType.SCENE_UPDATE,
                'data': ChannelType.DATA,
                'proto': ChannelType.PROTO,
                'tf': ChannelType.TRANSFORM,
                'grid': ChannelType.GRID,
                'pointcloud': ChannelType.POINT_CLOUD,
                'laser': ChannelType.LASER_SCAN,
                'log': ChannelType.LOG,
                'custom': ChannelType.CUSTOM
            }
            channel_type = type_map.get(channel_type.lower(), ChannelType.CUSTOM)
        
        channel = self.manager.create_channel(channel_type, name, topic, **kwargs)
        self.channels[name] = channel
        return channel
    
    def get_channel(self, name: str) -> Optional[BaseChannel]:
        """Get a channel by name"""
        return self.manager.get_channel(name)
    
    def publish(self, channel_name: str, data: Any, timestamp: Optional[float] = None) -> None:
        """Publish data to a named channel"""
        self.manager.publish(channel_name, data, timestamp)
    
    # Advanced features
    
    def create_tf_channel(self, name: str = "tf", topic: str = "/tf") -> BaseChannel:
        """Create a transform channel"""
        return self.create_channel(ChannelType.TRANSFORM, name, topic)
    
    def create_grid_channel(self, name: str, topic: str) -> BaseChannel:
        """Create a grid/map channel"""
        return self.create_channel(ChannelType.GRID, name, topic)
    
    def create_pointcloud_channel(self, name: str, topic: str) -> BaseChannel:
        """Create a point cloud channel"""
        return self.create_channel(ChannelType.POINT_CLOUD, name, topic)
    
    def create_laser_channel(self, name: str, topic: str) -> BaseChannel:
        """Create a laser scan channel"""
        return self.create_channel(ChannelType.LASER_SCAN, name, topic)
    
    def publish_transform(self, parent_frame: str, child_frame: str,
                         translation: tuple[float, float, float],
                         rotation: tuple[float, float, float, float],
                         timestamp: Optional[float] = None,
                         channel_name: str = "tf") -> None:
        """
        Publish a coordinate transform
        
        Args:
            parent_frame: Parent frame ID
            child_frame: Child frame ID
            translation: (x, y, z) translation
            rotation: (x, y, z, w) quaternion rotation
            timestamp: Optional timestamp
            channel_name: Name of TF channel to use
        """
        tf_channel = self.get_channel(channel_name)
        if not tf_channel:
            tf_channel = self.create_tf_channel(channel_name)
        
        # Use the TF channel's built-in method if available
        if hasattr(tf_channel, 'publish_transform') and callable(getattr(tf_channel, 'publish_transform')):
            tf_channel.publish_transform(parent_frame, child_frame, translation, rotation, timestamp)
    
    def publish_grid(self, grid_data, channel_name: str = "grid") -> None:
        """
        Publish grid/map data
        
        Args:
            grid_data: Grid object or numpy array
            channel_name: Name of grid channel to use
        """
        grid_channel = self.get_channel(channel_name)
        if not grid_channel:
            raise ValueError(f"Grid channel '{channel_name}' not found. Create it first.")
        
        grid_channel.publish(grid_data)
    
    def clear_scene(self) -> None:
        """Clear all entities from the scene"""
        if self.scene_channel:
            self.scene_channel.clear_scene()
    
    def update_entity(self, entity_id: str, **kwargs) -> None:
        """Update or create an entity in the scene"""
        if self.scene_channel:
            # Delete old entity first
            delete_update = self.create_scene_update([], deletions=[entity_id])
            self.scene_channel.publish(delete_update)
            
            # Create new entity
            entity = SceneEntity(id=entity_id, **kwargs)
            update = self.create_scene_update([entity])
            self.scene_channel.publish(update)
    
    # Status and monitoring
    
    def get_status(self) -> Dict[str, Any]:
        """Get helper status information"""
        return self.manager.get_status()
    
    def print_status(self) -> None:
        """Print a formatted status report"""
        self.manager.print_status()
    
    async def run_demo(self, duration_seconds: float = 10.0) -> None:
        """
        Run a simple demo with animated 3D content (backward compatible)
        """
        if not self.manager.is_running:
            raise RuntimeError("Server not started. Call start_server() first.")
        
        print("Running enhanced Foxglove demo...")
        start_time = time.time()
        
        while time.time() - start_time < duration_seconds:
            t = time.time() - start_time
            
            # Create animated content using new utilities
            sphere = self.create_sphere(
                position=(2 * math.cos(t), 2 * math.sin(t), 0.5),
                radius=0.3,
                color=(1.0, 0.5, 0.0, 1.0)
            )
            
            arrow = self.create_arrow(
                position=(0.0, 0.0, 0.3),
                direction=t,
                length=1.0,
                color=(0.0, 1.0, 1.0, 1.0)
            )
            
            # Create scene entities
            entities = [
                self.create_scene_entity("animated_sphere", spheres=[sphere]),
                self.create_scene_entity("rotating_arrow", arrows=[arrow])
            ]
            
            scene_update = self.create_scene_update(entities)
            self.log_scene(scene_update)
            
            # Send demo data
            self.log_data({
                "timestamp": time.time(),
                "demo_time": t,
                "sphere_position": [2 * math.cos(t), 2 * math.sin(t), 0.5],
                "arrow_angle": t,
                "using_enhanced_helper": True
            })
            
            await asyncio.sleep(0.1)  # Update at 10Hz
        
        print("Enhanced demo completed!")


if __name__ == "__main__":
    # Example: Run with original helper
    if "--original" in sys.argv:
        helper = FoxgloveHelper()
        helper.start_server()
        asyncio.run(helper.run_demo())
        helper.stop_server()
    
    # Example: Run with enhanced helper
    elif CHANNEL_SYSTEM_AVAILABLE:
        import sys
        enhanced_helper = EnhancedFoxgloveHelper()
        enhanced_helper.start_server()
        asyncio.run(enhanced_helper.run_demo())
        enhanced_helper.stop_server()
    
    # Fallback to original helper
    else:
        print("Enhanced helper not available, using original...")
        helper = FoxgloveHelper()
        helper.start_server()
        asyncio.run(helper.run_demo())
        helper.stop_server()