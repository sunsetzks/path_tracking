"""
Channel Manager for Foxglove SDK

This module provides a centralized management system for Foxglove channels,
server lifecycle, MCAP recording, and multi-channel operations.

Features:
- Multi-channel management
- Server lifecycle management
- MCAP recording
- Channel discovery and introspection
- Batch operations
- Context manager support

Author: Generated for path_tracking project
Date: 2025-01-27
"""

import asyncio
import time
from datetime import datetime
from pathlib import Path
from typing import Optional, Dict, Any, List, Union, Iterator, cast, TYPE_CHECKING
import logging

# Import channel system
from .channels import (
    BaseChannel, ChannelType, create_channel,
    SceneUpdateChannel, DataChannel, ProtoChannel, TfChannel,
    GridChannel, PointCloudChannel, LaserScanChannel, LogChannel, CustomChannel
)

# Import Foxglove schemas for type annotations
if TYPE_CHECKING:
    try:
        from foxglove.schemas import Timestamp
    except ImportError:
        Timestamp = None

# Import Foxglove SDK components
import foxglove
from foxglove import start_server, open_mcap
from foxglove.mcap import MCAPWriter


class ChannelManager:
    """
    Centralized manager for Foxglove channels and server operations
    
    This class provides a high-level interface for managing multiple channels,
    server lifecycle, MCAP recording, and batch operations.
    """
    
    def __init__(self, port: int = 8765, mcap_output_path: Optional[str] = None,
                 auto_start_server: bool = False):
        """
        Initialize the channel manager
        
        Args:
            port: WebSocket server port
            mcap_output_path: Optional path to save MCAP file
            auto_start_server: Whether to start server automatically
        """

        
        self.port = port
        self.mcap_output_path = mcap_output_path or self._generate_timestamped_filename()
        
        # Create log directory
        Path("logs").mkdir(exist_ok=True)
        
        # Server and recording management
        self.server = None
        self.mcap_sink: Optional[MCAPWriter] = None
        self.mcap_enabled: bool = False
        self.is_running: bool = False
        
        # Channel management
        self.channels: Dict[str, BaseChannel] = {}
        
        # Statistics and monitoring
        self.created_at = datetime.now()
        self.total_messages_published = 0
        
        # Logging
        self.logger = logging.getLogger(__name__)
        
        if auto_start_server:
            self.start_server()
    
    @staticmethod
    def _generate_timestamped_filename(base_name: str = "foxglove", extension: str = "mcap") -> str:
        """Generate a timestamped filename for MCAP recordings"""
        timestamp = datetime.now().strftime("%m-%d_%H-%M")
        return f"logs/{base_name}_{timestamp}.{extension}"
    
    def start_server(self, enable_mcap: bool = False) -> None:
        """
        Start the Foxglove WebSocket server
        
        Args:
            enable_mcap: Whether to enable MCAP recording
        """
        if self.is_running:
            self.logger.warning("Server is already running")
            return
        
        self.logger.info(f"Starting Foxglove server on ws://localhost:{self.port}")
        
        # Start the WebSocket server
        self.server = start_server(port=self.port)
        self.is_running = True
        
        # Optionally enable MCAP recording
        if enable_mcap or self.mcap_output_path:
            self.enable_mcap_recording()
        
        self.logger.info(f"✓ Foxglove server started on ws://localhost:{self.port}")
        self.logger.info(f"→ Connect Foxglove Studio to view the visualization")
        
        # Log existing channels
        if self.channels:
            self.logger.info(f"→ {len(self.channels)} channels available:")
            for name, channel in self.channels.items():
                self.logger.info(f"  - {name}: {channel.topic} ({channel.channel_type.value})")
    
    def stop_server(self) -> None:
        """Stop the server and cleanup resources"""
        if not self.is_running:
            return
        
        self.logger.info("Stopping Foxglove server...")
        
        if self.server:
            self.server.stop()
            self.server = None
        
        # Close MCAP sink if active
        self.disable_mcap_recording()
        
        self.is_running = False
        self.logger.info("✓ Server stopped")
    
    def enable_mcap_recording(self, output_path: Optional[str] = None) -> None:
        """
        Enable MCAP recording
        
        Args:
            output_path: Optional custom output path
        """
        if self.mcap_enabled:
            self.logger.warning("MCAP recording is already enabled")
            return
        
        if output_path:
            self.mcap_output_path = output_path
        
        try:
            self.mcap_sink = open_mcap(self.mcap_output_path)
            self.mcap_enabled = True
            self.logger.info(f"✓ MCAP recording enabled: {self.mcap_output_path}")
        except Exception as exc:
            self.logger.error(f"Failed to enable MCAP recording: {exc}")
    
    def disable_mcap_recording(self) -> None:
        """Disable MCAP recording and close the sink"""
        if not self.mcap_enabled:
            return
        
        if self.mcap_sink:
            try:
                self.mcap_sink.close()
                self.logger.info("✓ MCAP recording disabled")
            except Exception as exc:
                self.logger.error(f"Error closing MCAP sink: {exc}")
            finally:
                self.mcap_sink = None
                self.mcap_enabled = False
    
    # Channel Management Methods
    
    def create_channel(self, channel_type: ChannelType, topic: str,
                      **kwargs) -> BaseChannel:
        """
        Create and register a new channel

        Args:
            channel_type: Type of channel to create
            topic: Topic name for publishing (also used as channel name)
            **kwargs: Additional channel-specific parameters

        Returns:
            Created channel instance
        """
        # Use topic as the channel name
        name = topic

        if name in self.channels:
            raise ValueError(f"Channel '{name}' already exists")

        # Create the channel
        channel = create_channel(channel_type, name, topic, **kwargs)

        # Register the channel
        self.channels[name] = channel

        self.logger.info(f"✓ Created channel '{name}' on topic '{topic}' ({channel_type.value})")
        return channel
    
    def get_channel(self, name: str) -> Optional[BaseChannel]:
        """Get a channel by name"""
        return self.channels.get(name)
    

    
    def remove_channel(self, name: str) -> bool:
        """
        Remove a channel
        
        Args:
            name: Channel name to remove
            
        Returns:
            True if channel was removed, False if not found
        """
        if name not in self.channels:
            return False
        
        channel = self.channels[name]
        
        # Remove from channels mapping
        del self.channels[name]
        
        self.logger.info(f"✓ Removed channel '{name}'")
        return True
    
    def list_channels(self) -> List[str]:
        """Get list of all channel names"""
        return list(self.channels.keys())
    
    def get_channels_by_type(self, channel_type: ChannelType) -> List[BaseChannel]:
        """Get all channels of a specific type"""
        return [ch for ch in self.channels.values() if ch.channel_type == channel_type]
    
    # Convenient channel creation methods
    
    def create_scene_channel(self, topic: str, **kwargs) -> SceneUpdateChannel:
        """Create a scene update channel"""
        channel = self.create_channel(ChannelType.SCENE_UPDATE, topic, **kwargs)
        return cast(SceneUpdateChannel, channel)
    
    def create_data_channel(self, topic: str, schema: Optional[Dict] = None,
                           **kwargs) -> DataChannel:
        """Create a data channel"""
        channel = self.create_channel(ChannelType.DATA, topic, schema=schema, **kwargs)
        return cast(DataChannel, channel)
    
    def create_proto_channel(self, topic: str, proto_class, **kwargs) -> ProtoChannel:
        """Create a protobuf channel"""
        channel = self.create_channel(ChannelType.PROTO, topic, proto_class=proto_class, **kwargs)
        return cast(ProtoChannel, channel)
    
    def create_tf_channel(self, topic: str = "/tf", **kwargs) -> TfChannel:
        """Create a transform channel"""
        channel = self.create_channel(ChannelType.TRANSFORM, topic, **kwargs)
        return cast(TfChannel, channel)
    
    def create_grid_channel(self, topic: str, **kwargs) -> GridChannel:
        """Create a grid channel"""
        channel = self.create_channel(ChannelType.GRID, topic, **kwargs)
        return cast(GridChannel, channel)
    
    def create_pointcloud_channel(self, topic: str, **kwargs) -> PointCloudChannel:
        """Create a point cloud channel"""
        channel = self.create_channel(ChannelType.POINT_CLOUD, topic, **kwargs)
        return cast(PointCloudChannel, channel)
    
    def create_laser_channel(self, topic: str, **kwargs) -> LaserScanChannel:
        """Create a laser scan channel"""
        channel = self.create_channel(ChannelType.LASER_SCAN, topic, **kwargs)
        return cast(LaserScanChannel, channel)
    
    def create_log_channel(self, topic: str = "/logs", **kwargs) -> LogChannel:
        """Create a log channel"""
        channel = self.create_channel(ChannelType.LOG, topic, **kwargs)
        return cast(LogChannel, channel)
    
    # Publishing methods
    
    def publish(self, channel_name: str, data: Any, timestamp: Optional[Any] = None) -> None:
        """
        Publish data to a channel by name
        
        Args:
            channel_name: Name of the channel
            data: Data to publish
            timestamp: Optional timestamp
        """
        channel = self.get_channel(channel_name)
        if not channel:
            raise KeyError(f"Channel '{channel_name}' not found")
        
        channel.publish(data, timestamp)
        self.total_messages_published += 1
    

    
    def broadcast(self, data: Dict[str, Any], channels: Optional[List[str]] = None) -> None:
        """
        Broadcast data to multiple channels
        
        Args:
            data: Dictionary where keys are channel names and values are data to publish
            channels: Optional list of channel names to limit broadcast to
        """
        target_channels = channels or list(self.channels.keys())
        
        for channel_name in target_channels:
            if channel_name in data and channel_name in self.channels:
                self.publish(channel_name, data[channel_name])
    
    # Context manager support
    
    def __enter__(self):
        """Context manager entry"""
        if not self.is_running:
            self.start_server()
        return self
    
    def __exit__(self, exc_type, exc_value, traceback):
        """Context manager exit"""
        self.stop_server()
    
    # Information and monitoring
    
    def get_status(self) -> Dict[str, Any]:
        """Get manager status information"""
        return {
            "is_running": self.is_running,
            "port": self.port,
            "mcap_enabled": self.mcap_enabled,
            "mcap_output_path": self.mcap_output_path,
            "channel_count": len(self.channels),
            "total_messages": self.total_messages_published,
            "created_at": self.created_at.isoformat(),
            "channels": {name: ch.get_info() for name, ch in self.channels.items()}
        }
    
    def print_status(self) -> None:
        """Print a formatted status report"""
        status = self.get_status()
        
        print(f"\n=== Foxglove Channel Manager Status ===")
        print(f"Server: {'Running' if status['is_running'] else 'Stopped'} (port {status['port']})")
        print(f"MCAP Recording: {'Enabled' if status['mcap_enabled'] else 'Disabled'}")
        if status['mcap_enabled']:
            print(f"MCAP File: {status['mcap_output_path']}")
        print(f"Channels: {status['channel_count']}")
        print(f"Total Messages: {status['total_messages']}")
        print(f"Created: {status['created_at']}")
        
        if status['channels']:
            print(f"\n--- Channels ---")
            for name, info in status['channels'].items():
                print(f"  {name}: {info['topic']} ({info['type']}) - {info['message_count']} msgs")
    
    # Iterator support
    
    def __iter__(self) -> Iterator[BaseChannel]:
        """Iterate over all channels"""
        return iter(self.channels.values())
    
    def __len__(self) -> int:
        """Get number of channels"""
        return len(self.channels)
    
    def __contains__(self, name: str) -> bool:
        """Check if channel exists"""
        return name in self.channels
