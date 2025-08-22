"""
Channel Management System for Foxglove SDK

This module provides a comprehensive channel management system with different
channel types for various data formats and use cases.

Channel Types:
- SceneUpdateChannel: For 3D scene visualization
- ProtoChannel: For custom protobuf messages
- DataChannel: For JSON data
- TfChannel: For coordinate transformations
- GridChannel: For grid/map data
- PointCloudChannel: For point cloud data
- LaserScanChannel: For laser scan data

Author: Generated for path_tracking project
Date: 2025-01-27
"""

import asyncio
import time
from abc import ABC, abstractmethod
from datetime import datetime
from pathlib import Path
from typing import Optional, Dict, Any, List, Union, Type, cast, TYPE_CHECKING
from enum import Enum

# Type checking imports
if TYPE_CHECKING:
    from foxglove.schemas import FrameTransform, FrameTransforms, SceneUpdate, SceneEntity
    from foxglove import Channel
    from foxglove.channels import (
        SceneUpdateChannel as FoxgloveSceneUpdateChannel,
        FrameTransformsChannel as FoxgloveFrameTransformsChannel,
        GridChannel as FoxgloveGridChannel,
        PointCloudChannel as FoxglovePointCloudChannel,
        LaserScanChannel as FoxgloveLaserScanChannel,
        LogChannel as FoxgloveLogChannel
    )

# Import Foxglove SDK components
try:
    import foxglove
    from foxglove import Channel, start_server, open_mcap
    from foxglove.mcap import MCAPWriter
    from foxglove.channels import (
        SceneUpdateChannel as FoxgloveSceneUpdateChannel,
        FrameTransformsChannel as FoxgloveFrameTransformsChannel,
        GridChannel as FoxgloveGridChannel,
        PointCloudChannel as FoxglovePointCloudChannel,
        LaserScanChannel as FoxgloveLaserScanChannel,
        LogChannel as FoxgloveLogChannel
    )
    from foxglove.schemas import (
        SceneUpdate, SceneEntity, FrameTransform, FrameTransforms,
        Grid, PointCloud, LaserScan, Log, Color, Point3, Vector3,
        Pose, Quaternion, Timestamp, PackedElementField
    )
    FOXGLOVE_AVAILABLE = True
except ImportError:
    print("Warning: Foxglove SDK not available. Please install with: pip install foxglove-sdk")
    FOXGLOVE_AVAILABLE = False


class ChannelType(Enum):
    """Enumeration of supported channel types"""
    SCENE_UPDATE = "scene_update"
    PROTO = "proto"
    DATA = "data"
    TRANSFORM = "transform"
    GRID = "grid"
    POINT_CLOUD = "point_cloud"
    LASER_SCAN = "laser_scan"
    LOG = "log"
    CUSTOM = "custom"


class BaseChannel(ABC):
    """
    Abstract base class for all channel types
    
    This class defines the common interface that all channel implementations
    must follow, providing consistent behavior across different data types.
    """
    
    def __init__(self, name: str, topic: str, channel_type: ChannelType, 
                 description: str = "", **kwargs):
        """
        Initialize the base channel
        
        Args:
            name: Unique identifier for this channel
            topic: Topic name for publishing
            channel_type: Type of channel from ChannelType enum
            description: Optional description of the channel
            **kwargs: Additional channel-specific parameters
        """
        if not FOXGLOVE_AVAILABLE:
            raise ImportError("Foxglove SDK is not available")
            
        self.name = name
        self.topic = topic
        self.channel_type = channel_type
        self.description = description
        self.created_at = datetime.now()
        self.message_count = 0
        
        # Create the underlying Foxglove channel
        self._foxglove_channel = self._create_foxglove_channel(**kwargs)
    
    @abstractmethod
    def _create_foxglove_channel(self, **kwargs) -> Channel:
        """Create the underlying Foxglove channel instance"""
        pass
    
    @abstractmethod
    def publish(self, data: Any, timestamp: Optional[Timestamp] = None) -> None:
        """
        Publish data to this channel
        
        Args:
            data: Data to publish (format depends on channel type)
            timestamp: Optional timestamp (defaults to current time)
        """
        pass
    
    def log(self, data: Any) -> None:
        """Alias for publish to match Foxglove SDK naming"""
        self.publish(data)
    
    @property
    def foxglove_channel(self) -> Channel:
        """Get the underlying Foxglove channel"""
        return self._foxglove_channel
    
    def get_info(self) -> Dict[str, Any]:
        """Get channel information"""
        return {
            "name": self.name,
            "topic": self.topic,
            "type": self.channel_type.value,
            "description": self.description,
            "message_count": self.message_count,
            "created_at": self.created_at.isoformat()
        }


class SceneUpdateChannel(BaseChannel):
    """Channel for 3D scene visualization using SceneUpdate messages"""
    
    def __init__(self, name: str, topic: str, description: str = "3D Scene Updates", **kwargs):
        super().__init__(name, topic, ChannelType.SCENE_UPDATE, description, **kwargs)
        self._foxglove_channel = self._create_foxglove_channel(**kwargs)
    
    def _create_foxglove_channel(self, **kwargs) -> FoxgloveSceneUpdateChannel:
        """Create a SceneUpdate channel"""
        return FoxgloveSceneUpdateChannel(topic=self.topic)
    
    def publish(self, data: Union[SceneUpdate, List[SceneEntity]], timestamp: Optional[Timestamp] = None) -> None:
        """
        Publish scene data
        
        Args:
            data: SceneUpdate object or list of SceneEntity objects
            timestamp: Optional timestamp
        """
        if isinstance(data, list):
            # Convert list of entities to SceneUpdate
            scene_update = SceneUpdate(entities=data, deletions=[])
        else:
            scene_update = data
        
        # Use the underlying Foxglove SceneUpdateChannel which accepts SceneUpdate objects
        try:
            self._foxglove_channel.log(scene_update)
        except (TypeError, AttributeError) as e:
            # Fallback: log a warning if direct schema logging fails
            print(f"Warning: Failed to log SceneUpdate directly: {e}")
        self.message_count += 1
    
    def clear_scene(self) -> None:
        """Clear all entities from the scene"""
        clear_update = SceneUpdate(deletions=[], entities=[])
        self.publish(clear_update)


class DataChannel(BaseChannel):
    """Channel for JSON data messages"""
    
    def __init__(self, name: str, topic: str, schema: Optional[Dict] = None, 
                 description: str = "JSON Data", **kwargs):
        self.schema = schema
        super().__init__(name, topic, ChannelType.DATA, description, **kwargs)
    
    def _create_foxglove_channel(self, **kwargs) -> Channel:
        """Create a data channel with optional JSON schema"""
        if self.schema:
            # Check if it's a Foxglove schema object (has get_schema method) or JSON dict
            if hasattr(self.schema, 'get_schema') or not isinstance(self.schema, dict):
                return Channel(topic=self.topic, message_encoding="protobuf", schema=self.schema)
            else:
                return Channel(topic=self.topic, schema=self.schema)
        return Channel(topic=self.topic)
    
    def publish(self, data: Dict[str, Any], timestamp: Optional[Timestamp] = None) -> None:
        """
        Publish JSON data
        
        Args:
            data: Dictionary to publish as JSON
            timestamp: Optional timestamp
        """
        if timestamp:
            data = dict(data)  # Create a copy
            data["timestamp"] = timestamp
            
        self._foxglove_channel.log(data)
        self.message_count += 1


class ProtoChannel(BaseChannel):
    """Channel for custom protobuf messages"""
    
    def __init__(self, name: str, topic: str, proto_class: Type, 
                 description: str = "Protobuf Messages", **kwargs):
        self.proto_class = proto_class
        super().__init__(name, topic, ChannelType.PROTO, description, **kwargs)
    
    def _create_foxglove_channel(self, **kwargs) -> Channel:
        """Create a protobuf channel"""
        # Import here to avoid circular dependencies
        try:
            from .proto_helper import create_proto_channel
            return create_proto_channel(self.topic, self.proto_class)
        except ImportError:
            # Fallback to regular channel if proto_helper not available
            return Channel(topic=self.topic)
    
    def publish(self, data: Any, timestamp: Optional[Timestamp] = None) -> None:
        """
        Publish protobuf message
        
        Args:
            data: Protobuf message instance
            timestamp: Optional timestamp in nanoseconds
        """
        try:
            from .proto_helper import log_proto_message
            ts = timestamp if timestamp is not None else Timestamp.now()
            timestamp_ns = int(ts.sec * 1e9 + ts.nsec)
            log_proto_message(self._foxglove_channel, data, timestamp_ns)
        except ImportError:
            # Fallback to regular logging
            self._foxglove_channel.log(data)
        
        self.message_count += 1


class TfChannel(BaseChannel):
    """Channel for coordinate transformations"""
    
    def __init__(self, name: str, topic: str = "/tf", 
                 description: str = "Coordinate Transformations", **kwargs):
        super().__init__(name, topic, ChannelType.TRANSFORM, description, **kwargs)
        self._foxglove_channel = self._create_foxglove_channel(**kwargs)

    def _create_foxglove_channel(self, **kwargs) -> FoxgloveFrameTransformsChannel:
        """Create a transforms channel"""
        return FoxgloveFrameTransformsChannel(topic=self.topic, **kwargs)
    
    def publish(self, data: Union[FrameTransform, FrameTransforms, List[FrameTransform]], 
                timestamp: Optional[Timestamp] = None) -> None:
        """
        Publish transform data
        
        Args:
            data: FrameTransform, FrameTransforms, or list of FrameTransform objects
            timestamp: Optional timestamp
        """
        transforms: FrameTransforms
        
        if isinstance(data, FrameTransform):
            transforms = FrameTransforms(transforms=[data])
        elif isinstance(data, list):
            # Ensure all items in list are FrameTransform objects
            valid_transforms = [t for t in data if isinstance(t, FrameTransform)]
            transforms = FrameTransforms(transforms=valid_transforms)
        else:
            transforms = data
        
        # Use the channel's log method for Foxglove schema objects
        try:
            self._foxglove_channel.log(transforms)
        except (TypeError, AttributeError) as e:
            print(f"Warning: Failed to log FrameTransforms directly: {e}")
        self.message_count += 1
    
    def publish_transform(self, parent_frame: str, child_frame: str,
                         translation: tuple[float, float, float],
                         rotation: tuple[float, float, float, float],
                         timestamp: Optional[Timestamp] = None) -> None:
        """
        Publish a single transform
        
        Args:
            parent_frame: Parent frame ID
            child_frame: Child frame ID  
            translation: (x, y, z) translation
            rotation: (x, y, z, w) quaternion rotation
            timestamp: Optional timestamp
        """
        ts = timestamp if timestamp is not None else Timestamp.now()
        
        transform = FrameTransform(
            timestamp=ts,
            parent_frame_id=parent_frame,
            child_frame_id=child_frame,
            translation=Vector3(x=translation[0], y=translation[1], z=translation[2]),
            rotation=Quaternion(x=rotation[0], y=rotation[1], z=rotation[2], w=rotation[3])
        )
        
        self.publish(transform, timestamp)


class GridChannel(BaseChannel):
    """Channel for grid/map data"""
    
    def __init__(self, name: str, topic: str, description: str = "Grid Data", **kwargs):
        super().__init__(name, topic, ChannelType.GRID, description, **kwargs)
        self._foxglove_channel = self._create_foxglove_channel(**kwargs)
    
    def _create_foxglove_channel(self, **kwargs) -> FoxgloveGridChannel:
        """Create a grid channel"""
        return FoxgloveGridChannel(topic=self.topic, **kwargs)
    
    def publish(self, data: Grid, timestamp: Optional[Timestamp] = None) -> None:
        """
        Publish grid data
        
        Args:
            data: Grid object
            timestamp: Optional timestamp
        """
        if hasattr(self._foxglove_channel, 'log') and hasattr(self._foxglove_channel.log, '__call__'):
            self._foxglove_channel.log(data)
        self.message_count += 1


class PointCloudChannel(BaseChannel):
    """Channel for point cloud data"""
    
    def __init__(self, name: str, topic: str, description: str = "Point Cloud Data", **kwargs):
        super().__init__(name, topic, ChannelType.POINT_CLOUD, description, **kwargs)
        self._foxglove_channel = self._create_foxglove_channel(**kwargs)

    def _create_foxglove_channel(self, **kwargs) -> FoxglovePointCloudChannel:
        """Create a point cloud channel"""
        return FoxglovePointCloudChannel(topic=self.topic, **kwargs)
    
    def publish(self, data: PointCloud, timestamp: Optional[Timestamp] = None) -> None:
        """
        Publish point cloud data
        
        Args:
            data: PointCloud object
            timestamp: Optional timestamp
        """
        if hasattr(self._foxglove_channel, 'log') and hasattr(self._foxglove_channel.log, '__call__'):
            self._foxglove_channel.log(data)
        self.message_count += 1


class LaserScanChannel(BaseChannel):
    """Channel for laser scan data"""
    
    def __init__(self, name: str, topic: str, description: str = "Laser Scan Data", **kwargs):
        super().__init__(name, topic, ChannelType.LASER_SCAN, description, **kwargs)
        self._foxglove_channel = self._create_foxglove_channel(**kwargs)

    def _create_foxglove_channel(self, **kwargs) -> FoxgloveLaserScanChannel:
        """Create a laser scan channel"""
        return FoxgloveLaserScanChannel(topic=self.topic, **kwargs)
    
    def publish(self, data: LaserScan, timestamp: Optional[Timestamp] = None) -> None:
        """
        Publish laser scan data
        
        Args:
            data: LaserScan object
            timestamp: Optional timestamp
        """
        if hasattr(self._foxglove_channel, 'log') and hasattr(self._foxglove_channel.log, '__call__'):
            self._foxglove_channel.log(data)
        self.message_count += 1


class LogChannel(BaseChannel):
    """Channel for log messages"""
    
    def __init__(self, name: str, topic: str = "/logs", description: str = "Log Messages", **kwargs):
        super().__init__(name, topic, ChannelType.LOG, description, **kwargs)
        self._foxglove_channel = self._create_foxglove_channel(**kwargs)
    
    def _create_foxglove_channel(self, **kwargs) -> FoxgloveLogChannel:
        """Create a log channel"""
        return FoxgloveLogChannel(topic=self.topic, **kwargs)
    
    def publish(self, data: Union[Log, str], timestamp: Optional[Timestamp] = None) -> None:
        """
        Publish log message
        
        Args:
            data: Log object or string message
            timestamp: Optional timestamp
        """
        if isinstance(data, str):
            from foxglove.schemas import LogLevel
            ts = timestamp if timestamp is not None else Timestamp.now()
            log_msg = Log(
                timestamp=ts,
                level=LogLevel.Info,
                message=data
            )
        else:
            log_msg = data
            
        if hasattr(self._foxglove_channel, 'log') and hasattr(self._foxglove_channel.log, '__call__'):
            self._foxglove_channel.log(log_msg)
        self.message_count += 1


class CustomChannel(BaseChannel):
    """Channel for custom data types"""
    
    def __init__(self, name: str, topic: str, schema: Optional[Dict] = None,
                 description: str = "Custom Data", **kwargs):
        self.schema = schema
        super().__init__(name, topic, ChannelType.CUSTOM, description, **kwargs)
    
    def _create_foxglove_channel(self, **kwargs) -> Channel:
        """Create a custom channel"""
        if self.schema:
            # Check if it's a Foxglove schema object (has get_schema method) or JSON dict
            if hasattr(self.schema, 'get_schema') or not isinstance(self.schema, dict):
                return Channel(topic=self.topic, message_encoding="protobuf", schema=self.schema)
            else:
                return Channel(topic=self.topic, schema=self.schema)
        return Channel(topic=self.topic)
    
    def publish(self, data: Any, timestamp: Optional[Timestamp] = None) -> None:
        """
        Publish custom data
        
        Args:
            data: Data to publish
            timestamp: Optional timestamp
        """
        self._foxglove_channel.log(data)
        self.message_count += 1


# Channel factory function
def create_channel(channel_type: ChannelType, name: str, topic: str, **kwargs) -> BaseChannel:
    """
    Factory function to create channels of different types
    
    Args:
        channel_type: Type of channel to create
        name: Channel name
        topic: Topic name
        **kwargs: Additional channel-specific parameters
        
    Returns:
        BaseChannel instance of the appropriate type
    """
    channel_classes = {
        ChannelType.SCENE_UPDATE: SceneUpdateChannel,
        ChannelType.DATA: DataChannel,
        ChannelType.PROTO: ProtoChannel,
        ChannelType.TRANSFORM: TfChannel,
        ChannelType.GRID: GridChannel,
        ChannelType.POINT_CLOUD: PointCloudChannel,
        ChannelType.LASER_SCAN: LaserScanChannel,
        ChannelType.LOG: LogChannel,
        ChannelType.CUSTOM: CustomChannel,
    }
    
    channel_class = channel_classes.get(channel_type)
    if not channel_class:
        raise ValueError(f"Unsupported channel type: {channel_type}")
    
    return channel_class(name, topic, **kwargs)
