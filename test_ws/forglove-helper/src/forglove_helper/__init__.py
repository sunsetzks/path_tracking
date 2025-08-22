"""
Foxglove Helper Package

A reusable helper library for creating 3D visualizations with Foxglove SDK.

This package provides two approaches:
1. Original FoxgloveHelper - Simple interface for basic 3D visualization
2. New channel-based architecture - Advanced multi-channel management

Components:
- FoxgloveHelper: Original helper class
- EnhancedFoxgloveHelper: Enhanced version with channel architecture
- ChannelManager: Centralized channel management
- Channel Types: Specialized channels for different data types
- Utility Functions: Helpers for creating complex data structures
"""


# Try to import new channel system
try:
    from .channel_manager import ChannelManager, ChannelType
    from .channels import (
        BaseChannel, SceneUpdateChannel, DataChannel, ProtoChannel,
        TfChannel, GridChannel, PointCloudChannel, LaserScanChannel,
        LogChannel, CustomChannel, create_channel
    )
    from .channel_utils import (
        PrimitiveUtils, TransformUtils, GridUtils, 
        PointCloudUtils, LaserScanUtils, GridType
    )
    
    # Extended exports when channel system is available
    __all__ = [
        # Original
        'FoxgloveHelper',
        
        # Enhanced helper
        'EnhancedFoxgloveHelper',
        
        # Channel management
        'ChannelManager', 'ChannelType',
        
        # Channel types
        'BaseChannel', 'SceneUpdateChannel', 'DataChannel', 'ProtoChannel',
        'TfChannel', 'GridChannel', 'PointCloudChannel', 'LaserScanChannel',
        'LogChannel', 'CustomChannel', 'create_channel',
        
        # Utilities
        'PrimitiveUtils', 'TransformUtils', 'GridUtils', 
        'PointCloudUtils', 'LaserScanUtils', 'GridType'
    ]
    
    CHANNEL_SYSTEM_AVAILABLE = True

except ImportError as e:
    # Fallback when channel system is not available
    print(f"Warning: Channel system not fully available: {e}")
    __all__ = ['FoxgloveHelper']
    CHANNEL_SYSTEM_AVAILABLE = False

# Version info
__version__ = "2.0.0"
__author__ = "Generated for path_tracking project"

def main() -> None:
    print("Hello from forglove-helper!")
    print(f"Version: {__version__}")
    print(f"Channel system available: {CHANNEL_SYSTEM_AVAILABLE}")
    if CHANNEL_SYSTEM_AVAILABLE:
        print("✓ Enhanced features enabled")
    else:
        print("→ Basic features only")