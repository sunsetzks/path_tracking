"""
Vehicle State

This module contains the VehicleState dataclass used throughout the PathTracking project.
It's separated from vehicle_model.py to avoid circular imports with the estimators module.

Author: Assistant
"""

import math
from dataclasses import dataclass
from typing import Tuple, Union

import numpy as np


@dataclass
class VehicleState:
    """
    Structured representation of vehicle state

    Provides a clear interface for accessing and manipulating vehicle state components.
    All angles are in radians, distances in meters, velocities in m/s.
    """

    position_x: float = 0.0  # Vehicle position X coordinate [m]
    position_y: float = 0.0  # Vehicle position Y coordinate [m]
    yaw_angle: float = 0.0  # Vehicle heading angle [rad]
    velocity: float = 0.0  # Vehicle velocity [m/s]
    steering_angle: float = 0.0  # Steering wheel angle [rad]

    def to_array(self) -> np.ndarray:
        """
        Convert state to numpy array format

        Returns:
            np.ndarray: State as [position_x, position_y, yaw_angle, velocity, steering_angle]
        """
        return np.array(
            [
                self.position_x,
                self.position_y,
                self.yaw_angle,
                self.velocity,
                self.steering_angle,
            ]
        )

    @classmethod
    def from_array(cls, state_array: Union[np.ndarray, list]) -> "VehicleState":
        """
        Create VehicleState from array format

        Args:
            state_array: Array with [position_x, position_y, yaw_angle, velocity, steering_angle]

        Returns:
            VehicleState: New VehicleState instance
        """
        if len(state_array) != 5:
            raise ValueError("State array must have exactly 5 elements")

        return cls(
            position_x=float(state_array[0]),
            position_y=float(state_array[1]),
            yaw_angle=float(state_array[2]),
            velocity=float(state_array[3]),
            steering_angle=float(state_array[4]),
        )

    def get_position(self) -> Tuple[float, float]:
        """
        Get position as tuple

        Returns:
            Tuple[float, float]: (position_x, position_y)
        """
        return (self.position_x, self.position_y)

    def get_pose(self) -> Tuple[float, float, float]:
        """
        Get pose (position + orientation) as tuple

        Returns:
            Tuple[float, float, float]: (position_x, position_y, yaw_angle)
        """
        return (self.position_x, self.position_y, self.yaw_angle)

    def copy(self) -> "VehicleState":
        """
        Create a copy of the current state

        Returns:
            VehicleState: Copy of current state
        """
        return VehicleState(
            position_x=self.position_x,
            position_y=self.position_y,
            yaw_angle=self.yaw_angle,
            velocity=self.velocity,
            steering_angle=self.steering_angle,
        )

    def __str__(self) -> str:
        """String representation of vehicle state"""
        return (
            f"VehicleState(pos=({self.position_x:.2f}, {self.position_y:.2f}), "
            f"yaw={math.degrees(self.yaw_angle):.1f}°, "
            f"vel={self.velocity:.2f}m/s, "
            f"steer={math.degrees(self.steering_angle):.1f}°)"
        ) 