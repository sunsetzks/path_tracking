#!/usr/bin/env python3
"""
Python implementation of robot model extracted from C++ codebase.
Includes RobotConfig and RobotModel classes for fork truck visualization.
"""

import matplotlib.pyplot as plt
import matplotlib.patches as patches
from dataclasses import dataclass, field
from typing import List, Tuple, Optional
import math


@dataclass
class Pose2D:
    """2D pose representation"""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


@dataclass
class Point2D:
    """2D point representation"""
    x: float = 0.0
    y: float = 0.0


class Rectangle:
    """Rectangle class for robot components"""
    
    def __init__(self, width: float, height: float, center: Point2D, pose: Pose2D):
        self.width = width
        self.height = height
        self.center = center
        self.pose = pose
    
    def get_corners(self) -> List[Tuple[float, float]]:
        """Get the four corners of the rectangle in world coordinates"""
        # Local corners relative to center
        local_corners = [
            (-self.width/2, -self.height/2),
            (self.width/2, -self.height/2),
            (self.width/2, self.height/2),
            (-self.width/2, self.height/2)
        ]
        
        # Transform to world coordinates
        cos_theta = math.cos(self.pose.theta)
        sin_theta = math.sin(self.pose.theta)
        
        world_corners = []
        for lx, ly in local_corners:
            # Translate to rectangle center first
            lx += self.center.x
            ly += self.center.y
            
            # Rotate and translate to world coordinates
            wx = cos_theta * lx - sin_theta * ly + self.pose.x
            wy = sin_theta * lx + cos_theta * ly + self.pose.y
            world_corners.append((wx, wy))
        
        return world_corners


class RobotConfig:
    """Python implementation of RobotConfig from system_config.h"""
    
    def __init__(self):
        # Robot body dimensions in base frame (physical values)
        self._robot_x_min = -1.0
        self._robot_x_max = 1.0
        self._robot_y_min = -1.0
        self._robot_y_max = 1.0
        
        # Fork specifications (physical values)
        self._robot_fork_length = 1.0
        self._robot_fork_width = 0.1
        self._robot_fork_default_offset_to_sides = 0.1
        self._robot_fork_center_ys = []  # Empty means use default calculation
        
        # X-axis translation for R car forks (in meters)
        self._r_car_fork_x_translation = 0.0
        
        # Per-side padding for collision/inflation
        self._robot_padding_front = 0.0
        self._robot_padding_back = 0.0
        self._robot_padding_left = 0.0
        self._robot_padding_right = 0.0
    
    def validate_and_correct_config(self):
        """Validates and corrects the configuration values"""
        # Ensure robot dimensions are properly ordered
        if self._robot_x_min > self._robot_x_max:
            self._robot_x_min, self._robot_x_max = self._robot_x_max, self._robot_x_min
        
        if self._robot_y_min > self._robot_y_max:
            self._robot_y_min, self._robot_y_max = self._robot_y_max, self._robot_y_min
        
        # Ensure robot body is centered around origin
        if self._robot_x_min > 0:
            self._robot_x_min = -self._robot_x_min
        if self._robot_x_max < 0:
            self._robot_x_max = -self._robot_x_max
        
        if self._robot_y_min > 0:
            self._robot_y_min = -self._robot_y_min
        if self._robot_y_max < 0:
            self._robot_y_max = -self._robot_y_max
        
        # Ensure positive dimensions
        self._robot_fork_length = abs(self._robot_fork_length)
        self._robot_fork_width = abs(self._robot_fork_width)
        
        # Sort and remove duplicate fork centers
        self._robot_fork_center_ys = sorted(list(set(self._robot_fork_center_ys)))
        
        # Verify fork spacing is valid (no overlaps)
        if len(self._robot_fork_center_ys) > 1:
            for i in range(len(self._robot_fork_center_ys) - 1):
                current_fork_max = self._robot_fork_center_ys[i] + self._robot_fork_width / 2
                next_fork_min = self._robot_fork_center_ys[i + 1] - self._robot_fork_width / 2
                
                if current_fork_max >= next_fork_min:
                    raise ValueError(
                        f"Invalid fork configuration: Fork at y={self._robot_fork_center_ys[i]} "
                        f"overlaps with fork at y={self._robot_fork_center_ys[i + 1]} "
                        f"(fork width={self._robot_fork_width})"
                    )
        
        # Ensure r_car_fork_x_translation is non-negative
        self._r_car_fork_x_translation = max(0.0, self._r_car_fork_x_translation)
    
    # Getters (default: return padded values)
    def get_robot_x_min(self) -> float:
        return self._robot_x_min - self._robot_padding_back
    
    def get_robot_x_max(self) -> float:
        return self._robot_x_max + self._robot_padding_front
    
    def get_robot_y_min(self) -> float:
        return self._robot_y_min - self._robot_padding_right
    
    def get_robot_y_max(self) -> float:
        return self._robot_y_max + self._robot_padding_left
    
    def get_robot_x_width(self) -> float:
        return self.get_robot_x_max() - self.get_robot_x_min()
    
    def get_robot_y_width(self) -> float:
        return self.get_robot_y_max() - self.get_robot_y_min()
    
    def get_robot_body_x_min(self) -> float:
        return self.get_robot_x_min() + self.get_robot_fork_length()
    
    def get_robot_fork_length(self) -> float:
        return self._robot_fork_length + self._robot_padding_back
    
    def get_robot_fork_width(self) -> float:
        return self._robot_fork_width
    
    def get_robot_fork_center_ys(self) -> List[float]:
        if not self._robot_fork_center_ys:
            fork_offset = self._robot_fork_default_offset_to_sides
            # Use physical extents for default fork positions
            return [self._robot_y_max - fork_offset, self._robot_y_min + fork_offset]
        return self._robot_fork_center_ys.copy()
    
    def get_robot_fork_center_xs(self) -> List[float]:
        fork_center_x = self._robot_x_min + self.get_robot_fork_length() / 2
        ys = self.get_robot_fork_center_ys()
        return [fork_center_x] * len(ys)
    
    def get_robot_fork_centers(self) -> List[Tuple[float, float]]:
        fork_centers_y = self.get_robot_fork_center_ys()
        fork_centers_x = self.get_robot_fork_center_xs()
        return list(zip(fork_centers_x, fork_centers_y))
    
    def get_robot_forks_span_y_width(self) -> float:
        """Calculate total width occupied by forks in Y direction"""
        fork_centers = self.get_robot_fork_centers()
        if not fork_centers:
            return 0.0
        
        max_y = max(center[1] for center in fork_centers)
        min_y = min(center[1] for center in fork_centers)
        return max_y - min_y + self._robot_fork_width
    
    def get_r_car_fork_x_translation(self) -> float:
        return self._r_car_fork_x_translation
    
    # Physical getters (unpadded values)
    def get_robot_x_min_physical(self) -> float:
        return self._robot_x_min
    
    def get_robot_x_max_physical(self) -> float:
        return self._robot_x_max
    
    def get_robot_y_min_physical(self) -> float:
        return self._robot_y_min
    
    def get_robot_y_max_physical(self) -> float:
        return self._robot_y_max
    
    # Setters
    def set_robot_dimensions(self, x_min: float, x_max: float, y_min: float, y_max: float):
        self._robot_x_min = x_min
        self._robot_x_max = x_max
        self._robot_y_min = y_min
        self._robot_y_max = y_max
    
    def set_fork_specifications(self, length: float, width: float, center_ys: Optional[List[float]] = None):
        self._robot_fork_length = length
        self._robot_fork_width = width
        if center_ys is not None:
            self._robot_fork_center_ys = center_ys.copy()
    
    def set_padding(self, front: float = 0.0, back: float = 0.0, left: float = 0.0, right: float = 0.0):
        self._robot_padding_front = front
        self._robot_padding_back = back
        self._robot_padding_left = left
        self._robot_padding_right = right


class State:
    """Simple state representation"""
    def __init__(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0, kappa: float = 0.0):
        self.x = x
        self.y = y
        self.theta = theta
        self.kappa = kappa


class RobotModel:
    """Python implementation of RobotModel from robot_model.h"""
    
    class WheelParams:
        front_wheel_offset = 0.1
        front_wheel_x_size = 0.25
        front_wheel_y_size = 0.1
    
    @staticmethod
    def create_main_body_rectangle(config: RobotConfig, robot_pose: Pose2D) -> Rectangle:
        """Create the main body rectangle"""
        # Use padded extents for collision/visualization inflation
        body_x_min = config.get_robot_x_min() + config.get_robot_fork_length()
        body_x_max = config.get_robot_x_max()
        body_y_min = config.get_robot_y_min()
        body_y_max = config.get_robot_y_max()
        
        width = body_x_max - body_x_min
        height = body_y_max - body_y_min
        center = Point2D((body_x_max + body_x_min) / 2.0, (body_y_max + body_y_min) / 2.0)
        
        return Rectangle(width, height, center, robot_pose)
    
    @staticmethod
    def create_robot_rectangle_bounding_box(config: RobotConfig, robot_pose: Pose2D) -> Rectangle:
        """Create robot bounding box rectangle"""
        x_min = config.get_robot_x_min()
        x_max = config.get_robot_x_max()
        y_min = config.get_robot_y_min()
        y_max = config.get_robot_y_max()
        
        width = x_max - x_min
        height = y_max - y_min
        center = Point2D((x_max + x_min) / 2.0, 0.0)
        
        return Rectangle(width, height, center, robot_pose)
    
    @staticmethod
    def generate_fork_rectangles(config: RobotConfig, pose: Pose2D) -> List[Rectangle]:
        """Generate rectangles for robot forks"""
        fork_center_x = config.get_robot_x_min() + config.get_robot_fork_length() / 2
        fork_centers_y = config.get_robot_fork_center_ys()
        
        rectangles = []
        for fork_y in fork_centers_y:
            center = Point2D(fork_center_x, fork_y)
            rectangle = Rectangle(
                config.get_robot_fork_length(),
                config.get_robot_fork_width(),
                center,
                pose
            )
            rectangles.append(rectangle)
        
        return rectangles
    
    @staticmethod
    def generate_robot_rectangles(state: State, config: RobotConfig, include_center_box: bool = True) -> List[Rectangle]:
        """Generate all robot component rectangles"""
        wheel_base = config.get_robot_x_max()
        delta = math.atan(state.kappa * wheel_base)
        robot_pose = Pose2D(state.x, state.y, state.theta)
        
        rectangles = []
        
        # Main body rectangle
        rectangles.append(RobotModel.create_main_body_rectangle(config, robot_pose))
        
        # Fork rectangles
        fork_rectangles = RobotModel.generate_fork_rectangles(config, robot_pose)
        rectangles.extend(fork_rectangles)
        
        # Center box (optional)
        if include_center_box:
            center_box = Rectangle(0.1, 0.1, Point2D(0, 0), robot_pose)
            rectangles.append(center_box)
        
        # Front wheel
        wheel_x = config.get_robot_x_max() - RobotModel.WheelParams.front_wheel_offset - RobotModel.WheelParams.front_wheel_x_size / 2.0
        wheel_pose = Pose2D(
            robot_pose.x + wheel_x * math.cos(robot_pose.theta),
            robot_pose.y + wheel_x * math.sin(robot_pose.theta),
            robot_pose.theta + delta
        )
        
        front_wheel = Rectangle(
            RobotModel.WheelParams.front_wheel_x_size,
            RobotModel.WheelParams.front_wheel_y_size,
            Point2D(0, 0),
            wheel_pose
        )
        rectangles.append(front_wheel)
        
        # Wheel direction indicator
        wheel_indicator = Rectangle(
            RobotModel.WheelParams.front_wheel_x_size / 2,
            0.002,
            Point2D(RobotModel.WheelParams.front_wheel_x_size / 4, 0),
            wheel_pose
        )
        rectangles.append(wheel_indicator)
        
        return rectangles
    
    @staticmethod
    def generate_fork_and_body_rectangles(config: RobotConfig, state: State) -> List[Rectangle]:
        """Generate rectangles for robot forks and the main body"""
        robot_pose = Pose2D(state.x, state.y, state.theta)
        rectangles = []
        
        # Main body rectangle
        rectangles.append(RobotModel.create_main_body_rectangle(config, robot_pose))
        
        # Fork rectangles
        fork_rectangles = RobotModel.generate_fork_rectangles(config, robot_pose)
        rectangles.extend(fork_rectangles)
        
        return rectangles


def plot_fork_truck(config: RobotConfig, state: State, title: str = "Fork Truck Visualization"):
    """Plot the fork truck using matplotlib"""
    # Generate all rectangles
    rectangles = RobotModel.generate_robot_rectangles(state, config, include_center_box=True)
    
    # Create figure and axis
    fig, ax = plt.subplots(1, 1, figsize=(12, 8))
    
    # Color scheme for different components
    colors = ['lightblue', 'orange', 'orange', 'red', 'gray', 'black']  # body, forks, center, wheel, indicator
    labels = ['Main Body', 'Fork 1', 'Fork 2', 'Center Box', 'Front Wheel', 'Wheel Direction']
    
    # Plot each rectangle
    for i, rect in enumerate(rectangles):
        corners = rect.get_corners()
        # Close the polygon by adding the first point at the end
        corners.append(corners[0])
        
        x_coords = [corner[0] for corner in corners]
        y_coords = [corner[1] for corner in corners]
        
        color = colors[i % len(colors)]
        label = labels[i] if i < len(labels) else f'Component {i}'
        
        # Create polygon patch
        polygon = patches.Polygon([(x, y) for x, y in zip(x_coords[:-1], y_coords[:-1])], 
                                closed=True, facecolor=color, edgecolor='black', alpha=0.7)
        ax.add_patch(polygon)
        
        # Add label at rectangle center
        if i < 4:  # Only label main components
            center_x = sum(x_coords[:-1]) / len(x_coords[:-1])
            center_y = sum(y_coords[:-1]) / len(y_coords[:-1])
            ax.annotate(label, (center_x, center_y), ha='center', va='center', fontsize=8)
    
    # Set axis properties
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_title(title)
    
    # Add coordinate system arrows at origin
    ax.arrow(0, 0, 0.5, 0, head_width=0.1, head_length=0.1, fc='red', ec='red', label='X-axis')
    ax.arrow(0, 0, 0, 0.5, head_width=0.1, head_length=0.1, fc='green', ec='green', label='Y-axis')
    
    # Add robot state information
    info_text = f"Robot State:\nX: {state.x:.2f} m\nY: {state.y:.2f} m\nθ: {math.degrees(state.theta):.1f}°\nκ: {state.kappa:.3f}"
    ax.text(0.02, 0.98, info_text, transform=ax.transAxes, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
    
    plt.tight_layout()
    return fig, ax


def main():
    """Demonstrate the fork truck visualization"""
    # Create robot configuration
    config = RobotConfig()
    
    # Set typical fork truck dimensions (in meters)
    config.set_robot_dimensions(x_min=-1.2, x_max=1.5, y_min=-0.8, y_max=0.8)
    config.set_fork_specifications(length=1.0, width=0.15, center_ys=[-0.3, 0.3])
    config.set_padding(front=0.1, back=0.1, left=0.05, right=0.05)
    
    config.validate_and_correct_config()
    
    # Create different robot states for visualization
    states = [
        State(x=0.0, y=0.0, theta=0.0, kappa=0.0),  # Straight
        State(x=3.0, y=2.0, theta=math.pi/4, kappa=0.2),  # Turning
        State(x=6.0, y=0.0, theta=math.pi/2, kappa=-0.1),  # Different orientation
    ]
    
    titles = [
        "Fork Truck - Straight Position",
        "Fork Truck - Turning Right", 
        "Fork Truck - 90° Rotation"
    ]
    
    # Create subplots for multiple views
    fig = plt.figure(figsize=(15, 5))
    
    for i, (state, title) in enumerate(zip(states, titles)):
        ax = fig.add_subplot(1, 3, i+1)
        
        rectangles = RobotModel.generate_robot_rectangles(state, config, include_center_box=True)
        
        colors = ['lightblue', 'orange', 'orange', 'red', 'gray', 'black']
        
        for j, rect in enumerate(rectangles):
            corners = rect.get_corners()
            corners.append(corners[0])  # Close polygon
            
            x_coords = [corner[0] for corner in corners]
            y_coords = [corner[1] for corner in corners]
            
            color = colors[j % len(colors)]
            polygon = patches.Polygon([(x, y) for x, y in zip(x_coords[:-1], y_coords[:-1])], 
                                    closed=True, facecolor=color, edgecolor='black', alpha=0.7)
            ax.add_patch(polygon)
        
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.set_xlabel('X (meters)')
        ax.set_ylabel('Y (meters)')
        ax.set_title(title)
        
        # Set reasonable axis limits
        ax.set_xlim(state.x - 3, state.x + 3)
        ax.set_ylim(state.y - 2, state.y + 2)
        
        # Add coordinate system
        ax.arrow(state.x, state.y, 0.5*math.cos(state.theta), 0.5*math.sin(state.theta), 
                head_width=0.1, head_length=0.1, fc='red', ec='red')
        ax.arrow(state.x, state.y, -0.5*math.sin(state.theta), 0.5*math.cos(state.theta), 
                head_width=0.1, head_length=0.1, fc='green', ec='green')
    
    plt.tight_layout()
    plt.show()
    
    # Print configuration details
    print("Fork Truck Configuration:")
    print(f"Body dimensions: X({config.get_robot_x_min():.2f} to {config.get_robot_x_max():.2f}), Y({config.get_robot_y_min():.2f} to {config.get_robot_y_max():.2f})")
    print(f"Fork length: {config.get_robot_fork_length():.2f} m")
    print(f"Fork width: {config.get_robot_fork_width():.2f} m")
    print(f"Fork centers Y: {config.get_robot_fork_center_ys()}")
    print(f"Forks span Y width: {config.get_robot_forks_span_y_width():.2f} m")


if __name__ == "__main__":
    main()
