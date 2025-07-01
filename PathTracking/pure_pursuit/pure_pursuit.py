"""
Path tracking simulation with pure pursuit steering and PID speed control.

author: Atsushi Sakai (@Atsushi_twi)
        Guillaume Jacquenot (@Gjacquenot)
"""
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from typing import List, Tuple, Optional, Union, Any

import sys
import pathlib
sys.path.append(str(pathlib.Path(__file__).parent.parent))
from utils.angle import angle_mod


class VehicleConfig:
    """Vehicle configuration parameters"""
    def __init__(self) -> None:
        # Control parameters
        self.k: float = 0.1  # look forward gain
        self.look_ahead_dist: float = 2.0  # [m] look-ahead distance
        self.speed_p_gain: float = 1.0  # speed proportional gain
        self.dt: float = 0.1  # [s] time tick
        
        # Vehicle dimensions
        self.wheelbase: float = 2.9  # [m] wheel base of vehicle
        self.length: float = self.wheelbase + 1.0  # Vehicle length
        self.width: float = 2.0  # Vehicle width
        self.wheel_length: float = 0.6  # Wheel length
        self.wheel_width: float = 0.2  # Wheel width
        self.max_steer: float = math.pi / 4  # Maximum steering angle [rad]


class VehicleState:
    """Vehicle state class"""
    def __init__(self, x: float = 0.0, y: float = 0.0, yaw: float = 0.0, v: float = 0.0) -> None:
        self.x: float = x  # x position (rear)
        self.y: float = y  # y position (rear)
        self.yaw: float = yaw  # heading angle
        self.v: float = v  # velocity
        # Direction is determined by velocity sign (negative v means reverse)

    def update(self, acceleration: float, delta: float, config: VehicleConfig) -> None:
        """Update vehicle state"""
        self.x += self.v * math.cos(self.yaw) * config.dt
        self.y += self.v * math.sin(self.yaw) * config.dt
        self.yaw += self.v / config.wheelbase * math.tan(delta) * config.dt
        self.yaw = angle_mod(self.yaw)
        self.v += acceleration * config.dt

    def calc_distance(self, point_x: float, point_y: float) -> float:
        """Calculate distance to a point"""
        dx = self.x - point_x
        dy = self.y - point_y
        return math.hypot(dx, dy)


class VehicleStateHistory:
    """Class to store vehicle state history"""
    def __init__(self) -> None:
        self.x: List[float] = []
        self.y: List[float] = []
        self.yaw: List[float] = []
        self.v: List[float] = []
        # Removed direction array
        self.t: List[float] = []

    def append(self, t: float, state: VehicleState) -> None:
        """Append state at time t"""
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        # Removed direction append
        self.t.append(t)


class TargetCourse:
    """Target course class"""
    def __init__(self, cx: List[float], cy: List[float]) -> None:
        self.cx: List[float] = cx
        self.cy: List[float] = cy
        self.old_nearest_point_index: Optional[int] = None

    def search_target_index(self, state: VehicleState, config: VehicleConfig, look_ahead: Optional[float] = None) -> Tuple[int, float]:
        """Search target index for pure pursuit control"""
        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.x - icx for icx in self.cx]
            dy = [state.y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind], self.cy[ind])
            while True:
                if (ind + 1) >= len(self.cx):
                    break
                distance_next_index = state.calc_distance(self.cx[ind + 1], self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        # Use provided look_ahead distance or calculate default
        if look_ahead is None:
            # Update look ahead distance based on velocity
            look_ahead = config.k * state.v + config.look_ahead_dist

        # search look ahead target point index
        while look_ahead > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, look_ahead


class PurePursuitController:
    """Pure pursuit controller class"""
    def __init__(self, config: VehicleConfig) -> None:
        self.config: VehicleConfig = config
    
    def speed_control(self, target_speed: float, current_speed: float, is_reverse: bool = False) -> float:
        """Proportional speed control"""
        # Negate target speed if in reverse mode
        actual_target = -abs(target_speed) if is_reverse else abs(target_speed)
        return self.config.speed_p_gain * (actual_target - current_speed)
    
    def calc_look_ahead_distance(self, state: VehicleState) -> float:
        """Calculate look-ahead distance based on velocity"""
        return self.config.k * abs(state.v) + self.config.look_ahead_dist
    
    def steering_control(self, state: VehicleState, trajectory: TargetCourse, prev_index: int) -> Tuple[float, int]:
        """Pure pursuit steering control"""
        look_ahead = self.calc_look_ahead_distance(state)
        ind, _ = trajectory.search_target_index(state, self.config, look_ahead)

        if prev_index >= ind:
            ind = prev_index

        if ind < len(trajectory.cx):
            tx = trajectory.cx[ind]
            ty = trajectory.cy[ind]
        else:
            tx = trajectory.cx[-1]
            ty = trajectory.cy[-1]
            ind = len(trajectory.cx) - 1

        alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

        # Calculate steering angle
        delta = math.atan2(2.0 * self.config.wheelbase * math.sin(alpha) / look_ahead, 1.0)

        # Limit steering angle to max value
        delta = np.clip(delta, -self.config.max_steer, self.config.max_steer)

        return delta, ind


class Visualization:
    """Class for visualization"""
    def __init__(self) -> None:
        self.pause_simulation: bool = False
        
        # Setup figure
        self.fig: plt.Figure
        self.ax: plt.Axes
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.fig.canvas.mpl_connect('key_release_event', self.on_key)
        
    def on_key(self, event: Any) -> None:
        """Keyboard event handler"""
        if event.key == ' ':  # Space key
            self.pause_simulation = not self.pause_simulation
        elif event.key == 'escape':
            plt.close()
            
    def plot_vehicle(self, state: VehicleState, steer: float, config: VehicleConfig) -> None:
        """Plot vehicle with wheels"""
        x, y, yaw = state.x, state.y, state.yaw
            
        # Vehicle body color
        color = 'blue'
        front_wheel_color = 'darkblue'
        
        # Calculate vehicle body corners
        corners = np.array([
            [-config.length/2, config.width/2],
            [config.length/2, config.width/2],
            [config.length/2, -config.width/2],
            [-config.length/2, -config.width/2],
            [-config.length/2, config.width/2]
        ])

        # Rotation matrix
        c, s = np.cos(yaw), np.sin(yaw)
        rot = np.array([[c, -s], [s, c]])

        # Rotate and translate vehicle body
        rotated = corners @ rot.T
        rotated[:, 0] += x
        rotated[:, 1] += y

        # Plot vehicle body
        self.ax.plot(rotated[:, 0], rotated[:, 1], color=color)
        
        # Plot wheels
        self._plot_wheel(x + config.length/4 * c - config.width/2 * s,
                        y + config.length/4 * s + config.width/2 * c,
                        yaw, steer, front_wheel_color, config)
        self._plot_wheel(x + config.length/4 * c + config.width/2 * s,
                        y + config.length/4 * s - config.width/2 * c,
                        yaw, steer, front_wheel_color, config)
        self._plot_wheel(x - config.length/4 * c - config.width/2 * s,
                        y - config.length/4 * s + config.width/2 * c,
                        yaw, 0, color, config)
        self._plot_wheel(x - config.length/4 * c + config.width/2 * s,
                        y - config.length/4 * s - config.width/2 * c,
                        yaw, 0, color, config)

        # Add direction arrow
        arrow_length = config.length/3
        self.ax.arrow(x, y,
                    arrow_length * math.cos(yaw),
                    arrow_length * math.sin(yaw),
                    head_width=config.width/4, head_length=config.width/4,
                    fc='r', ec='r', alpha=0.5)
    
    def _plot_wheel(self, x: float, y: float, yaw: float, steer: float, color: str, config: VehicleConfig) -> None:
        """Plot single wheel"""
        wheel = np.array([
            [-config.wheel_length/2, config.wheel_width/2],
            [config.wheel_length/2, config.wheel_width/2],
            [config.wheel_length/2, -config.wheel_width/2],
            [-config.wheel_length/2, -config.wheel_width/2],
            [-config.wheel_length/2, config.wheel_width/2]
        ])

        # Rotate wheel if steering
        if steer != 0:
            c, s = np.cos(steer), np.sin(steer)
            rot_steer = np.array([[c, -s], [s, c]])
            wheel = wheel @ rot_steer.T

        # Apply vehicle heading rotation
        c, s = np.cos(yaw), np.sin(yaw)
        rot_yaw = np.array([[c, -s], [s, c]])
        wheel = wheel @ rot_yaw.T

        # Translate to position
        wheel[:, 0] += x
        wheel[:, 1] += y

        # Plot wheel with color
        self.ax.plot(wheel[:, 0], wheel[:, 1], color=color)
        
    def update_plot(self, state: VehicleState, target_ind: int, course: TargetCourse, 
                   states: VehicleStateHistory, target_speed: float, 
                   controller: PurePursuitController, config: VehicleConfig) -> None:
        """Update plot with current simulation state"""
        self.ax.clear()
        
        # Plot course and trajectory
        self.ax.plot(course.cx, course.cy, "-r", label="course")
        self.ax.plot(states.x, states.y, "-b", label="trajectory")
        self.ax.plot(course.cx[target_ind], course.cy[target_ind], "xg", label="target")
        
        # Plot vehicle
        steer, _ = controller.steering_control(state, course, target_ind)
        self.plot_vehicle(state, steer, config)
        
        # Set plot properties
        self.ax.axis("equal")
        self.ax.grid(True)
        self.ax.set_title(f"Pure Pursuit Control - Speed: {state.v * 3.6:.1f} km/h")
        self.ax.legend()
        
        # Add pause state display
        if self.pause_simulation:
            self.ax.text(0.02, 0.95, 'PAUSED', transform=self.ax.transAxes,
                    bbox=dict(facecolor='red', alpha=0.5))
            
    def show_result_plots(self, course: TargetCourse, states: VehicleStateHistory) -> None:
        """Show result plots after simulation"""
        plt.figure(figsize=(10, 8))
        
        # Plot trajectory
        plt.subplot(2, 1, 1)
        plt.plot(course.cx, course.cy, ".r", label="course")
        plt.plot(states.x, states.y, "-b", label="trajectory")
        plt.legend()
        plt.xlabel("x[m]")
        plt.ylabel("y[m]")
        plt.axis("equal")
        plt.grid(True)
        
        # Plot speed profile
        plt.subplot(2, 1, 2)
        plt.plot(states.t, [v * 3.6 for v in states.v], "-r")
        plt.xlabel("Time[s]")
        plt.ylabel("Speed[km/h]")
        plt.grid(True)
        
        plt.tight_layout()
        plt.show()


class Simulation:
    """Simulation class"""
    def __init__(self, config: VehicleConfig, controller: PurePursuitController, visualizer: Optional[Visualization] = None) -> None:
        self.config: VehicleConfig = config
        self.controller: PurePursuitController = controller
        self.visualizer: Optional[Visualization] = visualizer
        self.show_animation: bool = visualizer is not None
        
    def run(self, course: TargetCourse, initial_state: VehicleState, 
           target_speed: float, max_time: float = 100.0) -> Tuple[VehicleStateHistory, bool]:
        """Run simulation"""
        state = initial_state
        time = 0.0
        states = VehicleStateHistory()
        states.append(time, state)
        
        target_ind, _ = course.search_target_index(state, self.config)
        last_index = len(course.cx) - 1
        
        while max_time >= time and last_index > target_ind:
            # Calculate control inputs
            # Check if we're in reverse mode based on target speed
            is_reverse = target_speed < 0
            acceleration = self.controller.speed_control(target_speed, state.v, is_reverse)
            steer, target_ind = self.controller.steering_control(state, course, target_ind)
            
            # Update state
            state.update(acceleration, steer, self.config)
            
            # Update time and history
            time += self.config.dt
            states.append(time, state)
            
            # Visualization
            if self.show_animation:
                self.visualizer.update_plot(state, target_ind, course, states, target_speed, self.controller, self.config)
                plt.pause(0.001)
                
                # Handle pause state
                while self.visualizer.pause_simulation and plt.get_fignums():
                    plt.pause(0.1)  # Reduce CPU usage
                
                # Check if window was closed
                if not plt.get_fignums():
                    return states, False
        
        goal_reached = last_index <= target_ind
        
        # Show result plots if animation was enabled
        if self.show_animation and goal_reached:
            self.visualizer.show_result_plots(course, states)
            
        return states, goal_reached


def main() -> None:
    """Main function"""
    # Create configuration
    config = VehicleConfig()
    
    # Create controller
    controller = PurePursuitController(config)
    
    # Create visualizer if animation is enabled
    visualizer = Visualization()
    
    # Create simulation
    simulation = Simulation(config, controller, visualizer)
    
    # Create course
    is_reverse_mode = True
    cx = -1 * np.arange(0, 50, 0.5) if is_reverse_mode else np.arange(0, 50, 0.5)
    cy = [math.sin(ix / 5.0) * ix / 2.0 for ix in cx]
    course = TargetCourse(cx, cy)
    
    # Set initial state and target speed
    initial_state = VehicleState(
        x=-0.0, 
        y=-3.0, 
        yaw=math.pi if is_reverse_mode else 0.0, 
        v=0.0
    )
    target_speed = 10.0 / 3.6  # [m/s]
    
    # Run simulation
    # If in reverse mode, make target speed negative
    target_speed = -abs(target_speed) if is_reverse_mode else abs(target_speed)
    states, goal_reached = simulation.run(course, initial_state, target_speed)
    
    # Check if goal was reached
    if goal_reached:
        print("Goal reached!")
    else:
        print("Failed to reach goal.")


if __name__ == '__main__':
    print("Pure pursuit path tracking simulation start")
    main()
