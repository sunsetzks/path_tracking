"""
Trajectory planning utilities for MPC car tracking.

This module provides various trajectory generation methods including
straight lines, circular paths, figure-8 patterns, and spline-based paths.
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import List, Tuple, Optional, Callable
from dataclasses import dataclass
from scipy.interpolate import CubicSpline, interp1d
from scipy.optimize import minimize_scalar

from .vehicle_model import VehicleState, VehicleParameters


@dataclass
class TrajectoryPoint:
    """A point on the trajectory with state and optional curvature information.
    
    Attributes:
        state: Vehicle state at this point
        curvature: Path curvature (1/radius) at this point
        s: Arc length from trajectory start
        time: Time at this point
    """
    state: VehicleState
    curvature: float = 0.0
    s: float = 0.0
    time: float = 0.0


class TrajectoryPlanner:
    """Trajectory planner for generating reference paths."""
    
    def __init__(self, vehicle_params: Optional[VehicleParameters] = None):
        """Initialize trajectory planner.
        
        Args:
            vehicle_params: Vehicle parameters for constraint checking
        """
        self.vehicle_params = vehicle_params or VehicleParameters()
    
    def straight_line(self, 
                     start_state: VehicleState,
                     end_state: VehicleState,
                     num_points: int = 100,
                     target_speed: float = 10.0) -> List[TrajectoryPoint]:
        """Generate straight line trajectory.
        
        Args:
            start_state: Starting state
            end_state: Ending state
            num_points: Number of trajectory points
            target_speed: Target speed along trajectory
            
        Returns:
            List of trajectory points
        """
        # Linear interpolation between start and end
        x_points = np.linspace(start_state.x, end_state.x, num_points)
        y_points = np.linspace(start_state.y, end_state.y, num_points)
        
        # Calculate heading angles
        dx = end_state.x - start_state.x
        dy = end_state.y - start_state.y
        heading = np.arctan2(dy, dx)
        
        # Calculate arc lengths
        total_distance = np.sqrt(dx**2 + dy**2)
        s_points = np.linspace(0, total_distance, num_points)
        
        # Calculate time points based on target speed
        time_points = s_points / target_speed
        
        trajectory = []
        for i in range(num_points):
            state = VehicleState(
                x=x_points[i],
                y=y_points[i],
                yaw=heading,
                v=target_speed
            )
            trajectory.append(TrajectoryPoint(
                state=state,
                curvature=0.0,
                s=s_points[i],
                time=time_points[i]
            ))
        
        return trajectory
    
    def circular_path(self,
                     center: Tuple[float, float],
                     radius: float,
                     start_angle: float = 0.0,
                     end_angle: float = 2 * np.pi,
                     num_points: int = 100,
                     target_speed: float = 10.0,
                     clockwise: bool = False) -> List[TrajectoryPoint]:
        """Generate circular trajectory.
        
        Args:
            center: Circle center (x, y)
            radius: Circle radius
            start_angle: Start angle in radians
            end_angle: End angle in radians
            num_points: Number of trajectory points
            target_speed: Target speed along trajectory
            clockwise: Whether to go clockwise
            
        Returns:
            List of trajectory points
        """
        # Generate angle points
        if clockwise:
            angles = np.linspace(start_angle, start_angle - (end_angle - start_angle), num_points)
        else:
            angles = np.linspace(start_angle, end_angle, num_points)
        
        # Calculate positions
        x_points = center[0] + radius * np.cos(angles)
        y_points = center[1] + radius * np.sin(angles)
        
        # Calculate headings (tangent to circle)
        if clockwise:
            headings = angles - np.pi/2
        else:
            headings = angles + np.pi/2
        
        # Calculate arc lengths
        arc_length = radius * abs(end_angle - start_angle)
        s_points = np.linspace(0, arc_length, num_points)
        
        # Calculate time points
        time_points = s_points / target_speed
        
        # Curvature is constant for circle
        curvature = 1.0 / radius if not clockwise else -1.0 / radius
        
        trajectory = []
        for i in range(num_points):
            state = VehicleState(
                x=x_points[i],
                y=y_points[i],
                yaw=headings[i],
                v=target_speed
            )
            trajectory.append(TrajectoryPoint(
                state=state,
                curvature=curvature,
                s=s_points[i],
                time=time_points[i]
            ))
        
        return trajectory
    
    def figure_eight(self,
                    center: Tuple[float, float],
                    width: float,
                    height: float,
                    num_points: int = 200,
                    target_speed: float = 8.0) -> List[TrajectoryPoint]:
        """Generate figure-8 trajectory.
        
        Args:
            center: Center of figure-8
            width: Width of figure-8
            height: Height of figure-8
            num_points: Number of trajectory points
            target_speed: Target speed along trajectory
            
        Returns:
            List of trajectory points
        """
        # Parameter for figure-8 (lemniscate)
        t = np.linspace(0, 2*np.pi, num_points)
        
        # Lemniscate equations
        a = width / 2
        b = height / 2
        
        # Parametric equations for figure-8
        x_points = center[0] + a * np.cos(t) / (1 + np.sin(t)**2)
        y_points = center[1] + b * np.sin(t) * np.cos(t) / (1 + np.sin(t)**2)
        
        # Calculate headings from derivatives
        dx_dt = -a * np.sin(t) * (1 + np.sin(t)**2) - a * np.cos(t) * 2 * np.sin(t) * np.cos(t)
        dx_dt /= (1 + np.sin(t)**2)**2
        
        dy_dt = b * (np.cos(t)**2 - np.sin(t)**2) * (1 + np.sin(t)**2) - b * np.sin(t) * np.cos(t) * 2 * np.sin(t) * np.cos(t)
        dy_dt /= (1 + np.sin(t)**2)**2
        
        headings = np.arctan2(dy_dt, dx_dt)
        
        # Calculate arc lengths (approximate)
        s_points = np.zeros(num_points)
        for i in range(1, num_points):
            ds = np.sqrt((x_points[i] - x_points[i-1])**2 + (y_points[i] - y_points[i-1])**2)
            s_points[i] = s_points[i-1] + ds
        
        # Calculate time points
        time_points = s_points / target_speed
        
        # Calculate curvature (approximate)
        curvatures = np.zeros(num_points)
        for i in range(1, num_points-1):
            # Use finite differences to approximate curvature
            dx1 = x_points[i] - x_points[i-1]
            dy1 = y_points[i] - y_points[i-1]
            dx2 = x_points[i+1] - x_points[i]
            dy2 = y_points[i+1] - y_points[i]
            
            # Curvature formula
            cross_prod = dx1 * dy2 - dy1 * dx2
            norm_prod = (dx1**2 + dy1**2)**(3/2)
            
            if norm_prod > 1e-8:
                curvatures[i] = cross_prod / norm_prod
        
        trajectory = []
        for i in range(num_points):
            state = VehicleState(
                x=x_points[i],
                y=y_points[i],
                yaw=headings[i],
                v=target_speed
            )
            trajectory.append(TrajectoryPoint(
                state=state,
                curvature=curvatures[i],
                s=s_points[i],
                time=time_points[i]
            ))
        
        return trajectory
    
    def spline_path(self,
                   waypoints: List[Tuple[float, float]],
                   target_speed: float = 10.0,
                   num_points: int = 100,
                   smoothing_factor: float = 0.0) -> List[TrajectoryPoint]:
        """Generate smooth spline trajectory through waypoints.
        
        Args:
            waypoints: List of (x, y) waypoints
            target_speed: Target speed along trajectory
            num_points: Number of trajectory points
            smoothing_factor: Spline smoothing factor (0 = interpolation)
            
        Returns:
            List of trajectory points
        """
        if len(waypoints) < 2:
            raise ValueError("At least 2 waypoints required")
        
        # Extract x and y coordinates
        x_waypoints = [wp[0] for wp in waypoints]
        y_waypoints = [wp[1] for wp in waypoints]
        
        # Calculate cumulative distances for parameterization
        distances = [0.0]
        for i in range(1, len(waypoints)):
            dx = x_waypoints[i] - x_waypoints[i-1]
            dy = y_waypoints[i] - y_waypoints[i-1]
            distances.append(distances[-1] + np.sqrt(dx**2 + dy**2))
        
        # Create splines
        if len(waypoints) == 2:
            # Linear interpolation for 2 points
            spline_x = interp1d(distances, x_waypoints, kind='linear')
            spline_y = interp1d(distances, y_waypoints, kind='linear')
        else:
            # Cubic spline for more points
            spline_x = CubicSpline(distances, x_waypoints)
            spline_y = CubicSpline(distances, y_waypoints)
        
        # Generate trajectory points
        s_points = np.linspace(0, distances[-1], num_points)
        x_points = spline_x(s_points)
        y_points = spline_y(s_points)
        
        # Calculate headings from derivatives
        if len(waypoints) == 2:
            # Constant heading for linear path
            dx = x_waypoints[1] - x_waypoints[0]
            dy = y_waypoints[1] - y_waypoints[0]
            heading = np.arctan2(dy, dx)
            headings = np.full(num_points, heading)
        else:
            dx_ds = spline_x.derivative()(s_points)
            dy_ds = spline_y.derivative()(s_points)
            headings = np.arctan2(dy_ds, dx_ds)
        
        # Calculate curvature
        curvatures = np.zeros(num_points)
        if len(waypoints) > 2:
            d2x_ds2 = spline_x.derivative(2)(s_points)
            d2y_ds2 = spline_y.derivative(2)(s_points)
            
            for i in range(num_points):
                numerator = dx_ds[i] * d2y_ds2[i] - dy_ds[i] * d2x_ds2[i]
                denominator = (dx_ds[i]**2 + dy_ds[i]**2)**(3/2)
                if denominator > 1e-8:
                    curvatures[i] = numerator / denominator
        
        # Calculate time points
        time_points = s_points / target_speed
        
        trajectory = []
        for i in range(num_points):
            state = VehicleState(
                x=x_points[i],
                y=y_points[i],
                yaw=headings[i],
                v=target_speed
            )
            trajectory.append(TrajectoryPoint(
                state=state,
                curvature=curvatures[i],
                s=s_points[i],
                time=time_points[i]
            ))
        
        return trajectory
    
    def lane_change(self,
                   start_state: VehicleState,
                   lane_offset: float,
                   lane_change_distance: float,
                   target_speed: float = 15.0,
                   num_points: int = 50) -> List[TrajectoryPoint]:
        """Generate lane change trajectory.
        
        Args:
            start_state: Starting state
            lane_offset: Lateral offset for lane change
            lane_change_distance: Longitudinal distance for lane change
            target_speed: Target speed during maneuver
            num_points: Number of trajectory points
            
        Returns:
            List of trajectory points
        """
        # Use polynomial for smooth lane change
        x_points = np.linspace(0, lane_change_distance, num_points)
        
        # 5th order polynomial for smooth trajectory
        # Boundary conditions: y(0) = 0, y'(0) = 0, y''(0) = 0
        #                     y(L) = lane_offset, y'(L) = 0, y''(L) = 0
        L = lane_change_distance
        a5 = 6 * lane_offset / L**5
        a4 = -15 * lane_offset / L**4
        a3 = 10 * lane_offset / L**3
        
        y_local = a5 * x_points**5 + a4 * x_points**4 + a3 * x_points**3
        
        # Transform to global coordinates
        cos_yaw = np.cos(start_state.yaw)
        sin_yaw = np.sin(start_state.yaw)
        
        x_global = start_state.x + x_points * cos_yaw - y_local * sin_yaw
        y_global = start_state.y + x_points * sin_yaw + y_local * cos_yaw
        
        # Calculate headings
        dy_dx = 5*a5*x_points**4 + 4*a4*x_points**3 + 3*a3*x_points**2
        headings = start_state.yaw + np.arctan(dy_dx)
        
        # Calculate curvature
        d2y_dx2 = 20*a5*x_points**3 + 12*a4*x_points**2 + 6*a3*x_points
        curvatures = d2y_dx2 / (1 + dy_dx**2)**(3/2)
        
        # Calculate arc lengths and time
        s_points = np.zeros(num_points)
        for i in range(1, num_points):
            ds = np.sqrt((x_global[i] - x_global[i-1])**2 + (y_global[i] - y_global[i-1])**2)
            s_points[i] = s_points[i-1] + ds
        
        time_points = s_points / target_speed
        
        trajectory = []
        for i in range(num_points):
            state = VehicleState(
                x=x_global[i],
                y=y_global[i],
                yaw=headings[i],
                v=target_speed
            )
            trajectory.append(TrajectoryPoint(
                state=state,
                curvature=curvatures[i],
                s=s_points[i],
                time=time_points[i]
            ))
        
        return trajectory
    
    def resample_trajectory(self,
                          trajectory: List[TrajectoryPoint],
                          dt: float,
                          total_time: Optional[float] = None) -> List[VehicleState]:
        """Resample trajectory to fixed time intervals for MPC.
        
        Args:
            trajectory: Original trajectory
            dt: Desired time step
            total_time: Total time for resampling (if None, use trajectory duration)
            
        Returns:
            Resampled trajectory as list of vehicle states
        """
        if not trajectory:
            return []
        
        # Extract time and state data
        times = [tp.time for tp in trajectory]
        states = [tp.state for tp in trajectory]
        
        if total_time is None:
            total_time = times[-1]
        
        # Create interpolation functions
        x_interp = interp1d([tp.time for tp in trajectory], [tp.state.x for tp in trajectory], 
                           kind='linear', bounds_error=False, fill_value='extrapolate')
        y_interp = interp1d([tp.time for tp in trajectory], [tp.state.y for tp in trajectory], 
                           kind='linear', bounds_error=False, fill_value='extrapolate')
        yaw_interp = interp1d([tp.time for tp in trajectory], [tp.state.yaw for tp in trajectory], 
                             kind='linear', bounds_error=False, fill_value='extrapolate')
        v_interp = interp1d([tp.time for tp in trajectory], [tp.state.v for tp in trajectory], 
                           kind='linear', bounds_error=False, fill_value='extrapolate')
        
        # Resample at fixed intervals
        resampled_times = np.arange(0, total_time + dt, dt)
        resampled_states = []
        
        for t in resampled_times:
            state = VehicleState(
                x=float(x_interp(t)),
                y=float(y_interp(t)),
                yaw=float(yaw_interp(t)),
                v=float(v_interp(t))
            )
            resampled_states.append(state)
        
        return resampled_states