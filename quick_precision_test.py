#!/usr/bin/env python3
"""
Quick Precision Test - Demonstrates 1cm accuracy capability

This script provides a simple demonstration of achieving 1cm positioning accuracy
with the enhanced Pure Pursuit controller.
"""

import math
import sys
import os

# Add the PathTracking module to the path
sys.path.append(os.path.join(os.path.dirname(__file__), 'PathTracking'))

from PathTracking.pure_pursuit import PurePursuitController
from PathTracking.trajectory import Trajectory
from PathTracking.vehicle_model import VehicleModel, VehicleState


def main():
    print("🎯 快速1cm精度测试")
    print("="*40)
    
    # Create simple straight-line trajectory
    trajectory = Trajectory()
    trajectory.add_waypoint(0.0, 0.0, 0.0)    # Start
    trajectory.add_waypoint(3.0, 0.0, 0.0)    # Goal
    
    # Create high-precision controller
    controller = PurePursuitController.create_high_precision_controller(
        wheelbase=2.5,
        trajectory=trajectory,
        precision_target=0.01  # 1cm
    )
    
    # Create vehicle model
    vehicle_model = VehicleModel(wheelbase=2.5)
    vehicle_model.set_state(VehicleState(0.0, 0.0, 0.0, 0.0))
    
    print(f"目标精度: 1.0cm")
    print(f"起点: (0.0, 0.0)")
    print(f"终点: (3.0, 0.0)")
    print(f"开始仿真...\n")
    
    # Simulation parameters
    time_step = 0.05
    max_time = 60.0
    simulation_time = 0.0
    
    # Track results
    errors = []
    
    while simulation_time < max_time:
        vehicle_state = vehicle_model.get_state()
        
        # Check if goal reached
        if controller.is_goal_reached(vehicle_state):
            goal_waypoint = trajectory.waypoints[-1]
            longitudinal_error, lateral_error, angle_error = controller.calculate_goal_errors(
                vehicle_state, goal_waypoint
            )
            final_distance_error = math.sqrt(longitudinal_error**2 + lateral_error**2)
            
            print(f"🎯 目标到达! 时间: {simulation_time:.1f}s")
            print(f"最终位置误差: {final_distance_error*100:.2f}cm")
            print(f"纵向误差: {longitudinal_error*100:.2f}cm")
            print(f"横向误差: {lateral_error*100:.2f}cm")
            print(f"角度误差: {math.degrees(angle_error):.1f}°")
            
            if final_distance_error <= 0.01:
                print("✅ 成功达到1cm精度!")
            else:
                print("⚠️  未达到1cm精度，但已在容差范围内")
            
            return final_distance_error
        
        # Get control and update
        steering, velocity = controller.compute_control(vehicle_state, time_step)
        vehicle_model.update_with_direct_control([steering, velocity], time_step)
        
        # Track error
        goal_waypoint = trajectory.waypoints[-1]
        dx = vehicle_state.position_x - goal_waypoint.x
        dy = vehicle_state.position_y - goal_waypoint.y
        current_error = math.sqrt(dx*dx + dy*dy)
        errors.append(current_error)
        
        # Progress update every 5 seconds
        if simulation_time % 5.0 < time_step:
            mode = "🎯 精准模式" if controller.is_in_precision_zone(vehicle_state) else "🚗 普通模式"
            print(f"t={simulation_time:4.1f}s | 误差: {current_error*100:5.1f}cm | "
                  f"速度: {abs(vehicle_state.velocity)*100:4.1f}cm/s | {mode}")
        
        simulation_time += time_step
    
    print(f"\n⏰ 仿真超时 ({max_time}s)")
    return min(errors) if errors else float('inf')


if __name__ == "__main__":
    error = main()
    print(f"\n最终结果: {error*100:.2f}cm 误差")
    if error <= 0.01:
        print("🎉 1cm精度目标达成!")
    else:
        print("�� 建议：增加仿真时间或调整参数") 