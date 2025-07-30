/**
 * @file vehicle_model.cpp
 * @brief Implementation of vehicle kinematic model
 */

#include "vehicle_model.hpp"
#include <algorithm>
#include <cmath>

namespace hybrid_astar {

VehicleModel::VehicleModel(double wheelbase, double max_steer)
    : wheelbase_(wheelbase), max_steer_(max_steer) {}

std::vector<State> VehicleModel::simulate_motion(const State& state, double velocity,
                                                double steer_rate, double dt, int steps) const {
    std::vector<State> states;
    State current_state = state;
    
    for (int i = 0; i < steps; ++i) {
        // Update steering angle
        double new_steer = current_state.steer + steer_rate * dt;
        new_steer = std::clamp(new_steer, -max_steer_, max_steer_);
        
        // Bicycle model kinematics
        double v = (current_state.direction == DirectionMode::FORWARD) ? velocity : -velocity;
        
        // Update state using bicycle model
        double new_x = current_state.x + v * std::cos(current_state.yaw) * dt;
        double new_y = current_state.y + v * std::sin(current_state.yaw) * dt;
        double new_yaw = current_state.yaw + v * std::tan(new_steer) / wheelbase_ * dt;
        
        // Normalize yaw angle
        new_yaw = normalize_angle(new_yaw);
        
        current_state = State(new_x, new_y, new_yaw, current_state.direction, new_steer);
        states.push_back(current_state);
    }
    
    return states;
}

double VehicleModel::normalize_angle(double angle) {
    while (angle > M_PI) {
        angle -= 2 * M_PI;
    }
    while (angle < -M_PI) {
        angle += 2 * M_PI;
    }
    return angle;
}

} // namespace hybrid_astar
