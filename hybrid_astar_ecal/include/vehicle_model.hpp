/**
 * @file vehicle_model.hpp
 * @brief Vehicle kinematic model for path planning
 */

#pragma once

#include "common_types.hpp"
#include <vector>
#include <cmath>

namespace hybrid_astar {

/**
 * @brief Bicycle kinematic model for vehicle motion simulation
 */
class VehicleModel {
public:
    /**
     * @brief Constructor
     * @param wheelbase Vehicle wheelbase in meters
     * @param max_steer Maximum steering angle in radians
     */
    VehicleModel(double wheelbase, double max_steer);
    
    /**
     * @brief Simulate vehicle motion using bicycle model
     * @param state Initial vehicle state
     * @param velocity Forward velocity (m/s)
     * @param steer_rate Steering rate (rad/s)
     * @param dt Time step (s)
     * @param steps Number of simulation steps
     * @return Vector of simulated states
     */
    std::vector<State> simulate_motion(const State& state, double velocity,
                                      double steer_rate, double dt, int steps) const;
    
    /**
     * @brief Normalize angle to [-π, π]
     * @param angle Input angle in radians
     * @return Normalized angle
     */
    static double normalize_angle(double angle);
    
    /**
     * @brief Get maximum steering angle
     * @return Maximum steering angle in radians
     */
    double max_steer() const { return max_steer_; }
    
    /**
     * @brief Get wheelbase
     * @return Wheelbase in meters
     */
    double wheelbase() const { return wheelbase_; }

private:
    double wheelbase_;  ///< Vehicle wheelbase (m)
    double max_steer_;  ///< Maximum steering angle (rad)
};

} // namespace hybrid_astar
