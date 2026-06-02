/**
 * @file common_types.hpp
 * @brief Common type definitions for Hybrid A* planner
 */

#pragma once

#include <vector>
#include <memory>
#include <cmath>
#include <tuple>
#include <functional>
#include <unordered_map>
#include <string>

namespace hybrid_astar {

/**
 * @brief Direction modes for vehicle motion
 */
enum class DirectionMode {
    NONE = 0,
    FORWARD = 1,
    BACKWARD = 2
};

/**
 * @brief Vehicle state representation
 */
struct State {
    double x = 0.0;           ///< Position x coordinate (m)
    double y = 0.0;           ///< Position y coordinate (m)
    double yaw = 0.0;         ///< Heading angle (rad)
    DirectionMode direction = DirectionMode::NONE;  ///< Motion direction
    double steer = 0.0;       ///< Steering angle (rad)
    
    State() = default;
    
    State(double x_, double y_, double yaw_, 
          DirectionMode direction_ = DirectionMode::FORWARD, 
          double steer_ = 0.0)
        : x(x_), y(y_), yaw(yaw_), direction(direction_), steer(steer_) {}
    
    bool operator==(const State& other) const;
};

/**
 * @brief Cost components for path planning
 */
struct Costs {
    double distance = 0.0;    ///< Distance cost
    double steering = 0.0;    ///< Steering cost
    double turning = 0.0;     ///< Turning cost
    double cusp = 0.0;        ///< Direction change cost
    
    Costs() = default;
    
    Costs(double dist, double steer, double turn, double cusp_cost)
        : distance(dist), steering(steer), turning(turn), cusp(cusp_cost) {}
    
    double total() const {
        return distance + steering + turning + cusp;
    }
};

/**
 * @brief Planning node for A* algorithm
 */
struct Node {
    State state;                                    ///< Vehicle state
    double g_cost = 0.0;                           ///< Cost from start
    double h_cost = 0.0;                           ///< Heuristic cost to goal
    std::shared_ptr<Node> parent = nullptr;        ///< Parent node
    Costs costs;                                   ///< Individual cost components
    std::vector<State> forward_simulation_trajectory; ///< Detailed trajectory
    
    Node() = default;
    
    Node(const State& state_, double g_cost_, std::shared_ptr<Node> parent_ = nullptr)
        : state(state_), g_cost(g_cost_), parent(parent_) {}
    
    double f_cost() const {
        return g_cost + h_cost;
    }
};

/**
 * @brief Node comparator for priority queue
 */
struct NodeComparator {
    bool operator()(const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) const {
        return a->f_cost() > b->f_cost();  // Min-heap based on f_cost
    }
};

/**
 * @brief Hash function for state discretization
 */
struct StateKeyHash {
    std::size_t operator()(const std::tuple<int, int, int, int>& key) const {
        auto h1 = std::hash<int>{}(std::get<0>(key));
        auto h2 = std::hash<int>{}(std::get<1>(key));
        auto h3 = std::hash<int>{}(std::get<2>(key));
        auto h4 = std::hash<int>{}(std::get<3>(key));
        return h1 ^ (h2 << 1) ^ (h3 << 2) ^ (h4 << 3);
    }
};

/**
 * @brief Planning configuration
 */
struct PlanningConfig {
    // Vehicle parameters
    double wheelbase = 2.5;           ///< Vehicle wheelbase (m)
    double max_steer = 0.6;           ///< Maximum steering angle (rad)
    
    // Discretization parameters
    double grid_resolution = 1.0;     ///< Grid resolution (m)
    double angle_resolution = 0.1;    ///< Angle resolution (rad)
    double steer_resolution = 0.1;    ///< Steering resolution (rad)
    
    // Motion parameters
    double velocity = 2.0;            ///< Planning velocity (m/s)
    double simulation_time = 1.0;     ///< Simulation time per step (s)
    double dt = 0.1;                  ///< Integration time step (s)
    
    // Cost weights
    double w_steer = 1.0;             ///< Steering cost weight
    double w_turn = 1.0;              ///< Turning cost weight
    double w_cusp = 2.0;              ///< Direction change cost weight
    
    // Goal tolerance
    double position_tolerance = 1.0;   ///< Position tolerance (m)
    double angle_tolerance = 0.2;      ///< Angle tolerance (rad)
    
    // Algorithm limits
    int max_iterations = 10000;       ///< Maximum planning iterations
    bool debug_enabled = false;       ///< Enable debug visualization
};

} // namespace hybrid_astar
