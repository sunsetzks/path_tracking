/**
 * @file hybrid_astar.hpp
 * @brief Hybrid A* path planning algorithm
 */

#pragma once

#include "common_types.hpp"
#include "vehicle_model.hpp"
#include "collision_detector.hpp"
#include <vector>
#include <memory>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <optional>
#include <tuple>
#include <functional>
#include <chrono>

namespace hybrid_astar {

/**
 * @brief Hybrid A* path planning algorithm implementation
 */
class HybridAStar {
public:
    /**
     * @brief Constructor
     * @param config Planning configuration
     */
    explicit HybridAStar(const PlanningConfig& config);
    
    /**
     * @brief Set collision detector for collision checking
     * @param collision_detector Shared pointer to collision detector implementation
     */
    void set_collision_detector(std::shared_ptr<CollisionDetector> collision_detector);
    
    /**
     * @brief Plan path from start to goal
     * @param start Start state
     * @param goal Goal state
     * @param max_iterations Maximum planning iterations
     * @return Optional path as vector of nodes
     */
    std::optional<std::vector<std::shared_ptr<Node>>> plan_path(
        const State& start, const State& goal, int max_iterations = -1);
    
    /**
     * @brief Extract detailed path from planning nodes
     * @param path_nodes Vector of planning nodes
     * @return Detailed state sequence
     */
    std::vector<State> extract_detailed_path(
        const std::vector<std::shared_ptr<Node>>& path_nodes) const;
    
    /**
     * @brief Get planning statistics
     * @param path Optional planning result
     * @return Statistics map
     */
    std::unordered_map<std::string, double> get_statistics(
        const std::optional<std::vector<std::shared_ptr<Node>>>& path) const;
    
    /**
     * @brief Get explored nodes for visualization
     */
    const std::vector<std::shared_ptr<Node>>& get_explored_nodes() const {
        return explored_nodes_;
    }
    
    /**
     * @brief Get simulation trajectories for visualization
     */
    const std::vector<std::vector<State>>& get_simulation_trajectories() const {
        return simulation_trajectories_;
    }
    
    /**
     * @brief Enable/disable debug mode
     */
    void set_debug_enabled(bool enabled) {
        debug_enabled_ = enabled;
    }
    
    /**
     * @brief Get last planning time in milliseconds
     */
    double get_last_planning_time_ms() const {
        return last_planning_time_ms_;
    }

private:
    // Configuration
    PlanningConfig config_;
    VehicleModel vehicle_model_;
    
    // Motion primitives
    std::vector<double> steer_rates_;
    int simulation_steps_;
    
    // Collision detection
    std::shared_ptr<CollisionDetector> collision_detector_;
    
    // Visualization data
    std::vector<std::shared_ptr<Node>> explored_nodes_;
    std::vector<std::vector<State>> simulation_trajectories_;
    bool debug_enabled_ = false;
    
    // Performance timing
    mutable std::chrono::steady_clock::time_point planning_start_time_;
    mutable double last_planning_time_ms_ = 0.0;
    
    /**
     * @brief Check if state is collision-free
     */
    bool is_collision_free(const State& state) const;
    
    /**
     * @brief Calculate heuristic cost to goal
     */
    double heuristic_cost(const State& state, const State& goal) const;
    
    /**
     * @brief Generate successor nodes
     */
    std::vector<std::shared_ptr<Node>> get_successors(const std::shared_ptr<Node>& node);
    
    /**
     * @brief Discretize state for duplicate detection
     */
    std::tuple<int, int, int, int> discretize_state(const State& state) const;
    
    /**
     * @brief Check if goal is reached
     */
    bool is_goal_reached(const State& current, const State& goal) const;
    
    /**
     * @brief Reconstruct path from goal node
     */
    std::vector<std::shared_ptr<Node>> reconstruct_path(const std::shared_ptr<Node>& goal_node) const;
};

} // namespace hybrid_astar
