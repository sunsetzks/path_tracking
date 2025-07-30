/**
 * @file hybrid_astar.hpp
 * @brief Hybrid A* path planning algorithm
 */

#pragma once

#include "common_types.hpp"
#include "vehicle_model.hpp"
#include <vector>
#include <memory>
#include <queue>
#include <unordered_set>
#include <unordered_map>
#include <optional>
#include <tuple>
#include <functional>

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
     * @brief Set obstacle map for collision checking
     * @param obstacle_map 2D grid map (0=free, 1=occupied)
     * @param origin_x Map origin x coordinate (m)
     * @param origin_y Map origin y coordinate (m)
     */
    void set_obstacle_map(const std::vector<std::vector<int>>& obstacle_map,
                          double origin_x, double origin_y);
    
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

private:
    // Configuration
    PlanningConfig config_;
    VehicleModel vehicle_model_;
    
    // Motion primitives
    std::vector<double> steer_rates_;
    int simulation_steps_;
    
    // Obstacle map
    std::vector<std::vector<int>> obstacle_map_;
    int map_width_ = 0;
    int map_height_ = 0;
    double map_origin_x_ = 0.0;
    double map_origin_y_ = 0.0;
    
    // Visualization data
    std::vector<std::shared_ptr<Node>> explored_nodes_;
    std::vector<std::vector<State>> simulation_trajectories_;
    bool debug_enabled_ = false;
    
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
