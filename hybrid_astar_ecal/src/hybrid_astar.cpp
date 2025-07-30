/**
 * @file hybrid_astar.cpp
 * @brief Implementation of Hybrid A* Path Planning Algorithm for eCAL
 */

#include "hybrid_astar.hpp"
#include <iostream>
#include <algorithm>
#include <cassert>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace hybrid_astar {

// State implementation
bool State::operator==(const State& other) const {
    const double tolerance = 0.1;
    return (std::abs(x - other.x) < tolerance &&
            std::abs(y - other.y) < tolerance &&
            std::abs(yaw - other.yaw) < tolerance);
}



// HybridAStar implementation
HybridAStar::HybridAStar(const PlanningConfig& config)
    : config_(config)
    , vehicle_model_(config.wheelbase, config.max_steer)
    , simulation_steps_(static_cast<int>(config.simulation_time / config.dt))
{
    // Initialize steering angle rates for motion primitives (rad/s)
    steer_rates_ = {-M_PI/2, -M_PI/4, 0, M_PI/4, M_PI/2};
}

void HybridAStar::set_obstacle_map(const std::vector<std::vector<int>>& obstacle_map,
                                  double origin_x, double origin_y) {
    obstacle_map_ = obstacle_map;
    map_height_ = static_cast<int>(obstacle_map_.size());
    map_width_ = obstacle_map_.empty() ? 0 : static_cast<int>(obstacle_map_[0].size());
    map_origin_x_ = origin_x;
    map_origin_y_ = origin_y;
}

bool HybridAStar::is_collision_free(const State& state) const {
    if (obstacle_map_.empty()) {
        return true;
    }
    
    // Convert world coordinates to grid coordinates
    int grid_x = static_cast<int>((state.x - map_origin_x_) / config_.grid_resolution);
    int grid_y = static_cast<int>((state.y - map_origin_y_) / config_.grid_resolution);
    
    // Check bounds
    if (grid_x < 0 || grid_x >= map_width_ || grid_y < 0 || grid_y >= map_height_) {
        return false;
    }
    
    // Check obstacle
    return obstacle_map_[grid_y][grid_x] == 0;
}

double HybridAStar::heuristic_cost(const State& state, const State& goal) const {
    double dx = goal.x - state.x;
    double dy = goal.y - state.y;
    double distance_cost = std::sqrt(dx*dx + dy*dy);
    
    // Angular difference cost
    double angle_diff = std::abs(VehicleModel::normalize_angle(goal.yaw - state.yaw));
    double angle_cost = angle_diff * 2.0;
    
    return distance_cost + angle_cost;
}

std::optional<std::vector<std::shared_ptr<Node>>> HybridAStar::plan_path(
    const State& start, const State& goal, int max_iterations) {
    
    if (max_iterations <= 0) {
        max_iterations = config_.max_iterations;
    }
    
    // Initialize visualization data
    explored_nodes_.clear();
    simulation_trajectories_.clear();
    
    // Initialize
    std::priority_queue<std::shared_ptr<Node>, 
                       std::vector<std::shared_ptr<Node>>, 
                       NodeComparator> open_list;
    std::unordered_set<std::tuple<int, int, int, int>, StateKeyHash> closed_set;
    std::unordered_map<std::tuple<int, int, int, int>, std::shared_ptr<Node>, StateKeyHash> node_map;
    
    // Create start node
    auto start_node = std::make_shared<Node>(start, 0.0);
    start_node->h_cost = heuristic_cost(start, goal);
    
    open_list.push(start_node);
    auto start_key = discretize_state(start);
    node_map[start_key] = start_node;
    
    int iterations = 0;
    
    while (!open_list.empty() && iterations < max_iterations) {
        iterations++;
        
        // Get node with lowest f_cost
        auto current_node = open_list.top();
        open_list.pop();
        auto current_key = discretize_state(current_node->state);
        
        // Check if already processed
        if (closed_set.find(current_key) != closed_set.end()) {
            continue;
        }
        
        closed_set.insert(current_key);
        
        // Store explored node for visualization
        if (debug_enabled_) {
            explored_nodes_.push_back(current_node);
        }
        
        // Check if goal reached
        if (is_goal_reached(current_node->state, goal)) {
            std::cout << "Path found in " << iterations << " iterations" << std::endl;
            return reconstruct_path(current_node);
        }
        
        // Generate successors
        auto successors = get_successors(current_node);
        
        for (auto& successor : successors) {
            auto successor_key = discretize_state(successor->state);
            
            // Skip if already in closed set
            if (closed_set.find(successor_key) != closed_set.end()) {
                continue;
            }
            
            // Calculate heuristic cost
            successor->h_cost = heuristic_cost(successor->state, goal);
            
            // Check if this path to successor is better
            auto it = node_map.find(successor_key);
            if (it != node_map.end()) {
                if (successor->g_cost < it->second->g_cost) {
                    open_list.push(successor);
                }
            } else {
                // Add new node
                node_map[successor_key] = successor;
                open_list.push(successor);
            }
        }
    }
    
    std::cout << "No path found after " << iterations << " iterations" << std::endl;
    return std::nullopt;
}

std::vector<std::shared_ptr<Node>> HybridAStar::get_successors(const std::shared_ptr<Node>& node) {
    std::vector<std::shared_ptr<Node>> successors;
    
    for (double steer_rate : steer_rates_) {
        for (auto direction : {DirectionMode::FORWARD, DirectionMode::BACKWARD}) {
            // Create new state with current direction
            State current_state(node->state.x, node->state.y, node->state.yaw, 
                              direction, node->state.steer);
            
            // Simulate motion
            std::vector<State> simulated_states = vehicle_model_.simulate_motion(
                current_state, config_.velocity, steer_rate, config_.dt, simulation_steps_);
            
            if (simulated_states.empty()) {
                continue;
            }
            
            State final_state = simulated_states.back();
            
            // Check collision for all states in trajectory
            bool collision_free = true;
            for (const auto& state : simulated_states) {
                if (!is_collision_free(state)) {
                    collision_free = false;
                    break;
                }
            }
            
            if (!collision_free) {
                continue;
            }
            
            // Calculate costs
            double distance_cost = config_.velocity * config_.simulation_time;
            double steer_cost = std::abs(final_state.steer - node->state.steer) / vehicle_model_.max_steer();
            
            double turn_cost = 0.0;
            double cusp_cost = 0.0;
            if (node->parent != nullptr) {
                double yaw_diff = std::abs(VehicleModel::normalize_angle(final_state.yaw - node->parent->state.yaw));
                turn_cost = yaw_diff;
                
                if (node->state.direction != direction) {
                    cusp_cost = 1.0;
                }
            }
            
            double total_cost = distance_cost + 
                               config_.w_steer * steer_cost + 
                               config_.w_turn * turn_cost + 
                               config_.w_cusp * cusp_cost;
            
            // Total g_cost
            double total_g_cost = node->g_cost + total_cost;
            
            // Create successor node
            auto successor = std::make_shared<Node>(final_state, total_g_cost, node);
            successor->costs = Costs(distance_cost, steer_cost, turn_cost, cusp_cost);
            
            // Store trajectory for visualization
            std::vector<State> trajectory = {current_state};
            trajectory.insert(trajectory.end(), simulated_states.begin(), simulated_states.end());
            successor->forward_simulation_trajectory = trajectory;
            
            successors.push_back(successor);
        }
    }
    
    return successors;
}

std::tuple<int, int, int, int> HybridAStar::discretize_state(const State& state) const {
    int grid_x = static_cast<int>(state.x / config_.grid_resolution);
    int grid_y = static_cast<int>(state.y / config_.grid_resolution);
    int grid_yaw = static_cast<int>(state.yaw / config_.angle_resolution);
    int grid_steer = static_cast<int>(state.steer / config_.steer_resolution);
    return std::make_tuple(grid_x, grid_y, grid_yaw, grid_steer);
}

bool HybridAStar::is_goal_reached(const State& current, const State& goal) const {
    double position_error = std::sqrt(std::pow(current.x - goal.x, 2) + 
                                     std::pow(current.y - goal.y, 2));
    double angle_error = std::abs(VehicleModel::normalize_angle(current.yaw - goal.yaw));
    
    return (position_error <= config_.position_tolerance && 
            angle_error <= config_.angle_tolerance);
}

std::vector<std::shared_ptr<Node>> HybridAStar::reconstruct_path(const std::shared_ptr<Node>& goal_node) const {
    std::vector<std::shared_ptr<Node>> path;
    std::shared_ptr<Node> current = goal_node;
    
    while (current != nullptr) {
        path.push_back(current);
        current = current->parent;
    }
    
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<State> HybridAStar::extract_detailed_path(const std::vector<std::shared_ptr<Node>>& path_nodes) const {
    if (path_nodes.empty()) {
        return {};
    }
    
    std::vector<State> detailed_path;
    
    // Add the start node state
    detailed_path.push_back(path_nodes[0]->state);
    
    // For each subsequent node, add its trajectory states
    for (size_t i = 1; i < path_nodes.size(); ++i) {
        const auto& node = path_nodes[i];
        if (!node->forward_simulation_trajectory.empty()) {
            // Add all except the first (already added)
            for (size_t j = 1; j < node->forward_simulation_trajectory.size(); ++j) {
                detailed_path.push_back(node->forward_simulation_trajectory[j]);
            }
        }
    }
    
    return detailed_path;
}

std::unordered_map<std::string, double> HybridAStar::get_statistics(
    const std::optional<std::vector<std::shared_ptr<Node>>>& path) const {
    
    std::unordered_map<std::string, double> stats;
    
    if (!path.has_value()) {
        stats["path_found"] = 0.0;
        stats["nodes_explored"] = static_cast<double>(explored_nodes_.size());
        stats["trajectories_simulated"] = static_cast<double>(simulation_trajectories_.size());
        return stats;
    }
    
    const auto& path_nodes = path.value();
    
    // Calculate basic statistics
    double total_distance = 0.0;
    double max_steer = 0.0;
    double avg_steer = 0.0;
    int direction_changes = 0;
    
    for (size_t i = 0; i < path_nodes.size(); ++i) {
        const auto& node = path_nodes[i];
        
        if (i > 0) {
            double dx = node->state.x - path_nodes[i-1]->state.x;
            double dy = node->state.y - path_nodes[i-1]->state.y;
            total_distance += std::sqrt(dx*dx + dy*dy);
            
            if (node->state.direction != path_nodes[i-1]->state.direction) {
                direction_changes++;
            }
        }
        
        double abs_steer = std::abs(node->state.steer);
        max_steer = std::max(max_steer, abs_steer);
        avg_steer += abs_steer;
    }
    
    if (!path_nodes.empty()) {
        avg_steer /= path_nodes.size();
    }
    
    // Fill statistics
    stats["path_found"] = 1.0;
    stats["path_length_waypoints"] = static_cast<double>(path_nodes.size());
    stats["total_distance"] = total_distance;
    stats["max_steering_angle"] = max_steer;
    stats["average_steering_angle"] = avg_steer;
    stats["steering_utilization"] = vehicle_model_.max_steer() > 0 ? max_steer / vehicle_model_.max_steer() : 0.0;
    stats["direction_changes"] = static_cast<double>(direction_changes);
    stats["nodes_explored"] = static_cast<double>(explored_nodes_.size());
    
    return stats;
}

} // namespace hybrid_astar
