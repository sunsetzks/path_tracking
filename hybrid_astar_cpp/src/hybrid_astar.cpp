/**
 * @file hybrid_astar.cpp
 * @brief Implementation of Hybrid A* Path Planning Algorithm
 * 
 * @author Converted from Python implementation
 * @date 2025-07-29
 */

#include "hybrid_astar.hpp"
#include <iostream>
#include <algorithm>
#include <cassert>

namespace hybrid_astar {

// State implementation
bool State::operator==(const State& other) const {
    const double tolerance = 0.1;
    return (std::abs(x - other.x) < tolerance &&
            std::abs(y - other.y) < tolerance &&
            std::abs(yaw - other.yaw) < tolerance);
}

// StateHash implementation
std::size_t StateHash::operator()(const State& state) const {
    auto h1 = std::hash<int>{}(static_cast<int>(std::round(state.x * 100)));
    auto h2 = std::hash<int>{}(static_cast<int>(std::round(state.y * 100)));
    auto h3 = std::hash<int>{}(static_cast<int>(std::round(state.yaw * 100)));
    return h1 ^ (h2 << 1) ^ (h3 << 2);
}

// VehicleModel implementation
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

// HybridAStar implementation
HybridAStar::HybridAStar(const VehicleModel& vehicle_model,
                        double grid_resolution,
                        double angle_resolution,
                        double steer_resolution,
                        double velocity,
                        double simulation_time,
                        double dt)
    : vehicle_model_(vehicle_model)
    , grid_resolution_(grid_resolution)
    , angle_resolution_(angle_resolution)
    , steer_resolution_(steer_resolution)
    , velocity_(velocity)
    , simulation_time_(simulation_time)
    , dt_(dt)
    , simulation_steps_(static_cast<int>(simulation_time / dt))
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
    int grid_x = static_cast<int>((state.x - map_origin_x_) / grid_resolution_);
    int grid_y = static_cast<int>((state.y - map_origin_y_) / grid_resolution_);
    
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

double HybridAStar::calculate_steering_cost(double steer_angle) const {
    return std::abs(steer_angle) / vehicle_model_.max_steer();
}

double HybridAStar::calculate_turning_cost(double prev_yaw, double current_yaw) const {
    double yaw_diff = std::abs(VehicleModel::normalize_angle(current_yaw - prev_yaw));
    return yaw_diff;
}

double HybridAStar::calculate_cusp_cost(DirectionMode prev_direction, DirectionMode current_direction) const {
    // If either direction is NONE, no cusp cost
    if (prev_direction == DirectionMode::NONE || current_direction == DirectionMode::NONE) {
        return 0.0;
    }
    
    // Only count cusp cost for changes between FORWARD and BACKWARD
    if (prev_direction != current_direction) {
        return 1.0;
    }
    return 0.0;
}

std::pair<double, Costs> HybridAStar::calculate_total_cost(const std::shared_ptr<Node>& node,
                                                          const State& final_state,
                                                          DirectionMode direction) const {
    // Calculate individual costs
    double distance_cost = velocity_ * simulation_time_;
    double steer_cost = calculate_steering_cost(final_state.steer - node->state.steer);
    
    // Turning cost (requires parent)
    double turn_cost = 0.0;
    double cusp_cost = 0.0;
    if (node->parent != nullptr) {
        turn_cost = calculate_turning_cost(node->parent->state.yaw, final_state.yaw);
        cusp_cost = calculate_cusp_cost(node->state.direction, direction);
    }
    
    // Create costs object
    Costs costs(distance_cost, steer_cost, turn_cost, cusp_cost);
    
    // Calculate total weighted cost
    double total_cost = distance_cost + 
                       w_steer_ * steer_cost + 
                       w_turn_ * turn_cost + 
                       w_cusp_ * cusp_cost;
    
    return std::make_pair(total_cost, costs);
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
                current_state, velocity_, steer_rate, dt_, simulation_steps_);
            
            if (simulated_states.empty()) {
                continue;
            }
            
            State final_state = simulated_states.back();
            
            // Store simulation trajectory for visualization (only if debug is enabled)
            std::vector<State> trajectory = {current_state};
            trajectory.insert(trajectory.end(), simulated_states.begin(), simulated_states.end());
            
            if (debug_enabled_) {
                simulation_trajectories_.push_back(trajectory);
            }
            
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
            
            // Calculate all costs using the dedicated method
            auto [total_cost, costs] = calculate_total_cost(node, final_state, direction);
            
            // Total g_cost
            double total_g_cost = node->g_cost + total_cost;
            
            // Create successor node
            auto successor = std::make_shared<Node>(final_state, total_g_cost, node);
            successor->costs = costs;
            successor->forward_simulation_trajectory = trajectory;
            
            successors.push_back(successor);
        }
    }
    
    return successors;
}

std::tuple<int, int, int, int> HybridAStar::discretize_state(const State& state) const {
    int grid_x = static_cast<int>(state.x / grid_resolution_);
    int grid_y = static_cast<int>(state.y / grid_resolution_);
    int grid_yaw = static_cast<int>(state.yaw / angle_resolution_);
    int grid_steer = static_cast<int>(state.steer / steer_resolution_);
    return std::make_tuple(grid_x, grid_y, grid_yaw, grid_steer);
}

bool HybridAStar::is_goal_reached(const State& current, const State& goal,
                                 double position_tolerance, double angle_tolerance) const {
    double position_error = std::sqrt(std::pow(current.x - goal.x, 2) + 
                                     std::pow(current.y - goal.y, 2));
    double angle_error = std::abs(VehicleModel::normalize_angle(current.yaw - goal.yaw));
    
    return (position_error <= position_tolerance && angle_error <= angle_tolerance);
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

std::optional<std::vector<std::shared_ptr<Node>>> HybridAStar::plan_path(
    const State& start, const State& goal, int max_iterations) {
    
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
        
        // Store explored node for visualization (only if debug is enabled)
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

std::vector<State> HybridAStar::extract_detailed_path(const std::vector<std::shared_ptr<Node>>& path_nodes) const {
    if (path_nodes.empty()) {
        return {};
    }
    
    std::vector<State> detailed_path;
    
    // Add the start node state
    detailed_path.push_back(path_nodes[0]->state);
    State prev_final_state = path_nodes[0]->state;
    
    // For each subsequent node, add its trajectory states (excluding the first to avoid duplication)
    for (size_t i = 1; i < path_nodes.size(); ++i) {
        const auto& node = path_nodes[i];
        if (!node->forward_simulation_trajectory.empty()) {
            // Assert the first trajectory state matches the previous final state
            assert(node->forward_simulation_trajectory[0] == prev_final_state);
            
            // Add all except the first (already added)
            for (size_t j = 1; j < node->forward_simulation_trajectory.size(); ++j) {
                detailed_path.push_back(node->forward_simulation_trajectory[j]);
            }
            prev_final_state = node->forward_simulation_trajectory.back();
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
    
    // Extract states from nodes
    std::vector<State> states;
    for (const auto& node : path_nodes) {
        states.push_back(node->state);
    }
    
    // Basic path statistics
    double total_distance = 0.0;
    for (size_t i = 0; i < states.size() - 1; ++i) {
        double dx = states[i+1].x - states[i].x;
        double dy = states[i+1].y - states[i].y;
        total_distance += std::sqrt(dx*dx + dy*dy);
    }
    
    // Steering statistics
    std::vector<double> steer_angles;
    for (const auto& state : states) {
        steer_angles.push_back(state.steer);
    }
    
    double max_steer = 0.0;
    double avg_steer = 0.0;
    if (!steer_angles.empty()) {
        for (double steer : steer_angles) {
            double abs_steer = std::abs(steer);
            max_steer = std::max(max_steer, abs_steer);
            avg_steer += abs_steer;
        }
        avg_steer /= steer_angles.size();
    }
    
    // Direction changes
    int direction_changes = 0;
    for (size_t i = 1; i < states.size(); ++i) {
        if (states[i].direction != states[i-1].direction) {
            direction_changes++;
        }
    }
    
    // Curvature analysis
    std::vector<double> curvatures;
    for (size_t i = 1; i < states.size() - 1; ++i) {
        double x1 = states[i-1].x, y1 = states[i-1].y;
        double x2 = states[i].x, y2 = states[i].y;
        double x3 = states[i+1].x, y3 = states[i+1].y;
        
        double a = std::sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
        double b = std::sqrt((x3-x2)*(x3-x2) + (y3-y2)*(y3-y2));
        double c = std::sqrt((x3-x1)*(x3-x1) + (y3-y1)*(y3-y1));
        
        if (a > 1e-6 && b > 1e-6 && c > 1e-6) {
            double cross_product = (x2-x1)*(y3-y1) - (y2-y1)*(x3-x1);
            double area = 0.5 * std::abs(cross_product);
            double curvature = 4 * area / (a * b * c);
            curvatures.push_back(curvature);
        }
    }
    
    double max_curvature = 0.0;
    double avg_curvature = 0.0;
    if (!curvatures.empty()) {
        for (double curv : curvatures) {
            max_curvature = std::max(max_curvature, curv);
            avg_curvature += curv;
        }
        avg_curvature /= curvatures.size();
    }
    
    // Fill statistics
    stats["path_found"] = 1.0;
    stats["path_length_waypoints"] = static_cast<double>(path_nodes.size());
    stats["total_distance"] = total_distance;
    stats["average_waypoint_spacing"] = path_nodes.size() > 1 ? total_distance / (path_nodes.size() - 1) : 0.0;
    stats["max_steering_angle"] = max_steer;
    stats["average_steering_angle"] = avg_steer;
    stats["steering_utilization"] = vehicle_model_.max_steer() > 0 ? max_steer / vehicle_model_.max_steer() : 0.0;
    stats["direction_changes"] = static_cast<double>(direction_changes);
    stats["max_curvature"] = max_curvature;
    stats["average_curvature"] = avg_curvature;
    stats["nodes_explored"] = static_cast<double>(explored_nodes_.size());
    stats["trajectories_simulated"] = static_cast<double>(simulation_trajectories_.size());
    
    return stats;
}

} // namespace hybrid_astar
