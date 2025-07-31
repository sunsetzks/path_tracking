/**
 * @file collision_detector.cpp
 * @brief Implementation of collision detection interfaces
 */

#include "collision_detector.hpp"
#include <cmath>

namespace hybrid_astar {

// GridCollisionDetector implementation
GridCollisionDetector::GridCollisionDetector(const std::vector<std::vector<int>>& obstacle_map,
                                           double grid_resolution,
                                           double origin_x,
                                           double origin_y)
    : obstacle_map_(obstacle_map)
    , grid_resolution_(grid_resolution)
    , origin_x_(origin_x)
    , origin_y_(origin_y)
{
    map_height_ = static_cast<int>(obstacle_map_.size());
    map_width_ = obstacle_map_.empty() ? 0 : static_cast<int>(obstacle_map_[0].size());
}

bool GridCollisionDetector::is_collision_free(const State& state) const {
    if (obstacle_map_.empty()) {
        return true;
    }
    
    // Convert world coordinates to grid coordinates
    int grid_x = static_cast<int>((state.x - origin_x_) / grid_resolution_);
    int grid_y = static_cast<int>((state.y - origin_y_) / grid_resolution_);
    
    // Check bounds
    if (grid_x < 0 || grid_x >= map_width_ || grid_y < 0 || grid_y >= map_height_) {
        return false;
    }
    
    // Check obstacle
    return obstacle_map_[grid_y][grid_x] == 0;
}

void GridCollisionDetector::update_obstacle_map(const std::vector<std::vector<int>>& obstacle_map,
                                               double origin_x,
                                               double origin_y) {
    obstacle_map_ = obstacle_map;
    origin_x_ = origin_x;
    origin_y_ = origin_y;
    map_height_ = static_cast<int>(obstacle_map_.size());
    map_width_ = obstacle_map_.empty() ? 0 : static_cast<int>(obstacle_map_[0].size());
}

// GeometricCollisionDetector implementation
GeometricCollisionDetector::GeometricCollisionDetector(double vehicle_radius)
    : vehicle_radius_(vehicle_radius) {
}

void GeometricCollisionDetector::add_circle_obstacle(double x, double y, double radius) {
    circle_obstacles_.emplace_back(x, y, radius);
}

void GeometricCollisionDetector::add_rectangle_obstacle(double x_min, double y_min, 
                                                       double x_max, double y_max) {
    rectangle_obstacles_.emplace_back(x_min, y_min, x_max, y_max);
}

void GeometricCollisionDetector::clear_obstacles() {
    circle_obstacles_.clear();
    rectangle_obstacles_.clear();
}

bool GeometricCollisionDetector::is_collision_free(const State& state) const {
    // Check collision with circular obstacles
    for (const auto& circle : circle_obstacles_) {
        double dx = state.x - circle.x;
        double dy = state.y - circle.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        if (distance < (vehicle_radius_ + circle.radius)) {
            return false;
        }
    }
    
    // Check collision with rectangular obstacles
    for (const auto& rect : rectangle_obstacles_) {
        // Check if vehicle center is within expanded rectangle
        double expanded_x_min = rect.x_min - vehicle_radius_;
        double expanded_y_min = rect.y_min - vehicle_radius_;
        double expanded_x_max = rect.x_max + vehicle_radius_;
        double expanded_y_max = rect.y_max + vehicle_radius_;
        
        if (state.x >= expanded_x_min && state.x <= expanded_x_max &&
            state.y >= expanded_y_min && state.y <= expanded_y_max) {
            return false;
        }
    }
    
    return true;
}

} // namespace hybrid_astar
