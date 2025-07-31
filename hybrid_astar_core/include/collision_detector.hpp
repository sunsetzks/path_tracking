/**
 * @file collision_detector.hpp
 * @brief Collision detection interface for hybrid A* path planning
 */

#pragma once

#include "common_types.hpp"
#include <vector>

namespace hybrid_astar {

/**
 * @brief Abstract base class for collision detection
 */
class CollisionDetector {
public:
    virtual ~CollisionDetector() = default;
    
    /**
     * @brief Check if a state is collision-free
     * @param state The state to check
     * @return true if collision-free, false otherwise
     */
    virtual bool is_collision_free(const State& state) const = 0;
    
    /**
     * @brief Check if a trajectory is collision-free
     * @param trajectory Vector of states forming a trajectory
     * @return true if entire trajectory is collision-free, false otherwise
     */
    virtual bool is_trajectory_collision_free(const std::vector<State>& trajectory) const {
        for (const auto& state : trajectory) {
            if (!is_collision_free(state)) {
                return false;
            }
        }
        return true;
    }
};

/**
 * @brief Grid-based collision detector implementation
 */
class GridCollisionDetector : public CollisionDetector {
public:
    /**
     * @brief Constructor
     * @param obstacle_map 2D grid map (0=free, 1=occupied)
     * @param grid_resolution Grid resolution in meters
     * @param origin_x Map origin x coordinate (m)
     * @param origin_y Map origin y coordinate (m)
     */
    GridCollisionDetector(const std::vector<std::vector<int>>& obstacle_map,
                         double grid_resolution,
                         double origin_x = 0.0,
                         double origin_y = 0.0);
    
    /**
     * @brief Check if a state is collision-free
     * @param state The state to check
     * @return true if collision-free, false otherwise
     */
    bool is_collision_free(const State& state) const override;
    
    /**
     * @brief Update obstacle map
     * @param obstacle_map New obstacle map
     * @param origin_x New map origin x coordinate (m)
     * @param origin_y New map origin y coordinate (m)
     */
    void update_obstacle_map(const std::vector<std::vector<int>>& obstacle_map,
                            double origin_x = 0.0,
                            double origin_y = 0.0);

private:
    std::vector<std::vector<int>> obstacle_map_;
    double grid_resolution_;
    double origin_x_;
    double origin_y_;
    int map_width_;
    int map_height_;
};

/**
 * @brief Simple geometric collision detector for basic shapes
 */
class GeometricCollisionDetector : public CollisionDetector {
public:
    struct Circle {
        double x, y, radius;
        Circle(double x, double y, double radius) : x(x), y(y), radius(radius) {}
    };
    
    struct Rectangle {
        double x_min, y_min, x_max, y_max;
        Rectangle(double x_min, double y_min, double x_max, double y_max) 
            : x_min(x_min), y_min(y_min), x_max(x_max), y_max(y_max) {}
    };
    
    /**
     * @brief Constructor
     * @param vehicle_radius Radius of the vehicle for collision checking
     */
    explicit GeometricCollisionDetector(double vehicle_radius = 1.0);
    
    /**
     * @brief Add circular obstacle
     * @param x Center x coordinate
     * @param y Center y coordinate  
     * @param radius Obstacle radius
     */
    void add_circle_obstacle(double x, double y, double radius);
    
    /**
     * @brief Add rectangular obstacle
     * @param x_min Minimum x coordinate
     * @param y_min Minimum y coordinate
     * @param x_max Maximum x coordinate
     * @param y_max Maximum y coordinate
     */
    void add_rectangle_obstacle(double x_min, double y_min, double x_max, double y_max);
    
    /**
     * @brief Clear all obstacles
     */
    void clear_obstacles();
    
    /**
     * @brief Check if a state is collision-free
     * @param state The state to check
     * @return true if collision-free, false otherwise
     */
    bool is_collision_free(const State& state) const override;

private:
    double vehicle_radius_;
    std::vector<Circle> circle_obstacles_;
    std::vector<Rectangle> rectangle_obstacles_;
};

} // namespace hybrid_astar
