/**
 * @file collision_detector_example.cpp
 * @brief Example demonstrating the new collision detector interface
 */

#include "hybrid_astar.hpp"
#include "collision_detector.hpp"
#include <iostream>
#include <vector>
#include <memory>

int main() {
    using namespace hybrid_astar;
    
    // Create planning configuration
    PlanningConfig config;
    config.wheelbase = 2.5;
    config.max_steer = 0.6;
    config.velocity = 2.0;
    config.simulation_time = 1.0;
    config.dt = 0.1;
    config.grid_resolution = 0.5;
    config.angle_resolution = 0.1;
    config.steer_resolution = 0.1;
    config.position_tolerance = 1.0;
    config.angle_tolerance = 0.2;
    config.w_steer = 1.0;
    config.w_turn = 1.0;
    config.w_cusp = 10.0;
    config.max_iterations = 5000;
    config.debug_enabled = true;
    
    // Create hybrid A* planner
    HybridAStar planner(config);
    
    std::cout << "=== Grid-based Collision Detector Example ===" << std::endl;
    
    // Example 1: Grid-based collision detector
    {
        // Create a simple obstacle map (10x10 grid)
        std::vector<std::vector<int>> obstacle_map(10, std::vector<int>(10, 0));
        
        // Add some obstacles
        for (int i = 3; i <= 6; ++i) {
            for (int j = 3; j <= 6; ++j) {
                obstacle_map[i][j] = 1;  // Obstacle
            }
        }
        
        // Create grid collision detector
        auto grid_detector = std::make_shared<GridCollisionDetector>(
            obstacle_map, config.grid_resolution, 0.0, 0.0);
        
        // Set collision detector
        planner.set_collision_detector(grid_detector);
        
        // Test states
        State free_state(1.0, 1.0, 0.0, DirectionMode::FORWARD, 0.0);
        State obstacle_state(2.0, 2.0, 0.0, DirectionMode::FORWARD, 0.0);
        
        std::cout << "Free state (1.0, 1.0) collision-free: " 
                  << (grid_detector->is_collision_free(free_state) ? "Yes" : "No") << std::endl;
        std::cout << "Obstacle state (2.0, 2.0) collision-free: " 
                  << (grid_detector->is_collision_free(obstacle_state) ? "Yes" : "No") << std::endl;
        
        // Plan a path
        State start(0.5, 0.5, 0.0, DirectionMode::FORWARD, 0.0);
        State goal(4.5, 0.5, 0.0, DirectionMode::FORWARD, 0.0);
        
        auto path_result = planner.plan_path(start, goal, 1000);
        if (path_result.has_value()) {
            std::cout << "Grid-based planning: Found path with " 
                      << path_result.value().size() << " waypoints" << std::endl;
        } else {
            std::cout << "Grid-based planning: No path found" << std::endl;
        }
    }
    
    std::cout << "\n=== Geometric Collision Detector Example ===" << std::endl;
    
    // Example 2: Geometric collision detector
    {
        // Create geometric collision detector
        auto geometric_detector = std::make_shared<GeometricCollisionDetector>(0.5);  // 0.5m vehicle radius
        
        // Add some circular obstacles
        geometric_detector->add_circle_obstacle(3.0, 3.0, 1.0);  // Circle at (3,3) with radius 1m
        geometric_detector->add_circle_obstacle(6.0, 6.0, 1.5);  // Circle at (6,6) with radius 1.5m
        
        // Add rectangular obstacles
        geometric_detector->add_rectangle_obstacle(1.0, 5.0, 2.0, 7.0);  // Rectangle from (1,5) to (2,7)
        
        // Set collision detector
        planner.set_collision_detector(geometric_detector);
        
        // Test states
        State free_state(0.0, 0.0, 0.0, DirectionMode::FORWARD, 0.0);
        State near_circle(3.0, 2.0, 0.0, DirectionMode::FORWARD, 0.0);  // Near the circle
        State in_rectangle(1.5, 6.0, 0.0, DirectionMode::FORWARD, 0.0);  // Inside rectangle
        
        std::cout << "Free state (0.0, 0.0) collision-free: " 
                  << (geometric_detector->is_collision_free(free_state) ? "Yes" : "No") << std::endl;
        std::cout << "Near circle (3.0, 2.0) collision-free: " 
                  << (geometric_detector->is_collision_free(near_circle) ? "Yes" : "No") << std::endl;
        std::cout << "In rectangle (1.5, 6.0) collision-free: " 
                  << (geometric_detector->is_collision_free(in_rectangle) ? "Yes" : "No") << std::endl;
        
        // Plan a path
        State start(0.0, 0.0, 0.0, DirectionMode::FORWARD, 0.0);
        State goal(8.0, 8.0, M_PI/4, DirectionMode::FORWARD, 0.0);
        
        auto path_result = planner.plan_path(start, goal, 1000);
        if (path_result.has_value()) {
            std::cout << "Geometric planning: Found path with " 
                      << path_result.value().size() << " waypoints" << std::endl;
            
            // Get detailed path
            auto detailed_path = planner.extract_detailed_path(path_result.value());
            std::cout << "Detailed path has " << detailed_path.size() << " states" << std::endl;
        } else {
            std::cout << "Geometric planning: No path found" << std::endl;
        }
        
        // Show statistics
        auto stats = planner.get_statistics(path_result);
        std::cout << "Planning statistics:" << std::endl;
        for (const auto& [key, value] : stats) {
            std::cout << "  " << key << ": " << value << std::endl;
        }
    }
    
    return 0;
}
