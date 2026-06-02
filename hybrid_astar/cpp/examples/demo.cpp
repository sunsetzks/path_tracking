/**
 * @file demo.cpp
 * @brief Basic Hybrid A* demo without visualization
 */

#include "hybrid_astar.hpp"
#include "collision_detector.hpp"
#include <iostream>
#include <vector>
#include <chrono>
#include <memory>

using namespace hybrid_astar;

int main() {
    std::cout << "=== Hybrid A* Planning Demo ===" << std::endl;
    
    // Create planning configuration
    PlanningConfig config;
    config.grid_resolution = 1.0;
    config.angle_resolution = 0.1;
    config.max_steer = 0.6;
    config.wheelbase = 2.5;
    config.velocity = 2.0;
    config.position_tolerance = 1.0;
    config.angle_tolerance = 0.2;
    config.max_iterations = 5000;
    config.debug_enabled = true;
    
    // Create planner
    HybridAStar planner(config);
    
    // Create a simple obstacle map (empty for basic demo)
    int map_width = 50;
    int map_height = 50;
    std::vector<std::vector<int>> obstacle_map(map_height, std::vector<int>(map_width, 0));
    
    // Add some obstacles
    for (int i = 20; i < 30; ++i) {
        for (int j = 15; j < 35; ++j) {
            obstacle_map[i][j] = 1;
        }
    }
    
    // Create collision detector
    auto collision_detector = std::make_shared<GridCollisionDetector>(
        obstacle_map, config.grid_resolution, 0.0, 0.0);
    planner.set_collision_detector(collision_detector);
    
    // Define start and goal states
    State start_state(5.0, 5.0, 0.0, DirectionMode::FORWARD);
    State goal_state(40.0, 40.0, 0.0, DirectionMode::FORWARD);
    
    std::cout << "Start: (" << start_state.x << ", " << start_state.y << ", " << start_state.yaw << ")" << std::endl;
    std::cout << "Goal: (" << goal_state.x << ", " << goal_state.y << ", " << goal_state.yaw << ")" << std::endl;
    
    // Plan path
    std::cout << "Planning path..." << std::endl;
    auto start_time = std::chrono::high_resolution_clock::now();
    
    auto result = planner.plan_path(start_state, goal_state);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    if (result.has_value()) {
        auto path_nodes = result.value();
        auto detailed_path = planner.extract_detailed_path(path_nodes);
        
        std::cout << "Path found successfully!" << std::endl;
        std::cout << "Planning time: " << duration.count() << " ms" << std::endl;
        std::cout << "Path nodes: " << path_nodes.size() << std::endl;
        std::cout << "Detailed path length: " << detailed_path.size() << " states" << std::endl;
        
        // Print first few waypoints
        std::cout << "First few waypoints:" << std::endl;
        for (size_t i = 0; i < std::min(size_t(5), detailed_path.size()); ++i) {
            const auto& state = detailed_path[i];
            std::cout << "  [" << i << "] (" << state.x << ", " << state.y << ", " << state.yaw << ")" << std::endl;
        }
    } else {
        std::cout << "Path planning failed!" << std::endl;
        std::cout << "Planning time: " << duration.count() << " ms" << std::endl;
    }
    
    return 0;
}
