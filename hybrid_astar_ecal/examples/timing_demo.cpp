/**
 * @file timing_demo.cpp
 * @brief Demo showing timing performance measurements
 */

#include "hybrid_astar.hpp"
#include <iostream>
#include <vector>
#include <iomanip>

using namespace hybrid_astar;

int main() {
    std::cout << "=== Hybrid A* Timing Performance Demo ===" << std::endl;
    
    // Create planning configuration for easy path
    PlanningConfig config;
    config.max_steer = 0.6;
    config.simulation_time = 1.0;
    config.dt = 0.1;
    config.velocity = 2.0;
    config.wheelbase = 2.5;
    config.grid_resolution = 1.0;  // Larger resolution for faster planning
    config.position_tolerance = 2.0;  // More relaxed tolerance
    config.angle_tolerance = 0.5;
    config.max_iterations = 500;  // Limit iterations for timing test
    config.debug_enabled = true;
    
    // Create planner
    HybridAStar planner(config);
    planner.set_debug_enabled(true);
    
    // Create empty obstacle map for easy planning
    int map_width = 20;
    int map_height = 20;
    std::vector<std::vector<int>> obstacle_map(map_height, std::vector<int>(map_width, 0));
    
    // Set obstacle map
    double map_origin_x = -10.0;
    double map_origin_y = -10.0;
    planner.set_obstacle_map(obstacle_map, map_origin_x, map_origin_y);
    
    // Test different planning scenarios
    std::vector<std::pair<State, State>> test_cases;
    
    // Case 1: Short distance
    State start1, goal1;
    start1.x = -2.0; start1.y = -2.0; start1.yaw = 0.0; start1.steer = 0.0; start1.direction = DirectionMode::FORWARD;
    goal1.x = 2.0; goal1.y = 2.0; goal1.yaw = 0.0; goal1.steer = 0.0; goal1.direction = DirectionMode::FORWARD;
    test_cases.push_back({start1, goal1});
    
    // Case 2: Medium distance
    State start2, goal2;
    start2.x = -5.0; start2.y = -5.0; start2.yaw = 0.0; start2.steer = 0.0; start2.direction = DirectionMode::FORWARD;
    goal2.x = 5.0; goal2.y = 5.0; goal2.yaw = 0.0; goal2.steer = 0.0; goal2.direction = DirectionMode::FORWARD;
    test_cases.push_back({start2, goal2});
    
    // Case 3: Large angle difference
    State start3, goal3;
    start3.x = 0.0; start3.y = 0.0; start3.yaw = 0.0; start3.steer = 0.0; start3.direction = DirectionMode::FORWARD;
    goal3.x = 3.0; goal3.y = 0.0; goal3.yaw = 3.14; goal3.steer = 0.0; goal3.direction = DirectionMode::FORWARD;
    test_cases.push_back({start3, goal3});
    
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "\n=======================================" << std::endl;
    std::cout << "Performance Timing Results:" << std::endl;
    std::cout << "=======================================" << std::endl;
    
    for (size_t i = 0; i < test_cases.size(); ++i) {
        const auto& start = test_cases[i].first;
        const auto& goal = test_cases[i].second;
        
        std::cout << "\nTest Case " << (i + 1) << ":" << std::endl;
        std::cout << "  Start: (" << start.x << ", " << start.y << ", " << start.yaw << ")" << std::endl;
        std::cout << "  Goal:  (" << goal.x << ", " << goal.y << ", " << goal.yaw << ")" << std::endl;
        
        // Plan path
        auto path_nodes = planner.plan_path(start, goal);
        
        // Get timing information
        double planning_time = planner.get_last_planning_time_ms();
        
        // Get detailed statistics
        auto stats = planner.get_statistics(path_nodes);
        
        std::cout << "  Results:" << std::endl;
        std::cout << "    Planning Time: " << planning_time << " ms" << std::endl;
        std::cout << "    Path Found: " << (stats["path_found"] > 0 ? "Yes" : "No") << std::endl;
        std::cout << "    Nodes Explored: " << static_cast<int>(stats["nodes_explored"]) << std::endl;
        
        if (stats["path_found"] > 0) {
            std::cout << "    Path Length: " << static_cast<int>(stats["path_length_waypoints"]) << " waypoints" << std::endl;
            std::cout << "    Total Distance: " << stats["total_distance"] << " m" << std::endl;
            std::cout << "    Max Steering: " << stats["max_steering_angle"] << " rad" << std::endl;
            std::cout << "    Direction Changes: " << static_cast<int>(stats["direction_changes"]) << std::endl;
        }
    }
    
    std::cout << "\n=======================================" << std::endl;
    std::cout << "Timing Analysis Complete!" << std::endl;
    std::cout << "=======================================" << std::endl;
    
    return 0;
}
