/**
 * @file demo.cpp
 * @brief Demo with visualization support when available
 */

#include "hybrid_astar.hpp"
#ifdef ECAL_PROTOBUF_AVAILABLE
#include "visualization_publisher.hpp"
#endif
#include <iostream>
#include <vector>
#include <chrono>
#include <algorithm>

using namespace hybrid_astar;

int main() {
    std::cout << "=== Hybrid A* Planning Demo ===" << std::endl;
    
    // Create planning configuration
    PlanningConfig config;
    config.max_steer = 0.6;
    config.simulation_time = 1.0;
    config.dt = 0.1;
    config.velocity = 2.0;
    config.wheelbase = 2.5;
    config.grid_resolution = 0.5;
    config.position_tolerance = 1.0;
    config.angle_tolerance = 0.3;
    config.debug_enabled = true;
    
    // Create planner
    HybridAStar planner(config);
    
    // Create simple obstacle map (20x20 grid)
    int map_width = 20;
    int map_height = 20;
    std::vector<std::vector<int>> obstacle_map(map_height, std::vector<int>(map_width, 0));
    
    // Add some obstacles (walls)
    for (int i = 5; i < 15; ++i) {
        obstacle_map[10][i] = 1; // Horizontal wall
    }
    for (int i = 6; i < 10; ++i) {
        obstacle_map[i][8] = 1; // Vertical wall
    }
    
    // Set obstacle map
    double map_origin_x = -5.0;
    double map_origin_y = -5.0;
    planner.set_obstacle_map(obstacle_map, map_origin_x, map_origin_y);
    
    // Define start and goal states
    State start_state;
    start_state.x = -3.0;
    start_state.y = -3.0;
    start_state.yaw = 0.0;
    start_state.steer = 0.0;
    start_state.direction = DirectionMode::FORWARD;
    
    State goal_state;
    goal_state.x = 3.0;
    goal_state.y = 3.0;
    goal_state.yaw = 0.0;
    goal_state.steer = 0.0;
    goal_state.direction = DirectionMode::FORWARD;
    
    std::cout << "Start: (" << start_state.x << ", " << start_state.y << ", " << start_state.yaw << ")" << std::endl;
    std::cout << "Goal: (" << goal_state.x << ", " << goal_state.y << ", " << goal_state.yaw << ")" << std::endl;
    
#ifdef ECAL_PROTOBUF_AVAILABLE
    // Initialize visualization publisher when available
    VisualizationPublisher viz_pub("hybrid_astar_demo");
    viz_pub.initialize();
    std::cout << "Visualization enabled (eCAL)" << std::endl;
#else
    std::cout << "Running without visualization (eCAL not available)" << std::endl;
#endif
    
    // Plan path
    std::cout << "\nPlanning path..." << std::endl;
    auto start_time = std::chrono::high_resolution_clock::now();
    
    auto path_nodes = planner.plan_path(start_state, goal_state, 1000);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    if (path_nodes.has_value()) {
        std::cout << "Path found!" << std::endl;
        std::cout << "Path length: " << path_nodes->size() << " nodes" << std::endl;
        
        // Extract detailed path
        auto detailed_path = planner.extract_detailed_path(*path_nodes);
        std::cout << "Detailed path: " << detailed_path.size() << " points" << std::endl;
        
        // Get statistics
        auto statistics = planner.get_statistics(path_nodes);
        std::cout << "Statistics:" << std::endl;
        for (const auto& [key, value] : statistics) {
            std::cout << "  " << key << ": " << value << std::endl;
        }
        
#ifdef ECAL_PROTOBUF_AVAILABLE
        // Publish visualization when available
        viz_pub.publish_planning_result(
            start_state, goal_state,
            path_nodes, planner.get_explored_nodes(),
            detailed_path, statistics,
            obstacle_map, map_origin_x, map_origin_y, config.grid_resolution,
            duration.count()
        );
#endif
        
        // Print first few path points
        std::cout << "\nFirst 5 path points:" << std::endl;
        for (size_t i = 0; i < std::min(size_t(5), detailed_path.size()); ++i) {
            const auto& point = detailed_path[i];
            std::cout << "  " << i << ": (" << point.x << ", " << point.y << ", " << point.yaw << ")" << std::endl;
        }
        
    } else {
        std::cout << "No path found!" << std::endl;
#ifdef ECAL_PROTOBUF_AVAILABLE
        viz_pub.publish_planning_status(3, "Planning failed", 0, 1000);
#endif
    }
    
    std::cout << "Planning time: " << duration.count() << " ms" << std::endl;
    std::cout << "Explored nodes: " << planner.get_explored_nodes().size() << std::endl;
    
#ifdef ECAL_PROTOBUF_AVAILABLE
    viz_pub.shutdown();
#endif
    
    std::cout << "\nDemo completed." << std::endl;
    return 0;
}
