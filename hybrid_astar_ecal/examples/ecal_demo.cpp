/**
 * @file ecal_demo.cpp
 * @brief Demo with full eCAL integration
 */

#include "hybrid_astar.hpp"
#include "obstacle_map.hpp"
#include "visualization_publisher.hpp"
#include <iostream>
#include <vector>
#include <chrono>
#include <thread>

using namespace hybrid_astar;

int main() {
    std::cout << "=== Hybrid A* Planning with eCAL Demo ===" << std::endl;
    
    // Create planning configuration
    PlanningConfig config;
    config.max_steer = 0.6;
    config.grid_resolution = 0.5;
    config.angle_tolerance = 0.3;
    config.position_tolerance = 1.0;
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
    
    // Initialize visualization publisher
    VisualizationPublisher viz_pub("hybrid_astar_ecal_demo");
    bool init_success = viz_pub.initialize();
    
    if (!init_success) {
        std::cerr << "Failed to initialize eCAL publisher" << std::endl;
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
    // Publish initial status
    viz_pub.publish_planning_status(1, "Starting planning", 0, 100000); // Status 1 = PLANNING
    
    // Plan path with status updates
    std::cout << "\nPlanning path..." << std::endl;
    auto start_time = std::chrono::high_resolution_clock::now();
    
    auto path_nodes = planner.plan_path(start_state, goal_state, 100000);
    
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
        
        // Publish complete planning result
        viz_pub.visualize_path_planning(
            start_state, goal_state,
            path_nodes, planner.get_explored_nodes(),
            detailed_path, {}, // empty simulation trajectories
            obstacle_map, map_origin_x, map_origin_y, config.grid_resolution,
            duration.count()
        );
        
        // Publish success status
        viz_pub.publish_planning_status(2, "Planning completed successfully", 1000, 1000); // Status 2 = SUCCESS
        
        std::cout << "\nFirst 5 path points:" << std::endl;
        for (size_t i = 0; i < std::min(size_t(5), detailed_path.size()); ++i) {
            const auto& point = detailed_path[i];
            std::cout << "  " << i << ": (" << point.x << ", " << point.y << ", " << point.yaw << ")" << std::endl;
        }
        
    } else {
        std::cout << "No path found!" << std::endl;
        viz_pub.publish_planning_status(3, "Planning failed - no path found", 1000, 1000); // Status 3 = FAILED
    }
    
    std::cout << "Planning time: " << duration.count() << " ms" << std::endl;
    std::cout << "Explored nodes: " << planner.get_explored_nodes().size() << std::endl;
    
    // Keep publishing for a while to allow Foxglove to receive the data
    std::cout << "\nKeeping eCAL connection alive for 5 seconds..." << std::endl;
    for (int i = 0; i < 5; ++i) {
        std::this_thread::sleep_for(std::chrono::seconds(100));
        std::cout << "." << std::flush;
    }
    std::cout << std::endl;
    
    viz_pub.shutdown();
    
    std::cout << "\nDemo completed. Check Foxglove Studio for visualization!" << std::endl;
    return 0;
}
