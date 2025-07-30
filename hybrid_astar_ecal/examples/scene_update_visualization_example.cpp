/**
 * @file scene_update_visualization_example.cpp
 * @brief Example demonstrating Foxglove SceneUpdate visualization for Hybrid A*
 */

#include "visualization_publisher.hpp"
#include "hybrid_astar.hpp"
#include <iostream>
#include <vector>
#include <memory>
#include <thread>
#include <chrono>

using namespace hybrid_astar;

int main() {
    std::cout << "=== Foxglove SceneUpdate Visualization Example ===" << std::endl;
    
    // Create visualization publisher
    VisualizationPublisher visualizer("hybrid_astar_scene_demo");
    
    // Initialize publisher
    if (!visualizer.initialize()) {
        std::cerr << "Failed to initialize visualization publisher" << std::endl;
        return -1;
    }
    
    // Configure visualization settings
    VisualizationPublisher::VisualizationSettings settings;
    settings.path_line_thickness = 0.08;
    settings.path_alpha = 0.8;
    settings.max_exploration_nodes = 5000;
    settings.exploration_sphere_size = 0.04;
    settings.exploration_line_thickness = 0.02;
    settings.show_final_path_arrows = true;
    visualizer.update_settings(settings);
    
    // Create simple obstacle map
    std::vector<std::vector<int>> obstacle_map(50, std::vector<int>(50, 0));
    
    // Add rectangular obstacles
    for (int y = 20; y < 30; ++y) {
        for (int x = 15; x < 25; ++x) {
            obstacle_map[y][x] = 1;
        }
    }
    
    // Add another obstacle
    for (int y = 10; y < 15; ++y) {
        for (int x = 35; x < 45; ++x) {
            obstacle_map[y][x] = 1;
        }
    }
    
    // Define start and goal states
    State start(5.0, 5.0, M_PI/4, DirectionMode::FORWARD, 0.0);
    State goal(40.0, 40.0, M_PI/2, DirectionMode::FORWARD, 0.0);
    
    // Create mock planning result data
    std::vector<State> detailed_path;
    std::vector<std::shared_ptr<Node>> explored_nodes;
    std::vector<std::vector<State>> simulation_trajectories;
    
    // Generate sample path
    std::cout << "Generating sample path..." << std::endl;
    int path_steps = 100;
    for (int i = 0; i <= path_steps; ++i) {
        double t = static_cast<double>(i) / path_steps;
        
        // Simple interpolated path (with some curvature)
        double x = start.x + t * (goal.x - start.x) + 5.0 * std::sin(t * M_PI);
        double y = start.y + t * (goal.y - start.y) + 3.0 * std::cos(t * M_PI * 2);
        double yaw = start.yaw + t * (goal.yaw - start.yaw) + 0.5 * std::sin(t * M_PI * 3);
        double steer = 0.3 * std::sin(t * M_PI * 4); // Varying steering
        
        detailed_path.emplace_back(x, y, yaw, DirectionMode::FORWARD, steer);
    }
    
    // Generate mock explored nodes
    std::cout << "Generating exploration nodes..." << std::endl;
    for (int i = 0; i < 1000; ++i) {
        double x = start.x + (goal.x - start.x) * (rand() % 100) / 100.0;
        double y = start.y + (goal.y - start.y) * (rand() % 100) / 100.0;
        double yaw = (rand() % 628) / 100.0; // 0 to 2*pi
        
        auto node = std::make_shared<Node>();
        node->state = State(x, y, yaw, DirectionMode::FORWARD);
        node->g_cost = std::sqrt((x-start.x)*(x-start.x) + (y-start.y)*(y-start.y));
        node->h_cost = std::sqrt((x-goal.x)*(x-goal.x) + (y-goal.y)*(y-goal.y));
        
        explored_nodes.push_back(node);
        
        // Add some simulation trajectories
        if (i % 10 == 0) {
            std::vector<State> trajectory;
            for (int j = 0; j < 5; ++j) {
                double tx = x + j * 0.5 * std::cos(yaw);
                double ty = y + j * 0.5 * std::sin(yaw);
                trajectory.emplace_back(tx, ty, yaw, DirectionMode::FORWARD);
            }
            simulation_trajectories.push_back(trajectory);
        }
    }
    
    // Publish planning status
    std::cout << "Publishing planning status..." << std::endl;
    visualizer.publish_planning_status(1, "Planning in progress", 500, 1000);
    
    // Sleep a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    
    // Publish complete visualization
    std::cout << "Publishing complete visualization..." << std::endl;
    visualizer.visualize_path_planning(
        start, goal,
        std::nullopt, // No original path nodes in this example
        explored_nodes,
        detailed_path,
        simulation_trajectories,
        obstacle_map,
        0.0, 0.0, 1.0, // Map origin and resolution
        125.5 // Planning time in ms
    );
    
    std::cout << std::endl;
    std::cout << "✓ Visualization published successfully!" << std::endl;
    std::cout << "→ Connect Foxglove Studio to the eCAL network to view visualization" << std::endl;
    std::cout << "→ Add 3D panels and subscribe to the following topics:" << std::endl;
    std::cout << "  - /hybrid_astar/visualization/scene (obstacles)" << std::endl;
    std::cout << "  - /hybrid_astar/visualization/path (final path)" << std::endl;
    std::cout << "  - /hybrid_astar/visualization/exploration (search nodes)" << std::endl;
    std::cout << "  - /hybrid_astar/visualization/start_goal (start/goal)" << std::endl;
    std::cout << "  - /hybrid_astar/visualization/statistics (JSON data)" << std::endl;
    std::cout << std::endl;
    std::cout << "Press Enter to exit..." << std::endl;
    std::cin.get();
    
    // Shutdown
    visualizer.shutdown();
    
    std::cout << "Example completed." << std::endl;
    return 0;
}
