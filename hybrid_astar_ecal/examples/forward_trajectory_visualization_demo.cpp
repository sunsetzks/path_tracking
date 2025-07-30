/**
 * @file forward_trajectory_visualization_demo.cpp
 * @brief Demo showing forward simulation trajectory visualization for each node
 */

#include "hybrid_astar.hpp"
#include "visualization_publisher.hpp"
#include "scenario_generator.hpp"
#include <iostream>
#include <chrono>

int main() {
    std::cout << "=== Forward Trajectory Visualization Demo ===" << std::endl;
    
    // Initialize the visualization publisher
    hybrid_astar::VisualizationPublisher visualizer("hybrid_astar_forward_trajectory_demo");
    if (!visualizer.initialize()) {
        std::cerr << "Failed to initialize visualization publisher" << std::endl;
        return -1;
    }
    
    // Configure visualization settings to show forward simulation trajectories
    hybrid_astar::VisualizationPublisher::VisualizationSettings settings;
    settings.show_node_forward_trajectories = true;  // Enable forward trajectory visualization
    settings.max_exploration_nodes = 500;            // Limit nodes for better performance
    settings.exploration_line_thickness = 0.015;     // Make trajectories more visible
    settings.exploration_sphere_size = 0.05;         // Make nodes more visible
    settings.show_final_path_arrows = true;          // Also show path direction
    
    visualizer.update_settings(settings);
    
    std::cout << "✓ Visualization settings configured:" << std::endl;
    std::cout << "  - Forward trajectory visualization: ENABLED" << std::endl;
    std::cout << "  - Max exploration nodes: " << settings.max_exploration_nodes << std::endl;
    std::cout << "  - Trajectory line thickness: " << settings.exploration_line_thickness << std::endl;
    
    // Create a planning scenario
    auto scenario_generator = hybrid_astar::ScenarioGenerator();
    auto scenario = scenario_generator.create_L_shaped_scenario();
    
    std::cout << "\n--- Planning scenario ---" << std::endl;
    std::cout << "Start: (" << scenario.start.x << ", " << scenario.start.y 
              << ", " << scenario.start.yaw << ")" << std::endl;
    std::cout << "Goal: (" << scenario.goal.x << ", " << scenario.goal.y 
              << ", " << scenario.goal.yaw << ")" << std::endl;
    
    // Initialize planner
    hybrid_astar::HybridAStarConfig config;
    config.max_iterations = 2000;
    config.goal_tolerance_xy = 1.0;
    config.goal_tolerance_yaw = 0.3;
    
    hybrid_astar::HybridAStar planner(config);
    
    // Set up the planning problem
    planner.setup_planning_problem(
        scenario.obstacle_map,
        scenario.map_origin_x, scenario.map_origin_y,
        scenario.grid_resolution
    );
    
    // Plan path
    std::cout << "\n--- Planning path ---" << std::endl;
    auto start_time = std::chrono::high_resolution_clock::now();
    
    auto result = planner.plan_path(scenario.start, scenario.goal);
    
    auto end_time = std::chrono::high_resolution_clock::now();
    auto planning_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - start_time).count();
    
    if (result.path_found) {
        std::cout << "✓ Path found in " << planning_time_ms << " ms" << std::endl;
        std::cout << "  - Path length: " << result.detailed_path.size() << " points" << std::endl;
        std::cout << "  - Explored nodes: " << result.explored_nodes.size() << " nodes" << std::endl;
        
        // Count nodes with forward trajectories
        int nodes_with_trajectories = 0;
        int total_trajectory_points = 0;
        for (const auto& node : result.explored_nodes) {
            if (!node->forward_simulation_trajectory.empty()) {
                nodes_with_trajectories++;
                total_trajectory_points += node->forward_simulation_trajectory.size();
            }
        }
        
        std::cout << "  - Nodes with forward trajectories: " << nodes_with_trajectories << std::endl;
        std::cout << "  - Total trajectory points: " << total_trajectory_points << std::endl;
        
        // Visualize the result
        std::cout << "\n--- Publishing visualization ---" << std::endl;
        visualizer.visualize_path_planning(
            scenario.start,
            scenario.goal,
            result.path_nodes,
            result.explored_nodes,
            result.detailed_path,
            {},  // Empty simulation trajectories (we use node's forward_simulation_trajectory instead)
            scenario.obstacle_map,
            scenario.map_origin_x, scenario.map_origin_y, scenario.grid_resolution,
            static_cast<double>(planning_time_ms)
        );
        
        std::cout << "✓ Visualization published to eCAL channels" << std::endl;
        std::cout << "\nTo view the visualization:" << std::endl;
        std::cout << "1. Start Foxglove Studio" << std::endl;
        std::cout << "2. Connect to eCAL data source" << std::endl;
        std::cout << "3. Add 3D panels and subscribe to these topics:" << std::endl;
        std::cout << "   - /hybrid_astar/visualization/scene (obstacles)" << std::endl;
        std::cout << "   - /hybrid_astar/visualization/path (final path)" << std::endl;
        std::cout << "   - /hybrid_astar/visualization/exploration (nodes + forward trajectories)" << std::endl;
        std::cout << "   - /hybrid_astar/visualization/start_goal (start/goal positions)" << std::endl;
        std::cout << "\nNote: Forward simulation trajectories will appear as cyan-green lines" << std::endl;
        std::cout << "      emanating from each exploration node, showing the predicted vehicle motion." << std::endl;
        
    } else {
        std::cout << "✗ No path found" << std::endl;
        std::cout << "  - Explored nodes: " << result.explored_nodes.size() << std::endl;
        
        // Still visualize the exploration to show forward trajectories
        visualizer.visualize_path_planning(
            scenario.start,
            scenario.goal,
            std::nullopt,  // No path found
            result.explored_nodes,
            {},  // Empty path
            {},  // Empty simulation trajectories
            scenario.obstacle_map,
            scenario.map_origin_x, scenario.map_origin_y, scenario.grid_resolution,
            static_cast<double>(planning_time_ms)
        );
    }
    
    // Keep the program running for a while to allow visualization
    std::cout << "\nKeeping visualization active for 30 seconds..." << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(30));
    
    // Publish final status
    visualizer.publish_planning_status(
        result.path_found ? 1 : 0,
        result.path_found ? "Path planning completed successfully" : "Path planning failed",
        result.explored_nodes.size(),
        config.max_iterations
    );
    
    std::cout << "Demo completed." << std::endl;
    return 0;
}
