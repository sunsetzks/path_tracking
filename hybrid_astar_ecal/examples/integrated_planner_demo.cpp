/**
 * @file integrated_planner_demo.cpp
 * @brief Demonstrates integration of SceneUpdate visualization with Hybrid A* planner
 */

#include "visualization_publisher.hpp"
#include <iostream>
#include <vector>
#include <memory>
#include <thread>
#include <chrono>

using namespace hybrid_astar;

/**
 * @brief Mock Hybrid A* planner class for demonstration
 */
class MockHybridAStarPlanner {
public:
    struct PlanningResult {
        bool success = false;
        std::vector<State> path;
        std::vector<std::shared_ptr<Node>> explored_nodes;
        std::vector<std::vector<State>> simulation_trajectories;
        double planning_time_ms = 0.0;
    };
    
    MockHybridAStarPlanner(const std::vector<std::vector<int>>& obstacle_map,
                          double map_origin_x, double map_origin_y, double grid_resolution)
        : obstacle_map_(obstacle_map), map_origin_x_(map_origin_x), 
          map_origin_y_(map_origin_y), grid_resolution_(grid_resolution) {}
    
    PlanningResult plan_path(const State& start, const State& goal) {
        std::cout << "Planning path from (" << start.x << ", " << start.y 
                  << ") to (" << goal.x << ", " << goal.y << ")" << std::endl;
        
        auto start_time = std::chrono::high_resolution_clock::now();
        
        PlanningResult result;
        
        // Simulate planning process
        generate_exploration_data(start, goal, result);
        
        // Generate path if successful
        if (generate_path(start, goal, result)) {
            result.success = true;
            std::cout << "✓ Path found with " << result.path.size() << " waypoints" << std::endl;
        } else {
            result.success = false;
            std::cout << "✗ No path found" << std::endl;
        }
        
        auto end_time = std::chrono::high_resolution_clock::now();
        result.planning_time_ms = std::chrono::duration<double, std::milli>(
            end_time - start_time).count();
        
        std::cout << "Planning completed in " << result.planning_time_ms << " ms" << std::endl;
        
        return result;
    }
    
    const std::vector<std::vector<int>>& get_obstacle_map() const { return obstacle_map_; }
    double get_map_origin_x() const { return map_origin_x_; }
    double get_map_origin_y() const { return map_origin_y_; }
    double get_grid_resolution() const { return grid_resolution_; }
    
private:
    std::vector<std::vector<int>> obstacle_map_;
    double map_origin_x_, map_origin_y_, grid_resolution_;
    
    void generate_exploration_data(const State& start, const State& goal, PlanningResult& result) {
        // Generate mock explored nodes
        int num_nodes = 800;
        for (int i = 0; i < num_nodes; ++i) {
            // Create nodes in a search pattern from start toward goal
            double progress = static_cast<double>(i) / num_nodes;
            double spread = 15.0 * (1.0 - progress); // Narrowing search
            
            double base_x = start.x + progress * (goal.x - start.x);
            double base_y = start.y + progress * (goal.y - start.y);
            
            // Add random spread
            double x = base_x + spread * (2.0 * rand()/RAND_MAX - 1.0);
            double y = base_y + spread * (2.0 * rand()/RAND_MAX - 1.0);
            double yaw = 2.0 * M_PI * rand() / RAND_MAX;
            
            auto node = std::make_shared<Node>();
            node->state = State(x, y, yaw, DirectionMode::FORWARD);
            node->g_cost = std::sqrt((x-start.x)*(x-start.x) + (y-start.y)*(y-start.y));
            node->h_cost = std::sqrt((x-goal.x)*(x-goal.x) + (y-goal.y)*(y-goal.y)) * 1.2; // Admissible heuristic
            
            result.explored_nodes.push_back(node);
            
            // Add simulation trajectories for some nodes
            if (i % 15 == 0) {
                std::vector<State> trajectory;
                for (int j = 0; j < 8; ++j) {
                    double tx = x + j * 0.7 * std::cos(yaw + 0.1 * j);
                    double ty = y + j * 0.7 * std::sin(yaw + 0.1 * j);
                    double tyaw = yaw + 0.1 * j;
                    trajectory.emplace_back(tx, ty, tyaw, DirectionMode::FORWARD);
                }
                result.simulation_trajectories.push_back(trajectory);
            }
        }
        
        std::cout << "Generated " << result.explored_nodes.size() 
                  << " exploration nodes and " << result.simulation_trajectories.size() 
                  << " simulation trajectories" << std::endl;
    }
    
    bool generate_path(const State& start, const State& goal, PlanningResult& result) {
        // Simple path generation (avoiding obstacles would be more complex)
        int steps = 80;
        for (int i = 0; i <= steps; ++i) {
            double t = static_cast<double>(i) / steps;
            
            // Create smooth curved path
            double x = start.x + t * (goal.x - start.x) + 8.0 * std::sin(t * M_PI) * std::exp(-2*t);
            double y = start.y + t * (goal.y - start.y) + 5.0 * std::cos(t * M_PI * 1.5) * std::exp(-t);
            double yaw = start.yaw + t * (goal.yaw - start.yaw) + 0.3 * std::sin(t * M_PI * 4);
            
            // Add realistic steering angles
            double steer = 0.0;
            if (i > 0) {
                double dyaw = yaw - result.path.back().yaw;
                steer = std::max(-0.5, std::min(0.5, dyaw * 3.0)); // Clamp steering
            }
            
            result.path.emplace_back(x, y, yaw, DirectionMode::FORWARD, steer);
        }
        
        return true; // Always successful in this mock
    }
};

/**
 * @brief Integrated planning and visualization demo
 */
class PlannerVisualizationDemo {
public:
    PlannerVisualizationDemo() : visualizer_("hybrid_astar_integrated_demo") {
        setup_scenario();
    }
    
    bool initialize() {
        std::cout << "Initializing planner visualization demo..." << std::endl;
        
        if (!visualizer_.initialize()) {
            std::cerr << "Failed to initialize visualizer" << std::endl;
            return false;
        }
        
        // Configure visualization settings
        VisualizationPublisher::VisualizationSettings settings;
        settings.path_line_thickness = 0.1;
        settings.path_alpha = 0.9;
        settings.max_exploration_nodes = 2000;
        settings.exploration_sphere_size = 0.05;
        settings.exploration_line_thickness = 0.015;
        settings.show_final_path_arrows = true;
        visualizer_.update_settings(settings);
        
        std::cout << "✓ Demo initialized successfully" << std::endl;
        return true;
    }
    
    void run_planning_demo() {
        std::cout << std::endl << "=== Running Integrated Planning Demo ===" << std::endl;
        
        // Define planning scenarios
        std::vector<std::pair<State, State>> scenarios = {
            {State(2.0, 2.0, 0.0, DirectionMode::FORWARD), 
             State(45.0, 45.0, M_PI/2, DirectionMode::FORWARD)},
            {State(45.0, 2.0, M_PI, DirectionMode::FORWARD), 
             State(2.0, 45.0, M_PI/2, DirectionMode::FORWARD)},
            {State(25.0, 10.0, M_PI/4, DirectionMode::FORWARD), 
             State(35.0, 40.0, -M_PI/4, DirectionMode::FORWARD)}
        };
        
        for (size_t i = 0; i < scenarios.size(); ++i) {
            std::cout << std::endl << "--- Scenario " << (i+1) << " ---" << std::endl;
            
            const auto& start = scenarios[i].first;
            const auto& goal = scenarios[i].second;
            
            // Publish planning status
            visualizer_.publish_planning_status(
                0, "Starting planning for scenario " + std::to_string(i+1), 0, 100);
            
            // Run planning
            auto result = planner_->plan_path(start, goal);
            
            // Visualize results
            if (result.success) {
                visualize_planning_result(start, goal, result);
                std::cout << "✓ Visualization published for scenario " << (i+1) << std::endl;
            } else {
                std::cout << "✗ Planning failed for scenario " << (i+1) << std::endl;
            }
            
            // Wait between scenarios
            if (i < scenarios.size() - 1) {
                std::cout << "Waiting 3 seconds before next scenario..." << std::endl;
                std::this_thread::sleep_for(std::chrono::seconds(3));
            }
        }
    }
    
    void shutdown() {
        visualizer_.shutdown();
        std::cout << "Demo shut down successfully" << std::endl;
    }
    
private:
    VisualizationPublisher visualizer_;
    std::unique_ptr<MockHybridAStarPlanner> planner_;
    
    void setup_scenario() {
        // Create test environment with obstacles
        std::vector<std::vector<int>> obstacle_map(50, std::vector<int>(50, 0));
        
        // Large rectangular obstacle
        for (int y = 18; y < 32; ++y) {
            for (int x = 20; x < 28; ++x) {
                obstacle_map[y][x] = 1;
            }
        }
        
        // L-shaped obstacle
        for (int y = 5; y < 15; ++y) {
            for (int x = 35; x < 45; ++x) {
                obstacle_map[y][x] = 1;
            }
        }
        for (int y = 5; y < 10; ++y) {
            for (int x = 30; x < 35; ++x) {
                obstacle_map[y][x] = 1;
            }
        }
        
        // Circular-ish obstacle
        int cx = 15, cy = 40;
        for (int y = cy - 4; y <= cy + 4; ++y) {
            for (int x = cx - 4; x <= cx + 4; ++x) {
                if ((x-cx)*(x-cx) + (y-cy)*(y-cy) <= 16) {
                    if (x >= 0 && x < 50 && y >= 0 && y < 50) {
                        obstacle_map[y][x] = 1;
                    }
                }
            }
        }
        
        planner_ = std::make_unique<MockHybridAStarPlanner>(
            obstacle_map, 0.0, 0.0, 1.0);
        
        std::cout << "Created test environment with obstacles" << std::endl;
    }
    
    void visualize_planning_result(const State& start, const State& goal,
                                 const MockHybridAStarPlanner::PlanningResult& result) {
        
        visualizer_.visualize_path_planning(
            start, goal,
            std::nullopt, // No original path nodes in this mock
            result.explored_nodes,
            result.path,
            result.simulation_trajectories,
            planner_->get_obstacle_map(),
            planner_->get_map_origin_x(),
            planner_->get_map_origin_y(),
            planner_->get_grid_resolution(),
            result.planning_time_ms
        );
    }
};

int main() {
    std::cout << "=== Hybrid A* SceneUpdate Visualization Integration Demo ===" << std::endl;
    
    PlannerVisualizationDemo demo;
    
    if (!demo.initialize()) {
        return -1;
    }
    
    demo.run_planning_demo();
    
    std::cout << std::endl;
    std::cout << "✓ Demo completed successfully!" << std::endl;
    std::cout << "→ Check Foxglove Studio for visualization results" << std::endl;
    std::cout << "→ The demo published data to the following eCAL topics:" << std::endl;
    std::cout << "  - /hybrid_astar/visualization/scene" << std::endl;
    std::cout << "  - /hybrid_astar/visualization/path" << std::endl;
    std::cout << "  - /hybrid_astar/visualization/exploration" << std::endl;
    std::cout << "  - /hybrid_astar/visualization/start_goal" << std::endl;
    std::cout << "  - /hybrid_astar/visualization/statistics" << std::endl;
    std::cout << std::endl;
    std::cout << "Press Enter to exit..." << std::endl;
    std::cin.get();
    
    demo.shutdown();
    return 0;
}
