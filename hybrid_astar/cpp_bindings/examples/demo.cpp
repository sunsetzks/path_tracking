/**
 * @file demo.cpp
 * @brief Demo application for Hybrid A* algorithm
 * 
 * @author Converted from Python implementation
 * @date 2025-07-29
 */

#include "hybrid_astar.hpp"
#include <iostream>
#include <chrono>
#include <iomanip>

using namespace hybrid_astar;

void print_state(const State& state, const std::string& name) {
    std::cout << name << ": (" 
              << std::fixed << std::setprecision(2)
              << state.x << ", " << state.y << ", " 
              << state.yaw << " rad, dir=" << static_cast<int>(state.direction)
              << ", steer=" << state.steer << ")" << std::endl;
}

void print_statistics(const std::unordered_map<std::string, double>& stats) {
    std::cout << "\nPath Statistics:" << std::endl;
    std::cout << "=================" << std::endl;
    
    for (const auto& [key, value] : stats) {
        std::cout << std::setw(25) << std::left << key << ": ";
        if (key.find("found") != std::string::npos || key.find("changes") != std::string::npos ||
            key.find("waypoints") != std::string::npos || key.find("explored") != std::string::npos ||
            key.find("simulated") != std::string::npos) {
            std::cout << static_cast<int>(value);
        } else {
            std::cout << std::fixed << std::setprecision(3) << value;
        }
        std::cout << std::endl;
    }
}

void demo_simple_path() {
    std::cout << "=== Simple Path Planning Demo ===" << std::endl;
    
    // Create vehicle model
    VehicleModel vehicle(2.5, M_PI/4);
    std::cout << "Vehicle: wheelbase=" << vehicle.wheelbase() 
              << "m, max_steer=" << vehicle.max_steer() << " rad" << std::endl;
    
    // Create planner
    HybridAStar planner(vehicle, 1.0, M_PI/8, M_PI/16, 2.0, 1.0, 0.2);
    
    // Define start and goal
    State start(0.0, 0.0, 0.0, DirectionMode::NONE, 0.0);
    State goal(10.0, 5.0, M_PI/2, DirectionMode::FORWARD, 0.0);
    
    print_state(start, "Start");
    print_state(goal, "Goal");
    
    // Plan path
    std::cout << "\nPlanning path..." << std::endl;
    auto start_time = std::chrono::high_resolution_clock::now();
    auto path = planner.plan_path(start, goal, 5000);
    auto end_time = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    if (path.has_value()) {
        std::cout << "✓ Path found in " << duration.count() << "ms" << std::endl;
        
        // Print path waypoints
        std::cout << "\nPath waypoints:" << std::endl;
        for (size_t i = 0; i < path->size(); ++i) {
            const auto& node = (*path)[i];
            std::cout << "  " << i << ": ";
            print_state(node->state, "");
        }
        
        // Get detailed path
        auto detailed_path = planner.extract_detailed_path(*path);
        std::cout << "\nDetailed path has " << detailed_path.size() << " states" << std::endl;
        
        // Print statistics
        auto stats = planner.get_statistics(path);
        print_statistics(stats);
        
    } else {
        std::cout << "✗ No path found after " << duration.count() << "ms" << std::endl;
        auto stats = planner.get_statistics(std::nullopt);
        print_statistics(stats);
    }
}

void demo_obstacle_avoidance() {
    std::cout << "\n=== Obstacle Avoidance Demo ===" << std::endl;
    
    // Create vehicle model
    VehicleModel vehicle(2.5, M_PI/4);
    
    // Create planner with finer resolution
    HybridAStar planner(vehicle, 0.5, M_PI/8, M_PI/16, 2.0, 0.8, 0.1);
    
    // Create obstacle map
    int map_size = 40;
    std::vector<std::vector<int>> obstacle_map(map_size, std::vector<int>(map_size, 0));
    
    // Add rectangular obstacles
    std::cout << "Adding obstacles..." << std::endl;
    
    // Central obstacle
    for (int i = 15; i <= 25; ++i) {
        for (int j = 15; j <= 20; ++j) {
            obstacle_map[i][j] = 1;
        }
    }
    
    // Side obstacle
    for (int i = 8; i <= 12; ++i) {
        for (int j = 25; j <= 32; ++j) {
            obstacle_map[i][j] = 1;
        }
    }
    
    planner.set_obstacle_map(obstacle_map, -5.0, -5.0);
    
    // Define challenging start and goal
    State start(-3.0, -3.0, M_PI/4, DirectionMode::NONE, 0.0);
    State goal(12.0, 12.0, M_PI/2, DirectionMode::FORWARD, 0.0);
    
    print_state(start, "Start");
    print_state(goal, "Goal");
    
    // Plan path
    std::cout << "\nPlanning path with obstacles..." << std::endl;
    auto start_time = std::chrono::high_resolution_clock::now();
    auto path = planner.plan_path(start, goal, 10000);
    auto end_time = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    if (path.has_value()) {
        std::cout << "✓ Path found in " << duration.count() << "ms" << std::endl;
        
        // Print key waypoints
        std::cout << "\nKey waypoints:" << std::endl;
        auto& nodes = *path;
        std::cout << "  Start: ";
        print_state(nodes[0]->state, "");
        if (nodes.size() > 2) {
            std::cout << "  Mid:   ";
            print_state(nodes[nodes.size()/2]->state, "");
        }
        std::cout << "  End:   ";
        print_state(nodes.back()->state, "");
        
        // Print statistics
        auto stats = planner.get_statistics(path);
        print_statistics(stats);
        
    } else {
        std::cout << "✗ No path found after " << duration.count() << "ms" << std::endl;
        auto stats = planner.get_statistics(std::nullopt);
        print_statistics(stats);
    }
}

void demo_performance_comparison() {
    std::cout << "\n=== Performance Comparison Demo ===" << std::endl;
    
    VehicleModel vehicle(2.5, M_PI/4);
    
    struct TestConfig {
        std::string name;
        double grid_res;
        double angle_res;
        double steer_res;
        double sim_time;
        double dt;
    };
    
    std::vector<TestConfig> configs = {
        {"Coarse", 1.0, M_PI/6, M_PI/8, 1.0, 0.2},
        {"Medium", 0.5, M_PI/8, M_PI/16, 0.8, 0.1},
        {"Fine", 0.25, M_PI/12, M_PI/24, 0.5, 0.05}
    };
    
    State start(0.0, 0.0, 0.0, DirectionMode::NONE, 0.0);
    State goal(8.0, 8.0, M_PI/4, DirectionMode::FORWARD, 0.0);
    
    std::cout << "Testing different resolution configurations:" << std::endl;
    std::cout << std::setw(10) << "Config" 
              << std::setw(12) << "Time (ms)"
              << std::setw(12) << "Waypoints"
              << std::setw(12) << "Distance"
              << std::setw(12) << "Explored" << std::endl;
    std::cout << std::string(60, '-') << std::endl;
    
    for (const auto& config : configs) {
        HybridAStar planner(vehicle, config.grid_res, config.angle_res, 
                           config.steer_res, 2.0, config.sim_time, config.dt);
        
        auto start_time = std::chrono::high_resolution_clock::now();
        auto path = planner.plan_path(start, goal, 3000);
        auto end_time = std::chrono::high_resolution_clock::now();
        
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        auto stats = planner.get_statistics(path);
        
        std::cout << std::setw(10) << config.name
                  << std::setw(12) << duration.count();
        
        if (path.has_value()) {
            std::cout << std::setw(12) << static_cast<int>(stats.at("path_length_waypoints"))
                      << std::setw(12) << std::fixed << std::setprecision(1) << stats.at("total_distance")
                      << std::setw(12) << static_cast<int>(stats.at("nodes_explored"));
        } else {
            std::cout << std::setw(12) << "N/A"
                      << std::setw(12) << "N/A"
                      << std::setw(12) << static_cast<int>(stats.at("nodes_explored"));
        }
        std::cout << std::endl;
    }
}

int main() {
    std::cout << "Hybrid A* C++ Demo Application" << std::endl;
    std::cout << "===============================" << std::endl;
    
    try {
        demo_simple_path();
        demo_obstacle_avoidance();
        demo_performance_comparison();
        
        std::cout << "\n✓ All demos completed successfully!" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "✗ Demo failed with exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "✗ Demo failed with unknown exception" << std::endl;
        return 1;
    }
    
    return 0;
}
