/**
 * @file advanced_demo.cpp
 * @brief Advanced demo application for Hybrid A* algorithm
 * 
 * This demo showcases more complex scenarios including:
 * - Multiple obstacle configurations
 * - Parking scenarios
 * - Performance benchmarking
 * - Path visualization
 * 
 * @author Converted from Python implementation
 * @date 2025-07-29
 */

#include "hybrid_astar.hpp"
#include <iostream>
#include <chrono>
#include <iomanip>
#include <random>
#include <algorithm>

using namespace hybrid_astar;

// Utility functions
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

// Create a random obstacle map
std::vector<std::vector<int>> create_random_obstacle_map(int size, int num_obstacles, int max_obstacle_size) {
    std::vector<std::vector<int>> obstacle_map(size, std::vector<int>(size, 0));
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(0, size - 1);
    
    for (int i = 0; i < num_obstacles; ++i) {
        int x = dis(gen);
        int y = dis(gen);
        int width = std::max(1, dis(gen) % max_obstacle_size);
        int height = std::max(1, dis(gen) % max_obstacle_size);
        
        // Ensure obstacle doesn't go out of bounds
        width = std::min(width, size - x);
        height = std::min(height, size - y);
        
        for (int j = x; j < x + width; ++j) {
            for (int k = y; k < y + height; ++k) {
                obstacle_map[j][k] = 1;
            }
        }
    }
    
    return obstacle_map;
}

// Create a parking lot environment
std::vector<std::vector<int>> create_parking_lot(int size) {
    std::vector<std::vector<int>> obstacle_map(size, std::vector<int>(size, 0));
    
    // Create walls
    for (int i = 0; i < size; ++i) {
        obstacle_map[0][i] = 1;  // Top wall
        obstacle_map[size-1][i] = 1;  // Bottom wall
        obstacle_map[i][0] = 1;  // Left wall
        obstacle_map[i][size-1] = 1;  // Right wall
    }
    
    // Create parking spots
    int spot_width = 3;
    int spot_height = 5;
    int spacing = 2;
    
    for (int row = 2; row < size - spot_height - 2; row += spot_height + spacing) {
        for (int col = 2; col < size - spot_width - 2; col += spot_width + spacing) {
            // Parked car
            for (int i = row; i < row + spot_width; ++i) {
                for (int j = col; j < col + spot_height; ++j) {
                    if (i < size && j < size) {
                        obstacle_map[i][j] = 1;
                    }
                }
            }
        }
    }
    
    return obstacle_map;
}

// Create a maze-like environment
std::vector<std::vector<int>> create_maze(int size) {
    std::vector<std::vector<int>> obstacle_map(size, std::vector<int>(size, 0));
    
    // Create outer walls
    for (int i = 0; i < size; ++i) {
        obstacle_map[0][i] = 1;
        obstacle_map[size-1][i] = 1;
        obstacle_map[i][0] = 1;
        obstacle_map[i][size-1] = 1;
    }
    
    // Create internal walls
    for (int i = 2; i < size - 2; i += 4) {
        for (int j = 1; j < size - 1; ++j) {
            if (i + 1 < size) {
                obstacle_map[i][j] = 1;
                obstacle_map[i+1][j] = 1;
            }
        }
    }
    
    // Create some openings
    for (int i = 4; i < size - 4; i += 8) {
        for (int j = 2; j < size - 2; j += 4) {
            if (i < size && j < size) {
                obstacle_map[i][j] = 0;
                obstacle_map[i][j+1] = 0;
            }
        }
    }
    
    return obstacle_map;
}

void demo_random_obstacles() {
    std::cout << "\n=== Random Obstacles Demo ===" << std::endl;
    
    // Create vehicle model
    VehicleModel vehicle(2.5, M_PI/4);
    
    // Create planner
    HybridAStar planner(vehicle, 0.5, M_PI/8, M_PI/16, 2.0, 0.8, 0.1);
    
    // Create random obstacle map
    int map_size = 30;
    auto obstacle_map = create_random_obstacle_map(map_size, 15, 5);
    planner.set_obstacle_map(obstacle_map, -5.0, -5.0);
    
    // Define start and goal
    State start(-3.0, -3.0, M_PI/4, DirectionMode::NONE, 0.0);
    State goal(12.0, 12.0, M_PI/2, DirectionMode::FORWARD, 0.0);
    
    print_state(start, "Start");
    print_state(goal, "Goal");
    
    // Plan path
    std::cout << "\nPlanning path with random obstacles..." << std::endl;
    auto start_time = std::chrono::high_resolution_clock::now();
    auto path = planner.plan_path(start, goal, 15000);
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

void demo_parking_scenario() {
    std::cout << "\n=== Parking Scenario Demo ===" << std::endl;
    
    // Create vehicle model
    VehicleModel vehicle(2.7, M_PI/3);  // More agile vehicle for parking
    
    // Create planner with finer resolution for parking
    HybridAStar planner(vehicle, 0.3, M_PI/12, M_PI/24, 1.5, 0.6, 0.05);
    
    // Create parking lot environment
    int map_size = 25;
    auto obstacle_map = create_parking_lot(map_size);
    planner.set_obstacle_map(obstacle_map, -2.0, -2.0);
    
    // Define parking scenario
    State start(10.0, 18.0, -M_PI/2, DirectionMode::NONE, 0.0);  // Coming from top
    State goal(12.0, 8.0, 0.0, DirectionMode::FORWARD, 0.0);     // Parking spot
    
    print_state(start, "Start");
    print_state(goal, "Goal");
    
    // Plan path
    std::cout << "\nPlanning parking maneuver..." << std::endl;
    auto start_time = std::chrono::high_resolution_clock::now();
    auto path = planner.plan_path(start, goal, 20000);
    auto end_time = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    if (path.has_value()) {
        std::cout << "✓ Parking path found in " << duration.count() << "ms" << std::endl;
        
        // Print key waypoints
        std::cout << "\nKey waypoints:" << std::endl;
        auto& nodes = *path;
        std::cout << "  Start: ";
        print_state(nodes[0]->state, "");
        if (nodes.size() > 4) {
            std::cout << "  Approach: ";
            print_state(nodes[nodes.size()/4]->state, "");
            std::cout << "  Mid:      ";
            print_state(nodes[nodes.size()/2]->state, "");
        }
        std::cout << "  End:      ";
        print_state(nodes.back()->state, "");
        
        // Print statistics
        auto stats = planner.get_statistics(path);
        print_statistics(stats);
        
    } else {
        std::cout << "✗ No parking path found after " << duration.count() << "ms" << std::endl;
        auto stats = planner.get_statistics(std::nullopt);
        print_statistics(stats);
    }
}

void demo_maze_navigation() {
    std::cout << "\n=== Maze Navigation Demo ===" << std::endl;
    
    // Create vehicle model
    VehicleModel vehicle(2.0, M_PI/3);  // Smaller vehicle for tight spaces
    
    // Create planner with fine resolution for maze
    HybridAStar planner(vehicle, 0.4, M_PI/10, M_PI/20, 1.0, 0.5, 0.05);
    
    // Create maze environment
    int map_size = 25;
    auto obstacle_map = create_maze(map_size);
    planner.set_obstacle_map(obstacle_map, -2.0, -2.0);
    
    // Define maze navigation scenario
    State start(1.0, 1.0, 0.0, DirectionMode::NONE, 0.0);   // Start at entrance
    State goal(20.0, 20.0, M_PI/2, DirectionMode::FORWARD, 0.0);  // Goal at exit
    
    print_state(start, "Start");
    print_state(goal, "Goal");
    
    // Plan path
    std::cout << "\nPlanning maze navigation..." << std::endl;
    auto start_time = std::chrono::high_resolution_clock::now();
    auto path = planner.plan_path(start, goal, 30000);
    auto end_time = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    
    if (path.has_value()) {
        std::cout << "✓ Maze path found in " << duration.count() << "ms" << std::endl;
        
        // Print key waypoints
        std::cout << "\nKey waypoints:" << std::endl;
        auto& nodes = *path;
        std::cout << "  Start: ";
        print_state(nodes[0]->state, "");
        if (nodes.size() > 4) {
            std::cout << "  1/4:   ";
            print_state(nodes[nodes.size()/4]->state, "");
            std::cout << "  1/2:   ";
            print_state(nodes[nodes.size()/2]->state, "");
            std::cout << "  3/4:   ";
            print_state(nodes[3*nodes.size()/4]->state, "");
        }
        std::cout << "  End:   ";
        print_state(nodes.back()->state, "");
        
        // Print statistics
        auto stats = planner.get_statistics(path);
        print_statistics(stats);
        
    } else {
        std::cout << "✗ No maze path found after " << duration.count() << "ms" << std::endl;
        auto stats = planner.get_statistics(std::nullopt);
        print_statistics(stats);
    }
}

void demo_performance_benchmark() {
    std::cout << "\n=== Performance Benchmark Demo ===" << std::endl;
    
    VehicleModel vehicle(2.5, M_PI/4);
    
    struct BenchmarkConfig {
        std::string name;
        double grid_res;
        double angle_res;
        double steer_res;
        double sim_time;
        double dt;
        int max_iter;
    };
    
    std::vector<BenchmarkConfig> configs = {
        {"Ultra Fast", 2.0, M_PI/4, M_PI/6, 1.0, 0.2, 1000},
        {"Fast", 1.0, M_PI/6, M_PI/8, 0.8, 0.15, 2000},
        {"Balanced", 0.5, M_PI/8, M_PI/16, 0.6, 0.1, 5000},
        {"Precise", 0.25, M_PI/12, M_PI/24, 0.4, 0.05, 10000},
        {"Ultra Precise", 0.125, M_PI/16, M_PI/32, 0.3, 0.025, 20000}
    };
    
    State start(0.0, 0.0, 0.0, DirectionMode::NONE, 0.0);
    State goal(15.0, 10.0, M_PI/4, DirectionMode::FORWARD, 0.0);
    
    // Create a moderate obstacle map
    int map_size = 40;
    std::vector<std::vector<int>> obstacle_map(map_size, std::vector<int>(map_size, 0));
    
    // Add some obstacles
    for (int i = 5; i <= 15; ++i) {
        for (int j = 8; j <= 12; ++j) {
            obstacle_map[i][j] = 1;
        }
    }
    
    std::cout << "Testing different resolution configurations:" << std::endl;
    std::cout << std::setw(15) << "Config" 
              << std::setw(12) << "Time (ms)"
              << std::setw(12) << "Waypoints"
              << std::setw(12) << "Distance"
              << std::setw(12) << "Explored"
              << std::setw(12) << "Success" << std::endl;
    std::cout << std::string(75, '-') << std::endl;
    
    for (const auto& config : configs) {
        HybridAStar planner(vehicle, config.grid_res, config.angle_res, 
                           config.steer_res, 2.0, config.sim_time, config.dt);
        
        // Scale obstacle map based on resolution
        int scaled_map_size = static_cast<int>(map_size * config.grid_res);
        std::vector<std::vector<int>> scaled_obstacle_map(scaled_map_size, std::vector<int>(scaled_map_size, 0));
        
        for (int i = 0; i < scaled_map_size && i * config.grid_res < map_size; ++i) {
            for (int j = 0; j < scaled_map_size && j * config.grid_res < map_size; ++j) {
                int orig_i = static_cast<int>(i / config.grid_res);
                int orig_j = static_cast<int>(j / config.grid_res);
                if (orig_i < map_size && orig_j < map_size) {
                    scaled_obstacle_map[i][j] = obstacle_map[orig_i][orig_j];
                }
            }
        }
        
        planner.set_obstacle_map(scaled_obstacle_map, -5.0 * config.grid_res, -5.0 * config.grid_res);
        
        auto start_time = std::chrono::high_resolution_clock::now();
        auto path = planner.plan_path(start, goal, config.max_iter);
        auto end_time = std::chrono::high_resolution_clock::now();
        
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
        auto stats = planner.get_statistics(path);
        
        std::cout << std::setw(15) << config.name
                  << std::setw(12) << duration.count();
        
        if (path.has_value()) {
            std::cout << std::setw(12) << static_cast<int>(stats.at("path_length_waypoints"))
                      << std::setw(12) << std::fixed << std::setprecision(1) << stats.at("total_distance")
                      << std::setw(12) << static_cast<int>(stats.at("nodes_explored"))
                      << std::setw(12) << "✓";
        } else {
            std::cout << std::setw(12) << "N/A"
                      << std::setw(12) << "N/A"
                      << std::setw(12) << static_cast<int>(stats.at("nodes_explored"))
                      << std::setw(12) << "✗";
        }
        std::cout << std::endl;
    }
}

int main() {
    std::cout << "Hybrid A* C++ Advanced Demo Application" << std::endl;
    std::cout << "========================================" << std::endl;
    
    try {
        demo_random_obstacles();
        demo_parking_scenario();
        demo_maze_navigation();
        demo_performance_benchmark();
        
        std::cout << "\n✓ All advanced demos completed successfully!" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "✗ Demo failed with exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "✗ Demo failed with unknown exception" << std::endl;
        return 1;
    }
    
    return 0;
}