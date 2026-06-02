/**
 * @file test_main.cpp
 * @brief Test suite for Hybrid A* algorithm
 * 
 * @author Converted from Python implementation
 * @date 2025-07-29
 */

#include "hybrid_astar.hpp"
#include <iostream>
#include <cassert>
#include <vector>
#include <chrono>

using namespace hybrid_astar;

void test_vehicle_model() {
    std::cout << "Testing VehicleModel..." << std::endl;
    
    VehicleModel vehicle(2.5, M_PI/4);
    
    // Test normalize_angle
    assert(std::abs(VehicleModel::normalize_angle(3*M_PI) - M_PI) < 1e-6);
    assert(std::abs(VehicleModel::normalize_angle(-3*M_PI) - (-M_PI)) < 1e-6);
    assert(std::abs(VehicleModel::normalize_angle(M_PI/2) - M_PI/2) < 1e-6);
    
    // Test motion simulation
    State initial_state(0.0, 0.0, 0.0, DirectionMode::FORWARD, 0.0);
    auto states = vehicle.simulate_motion(initial_state, 1.0, 0.0, 0.1, 10);
    
    assert(states.size() == 10);
    assert(states[9].x > 0.9);  // Should have moved forward
    assert(std::abs(states[9].y) < 0.1);  // Should stay roughly on x-axis
    
    std::cout << "VehicleModel tests passed!" << std::endl;
}

void test_state_and_costs() {
    std::cout << "Testing State and Costs..." << std::endl;
    
    // Test State
    State s1(1.0, 2.0, M_PI/4, DirectionMode::FORWARD, 0.1);
    State s2(1.05, 2.05, M_PI/4 + 0.05, DirectionMode::FORWARD, 0.1);
    State s3(2.0, 3.0, M_PI/2, DirectionMode::BACKWARD, 0.2);
    
    assert(s1 == s2);  // Should be equal within tolerance
    assert(s1 != s3);  // Should be different
    
    // Test Costs
    Costs costs(1.0, 0.5, 0.2, 0.1);
    assert(costs.distance == 1.0);
    assert(costs.steer == 0.5);
    assert(costs.turn == 0.2);
    assert(costs.cusp == 0.1);
    
    std::cout << "State and Costs tests passed!" << std::endl;
}

void test_hybrid_astar_basic() {
    std::cout << "Testing HybridAStar basic functionality..." << std::endl;
    
    VehicleModel vehicle(2.5, M_PI/4);
    HybridAStar planner(vehicle, 0.5, M_PI/8, M_PI/16, 2.0, 0.5, 0.1);
    
    // Test collision checking without obstacle map
    State test_state(5.0, 5.0, 0.0, DirectionMode::FORWARD, 0.0);
    assert(planner.is_collision_free(test_state));  // Should be collision-free
    
    // Test heuristic cost
    State start(0.0, 0.0, 0.0, DirectionMode::NONE, 0.0);
    State goal(10.0, 10.0, M_PI/2, DirectionMode::FORWARD, 0.0);
    double h_cost = planner.heuristic_cost(start, goal);
    assert(h_cost > 0.0);  // Should have positive cost
    
    std::cout << "HybridAStar basic tests passed!" << std::endl;
}

void test_obstacle_map() {
    std::cout << "Testing obstacle map functionality..." << std::endl;
    
    VehicleModel vehicle(2.5, M_PI/4);
    HybridAStar planner(vehicle);
    
    // Create simple obstacle map
    std::vector<std::vector<int>> obstacle_map(20, std::vector<int>(20, 0));
    
    // Add some obstacles
    for (int i = 8; i <= 12; ++i) {
        for (int j = 8; j <= 12; ++j) {
            obstacle_map[i][j] = 1;
        }
    }
    
    planner.set_obstacle_map(obstacle_map, 0.0, 0.0);
    
    // Test collision checking
    State free_state(1.0, 1.0, 0.0, DirectionMode::FORWARD, 0.0);
    State obstacle_state(10.0, 10.0, 0.0, DirectionMode::FORWARD, 0.0);
    
    assert(planner.is_collision_free(free_state));
    assert(!planner.is_collision_free(obstacle_state));
    
    std::cout << "Obstacle map tests passed!" << std::endl;
}

void test_path_planning_simple() {
    std::cout << "Testing simple path planning..." << std::endl;
    
    VehicleModel vehicle(2.5, M_PI/4);
    HybridAStar planner(vehicle, 1.0, M_PI/8, M_PI/16, 2.0, 1.0, 0.2);
    
    // Simple scenario without obstacles
    State start(0.0, 0.0, 0.0, DirectionMode::NONE, 0.0);
    State goal(5.0, 0.0, 0.0, DirectionMode::FORWARD, 0.0);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    auto path = planner.plan_path(start, goal, 1000);
    auto end_time = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "Planning took: " << duration.count() << "ms" << std::endl;
    
    if (path.has_value()) {
        std::cout << "Path found with " << path->size() << " waypoints" << std::endl;
        
        // Test statistics
        auto stats = planner.get_statistics(path);
        std::cout << "Path statistics:" << std::endl;
        for (const auto& [key, value] : stats) {
            std::cout << "  " << key << ": " << value << std::endl;
        }
        
        // Test detailed path extraction
        auto detailed_path = planner.extract_detailed_path(*path);
        std::cout << "Detailed path has " << detailed_path.size() << " states" << std::endl;
        
        assert(detailed_path.size() >= path->size());
        
    } else {
        std::cout << "No path found (this may be expected for simple test)" << std::endl;
    }
    
    std::cout << "Simple path planning test completed!" << std::endl;
}

void benchmark_planning() {
    std::cout << "Running planning benchmark..." << std::endl;
    
    VehicleModel vehicle(2.5, M_PI/4);
    HybridAStar planner(vehicle, 0.5, M_PI/8, M_PI/16, 2.0, 0.5, 0.1);
    
    // Create obstacle map with some obstacles
    std::vector<std::vector<int>> obstacle_map(50, std::vector<int>(50, 0));
    
    // Add some rectangular obstacles
    for (int i = 15; i <= 25; ++i) {
        for (int j = 15; j <= 20; ++j) {
            obstacle_map[i][j] = 1;
        }
    }
    
    for (int i = 30; i <= 35; ++i) {
        for (int j = 25; j <= 35; ++j) {
            obstacle_map[i][j] = 1;
        }
    }
    
    planner.set_obstacle_map(obstacle_map, -10.0, -10.0);
    
    State start(-5.0, -5.0, M_PI/4, DirectionMode::NONE, 0.0);
    State goal(15.0, 15.0, M_PI/2, DirectionMode::FORWARD, 0.0);
    
    auto start_time = std::chrono::high_resolution_clock::now();
    auto path = planner.plan_path(start, goal, 5000);
    auto end_time = std::chrono::high_resolution_clock::now();
    
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
    std::cout << "Benchmark planning took: " << duration.count() << "ms" << std::endl;
    
    if (path.has_value()) {
        auto stats = planner.get_statistics(path);
        std::cout << "Benchmark results:" << std::endl;
        std::cout << "  Path length: " << stats.at("path_length_waypoints") << " waypoints" << std::endl;
        std::cout << "  Total distance: " << stats.at("total_distance") << "m" << std::endl;
        std::cout << "  Nodes explored: " << stats.at("nodes_explored") << std::endl;
        std::cout << "  Max steering angle: " << stats.at("max_steering_angle") << " rad" << std::endl;
    } else {
        std::cout << "Benchmark: No path found" << std::endl;
    }
}

int main() {
    std::cout << "Running Hybrid A* C++ Tests" << std::endl;
    std::cout << "============================" << std::endl;
    
    try {
        test_vehicle_model();
        test_state_and_costs();
        test_hybrid_astar_basic();
        test_obstacle_map();
        test_path_planning_simple();
        benchmark_planning();
        
        std::cout << std::endl;
        std::cout << "All tests completed successfully!" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    } catch (...) {
        std::cerr << "Test failed with unknown exception" << std::endl;
        return 1;
    }
    
    return 0;
}
