/**
 * @file test_main.cpp
 * @brief Test suite for Hybrid A* algorithm
 * 
 * This file contains unit tests for the Hybrid A* implementation.
 * 
 * @author Converted from Python implementation
 * @date 2025-07-29
 */

#include "hybrid_astar.hpp"
#include <iostream>
#include <cassert>
#include <chrono>
#include <cmath>

using namespace hybrid_astar;

// Test utilities
bool test_passed = true;
int test_count = 0;
int passed_count = 0;

void run_test(const std::string& test_name, std::function<void()> test_func) {
    test_count++;
    std::cout << "Running test: " << test_name << "... ";
    
    try {
        test_func();
        std::cout << "PASSED" << std::endl;
        passed_count++;
    } catch (const std::exception& e) {
        std::cout << "FAILED: " << e.what() << std::endl;
        test_passed = false;
    } catch (...) {
        std::cout << "FAILED with unknown exception" << std::endl;
        test_passed = false;
    }
}

// Test cases
void test_state_equality() {
    State s1(1.0, 2.0, M_PI/4, DirectionMode::FORWARD, 0.1);
    State s2(1.05, 2.05, M_PI/4 + 0.05, DirectionMode::FORWARD, 0.1);
    State s3(1.0, 2.0, M_PI/4, DirectionMode::BACKWARD, 0.1);
    
    // Test equality within tolerance
    assert(s1 == s2);
    
    // Test inequality
    assert(s1 != s3);
    
    // Test direction modes
    assert(s1.direction == DirectionMode::FORWARD);
    assert(s3.direction == DirectionMode::BACKWARD);
}

void test_vehicle_model() {
    VehicleModel vehicle(2.5, M_PI/4);
    
    // Test getters
    assert(std::abs(vehicle.wheelbase() - 2.5) < 1e-6);
    assert(std::abs(vehicle.max_steer() - M_PI/4) < 1e-6);
    
    // Test motion simulation
    State initial(0.0, 0.0, 0.0, DirectionMode::FORWARD, 0.0);
    auto states = vehicle.simulate_motion(initial, 1.0, 0.0, 0.1, 10);
    
    assert(states.size() == 10);
    assert(states[0] == initial);
    
    // Test final position (should be approximately 1.0m forward)
    assert(std::abs(states.back().x - 1.0) < 0.01);
    assert(std::abs(states.back().y) < 0.01);
    assert(std::abs(states.back().yaw) < 0.01);
    
    // Test angle normalization
    double angle = 3 * M_PI;
    double normalized = VehicleModel::normalize_angle(angle);
    assert(normalized < M_PI && normalized >= -M_PI);
}

void test_heuristic_cost() {
    VehicleModel vehicle(2.5, M_PI/4);
    HybridAStar planner(vehicle);
    
    State start(0.0, 0.0, 0.0, DirectionMode::NONE, 0.0);
    State goal(10.0, 10.0, M_PI/2, DirectionMode::FORWARD, 0.0);
    
    double cost = planner.heuristic_cost(start, goal);
    
    // Should be approximately Euclidean distance plus some angular cost
    double euclidean_dist = std::sqrt(10.0 * 10.0 + 10.0 * 10.0);
    assert(cost > euclidean_dist - 1.0 && cost < euclidean_dist + 5.0);
}

void test_collision_detection() {
    VehicleModel vehicle(2.5, M_PI/4);
    HybridAStar planner(vehicle, 1.0, M_PI/8, M_PI/16, 2.0, 1.0, 0.2);
    
    // Create simple obstacle map
    std::vector<std::vector<int>> obstacle_map(10, std::vector<int>(10, 0));
    obstacle_map[5][5] = 1;  // Single obstacle at center
    
    planner.set_obstacle_map(obstacle_map, 0.0, 0.0);
    
    // Test free state
    State free_state(1.0, 1.0, 0.0, DirectionMode::FORWARD, 0.0);
    assert(planner.is_collision_free(free_state));
    
    // Test obstacle state
    State obstacle_state(5.0, 5.0, 0.0, DirectionMode::FORWARD, 0.0);
    assert(!planner.is_collision_free(obstacle_state));
    
    // Test out of bounds
    State out_of_bounds(15.0, 15.0, 0.0, DirectionMode::FORWARD, 0.0);
    assert(!planner.is_collision_free(out_of_bounds));
}

void test_simple_path_planning() {
    VehicleModel vehicle(2.5, M_PI/4);
    HybridAStar planner(vehicle, 1.0, M_PI/8, M_PI/16, 2.0, 1.0, 0.2);
    
    // No obstacles
    std::vector<std::vector<int>> obstacle_map(20, std::vector<int>(20, 0));
    planner.set_obstacle_map(obstacle_map, 0.0, 0.0);
    
    State start(1.0, 1.0, 0.0, DirectionMode::NONE, 0.0);
    State goal(10.0, 10.0, M_PI/2, DirectionMode::FORWARD, 0.0);
    
    auto path = planner.plan_path(start, goal, 1000);
    
    assert(path.has_value());
    assert(!path->empty());
    
    // Check start and goal match
    assert((*path)[0]->state == start);
    assert((*path).back()->state.x == goal.x);
    assert((*path).back()->state.y == goal.y);
    
    // Check path statistics
    auto stats = planner.get_statistics(path);
    assert(stats.at("path_found") == 1.0);
    assert(stats.at("path_length_waypoints") > 0);
    assert(stats.at("total_distance") > 0);
}

void test_obstacle_avoidance() {
    VehicleModel vehicle(2.5, M_PI/4);
    HybridAStar planner(vehicle, 0.5, M_PI/8, M_PI/16, 2.0, 0.8, 0.1);
    
    // Create obstacle map
    std::vector<std::vector<int>> obstacle_map(20, std::vector<int>(20, 0));
    
    // Add obstacle in the middle
    for (int i = 8; i <= 12; ++i) {
        for (int j = 8; j <= 12; ++j) {
            obstacle_map[i][j] = 1;
        }
    }
    
    planner.set_obstacle_map(obstacle_map, 0.0, 0.0);
    
    // Define start and goal with obstacle in between
    State start(2.0, 2.0, 0.0, DirectionMode::NONE, 0.0);
    State goal(15.0, 15.0, M_PI/4, DirectionMode::FORWARD, 0.0);
    
    auto path = planner.plan_path(start, goal, 5000);
    
    // Should find a path around the obstacle
    assert(path.has_value());
    assert(!path->empty());
    
    // Check that path doesn't go through obstacle
    for (const auto& node : *path) {
        int grid_x = static_cast<int>((node->state.x) / planner.get_grid_resolution());
        int grid_y = static_cast<int>((node->state.y) / planner.get_grid_resolution());
        
        if (grid_x >= 0 && grid_x < 20 && grid_y >= 0 && grid_y < 20) {
            assert(obstacle_map[grid_y][grid_x] == 0);
        }
    }
}

void test_detailed_path_extraction() {
    VehicleModel vehicle(2.5, M_PI/4);
    HybridAStar planner(vehicle, 1.0, M_PI/8, M_PI/16, 2.0, 1.0, 0.2);
    
    std::vector<std::vector<int>> obstacle_map(15, std::vector<int>(15, 0));
    planner.set_obstacle_map(obstacle_map, 0.0, 0.0);
    
    State start(1.0, 1.0, 0.0, DirectionMode::NONE, 0.0);
    State goal(10.0, 10.0, M_PI/2, DirectionMode::FORWARD, 0.0);
    
    auto path = planner.plan_path(start, goal, 1000);
    assert(path.has_value());
    
    // Extract detailed path
    auto detailed_path = planner.extract_detailed_path(*path);
    
    // Detailed path should have more states than waypoints
    assert(detailed_path.size() >= path->size());
    
    // First state should match start
    assert(detailed_path[0] == start);
    
    // Last state should be close to goal
    assert(std::abs(detailed_path.back().x - goal.x) < 1.0);
    assert(std::abs(detailed_path.back().y - goal.y) < 1.0);
}

void test_performance() {
    VehicleModel vehicle(2.5, M_PI/4);
    
    // Test with different resolutions
    std::vector<double> resolutions = {2.0, 1.0, 0.5};
    
    for (double res : resolutions) {
        HybridAStar planner(vehicle, res, M_PI/8, M_PI/16, 2.0, 1.0, 0.2);
        
        std::vector<std::vector<int>> obstacle_map(20, std::vector<int>(20, 0));
        planner.set_obstacle_map(obstacle_map, 0.0, 0.0);
        
        State start(1.0, 1.0, 0.0, DirectionMode::NONE, 0.0);
        State goal(10.0, 10.0, M_PI/2, DirectionMode::FORWARD, 0.0);
        
        auto start_time = std::chrono::high_resolution_clock::now();
        auto path = planner.plan_path(start, goal, 1000);
        auto end_time = std::chrono::high_resolution_clock::now();
        
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        
        // Should complete in reasonable time
        assert(duration.count() < 1000000);  // Less than 1 second
        
        if (path.has_value()) {
            auto stats = planner.get_statistics(path);
            assert(stats.at("path_found") == 1.0);
        }
    }
}

void test_goal_reached() {
    VehicleModel vehicle(2.5, M_PI/4);
    HybridAStar planner(vehicle, 1.0, M_PI/8, M_PI/16, 2.0, 1.0, 0.2);
    
    State goal(10.0, 10.0, M_PI/2, DirectionMode::FORWARD, 0.0);
    
    // Test exact match
    State exact_goal(10.0, 10.0, M_PI/2, DirectionMode::FORWARD, 0.0);
    assert(planner.is_goal_reached(exact_goal, goal));
    
    // Test within tolerance
    State close_goal(10.5, 10.2, M_PI/2 + 0.1, DirectionMode::FORWARD, 0.0);
    assert(planner.is_goal_reached(close_goal, goal));
    
    // Test too far
    State far_goal(15.0, 15.0, M_PI/2, DirectionMode::FORWARD, 0.0);
    assert(!planner.is_goal_reached(far_goal, goal));
    
    // Test wrong angle
    State wrong_angle(10.0, 10.0, 0.0, DirectionMode::FORWARD, 0.0);
    assert(!planner.is_goal_reached(wrong_angle, goal));
}

// Main test function
int main(int argc, char* argv[]) {
    std::cout << "Hybrid A* C++ Test Suite" << std::endl;
    std::cout << "=========================" << std::endl;
    
    // Run individual tests based on command line arguments
    std::string test_type = "all";
    if (argc > 1) {
        test_type = argv[1];
    }
    
    // Basic functionality tests
    if (test_type == "all" || test_type == "basic") {
        run_test("State Equality", test_state_equality);
        run_test("Vehicle Model", test_vehicle_model);
        run_test("Heuristic Cost", test_heuristic_cost);
        run_test("Collision Detection", test_collision_detection);
    }
    
    // Path planning tests
    if (test_type == "all" || test_type == "planning") {
        run_test("Simple Path Planning", test_simple_path_planning);
        run_test("Obstacle Avoidance", test_obstacle_avoidance);
        run_test("Detailed Path Extraction", test_detailed_path_extraction);
    }
    
    // Advanced tests
    if (test_type == "all" || test_type == "advanced") {
        run_test("Goal Reached Check", test_goal_reached);
        run_test("Performance", test_performance);
    }
    
    // Print summary
    std::cout << "\nTest Summary:" << std::endl;
    std::cout << "=============" << std::endl;
    std::cout << "Total tests: " << test_count << std::endl;
    std::cout << "Passed: " << passed_count << std::endl;
    std::cout << "Failed: " << test_count - passed_count << std::endl;
    
    if (test_passed) {
        std::cout << "\n✓ All tests passed!" << std::endl;
        return 0;
    } else {
        std::cout << "\n✗ Some tests failed!" << std::endl;
        return 1;
    }
}