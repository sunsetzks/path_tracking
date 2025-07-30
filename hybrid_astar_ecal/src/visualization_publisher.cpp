/**
 * @file visualization_publisher.cpp
 * @brief Implementation of eCAL-based visualization publisher (stub version)
 */

#include "visualization_publisher.hpp"
#include <iostream>

namespace hybrid_astar {

VisualizationPublisher::VisualizationPublisher(const std::string& node_name)
    : node_name_(node_name), initialized_(false) {
#ifndef ECAL_PROTOBUF_AVAILABLE
    planning_result_pub_ = nullptr;
    planning_status_pub_ = nullptr;
    obstacle_map_pub_ = nullptr;
    path_pub_ = nullptr;
    markers_pub_ = nullptr;
#endif
}

VisualizationPublisher::~VisualizationPublisher() {
    shutdown();
}

bool VisualizationPublisher::initialize() {
#ifdef ECAL_PROTOBUF_AVAILABLE
    try {
        // Initialize eCAL
        eCAL::Initialize(0, nullptr, node_name_.c_str());
        
        // Create publishers
        planning_result_pub_ = std::make_unique<eCAL::protobuf::CPublisher<hybrid_astar::PlanningResult>>("planning_result");
        planning_status_pub_ = std::make_unique<eCAL::protobuf::CPublisher<hybrid_astar::PlanningStatus>>("planning_status");
        obstacle_map_pub_ = std::make_unique<eCAL::protobuf::CPublisher<foxglove::Grid>>("obstacle_map");
        path_pub_ = std::make_unique<eCAL::protobuf::CPublisher<foxglove::Path>>("planned_path");
        markers_pub_ = std::make_unique<eCAL::protobuf::CPublisher<foxglove::MarkerArray>>("planning_markers");
        
        initialized_ = true;
        std::cout << "eCAL visualization publisher initialized successfully" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize eCAL publisher: " << e.what() << std::endl;
        return false;
    }
#else
    std::cout << "eCAL visualization publisher: stub mode (eCAL not available)" << std::endl;
    initialized_ = true;
    return true;
#endif
}

void VisualizationPublisher::shutdown() {
    if (initialized_) {
#ifdef ECAL_PROTOBUF_AVAILABLE
        planning_result_pub_.reset();
        planning_status_pub_.reset();
        obstacle_map_pub_.reset();
        path_pub_.reset();
        markers_pub_.reset();
        
        eCAL::Finalize();
#endif
        initialized_ = false;
        std::cout << "eCAL visualization publisher shutdown" << std::endl;
    }
}

void VisualizationPublisher::publish_planning_result(
    const State& start_state,
    const State& goal_state,
    const std::optional<std::vector<std::shared_ptr<Node>>>& path_nodes,
    const std::vector<std::shared_ptr<Node>>& explored_nodes,
    const std::vector<State>& detailed_path,
    const std::unordered_map<std::string, double>& statistics,
    const std::vector<std::vector<int>>& obstacle_map,
    double map_origin_x, double map_origin_y, double grid_resolution,
    double planning_time_ms) {
    
    if (!initialized_) {
        std::cerr << "Publisher not initialized" << std::endl;
        return;
    }
    
#ifdef ECAL_PROTOBUF_AVAILABLE
    // Full implementation with eCAL (will be implemented when protobuf is available)
    std::cout << "Publishing planning result with eCAL..." << std::endl;
#else
    // Stub implementation
    std::cout << "Visualization Publisher (stub): Publishing planning result" << std::endl;
    std::cout << "  - Start: (" << start_state.x << ", " << start_state.y << ", " << start_state.yaw << ")" << std::endl;
    std::cout << "  - Goal: (" << goal_state.x << ", " << goal_state.y << ", " << goal_state.yaw << ")" << std::endl;
    std::cout << "  - Path found: " << (path_nodes.has_value() ? "Yes" : "No") << std::endl;
    std::cout << "  - Explored nodes: " << explored_nodes.size() << std::endl;
    std::cout << "  - Detailed path points: " << detailed_path.size() << std::endl;
    std::cout << "  - Planning time: " << planning_time_ms << " ms" << std::endl;
    std::cout << "  - Map size: " << obstacle_map.size() << "x" << (obstacle_map.empty() ? 0 : obstacle_map[0].size()) << std::endl;
#endif
}

void VisualizationPublisher::publish_planning_status(
    int status,
    const std::string& message,
    int current_iteration,
    int max_iterations) {
    
    if (!initialized_) return;
    
#ifdef ECAL_PROTOBUF_AVAILABLE
    // Full implementation with eCAL
    std::cout << "Publishing planning status with eCAL..." << std::endl;
#else
    // Stub implementation
    std::cout << "Visualization Publisher (stub): Status=" << status << ", Message=" << message;
    if (max_iterations > 0) {
        std::cout << ", Progress=" << current_iteration << "/" << max_iterations;
    }
    std::cout << std::endl;
#endif
}

void VisualizationPublisher::publish_obstacle_map(
    const std::vector<std::vector<int>>& obstacle_map,
    double origin_x, double origin_y, double grid_resolution) {
    
    if (!initialized_) return;
    
#ifdef ECAL_PROTOBUF_AVAILABLE
    // Full implementation with eCAL
    std::cout << "Publishing obstacle map with eCAL..." << std::endl;
#else
    // Stub implementation
    std::cout << "Visualization Publisher (stub): Publishing obstacle map (" 
              << obstacle_map.size() << "x" << (obstacle_map.empty() ? 0 : obstacle_map[0].size()) 
              << ") at origin (" << origin_x << ", " << origin_y << ") with resolution " << grid_resolution << std::endl;
#endif
}

// Stub implementations for helper functions
void* VisualizationPublisher::get_current_time() const {
#ifdef ECAL_PROTOBUF_AVAILABLE
    // Will return actual foxglove::Time when available
    return nullptr;
#else
    return nullptr;
#endif
}

void* VisualizationPublisher::state_to_proto(const State& state) const {
    return nullptr;
}

void* VisualizationPublisher::node_to_proto(const std::shared_ptr<Node>& node) const {
    return nullptr;
}

void* VisualizationPublisher::create_obstacle_map_grid(
    const std::vector<std::vector<int>>& obstacle_map,
    double origin_x, double origin_y, double grid_resolution) const {
    return nullptr;
}

void* VisualizationPublisher::create_path_message(const std::vector<State>& states) const {
    return nullptr;
}

void* VisualizationPublisher::create_exploration_markers(
    const std::vector<std::shared_ptr<Node>>& explored_nodes) const {
    return nullptr;
}

void* VisualizationPublisher::create_vehicle_markers(
    const State& start_state, const State& goal_state) const {
    return nullptr;
}

void* VisualizationPublisher::yaw_to_quaternion(double yaw) const {
    return nullptr;
}

} // namespace hybrid_astar
