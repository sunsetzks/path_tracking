/**
 * @file visualization_publisher.hpp
 * @brief eCAL-based visualization publisher for Hybrid A* planning
 */

#pragma once

#include "common_types.hpp"
#include <memory>
#include <string>
#include <chrono>
#include <optional>
#include <unordered_map>
#include <vector>

// Forward declarations to avoid include dependencies when protobuf is not available
#ifdef ECAL_PROTOBUF_AVAILABLE
#include "hybrid_astar.pb.h"
#include "foxglove/Grid.pb.h"
#include "foxglove/Path.pb.h" 
#include "foxglove/MarkerArray.pb.h"
#include "foxglove/Time.pb.h"
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#endif

namespace hybrid_astar {

// Forward declarations for foxglove types (they are in ::foxglove namespace)
class PlanningResult;
class PlanningStatus;

/**
 * @brief Publisher for Hybrid A* visualization data using eCAL and Foxglove
 */
class VisualizationPublisher {
public:
    /**
     * @brief Constructor
     * @param node_name eCAL node name
     */
    explicit VisualizationPublisher(const std::string& node_name = "hybrid_astar_viz");
    
    /**
     * @brief Destructor
     */
    ~VisualizationPublisher();
    
    /**
     * @brief Initialize eCAL and create publishers
     */
    bool initialize();
    
    /**
     * @brief Shutdown eCAL and cleanup
     */
    void shutdown();
    
    /**
     * @brief Publish complete planning result
     * @param start_state Start state
     * @param goal_state Goal state
     * @param path_nodes Optional planning result
     * @param explored_nodes Explored nodes during planning
     * @param detailed_path Detailed state sequence
     * @param statistics Planning statistics
     * @param obstacle_map Obstacle map
     * @param planning_time_ms Planning time in milliseconds
     */
    void publish_planning_result(
        const State& start_state,
        const State& goal_state,
        const std::optional<std::vector<std::shared_ptr<Node>>>& path_nodes,
        const std::vector<std::shared_ptr<Node>>& explored_nodes,
        const std::vector<State>& detailed_path,
        const std::unordered_map<std::string, double>& statistics,
        const std::vector<std::vector<int>>& obstacle_map,
        double map_origin_x, double map_origin_y, double grid_resolution,
        double planning_time_ms);
    
    /**
     * @brief Publish planning status update
     * @param status Current planning status
     * @param message Status message
     * @param current_iteration Current iteration
     * @param max_iterations Maximum iterations
     */
    void publish_planning_status(
        int status,  // Simplified to int to avoid enum dependency
        const std::string& message,
        int current_iteration = 0,
        int max_iterations = 0);
    
    /**
     * @brief Publish obstacle map only
     */
    void publish_obstacle_map(
        const std::vector<std::vector<int>>& obstacle_map,
        double origin_x, double origin_y, double grid_resolution);

private:
    std::string node_name_;
    bool initialized_;
    
#ifdef ECAL_PROTOBUF_AVAILABLE
    // eCAL publishers - only compiled when protobuf is available
    std::unique_ptr<eCAL::protobuf::CPublisher<hybrid_astar::PlanningResult>> planning_result_pub_;
    std::unique_ptr<eCAL::protobuf::CPublisher<hybrid_astar::PlanningStatus>> planning_status_pub_;
    std::unique_ptr<eCAL::protobuf::CPublisher<foxglove::Grid>> obstacle_map_pub_;
    std::unique_ptr<eCAL::protobuf::CPublisher<foxglove::Path>> path_pub_;
    std::unique_ptr<eCAL::protobuf::CPublisher<foxglove::MarkerArray>> markers_pub_;
#else
    // Placeholder members for when eCAL is not available
    void* planning_result_pub_;
    void* planning_status_pub_;
    void* obstacle_map_pub_;
    void* path_pub_;
    void* markers_pub_;
#endif
    
    /**
     * @brief Get current timestamp in Foxglove format
     */
    void* get_current_time() const;  // Return void* when protobuf not available
    
    /**
     * @brief Convert State to VehicleState protobuf message
     */
    void* state_to_proto(const State& state) const;
    
    /**
     * @brief Convert planning node to protobuf message
     */
    void* node_to_proto(const std::shared_ptr<Node>& node) const;
    
    /**
     * @brief Create obstacle map grid message
     */
    void* create_obstacle_map_grid(
        const std::vector<std::vector<int>>& obstacle_map,
        double origin_x, double origin_y, double grid_resolution) const;
    
    /**
     * @brief Create path visualization message
     */
    void* create_path_message(const std::vector<State>& states) const;
    
    /**
     * @brief Create exploration markers
     */
    void* create_exploration_markers(
        const std::vector<std::shared_ptr<Node>>& explored_nodes) const;
    
    /**
     * @brief Create vehicle markers for start and goal
     */
    void* create_vehicle_markers(
        const State& start_state, const State& goal_state) const;
    
    /**
     * @brief Convert yaw angle to quaternion
     */
    void* yaw_to_quaternion(double yaw) const;
};

} // namespace hybrid_astar
