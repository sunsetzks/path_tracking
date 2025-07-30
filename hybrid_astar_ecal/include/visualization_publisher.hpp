/**
 * @file visualization_publisher.hpp
 * @brief eCAL-based visualization publisher for Hybrid A* planning using Foxglove SceneUpdate
 */

#pragma once

#include "common_types.hpp"
#include <memory>
#include <string>
#include <optional>
#include <vector>
#include <cmath>

// eCAL and Foxglove protobuf includes
#include "foxglove/SceneUpdate.pb.h"
#include "foxglove/LinePrimitive.pb.h"
#include "foxglove/SpherePrimitive.pb.h"
#include "foxglove/ArrowPrimitive.pb.h"
#include "foxglove/CubePrimitive.pb.h"
#include "foxglove/Color.pb.h"
#include "foxglove/Point3.pb.h"
#include "foxglove/Vector3.pb.h"
#include "foxglove/Pose.pb.h"
#include "foxglove/Quaternion.pb.h"
#include <ecal/ecal.h>
#include <ecal/msg/protobuf/publisher.h>
#include <ecal/msg/string/publisher.h>
#include <google/protobuf/timestamp.pb.h>
#include <google/protobuf/duration.pb.h>

namespace hybrid_astar {

/**
 * @brief Publisher for Hybrid A* visualization data using eCAL and Foxglove SceneUpdate
 * 
 * This class provides real-time visualization capabilities similar to the Python FoxgloveHybridAStarVisualizer
 * using SceneUpdate proto messages with separate channels for different visualization components.
 * 
 * Visualization Channels:
 * - /hybrid_astar/visualization/scene: Obstacles visualization
 * - /hybrid_astar/visualization/path: Final planned path with vehicle orientation arrows
 * - /hybrid_astar/visualization/exploration: Exploration nodes and simulation trajectories
 * - /hybrid_astar/visualization/start_goal: Start and goal positions with orientation arrows
 * - /hybrid_astar/visualization/statistics: Path planning statistics (JSON)
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
     * @brief Visualize complete path planning results with separate SceneUpdate messages
     * @param start_state Start state
     * @param goal_state Goal state
     * @param path_nodes Optional planning result nodes
     * @param explored_nodes Explored nodes during planning
     * @param detailed_path Detailed state sequence
     * @param simulation_trajectories Forward simulation trajectories
     * @param obstacle_map 2D obstacle map
     * @param map_origin_x Map origin X coordinate
     * @param map_origin_y Map origin Y coordinate
     * @param grid_resolution Grid resolution
     * @param planning_time_ms Planning time in milliseconds
     */
    void visualize_path_planning(
        const State& start_state,
        const State& goal_state,
        const std::optional<std::vector<std::shared_ptr<Node>>>& path_nodes,
        const std::vector<std::shared_ptr<Node>>& explored_nodes,
        const std::vector<State>& detailed_path,
        const std::vector<std::vector<State>>& simulation_trajectories,
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
        int status,
        const std::string& message,
        int current_iteration = 0,
        int max_iterations = 0);
    
    /**
     * @brief Settings for visualization
     */
    struct VisualizationSettings {
        double path_line_thickness = 0.05;
        double path_alpha = 0.5;
        int max_exploration_nodes = 100000;
        double exploration_sphere_size = 0.03;
        double exploration_line_thickness = 0.01;
        bool show_final_path_arrows = false;
        bool show_node_forward_trajectories = true;  // New option to show forward simulation trajectories
    };
    
    /**
     * @brief Update visualization settings
     */
    void update_settings(const VisualizationSettings& settings) { settings_ = settings; }

private:
    std::string node_name_;
    bool initialized_;
    VisualizationSettings settings_;
    
    // eCAL publishers for separate visualization channels
    std::unique_ptr<eCAL::protobuf::CPublisher<foxglove::SceneUpdate>> scene_pub_;           // Obstacles
    std::unique_ptr<eCAL::protobuf::CPublisher<foxglove::SceneUpdate>> path_pub_;            // Final path
    std::unique_ptr<eCAL::protobuf::CPublisher<foxglove::SceneUpdate>> exploration_pub_;     // Exploration nodes
    std::unique_ptr<eCAL::protobuf::CPublisher<foxglove::SceneUpdate>> start_goal_pub_;      // Start/goal positions
    std::unique_ptr<eCAL::string::CPublisher<std::string>> statistics_pub_;                  // JSON statistics
    
    /**
     * @brief Create SceneUpdate for obstacles visualization
     */
    foxglove::SceneUpdate create_scene_update(const std::vector<std::vector<int>>& obstacle_map,
                                             double map_origin_x, double map_origin_y, double grid_resolution);
    
    /**
     * @brief Create SceneUpdate for start and goal position visualization
     */
    foxglove::SceneUpdate create_start_goal_scene_update(const State& start, const State& goal);
    
    /**
     * @brief Create SceneUpdate for path visualization
     */
    foxglove::SceneUpdate create_path_scene_update(const std::vector<State>& path);
    
    /**
     * @brief Create SceneUpdate for exploration nodes visualization
     */
    foxglove::SceneUpdate create_exploration_scene_update(
        const std::vector<std::shared_ptr<Node>>& explored_nodes,
        const std::vector<std::vector<State>>& simulation_trajectories);
    
    /**
     * @brief Create statistics data as JSON string
     */
    std::string create_statistics(const std::vector<State>& path,
                                 const std::vector<std::shared_ptr<Node>>& explored_nodes,
                                 double planning_time_ms);
    
    /**
     * @brief Helper functions for creating Foxglove primitives
     */
    google::protobuf::Timestamp get_current_timestamp() const;
    
    foxglove::Color create_color(double r, double g, double b, double a = 1.0) const;
    
    foxglove::Point3 create_point3(double x, double y, double z = 0.0) const;
    
    foxglove::Vector3 create_vector3(double x, double y, double z) const;
    
    foxglove::Quaternion yaw_to_quaternion(double yaw) const;
    
    foxglove::Pose create_pose(double x, double y, double z, double yaw) const;
    
    foxglove::ArrowPrimitive create_arrow(const foxglove::Pose& pose, 
                                         double shaft_length, double shaft_diameter,
                                         double head_length, double head_diameter,
                                         const foxglove::Color& color) const;
    
    foxglove::SpherePrimitive create_sphere(const foxglove::Pose& pose,
                                           const foxglove::Vector3& size,
                                           const foxglove::Color& color) const;
    
    foxglove::LinePrimitive create_line(const std::vector<foxglove::Point3>& points,
                                       double thickness,
                                       const foxglove::Color& color,
                                       foxglove::LinePrimitive::Type type = foxglove::LinePrimitive::LINE_STRIP) const;
    
    foxglove::CubePrimitive create_cube(const foxglove::Pose& pose,
                                       const foxglove::Vector3& size,
                                       const foxglove::Color& color) const;
};

} // namespace hybrid_astar
