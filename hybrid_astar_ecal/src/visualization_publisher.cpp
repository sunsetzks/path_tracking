/**
 * @file visualization_publisher.cpp
 * @brief Implementation of eCAL-based visualization publisher using Foxglove SceneUpdate
 */

#include "visualization_publisher.hpp"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <limits>
#include <ctime>
#include <chrono>

namespace hybrid_astar {

VisualizationPublisher::VisualizationPublisher(const std::string& node_name)
    : node_name_(node_name), initialized_(false), settings_() {
}

VisualizationPublisher::~VisualizationPublisher() {
    shutdown();
}

bool VisualizationPublisher::initialize() {
    try {
        // Initialize eCAL
        eCAL::Initialize(0, nullptr, node_name_.c_str());
        
        // Create separate publishers for different visualization channels
        scene_pub_ = std::make_unique<eCAL::protobuf::CPublisher<foxglove::SceneUpdate>>("/hybrid_astar/visualization/scene");
        path_pub_ = std::make_unique<eCAL::protobuf::CPublisher<foxglove::SceneUpdate>>("/hybrid_astar/visualization/path");
        exploration_pub_ = std::make_unique<eCAL::protobuf::CPublisher<foxglove::SceneUpdate>>("/hybrid_astar/visualization/exploration");
        start_goal_pub_ = std::make_unique<eCAL::protobuf::CPublisher<foxglove::SceneUpdate>>("/hybrid_astar/visualization/start_goal");
        statistics_pub_ = std::make_unique<eCAL::string::CPublisher<std::string>>("/hybrid_astar/visualization/statistics");
        
        initialized_ = true;
        std::cout << "✓ eCAL SceneUpdate visualization publisher initialized successfully" << std::endl;
        std::cout << "→ Channels created:" << std::endl;
        std::cout << "  - /hybrid_astar/visualization/scene (obstacles)" << std::endl;
        std::cout << "  - /hybrid_astar/visualization/path (final path)" << std::endl;
        std::cout << "  - /hybrid_astar/visualization/exploration (search nodes)" << std::endl;
        std::cout << "  - /hybrid_astar/visualization/start_goal (start/goal positions)" << std::endl;
        std::cout << "  - /hybrid_astar/visualization/statistics (JSON data)" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Failed to initialize eCAL SceneUpdate publisher: " << e.what() << std::endl;
        return false;
    }
}
void VisualizationPublisher::shutdown() {
    if (initialized_) {
        scene_pub_.reset();
        path_pub_.reset();
        exploration_pub_.reset();
        start_goal_pub_.reset();
        statistics_pub_.reset();
        
        eCAL::Finalize();
        initialized_ = false;
        std::cout << "eCAL SceneUpdate visualization publisher shutdown" << std::endl;
    }
}

void VisualizationPublisher::visualize_path_planning(
    const State& start_state,
    const State& goal_state,
    const std::optional<std::vector<std::shared_ptr<Node>>>& path_nodes,
    const std::vector<std::shared_ptr<Node>>& explored_nodes,
    const std::vector<State>& detailed_path,
    const std::vector<std::vector<State>>& simulation_trajectories,
    const std::vector<std::vector<int>>& obstacle_map,
    double map_origin_x, double map_origin_y, double grid_resolution,
    double planning_time_ms) {
    
    if (!initialized_) {
        std::cerr << "Publisher not initialized" << std::endl;
        return;
    }
    
    try {
        // Create and publish separate scene updates
        auto obstacle_scene = create_scene_update(obstacle_map, map_origin_x, map_origin_y, grid_resolution);
        scene_pub_->Send(obstacle_scene);
        
        auto start_goal_scene = create_start_goal_scene_update(start_state, goal_state);
        start_goal_pub_->Send(start_goal_scene);
        
        auto path_scene = create_path_scene_update(detailed_path);
        path_pub_->Send(path_scene);
        
        auto exploration_scene = create_exploration_scene_update(explored_nodes, simulation_trajectories);
        exploration_pub_->Send(exploration_scene);
        
        auto statistics = create_statistics(detailed_path, explored_nodes, planning_time_ms);
        statistics_pub_->Send(statistics);
        
        std::cout << "Published complete path planning visualization" << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Error publishing visualization: " << e.what() << std::endl;
    }
}

void VisualizationPublisher::publish_planning_status(
    int status,
    const std::string& message,
    int current_iteration,
    int max_iterations) {
    
    if (!initialized_) return;
    
    // Create status as JSON
    std::ostringstream json_stream;
    json_stream << "{"
                << "\"status\": " << status << ","
                << "\"message\": \"" << message << "\","
                << "\"current_iteration\": " << current_iteration << ","
                << "\"max_iterations\": " << max_iterations << ","
                << "\"timestamp\": " << std::time(nullptr)
                << "}";
    
    statistics_pub_->Send(json_stream.str());
    std::cout << "Published planning status with eCAL" << std::endl;
}

// Implementation of SceneUpdate creation methods

foxglove::SceneUpdate VisualizationPublisher::create_scene_update(
    const std::vector<std::vector<int>>& obstacle_map,
    double map_origin_x, double map_origin_y, double grid_resolution) {
    
    foxglove::SceneUpdate scene_update;
    
    if (!obstacle_map.empty()) {
        auto entity = scene_update.add_entities();
        entity->set_id("obstacles");
        entity->set_frame_id("map");
        *entity->mutable_timestamp() = get_current_timestamp();
        
        int height = obstacle_map.size();
        int width = obstacle_map[0].size();
        
        // Create cubes for obstacles
        for (int y = 0; y < height; ++y) {
            for (int x = 0; x < width; ++x) {
                if (obstacle_map[y][x] > 0) {
                    auto cube = entity->add_cubes();
                    
                    double world_x = map_origin_x + x * grid_resolution;
                    double world_y = map_origin_y + y * grid_resolution;
                    
                    *cube->mutable_pose() = create_pose(world_x, world_y, 0.5, 0.0);
                    *cube->mutable_size() = create_vector3(grid_resolution, grid_resolution, 1.0);
                    *cube->mutable_color() = create_color(0.5, 0.5, 0.5, 0.8);
                }
            }
        }
    }
    
    return scene_update;
}

foxglove::SceneUpdate VisualizationPublisher::create_start_goal_scene_update(
    const State& start, const State& goal) {
    
    foxglove::SceneUpdate scene_update;
    auto entity = scene_update.add_entities();
    entity->set_id("start_goal");
    entity->set_frame_id("map");
    *entity->mutable_timestamp() = get_current_timestamp();
    
    // Start position as green sphere
    auto start_sphere = entity->add_spheres();
    *start_sphere->mutable_pose() = create_pose(start.x, start.y, 0.0, 0.0);
    *start_sphere->mutable_size() = create_vector3(0.5, 0.5, 0.5);
    *start_sphere->mutable_color() = create_color(0.0, 1.0, 0.0, settings_.start_goal_alpha);
    
    // Start orientation arrow
    auto start_arrow = entity->add_arrows();
    *start_arrow->mutable_pose() = create_pose(start.x, start.y, 0.0, start.yaw);
    start_arrow->set_shaft_length(1.2);
    start_arrow->set_shaft_diameter(0.1);
    start_arrow->set_head_length(0.3);
    start_arrow->set_head_diameter(0.2);
    *start_arrow->mutable_color() = create_color(0.0, 0.8, 0.0, settings_.start_goal_alpha);
    
    // Goal position as red sphere
    auto goal_sphere = entity->add_spheres();
    *goal_sphere->mutable_pose() = create_pose(goal.x, goal.y, 0.0, 0.0);
    *goal_sphere->mutable_size() = create_vector3(0.5, 0.5, 0.5);
    *goal_sphere->mutable_color() = create_color(1.0, 0.0, 0.0, settings_.start_goal_alpha);
    
    // Goal orientation arrow
    auto goal_arrow = entity->add_arrows();
    *goal_arrow->mutable_pose() = create_pose(goal.x, goal.y, 0.0, goal.yaw);
    goal_arrow->set_shaft_length(1.2);
    goal_arrow->set_shaft_diameter(0.1);
    goal_arrow->set_head_length(0.3);
    goal_arrow->set_head_diameter(0.2);
    *goal_arrow->mutable_color() = create_color(0.8, 0.0, 0.0, settings_.start_goal_alpha);
    
    return scene_update;
}

foxglove::SceneUpdate VisualizationPublisher::create_path_scene_update(
    const std::vector<State>& path) {
    
    foxglove::SceneUpdate scene_update;
    
    if (path.size() > 1) {
        auto entity = scene_update.add_entities();
        entity->set_id("final_path");
        entity->set_frame_id("map");
        *entity->mutable_timestamp() = get_current_timestamp();
        
        // Helper function to get segment color
        auto get_color = [this](DirectionMode direction) -> foxglove::Color {
            if (direction == DirectionMode::FORWARD) {
                return create_color(0.0, 1.0, 0.0, settings_.path_alpha);  // Green
            } else if (direction == DirectionMode::BACKWARD) {
                return create_color(1.0, 0.0, 0.0, settings_.path_alpha);  // Red
            }
            return create_color(0.5, 0.5, 0.5, settings_.path_alpha);  // Gray for unknown
        };
        
        // Create path segments by direction (skip NONE directions)
        std::vector<foxglove::Point3> current_segment;
        DirectionMode current_direction = DirectionMode::NONE;
        
        for (const auto& state : path) {
            // Skip NONE directions
            if (state.direction == DirectionMode::NONE) continue;
            
            foxglove::Point3 point = create_point3(state.x, state.y, 0.0);
            
            // Start new segment if direction changes
            if (state.direction != current_direction) {
                // Save previous segment if valid
                if (current_segment.size() > 1) {
                    auto line = entity->add_lines();
                    *line = create_line(current_segment, settings_.path_line_thickness, get_color(current_direction));
                }
                
                // Start new segment - if we have a previous segment, add the last point to ensure continuity
                if (!current_segment.empty()) {
                    current_segment = {current_segment.back(), point};  // Add connecting point
                } else {
                    current_segment = {point};
                }
                current_direction = state.direction;
            } else {
                current_segment.push_back(point);
            }
        }
        
        // Save final segment
        if (current_segment.size() > 1) {
            auto line = entity->add_lines();
            *line = create_line(current_segment, settings_.path_line_thickness, get_color(current_direction));
        }
        
        // Add orientation arrows if enabled
        if (settings_.show_final_path_arrows) {
            int step = std::max(1, static_cast<int>(path.size()) / 20);
            for (size_t i = 0; i < path.size(); i += step) {
                if (path[i].direction == DirectionMode::NONE) continue;  // Skip NONE directions
                
                auto arrow = entity->add_arrows();
                *arrow->mutable_pose() = create_pose(path[i].x, path[i].y, 0.0, path[i].yaw);
                arrow->set_shaft_length(0.8);
                arrow->set_shaft_diameter(0.05);
                arrow->set_head_length(0.2);
                arrow->set_head_diameter(0.1);
                *arrow->mutable_color() = get_color(path[i].direction);
            }
        }
    }
    
    return scene_update;
}

foxglove::SceneUpdate VisualizationPublisher::create_exploration_scene_update(
    const std::vector<std::shared_ptr<Node>>& explored_nodes,
    const std::vector<std::vector<State>>& simulation_trajectories) {
    
    foxglove::SceneUpdate scene_update;
    auto entity = scene_update.add_entities();
    entity->set_id("exploration");
    entity->set_frame_id("map");
    *entity->mutable_timestamp() = get_current_timestamp();
    
    // Limit number of nodes to avoid overwhelming visualization
    size_t max_nodes = std::min(static_cast<size_t>(settings_.max_exploration_nodes), explored_nodes.size());
    
    // Visualization for exploration nodes as spheres (colored by cost)
    if (!explored_nodes.empty()) {
        // Find min/max costs for color mapping
        double min_cost = std::numeric_limits<double>::max();
        double max_cost = std::numeric_limits<double>::min();
        
        for (size_t i = 0; i < max_nodes; ++i) {
            min_cost = std::min(min_cost, explored_nodes[i]->f_cost());
            max_cost = std::max(max_cost, explored_nodes[i]->f_cost());
        }
        
        double cost_range = max_cost - min_cost;
        if (cost_range < 1e-6) cost_range = 1.0; // Avoid division by zero
        
        for (size_t i = 0; i < max_nodes; ++i) {
            auto sphere = entity->add_spheres();
            const auto& node = explored_nodes[i];
            
            *sphere->mutable_pose() = create_pose(node->state.x, node->state.y, 0.0, 0.0);
            *sphere->mutable_size() = create_vector3(settings_.exploration_sphere_size, 
                                                    settings_.exploration_sphere_size, 
                                                    settings_.exploration_sphere_size);
            
            // Color by cost: blue (low cost) to red (high cost)
            double normalized_cost = (node->f_cost() - min_cost) / cost_range;
            *sphere->mutable_color() = create_color(normalized_cost, 0.0, 1.0 - normalized_cost, 0.6);
        }
    }
    
    // Visualization for each node's forward simulation trajectory as lines
    if (settings_.show_node_forward_trajectories) {
        for (size_t i = 0; i < max_nodes; ++i) {
            const auto& node = explored_nodes[i];
            const auto& trajectory = node->forward_simulation_trajectory;
            
            if (trajectory.size() > 1) {
                // Convert trajectory to points
                std::vector<foxglove::Point3> points;
                for (const auto& state : trajectory) {
                    points.push_back(create_point3(state.x, state.y, 0.0));
                }
                
                // Determine color based on the first point's direction (entire trajectory has same direction)
                foxglove::Color trajectory_color;
                if (trajectory[0].direction == DirectionMode::FORWARD) {
                    trajectory_color = create_color(0.0, 0.8, 0.0, 0.3);  // Green for forward
                } else if (trajectory[0].direction == DirectionMode::BACKWARD) {
                    trajectory_color = create_color(0.8, 0.0, 0.0, 0.3);  // Red for backward
                } else {
                    trajectory_color = create_color(0.5, 0.5, 0.5, 0.3);  // Gray for unknown
                }
                
                // Create the line for the entire trajectory
                auto line = entity->add_lines();
                *line = create_line(points, settings_.exploration_line_thickness * 0.8, trajectory_color);
            }
        }
    }
    
    // Also visualize the original simulation trajectories if available (for backward compatibility)
    // These are shown in blue with lower opacity
    for (size_t i = 0; i < std::min(simulation_trajectories.size(), max_nodes); ++i) {
        const auto& trajectory = simulation_trajectories[i];
        if (trajectory.size() > 1) {
            std::vector<foxglove::Point3> points;
            for (const auto& state : trajectory) {
                points.push_back(create_point3(state.x, state.y, 0.0));
            }
            
            auto line = entity->add_lines();
            *line = create_line(points, settings_.exploration_line_thickness * 0.6,
                               create_color(0.0, 0.5, 1.0, 0.2));
        }
    }
    
    return scene_update;
}

std::string VisualizationPublisher::create_statistics(
    const std::vector<State>& path,
    const std::vector<std::shared_ptr<Node>>& explored_nodes,
    double planning_time_ms) {
    
    // Calculate path statistics
    double total_distance = 0.0;
    double max_steering_angle = 0.0;
    double avg_steering_angle = 0.0;
    int direction_changes = 0;
    
    if (path.size() > 1) {
        for (size_t i = 1; i < path.size(); ++i) {
            double dx = path[i].x - path[i-1].x;
            double dy = path[i].y - path[i-1].y;
            total_distance += std::sqrt(dx*dx + dy*dy);
            
            // Assuming we have steering angle in the state (if available)
            // This would need to be adapted based on actual State structure
            if (path[i].steer != 0.0) {
                max_steering_angle = std::max(max_steering_angle, std::abs(path[i].steer));
                avg_steering_angle += std::abs(path[i].steer);
            }
        }
        avg_steering_angle /= path.size();
    }
    
    double nodes_per_second = planning_time_ms > 0 ? (explored_nodes.size() * 1000.0) / planning_time_ms : 0.0;
    
    // Create JSON statistics
    std::ostringstream json_stream;
    json_stream << std::fixed << std::setprecision(3);
    json_stream << "{"
                << "\"timestamp\": " << std::time(nullptr) << ","
                << "\"path_length\": " << path.size() << ","
                << "\"nodes_explored\": " << explored_nodes.size() << ","
                << "\"total_distance\": " << total_distance << ","
                << "\"max_steering_angle\": " << max_steering_angle << ","
                << "\"avg_steering_angle\": " << avg_steering_angle << ","
                << "\"direction_changes\": " << direction_changes << ","
                << "\"search_time_seconds\": " << (planning_time_ms / 1000.0) << ","
                << "\"nodes_per_second\": " << nodes_per_second
                << "}";
    
    return json_stream.str();
}

// Helper function implementations

google::protobuf::Timestamp VisualizationPublisher::get_current_timestamp() const {
    google::protobuf::Timestamp timestamp;
    auto now = std::chrono::system_clock::now();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
    auto nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch() % std::chrono::seconds(1)).count();
    
    timestamp.set_seconds(seconds);
    timestamp.set_nanos(static_cast<int32_t>(nanos));
    return timestamp;
}

foxglove::Color VisualizationPublisher::create_color(double r, double g, double b, double a) const {
    foxglove::Color color;
    color.set_r(r);
    color.set_g(g);
    color.set_b(b);
    color.set_a(a);
    return color;
}

foxglove::Point3 VisualizationPublisher::create_point3(double x, double y, double z) const {
    foxglove::Point3 point;
    point.set_x(x);
    point.set_y(y);
    point.set_z(z);
    return point;
}

foxglove::Vector3 VisualizationPublisher::create_vector3(double x, double y, double z) const {
    foxglove::Vector3 vector;
    vector.set_x(x);
    vector.set_y(y);
    vector.set_z(z);
    return vector;
}

foxglove::Quaternion VisualizationPublisher::yaw_to_quaternion(double yaw) const {
    foxglove::Quaternion quat;
    double half_yaw = yaw * 0.5;
    quat.set_x(0.0);
    quat.set_y(0.0);
    quat.set_z(std::sin(half_yaw));
    quat.set_w(std::cos(half_yaw));
    return quat;
}

foxglove::Pose VisualizationPublisher::create_pose(double x, double y, double z, double yaw) const {
    foxglove::Pose pose;
    auto position = pose.mutable_position();
    position->set_x(x);
    position->set_y(y);
    position->set_z(z);
    
    auto orientation = pose.mutable_orientation();
    auto quat = yaw_to_quaternion(yaw);
    orientation->set_x(quat.x());
    orientation->set_y(quat.y());
    orientation->set_z(quat.z());
    orientation->set_w(quat.w());
    
    return pose;
}

foxglove::ArrowPrimitive VisualizationPublisher::create_arrow(
    const foxglove::Pose& pose, 
    double shaft_length, double shaft_diameter,
    double head_length, double head_diameter,
    const foxglove::Color& color) const {
    
    foxglove::ArrowPrimitive arrow;
    *arrow.mutable_pose() = pose;
    arrow.set_shaft_length(shaft_length);
    arrow.set_shaft_diameter(shaft_diameter);
    arrow.set_head_length(head_length);
    arrow.set_head_diameter(head_diameter);
    *arrow.mutable_color() = color;
    return arrow;
}

foxglove::SpherePrimitive VisualizationPublisher::create_sphere(
    const foxglove::Pose& pose,
    const foxglove::Vector3& size,
    const foxglove::Color& color) const {
    
    foxglove::SpherePrimitive sphere;
    *sphere.mutable_pose() = pose;
    *sphere.mutable_size() = size;
    *sphere.mutable_color() = color;
    return sphere;
}

foxglove::LinePrimitive VisualizationPublisher::create_line(
    const std::vector<foxglove::Point3>& points,
    double thickness,
    const foxglove::Color& color,
    foxglove::LinePrimitive::Type type) const {
    
    foxglove::LinePrimitive line;
    line.set_type(type);
    line.set_thickness(thickness);
    *line.mutable_color() = color;
    
    // Set pose to origin since points are in world coordinates
    *line.mutable_pose() = create_pose(0, 0, 0, 0);
    
    for (const auto& point : points) {
        *line.add_points() = point;
    }
    
    return line;
}

foxglove::CubePrimitive VisualizationPublisher::create_cube(
    const foxglove::Pose& pose,
    const foxglove::Vector3& size,
    const foxglove::Color& color) const {
    
    foxglove::CubePrimitive cube;
    *cube.mutable_pose() = pose;
    *cube.mutable_size() = size;
    *cube.mutable_color() = color;
    return cube;
}

} // namespace hybrid_astar
