/**
 * @file hybrid_astar.hpp
 * @brief Hybrid A* Path Planning Algorithm
 * 
 * Implementation with steering angle cost, turning cost, cusp cost, 
 * and forward simulation considering steering angle velocity.
 * 
 * @author Converted from Python implementation
 * @date 2025-07-29
 */

#ifndef HYBRID_ASTAR_HPP
#define HYBRID_ASTAR_HPP

#define _USE_MATH_DEFINES
#include <vector>
#include <memory>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <cmath>
#include <optional>
#include <functional>
#include <tuple>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace hybrid_astar {

/**
 * @brief Vehicle direction mode
 */
enum class DirectionMode {
    FORWARD = 1,
    BACKWARD = -1,
    NONE = 0
};

/**
 * @brief Cost components for path planning
 */
struct Costs {
    double distance = 0.0;  ///< Distance traveled cost
    double steer = 0.0;     ///< Steering angle cost
    double turn = 0.0;      ///< Turning cost
    double cusp = 0.0;      ///< Cusp (direction change) cost
    
    Costs() = default;
    Costs(double d, double s, double t, double c) 
        : distance(d), steer(s), turn(t), cusp(c) {}
};

/**
 * @brief Vehicle state representation
 */
struct State {
    double x = 0.0;         ///< X position
    double y = 0.0;         ///< Y position
    double yaw = 0.0;       ///< Heading angle in radians
    DirectionMode direction = DirectionMode::NONE;  ///< Direction mode
    double steer = 0.0;     ///< Steering angle in radians
    
    State() = default;
    State(double x_, double y_, double yaw_, DirectionMode dir = DirectionMode::NONE, double steer_ = 0.0)
        : x(x_), y(y_), yaw(yaw_), direction(dir), steer(steer_) {}
    
    bool operator==(const State& other) const;
    bool operator!=(const State& other) const { return !(*this == other); }
};

/**
 * @brief Hash function for State to use in unordered containers
 */
struct StateHash {
    std::size_t operator()(const State& state) const;
};

// Forward declaration
class Node;

/**
 * @brief A* search node
 */
class Node {
public:
    State state;                    ///< Vehicle state
    double g_cost = 0.0;           ///< Cost from start
    double h_cost = 0.0;           ///< Heuristic cost to goal
    std::shared_ptr<Node> parent = nullptr;  ///< Parent node
    Costs costs;                   ///< Cost components
    std::vector<State> forward_simulation_trajectory;  ///< Forward simulation trajectory
    
    Node() = default;
    Node(const State& s) : state(s) {}
    Node(const State& s, double g, std::shared_ptr<Node> p = nullptr)
        : state(s), g_cost(g), parent(p) {}
    
    double f_cost() const { return g_cost + h_cost; }
    
    // Backward compatibility properties
    double steer_cost() const { return costs.steer; }
    double turn_cost() const { return costs.turn; }
    double cusp_cost() const { return costs.cusp; }
};

/**
 * @brief Comparison function for priority queue (min-heap)
 */
struct NodeComparator {
    bool operator()(const std::shared_ptr<Node>& a, const std::shared_ptr<Node>& b) const {
        return a->f_cost() > b->f_cost();  // Min-heap: lower f_cost has higher priority
    }
};

/**
 * @brief Simple bicycle model for vehicle simulation
 */
class VehicleModel {
private:
    double wheelbase_;     ///< Distance between front and rear axles (m)
    double max_steer_;     ///< Maximum steering angle (rad)

public:
    explicit VehicleModel(double wheelbase = 2.5, double max_steer = M_PI/4);
    
    /**
     * @brief Forward simulate vehicle motion
     * @param state Initial state
     * @param velocity Linear velocity (m/s)
     * @param steer_rate Steering angle rate (rad/s)
     * @param dt Time step (s)
     * @param steps Number of simulation steps
     * @return Vector of simulated states
     */
    std::vector<State> simulate_motion(const State& state, double velocity, 
                                     double steer_rate, double dt, int steps = 1) const;
    
    /**
     * @brief Normalize angle to [-pi, pi]
     */
    static double normalize_angle(double angle);
    
    // Getters
    double wheelbase() const { return wheelbase_; }
    double max_steer() const { return max_steer_; }
};

/**
 * @brief Hybrid A* path planning algorithm
 */
class HybridAStar {
private:
    VehicleModel vehicle_model_;
    double grid_resolution_;
    double angle_resolution_;
    double steer_resolution_;
    double velocity_;
    double simulation_time_;
    double dt_;
    int simulation_steps_;
    
    // Cost weights
    double w_steer_ = 10.0;
    double w_turn_ = 15.0;
    double w_cusp_ = 10.0;
    double w_obstacle_ = 1000.0;
    
    // Steering angle rates for motion primitives (rad/s)
    std::vector<double> steer_rates_;
    
    // Obstacle map
    std::vector<std::vector<int>> obstacle_map_;
    int map_width_ = 0;
    int map_height_ = 0;
    double map_origin_x_ = 0.0;
    double map_origin_y_ = 0.0;
    
    // Visualization data
    std::vector<std::shared_ptr<Node>> explored_nodes_;
    std::vector<std::vector<State>> simulation_trajectories_;

public:
    /**
     * @brief Constructor
     */
    HybridAStar(const VehicleModel& vehicle_model,
                double grid_resolution = 1.0,
                double angle_resolution = M_PI/8,
                double steer_resolution = M_PI/16,
                double velocity = 2.0,
                double simulation_time = 1.0,
                double dt = 0.1);
    
    /**
     * @brief Set obstacle map
     */
    void set_obstacle_map(const std::vector<std::vector<int>>& obstacle_map,
                         double origin_x = 0.0, double origin_y = 0.0);
    
    /**
     * @brief Check if state is collision-free
     */
    bool is_collision_free(const State& state) const;
    
    /**
     * @brief Calculate heuristic cost
     */
    double heuristic_cost(const State& state, const State& goal) const;
    
    /**
     * @brief Plan path using Hybrid A* algorithm
     */
    std::optional<std::vector<std::shared_ptr<Node>>> plan_path(
        const State& start, const State& goal, int max_iterations = 10000);
    
    /**
     * @brief Extract detailed path states using simulation trajectories
     */
    std::vector<State> extract_detailed_path(const std::vector<std::shared_ptr<Node>>& path_nodes) const;
    
    /**
     * @brief Get path and search statistics
     */
    std::unordered_map<std::string, double> get_statistics(
        const std::optional<std::vector<std::shared_ptr<Node>>>& path) const;
    
    // Getters for visualization data
    const std::vector<std::shared_ptr<Node>>& get_explored_nodes() const { return explored_nodes_; }
    const std::vector<std::vector<State>>& get_simulation_trajectories() const { return simulation_trajectories_; }
    const std::vector<std::vector<int>>& get_obstacle_map() const { return obstacle_map_; }
    double get_map_origin_x() const { return map_origin_x_; }
    double get_map_origin_y() const { return map_origin_y_; }
    double get_grid_resolution() const { return grid_resolution_; }
    const VehicleModel& get_vehicle_model() const { return vehicle_model_; }

private:
    /**
     * @brief Calculate cost based on steering angle magnitude
     */
    double calculate_steering_cost(double steer_angle) const;
    
    /**
     * @brief Calculate cost based on turning (yaw change)
     */
    double calculate_turning_cost(double prev_yaw, double current_yaw) const;
    
    /**
     * @brief Calculate cost for direction changes (cusps)
     */
    double calculate_cusp_cost(DirectionMode prev_direction, DirectionMode current_direction) const;
    
    /**
     * @brief Calculate all cost components for a given successor state
     */
    std::pair<double, Costs> calculate_total_cost(const std::shared_ptr<Node>& node,
                                                 const State& final_state,
                                                 DirectionMode direction) const;
    
    /**
     * @brief Generate successor nodes using motion primitives
     */
    std::vector<std::shared_ptr<Node>> get_successors(const std::shared_ptr<Node>& node);
    
    /**
     * @brief Discretize continuous state for duplicate detection
     */
    std::tuple<int, int, int, int> discretize_state(const State& state) const;
    
    /**
     * @brief Check if goal is reached within tolerances
     */
    bool is_goal_reached(const State& current, const State& goal,
                        double position_tolerance = 1.0,
                        double angle_tolerance = M_PI/6) const;
    
    /**
     * @brief Reconstruct path from goal node to start
     */
    std::vector<std::shared_ptr<Node>> reconstruct_path(const std::shared_ptr<Node>& goal_node) const;
};

/**
 * @brief Hash function for discretized state tuple
 */
struct StateKeyHash {
    std::size_t operator()(const std::tuple<int, int, int, int>& key) const {
        auto h1 = std::hash<int>{}(std::get<0>(key));
        auto h2 = std::hash<int>{}(std::get<1>(key));
        auto h3 = std::hash<int>{}(std::get<2>(key));
        auto h4 = std::hash<int>{}(std::get<3>(key));
        return h1 ^ (h2 << 1) ^ (h3 << 2) ^ (h4 << 3);
    }
};

} // namespace hybrid_astar

#endif // HYBRID_ASTAR_HPP
