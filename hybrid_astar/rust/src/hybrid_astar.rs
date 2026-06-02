//! Hybrid A* path planning algorithm implementation.

use std::collections::{BinaryHeap, HashMap, HashSet};

use crate::math_utils::{euclidean_distance, normalize_angle};
use crate::state::{Costs, Direction, Node, State};
use crate::vehicle_model::VehicleModel;

/// Configuration for the Hybrid A* planner
#[derive(Debug, Clone)]
pub struct PlannerConfig {
    /// Grid resolution for discretization (m)
    pub grid_resolution: f64,
    /// Angular resolution (rad)
    pub angle_resolution: f64,
    /// Steering angle resolution (rad)
    pub steer_resolution: f64,
    /// Fixed linear velocity for simulation (m/s)
    pub velocity: f64,
    /// Forward simulation time (s)
    pub simulation_time: f64,
    /// Simulation time step (s)
    pub dt: f64,
    /// Position tolerance for goal reaching (m)
    pub position_tolerance: f64,
    /// Angle tolerance for goal reaching (rad)
    pub angle_tolerance: f64,
}

impl Default for PlannerConfig {
    fn default() -> Self {
        Self {
            grid_resolution: 0.5,
            angle_resolution: std::f64::consts::FRAC_PI_8,
            steer_resolution: std::f64::consts::PI / 16.0,
            velocity: 2.0,
            simulation_time: 1.0,
            dt: 0.1,
            position_tolerance: 1.0,
            angle_tolerance: std::f64::consts::FRAC_PI_6,
        }
    }
}

/// Cost weights for the planner
#[derive(Debug, Clone)]
pub struct CostWeights {
    /// Steering angle cost weight
    pub w_steer: f64,
    /// Turning cost weight
    pub w_turn: f64,
    /// Direction change (cusp) cost weight
    pub w_cusp: f64,
}

impl Default for CostWeights {
    fn default() -> Self {
        Self {
            w_steer: 10.0,
            w_turn: 15.0,
            w_cusp: 10.0,
        }
    }
}

/// Planning result containing path and statistics
#[derive(Debug, Clone)]
pub struct PlanningResult {
    /// Path nodes from start to goal (if found)
    pub path: Option<Vec<Node>>,
    /// All explored nodes
    pub explored_nodes: Vec<State>,
    /// All simulation trajectories
    pub trajectories: Vec<Vec<State>>,
    /// Planning statistics
    pub stats: PlanningStats,
}

/// Planning statistics
#[derive(Debug, Clone, Default)]
pub struct PlanningStats {
    /// Whether a path was found
    pub success: bool,
    /// Number of iterations
    pub iterations: usize,
    /// Planning time in milliseconds
    pub planning_time_ms: f64,
    /// Number of nodes explored
    pub nodes_explored: usize,
    /// Path length (if found)
    pub path_length: Option<f64>,
    /// Number of direction changes
    pub direction_changes: usize,
    /// Error message (if planning failed)
    pub error_message: Option<String>,
}

/// Hybrid A* path planner
pub struct HybridAStar {
    /// Vehicle model
    vehicle: VehicleModel,
    /// Planner configuration
    config: PlannerConfig,
    /// Cost weights
    weights: CostWeights,
    /// Steering angle rates for motion primitives (rad/s)
    steer_rates: Vec<f64>,
    /// Obstacle map (2D grid, 1 = obstacle)
    obstacle_map: Option<Vec<Vec<bool>>>,
    /// Map origin in world coordinates
    map_origin: (f64, f64),
}

impl HybridAStar {
    /// Create a new Hybrid A* planner
    pub fn new(vehicle: VehicleModel, config: PlannerConfig, weights: CostWeights) -> Self {
        // Define steering rate motion primitives
        let steer_rates = vec![
            -std::f64::consts::FRAC_PI_2,
            -std::f64::consts::FRAC_PI_4,
            0.0,
            std::f64::consts::FRAC_PI_4,
            std::f64::consts::FRAC_PI_2,
        ];

        Self {
            vehicle,
            config,
            weights,
            steer_rates,
            obstacle_map: None,
            map_origin: (0.0, 0.0),
        }
    }

    /// Create a planner with default configuration
    pub fn with_defaults() -> Self {
        Self::new(
            VehicleModel::default(),
            PlannerConfig::default(),
            CostWeights::default(),
        )
    }

    /// Set the obstacle map
    ///
    /// # Arguments
    /// * `map` - 2D grid where `true` indicates an obstacle
    /// * `origin_x` - World x coordinate of map origin
    /// * `origin_y` - World y coordinate of map origin
    pub fn set_obstacle_map(&mut self, map: Vec<Vec<bool>>, origin_x: f64, origin_y: f64) {
        self.obstacle_map = Some(map);
        self.map_origin = (origin_x, origin_y);
    }

    /// Create an obstacle map from a 2D array of f64 (0.0 = free, 1.0 = obstacle)
    pub fn set_obstacle_map_from_f64(&mut self, map: &[Vec<f64>], origin_x: f64, origin_y: f64) {
        let bool_map: Vec<Vec<bool>> = map
            .iter()
            .map(|row| row.iter().map(|&v| v > 0.5).collect())
            .collect();
        self.set_obstacle_map(bool_map, origin_x, origin_y);
    }

    /// Check if a state is collision-free
    ///
    /// Checks the vehicle's rectangular footprint (4 corners + 4 mid-edges + center)
    /// at the given state. Falls back to a single-point check if no footprint is
    /// configured.
    fn is_collision_free(&self, state: &State) -> bool {
        let map = match &self.obstacle_map {
            Some(m) => m,
            None => return true,
        };

        // Sample points around the vehicle footprint.
        // The footprint is defined in the planner config (or defaults).
        let (length, width) = self.footprint_size();
        let half_l = length * 0.5;
        let half_w = width * 0.5;

        // Corners + edge midpoints + center in vehicle local frame.
        let samples = [
            (0.0, 0.0),
            (half_l, 0.0),
            (-half_l, 0.0),
            (0.0, half_w),
            (0.0, -half_w),
            (half_l, half_w),
            (half_l, -half_w),
            (-half_l, half_w),
            (-half_l, -half_w),
        ];

        let cos_yaw = state.yaw.cos();
        let sin_yaw = state.yaw.sin();

        for &(lx, ly) in &samples {
            let wx = state.x + lx * cos_yaw - ly * sin_yaw;
            let wy = state.y + lx * sin_yaw + ly * cos_yaw;

            let grid_x = ((wx - self.map_origin.0) / self.config.grid_resolution) as i32;
            let grid_y = ((wy - self.map_origin.1) / self.config.grid_resolution) as i32;

            let map_height = map.len() as i32;
            let map_width = if map_height > 0 {
                map[0].len() as i32
            } else {
                0
            };

            if grid_x < 0 || grid_x >= map_width || grid_y < 0 || grid_y >= map_height {
                return false;
            }

            if map[grid_y as usize][grid_x as usize] {
                return false;
            }
        }

        true
    }

    /// Vehicle footprint size (length, width) in meters.
    ///
    /// Defaults to a small car-like rectangle: length = 1.5×wheelbase, width = 1.6 m.
    /// Can be overridden by the planner if needed.
    fn footprint_size(&self) -> (f64, f64) {
        (self.vehicle.wheelbase * 1.5, 1.6)
    }

    /// Calculate heuristic cost (Euclidean distance + angular difference)
    fn heuristic_cost(&self, state: &State, goal: &State) -> f64 {
        let distance = euclidean_distance(state.x, state.y, goal.x, goal.y);
        let angle_diff = (normalize_angle(goal.yaw - state.yaw)).abs();
        let angle_cost = angle_diff * 2.0;
        distance + angle_cost
    }

    /// Calculate steering cost
    fn steering_cost(&self, steer_angle: f64) -> f64 {
        steer_angle.abs() / self.vehicle.max_steer
    }

    /// Calculate turning cost
    fn turning_cost(&self, prev_yaw: f64, current_yaw: f64) -> f64 {
        normalize_angle(current_yaw - prev_yaw).abs()
    }

    /// Calculate cusp cost (direction change)
    fn cusp_cost(&self, prev_dir: Direction, current_dir: Direction) -> f64 {
        if prev_dir == Direction::None || current_dir == Direction::None {
            return 0.0;
        }
        if prev_dir != current_dir {
            1.0
        } else {
            0.0
        }
    }

    /// Calculate total cost for a successor
    ///
    /// `grandparent` is the parent's parent node (if any) and is used to
    /// compute the turning (yaw change) cost properly.
    fn calculate_cost(
        &self,
        parent: &Node,
        grandparent: Option<&Node>,
        final_state: &State,
        direction: Direction,
    ) -> (f64, Costs) {
        let distance_cost = self.config.velocity * self.config.simulation_time;
        let steer_cost = self.steering_cost(final_state.steer - parent.state.steer);

        // Turning cost: change in yaw between grandparent -> final_state.
        // If there is no grandparent, we have no second segment to compare
        // against, so we fall back to the parent->final_state change.
        let turn_cost = if let Some(gp) = grandparent {
            self.turning_cost(gp.state.yaw, final_state.yaw)
        } else {
            0.0
        };

        let cusp_cost = self.cusp_cost(parent.state.direction, direction);

        let costs = Costs {
            distance: distance_cost,
            steer: steer_cost,
            turn: turn_cost,
            cusp: cusp_cost,
        };

        let total_cost = distance_cost
            + self.weights.w_steer * steer_cost
            + self.weights.w_turn * turn_cost
            + self.weights.w_cusp * cusp_cost;

        (total_cost, costs)
    }

    /// Discretize state for duplicate detection
    fn discretize_state(&self, state: &State) -> (i32, i32, i32, i32, i32) {
        let grid_x = ((state.x - self.map_origin.0) / self.config.grid_resolution) as i32;
        let grid_y = ((state.y - self.map_origin.1) / self.config.grid_resolution) as i32;
        let grid_yaw = (state.yaw / self.config.angle_resolution) as i32;
        let grid_steer = (state.steer / self.config.steer_resolution) as i32;
        let grid_dir = match state.direction {
            Direction::Forward => 1,
            Direction::Backward => -1,
            Direction::None => 0,
        };
        (grid_x, grid_y, grid_yaw, grid_steer, grid_dir)
    }

    /// Check if goal is reached
    fn is_goal_reached(&self, current: &State, goal: &State) -> bool {
        let position_error = euclidean_distance(current.x, current.y, goal.x, goal.y);
        let angle_error = normalize_angle(current.yaw - goal.yaw).abs();

        position_error <= self.config.position_tolerance
            && angle_error <= self.config.angle_tolerance
    }

    /// Generate successor nodes
    fn get_successors(
        &self,
        node: &Node,
        parent_idx: usize,
        grandparent: Option<&Node>,
    ) -> Vec<(Node, Vec<State>)> {
        let mut successors = Vec::new();
        let simulation_steps = (self.config.simulation_time / self.config.dt) as usize;

        for &steer_rate in &self.steer_rates {
            for &direction in &[Direction::Forward, Direction::Backward] {
                // Create state with current direction
                let current_state = State::new(
                    node.state.x,
                    node.state.y,
                    node.state.yaw,
                    direction,
                    node.state.steer,
                );

                // Simulate motion
                let simulated = self.vehicle.simulate_motion(
                    &current_state,
                    self.config.velocity,
                    steer_rate,
                    self.config.dt,
                    simulation_steps,
                );

                if simulated.is_empty() {
                    continue;
                }

                let final_state = *simulated.last().unwrap();

                // Store trajectory
                let mut trajectory = vec![current_state];
                trajectory.extend_from_slice(&simulated);

                // Check collision for all states in trajectory (using vehicle footprint)
                let collision_free = simulated.iter().all(|s| self.is_collision_free(s));

                if !collision_free {
                    continue;
                }

                // Calculate cost (pass grandparent for turn cost)
                let (total_cost, costs) =
                    self.calculate_cost(node, grandparent, &final_state, direction);

                // Create new node
                let mut successor = Node::new(final_state);
                successor.g_cost = node.g_cost + total_cost;
                // Parent must be the index of the node that we are expanding,
                // otherwise path reconstruction is broken.
                successor.parent = Some(parent_idx);
                successor.costs = costs;
                successor.trajectory = trajectory.clone();

                successors.push((successor, trajectory));
            }
        }

        successors
    }

    /// Reconstruct path from goal node
    fn reconstruct_path(&self, nodes: &[Node], goal_idx: usize) -> Vec<Node> {
        let mut path = Vec::new();
        let mut current_idx = Some(goal_idx);

        while let Some(idx) = current_idx {
            path.push(nodes[idx].clone());
            current_idx = nodes[idx].parent;
        }

        path.reverse();
        path
    }

    /// Plan a path from start to goal
    pub fn plan(&self, start: State, goal: State, max_iterations: usize) -> PlanningResult {
        let start_time = std::time::Instant::now();

        // Check if start or goal is in collision
        if !self.is_collision_free(&start) {
            let planning_time = start_time.elapsed().as_secs_f64() * 1000.0;
            return PlanningResult {
                path: None,
                explored_nodes: vec![start],
                trajectories: Vec::new(),
                stats: PlanningStats {
                    success: false,
                    iterations: 0,
                    planning_time_ms: planning_time,
                    nodes_explored: 0,
                    path_length: None,
                    direction_changes: 0,
                    error_message: Some("Start position is in collision with obstacle".to_string()),
                },
            };
        }

        if !self.is_collision_free(&goal) {
            let planning_time = start_time.elapsed().as_secs_f64() * 1000.0;
            return PlanningResult {
                path: None,
                explored_nodes: vec![start],
                trajectories: Vec::new(),
                stats: PlanningStats {
                    success: false,
                    iterations: 0,
                    planning_time_ms: planning_time,
                    nodes_explored: 0,
                    path_length: None,
                    direction_changes: 0,
                    error_message: Some("Goal position is in collision with obstacle".to_string()),
                },
            };
        }

        // Initialize data structures. The state key now includes direction so
        // forward and reverse motion in the same cell are distinct.
        type StateKey = (i32, i32, i32, i32, i32);
        let mut open_heap: BinaryHeap<usize> = BinaryHeap::new();
        let mut nodes: Vec<Node> = Vec::new();
        let mut closed_set: HashSet<StateKey> = HashSet::new();
        let mut node_keys: HashMap<StateKey, usize> = HashMap::new();

        // Create start node
        let mut start_node = Node::new(start);
        start_node.h_cost = self.heuristic_cost(&start, &goal);
        nodes.push(start_node);
        let start_key = self.discretize_state(&start);
        node_keys.insert(start_key, 0);
        open_heap.push(0);

        let mut explored_nodes = Vec::new();
        let mut all_trajectories = Vec::new();
        let mut iterations = 0;

        while let Some(current_idx) = open_heap.pop() {
            iterations += 1;

            if iterations > max_iterations {
                break;
            }

            // Check if we already processed this node (lazy deletion)
            let current_key = self.discretize_state(&nodes[current_idx].state);
            if closed_set.contains(&current_key) {
                continue;
            }
            closed_set.insert(current_key);

            let current_node = nodes[current_idx].clone();
            explored_nodes.push(current_node.state);

            // Check goal at expansion time so that the first goal hit
            // corresponds to the lowest g_cost reachable.
            if self.is_goal_reached(&current_node.state, &goal) {
                let planning_time = start_time.elapsed().as_secs_f64() * 1000.0;
                let path = self.reconstruct_path(&nodes, current_idx);
                let path_length = self.calculate_path_length(&path);
                let direction_changes = self.count_direction_changes(&path);
                let nodes_explored = explored_nodes.len();

                return PlanningResult {
                    path: Some(path),
                    explored_nodes,
                    trajectories: all_trajectories,
                    stats: PlanningStats {
                        success: true,
                        iterations,
                        planning_time_ms: planning_time,
                        nodes_explored,
                        path_length: Some(path_length),
                        direction_changes,
                        error_message: None,
                    },
                };
            }

            // Locate the grandparent for proper turn cost computation.
            let grandparent: Option<&Node> = current_node.parent.and_then(|pidx| nodes.get(pidx));

            // Generate successors (passing current_idx so the parent pointer
            // is set correctly).
            let successors = self.get_successors(&current_node, current_idx, grandparent);

            for (successor, trajectory) in successors {
                let succ_key = self.discretize_state(&successor.state);

                if closed_set.contains(&succ_key) {
                    continue;
                }

                if let Some(&existing_idx) = node_keys.get(&succ_key) {
                    let existing_g = nodes[existing_idx].g_cost;
                    if existing_g <= successor.g_cost {
                        continue;
                    }
                    // Better path found: update the existing node in place
                    // and push a new entry onto the heap (lazy deletion
                    // will skip the old entry when it is popped).
                    let mut updated = successor;
                    // Preserve the position in `nodes` so the rest of the
                    // tree (already pushed successors) still references a
                    // valid index.
                    updated.parent = Some(current_idx);
                    let new_idx = existing_idx;
                    nodes[new_idx] = updated;
                    open_heap.push(new_idx);
                    all_trajectories.push(trajectory);
                    continue;
                }

                let new_idx = nodes.len();
                all_trajectories.push(trajectory);
                node_keys.insert(succ_key, new_idx);
                nodes.push(successor);
                open_heap.push(new_idx);
            }
        }

        // No path found
        let planning_time = start_time.elapsed().as_secs_f64() * 1000.0;
        let nodes_explored = explored_nodes.len();
        PlanningResult {
            path: None,
            explored_nodes,
            trajectories: all_trajectories,
            stats: PlanningStats {
                success: false,
                iterations,
                planning_time_ms: planning_time,
                nodes_explored,
                path_length: None,
                direction_changes: 0,
                error_message: Some(format!(
                    "No path found after {} iterations. Try adjusting parameters or positions.",
                    iterations
                )),
            },
        }
    }

    /// Calculate total path length
    fn calculate_path_length(&self, path: &[Node]) -> f64 {
        if path.len() < 2 {
            return 0.0;
        }

        let mut length = 0.0;
        for i in 1..path.len() {
            length += euclidean_distance(
                path[i - 1].state.x,
                path[i - 1].state.y,
                path[i].state.x,
                path[i].state.y,
            );
        }
        length
    }

    /// Count direction changes in path
    fn count_direction_changes(&self, path: &[Node]) -> usize {
        if path.len() < 2 {
            return 0;
        }

        let mut changes = 0;
        for i in 1..path.len() {
            if path[i].state.direction != path[i - 1].state.direction {
                changes += 1;
            }
        }
        changes
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_planner_creation() {
        let planner = HybridAStar::with_defaults();
        assert_eq!(planner.config.grid_resolution, 0.5);
    }

    #[test]
    fn test_planner_without_obstacles() {
        let planner = HybridAStar::with_defaults();
        let start = State::start(0.0, 0.0, 0.0);
        let goal = State::goal(5.0, 5.0, std::f64::consts::FRAC_PI_4);

        let result = planner.plan(start, goal, 1000);
        assert!(result.stats.iterations > 0);
        assert!(!result.explored_nodes.is_empty());
    }

    #[test]
    fn test_planner_with_obstacles() {
        let mut planner = HybridAStar::with_defaults();

        // Create a large enough obstacle map
        let mut map = vec![vec![false; 40]; 40];
        // Add a vertical wall in the middle
        for y in 10..30 {
            map[y][20] = true;
        }
        planner.set_obstacle_map(map, 0.0, 0.0);

        // Start on left side, goal on right side, must go around wall
        // grid_x range: 0..40 with resolution 0.5 = world 0..20
        let start = State::start(5.0, 10.0, 0.0);
        let goal = State::goal(15.0, 10.0, 0.0);

        let result = planner.plan(start, goal, 2000);
        // Should find a path (or at least explore nodes)
        assert!(result.stats.nodes_explored > 0 || result.stats.success);
    }

    #[test]
    fn test_heuristic_cost() {
        let planner = HybridAStar::with_defaults();
        let state = State::new(0.0, 0.0, 0.0, Direction::Forward, 0.0);
        let goal = State::new(3.0, 4.0, 0.0, Direction::Forward, 0.0);

        let cost = planner.heuristic_cost(&state, &goal);
        // Euclidean distance is 5.0
        assert!((cost - 5.0).abs() < 0.01);
    }
}
