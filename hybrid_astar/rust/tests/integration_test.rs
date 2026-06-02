//! Integration tests for the Hybrid A* planner.
//!
//! These exercise the full plan() loop with realistic obstacle maps to
//! guard against regressions in path reconstruction and goal reaching.

use hybrid_astar_rust::{CostWeights, Direction, HybridAStar, PlannerConfig, State, VehicleModel};

fn build_basic_map(size: usize) -> (Vec<Vec<bool>>, f64) {
    let mut map = vec![vec![false; size]; size];
    for i in 0..size {
        map[0][i] = true;
        map[size - 1][i] = true;
        map[i][0] = true;
        map[i][size - 1] = true;
    }
    // A block of obstacle in the middle
    for y in 20..25 {
        for x in 20..28 {
            map[y][x] = true;
        }
    }
    (map, 0.5)
}

fn planner() -> HybridAStar {
    let vehicle = VehicleModel::new(2.5, std::f64::consts::FRAC_PI_4);
    let config = PlannerConfig {
        grid_resolution: 0.5,
        angle_resolution: std::f64::consts::PI / 8.0,
        steer_resolution: std::f64::consts::PI / 16.0,
        velocity: 2.0,
        simulation_time: 1.0,
        dt: 0.1,
        position_tolerance: 1.0,
        angle_tolerance: std::f64::consts::PI / 6.0,
    };
    let weights = CostWeights {
        w_steer: 10.0,
        w_turn: 15.0,
        w_cusp: 10.0,
    };
    HybridAStar::new(vehicle, config, weights)
}

#[test]
fn finds_straight_path_when_no_obstacles() {
    let mut p = planner();
    let map: Vec<Vec<bool>> = vec![vec![false; 40]; 40];
    p.set_obstacle_map(map, 0.0, 0.0);
    let start = State::start(5.0, 5.0, 0.0);
    let goal = State::goal(10.0, 5.0, 0.0);

    let result = p.plan(start, goal, 5000);
    assert!(result.stats.success, "should find a path");
    let path = result.path.expect("path should be present");
    assert!(path.len() >= 2, "path should contain at least 2 nodes");

    // The reconstructed path must start at the start and end near the goal
    let first = &path.first().unwrap().state;
    let last = &path.last().unwrap().state;
    assert!((first.x - 5.0).abs() < 1e-6, "path should start at start.x");
    assert!((first.y - 5.0).abs() < 1e-6, "path should start at start.y");
    let end_dist = ((last.x - 10.0).powi(2) + (last.y - 5.0).powi(2)).sqrt();
    assert!(end_dist <= 1.0, "path should end within 1 m of the goal");
}

#[test]
fn reconstructs_path_correctly_around_obstacle() {
    let mut p = planner();
    let (map, origin) = build_basic_map(60);
    p.set_obstacle_map(map, origin, origin);

    // Start (5, 5) and goal (25, 5) are both inside the walled 60x60 map
    // (which covers 0..30 m). The obstacle block sits at world (10..14, 10..12.5).
    let start = State::new(5.0, 5.0, 0.0, Direction::Forward, 0.0);
    let goal = State::new(25.0, 5.0, 0.0, Direction::Forward, 0.0);

    let result = p.plan(start, goal, 8000);
    assert!(result.stats.success, "should find a path around obstacle");

    let path = result.path.expect("path present");
    assert!(path.len() >= 2);

    // First node is the start, last is within tolerance of the goal
    let first = &path.first().unwrap().state;
    let last = &path.last().unwrap().state;
    assert!((first.x - 5.0).abs() < 1e-6);
    assert!((first.y - 5.0).abs() < 1e-6);

    // Verify continuity: each path segment is bounded by one simulation step
    for w in path.windows(2) {
        let dx = w[1].state.x - w[0].state.x;
        let dy = w[1].state.y - w[0].state.y;
        let dist = (dx * dx + dy * dy).sqrt();
        assert!(
            dist < 4.5,
            "path segment too long: {:.2} m (reconstruction broken?)",
            dist
        );
    }

    let final_distance_to_goal = ((last.x - 25.0).powi(2) + (last.y - 5.0).powi(2)).sqrt();
    assert!(
        final_distance_to_goal <= 1.0,
        "path does not end at goal: dist={}",
        final_distance_to_goal
    );
}

#[test]
fn reports_failure_when_blocked_in() {
    let mut p = planner();
    let mut map = vec![vec![false; 40]; 40];
    // Surround the start in a box of obstacles.
    for y in 4..8 {
        for x in 4..8 {
            map[y][x] = true;
        }
    }
    // Carve out a single starting cell.
    map[5][5] = false;
    p.set_obstacle_map(map, 0.0, 0.0);

    let start = State::new(2.5, 2.5, 0.0, Direction::Forward, 0.0);
    let goal = State::new(15.0, 15.0, 0.0, Direction::Forward, 0.0);

    // The start is in collision, so the planner must report failure with
    // a clear error message rather than panicking.
    let result = p.plan(start, goal, 1000);
    assert!(!result.stats.success);
    assert!(result.stats.error_message.is_some());
}

#[test]
fn map_origin_is_respected() {
    // Map origin at (-10, -10); cell (0,0) corresponds to world (-10,-10).
    // The 40x40 map at 0.5 m resolution covers -10..10 m.
    let mut p = planner();
    let map = vec![vec![false; 40]; 40];
    p.set_obstacle_map(map, -10.0, -10.0);

    let start = State::start(-5.0, -5.0, 0.0);
    let goal = State::goal(5.0, 5.0, 0.0);
    let result = p.plan(start, goal, 20000);
    assert!(
        result.stats.success,
        "should find a path with non-zero map origin"
    );
    let path = result.path.expect("path");
    let first = &path.first().unwrap().state;
    let last = &path.last().unwrap().state;
    assert!((first.x - -5.0).abs() < 1e-6);
    assert!((first.y - -5.0).abs() < 1e-6);
    let end_dist = ((last.x - 5.0).powi(2) + (last.y - 5.0).powi(2)).sqrt();
    assert!(end_dist <= 1.0);
}
