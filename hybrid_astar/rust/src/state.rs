//! State and direction definitions for the Hybrid A* algorithm.

use std::fmt;

/// Vehicle direction mode
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum Direction {
    Forward,
    Backward,
    None,
}

impl Direction {
    /// Get the velocity multiplier for this direction
    pub fn velocity_multiplier(&self) -> f64 {
        match self {
            Direction::Forward => 1.0,
            Direction::Backward => -1.0,
            Direction::None => 0.0,
        }
    }
}

impl fmt::Display for Direction {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Direction::Forward => write!(f, "FORWARD"),
            Direction::Backward => write!(f, "BACKWARD"),
            Direction::None => write!(f, "NONE"),
        }
    }
}

/// Vehicle state representation
#[derive(Debug, Clone, Copy)]
pub struct State {
    pub x: f64,
    pub y: f64,
    pub yaw: f64, // heading angle in radians
    pub direction: Direction,
    pub steer: f64, // steering angle in radians
}

impl State {
    /// Create a new state
    pub fn new(x: f64, y: f64, yaw: f64, direction: Direction, steer: f64) -> Self {
        Self {
            x,
            y,
            yaw,
            direction,
            steer,
        }
    }

    /// Create a start state with default values
    pub fn start(x: f64, y: f64, yaw: f64) -> Self {
        Self::new(x, y, yaw, Direction::Forward, 0.0)
    }

    /// Create a goal state
    pub fn goal(x: f64, y: f64, yaw: f64) -> Self {
        Self::new(x, y, yaw, Direction::Forward, 0.0)
    }
}

impl PartialEq for State {
    fn eq(&self, other: &Self) -> bool {
        (self.x - other.x).abs() < 0.1
            && (self.y - other.y).abs() < 0.1
            && (self.yaw - other.yaw).abs() < 0.1
    }
}

impl Eq for State {}

impl std::hash::Hash for State {
    fn hash<H: std::hash::Hasher>(&self, state: &mut H) {
        // Hash rounded values for discretization (position + heading)
        ((self.x * 100.0) as i64).hash(state);
        ((self.y * 100.0) as i64).hash(state);
        ((self.yaw * 100.0) as i64).hash(state);
        // Include direction in the hash so opposite motion directions
        // at the same cell are not considered duplicates.
        let dir_tag: i32 = match self.direction {
            Direction::Forward => 1,
            Direction::Backward => -1,
            Direction::None => 0,
        };
        dir_tag.hash(state);
    }
}

impl fmt::Display for State {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "State({:.2}, {:.2}, {:.2}°, {}, {:.2}°)",
            self.x,
            self.y,
            self.yaw.to_degrees(),
            self.direction,
            self.steer.to_degrees()
        )
    }
}

/// Cost components for path planning
#[derive(Debug, Clone, Copy, Default)]
pub struct Costs {
    pub distance: f64,
    pub steer: f64,
    pub turn: f64,
    pub cusp: f64,
}

impl Costs {
    pub fn new() -> Self {
        Self::default()
    }
}

/// A* search node
#[derive(Debug, Clone)]
pub struct Node {
    pub state: State,
    pub g_cost: f64,
    pub h_cost: f64,
    pub parent: Option<usize>, // Index in the nodes vector
    pub costs: Costs,
    pub trajectory: Vec<State>, // Forward simulation trajectory
}

impl Node {
    /// Create a new node
    pub fn new(state: State) -> Self {
        Self {
            state,
            g_cost: 0.0,
            h_cost: 0.0,
            parent: None,
            costs: Costs::new(),
            trajectory: Vec::new(),
        }
    }

    /// Get total cost (f = g + h)
    pub fn f_cost(&self) -> f64 {
        self.g_cost + self.h_cost
    }
}

impl PartialEq for Node {
    fn eq(&self, other: &Self) -> bool {
        self.f_cost() == other.f_cost()
    }
}

impl Eq for Node {}

impl PartialOrd for Node {
    fn partial_cmp(&self, other: &Self) -> Option<std::cmp::Ordering> {
        // Reverse ordering for min-heap (we want lowest f_cost first)
        other.f_cost().partial_cmp(&self.f_cost())
    }
}

impl Ord for Node {
    fn cmp(&self, other: &Self) -> std::cmp::Ordering {
        self.partial_cmp(other).unwrap_or(std::cmp::Ordering::Equal)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_state_creation() {
        let state = State::new(1.0, 2.0, 0.5, Direction::Forward, 0.1);
        assert_eq!(state.x, 1.0);
        assert_eq!(state.y, 2.0);
        assert_eq!(state.yaw, 0.5);
        assert_eq!(state.direction, Direction::Forward);
        assert_eq!(state.steer, 0.1);
    }

    #[test]
    fn test_state_equality() {
        let s1 = State::new(1.0, 2.0, 0.5, Direction::Forward, 0.1);
        let s2 = State::new(1.0, 2.0, 0.5, Direction::Backward, 0.2);
        assert_eq!(s1, s2); // Only x, y, yaw matter for equality
    }

    #[test]
    fn test_direction_velocity() {
        assert_eq!(Direction::Forward.velocity_multiplier(), 1.0);
        assert_eq!(Direction::Backward.velocity_multiplier(), -1.0);
        assert_eq!(Direction::None.velocity_multiplier(), 0.0);
    }

    #[test]
    fn test_node_ordering() {
        let mut n1 = Node::new(State::new(0.0, 0.0, 0.0, Direction::Forward, 0.0));
        n1.g_cost = 1.0;
        n1.h_cost = 2.0;

        let mut n2 = Node::new(State::new(1.0, 1.0, 0.0, Direction::Forward, 0.0));
        n2.g_cost = 2.0;
        n2.h_cost = 1.0;

        // n1 has f_cost=3.0, n2 has f_cost=3.0
        assert_eq!(n1.f_cost(), n2.f_cost());
    }
}
