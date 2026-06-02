//! Vehicle kinematic model for the Hybrid A* algorithm.

use crate::math_utils::{clip, normalize_angle};
use crate::state::State;

/// Simple bicycle model for vehicle simulation
#[derive(Debug, Clone)]
pub struct VehicleModel {
    /// Distance between front and rear axles (m)
    pub wheelbase: f64,
    /// Maximum steering angle (rad)
    pub max_steer: f64,
}

impl VehicleModel {
    /// Create a new vehicle model
    pub fn new(wheelbase: f64, max_steer: f64) -> Self {
        Self {
            wheelbase,
            max_steer,
        }
    }

    /// Create a default vehicle model
    pub fn default() -> Self {
        Self::new(2.5, std::f64::consts::FRAC_PI_4) // 45 degrees max steer
    }

    /// Forward simulate vehicle motion with fixed linear velocity and steering rate
    ///
    /// # Arguments
    /// * `state` - Initial state
    /// * `velocity` - Linear velocity (m/s)
    /// * `steer_rate` - Steering angle rate (rad/s)
    /// * `dt` - Time step (s)
    /// * `steps` - Number of simulation steps
    ///
    /// # Returns
    /// List of simulated states
    pub fn simulate_motion(
        &self,
        state: &State,
        velocity: f64,
        steer_rate: f64,
        dt: f64,
        steps: usize,
    ) -> Vec<State> {
        let mut states = Vec::with_capacity(steps);
        let mut current = *state;

        for _ in 0..steps {
            // Update steering angle
            let new_steer = clip(
                current.steer + steer_rate * dt,
                -self.max_steer,
                self.max_steer,
            );

            // Calculate velocity based on the (potentially evolving) direction.
            let v = velocity * current.direction.velocity_multiplier();

            // Bicycle model kinematics
            let new_x = current.x + v * current.yaw.cos() * dt;
            let new_y = current.y + v * current.yaw.sin() * dt;
            let new_yaw = current.yaw + v * new_steer.tan() / self.wheelbase * dt;

            // Normalize yaw angle
            let new_yaw = normalize_angle(new_yaw);

            current = State::new(new_x, new_y, new_yaw, current.direction, new_steer);
            states.push(current);
        }

        states
    }

    /// Check if a state is within the vehicle's kinematic constraints
    pub fn is_valid_state(&self, state: &State) -> bool {
        state.steer.abs() <= self.max_steer
    }

    /// Get the vehicle's turning radius for a given steering angle
    pub fn turning_radius(&self, steer_angle: f64) -> f64 {
        if steer_angle.abs() < 1e-10 {
            f64::INFINITY
        } else {
            self.wheelbase / steer_angle.tan().abs()
        }
    }

    /// Get the minimum turning radius
    pub fn min_turning_radius(&self) -> f64 {
        self.turning_radius(self.max_steer)
    }
}

impl Default for VehicleModel {
    fn default() -> Self {
        Self::new(2.5, std::f64::consts::FRAC_PI_4)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Direction;

    #[test]
    fn test_vehicle_model_creation() {
        let vm = VehicleModel::new(2.5, std::f64::consts::FRAC_PI_4);
        assert_eq!(vm.wheelbase, 2.5);
        assert_eq!(vm.max_steer, std::f64::consts::FRAC_PI_4);
    }

    #[test]
    fn test_simulate_motion_straight() {
        let vm = VehicleModel::new(2.5, std::f64::consts::FRAC_PI_4);
        let state = State::new(0.0, 0.0, 0.0, Direction::Forward, 0.0);

        let states = vm.simulate_motion(&state, 1.0, 0.0, 0.1, 10);

        assert_eq!(states.len(), 10);
        // After 10 steps of 0.1s at 1m/s, should move 1m in x direction
        assert!((states.last().unwrap().x - 1.0).abs() < 0.01);
        assert!(states.last().unwrap().y.abs() < 0.01);
    }

    #[test]
    fn test_simulate_motion_turning() {
        let vm = VehicleModel::new(2.5, std::f64::consts::FRAC_PI_4);
        let state = State::new(0.0, 0.0, 0.0, Direction::Forward, 0.0);

        let states = vm.simulate_motion(&state, 1.0, 1.0, 0.1, 10);

        assert_eq!(states.len(), 10);
        // Vehicle should have turned
        assert!(states.last().unwrap().yaw.abs() > 0.0);
    }

    #[test]
    fn test_min_turning_radius() {
        let vm = VehicleModel::new(2.5, std::f64::consts::FRAC_PI_4);
        let min_radius = vm.min_turning_radius();

        // For 45 degrees, radius = wheelbase / tan(45°) = wheelbase
        assert!((min_radius - 2.5).abs() < 0.01);
    }
}
