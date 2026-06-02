//! Mathematical utility functions for the Hybrid A* algorithm.

/// Normalize angle to [-π, π] range
pub fn normalize_angle(angle: f64) -> f64 {
    let mut a = angle;
    while a > std::f64::consts::PI {
        a -= 2.0 * std::f64::consts::PI;
    }
    while a < -std::f64::consts::PI {
        a += 2.0 * std::f64::consts::PI;
    }
    a
}

/// Calculate Euclidean distance between two points
pub fn euclidean_distance(x1: f64, y1: f64, x2: f64, y2: f64) -> f64 {
    let dx = x2 - x1;
    let dy = y2 - y1;
    (dx * dx + dy * dy).sqrt()
}

/// Clip a value to be within [min, max] range
pub fn clip(value: f64, min: f64, max: f64) -> f64 {
    value.max(min).min(max)
}

/// Round to specified precision (for state discretization)
pub fn round_to_precision(value: f64, precision: f64) -> f64 {
    (value / precision).round() * precision
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_normalize_angle() {
        assert!((normalize_angle(0.0) - 0.0).abs() < 1e-10);
        assert!((normalize_angle(std::f64::consts::PI) - std::f64::consts::PI).abs() < 1e-10);
        assert!((normalize_angle(-std::f64::consts::PI) - (-std::f64::consts::PI)).abs() < 1e-10);
        assert!((normalize_angle(3.0 * std::f64::consts::PI) - std::f64::consts::PI).abs() < 1e-10);
        assert!(
            (normalize_angle(-3.0 * std::f64::consts::PI) - (-std::f64::consts::PI)).abs() < 1e-10
        );
    }

    #[test]
    fn test_euclidean_distance() {
        assert!((euclidean_distance(0.0, 0.0, 3.0, 4.0) - 5.0).abs() < 1e-10);
        assert!((euclidean_distance(1.0, 1.0, 1.0, 1.0)) < 1e-10);
    }

    #[test]
    fn test_clip() {
        assert!((clip(5.0, 0.0, 10.0) - 5.0).abs() < 1e-10);
        assert!((clip(-5.0, 0.0, 10.0)) < 1e-10);
        assert!((clip(15.0, 0.0, 10.0) - 10.0).abs() < 1e-10);
    }
}
