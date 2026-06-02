# Hybrid A* Path Planning - Rust Implementation

A high-performance Rust implementation of the Hybrid A* path planning algorithm with an interactive GUI demo.

## Features

- **Core Algorithm**: Complete Hybrid A* implementation with bicycle model kinematics
- **Vehicle Footprint**: Rectangular collision check (not just a single point)
- **Direction-aware Search**: Forward and reverse states are tracked separately
- **Optimal Goal Detection**: Goal check at expansion time guarantees the first hit is on the optimal path
- **Interactive GUI**: Real-time visualization using egui
  - Start/goal pose shown as oriented vehicle rectangles
  - Path drawn as a halo'd polyline with per-waypoint direction ticks
  - Vehicle-shape previews sampled along the final path
  - Color-coded explored nodes (forward vs reverse)
  - Hover footprint preview
  - Live planning in a background thread (UI stays responsive)
  - "Fit View" auto-frames start, goal and path
  - Grid coordinate labels
- **Multiple Scenarios**: Pre-built scenarios (Basic Navigation, Parking, U-Turn, Custom)
- **Parameter Tuning**: Adjust algorithm parameters and cost weights in real-time

## Requirements

- Rust 1.70+ (with Cargo)
- For GUI: System libraries for egui (see [eframe dependencies](https://github.com/emilk/egui))

## Building and Running

```bash
cd hybrid_astar/rust

# Build and run the GUI demo
cargo run --release

# Run tests
cargo test
```

## Project Structure

```
hybrid_astar/rust/
├── Cargo.toml           # Package configuration
├── README.md            # This file
└── src/
    ├── lib.rs           # Library exports
    ├── main.rs          # GUI demo application
    ├── math_utils.rs    # Mathematical utilities
    ├── state.rs         # State and direction definitions
    ├── vehicle_model.rs # Bicycle model implementation
    └── hybrid_astar.rs  # Core algorithm
```

## Usage

### GUI Controls

| Control | Action |
|---------|--------|
| Left-click | Set start position |
| Shift+click | Set goal position |
| Drag | Pan view |
| Scroll | Zoom in/out |

### Programmatic Usage

```rust
use hybrid_astar_rust::{HybridAStar, VehicleModel, State, Direction, PlannerConfig, CostWeights};

// Create vehicle model
let vehicle = VehicleModel::new(2.5, std::f64::consts::FRAC_PI_4);

// Configure planner
let config = PlannerConfig {
    grid_resolution: 0.5,
    velocity: 2.0,
    simulation_time: 1.0,
    ..Default::default()
};

let weights = CostWeights {
    w_steer: 10.0,
    w_turn: 15.0,
    w_cusp: 10.0,
};

// Create planner
let mut planner = HybridAStar::new(vehicle, config, weights);

// Set obstacle map (optional)
let obstacle_map = vec![vec![false; 40]; 40];
planner.set_obstacle_map(obstacle_map, -5.0, -5.0);

// Define start and goal
let start = State::start(0.0, 0.0, 0.0);
let goal = State::goal(10.0, 10.0, std::f64::consts::FRAC_PI_4);

// Plan path
let result = planner.plan(start, goal, 5000);

if result.stats.success {
    println!("Path found! Length: {:.2} m", result.stats.path_length.unwrap());
} else {
    println!("No path found after {} iterations", result.stats.iterations);
}
```

## Algorithm Parameters

| Parameter | Description | Default |
|-----------|-------------|---------|
| `grid_resolution` | Grid cell size (m) | 0.5 |
| `angle_resolution` | Heading discretization (rad) | π/8 |
| `steer_resolution` | Steering discretization (rad) | π/16 |
| `velocity` | Simulation velocity (m/s) | 2.0 |
| `simulation_time` | Forward simulation duration (s) | 1.0 |
| `dt` | Time step (s) | 0.1 |
| `position_tolerance` | Goal position tolerance (m) | 1.0 |
| `angle_tolerance` | Goal angle tolerance (rad) | π/6 |

## Cost Weights

| Weight | Description | Default |
|--------|-------------|---------|
| `w_steer` | Steering angle penalty | 10.0 |
| `w_turn` | Turning rate penalty | 15.0 |
| `w_cusp` | Direction change penalty | 10.0 |

## Performance

The Rust implementation provides significant performance improvements over Python:

- **Memory Safety**: Guaranteed by Rust's ownership system
- **Zero-Cost Abstractions**: No runtime overhead for generics
- **Efficient Data Structures**: Custom implementations optimized for path planning

Typical performance on modern hardware:
- Small maps (20×20): < 50ms
- Medium maps (40×40): < 200ms
- Large maps (100×100): < 1s

## License

MIT License
