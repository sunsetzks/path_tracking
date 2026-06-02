//! Hybrid A* Path Planning Algorithm - GUI Demo
//!
//! Interactive visualization of the Hybrid A* algorithm using egui.

use std::sync::mpsc::{channel, Receiver};
use std::thread;

use eframe::egui;
use hybrid_astar_rust::{
    CostWeights, Direction, HybridAStar, PlannerConfig, PlanningResult, State, VehicleModel,
};

fn main() -> eframe::Result<()> {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1280.0, 820.0])
            .with_title("Hybrid A* Path Planning Demo"),
        ..Default::default()
    };

    eframe::run_native(
        "Hybrid A* Demo",
        options,
        Box::new(|cc| {
            let mut style = (*cc.egui_ctx.style()).clone();
            style.spacing.item_spacing = egui::vec2(6.0, 6.0);
            cc.egui_ctx.set_style(style);
            Ok(Box::new(App::new()))
        }),
    )
}

struct App {
    config: PlannerConfig,
    weights: CostWeights,
    map_size: usize,
    obstacle_density: f64,
    start_pos: [f64; 2],
    start_yaw: f64,
    goal_pos: [f64; 2],
    goal_yaw: f64,
    obstacle_map: Vec<Vec<bool>>,
    map_offset: [f64; 2],
    result: Option<PlanningResult>,
    selected_path_point: Option<usize>,
    show_explored: bool,
    show_trajectories: bool,
    show_grid: bool,
    show_vehicle_shape: bool,
    show_footprint: bool,
    show_start_goal_arrows: bool,
    zoom: f32,
    pan: [f32; 2],
    is_planning: bool,
    scenario: Scenario,
    planning_rx: Option<Receiver<PlanningResult>>,
    progress_msg: String,
    zoom_to_fit_pending: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
enum Scenario {
    BasicNavigation,
    Parking,
    UTurn,
    Custom,
}

impl App {
    fn new() -> Self {
        let map_size = 80;
        let map_offset = [0.0, 0.0];
        let mut app = Self {
            config: PlannerConfig::default(),
            weights: CostWeights::default(),
            map_size,
            obstacle_density: 0.3,
            start_pos: [5.0, 5.0],
            start_yaw: 0.0,
            goal_pos: [35.0, 35.0],
            goal_yaw: std::f64::consts::FRAC_PI_4,
            obstacle_map: vec![vec![false; map_size]; map_size],
            map_offset,
            result: None,
            selected_path_point: None,
            show_explored: true,
            show_trajectories: false,
            show_grid: true,
            show_vehicle_shape: true,
            show_footprint: true,
            show_start_goal_arrows: true,
            zoom: 12.0,
            pan: [0.0, 0.0],
            is_planning: false,
            scenario: Scenario::BasicNavigation,
            planning_rx: None,
            progress_msg: String::new(),
            zoom_to_fit_pending: true,
        };
        app.generate_scenario();
        app
    }

    fn generate_scenario(&mut self) {
        let size = self.map_size;
        self.obstacle_map = vec![vec![false; size]; size];

        match self.scenario {
            Scenario::BasicNavigation => {
                for i in 0..size {
                    self.obstacle_map[0][i] = true;
                    self.obstacle_map[size - 1][i] = true;
                    self.obstacle_map[i][0] = true;
                    self.obstacle_map[i][size - 1] = true;
                }
                for y in 15..25 {
                    for x in 10..13 {
                        if x < size && y < size {
                            self.obstacle_map[y][x] = true;
                        }
                    }
                }
                for y in 20..23 {
                    for x in 20..30 {
                        if x < size && y < size {
                            self.obstacle_map[y][x] = true;
                        }
                    }
                }
                for y in 50..58 {
                    for x in 40..45 {
                        if x < size && y < size {
                            self.obstacle_map[y][x] = true;
                        }
                    }
                }

                self.start_pos = [3.0, 3.0];
                self.start_yaw = 0.0;
                self.goal_pos = [60.0, 60.0];
                self.goal_yaw = std::f64::consts::FRAC_PI_4;
            }
            Scenario::Parking => {
                for i in 0..size {
                    self.obstacle_map[0][i] = true;
                    self.obstacle_map[size - 1][i] = true;
                }
                for i in (6..size - 4).step_by(8) {
                    for y in 8..12 {
                        for x in i..(i + 5) {
                            if x < size && y < size {
                                self.obstacle_map[y][x] = true;
                            }
                        }
                    }
                    for y in 28..32 {
                        for x in i..(i + 5) {
                            if x < size && y < size {
                                self.obstacle_map[y][x] = true;
                            }
                        }
                    }
                }
                for y in 18..22 {
                    for x in 8..32 {
                        if x < size && y < size {
                            self.obstacle_map[y][x] = true;
                        }
                    }
                }

                self.start_pos = [6.0, 40.0];
                self.start_yaw = -std::f64::consts::FRAC_PI_2;
                self.goal_pos = [38.0, 10.0];
                self.goal_yaw = std::f64::consts::FRAC_PI_2;
            }
            Scenario::UTurn => {
                for i in 0..size {
                    for j in 0..10 {
                        if j < size {
                            self.obstacle_map[j][i] = true;
                        }
                    }
                    for j in 30..size {
                        if j < size {
                            self.obstacle_map[j][i] = true;
                        }
                    }
                }
                for i in size - 10..size {
                    for j in 10..30 {
                        if i < size && j < size {
                            self.obstacle_map[j][i] = true;
                        }
                    }
                }

                self.start_pos = [10.0, 20.0];
                self.start_yaw = 0.0;
                self.goal_pos = [55.0, 20.0];
                self.goal_yaw = 0.0;
            }
            Scenario::Custom => {
                let mut rng_state: u64 = 12345;
                for y in 0..size {
                    for x in 0..size {
                        rng_state = rng_state.wrapping_mul(1103515245).wrapping_add(12345);
                        let r = ((rng_state >> 16) & 0xFFFF) as f64 / 65535.0;
                        self.obstacle_map[y][x] = r < self.obstacle_density;
                    }
                }

                let sx =
                    ((self.start_pos[0] - self.map_offset[0]) / self.config.grid_resolution) as i32;
                let sy =
                    ((self.start_pos[1] - self.map_offset[1]) / self.config.grid_resolution) as i32;
                let gx =
                    ((self.goal_pos[0] - self.map_offset[0]) / self.config.grid_resolution) as i32;
                let gy =
                    ((self.goal_pos[1] - self.map_offset[1]) / self.config.grid_resolution) as i32;

                for radius in 0..3 {
                    for dy in -(radius as i32)..=(radius as i32) {
                        for dx in -(radius as i32)..=(radius as i32) {
                            let nx = (sx + dx).max(0) as usize;
                            let ny = (sy + dy).max(0) as usize;
                            if nx < size && ny < size {
                                self.obstacle_map[ny][nx] = false;
                            }
                            let nx2 = (gx + dx).max(0) as usize;
                            let ny2 = (gy + dy).max(0) as usize;
                            if nx2 < size && ny2 < size {
                                self.obstacle_map[ny2][nx2] = false;
                            }
                        }
                    }
                }
            }
        }
        self.result = None;
    }

    /// Run planning in a background thread.
    fn run_planning(&mut self) {
        if self.is_planning {
            return;
        }
        self.is_planning = true;
        self.result = None;
        self.progress_msg = "Planning...".to_string();

        let vehicle = VehicleModel::new(2.5, std::f64::consts::FRAC_PI_4);
        let config = self.config.clone();
        let weights = self.weights.clone();
        let f64_map: Vec<Vec<f64>> = self
            .obstacle_map
            .iter()
            .map(|row| row.iter().map(|&b| if b { 1.0 } else { 0.0 }).collect())
            .collect();
        let origin = self.map_offset;
        let start = State::new(
            self.start_pos[0],
            self.start_pos[1],
            self.start_yaw,
            Direction::Forward,
            0.0,
        );
        let goal = State::new(
            self.goal_pos[0],
            self.goal_pos[1],
            self.goal_yaw,
            Direction::Forward,
            0.0,
        );

        let (tx, rx) = channel();
        self.planning_rx = Some(rx);

        thread::spawn(move || {
            let mut planner = HybridAStar::new(vehicle, config, weights);
            planner.set_obstacle_map_from_f64(&f64_map, origin[0], origin[1]);
            let result = planner.plan(start, goal, 10000);
            let _ = tx.send(result);
        });
    }

    fn check_planning_result(&mut self) {
        if let Some(rx) = &self.planning_rx {
            if let Ok(result) = rx.try_recv() {
                self.is_planning = false;
                self.planning_rx = None;
                self.progress_msg = if result.stats.success {
                    format!(
                        "Path found in {:.0} ms ({} iters)",
                        result.stats.planning_time_ms, result.stats.iterations
                    )
                } else {
                    "No path found".to_string()
                };
                self.result = Some(result);
                self.selected_path_point = None;
            }
        }
    }

    fn is_in_collision(&self, pos: [f64; 2]) -> bool {
        let grid_x = ((pos[0] - self.map_offset[0]) / self.config.grid_resolution) as i32;
        let grid_y = ((pos[1] - self.map_offset[1]) / self.config.grid_resolution) as i32;
        if grid_x < 0
            || grid_x >= self.map_size as i32
            || grid_y < 0
            || grid_y >= self.map_size as i32
        {
            return true;
        }
        self.obstacle_map[grid_y as usize][grid_x as usize]
    }

    fn get_position_status(&self) -> (String, String) {
        let start_status = if self.is_in_collision(self.start_pos) {
            "In Obstacle!".to_string()
        } else {
            "OK".to_string()
        };
        let goal_status = if self.is_in_collision(self.goal_pos) {
            "In Obstacle!".to_string()
        } else {
            "OK".to_string()
        };
        (start_status, goal_status)
    }

    fn world_to_screen(&self, pos: [f64; 2], canvas_rect: egui::Rect) -> egui::Pos2 {
        let center_x = canvas_rect.center().x;
        let center_y = canvas_rect.center().y;
        let x = center_x + (pos[0] as f32 + self.pan[0]) * self.zoom;
        let y = center_y - (pos[1] as f32 + self.pan[1]) * self.zoom;
        egui::Pos2::new(x, y)
    }

    fn screen_to_world(&self, pos: egui::Pos2, canvas_rect: egui::Rect) -> [f64; 2] {
        let center_x = canvas_rect.center().x;
        let center_y = canvas_rect.center().y;
        let x = (pos.x - center_x) as f64 / self.zoom as f64 - self.pan[0] as f64;
        let y = -(pos.y - center_y) as f64 / self.zoom as f64 - self.pan[1] as f64;
        [x, y]
    }

    /// Zoom/pan so the start, goal and path are all visible.
    fn perform_zoom_to_fit(&mut self, canvas_rect: egui::Rect) {
        if canvas_rect.width() < 1.0 || canvas_rect.height() < 1.0 {
            return;
        }
        let mut min_x = f64::INFINITY;
        let mut min_y = f64::INFINITY;
        let mut max_x = f64::NEG_INFINITY;
        let mut max_y = f64::NEG_INFINITY;
        let mut update = |x: f64, y: f64| {
            if x < min_x {
                min_x = x;
            }
            if x > max_x {
                max_x = x;
            }
            if y < min_y {
                min_y = y;
            }
            if y > max_y {
                max_y = y;
            }
        };
        update(self.start_pos[0], self.start_pos[1]);
        update(self.goal_pos[0], self.goal_pos[1]);
        update(0.0, 0.0);
        update(
            self.map_size as f64 * self.config.grid_resolution,
            self.map_size as f64 * self.config.grid_resolution,
        );
        if let Some(result) = &self.result {
            if let Some(path) = &result.path {
                for n in path {
                    update(n.state.x, n.state.y);
                }
            }
        }
        let pad = 2.0_f64;
        let w = (max_x - min_x + pad * 2.0).max(1.0);
        let h = (max_y - min_y + pad * 2.0).max(1.0);
        let zx = (canvas_rect.width() as f64) / w;
        let zy = (canvas_rect.height() as f64) / h;
        self.zoom = (zx.min(zy) as f32).clamp(2.0, 80.0);
        let cx = (min_x + max_x) * 0.5;
        let cy = (min_y + max_y) * 0.5;
        self.pan = [-(cx as f32), -(cy as f32)];
    }

    fn draw_vehicle_shape(
        &self,
        painter: &egui::Painter,
        pos: [f64; 2],
        yaw: f64,
        rect: egui::Rect,
        body: egui::Color32,
        arrow: egui::Color32,
        scale: f32,
    ) {
        let length = (self.config.grid_resolution * 4.0) as f32 * scale;
        let width = (self.config.grid_resolution * 2.0) as f32 * scale;
        let center = self.world_to_screen(pos, rect);
        let cos_y = yaw.cos() as f32;
        let sin_y = yaw.sin() as f32;
        let hl = length * 0.5;
        let hw = width * 0.5;
        let corners = [[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]];
        let pts: Vec<egui::Pos2> = corners
            .iter()
            .map(|c| {
                let wx = pos[0] as f32 + c[0] * cos_y - c[1] * sin_y;
                let wy = pos[1] as f32 + c[0] * sin_y + c[1] * cos_y;
                self.world_to_screen([wx as f64, wy as f64], rect)
            })
            .collect();
        painter.add(egui::Shape::convex_polygon(
            pts,
            body,
            egui::Stroke::new(1.0, egui::Color32::BLACK),
        ));
        let front = self.world_to_screen(
            [
                pos[0] + (length * 0.7) as f64 * cos_y as f64,
                pos[1] + (length * 0.7) as f64 * sin_y as f64,
            ],
            rect,
        );
        painter.line_segment([center, front], egui::Stroke::new(2.0, arrow));
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        self.check_planning_result();
        if self.is_planning {
            ctx.request_repaint_after(std::time::Duration::from_millis(100));
        }

        // ===== Left control panel =====
        egui::SidePanel::left("controls")
            .min_width(280.0)
            .show(ctx, |ui| {
                ui.heading("Hybrid A* Demo");
                ui.separator();

                ui.label("Scenario:");
                ui.horizontal_wrapped(|ui| {
                    if ui
                        .selectable_label(self.scenario == Scenario::BasicNavigation, "Basic")
                        .clicked()
                    {
                        self.scenario = Scenario::BasicNavigation;
                        self.generate_scenario();
                    }
                    if ui
                        .selectable_label(self.scenario == Scenario::Parking, "Parking")
                        .clicked()
                    {
                        self.scenario = Scenario::Parking;
                        self.generate_scenario();
                    }
                    if ui
                        .selectable_label(self.scenario == Scenario::UTurn, "U-Turn")
                        .clicked()
                    {
                        self.scenario = Scenario::UTurn;
                        self.generate_scenario();
                    }
                    if ui
                        .selectable_label(self.scenario == Scenario::Custom, "Custom")
                        .clicked()
                    {
                        self.scenario = Scenario::Custom;
                        self.generate_scenario();
                    }
                });

                ui.separator();

                let (start_status, goal_status) = self.get_position_status();
                let start_color = if self.is_in_collision(self.start_pos) {
                    egui::Color32::RED
                } else {
                    egui::Color32::GREEN
                };
                ui.colored_label(start_color, format!("Start: {}", start_status));
                ui.horizontal(|ui| {
                    ui.label("X:");
                    ui.add(
                        egui::DragValue::new(&mut self.start_pos[0])
                            .speed(0.5)
                            .range(0.0..=80.0),
                    );
                    ui.label("Y:");
                    ui.add(
                        egui::DragValue::new(&mut self.start_pos[1])
                            .speed(0.5)
                            .range(0.0..=80.0),
                    );
                });
                ui.horizontal(|ui| {
                    ui.label("Yaw:");
                    ui.add(
                        egui::DragValue::new(&mut self.start_yaw)
                            .speed(0.1)
                            .range(-std::f64::consts::PI..=std::f64::consts::PI),
                    );
                });

                ui.separator();

                let goal_color = if self.is_in_collision(self.goal_pos) {
                    egui::Color32::RED
                } else {
                    egui::Color32::GREEN
                };
                ui.colored_label(goal_color, format!("Goal: {}", goal_status));
                ui.horizontal(|ui| {
                    ui.label("X:");
                    ui.add(
                        egui::DragValue::new(&mut self.goal_pos[0])
                            .speed(0.5)
                            .range(0.0..=80.0),
                    );
                    ui.label("Y:");
                    ui.add(
                        egui::DragValue::new(&mut self.goal_pos[1])
                            .speed(0.5)
                            .range(0.0..=80.0),
                    );
                });
                ui.horizontal(|ui| {
                    ui.label("Yaw:");
                    ui.add(
                        egui::DragValue::new(&mut self.goal_yaw)
                            .speed(0.1)
                            .range(-std::f64::consts::PI..=std::f64::consts::PI),
                    );
                });

                ui.separator();

                ui.label("Algorithm Parameters:");
                ui.horizontal(|ui| {
                    ui.label("Grid Res:");
                    ui.add(
                        egui::DragValue::new(&mut self.config.grid_resolution)
                            .speed(0.1)
                            .range(0.1..=2.0),
                    );
                });
                ui.horizontal(|ui| {
                    ui.label("Velocity:");
                    ui.add(
                        egui::DragValue::new(&mut self.config.velocity)
                            .speed(0.5)
                            .range(0.5..=5.0),
                    );
                });
                ui.horizontal(|ui| {
                    ui.label("Sim Time:");
                    ui.add(
                        egui::DragValue::new(&mut self.config.simulation_time)
                            .speed(0.1)
                            .range(0.1..=2.0),
                    );
                });
                ui.horizontal(|ui| {
                    ui.label("Pos Tol:");
                    ui.add(
                        egui::DragValue::new(&mut self.config.position_tolerance)
                            .speed(0.1)
                            .range(0.1..=5.0),
                    );
                });
                ui.horizontal(|ui| {
                    ui.label("Ang Tol:");
                    ui.add(
                        egui::DragValue::new(&mut self.config.angle_tolerance)
                            .speed(0.05)
                            .range(0.05..=1.5),
                    );
                });

                ui.separator();

                ui.label("Cost Weights:");
                ui.horizontal(|ui| {
                    ui.label("Steer:");
                    ui.add(
                        egui::DragValue::new(&mut self.weights.w_steer)
                            .speed(1.0)
                            .range(0.0..=50.0),
                    );
                });
                ui.horizontal(|ui| {
                    ui.label("Turn:");
                    ui.add(
                        egui::DragValue::new(&mut self.weights.w_turn)
                            .speed(1.0)
                            .range(0.0..=50.0),
                    );
                });
                ui.horizontal(|ui| {
                    ui.label("Cusp:");
                    ui.add(
                        egui::DragValue::new(&mut self.weights.w_cusp)
                            .speed(1.0)
                            .range(0.0..=100.0),
                    );
                });

                ui.separator();

                ui.label("Visualization:");
                ui.checkbox(&mut self.show_explored, "Show Explored Nodes");
                ui.checkbox(&mut self.show_trajectories, "Show Primitive Trajectories");
                ui.checkbox(&mut self.show_grid, "Show Grid");
                ui.checkbox(&mut self.show_vehicle_shape, "Show Vehicle Shapes");
                ui.checkbox(&mut self.show_footprint, "Show Footprint on Hover");
                ui.checkbox(&mut self.show_start_goal_arrows, "Show Start/Goal Arrows");

                ui.separator();

                ui.horizontal(|ui| {
                    let run_btn =
                        ui.add_enabled(!self.is_planning, egui::Button::new("Run Planning"));
                    if run_btn.clicked() {
                        self.run_planning();
                    }
                    if ui.button("Reset View").clicked() {
                        self.zoom = 12.0;
                        self.pan = [0.0, 0.0];
                    }
                });
                if ui.button("Fit View").clicked() {
                    self.zoom_to_fit_pending = true;
                }

                ui.separator();

                if self.is_planning {
                    ui.colored_label(
                        egui::Color32::from_rgb(255, 165, 0),
                        format!("{} (running in background)", self.progress_msg),
                    );
                } else if !self.progress_msg.is_empty() {
                    ui.label(&self.progress_msg);
                }

                if let Some(ref result) = self.result {
                    ui.separator();
                    ui.label("Results:");
                    if result.stats.success {
                        ui.colored_label(egui::Color32::GREEN, "Path Found");
                    } else {
                        ui.colored_label(egui::Color32::RED, "Planning Failed");
                    }
                    ui.label(format!("  Iterations: {}", result.stats.iterations));
                    ui.label(format!("  Time: {:.2} ms", result.stats.planning_time_ms));
                    ui.label(format!("  Explored: {}", result.stats.nodes_explored));
                    if let Some(length) = result.stats.path_length {
                        ui.label(format!("  Path Length: {:.2} m", length));
                    }
                    ui.label(format!(
                        "  Direction Changes: {}",
                        result.stats.direction_changes
                    ));
                    if let Some(ref path) = result.path {
                        ui.label(format!("  Path Points: {}", path.len()));
                    }
                    if let Some(ref error_msg) = result.stats.error_message {
                        ui.separator();
                        ui.colored_label(
                            egui::Color32::from_rgb(255, 165, 0),
                            format!("{}", error_msg),
                        );
                    }
                }
            });

        // ===== Central canvas =====
        egui::CentralPanel::default().show(ctx, |ui| {
            let (response, painter) = ui.allocate_painter(
                egui::Vec2::new(ui.available_width(), ui.available_height()),
                egui::Sense::click_and_drag(),
            );
            let rect = response.rect;

            // Fit-view deferred until we know the rect size
            if self.zoom_to_fit_pending {
                self.zoom_to_fit_pending = false;
                self.perform_zoom_to_fit(rect);
            }

            // Zoom on scroll
            if response.hovered() {
                let scroll = ui.input(|i| i.smooth_scroll_delta.y);
                if scroll != 0.0 {
                    let factor = if scroll > 0.0 { 1.1 } else { 0.9 };
                    self.zoom = (self.zoom * factor).clamp(2.0, 80.0);
                }
            }

            // Pan: middle-drag, or shift+left-drag
            if response.dragged_by(egui::PointerButton::Middle)
                || (response.dragged_by(egui::PointerButton::Primary)
                    && ui.input(|i| i.modifiers.shift))
            {
                let drag = response.drag_delta();
                self.pan[0] += drag.x / self.zoom;
                self.pan[1] -= drag.y / self.zoom;
            }

            // Background
            painter.rect_filled(rect, 0.0, egui::Color32::from_rgb(245, 245, 248));

            // Grid
            if self.show_grid {
                let cell_world = self.config.grid_resolution;
                let step_cells: i32 = if self.zoom * cell_world as f32 > 6.0 {
                    1
                } else if self.zoom * cell_world as f32 * 2.0 > 6.0 {
                    2
                } else {
                    5
                };
                let cells = (self.map_size / step_cells as usize) as i32;
                for i in 0..=cells {
                    let wx = (i * step_cells) as f64 * cell_world + self.map_offset[0];
                    let wy = (i * step_cells) as f64 * cell_world + self.map_offset[1];
                    let p1 = self.world_to_screen([wx, self.map_offset[1]], rect);
                    let p2 = self.world_to_screen(
                        [wx, self.map_size as f64 * cell_world + self.map_offset[1]],
                        rect,
                    );
                    painter.line_segment(
                        [p1, p2],
                        egui::Stroke::new(
                            0.5,
                            egui::Color32::from_rgba_premultiplied(200, 200, 220, 200),
                        ),
                    );
                    let p3 = self.world_to_screen([self.map_offset[0], wy], rect);
                    let p4 = self.world_to_screen(
                        [self.map_size as f64 * cell_world + self.map_offset[0], wy],
                        rect,
                    );
                    painter.line_segment(
                        [p3, p4],
                        egui::Stroke::new(
                            0.5,
                            egui::Color32::from_rgba_premultiplied(200, 200, 220, 200),
                        ),
                    );
                }
                if self.zoom > 4.0 {
                    let label_step = 10.0_f64;
                    let mut v = 0.0;
                    while v <= self.map_size as f64 * cell_world {
                        let p_x = self.world_to_screen([v, self.map_offset[1]], rect);
                        let p_y = self.world_to_screen([self.map_offset[0], v], rect);
                        painter.text(
                            p_x,
                            egui::Align2::CENTER_TOP,
                            format!("{:.0}", v),
                            egui::FontId::proportional(10.0),
                            egui::Color32::from_rgb(120, 120, 140),
                        );
                        painter.text(
                            p_y,
                            egui::Align2::LEFT_CENTER,
                            format!("{:.0}", v),
                            egui::FontId::proportional(10.0),
                            egui::Color32::from_rgb(120, 120, 140),
                        );
                        v += label_step;
                    }
                }
            }

            // Obstacles
            for (y, row) in self.obstacle_map.iter().enumerate() {
                for (x, &is_obstacle) in row.iter().enumerate() {
                    if is_obstacle {
                        let world_x = x as f64 * self.config.grid_resolution + self.map_offset[0];
                        let world_y = y as f64 * self.config.grid_resolution + self.map_offset[1];
                        let pos = self.world_to_screen([world_x, world_y], rect);
                        let s = self.zoom as f32 * 0.95;
                        painter.rect_filled(
                            egui::Rect::from_center_size(pos, egui::vec2(s, s)),
                            1.0,
                            egui::Color32::from_rgb(60, 60, 75),
                        );
                    }
                }
            }

            // Explored nodes
            if self.show_explored {
                if let Some(ref result) = self.result {
                    let max_drawn = 8000usize;
                    let step = (result.explored_nodes.len() / max_drawn.max(1)).max(1);
                    for (i, node) in result.explored_nodes.iter().enumerate() {
                        if i % step != 0 {
                            continue;
                        }
                        let pos = self.world_to_screen([node.x, node.y], rect);
                        let color = match node.direction {
                            Direction::Forward => {
                                egui::Color32::from_rgba_premultiplied(80, 140, 230, 90)
                            }
                            Direction::Backward => {
                                egui::Color32::from_rgba_premultiplied(230, 140, 80, 90)
                            }
                            Direction::None => {
                                egui::Color32::from_rgba_premultiplied(150, 150, 150, 80)
                            }
                        };
                        painter.circle_filled(pos, 2.5, color);
                    }
                }
            }

            // Forward-simulation trajectories
            if self.show_trajectories {
                if let Some(ref result) = self.result {
                    for trajectory in &result.trajectories {
                        if trajectory.len() > 1 {
                            let points: Vec<egui::Pos2> = trajectory
                                .iter()
                                .map(|s| self.world_to_screen([s.x, s.y], rect))
                                .collect();
                            painter.add(egui::Shape::line(
                                points,
                                egui::Stroke::new(
                                    0.6,
                                    egui::Color32::from_rgba_premultiplied(150, 200, 255, 50),
                                ),
                            ));
                        }
                    }
                }
            }

            // Path
            if let Some(ref result) = self.result {
                if let Some(ref path) = result.path {
                    if path.len() > 1 {
                        let points: Vec<egui::Pos2> = path
                            .iter()
                            .map(|n| self.world_to_screen([n.state.x, n.state.y], rect))
                            .collect();
                        // Halo
                        painter.add(egui::Shape::line(
                            points.clone(),
                            egui::Stroke::new(
                                7.0,
                                egui::Color32::from_rgba_premultiplied(0, 0, 0, 60),
                            ),
                        ));
                        // Main path
                        painter.add(egui::Shape::line(
                            points.clone(),
                            egui::Stroke::new(3.0, egui::Color32::from_rgb(0, 200, 0)),
                        ));
                        for n in path {
                            let pos = self.world_to_screen([n.state.x, n.state.y], rect);
                            let c = match n.state.direction {
                                Direction::Forward => egui::Color32::from_rgb(0, 200, 0),
                                Direction::Backward => egui::Color32::from_rgb(200, 140, 0),
                                Direction::None => egui::Color32::from_rgb(120, 120, 120),
                            };
                            painter.circle_filled(pos, 3.0, c);
                            let yaw = n.state.yaw as f32;
                            let tip = self.world_to_screen(
                                [
                                    n.state.x + 0.6 * yaw.cos() as f64,
                                    n.state.y + 0.6 * yaw.sin() as f64,
                                ],
                                rect,
                            );
                            painter.line_segment([pos, tip], egui::Stroke::new(1.2, c));
                        }
                    }
                }
            }

            // Vehicle shapes along path
            if self.show_vehicle_shape {
                if let Some(ref result) = self.result {
                    if let Some(ref path) = result.path {
                        let step = (path.len() / 12).max(1);
                        for (i, n) in path.iter().enumerate() {
                            if i % step != 0 && i != path.len() - 1 {
                                continue;
                            }
                            let body_color = match n.state.direction {
                                Direction::Forward => {
                                    egui::Color32::from_rgba_premultiplied(0, 180, 0, 180)
                                }
                                Direction::Backward => {
                                    egui::Color32::from_rgba_premultiplied(200, 140, 0, 180)
                                }
                                Direction::None => {
                                    egui::Color32::from_rgba_premultiplied(120, 120, 120, 180)
                                }
                            };
                            self.draw_vehicle_shape(
                                &painter,
                                [n.state.x, n.state.y],
                                n.state.yaw,
                                rect,
                                body_color,
                                egui::Color32::WHITE,
                                0.6,
                            );
                        }
                    }
                }
            }

            // Start/goal pose markers
            self.draw_vehicle_shape(
                &painter,
                self.start_pos,
                self.start_yaw,
                rect,
                egui::Color32::from_rgb(0, 200, 0),
                egui::Color32::WHITE,
                1.0,
            );
            self.draw_vehicle_shape(
                &painter,
                self.goal_pos,
                self.goal_yaw,
                rect,
                egui::Color32::from_rgb(220, 40, 40),
                egui::Color32::WHITE,
                1.0,
            );
            let s_start = self.world_to_screen(self.start_pos, rect);
            let s_goal = self.world_to_screen(self.goal_pos, rect);
            painter.text(
                s_start,
                egui::Align2::CENTER_CENTER,
                "S",
                egui::FontId::proportional(13.0),
                egui::Color32::WHITE,
            );
            painter.text(
                s_goal,
                egui::Align2::CENTER_CENTER,
                "G",
                egui::FontId::proportional(13.0),
                egui::Color32::WHITE,
            );

            // Hover footprint preview
            if self.show_footprint {
                if let Some(hover) = response.hover_pos() {
                    let wp = self.screen_to_world(hover, rect);
                    let yaw_guess = (self.goal_pos[0] - self.start_pos[0])
                        .atan2(self.goal_pos[1] - self.start_pos[1]);
                    let length = self.config.grid_resolution * 4.0;
                    let width = self.config.grid_resolution * 2.0;
                    let hl = length * 0.5;
                    let hw = width * 0.5;
                    let cos_y = yaw_guess.cos();
                    let sin_y = yaw_guess.sin();
                    let corners = [[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]];
                    let pts: Vec<egui::Pos2> = corners
                        .iter()
                        .map(|c| {
                            let wx = wp[0] + c[0] * cos_y - c[1] * sin_y;
                            let wy = wp[1] + c[0] * sin_y + c[1] * cos_y;
                            self.world_to_screen([wx, wy], rect)
                        })
                        .collect();
                    painter.add(egui::Shape::convex_polygon(
                        pts,
                        egui::Color32::from_rgba_premultiplied(120, 120, 220, 80),
                        egui::Stroke::new(1.0, egui::Color32::from_rgb(80, 80, 200)),
                    ));
                }
            }

            // Click to set start/goal
            if response.clicked() {
                if let Some(click_pos) = response.interact_pointer_pos() {
                    let world_pos = self.screen_to_world(click_pos, rect);
                    if ui.input(|i| i.modifiers.shift || i.modifiers.ctrl || i.modifiers.command) {
                        self.goal_pos = world_pos;
                    } else {
                        self.start_pos = world_pos;
                    }
                }
            }

            // Instructions
            let instructions = [
                "Left-click: set Start  |  Shift/Ctrl+click: set Goal",
                "Drag with shift, or middle-drag: pan  |  Scroll: zoom",
            ];
            let mut y = rect.min.y + 8.0;
            for instr in instructions {
                painter.text(
                    egui::pos2(rect.min.x + 10.0, y),
                    egui::Align2::LEFT_TOP,
                    instr,
                    egui::FontId::proportional(13.0),
                    egui::Color32::from_rgb(60, 60, 80),
                );
                y += 18.0;
            }
            // Legend
            let legend_items = [
                ("Start", egui::Color32::from_rgb(0, 200, 0)),
                ("Goal", egui::Color32::from_rgb(220, 40, 40)),
                ("Path (Fwd)", egui::Color32::from_rgb(0, 200, 0)),
                ("Path (Rev)", egui::Color32::from_rgb(200, 140, 0)),
                (
                    "Explored (Fwd)",
                    egui::Color32::from_rgba_premultiplied(80, 140, 230, 200),
                ),
                (
                    "Explored (Rev)",
                    egui::Color32::from_rgba_premultiplied(230, 140, 80, 200),
                ),
            ];
            let legend_x = rect.min.x + 10.0;
            let mut ly = rect.max.y - 18.0 * (legend_items.len() as f32) - 8.0;
            for (label, color) in legend_items {
                painter.rect_filled(
                    egui::Rect::from_min_size(
                        egui::pos2(legend_x, ly + 4.0),
                        egui::vec2(12.0, 10.0),
                    ),
                    1.0,
                    color,
                );
                painter.text(
                    egui::pos2(legend_x + 18.0, ly + 3.0),
                    egui::Align2::LEFT_TOP,
                    label,
                    egui::FontId::proportional(12.0),
                    egui::Color32::from_rgb(40, 40, 50),
                );
                ly += 18.0;
            }
        });
    }
}
