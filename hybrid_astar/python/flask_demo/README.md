# Hybrid A* Flask Web Demo

A small Flask web app that exposes the `HybridAStar` planner over HTTP/JSON
and ships a vanilla-JS single-page UI for interactive planning.

## Features

- **Three built-in scenarios** — `basic` (corridor with obstacles),
  `narrow` (wall with a tight gap), `u_turn` (180° maneuver).
- **Interactive map editor** — paint/erase obstacle cells directly on the
  canvas, or click to reposition the start and goal. Right-click while
  the start/goal tool is active rotates the heading to point at the
  click.
- **Live planner parameters** — adjust grid resolution, max iterations,
  velocity, simulation time, angle/steer resolution, and the
  `w_steer` / `w_turn` / `w_cusp` cost weights from the sidebar.
- **Server-rendered search tree** — the planner's `HybridAStarVisualizer`
  figure is saved to a base64 PNG and displayed in the side panel.
- **Path-only PNG** — `/api/render` produces a clean, exploration-free
  image of just the planned trajectory.

## Install

The workspace uses a uv-managed venv at `.venv/`. Install Flask (and
Pillow for the grid-to-PNG conversion) into it:

```bash
uv pip install --python /home/zks/ws/path_tracking_experiments/.venv/bin/python \
    flask pillow
```

Flask and Pillow are also listed under `flask_demo` in
[`pyproject.toml`](../pyproject.toml).

## Run

The `astar_project` package is **not** pip-installable in editable mode
(it lacks a PEP 660 build backend), so we rely on `PYTHONPATH`:

```bash
cd /home/zks/ws/path_tracking_experiments/hybrid_astar/python
source ../../.venv/bin/activate
export PYTHONPATH=.

# Option A: convenience entry point
python -m flask_demo.run

# Option B: Flask CLI
flask --app flask_demo.app run --host 127.0.0.1 --port 5000
```

Then open <http://127.0.0.1:5000/> in a browser.

## API

| Method | Path                | Description                                    |
| ------ | ------------------- | ---------------------------------------------- |
| GET    | `/`                 | Single-page HTML UI                            |
| GET    | `/api/health`       | `{"ok": true}`                                 |
| GET    | `/api/scenarios`    | List of scenarios with metadata + occupancy PNG |
| POST   | `/api/plan`         | Run the planner, return JSON + two PNGs        |
| POST   | `/api/render`       | Render a planned path to PNG                   |

### `POST /api/plan` body

```json
{
  "scenario": "basic",
  "grid_override": [[0,0,...], ...],   // optional, must match scenario shape
  "origin_x": -5.0,                     // optional, defaults to scenario
  "origin_y": -5.0,
  "start": {"x": 2.0, "y": 2.0, "yaw": 0.0, "direction": 1},
  "goal":  {"x": 22.0, "y": 20.0, "yaw": 1.57, "direction": 1},
  "params": {
    "grid_resolution": 0.5, "angle_resolution": 0.39, "steer_resolution": 0.2,
    "velocity": 3.0, "simulation_time": 0.8, "dt": 0.1, "max_iterations": 3000
  },
  "weights": {"w_steer": 8.0, "w_turn": 12.0, "w_cusp": 100.0}
}
```

### `POST /api/plan` response (success)

```json
{
  "success": true,
  "scenario": "basic",
  "waypoints": [{"x":..., "y":..., "yaw":..., "steer":..., "direction": 1}, ...],
  "statistics": {
    "path_nodes": 13, "detailed_waypoints": 97, "nodes_explored": 750,
    "total_distance": 28.8, "search_time_seconds": 0.12, ...
  },
  "images": {
    "path_png": "<base64 PNG>",
    "search_png": "<base64 PNG>"
  }
}
```

## Coordinate convention

The planner uses the same world-to-grid mapping as
`HybridAStar.set_obstacle_map`:

```
x = origin_x + j * grid_resolution
y = origin_y + (H - 1 - i) * grid_resolution
```

where `(i, j)` is the grid cell with `i` the row (0 at top) and `j` the
column (0 at left). The HTML canvas mirrors this with a y-flip so the
"up" direction in the world matches "up" on the screen.

## Tests

```bash
cd /home/zks/ws/path_tracking_experiments/hybrid_astar/python
PYTHONPATH=. pytest tests/test_flask_demo.py -v
```

The tests use Flask's test client (no live HTTP server) and cover health,
scenario listing, plan success, collision rejection, and PNG rendering.
