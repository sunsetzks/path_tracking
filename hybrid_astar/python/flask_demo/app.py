"""Flask web demo for the Hybrid A* path planner.

Routes
------
- ``GET  /``                       — single-page HTML UI
- ``GET  /api/health``             — ``{"ok": true}``
- ``GET  /api/scenarios``          — list of scenarios with metadata
- ``POST /api/plan``               — run the planner, return JSON + search PNG
- ``POST /api/render``             — render an already-planned path to PNG

Run via ``python -m flask_demo.run`` (port 5000).
"""

from __future__ import annotations

import base64
import io
import json
import logging
from collections import OrderedDict
from threading import Lock
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

# Matplotlib must use a non-interactive backend before pyplot is imported
# anywhere else, so we set it before importing the planner (which uses
# matplotlib in its visualizer).
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

from flask import Flask, jsonify, render_template, request

# The package is not pip-installable (no PEP 660 backend), so we rely on
# the caller having ``hybrid_astar/python`` on ``PYTHONPATH``.
from astar_project.hybrid_astar import (
    DirectionMode,
    HybridAStar,
    State,
    VehicleModel,
)
from astar_project.visualizer import HybridAStarVisualizer

from .scenarios import Scenario, get_scenario, list_scenarios

log = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

DIRECTION_FROM_INT = {1: DirectionMode.FORWARD, -1: DirectionMode.BACKWARD}
INT_FROM_DIRECTION = {v: k for k, v in DIRECTION_FROM_INT.items()}


def state_to_dict(s: State) -> Dict[str, Any]:
    """Serialize a :class:`State` to JSON-safe dict."""
    return {
        "x": float(s.x),
        "y": float(s.y),
        "yaw": float(s.yaw),
        "steer": float(s.steer),
        "direction": INT_FROM_DIRECTION.get(s.direction, 0),
    }


def pose_from_dict(d: Dict[str, Any]) -> Tuple[float, float, float, DirectionMode]:
    """Deserialize a pose dict into ``(x, y, yaw, direction)``."""
    direction = DIRECTION_FROM_INT.get(int(d.get("direction", 1)),
                                       DirectionMode.FORWARD)
    return (float(d["x"]), float(d["y"]), float(d["yaw"]), direction)


def encode_png_bytes(fig) -> str:
    """Serialize a matplotlib ``Figure`` to a base64 PNG string."""
    buf = io.BytesIO()
    fig.savefig(buf, format="png", bbox_inches="tight", dpi=100)
    plt.close(fig)
    return base64.b64encode(buf.getvalue()).decode("ascii")


def encode_array_png(arr: np.ndarray, cmap: str = "Greys") -> str:
    """Render a 2D numpy array to a base64 PNG using matplotlib."""
    fig, ax = plt.subplots(figsize=(4, 4))
    ax.imshow(arr, cmap=cmap, origin="upper", interpolation="nearest")
    ax.set_xticks([])
    ax.set_yticks([])
    fig.tight_layout(pad=0)
    return encode_png_bytes(fig)


def grid_cells(grid: np.ndarray) -> List[List[int]]:
    """Return ``[[i, j], ...]`` for every obstacle cell (value == 1)."""
    ijs = np.argwhere(grid > 0)
    return [[int(i), int(j)] for i, j in ijs]


# ---------------------------------------------------------------------------
# Planner cache
# ---------------------------------------------------------------------------

class _PlannerCache:
    """Tiny LRU cache of ``HybridAStar`` instances keyed by
    ``(grid_bytes, grid_resolution, params)`` so repeated identical
    plan requests reuse the obstacle-map setup."""

    def __init__(self, capacity: int = 4) -> None:
        self._capacity = capacity
        self._lock = Lock()
        self._store: "OrderedDict[Tuple, HybridAStar]" = OrderedDict()

    def _key(self, grid: np.ndarray, origin_x: float, origin_y: float,
             params: Dict[str, Any]) -> Tuple:
        # ``grid.tobytes()`` is fast and hashable; combined with
        # origin/params it uniquely identifies the planner state.
        return (grid.tobytes(), float(origin_x), float(origin_y),
                tuple(sorted(params.items())))

    def get_or_create(self, grid: np.ndarray, origin_x: float, origin_y: float,
                      params: Dict[str, Any], weights: Dict[str, float],
                      vehicle: VehicleModel) -> HybridAStar:
        key = self._key(grid, origin_x, origin_y, {**params, **weights})
        with self._lock:
            if key in self._store:
                self._store.move_to_end(key)
                return self._store[key]
            planner = HybridAStar(
                vehicle_model=vehicle,
                grid_resolution=float(params["grid_resolution"]),
                angle_resolution=float(params["angle_resolution"]),
                steer_resolution=float(params["steer_resolution"]),
                velocity=float(params["velocity"]),
                simulation_time=float(params["simulation_time"]),
                dt=float(params["dt"]),
            )
            planner.w_steer = float(weights["w_steer"])
            planner.w_turn = float(weights["w_turn"])
            planner.w_cusp = float(weights["w_cusp"])
            planner.set_obstacle_map(grid, origin_x=origin_x, origin_y=origin_y)
            self._store[key] = planner
            while len(self._store) > self._capacity:
                self._store.popitem(last=False)
            return planner


_CACHE = _PlannerCache(capacity=4)


# ---------------------------------------------------------------------------
# Visualization
# ---------------------------------------------------------------------------

def _render_search_png(planner: HybridAStar, start: State, goal: State,
                       path_nodes: List[Any]) -> str:
    """Render the planner's ``HybridAStarVisualizer`` figure to base64 PNG.

    We use the visualizer in two phases: it builds the figure and stores
    it on ``self.fig``; we then ``savefig`` it ourselves so the same code
    works under the Agg backend without ``plt.show()``.
    """
    visualizer = HybridAStarVisualizer()
    # We always pass the original path nodes so the visualizer can
    # reconstruct the search tree. ``show_costs=False`` keeps the figure
    # compact for the web UI.
    visualizer.visualize_node_path(
        path_nodes=path_nodes,
        start=start,
        goal=goal,
        planner_instance=planner,
        explored_nodes=planner.explored_nodes,
        simulation_trajectories=planner.simulation_trajectories,
        obstacle_map=planner.obstacle_map,
        map_origin_x=planner.map_origin_x,
        map_origin_y=planner.map_origin_y,
        grid_resolution=planner.grid_resolution,
        vehicle_model=planner.vehicle_model,
        show_exploration=True,
        show_trajectories=True,
        show_costs=False,
    )
    if visualizer.fig is None:
        # Defensive: if the visualizer bailed out, return a blank PNG.
        fig, _ = plt.subplots()
        return encode_png_bytes(fig)
    return encode_png_bytes(visualizer.fig)


def _render_path_png(grid: np.ndarray, origin_x: float, origin_y: float,
                     grid_resolution: float, waypoints: List[Dict[str, Any]],
                     start: Dict[str, Any], goal: Dict[str, Any]) -> str:
    """Render a clean path-only PNG (no search-tree clutter)."""
    fig, ax = plt.subplots(figsize=(6, 6))
    h, w = grid.shape
    extent = (origin_x, origin_x + w * grid_resolution,
              origin_y, origin_y + h * grid_resolution)
    ax.imshow(grid, cmap="Greys", origin="upper", interpolation="nearest",
              extent=extent, vmin=0, vmax=1, alpha=0.6)
    if waypoints:
        xs = [w["x"] for w in waypoints]
        ys = [w["y"] for w in waypoints]
        # Color by |steer|
        steers = np.array([abs(w["steer"]) for w in waypoints])
        if steers.max() > 0:
            norm = steers / steers.max()
        else:
            norm = np.zeros_like(steers)
        for k in range(len(xs) - 1):
            ax.plot(xs[k:k + 2], ys[k:k + 2],
                    color=plt.cm.cool(norm[k]), linewidth=2.0)
        ax.plot(xs, ys, color="black", linewidth=0.5, alpha=0.4)
    # Start (green triangle) and goal (red star)
    ax.scatter([start["x"]], [start["y"]], marker="^", s=160,
               c="lime", edgecolors="black", linewidths=1.0, zorder=5,
               label="start")
    ax.scatter([goal["x"]], [goal["y"]], marker="*", s=220,
               c="red", edgecolors="black", linewidths=1.0, zorder=5,
               label="goal")
    ax.set_aspect("equal", adjustable="box")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=9)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Planned Path")
    fig.tight_layout()
    return encode_png_bytes(fig)


# ---------------------------------------------------------------------------
# Flask app factory
# ---------------------------------------------------------------------------

def create_app() -> Flask:
    app = Flask(__name__, template_folder="templates", static_folder="static")
    logging.basicConfig(level=logging.INFO)

    # ---- HTML index ----
    @app.get("/")
    def index():
        return render_template("index.html")

    # ---- Health ----
    @app.get("/api/health")
    def health():
        return jsonify({"ok": True})

    # ---- Scenarios ----
    @app.get("/api/scenarios")
    def scenarios():
        out = []
        for s in list_scenarios():
            meta = s.to_metadata()
            meta["obstacle_cells"] = grid_cells(s.grid)
            meta["occupancy_png"] = encode_array_png(s.grid)
            out.append(meta)
        return jsonify(out)

    # ---- Plan ----
    @app.post("/api/plan")
    def plan():
        body = request.get_json(silent=True) or {}
        try:
            scenario_name = body.get("scenario", "basic")
            scenario: Scenario = get_scenario(scenario_name)
        except KeyError as e:
            return jsonify({"error": str(e)}), 400

        # Resolve grid: client may send an override (list-of-lists of 0/1)
        grid_override = body.get("grid_override")
        if grid_override is not None:
            try:
                grid = np.asarray(grid_override, dtype=np.uint8)
            except (TypeError, ValueError):
                return jsonify({"error": "grid_override must be a 2D array"}), 400
            if grid.ndim != 2 or grid.shape != scenario.grid.shape:
                return jsonify({
                    "error": "grid_override shape must match the scenario",
                    "expected": list(scenario.grid.shape),
                    "got": list(grid.shape),
                }), 400
            origin_x = float(body.get("origin_x", scenario.origin_x))
            origin_y = float(body.get("origin_y", scenario.origin_y))
        else:
            grid = scenario.grid
            origin_x = scenario.origin_x
            origin_y = scenario.origin_y

        # Resolve params + weights
        params = {
            "grid_resolution": scenario.grid_resolution,
            "angle_resolution": scenario.angle_resolution,
            "steer_resolution": scenario.steer_resolution,
            "velocity": scenario.velocity,
            "simulation_time": scenario.simulation_time,
            "dt": scenario.dt,
            "max_iterations": scenario.max_iterations,
            **(body.get("params") or {}),
        }
        weights = {
            "w_steer": scenario.w_steer,
            "w_turn": scenario.w_turn,
            "w_cusp": scenario.w_cusp,
            **(body.get("weights") or {}),
        }

        # Resolve start / goal
        if "start" in body and body["start"] is not None:
            sx, sy, syaw, sdir = pose_from_dict(body["start"])
        else:
            sx, sy, syaw, sdir = (scenario.start.x, scenario.start.y,
                                   scenario.start.yaw, scenario.start.direction)
        if "goal" in body and body["goal"] is not None:
            gx, gy, gyaw, gdir = pose_from_dict(body["goal"])
        else:
            gx, gy, gyaw, gdir = (scenario.goal.x, scenario.goal.y,
                                  scenario.goal.yaw, scenario.goal.direction)

        start = State(x=sx, y=sy, yaw=syaw, direction=sdir)
        goal = State(x=gx, y=gy, yaw=gyaw, direction=gdir)

        # Build planner (cached) and plan
        vehicle = VehicleModel(wheelbase=2.5, max_steer=np.pi / 3)
        try:
            planner = _CACHE.get_or_create(grid, origin_x, origin_y,
                                           params, weights, vehicle)
        except KeyError as e:
            return jsonify({"error": f"Missing parameter: {e}"}), 400

        # Collision check for start/goal
        if not planner.is_collision_free(start):
            return jsonify({
                "error": "Start pose is in collision or out of bounds",
                "start": state_to_dict(start),
            }), 400
        if not planner.is_collision_free(goal):
            return jsonify({
                "error": "Goal pose is in collision or out of bounds",
                "goal": state_to_dict(goal),
            }), 400

        max_iterations = int(params.get("max_iterations",
                                        scenario.max_iterations))
        path_nodes = planner.plan_path(start, goal,
                                       max_iterations=max_iterations)
        if path_nodes is None:
            return jsonify({
                "success": False,
                "error": "No path found within max_iterations",
                "stats": {
                    "max_iterations": max_iterations,
                    "nodes_explored": len(planner.explored_nodes),
                },
            }), 200

        detailed = planner.extract_detailed_path(path_nodes)
        waypoints = [state_to_dict(s) for s in detailed]
        stats = planner.get_statistics(path_nodes)
        timing = planner.get_timing_stats()
        path_png = _render_path_png(grid, origin_x, origin_y,
                                    float(params["grid_resolution"]),
                                    waypoints,
                                    state_to_dict(start),
                                    state_to_dict(goal))
        search_png = _render_search_png(planner, start, goal, path_nodes)

        return jsonify({
            "success": True,
            "scenario": scenario_name,
            "waypoints": waypoints,
            "statistics": {
                "path_nodes": len(path_nodes),
                "detailed_waypoints": len(detailed),
                "nodes_explored": len(planner.explored_nodes),
                "max_iterations": max_iterations,
                **stats,
                **timing,
            },
            "images": {
                "path_png": path_png,
                "search_png": search_png,
            },
        })

    # ---- Render an existing path to PNG ----
    @app.post("/api/render")
    def render():
        body = request.get_json(silent=True) or {}
        try:
            scenario: Scenario = get_scenario(body.get("scenario", "basic"))
        except KeyError as e:
            return jsonify({"error": str(e)}), 400

        waypoints = body.get("waypoints") or []
        start_d = body.get("start") or {"x": scenario.start.x,
                                         "y": scenario.start.y,
                                         "yaw": scenario.start.yaw}
        goal_d = body.get("goal") or {"x": scenario.goal.x,
                                       "y": scenario.goal.y,
                                       "yaw": scenario.goal.yaw}
        if not waypoints:
            return jsonify({"error": "waypoints must be a non-empty list"}), 400

        path_png = _render_path_png(
            scenario.grid, scenario.origin_x, scenario.origin_y,
            scenario.grid_resolution, waypoints, start_d, goal_d,
        )
        return jsonify({"image": path_png})

    return app


# Module-level WSGI app for ``flask --app flask_demo.app run``
app = create_app()
