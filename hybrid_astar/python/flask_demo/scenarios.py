"""Built-in planning scenarios for the Flask demo.

Each scenario is a self-contained dict: obstacle grid, world origin, default
start/goal poses, planner hyperparameters, and cost weights. Scenarios are
validated at import time so a typo or a colliding default pose fails fast.

Coordinate convention
---------------------
The planner maps grid cell ``(i, j)`` to world coordinate
``(origin_x + j * grid_resolution, origin_y + (map_height - 1 - i) * grid_resolution)``
(``set_obstacle_map`` flips the rows so y increases upward in world space).
``start.x/y``, ``start.yaw`` and the goal are in world coordinates.

The fixes vs. ``examples/demo.py``
----------------------------------
The original example uses ``origin_x=-5, origin_y=-5`` and
``grid_resolution=0.3`` for a 50x50 parking grid and ``0.4`` for a 40x40
U-turn grid, which makes the world extents only ``(0, 0)..(10, 10)`` and
``(0, 0)..(14, 14)`` respectively. The example's default start/goal
poses (``(5, 25)`` for parking, ``(2, 20)`` for U-turn) sit far outside
the map, so the planner bails on iteration 1. We replace the two broken
scenarios with solvable variants:

- ``narrow``: 40x40 grid with a vertical wall that has a 5-cell gap;
  the vehicle must steer through the gap. Replaces the original
  parking lot, which the planner cannot solve in a reasonable number
  of iterations because the parking-spot aisles are too narrow.
- ``u_turn``: 50x50 grid (world extents ``(-5, -5)..(20, 20)``),
  start ``(3, 13)`` in the corridor, goal ``(14, 5)`` yaw ``pi`` in
  the same corridor. The original example's start and goal were both
  at ``(2, 20)`` (out of bounds) with opposite yaw.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Tuple

import numpy as np

# Avoid importing the planner at module load to keep this file cheap to
# import for tests / docs. We use lightweight stdlib types only.
#
# We DO import numpy for grid construction. The planner itself is imported
# lazily inside ``app.py`` to avoid a hard dependency from the scenarios
# module.

Direction = int  # 1 = forward, -1 = backward (matches ``DirectionMode``)
FORWARD: Direction = 1
BACKWARD: Direction = -1


@dataclass
class Pose:
    """Default start or goal pose in world coordinates."""

    x: float
    y: float
    yaw: float  # radians
    direction: Direction = FORWARD


@dataclass
class Scenario:
    """A predefined planning scenario."""

    name: str
    description: str
    grid: np.ndarray  # shape (H, W), 1 = obstacle
    origin_x: float
    origin_y: float
    grid_resolution: float
    start: Pose
    goal: Pose
    # Planner hyperparameters
    angle_resolution: float
    steer_resolution: float
    velocity: float
    simulation_time: float
    dt: float
    max_iterations: int
    # Cost weights
    w_steer: float
    w_turn: float
    w_cusp: float

    # Derived
    @property
    def map_width(self) -> int:
        return int(self.grid.shape[1])

    @property
    def map_height(self) -> int:
        return int(self.grid.shape[0])

    def to_metadata(self) -> Dict[str, Any]:
        """Return a JSON-serializable summary (no grid pixels)."""
        return {
            "name": self.name,
            "description": self.description,
            "map_size": [self.map_height, self.map_width],
            "origin": [self.origin_x, self.origin_y],
            "grid_resolution": self.grid_resolution,
            "start": {"x": self.start.x, "y": self.start.y,
                      "yaw": self.start.yaw, "direction": self.start.direction},
            "goal": {"x": self.goal.x, "y": self.goal.y,
                     "yaw": self.goal.yaw, "direction": self.goal.direction},
            "params": {
                "angle_resolution": self.angle_resolution,
                "steer_resolution": self.steer_resolution,
                "velocity": self.velocity,
                "simulation_time": self.simulation_time,
                "dt": self.dt,
                "max_iterations": self.max_iterations,
            },
            "weights": {
                "w_steer": self.w_steer,
                "w_turn": self.w_turn,
                "w_cusp": self.w_cusp,
            },
        }


# --- Obstacle-map builders ---------------------------------------------------

def _basic_grid() -> np.ndarray:
    """60x60 corridor with three interior obstacles (the one that works
    in ``examples/demo.py``)."""
    g = np.zeros((60, 60), dtype=np.uint8)
    g[0:5, :] = 1
    g[-5:, :] = 1
    g[:, 0:5] = 1
    g[:, -5:] = 1
    g[20:40, 15:20] = 1
    g[25:30, 30:50] = 1
    g[10:15, 35:45] = 1
    return g


def _narrow_grid() -> np.ndarray:
    """40x40 grid with a vertical wall that has a 5-cell gap.

    The vehicle must steer precisely through the gap to reach the
    goal on the other side. Replaces the original parking lot, which
    the planner cannot solve because the parking-spot aisles are
    too narrow for the discrete search.
    """
    g = np.zeros((40, 40), dtype=np.uint8)
    # Border walls
    g[0:3, :] = 1
    g[-3:, :] = 1
    g[:, 0:3] = 1
    g[:, -3:] = 1
    # Vertical wall with a gap in the middle
    g[5:18, 19:21] = 1
    g[22:35, 19:21] = 1
    return g


def _u_turn_grid() -> np.ndarray:
    """50x50 corridor for a U-turn maneuver (expanded from the
    40x40 version in ``examples/demo.py``)."""
    g = np.zeros((50, 50), dtype=np.uint8)
    g[:10, :] = 1
    g[40:, :] = 1
    g[:, :10] = 1
    g[:, 40:] = 1
    return g


# --- Helpers -----------------------------------------------------------------

def _world_to_cell(x: float, y: float, origin_x: float, origin_y: float,
                   grid_resolution: float, height: int) -> Tuple[int, int]:
    """Convert a world (x, y) to grid cell (i, j) using the same convention
    as ``HybridAStar.set_obstacle_map``."""
    j = int(round((x - origin_x) / grid_resolution))
    i = int(round((height - 1) - (y - origin_y) / grid_resolution))
    return i, j


def _assert_free(s: Scenario, pose: Pose, label: str) -> None:
    """Raise ``ValueError`` if a default pose sits on an obstacle or out of
    bounds. Catches typos at import time."""
    i, j = _world_to_cell(pose.x, pose.y, s.origin_x, s.origin_y,
                          s.grid_resolution, s.map_height)
    h, w = s.grid.shape
    if not (0 <= i < h and 0 <= j < w):
        raise ValueError(
            f"[{s.name}] {label} pose ({pose.x:.1f}, {pose.y:.1f}) is "
            f"out of bounds: cell ({i}, {j}) not in {(h, w)}"
        )
    if s.grid[i, j] != 0:
        raise ValueError(
            f"[{s.name}] {label} pose ({pose.x:.1f}, {pose.y:.1f}) is "
            f"on an obstacle at cell ({i}, {j})"
        )


# --- Scenarios ---------------------------------------------------------------

def _make_basic() -> Scenario:
    g = _basic_grid()
    s = Scenario(
        name="basic",
        description="60x60 corridor with three interior obstacles. "
                    "Tests standard heuristic efficiency.",
        grid=g,
        origin_x=-5.0,
        origin_y=-5.0,
        grid_resolution=0.5,
        start=Pose(x=2.0, y=2.0, yaw=0.0, direction=FORWARD),
        goal=Pose(x=22.0, y=20.0, yaw=np.pi / 2, direction=FORWARD),
        angle_resolution=np.pi / 8,
        steer_resolution=np.pi / 16,
        velocity=3.0,
        simulation_time=0.8,
        dt=0.1,
        max_iterations=3000,
        w_steer=8.0,
        w_turn=12.0,
        w_cusp=100.0,
    )
    _assert_free(s, s.start, "start")
    _assert_free(s, s.goal, "goal")
    return s


def _make_narrow() -> Scenario:
    """A narrow-passage scenario.

    The vehicle must steer through a 2-cell (1 m) gap in a vertical
    wall. Tests tight-maneuvering heuristics and the cusp cost
    (the planner may need to reverse to align with the gap).
    """
    g = _narrow_grid()
    s = Scenario(
        name="narrow",
        description="40x40 grid with a vertical wall and a narrow gap. "
                    "Tests precise steering through a tight passage.",
        grid=g,
        origin_x=-5.0,
        origin_y=-5.0,
        grid_resolution=0.5,
        start=Pose(x=0.0, y=10.0, yaw=0.0, direction=FORWARD),
        goal=Pose(x=12.0, y=10.0, yaw=0.0, direction=FORWARD),
        angle_resolution=np.pi / 12,
        steer_resolution=np.pi / 24,
        velocity=2.0,
        simulation_time=0.6,
        dt=0.1,
        max_iterations=5000,
        w_steer=8.0,
        w_turn=12.0,
        w_cusp=50.0,
    )
    _assert_free(s, s.start, "start")
    _assert_free(s, s.goal, "goal")
    return s


def _make_u_turn() -> Scenario:
    """The fixed U-Turn scenario.

    Grid is expanded to 50x50 with walls at the 10-cell border so the
    free corridor is rows 10..39, cols 10..39. The start and goal sit
    inside the corridor with opposite yaw for a 180-degree maneuver.
    The planner needs more iterations than the default for this
    scenario, so ``max_iterations`` is bumped to 20000.
    """
    g = _u_turn_grid()
    s = Scenario(
        name="u_turn",
        description="50x50 corridor. Tests a 180-degree U-turn.",
        grid=g,
        origin_x=-5.0,
        origin_y=-5.0,
        grid_resolution=0.5,
        start=Pose(x=3.0, y=13.0, yaw=0.0, direction=FORWARD),
        goal=Pose(x=14.0, y=5.0, yaw=np.pi, direction=FORWARD),
        angle_resolution=np.pi / 10,
        steer_resolution=np.pi / 20,
        velocity=2.0,
        simulation_time=0.7,
        dt=0.1,
        max_iterations=20000,
        w_steer=6.0,
        w_turn=10.0,
        w_cusp=40.0,
    )
    _assert_free(s, s.start, "start")
    _assert_free(s, s.goal, "goal")
    return s


# Registry -------------------------------------------------------------------

SCENARIOS: Dict[str, Scenario] = {
    s.name: s
    for s in (_make_basic(), _make_narrow(), _make_u_turn())
}


def list_scenarios() -> List[Scenario]:
    """Return all scenarios in display order."""
    return [SCENARIOS[k] for k in ("basic", "narrow", "u_turn")]


def get_scenario(name: str) -> Scenario:
    """Look up a scenario by name, raising ``KeyError`` with a helpful
    message if unknown."""
    try:
        return SCENARIOS[name]
    except KeyError as e:
        raise KeyError(
            f"Unknown scenario {name!r}. Available: {sorted(SCENARIOS)}"
        ) from e
