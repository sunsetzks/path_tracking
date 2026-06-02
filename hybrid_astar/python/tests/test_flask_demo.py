"""Tests for the Flask web demo.

Uses Flask's test client (no live HTTP server needed).
Run with:
    PYTHONPATH=. pytest tests/test_flask_demo.py -v
"""

from __future__ import annotations

import base64
import os
import sys

import pytest

# Ensure the test can import both the hybrid_astar package and the
# flask_demo package whether the tests are run from the repo root or
# from the tests/ directory.
_HERE = os.path.dirname(os.path.abspath(__file__))
_PYTHON_DIR = os.path.dirname(_HERE)
if _PYTHON_DIR not in sys.path:
    sys.path.insert(0, _PYTHON_DIR)

from flask_demo.app import create_app  # noqa: E402


PNG_MAGIC = b"\x89PNG\r\n\x1a\n"


@pytest.fixture(scope="module")
def client():
    app = create_app()
    app.config.update(TESTING=True)
    with app.test_client() as c:
        yield c


# ---------------------------------------------------------------------------
# /api/health
# ---------------------------------------------------------------------------

def test_health(client):
    resp = client.get("/api/health")
    assert resp.status_code == 200
    assert resp.get_json() == {"ok": True}


# ---------------------------------------------------------------------------
# /api/scenarios
# ---------------------------------------------------------------------------

def test_scenarios_listed(client):
    resp = client.get("/api/scenarios")
    assert resp.status_code == 200
    data = resp.get_json()
    assert isinstance(data, list)
    names = {s["name"] for s in data}
    assert {"basic", "narrow", "u_turn"}.issubset(names)
    for s in data:
        assert "map_size" in s and len(s["map_size"]) == 2
        assert "origin" in s and len(s["origin"]) == 2
        assert "start" in s and "goal" in s
        assert "params" in s and "weights" in s
        assert "obstacle_cells" in s
        assert "occupancy_png" in s
        # Occupancy PNG is valid base64-encoded PNG
        png = base64.b64decode(s["occupancy_png"])
        assert png[:8] == PNG_MAGIC


# ---------------------------------------------------------------------------
# /api/plan — success path
# ---------------------------------------------------------------------------

def test_plan_basic_succeeds(client):
    resp = client.post(
        "/api/plan",
        json={"scenario": "basic"},
    )
    assert resp.status_code == 200, resp.get_json()
    data = resp.get_json()
    assert data["success"] is True
    assert data["scenario"] == "basic"
    assert len(data["waypoints"]) > 5
    # Statistics
    stats = data["statistics"]
    assert stats["path_nodes"] > 0
    assert stats["detailed_waypoints"] > 0
    assert stats["nodes_explored"] > 0
    # Images
    for key in ("path_png", "search_png"):
        png = base64.b64decode(data["images"][key])
        assert png[:8] == PNG_MAGIC


def test_plan_all_scenarios_succeed(client):
    """Each built-in scenario should plan successfully with defaults."""
    for name in ("basic", "narrow", "u_turn"):
        resp = client.post("/api/plan", json={"scenario": name})
        assert resp.status_code == 200, (name, resp.get_json())
        data = resp.get_json()
        assert data["success"] is True, (name, data)
        assert len(data["waypoints"]) > 0, name


# ---------------------------------------------------------------------------
# /api/plan — failure paths
# ---------------------------------------------------------------------------

def test_plan_unknown_scenario_returns_400(client):
    resp = client.post("/api/plan", json={"scenario": "nonexistent"})
    assert resp.status_code == 400
    assert "error" in resp.get_json()


def test_plan_invalid_grid_returns_400(client):
    """If the override grid puts the start in collision, the API
    should return 400 with a descriptive error."""
    # Build a grid that is entirely obstacles (50x50 to match "narrow").
    full_block = [[1] * 40 for _ in range(40)]
    resp = client.post(
        "/api/plan",
        json={"scenario": "narrow", "grid_override": full_block},
    )
    assert resp.status_code == 400
    body = resp.get_json()
    assert "error" in body
    assert "collision" in body["error"].lower() or "bounds" in body["error"].lower()


def test_plan_wrong_grid_shape_returns_400(client):
    bad_grid = [[0] * 5 for _ in range(5)]  # doesn't match any scenario
    resp = client.post(
        "/api/plan",
        json={"scenario": "basic", "grid_override": bad_grid},
    )
    assert resp.status_code == 400


# ---------------------------------------------------------------------------
# /api/render
# ---------------------------------------------------------------------------

def test_render_returns_valid_png(client):
    resp = client.post(
        "/api/render",
        json={
            "scenario": "basic",
            "waypoints": [
                {"x": 2.0, "y": 2.0, "yaw": 0.0, "steer": 0.0, "direction": 1},
                {"x": 4.0, "y": 3.0, "yaw": 0.1, "steer": 0.05, "direction": 1},
                {"x": 6.0, "y": 5.0, "yaw": 0.2, "steer": 0.1, "direction": 1},
            ],
            "start": {"x": 2.0, "y": 2.0, "yaw": 0.0},
            "goal":  {"x": 6.0, "y": 5.0, "yaw": 0.2},
        },
    )
    assert resp.status_code == 200
    body = resp.get_json()
    assert "image" in body
    png = base64.b64decode(body["image"])
    assert png[:8] == PNG_MAGIC


def test_render_empty_waypoints_returns_400(client):
    resp = client.post(
        "/api/render",
        json={"scenario": "basic", "waypoints": []},
    )
    assert resp.status_code == 400


# ---------------------------------------------------------------------------
# /  (HTML index)
# ---------------------------------------------------------------------------

def test_index_returns_html(client):
    resp = client.get("/")
    assert resp.status_code == 200
    body = resp.get_data(as_text=True)
    assert "<html" in body.lower()
    assert "Hybrid A* Web Demo" in body
