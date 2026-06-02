/* Hybrid A* Web Demo — vanilla JS frontend.
 *
 * Talks to the Flask backend at /api/* and renders the occupancy grid,
 * the planned path, and the server-rendered search tree on a single
 * <canvas>.
 *
 * Coordinate convention
 * ---------------------
 * The planner uses the same world-to-grid mapping as the backend
 * (see scenarios.py). On the canvas we map world (x, y) to pixel
 * (px, py) by:
 *     px = (x - originX) * (canvasW / (cols * gridRes))
 *     py = canvasH - (y - originY) * (canvasH / (rows * gridRes))
 * (the y-flip matches matplotlib's "origin=upper" for the occupancy
 * image).
 */

(function () {
  "use strict";

  // ---------- DOM ----------
  const $ = (id) => document.getElementById(id);
  const canvas = $("map");
  const ctx = canvas.getContext("2d");
  const scenarioSelect = $("scenario-select");
  const scenarioDesc = $("scenario-desc");
  const statusBar = $("status");
  const stats = {
    status: $("stat-status"),
    pathNodes: $("stat-path-nodes"),
    waypoints: $("stat-waypoints"),
    distance: $("stat-distance"),
    time: $("stat-time"),
    explored: $("stat-explored"),
  };
  const searchPng = $("search-png");
  const paramFields = {
    "grid-resolution": $("param-grid-resolution"),
    "max-iterations": $("param-max-iterations"),
    velocity: $("param-velocity"),
    "simulation-time": $("param-simulation-time"),
    "angle-resolution": $("param-angle-resolution"),
    "steer-resolution": $("param-steer-resolution"),
  };
  const weightFields = {
    w_steer: $("w-steer"),
    w_turn: $("w-turn"),
    w_cusp: $("w-cusp"),
  };

  // ---------- State ----------
  let scenarios = [];          // list from /api/scenarios
  let currentScenario = null;  // selected scenario object
  let grid = [];               // 2D array 0/1 (mutable for paint tools)
  let waypoints = [];          // last planned path (array of {x, y, yaw, steer, direction})
  let start = null;            // {x, y, yaw, direction}
  let goal = null;             // {x, y, yaw, direction}
  let mouseDown = false;
  let lastPaintedCell = null;

  // ---------- Utility ----------
  function setStatus(msg, kind) {
    statusBar.textContent = msg;
    statusBar.className = "statusbar" + (kind ? " " + kind : "");
  }

  function worldToCell(x, y) {
    const res = currentScenario.grid_resolution;
    const j = Math.round((x - currentScenario.origin[0]) / res);
    const i = Math.round(
      (currentScenario.map_size[0] - 1) -
        (y - currentScenario.origin[1]) / res
    );
    return { i, j };
  }

  function cellToWorld(i, j) {
    const res = currentScenario.grid_resolution;
    const x = currentScenario.origin[0] + j * res;
    const y = currentScenario.origin[1] +
      (currentScenario.map_size[0] - 1 - i) * res;
    return { x, y };
  }

  function pixelToWorld(px, py) {
    const rect = canvas.getBoundingClientRect();
    const scaleX = canvas.width / rect.width;
    const scaleY = canvas.height / rect.height;
    const xPx = (px - rect.left) * scaleX;
    const yPx = (py - rect.top) * scaleY;
    const H = currentScenario.map_size[0];
    const W = currentScenario.map_size[1];
    const ox = currentScenario.origin[0];
    const oy = currentScenario.origin[1];
    const res = currentScenario.grid_resolution;
    const x = ox + (xPx / canvas.width) * (W * res);
    const y = oy + (1 - yPx / canvas.height) * (H * res);
    return { x, y };
  }

  function pixelToCell(px, py) {
    const { x, y } = pixelToWorld(px, py);
    return worldToCell(x, y);
  }

  function getActiveTool() {
    const r = document.querySelector('input[name="tool"]:checked');
    return r ? r.value : "select";
  }

  // ---------- Rendering ----------
  function drawGrid() {
    if (!currentScenario) return;
    const H = currentScenario.map_size[0];
    const W = currentScenario.map_size[1];
    const cw = canvas.width / W;
    const ch = canvas.height / H;
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    // Free background
    ctx.fillStyle = "#fafbfc";
    ctx.fillRect(0, 0, canvas.width, canvas.height);
    // Obstacles
    ctx.fillStyle = "#333";
    for (let i = 0; i < H; i++) {
      for (let j = 0; j < W; j++) {
        if (grid[i] && grid[i][j]) {
          ctx.fillRect(j * cw, i * ch, cw + 0.5, ch + 0.5);
        }
      }
    }
    // Grid lines
    ctx.strokeStyle = "rgba(0,0,0,0.06)";
    ctx.lineWidth = 1;
    for (let j = 0; j <= W; j++) {
      ctx.beginPath();
      ctx.moveTo(j * cw + 0.5, 0);
      ctx.lineTo(j * cw + 0.5, canvas.height);
      ctx.stroke();
    }
    for (let i = 0; i <= H; i++) {
      ctx.beginPath();
      ctx.moveTo(0, i * ch + 0.5);
      ctx.lineTo(canvas.width, i * ch + 0.5);
      ctx.stroke();
    }
  }

  function worldXYToPixel(x, y) {
    const H = currentScenario.map_size[0];
    const W = currentScenario.map_size[1];
    const ox = currentScenario.origin[0];
    const oy = currentScenario.origin[1];
    const res = currentScenario.grid_resolution;
    const px = ((x - ox) / (W * res)) * canvas.width;
    const py = (1 - (y - oy) / (H * res)) * canvas.height;
    return { px, py };
  }

  function drawPath() {
    if (!waypoints || waypoints.length < 2) return;
    const pts = waypoints.map((w) => worldXYToPixel(w.x, w.y));
    // Color segments by |steer|
    const steers = waypoints.map((w) => Math.abs(w.steer));
    const smax = Math.max(...steers, 1e-9);
    ctx.lineWidth = 2.5;
    for (let k = 0; k < pts.length - 1; k++) {
      const t = steers[k] / smax;
      // blue (cool) -> red (warm)
      const r = Math.round(43 + t * (220 - 43));
      const g = Math.round(108 - t * 80);
      const b = Math.round(180 - t * 130);
      ctx.strokeStyle = "rgb(" + r + "," + g + "," + b + ")";
      ctx.beginPath();
      ctx.moveTo(pts[k].px, pts[k].py);
      ctx.lineTo(pts[k + 1].px, pts[k + 1].py);
      ctx.stroke();
    }
  }

  function drawPose(pose, color, marker) {
    if (!pose) return;
    const { px, py } = worldXYToPixel(pose.x, pose.y);
    ctx.save();
    ctx.translate(px, py);
    // Yaw: 0 rad = +x in world, canvas x is also +x, canvas y is flipped.
    // World rotation: angle from +x axis CCW. Canvas: we want the triangle
    // to point in the heading direction in world space. The y-flip
    // means a positive yaw (CCW in world) corresponds to a CCW rotation
    // on the canvas as well (since canvas y points down, "CCW" from the
    // viewer's perspective is actually CW in screen coordinates — but
    // we're rotating the marker, so negate).
    ctx.rotate(-pose.yaw);
    ctx.fillStyle = color;
    ctx.strokeStyle = "#000";
    ctx.lineWidth = 1.2;
    const size = 10;
    ctx.beginPath();
    if (marker === "triangle") {
      ctx.moveTo(size, 0);
      ctx.lineTo(-size * 0.6, -size * 0.7);
      ctx.lineTo(-size * 0.6, size * 0.7);
      ctx.closePath();
    } else if (marker === "star") {
      for (let k = 0; k < 5; k++) {
        const a = (k * 2 * Math.PI) / 5 - Math.PI / 2;
        const r1 = size * 1.1;
        const r2 = size * 0.45;
        ctx.lineTo(Math.cos(a) * r1, Math.sin(a) * r1);
        const a2 = a + Math.PI / 5;
        ctx.lineTo(Math.cos(a2) * r2, Math.sin(a2) * r2);
      }
      ctx.closePath();
    }
    ctx.fill();
    ctx.stroke();
    ctx.restore();
  }

  function drawStartGoal() {
    drawPose(start, "#28a745", "triangle");
    drawPose(goal, "#dc3545", "star");
  }

  function redraw() {
    drawGrid();
    drawPath();
    drawStartGoal();
  }

  // ---------- Scenarios ----------
  async function loadScenarios() {
    const resp = await fetch("/api/scenarios");
    scenarios = await resp.json();
    scenarioSelect.innerHTML = "";
    for (const s of scenarios) {
      const opt = document.createElement("option");
      opt.value = s.name;
      opt.textContent = s.name;
      scenarioSelect.appendChild(opt);
    }
    selectScenario(scenarios[0] && scenarios[0].name);
  }

  function selectScenario(name) {
    const s = scenarios.find((x) => x.name === name);
    if (!s) return;
    currentScenario = s;
    grid = s.obstacle_cells.length
      ? gridFromCells(s.obstacle_cells, s.map_size[0], s.map_size[1])
      : makeEmptyGrid(s.map_size[0], s.map_size[1]);
    start = { ...s.start };
    goal = { ...s.goal };
    waypoints = [];
    scenarioDesc.textContent = s.description;
    populateParamFields();
    redraw();
    searchPng.removeAttribute("src");
    clearStats();
    setStatus("Loaded scenario '" + s.name + "'.");
  }

  function makeEmptyGrid(H, W) {
    const g = new Array(H);
    for (let i = 0; i < H; i++) {
      g[i] = new Array(W).fill(0);
    }
    return g;
  }

  function gridFromCells(cells, H, W) {
    const g = makeEmptyGrid(H, W);
    for (const [i, j] of cells) g[i][j] = 1;
    return g;
  }

  function populateParamFields() {
    const p = currentScenario.params;
    paramFields["grid-resolution"].value = p.grid_resolution;
    paramFields["max-iterations"].value = p.max_iterations;
    paramFields["velocity"].value = p.velocity;
    paramFields["simulation-time"].value = p.simulation_time;
    paramFields["angle-resolution"].value = p.angle_resolution;
    paramFields["steer-resolution"].value = p.steer_resolution;
    const w = currentScenario.weights;
    weightFields.w_steer.value = w.w_steer;
    weightFields.w_turn.value = w.w_turn;
    weightFields.w_cusp.value = w.w_cusp;
  }

  function readParamsAndWeights() {
    const params = {};
    for (const k in paramFields) params[k] = parseFloat(paramFields[k].value);
    const weights = {};
    for (const k in weightFields) weights[k] = parseFloat(weightFields[k].value);
    return { params, weights };
  }

  function gridToList() {
    return grid.map((row) => Array.from(row));
  }

  function clearStats() {
    stats.status.textContent = "—";
    stats.pathNodes.textContent = "—";
    stats.waypoints.textContent = "—";
    stats.distance.textContent = "—";
    stats.time.textContent = "—";
    stats.explored.textContent = "—";
  }

  // ---------- Canvas interaction ----------
  function onCanvasMouseDown(ev) {
    mouseDown = true;
    handleCanvasEvent(ev);
  }
  function onCanvasMouseMove(ev) {
    if (!mouseDown) return;
    handleCanvasEvent(ev);
  }
  function onCanvasMouseUp() { mouseDown = false; lastPaintedCell = null; }
  function onCanvasContextMenu(ev) {
    ev.preventDefault();
    if (!currentScenario) return;
    const { x, y } = pixelToWorld(ev.clientX, ev.clientY);
    const tool = getActiveTool();
    if (tool === "start" && start) {
      const dx = x - start.x, dy = y - start.y;
      start.yaw = Math.atan2(dy, dx);
    } else if (tool === "goal" && goal) {
      const dx = x - goal.x, dy = y - goal.y;
      goal.yaw = Math.atan2(dy, dx);
    }
    redraw();
  }

  function handleCanvasEvent(ev) {
    if (!currentScenario) return;
    const { i, j } = pixelToCell(ev.clientX, ev.clientY);
    const H = currentScenario.map_size[0];
    const W = currentScenario.map_size[1];
    if (i < 0 || i >= H || j < 0 || j >= W) return;
    const tool = getActiveTool();
    if (tool === "paint-obstacle") {
      if (lastPaintedCell && lastPaintedCell.i === i && lastPaintedCell.j === j) return;
      grid[i][j] = 1;
      lastPaintedCell = { i, j };
      redraw();
    } else if (tool === "erase-obstacle") {
      if (lastPaintedCell && lastPaintedCell.i === i && lastPaintedCell.j === j) return;
      grid[i][j] = 0;
      lastPaintedCell = { i, j };
      redraw();
    } else if (tool === "start") {
      const { x, y } = cellToWorld(i, j);
      if (start) {
        const dx = x - start.x, dy = y - start.y;
        if (Math.hypot(dx, dy) > 1e-6) start.yaw = Math.atan2(dy, dx);
      }
      start = { x, y, yaw: start ? start.yaw : 0, direction: 1 };
      redraw();
    } else if (tool === "goal") {
      const { x, y } = cellToWorld(i, j);
      if (goal) {
        const dx = x - goal.x, dy = y - goal.y;
        if (Math.hypot(dx, dy) > 1e-6) goal.yaw = Math.atan2(dy, dx);
      }
      goal = { x, y, yaw: goal ? goal.yaw : 0, direction: 1 };
      redraw();
    } else if (tool === "select") {
      const v = grid[i][j];
      const { x, y } = cellToWorld(i, j);
      setStatus("Cell (" + i + ", " + j + ") = " + (v ? "obstacle" : "free") +
                "  world (" + x.toFixed(2) + ", " + y.toFixed(2) + ")");
    }
  }

  // ---------- Plan / Render ----------
  async function planPath() {
    if (!currentScenario) return;
    setStatus("Planning...");
    stats.status.textContent = "running...";
    const { params, weights } = readParamsAndWeights();
    const body = {
      scenario: currentScenario.name,
      grid_override: gridToList(),
      origin_x: currentScenario.origin[0],
      origin_y: currentScenario.origin[1],
      start: start,
      goal: goal,
      params: params,
      weights: weights,
    };
    try {
      const resp = await fetch("/api/plan", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(body),
      });
      const data = await resp.json();
      if (!resp.ok || data.error) {
        setStatus("Plan failed: " + (data.error || resp.status), "error");
        stats.status.textContent = "failed";
        return;
      }
      if (!data.success) {
        setStatus("No path found.", "error");
        stats.status.textContent = "no path";
        stats.explored.textContent = data.statistics.nodes_explored;
        return;
      }
      waypoints = data.waypoints;
      stats.status.textContent = "ok";
      stats.pathNodes.textContent = data.statistics.path_nodes;
      stats.waypoints.textContent = data.statistics.detailed_waypoints;
      stats.distance.textContent =
        (data.statistics.total_distance || 0).toFixed(2);
      stats.time.textContent =
        (data.statistics.search_time_seconds || 0).toFixed(3);
      stats.explored.textContent = data.statistics.nodes_explored;
      if (data.images && data.images.search_png) {
        searchPng.src =
          "data:image/png;base64," + data.images.search_png;
      }
      redraw();
      setStatus("Path found: " + data.waypoints.length + " waypoints.",
                "success");
    } catch (err) {
      setStatus("Plan error: " + err.message, "error");
    }
  }

  async function renderPath() {
    if (!currentScenario) return;
    if (!waypoints || !waypoints.length) {
      setStatus("Nothing to render — plan a path first.", "error");
      return;
    }
    setStatus("Rendering path PNG...");
    try {
      const resp = await fetch("/api/render", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({
          scenario: currentScenario.name,
          waypoints: waypoints,
          start: start,
          goal: goal,
        }),
      });
      const data = await resp.json();
      if (!resp.ok || data.error) {
        setStatus("Render failed: " + (data.error || resp.status), "error");
        return;
      }
      // Open the image in a new tab
      const w = window.open();
      if (w) {
        w.document.write(
          "<title>Path PNG</title><body style='margin:0;background:#222;'>" +
          "<img src='data:image/png;base64," + data.image +
          "' style='max-width:100%;display:block;margin:auto;'/></body>"
        );
      }
      setStatus("Rendered.", "success");
    } catch (err) {
      setStatus("Render error: " + err.message, "error");
    }
  }

  // ---------- Wire up ----------
  function init() {
    scenarioSelect.addEventListener("change", (e) => {
      selectScenario(e.target.value);
    });
    $("btn-reset").addEventListener("click", () => {
      if (currentScenario) selectScenario(currentScenario.name);
    });
    $("btn-clear-path").addEventListener("click", () => {
      waypoints = [];
      searchPng.removeAttribute("src");
      clearStats();
      redraw();
      setStatus("Path cleared.");
    });
    $("btn-plan").addEventListener("click", planPath);
    $("btn-render").addEventListener("click", renderPath);
    canvas.addEventListener("mousedown", onCanvasMouseDown);
    canvas.addEventListener("mousemove", onCanvasMouseMove);
    window.addEventListener("mouseup", onCanvasMouseUp);
    canvas.addEventListener("contextmenu", onCanvasContextMenu);
    loadScenarios().catch((e) => setStatus("Load error: " + e.message, "error"));
  }

  if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", init);
  } else {
    init();
  }
})();
