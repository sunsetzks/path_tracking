"""Flask web demo for the Hybrid A* path planner.

Exposes :class:`astar_project.hybrid_astar.HybridAStar` over HTTP/JSON
and ships a small vanilla-JS single-page UI for interactive planning.

Run via ``python -m flask_demo.run`` or ``flask --app flask_demo.app run``.
"""
