"""Convenience entry point: ``python -m flask_demo.run``.

Starts the Flask development server on 127.0.0.1:5000.
"""

from __future__ import annotations

import argparse
import logging


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1",
                        help="bind address (default: 127.0.0.1)")
    parser.add_argument("--port", type=int, default=5000,
                        help="bind port (default: 5000)")
    parser.add_argument("--debug", action="store_true",
                        help="enable Flask debug mode")
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s %(levelname)s %(name)s: %(message)s")
    from .app import create_app
    app = create_app()
    print(f"\n  Hybrid A* Flask demo running at "
          f"http://{args.host}:{args.port}/\n  Press Ctrl+C to stop.\n")
    app.run(host=args.host, port=args.port, debug=args.debug, use_reloader=False)


if __name__ == "__main__":
    main()
