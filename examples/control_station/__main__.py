#!/usr/bin/env python3
"""Launch the pyhulax control station.

    python examples/control_station/__main__.py            # single drone
    python examples/control_station/__main__.py --dual     # two drones

From a checkout install the GUI deps:  pip install -e ".[gui]"  (PySide6 + numpy).
The simulated demos need no drone; the live demo also needs the video + YOLO deps.
"""

import os
import sys

# Put this package directory on sys.path so the sibling modules (registry,
# runner, app, app_dual) import cleanly whether launched as a file or a module.
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)


def main() -> None:
    dual = "--dual" in sys.argv
    module = "app_dual" if dual else "app"
    try:
        mod = __import__(module)  # path set above
    except ImportError as exc:
        raise SystemExit(
            f"Cannot start the control station: {exc}\n"
            f"  install the GUI deps from a checkout:  pip install -e '.[gui]'  (PySide6)"
        )
    mod.main()


if __name__ == "__main__":
    main()
