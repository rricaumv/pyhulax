#!/usr/bin/env python3
"""Launch the pyhulax control station.

    python examples/control_station/__main__.py

Needs the GUI extra:  pip install "pyhulax[gui]"   (PySide6 + numpy)
This shell runs entirely on the simulated StubRunner - no drone required.
"""

import os
import sys

# Put this package directory on sys.path so the sibling modules (registry,
# runner, app) import cleanly whether launched as a file or a module.
_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)


def main() -> None:
    try:
        import app  # noqa: E402  (path set above)
    except ImportError as exc:
        raise SystemExit(
            f"Cannot start the control station: {exc}\n"
            f"  install the GUI extra:  pip install 'pyhulax[gui]'  (PySide6)"
        )
    app.main()


if __name__ == "__main__":
    main()
