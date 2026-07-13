#!/usr/bin/env python3
"""Script wrapper for ROS2 local navigation debug outputs."""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from perception.local_navigation_node import main


if __name__ == "__main__":
    raise SystemExit(main())
