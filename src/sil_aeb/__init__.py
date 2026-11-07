"""SIL AEB package entry point."""

from __future__ import annotations

import sys
from pathlib import Path


def main() -> None:
    """Delegate the package entry point to main_launcher.main()."""
    root_dir = Path(__file__).resolve().parents[2]
    if str(root_dir) not in sys.path:
        sys.path.insert(0, str(root_dir))

    from main_launcher import main as launcher_main

    launcher_main()
