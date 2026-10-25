"""Puts the package source on the path, so `pytest` works here without an install."""

import pathlib
import sys

_SRC = str(pathlib.Path(__file__).parent)
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)
