"""Import the operator console from the repository this package lives in.

`web_ui` is a plain Python package at the repository root rather than a ROS
one, so a node launched from an installed workspace does not find it on
sys.path. Locating it here keeps one copy of the console and one copy of the
contract its adapters implement; a second copy would drift, and the thing that
would drift is what the page says about the safety latch.
"""

import importlib.util
from pathlib import Path
import sys


def repository_root() -> Path | None:
    """Find the checkout by looking for web_ui beside this package.

    `resolve()` first: colcon --symlink-install puts a symlink in the install
    tree, and the repository is only above the file it points at.
    """
    for parent in Path(__file__).resolve().parents:
        if (parent / "web_ui" / "app.py").is_file():
            return parent
    return None


if importlib.util.find_spec("web_ui") is None:  # pragma: no cover - path setup
    _root = repository_root()
    if _root is None:
        raise ImportError(
            "the operator console (web_ui) is not importable and was not found "
            "next to robot_console: set PYTHONPATH to the repository root"
        )
    sys.path.insert(0, str(_root))

import web_ui.app as _console  # noqa: E402 - only importable once path is set

BadRequest = _console.BadRequest
GoalStatus = _console.GoalStatus
RobotWebApp = _console.RobotWebApp
make_handler = _console.make_handler

# The page's static files ship beside the module that serves them, so this
# follows web_ui wherever it was found rather than guessing a second time.
WEB_ROOT = Path(_console.__file__).resolve().parent / "web"

__all__ = [
    "WEB_ROOT",
    "BadRequest",
    "GoalStatus",
    "RobotWebApp",
    "make_handler",
    "repository_root",
]
