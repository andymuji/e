#!/usr/bin/env bash
# Run the repository's Python checks. See CLAUDE.md "Development Checks".
# Usage: check.sh [tests|lint|compile|all]   (default: all)
set -uo pipefail
cd "$(git rev-parse --show-toplevel)" || exit 1

SRC=robot/ros2_ws/src

# Source ROS 2 when it is installed but not actually importable, so the
# rclpy-dependent suites run instead of skipping. Test the import, not
# ROS_DISTRO: the devcontainer exports ROS_DISTRO=jazzy without ROS being on
# the Python path, so that variable says nothing about whether rclpy loads.
# A Codespace without the devcontainer has no /opt/ros and skips as before.
if ! python3 -c "import rclpy" >/dev/null 2>&1 && [[ -f /opt/ros/jazzy/setup.bash ]]; then
  set +u
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  set -u
fi

# Append, never replace: overwriting PYTHONPATH drops ROS's own site-packages
# and makes the safety-node tests skip even inside the container.
export PYTHONPATH=$SRC/robot_core:$SRC/robot_locations:$SRC/robot_navigation:$SRC/robot_safety:$SRC/robot_voice${PYTHONPATH:+:$PYTHONPATH}

SUITES=(
  $SRC/robot_bringup/tests
  $SRC/robot_core/tests
  $SRC/robot_description/tests
  $SRC/robot_locations/tests
  $SRC/robot_navigation/tests
  $SRC/robot_safety/tests
  $SRC/robot_voice/tests
  web_ui/tests
)

rc=0
what=${1:-all}

if [[ $what == tests || $what == all ]]; then
  for d in "${SUITES[@]}"; do
    out=$(python3 -m unittest discover -s "$d" "${@:2}" 2>&1)
    last=$(printf '%s\n' "$out" | tail -1)
    if [[ $last == OK* ]]; then
      printf '%-48s %s\n' "$d" "$last"
    else
      printf '%-48s FAILED\n' "$d"
      printf '%s\n' "$out"
      rc=1
    fi
  done
fi

if [[ $what == compile || $what == all ]]; then
  python3 -m compileall -q $SRC web_ui || rc=1
fi

if [[ $what == lint || $what == all ]]; then
  ruff check $SRC web_ui || rc=1
fi

# Say plainly whether everything selected passed. Without this the last line of
# a full run is ruff's own "All checks passed!", which reports on lint alone and
# prints even when a suite above it failed.
if [[ $rc -eq 0 ]]; then
  echo "check.sh: PASSED"
else
  echo "check.sh: FAILED"
fi

exit $rc
