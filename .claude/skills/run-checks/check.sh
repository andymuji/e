#!/usr/bin/env bash
# Run the repository's Python checks. See CLAUDE.md "Development Checks".
# Usage: check.sh [tests|lint|compile|all]   (default: all)
set -uo pipefail
cd "$(git rev-parse --show-toplevel)" || exit 1

SRC=robot/ros2_ws/src
export PYTHONPATH=$SRC/robot_core:$SRC/robot_locations:$SRC/robot_navigation:$SRC/robot_safety:$SRC/robot_voice

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

exit $rc
