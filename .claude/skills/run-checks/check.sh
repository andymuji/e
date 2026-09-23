#!/usr/bin/env bash
# Run the repository's Python checks. See AGENTS.md "Development Checks".
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

# Every package that holds an importable Python module, discovered rather than
# listed: a package added to the workspace is on the path and under test the
# moment it exists, instead of passing here because nobody remembered to add
# a line. The glob is sorted, so the run order stays stable.
PACKAGE_PATHS=()
for pkg in "$SRC"/*/; do
  pkg=${pkg%/}
  # A package directory holding a module directory of the same name is an
  # importable one; robot_bringup and robot_description hold only launch
  # files and config, so they are not on the path and never were.
  [[ -d $pkg/$(basename "$pkg") ]] && PACKAGE_PATHS+=("$pkg")
done
PACKAGES=$(IFS=:; echo "${PACKAGE_PATHS[*]}")

# Append, never replace: overwriting PYTHONPATH drops ROS's own site-packages
# and makes the safety-node tests skip even inside the container.
export PYTHONPATH=$PACKAGES${PYTHONPATH:+:$PYTHONPATH}

SUITES=()
for d in "$SRC"/*/tests; do
  [[ -d $d ]] && SUITES+=("$d")
done
SUITES+=(web_ui/tests)

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

  # The guard hook in front of the derived safety distances. Not a Python
  # suite and not in a ROS package, so the glob above cannot find it, but it
  # is repo safety machinery and it spent a whole round of work with a hole
  # in it. Takes no flags, so "${@:2}" is deliberately not passed on.
  # Needs jq, which the devcontainer has; reported as skipped rather than
  # failed where it is absent, so a bare Codespace still runs the rest.
  guard_test=.claude/hooks/test-guard-derived-distances.sh
  if [[ -x $guard_test ]]; then
    if command -v jq >/dev/null 2>&1; then
      if out=$("$guard_test" 2>&1); then
        printf '%-48s %s\n' ".claude/hooks" "$(printf '%s\n' "$out" | tail -1)"
      else
        printf '%-48s FAILED\n' ".claude/hooks"
        printf '%s\n' "$out"
        rc=1
      fi
    else
      printf '%-48s %s\n' ".claude/hooks" "skipped (no jq)"
    fi
  fi
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
