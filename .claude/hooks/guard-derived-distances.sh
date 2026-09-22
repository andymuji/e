#!/usr/bin/env bash
# PreToolUse guard: refuse edits that change the derived safety distances.
#
# stop_distance and caution_distance are computed by robot_safety.distances
# from base_dynamics.yaml. Editing them directly makes the published numbers
# disagree with the model behind them; the robot_bringup tests fail on that
# drift. The rest of each file stays editable.
#
# Both gate parameter files are guarded. safety_navigation.yaml carries the
# same two derived values as safety.yaml and is the file autonomous navigation
# actually loads, so guarding only safety.yaml would leave the stricter
# configuration - the one that runs when Nav2 is driving - unprotected.
set -uo pipefail

payload=$(cat)
file=$(printf '%s' "$payload" | jq -r '.tool_input.file_path // empty')

case "$file" in
  */robot_bringup/config/safety.yaml) ;;
  */robot_bringup/config/safety_navigation.yaml) ;;
  *) exit 0 ;;
esac

touched=$(printf '%s' "$payload" | jq -r '
  [ .tool_input.content?,
    .tool_input.new_string?,
    .tool_input.old_string?,
    (.tool_input.edits? // [] | .[] | .new_string, .old_string)
  ] | map(select(. != null)) | join("\n")')

if printf '%s\n' "$touched" | grep -qE '^[[:space:]]*(stop_distance|caution_distance)[[:space:]]*:'; then
  jq -n '{
    hookSpecificOutput: {
      hookEventName: "PreToolUse",
      permissionDecision: "deny",
      permissionDecisionReason: "stop_distance and caution_distance are derived, not chosen. Change the inputs in robot/ros2_ws/src/robot_bringup/config/base_dynamics.yaml instead - robot_safety.distances recomputes them, and the robot_bringup tests fail on drift. See AGENTS.md, Safety Rules."
    }
  }'
fi
exit 0
