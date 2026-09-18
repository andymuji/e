#!/usr/bin/env bash
# PreToolUse guard: refuse edits that change the derived safety distances.
#
# stop_distance and caution_distance in safety.yaml are computed by
# robot_safety.distances from base_dynamics.yaml. Editing them directly makes
# the published numbers disagree with the model behind them; the robot_bringup
# tests fail on that drift. The rest of safety.yaml stays editable.
set -uo pipefail

payload=$(cat)
file=$(printf '%s' "$payload" | jq -r '.tool_input.file_path // empty')

case "$file" in
  */robot_bringup/config/safety.yaml) ;;
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
      permissionDecisionReason: "stop_distance and caution_distance are derived, not chosen. Change the inputs in robot/ros2_ws/src/robot_bringup/config/base_dynamics.yaml instead - robot_safety.distances recomputes them, and the robot_bringup tests fail on drift. See CLAUDE.md, Safety Rules."
    }
  }'
fi
exit 0
