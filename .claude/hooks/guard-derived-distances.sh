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
#
# Two branches, because the two tool shapes give the guard different evidence:
#
#   Write/Edit  the new content is in the payload, so the guard reads it and
#               denies only when a guarded line is actually touched.
#   Bash        the payload holds a shell command, not its result. What a
#               `sed -i`, a heredoc or a redirect would leave in the file
#               cannot be known without running it. So this branch judges the
#               command instead: anything naming a guarded file that is not
#               plainly read-only is refused, and the caller is sent to
#               Write/Edit where the content CAN be inspected.
#
# That asymmetry is deliberate. The Bash branch is blunter than the Write/Edit
# one and errs towards refusing: a false refusal costs one retry through Write,
# while a false allow puts an unreviewed number into the one file every motion
# command is measured against.
set -uo pipefail

payload=$(cat)
file=$(printf '%s' "$payload" | jq -r '.tool_input.file_path // empty')
command=$(printf '%s' "$payload" | jq -r '.tool_input.command // empty')

# Matches the guarded files wherever they are referenced from.
guarded_re='(^|/)safety(_navigation)?\.yaml'

deny() {
  jq -n --arg reason "$1" '{
    hookSpecificOutput: {
      hookEventName: "PreToolUse",
      permissionDecision: "deny",
      permissionDecisionReason: $reason
    }
  }'
  exit 0
}

derived_reason="stop_distance and caution_distance are derived, not chosen. Change the inputs in robot/ros2_ws/src/robot_bringup/config/base_dynamics.yaml instead - robot_safety.distances recomputes them, and the robot_bringup tests fail on drift. See AGENTS.md, Safety Rules."

# ---------------------------------------------------------------- Bash branch
if [ -n "$command" ]; then
  # Not about a guarded file at all.
  printf '%s' "$command" | grep -qE "$guarded_re" || exit 0

  # Plainly read-only: a known read command in first position, and none of the
  # constructs that write. Anything else naming a guarded file is refused.
  if printf '%s' "$command" \
       | grep -qE '^[[:space:]]*(cat|bat|head|tail|less|more|grep|egrep|fgrep|rg|wc|md5sum|sha256sum|ls|stat|file|diff|realpath|readlink|git[[:space:]]+(diff|show|log|status|blame))\b' \
     && ! printf '%s' "$command" | grep -qE '(>|>>|\btee\b|\bdd\b|\bsed\b[^|;&]*-i|\bperl\b[^|;&]*-i|\bpython3?\b[^|;&]*-i|\bcp\b|\bmv\b|\bln\b|\binstall\b|\btruncate\b|\bpatch\b|\bshred\b|\brm\b)'; then
    exit 0
  fi

  deny "$derived_reason

This came through Bash, where the guard can see the command but not the file it would leave behind, so it cannot tell whether a derived line was touched. Make this change with the Write or Edit tool instead - there the new content is inspected, and an edit that leaves stop_distance and caution_distance alone passes straight through. See AGENTS.md, Safety Rules."
fi

# ----------------------------------------------------------- Write/Edit branch
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
  deny "$derived_reason"
fi
exit 0
