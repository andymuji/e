#!/usr/bin/env bash
# Tests for guard-derived-distances.sh.
#
# The guard is the fast failure in front of the derived safety distances. It
# went a whole round of work matching Write|Edit only, so a `sed -i` on
# safety.yaml passed straight through it and nobody noticed until an agent
# reported it. A guard with no test is how that happens, so: a test.
#
# Each case feeds the hook one synthetic PreToolUse payload on stdin and
# checks only whether a denial came back, which is the hook's entire contract.
set -uo pipefail
cd "$(dirname "$0")" || exit 1

GUARD=./guard-derived-distances.sh
CONFIG=robot/ros2_ws/src/robot_bringup/config
pass=0
fail=0

# expect <allow|deny> <description> <json payload>
expect() {
  local want=$1 desc=$2 payload=$3 got out
  out=$(printf '%s' "$payload" | "$GUARD" 2>&1)
  if printf '%s' "$out" | grep -q '"permissionDecision": *"deny"'; then
    got=deny
  else
    got=allow
  fi
  if [[ $got == "$want" ]]; then
    pass=$((pass + 1))
  else
    fail=$((fail + 1))
    printf 'FAIL  expected %-5s got %-5s  %s\n' "$want" "$got" "$desc"
  fi
}

bash_payload() { jq -n --arg c "$1" '{tool_name:"Bash",tool_input:{command:$c}}'; }
write_payload() {
  jq -n --arg f "$1" --arg c "$2" \
    '{tool_name:"Write",tool_input:{file_path:$f,content:$c}}'
}
edit_payload() {
  jq -n --arg f "$1" --arg o "$2" --arg n "$3" \
    '{tool_name:"Edit",tool_input:{file_path:$f,old_string:$o,new_string:$n}}'
}

# --- Bash branch: the gap this test exists for -------------------------------
expect deny "sed -i on safety.yaml" \
  "$(bash_payload "sed -i 's/0.45/0.30/' $CONFIG/safety.yaml")"
expect deny "sed -i on safety_navigation.yaml" \
  "$(bash_payload "sed -i 's/0.45/0.30/' $CONFIG/safety_navigation.yaml")"
expect deny "append via redirect" \
  "$(bash_payload "echo 'stop_distance: 0.1' >> $CONFIG/safety.yaml")"
expect deny "overwrite via heredoc" \
  "$(bash_payload "cat > $CONFIG/safety.yaml <<'EOF'
stop_distance: 0.1
EOF")"
expect deny "python rewrite" \
  "$(bash_payload "python3 -c \"open('$CONFIG/safety.yaml','w').write('x')\"")"
expect deny "tee into the file" \
  "$(bash_payload "echo x | tee $CONFIG/safety.yaml")"
expect deny "copy over the file" \
  "$(bash_payload "cp /tmp/mine.yaml $CONFIG/safety.yaml")"
expect deny "move over the file" \
  "$(bash_payload "mv /tmp/mine.yaml $CONFIG/safety_navigation.yaml")"
expect deny "delete the file" \
  "$(bash_payload "rm $CONFIG/safety.yaml")"

# Reading is not editing. These must stay out of the way.
expect allow "cat the file" "$(bash_payload "cat $CONFIG/safety.yaml")"
expect allow "grep the file" \
  "$(bash_payload "grep stop_distance $CONFIG/safety_navigation.yaml")"
expect allow "cat piped to grep" \
  "$(bash_payload "cat $CONFIG/safety.yaml | grep -n distance")"
expect allow "git diff the file" \
  "$(bash_payload "git diff $CONFIG/safety.yaml")"

# Commands that touch neither guarded file are none of the guard's business.
expect allow "unrelated build" "$(bash_payload "colcon build --symlink-install")"
expect allow "edits nav2.yaml" \
  "$(bash_payload "sed -i 's/a/b/' $CONFIG/nav2.yaml")"
expect allow "edits base_dynamics.yaml, the sanctioned input" \
  "$(bash_payload "sed -i 's/a/b/' $CONFIG/base_dynamics.yaml")"

# --- Write/Edit branch: unchanged behaviour ----------------------------------
expect deny "Write touching stop_distance" \
  "$(write_payload "/x/$CONFIG/safety.yaml" "safety_controller:
    stop_distance: 0.30")"
expect deny "Write touching caution_distance" \
  "$(write_payload "/x/$CONFIG/safety_navigation.yaml" "    caution_distance: 0.5")"
expect deny "Edit replacing stop_distance" \
  "$(edit_payload "/x/$CONFIG/safety.yaml" "    stop_distance: 0.45" "    stop_distance: 0.30")"
expect allow "Write leaving the derived lines alone" \
  "$(write_payload "/x/$CONFIG/safety.yaml" "    sensor_timeout: 0.5")"
expect allow "Edit on an unguarded file" \
  "$(edit_payload "/x/$CONFIG/nav2.yaml" "a" "b")"

# A derived name in prose, not as a key, is not an edit to it.
expect allow "Write mentioning the name in a comment" \
  "$(write_payload "/x/$CONFIG/safety.yaml" "# stop_distance is derived from base_dynamics.yaml")"

printf 'guard-derived-distances: %d passed, %d failed\n' "$pass" "$fail"
[[ $fail -eq 0 ]]
