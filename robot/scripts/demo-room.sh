#!/usr/bin/env bash
# Turn a Polycam phone scan (.glb, or the .zip it comes in) into a labelled
# floor plan and open it. Output goes to a temp folder, never the repo
# (scans show the inside of a home - docs/guides/privacy.md).
# Usage: robot/scripts/demo-room.sh SCAN.zip|SCAN.glb
set -euo pipefail

SCAN=${1:?usage: $0 SCAN.zip|SCAN.glb}
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=$(mktemp -d)

case $SCAN in
  *.zip) python3 -c 'import sys, zipfile; z = zipfile.ZipFile(sys.argv[1])
sys.stdout.buffer.write(z.read(next(n for n in z.namelist() if n.lower().endswith(".glb"))))' "$SCAN" > "$OUT/room.glb" ;;
  *) cp "$SCAN" "$OUT/room.glb" ;;
esac

PYTHONPATH="$HERE/../ros2_ws/src/robot_bringup" python3 -m robot_bringup.scan_to_map \
  "$OUT/room.glb" -f "$OUT/room" --report "$OUT/room.json" > /dev/null
PNG="$OUT/$(basename "${SCAN%.*}") floor plan.png"
python3 "$HERE/draw_floor_plan.py" "$OUT/room.pgm" "$OUT/room.yaml" "$OUT/room.json" "$PNG"

python3 -c 'import json, sys; r = json.load(open(sys.argv[1]))
print("Floor area %.1f m2, %d things on the floor" % (r["floor"]["area_m2"], len(r["obstacles"])))' "$OUT/room.json"
if [ "$(uname)" = Darwin ]; then open "$PNG"
elif [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ] && command -v xdg-open > /dev/null; then xdg-open "$PNG"
else echo "Floor plan: $PNG"
fi
