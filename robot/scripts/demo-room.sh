#!/usr/bin/env bash
# Turn a phone scan of a room into the robot's map, live: say what is on
# the floor, open the map as a picture, and serve the 3D room page.
#
# Usage, from anywhere:
#   robot/scripts/demo-room.sh docs/runs/10_12_2026.zip [PORT]   default 8000
# The scan is a Polycam export, .glb or the .zip it is sent in.
#
# Needs only python3 - no ROS, nothing installed. Everything it makes goes
# in a temporary folder, deleted on Ctrl+C, never in the repository: a
# textured scan shows the inside of a home (docs/privacy.md). Nothing here
# can move anything.
set -euo pipefail

SCAN=${1:?usage: $0 SCAN.zip|SCAN.glb [PORT]}
PORT=${2:-8000}
REPO=$(cd "$(dirname "$0")/../.." && pwd)
OUT=$(mktemp -d)
trap 'rm -rf "$OUT"' EXIT

# Open a file or link the way this computer does: Preview and the browser
# on a Mac, the desktop's own viewer on Linux. A machine with no screen,
# such as a Codespace, cannot; the link printed below still works there.
show() {
  if command -v open >/dev/null && [ "$(uname)" = Darwin ]; then open "$1"
  elif command -v xdg-open >/dev/null && [ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]; then
    xdg-open "$1" >/dev/null 2>&1
  else return 1
  fi
}

echo "== Reading the scan: $(basename "$SCAN")"
case $SCAN in
  *.zip) python3 -I -c '
import sys, zipfile
z = zipfile.ZipFile(sys.argv[1])
glb = next(n for n in z.namelist() if n.lower().endswith(".glb"))
open(sys.argv[2], "wb").write(z.read(glb))' "$SCAN" "$OUT/room.glb" ;;
  *) cp "$SCAN" "$OUT/room.glb" ;;
esac

echo "== Turning it into the robot's map"
PYTHONPATH=$REPO/robot/ros2_ws/src/robot_bringup \
  python3 -m robot_bringup.scan_to_map "$OUT/room.glb" -f "$OUT/room" --report "$OUT/room.json"

echo
echo "== What stands on the floor"
python3 -I - "$OUT/room.json" <<'EOF'
import json, sys
r = json.load(open(sys.argv[1]))
print(f"  floor: {r['floor']['area_m2']} m2, tilted {r['floor']['tilt_deg']} deg")
print(f"  walls: {len(r['walls'])} pieces")
for o in r["obstacles"]:
    above = o["beam_range"][0] > r["lidar_height"]
    sees = {"sees": "lidar sees it", "marginal": "lidar may miss it",
            "misses": "above the lidar's beam" if above else "too low for the lidar"}[o["lidar"]]
    trip = ", TRIP HAZARD" if o["trip_hazard"] else ""
    tall = (f"hanging {o['beam_range'][0] * 100:.0f} cm up" if above
            else f"{o['height'] * 100:.0f} cm tall")
    print(f"  {o['kind']} at ({o['center'][0]:.1f}, {o['center'][1]:.1f}) m: "
          f"{tall}, {sees}{trip}")
EOF

# The map as a picture, four screen pixels to each 5 cm cell: black is
# wall, white is free floor, grey is floor the phone never saw.
python3 -I - "$OUT/room.pgm" "$OUT/room.png" <<'EOF'
import struct, sys, zlib
head, size, _, pixels = open(sys.argv[1], "rb").read().split(b"\n", 3)
w, h = map(int, size.split())
k = 4
rows = b"".join(b"\0" + bytes(p for p in pixels[y * w:(y + 1) * w] for _ in range(k))
                for y in range(h) for _ in range(k))
def chunk(kind, body):
    return struct.pack(">I", len(body)) + kind + body + struct.pack(">I", zlib.crc32(kind + body))
open(sys.argv[2], "wb").write(
    b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w * k, h * k, 8, 0, 0, 0, 0))
    + chunk(b"IDAT", zlib.compress(rows)) + chunk(b"IEND", b""))
EOF

echo
echo "== The map"
cp "$REPO/docs/room-viewer/index.html" "$OUT/"
URL=http://localhost:$PORT
show "$OUT/room.png" || echo "  (no screen here to open it on: it is at $URL/room.png)"

echo
echo "== The 3D room: $URL"
echo "   (In a Codespace, click \"Open in Browser\" when VS Code offers it.)"
echo "   Ctrl+C to stop."
(sleep 1; show "$URL" || true) &
python3 -m http.server "$PORT" --bind 127.0.0.1 --directory "$OUT" 2>/dev/null
