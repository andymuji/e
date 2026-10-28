#!/usr/bin/env python3
"""Draw a phone-scan map as a labelled floor plan PNG.

Usage: draw_floor_plan.py ROOM.pgm ROOM.yaml ROOM.json OUT.png
"""
import json
import sys

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:
    print("Pillow is missing - run: pip3 install pillow")
    sys.exit(1)

SCALE = 7          # screen pixels per map cell
MARGIN = 70        # room for the dimension lines
RED = (210, 30, 30)
BLUE = (30, 80, 200)


def font(size):
    for name in ("/System/Library/Fonts/Helvetica.ttc",
                 "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                 "DejaVuSans.ttf", "Arial.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            pass
    return ImageFont.load_default()


def read_pgm(path):
    with open(path, "rb") as f:
        data = f.read()
    # Header is "P5 <w> <h> <max>" then raw bytes.
    parts = data.split(None, 4)
    w, h = int(parts[1]), int(parts[2])
    return w, h, data[len(data) - w * h:]


def read_yaml(path):
    # Only two keys are needed, so no yaml library.
    info = {}
    for line in open(path):
        key, _, value = line.partition(":")
        if key.strip() == "resolution":
            info["resolution"] = float(value)
        elif key.strip() == "origin":
            info["origin"] = [float(v) for v in value.strip(" []\n").split(",")]
    return info


def describe(o, lidar_height):
    low, high = o["beam_range"]
    if low > lidar_height:
        text = f"hanging {round(low * 100)} cm up"
    else:
        text = f"{round(o['height'] * 100)} cm tall"
    if o["kind"] == "step":
        text = "step, " + text
    if o["trip_hazard"]:
        text += " - trip hazard"
    elif o["lidar"] == "misses":
        text += " - lidar can't see it"
    return f"{o['id']}: {text}"


def main():
    if len(sys.argv) != 5:
        print(__doc__.strip())
        sys.exit(2)
    pgm, yaml_path, json_path, out = sys.argv[1:]
    w, h, pixels = read_pgm(pgm)
    info = read_yaml(yaml_path)
    res, (ox, oy) = info["resolution"], info["origin"][:2]
    report = json.load(open(json_path))

    # World metres -> picture pixels. Row 0 of the pgm is the top (max y).
    def px(x, y):
        return (MARGIN + (x - ox) / res * SCALE,
                MARGIN + (h - (y - oy) / res) * SCALE)

    big, small = font(22), font(16)
    obstacles = report["obstacles"]
    legend = [describe(o, report["lidar_height"]) for o in obstacles]
    panel_w = 360
    W = w * SCALE + 2 * MARGIN + panel_w
    H = max(h * SCALE + 2 * MARGIN, 260 + 26 * len(legend))
    img = Image.new("RGB", (W, H), "white")
    d = ImageDraw.Draw(img)

    # The map itself: 0 is wall, 254 free, anything else unscanned.
    shades = {0: (0, 0, 0), 254: (255, 255, 255)}
    grid = Image.new("RGB", (w, h))
    grid.putdata([shades.get(p, (215, 215, 215)) for p in pixels])
    img.paste(grid.resize((w * SCALE, h * SCALE), Image.NEAREST), (MARGIN, MARGIN))

    # Overall size of the room: the box around every scanned cell.
    cols = [i % w for i, p in enumerate(pixels) if p != 205]
    rows = [i // w for i, p in enumerate(pixels) if p != 205]
    left, right = MARGIN + min(cols) * SCALE, MARGIN + (max(cols) + 1) * SCALE
    top, bottom = MARGIN + min(rows) * SCALE, MARGIN + (max(rows) + 1) * SCALE
    width = (right - left) / SCALE * res
    length = (bottom - top) / SCALE * res
    ty = MARGIN - 30
    d.line([(left, ty), (right, ty)], fill=BLUE, width=2)
    for x in (left, right):
        d.line([(x, ty - 8), (x, ty + 8)], fill=BLUE, width=2)
    d.text(((left + right) / 2, ty - 4), f"{width:.1f} m", fill=BLUE,
           font=big, anchor="md")
    lx = MARGIN - 30
    d.line([(lx, top), (lx, bottom)], fill=BLUE, width=2)
    for y in (top, bottom):
        d.line([(lx - 8, y), (lx + 8, y)], fill=BLUE, width=2)
    label = Image.new("RGBA", (200, 30), (0, 0, 0, 0))
    ImageDraw.Draw(label).text((100, 15), f"{length:.1f} m", fill=BLUE,
                               font=big, anchor="mm")
    label = label.rotate(90, expand=True)
    img.paste(label, (int(lx - 32), int((top + bottom) / 2 - 100)), label)

    # Obstacles: outline plus a numbered circle, since many are tiny.
    for o in obstacles:
        points = [px(x, y) for x, y in o["outline"]]
        if len(points) > 2:
            d.polygon(points, outline=RED)
        cx, cy = px(*o["center"])
        d.ellipse([cx - 13, cy - 13, cx + 13, cy + 13], outline=RED, width=3)
        d.text((cx + 15, cy - 15), str(o["id"]), fill=RED, font=big)

    # Key on the right.
    x = w * SCALE + 2 * MARGIN
    y = MARGIN
    d.text((x, y), "Floor plan", fill="black", font=big)
    y += 34
    d.text((x, y), f"Floor area: {report['floor']['area_m2']:.1f} m²",
           fill="black", font=small)
    y += 30
    bar = 1.0 / res * SCALE
    d.line([(x, y + 10), (x + bar, y + 10)], fill="black", width=4)
    d.text((x + bar + 10, y + 10), "1 m", fill="black", font=small, anchor="lm")
    y += 34
    for colour, name in (((0, 0, 0), "wall"), ((215, 215, 215), "not scanned")):
        d.rectangle([x, y, x + 18, y + 18], fill=colour, outline="black")
        d.text((x + 28, y), name, fill="black", font=small)
        y += 26
    y += 14
    d.text((x, y), "On the floor (red):", fill=RED, font=small)
    y += 26
    for line in legend or ["nothing found"]:
        d.text((x, y), line, fill="black", font=small)
        y += 26

    img.save(out)


if __name__ == "__main__":
    main()
