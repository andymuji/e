"""Turn an occupancy grid into a picture the console page can draw.

The page is a plain SVG with no map library behind it, so the grid travels as
one grayscale PNG in a data URL rather than as tens of thousands of cells. PNG
because a browser can draw it without help; written here from zlib because the
alternative is a image library the robot does not otherwise need.

Unknown cells stay visibly unknown. A map's blank areas are places the robot
has not seen, and drawing them as free floor would tell an operator the
opposite of the truth.
"""

import base64
from collections.abc import Sequence
import math
import struct
import zlib

# The greys ROS map tools use, so a map looks the same here as in RViz.
OCCUPIED = 0
UNKNOWN = 205
FREE = 254
BETWEEN = 128

# Matching map_server's defaults: below free_thresh is floor, above
# occupied_thresh is wall, and the band between is neither.
FREE_THRESHOLD = 25
OCCUPIED_THRESHOLD = 65


def shade(value: int) -> int:
    """One occupancy value as a grey. Anything off the 0-100 scale is unknown."""
    if value < 0 or value > 100:
        return UNKNOWN
    if value >= OCCUPIED_THRESHOLD:
        return OCCUPIED
    if value <= FREE_THRESHOLD:
        return FREE
    return BETWEEN


def stride_for(width: int, height: int, max_pixels: int) -> int:
    """How many cells to fold into one pixel to stay under max_pixels."""
    cells = width * height
    if cells <= max_pixels or max_pixels <= 0:
        return 1
    return max(1, math.ceil(math.sqrt(cells / max_pixels)))


def _rows(
    data: Sequence[int], width: int, height: int, stride: int
) -> list[bytes]:
    """Grid rows as PNG rows: top of the image first, most-occupied wins.

    An occupancy grid starts at its origin, which is the bottom-left corner in
    the map frame, so the rows come out in reverse. Folding a block of cells
    down to its highest occupancy rather than to a sampled one keeps a wall
    one cell thick from disappearing when the map is shrunk to fit.
    """
    out_width = (width + stride - 1) // stride
    out_height = (height + stride - 1) // stride
    rows = []
    for out_y in range(out_height - 1, -1, -1):
        row = bytearray(out_width)
        for out_x in range(out_width):
            worst = -1
            for y in range(out_y * stride, min((out_y + 1) * stride, height)):
                base = y * width
                for x in range(out_x * stride, min((out_x + 1) * stride, width)):
                    worst = max(worst, data[base + x])
            row[out_x] = shade(worst)
        rows.append(bytes(row))
    return rows


def _chunk(tag: bytes, payload: bytes) -> bytes:
    checksum = zlib.crc32(tag + payload) & 0xFFFFFFFF
    return struct.pack(">I", len(payload)) + tag + payload + struct.pack(">I", checksum)


def grayscale_png(width: int, height: int, rows: Sequence[bytes]) -> bytes:
    """The smallest PNG that says what these rows are: 8-bit grayscale."""
    # Filter byte 0 on every row: no prediction, which zlib compresses well
    # enough for a floor plan of flat greys.
    raw = b"".join(b"\x00" + bytes(row) for row in rows)
    header = struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)
    return (
        b"\x89PNG\r\n\x1a\n"
        + _chunk(b"IHDR", header)
        + _chunk(b"IDAT", zlib.compress(raw, 9))
        + _chunk(b"IEND", b"")
    )


def data_url(
    data: Sequence[int], width: int, height: int, max_pixels: int = 640_000
) -> str | None:
    """The grid as an <image> href, or None when there is nothing to draw."""
    if width <= 0 or height <= 0 or len(data) < width * height:
        # A short payload is a truncated map, not a map with free space at the
        # end of it.
        return None
    stride = stride_for(width, height, max_pixels)
    rows = _rows(data, width, height, stride)
    png = grayscale_png(len(rows[0]), len(rows), rows)
    return "data:image/png;base64," + base64.b64encode(png).decode("ascii")
