"""Renders assets/icon.svg into the program icons: icon.ico (Windows), icon.icns (macOS) and
icon-128.rgba (raw pixels for the window icon).

    python scripts/make_icons.py

Needs Python 3 with numpy. The renderer only understands what icon.svg uses: one rounded
rectangle (fill and stroke) and filled circles (with fill-opacity) drawn inside it.
"""

from __future__ import annotations

import re
import struct
import zlib
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
ASSETS = ROOT / "assets"
VIEW = 256.0


def colour(hex_value: str) -> np.ndarray:
    h = hex_value.lstrip("#")
    return np.array([int(h[i:i + 2], 16) / 255 for i in (0, 2, 4)], dtype=np.float32)


def parse(svg: str):
    rect = re.search(r'<rect x="([\d.]+)" y="([\d.]+)" width="([\d.]+)" height="([\d.]+)" rx="([\d.]+)" '
                     r'fill="(#\w+)"(?: stroke="(#\w+)" stroke-width="([\d.]+)")?', svg)
    x, y, w, h, rx = (float(v) for v in rect.group(1, 2, 3, 4, 5))
    tile = {"centre": (x + w / 2, y + h / 2), "half": (w / 2, h / 2), "radius": rx, "fill": colour(rect.group(6)),
            "stroke": colour(rect.group(7)) if rect.group(7) else None,
            "width": float(rect.group(8)) if rect.group(8) else 0.0}
    circles = [(float(cx), float(cy), float(r), colour(fill), float(op)) for cx, cy, r, fill, op in re.findall(
        r'<circle cx="([\d.-]+)" cy="([\d.-]+)" r="([\d.]+)" fill="(#\w+)" fill-opacity="([\d.]+)"', svg)]
    return tile, circles


def render(tile, circles, size: int) -> np.ndarray:
    """RGBA8 image of the given size: supersampled, then box-filtered."""
    ss = max(4, -(-256 // size))
    n = size * ss
    scale = n / VIEW
    coords = (np.arange(n, dtype=np.float32) + 0.5) / scale
    px, py = np.meshgrid(coords, coords)
    rgb = np.zeros((n, n, 3), np.float32)              # premultiplied colour
    alpha = np.zeros((n, n), np.float32)

    def over(mask, col, op=1.0):
        a = mask.astype(np.float32) * op
        rgb[:] = rgb * (1 - a)[..., None] + col[None, None, :] * a[..., None]
        alpha[:] = alpha * (1 - a) + a

    (cx, cy), (hx, hy), r = tile["centre"], tile["half"], tile["radius"]
    qx, qy = np.abs(px - cx) - (hx - r), np.abs(py - cy) - (hy - r)
    dist = np.hypot(np.maximum(qx, 0), np.maximum(qy, 0)) + np.minimum(np.maximum(qx, qy), 0) - r
    over(dist <= 0, tile["fill"])
    if tile["stroke"] is not None:
        over(np.abs(dist) <= tile["width"] / 2, tile["stroke"])
    inside = dist <= 0
    for x, y, rad, col, op in circles:
        over(((px - x) ** 2 + (py - y) ** 2 <= rad * rad) & inside, col, op)

    rgb = rgb.reshape(size, ss, size, ss, 3).mean(axis=(1, 3))
    alpha = alpha.reshape(size, ss, size, ss).mean(axis=(1, 3))
    with np.errstate(invalid="ignore", divide="ignore"):
        straight = np.where(alpha[..., None] > 0, rgb / alpha[..., None], 0)
    out = np.concatenate([straight, alpha[..., None]], axis=2)
    return np.clip(np.round(out * 255), 0, 255).astype(np.uint8)


def png(image: np.ndarray) -> bytes:
    h, w, _ = image.shape
    raw = b"".join(b"\x00" + image[row].tobytes() for row in range(h))

    def chunk(kind: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data))
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 6, 0, 0, 0))
            + chunk(b"IDAT", zlib.compress(raw, 9)) + chunk(b"IEND", b""))


def ico(images: dict[int, bytes]) -> bytes:
    sizes = sorted(images)
    head = struct.pack("<HHH", 0, 1, len(sizes))
    offset = 6 + 16 * len(sizes)
    entries, data = b"", b""
    for s in sizes:
        blob = images[s]
        entries += struct.pack("<BBBBHHII", s % 256, s % 256, 0, 0, 1, 32, len(blob), offset + len(data))
        data += blob
    return head + entries + data


def icns(images: dict[int, bytes]) -> bytes:
    kinds = [("icp4", 16), ("icp5", 32), ("icp6", 64), ("ic07", 128), ("ic08", 256), ("ic09", 512),
             ("ic10", 1024), ("ic11", 32), ("ic12", 64), ("ic13", 256), ("ic14", 512)]
    body = b"".join(k.encode() + struct.pack(">I", 8 + len(images[s])) + images[s] for k, s in kinds)
    return b"icns" + struct.pack(">I", 8 + len(body)) + body


def main():
    tile, circles = parse((ASSETS / "icon.svg").read_text(encoding="utf-8"))
    pngs = {s: png(render(tile, circles, s)) for s in (16, 20, 24, 32, 40, 48, 64, 128, 256, 512, 1024)}
    (ASSETS / "icon.ico").write_bytes(ico({s: pngs[s] for s in (16, 20, 24, 32, 40, 48, 64, 128, 256)}))
    (ASSETS / "icon.icns").write_bytes(icns(pngs))
    (ASSETS / "icon-128.rgba").write_bytes(render(tile, circles, 128).tobytes())
    (ASSETS / "icon-256.png").write_bytes(pngs[256])
    print("wrote icon.ico, icon.icns, icon-128.rgba, icon-256.png")


if __name__ == "__main__":
    main()
