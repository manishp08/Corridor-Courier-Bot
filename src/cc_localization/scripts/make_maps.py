#!/usr/bin/env python3
"""Rasterise reference maps (PGM + YAML) from the world manifests.

These are what a perfect LiDAR SLAM pass on the *empty* world would produce:
walls only; no glass (the LiDAR cannot see it), no clutter, no people. They
let AMCL run out of the box. Milestone 4 replaces them with slam_toolbox maps
(ros2 run nav2_map_server map_saver_cli -f maps/<world>) and compares both.

  python3 src/cc_localization/scripts/make_maps.py
"""
import json
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve()
WORLDS = HERE.parents[2] / "cc_gazebo" / "worlds"
OUT = HERE.parents[1] / "maps"
RES = 0.05
WALL_T = 0.1


def rasterize(manifest):
    x0, y0, x1, y1 = manifest["bounds"]
    nx, ny = int(np.ceil((x1 - x0) / RES)), int(np.ceil((y1 - y0) / RES))
    ys, xs = np.mgrid[0:ny, 0:nx]
    cx = x0 + (xs + 0.5) * RES
    cy = y0 + (ys + 0.5) * RES
    occ = np.zeros((ny, nx), bool)
    for ax, ay, bx, by in manifest["walls"]:
        ex, ey = bx - ax, by - ay
        t = np.clip(((cx - ax) * ex + (cy - ay) * ey) / max(ex * ex + ey * ey, 1e-12), 0, 1)
        occ |= np.hypot(ax + t * ex - cx, ay + t * ey - cy) <= WALL_T / 2 + RES / 2
    return occ, (x0, y0)


def main():
    OUT.mkdir(exist_ok=True)
    for f in sorted(WORLDS.glob("*.json")):
        m = json.loads(f.read_text())
        occ, origin = rasterize(m)
        img = np.where(occ, 0, 254).astype(np.uint8)[::-1]      # PGM rows go top-down
        name = m["world"]
        with open(OUT / f"{name}.pgm", "wb") as fp:
            fp.write(f"P5\n{img.shape[1]} {img.shape[0]}\n255\n".encode())
            fp.write(img.tobytes())
        (OUT / f"{name}.yaml").write_text(
            f"image: {name}.pgm\nmode: trinary\nresolution: {RES}\n"
            f"origin: [{origin[0]}, {origin[1]}, 0.0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
        print("wrote", OUT / f"{name}.yaml", file=sys.stderr)


if __name__ == "__main__":
    main()
