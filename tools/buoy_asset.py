#!/usr/bin/env python3
"""Convierte el asset de la boya (Meshy AI, OBJ con y hacia arriba) para Stonefish.

    uv run --with numpy --with scipy --with fast-simplification --with pyyaml \\
        python tools/buoy_asset.py <boya.obj> <boya.png> data/buoy [--target-faces 5000] [--height 0.30]

Salida en <out>:
  buoy_vis.obj  malla visual decimada con UV y normales, NED (z abajo), escalada, centrada en su caja
  buoy.png      textura (copia)
  buoy_phy.obj  casco convexo para la física
  buoy.yaml     height, eye (posición del ojal en ese marco) y volume del casco

La decimación se hace sobre la malla soldada por posición (sin grietas en las costuras de UV);
cada esquina de cara conserva la UV de la esquina original de la que viene.
"""
import argparse
import shutil
from pathlib import Path

import fast_simplification
import numpy as np
import yaml
from scipy.spatial import ConvexHull

EYE_FRACTION = 0.03   # el ojal: vértices en el 3 % más alto
HULL_FACES = 400      # decimación previa al casco convexo de la física


def read_obj(path):
    v, t, faces = [], [], []
    for line in Path(path).read_text().splitlines():
        p = line.split()
        if not p:
            continue
        if p[0] == "v":
            v.append([float(x) for x in p[1:4]])
        elif p[0] == "vt":
            t.append([float(x) for x in p[1:3]])
        elif p[0] == "f":
            corners = [c.split("/") for c in p[1:]]
            vi = [int(c[0]) - 1 for c in corners]
            ti = [int(c[1]) - 1 if len(c) > 1 and c[1] else -1 for c in corners]
            for k in range(1, len(corners) - 1):        # triangula polígonos en abanico
                faces.append(([vi[0], vi[k], vi[k + 1]], [ti[0], ti[k], ti[k + 1]]))
    fv = np.array([f[0] for f in faces], dtype=np.int64)
    ft = np.array([f[1] for f in faces], dtype=np.int64)
    return np.array(v, float), np.array(t, float).reshape(-1, 2), fv, ft


def to_ned(v):
    """Asset (y arriba) → NED (z abajo): (x, y, z) → (x, z, −y), giro propio."""
    return np.column_stack([v[:, 0], v[:, 2], -v[:, 1]])


def decimate(points, fv, ft, target):
    if len(fv) <= target:
        return points, fv, ft
    pts, _, collapses = fast_simplification.simplify(points, fv, target_count=target, return_collapses=True)
    pts, faces, mapping = fast_simplification.replay_simplification(points, fv, collapses)
    mapped = mapping[fv]
    keep = (mapped[:, 0] != mapped[:, 1]) & (mapped[:, 1] != mapped[:, 2]) & (mapped[:, 0] != mapped[:, 2])
    _, first = np.unique(np.sort(mapped[keep], axis=1), axis=0, return_index=True)
    rows = np.flatnonzero(keep)[np.sort(first)]
    return pts, mapped[rows], ft[rows]


def vertex_normals(v, f):
    n = np.zeros_like(v)
    fn = np.cross(v[f[:, 1]] - v[f[:, 0]], v[f[:, 2]] - v[f[:, 0]])
    for k in range(3):
        np.add.at(n, f[:, k], fn)
    return n / np.maximum(np.linalg.norm(n, axis=1, keepdims=True), 1e-12)


def write_vis(path, v, uv, fv, ft):
    n = vertex_normals(v, fv)
    lines = ["# GENERADO por tools/buoy_asset.py"]
    lines += ["v %.6f %.6f %.6f" % tuple(p) for p in v]
    lines += ["vt %.6f %.6f" % tuple(p) for p in uv]
    lines += ["vn %.6f %.6f %.6f" % tuple(p) for p in n]
    lines += ["f " + " ".join(f"{a + 1}/{b + 1}/{a + 1}" for a, b in zip(f, t)) for f, t in zip(fv, ft)]
    Path(path).write_text("\n".join(lines) + "\n")


def write_hull(path, v):
    hull = ConvexHull(v)
    idx = {old: new for new, old in enumerate(hull.vertices)}
    pts = np.round(v[hull.vertices], 9)   # lo que se escribe; el volumen se calcula sobre esto
    lines = ["# GENERADO por tools/buoy_asset.py (casco convexo)"]
    lines += ["v %.9f %.9f %.9f" % tuple(p) for p in pts]
    for k, s in enumerate(hull.simplices):
        a, b, c = (idx[i] for i in s)
        nrm = hull.equations[k][:3]
        if np.dot(np.cross(pts[b] - pts[a], pts[c] - pts[a]), nrm) < 0:
            b, c = c, b
        lines.append("vn %.6f %.6f %.6f" % tuple(nrm))
        lines.append(f"f {a + 1}//{k + 1} {b + 1}//{k + 1} {c + 1}//{k + 1}")
    Path(path).write_text("\n".join(lines) + "\n")
    return ConvexHull(pts).volume


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("obj")
    ap.add_argument("png")
    ap.add_argument("out")
    ap.add_argument("--target-faces", type=int, default=5000)
    ap.add_argument("--height", type=float, default=0.30)
    a = ap.parse_args(argv)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)

    v, uv, fv, ft = read_obj(a.obj)
    v = to_ned(v)
    pts, fv2, ft2 = decimate(v, fv, ft, a.target_faces)
    # marco común: caja de la malla visual ya decimada, escalada a la altura pedida
    lo, hi = pts.min(axis=0), pts.max(axis=0)
    scale = a.height / (hi[2] - lo[2])
    pts = (pts - (lo + hi) / 2) * scale
    v = (v - (lo + hi) / 2) * scale
    zmin = v[:, 2].min()
    eye = v[v[:, 2] <= zmin + EYE_FRACTION * a.height].mean(axis=0)

    write_vis(out / "buoy_vis.obj", pts, uv, fv2, ft2)
    hull_pts, _, _ = decimate(v, fv, ft, HULL_FACES)
    volume = write_hull(out / "buoy_phy.obj", hull_pts)
    shutil.copyfile(a.png, out / "buoy.png")
    meta = {"height": float(a.height), "eye": [float(x) for x in eye], "volume": float(volume)}
    (out / "buoy.yaml").write_text(yaml.safe_dump(meta, sort_keys=False))
    print(f"{len(fv2)} caras, ojal {np.round(eye, 4).tolist()}, volumen {volume * 1000:.2f} L")


if __name__ == "__main__":
    main()
