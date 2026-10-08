#!/usr/bin/env python3
"""Build the diagnostic asset from the specific 2026 two-shooter Onshape export.

Usage: python build_model.py '/path/to/Assembly 1.gltf'
Node IDs refer to that export, not arbitrary later Onshape versions. No robot
calibration is inferred here; see visualization/advantagescope/README.md.
"""

import argparse
import base64
import json
import math
import re
from pathlib import Path

import numpy as np
import trimesh

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("source", type=Path)
args = parser.parse_args()
# Generated from the compiled ShooterConstants, not a second hand-maintained geometry.
geometry = json.loads((Path(__file__).parent / "geometry.json").read_text())
code_pivot = np.array(geometry["turret"])
flip = np.diag([-1.0, -1.0, 1.0])
code_hood_pivot = code_pivot + flip @ np.array(geometry["hoodOffset"])
out = (
    Path(__file__).resolve().parents[2] / "visualization/advantagescope/Robot_3464_2026"
)
out.mkdir(parents=True, exist_ok=True)
d = json.loads(args.source.read_text())
assert (
    len(d["nodes"]) == 5210
), "Different CAD export: re-identify the parts before building."
assert d["nodes"][2330]["name"] == "real ahh turret <2>"
assert d["nodes"][3036]["name"] == "real ahh turret <1>"
assert d["nodes"][2923]["name"] == "Right Aimer Panel"
buf = base64.b64decode(d["buffers"][0]["uri"].split(",", 1)[1])
world = {}
parent = {}


def walk(i, m):
    n = d["nodes"][i]
    a = np.array(n.get("matrix", np.eye(4).T.flatten())).reshape(4, 4).T
    m = m @ a
    world[i] = m
    for c in n.get("children", []):
        parent[c] = i
        walk(c, m)


walk(0, np.eye(4))


def acc(i):
    a = d["accessors"][i]
    v = d["bufferViews"][a["bufferView"]]
    types = {5126: "<f4", 5125: "<u4", 5123: "<u2", 5121: "u1"}
    k = {"SCALAR": 1, "VEC3": 3, "VEC2": 2, "VEC4": 4}[a["type"]]
    return np.frombuffer(
        buf,
        dtype=types[a["componentType"]],
        count=a["count"] * k,
        offset=v.get("byteOffset", 0) + a.get("byteOffset", 0),
    ).reshape(-1, k)


meshes = {}
for i, m in enumerate(d["meshes"]):
    parts = []
    for x in m["primitives"]:
        if x.get("mode", 4) != 4:
            continue
        parts.append(
            (
                acc(x["attributes"]["POSITION"]).copy(),
                acc(x["indices"]).reshape(-1, 3).copy(),
                x.get("material", 0),
            )
        )
    meshes[i] = parts
bounds = {}
for i, n in enumerate(d["nodes"]):
    if "mesh" not in n:
        continue
    v = np.concatenate([x[0] for x in meshes[n["mesh"]]])
    v = v @ world[i][:3, :3].T + world[i][:3, 3]
    bounds[i] = np.array([v.min(0), v.max(0)])

# The supplied rear-view annotation retains turret <2>, on CAD +Y.
pivot = np.array([-0.09523143, 0.21118073, 0.36703])
angle = -math.pi / 2 + math.atan2(0.00547, 0.99999)
R = np.array(
    [
        [math.cos(angle), -math.sin(angle), 0],
        [math.sin(angle), math.cos(angle), 0],
        [0, 0, 1],
    ]
)
hood_source = np.array([-0.095, 0.12227, 0.42037])
hood_pivot = (hood_source - pivot) @ R.T + pivot
hood_ids = {2923, 2965, 2925, 2949}
body_roots = {5, 11, 60, 139, 443, 637, 1790}
# Fixed bearing plate/ring and mounting tubes are part of the body, not the rotating assembly.
fixed_ids = {2332, 2336, 2360, 2374, 2366, 2382, 2370}
rotating_ids = {2346} | set(range(2914, 3036))
scenes = [trimesh.Scene(), trimesh.Scene(), trimesh.Scene()]
stats = [0, 0, 0]
cache = {}
kept = []


def root(i):
    while parent.get(i, 0) != 0:
        i = parent[i]
    return i


for i, n in enumerate(d["nodes"]):
    if "mesh" not in n:
        continue
    rt = root(i)
    if rt == 3036:
        continue
    if i in hood_ids:
        group = 2
    elif i in rotating_ids:
        group = 1
    elif rt in body_roots or i in fixed_ids:
        group = 0
    else:
        continue
    name = n.get("name", "")
    b = bounds[i]
    size = b[1] - b[0]
    c = b.mean(0)
    if (
        re.search(
            "screw|SHCS|washer|bearing|nut|retaining|spacer|sun gear|planet gear",
            name,
            re.I,
        )
        or max(size) < 0.035
    ):
        continue
    # Disconnected/exploded CAD instances are not useful display geometry.
    if abs(c[1]) > 0.49 or c[2] < 0:
        continue
    if group > 0 and (c[0] < -0.25 or c[1] < 0.07 or c[1] > 0.38):
        continue
    mi = n["mesh"]
    if mi not in cache:
        vv = []
        ff = []
        count = 0
        for v, f, mat in meshes[mi]:
            vv.append(v)
            ff.append(f + count)
            count += len(v)
        v = np.concatenate(vv)
        f = np.concatenate(ff)
        # Weld CAD face seams and cluster sub-millimetre detail before decimation.
        _, idx, inv = np.unique(
            np.round(v / 0.0008).astype(np.int64),
            axis=0,
            return_index=True,
            return_inverse=True,
        )
        v = v[idx]
        f = inv[f]
        f = f[(f[:, 0] != f[:, 1]) & (f[:, 0] != f[:, 2]) & (f[:, 1] != f[:, 2])]
        _, keep = np.unique(np.sort(f, axis=1), axis=0, return_index=True)
        f = f[keep]
        m = trimesh.Trimesh(v, f, process=False)
        if len(m.faces) > 1600:
            m = m.simplify_quadric_decimation(face_count=1600, aggression=10)
        mat = max(meshes[mi], key=lambda t: len(t[1]))[2]
        col = (
            d["materials"][mat]
            .get("pbrMetallicRoughness", {})
            .get("baseColorFactor", [0.5, 0.5, 0.5, 1])
        )
        cache[mi] = (m, col)
    m, col = cache[mi]
    v = m.vertices @ world[i][:3, :3].T + world[i][:3, 3]
    if group > 0:
        v = (v - pivot) @ R.T + pivot
        # Keep each mesh's shape but place its joint at the shared code's zero-angle pose.
        v += (code_pivot - pivot) if group == 1 else (code_hood_pivot - hood_pivot)
    elif i in fixed_ids:
        v += code_pivot - pivot
    if group == 2:
        col = [0.95, 0.6, 0.15, 1]  # Identify the moving hood in the diagnostic model.
    m = trimesh.Trimesh(v, m.faces.copy(), process=False)
    normals = np.zeros_like(m.vertices)
    for corner in range(3):
        np.add.at(
            normals,
            m.faces[:, corner],
            m.face_normals * m.face_angles[:, corner, None],
        )
    lengths = np.linalg.norm(normals, axis=1)
    normals /= np.maximum(lengths[:, None], 1e-12)
    m.vertex_normals = normals
    # AdvantageScope reads material colors and skips meshes without normals.
    # Vertex colors alone render in some viewers but are discarded by its loader.
    m.visual = trimesh.visual.TextureVisuals(
        material=trimesh.visual.material.PBRMaterial(
            baseColorFactor=(np.array(col) * 255).astype(np.uint8),
            metallicFactor=0.0,
            roughnessFactor=0.8,
        )
    )
    scenes[group].add_geometry(m, node_name=f"{i}_{name}", geom_name=f"{i}_{name}")
    stats[group] += len(m.faces)
    kept.append((i, group, name))
for j, s in enumerate(scenes):
    fn = "model.glb" if j == 0 else f"model_{j-1}.glb"
    (out / fn).write_bytes(s.export(file_type="glb", include_normals=True))
# Meshes are stored in the shared code's zero-angle reference pose. Undo that
# placement before applying the live robot-relative component poses from Java.
flip = np.diag([-1.0, -1.0, 1.0])
config = {
    "name": "3464 2026 — CAD draft",
    "rotations": [],
    "position": [0, 0, 0],
    "cameras": [],
    "components": [],
}
for joint in [code_pivot, code_hood_pivot]:
    config["components"].append(
        {
            "zeroedRotations": [{"axis": "z", "degrees": 180}],
            "zeroedPosition": (-flip @ joint).tolist(),
        }
    )
(out / "config.json").write_text(json.dumps(config, indent=2) + "\n")
print(
    "Turret pivot",
    code_pivot.tolist(),
    "Hood pivot",
    code_hood_pivot.tolist(),
    "Hood local offset",
    geometry["hoodOffset"],
)
print(
    "Triangles:",
    stats,
    "Size MB:",
    sum(x.stat().st_size for x in out.glob("*.glb")) / 1e6,
)
