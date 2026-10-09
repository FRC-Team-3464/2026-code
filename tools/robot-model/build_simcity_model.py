#!/usr/bin/env python3
"""Losslessly package 2026-ROBOT.gltf as an articulated SIMCITY-3464-2026 asset.

All nodes, materials, normals, UVs, triangles, and assembly transforms are retained.
Only primitives with identical render settings are batched, and byte-identical
vertex records are shared. No parts are omitted or geometry decimated.
"""

import argparse
import base64
import copy
import hashlib
import json
import struct
from pathlib import Path

import numpy as np


def write_component(path, cad, binary, members):
    """Copy a mesh subset without changing vertices or assembly transforms."""
    asset = copy.deepcopy(cad)
    meshes = sorted({cad["nodes"][i]["mesh"] for i in members})
    mesh_map = {old: new for new, old in enumerate(meshes)}
    for i, node in enumerate(asset["nodes"]):
        if "mesh" in node:
            if i in members:
                node["mesh"] = mesh_map[node["mesh"]]
            else:
                del node["mesh"]
    asset["meshes"] = [asset["meshes"][i] for i in meshes]
    accessors = set()
    for mesh in asset["meshes"]:
        for p in mesh["primitives"]:
            accessors.update(p["attributes"].values())
            accessors.add(p["indices"])
    accessors = sorted(accessors)
    mapping = {old: new for new, old in enumerate(accessors)}
    packed = bytearray()
    asset["accessors"], asset["bufferViews"] = [], []
    for i in accessors:
        a = copy.deepcopy(cad["accessors"][i])
        view = copy.deepcopy(cad["bufferViews"][a["bufferView"]])
        begin = view.get("byteOffset", 0)
        packed.extend(b"\0" * (-len(packed) % 4))
        view["byteOffset"] = len(packed)
        packed.extend(binary[begin : begin + view["byteLength"]])
        a["bufferView"] = len(asset["bufferViews"])
        asset["bufferViews"].append(view)
        asset["accessors"].append(a)
    for mesh in asset["meshes"]:
        for p in mesh["primitives"]:
            p["attributes"] = {name: mapping[i] for name, i in p["attributes"].items()}
            p["indices"] = mapping[p["indices"]]
    asset["buffers"] = [{"byteLength": len(packed)}]
    header = json.dumps(asset, separators=(",", ":")).encode()
    header += b" " * (-len(header) % 4)
    packed.extend(b"\0" * (-len(packed) % 4))
    glb = struct.pack("<III", 0x46546C67, 2, 28 + len(header) + len(packed))
    glb += struct.pack("<II", len(header), 0x4E4F534A) + header
    glb += struct.pack("<II", len(packed), 0x004E4942) + packed
    path.write_bytes(glb)
    return len(glb)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    args = parser.parse_args()
    source = args.source.read_bytes()
    cad = json.loads(source)
    assert len(cad["buffers"]) == 1, "Expected the embedded Onshape export"
    original = base64.b64decode(cad["buffers"][0]["uri"].split(",", 1)[1])
    output = copy.deepcopy(cad)
    output["accessors"], output["bufferViews"] = [], []
    binary = bytearray()

    def read(i):
        a = cad["accessors"][i]
        view = cad["bufferViews"][a["bufferView"]]
        assert "byteStride" not in view and "sparse" not in a
        assert not a.get("normalized", False)
        dtype = {5125: "<u4", 5126: "<f4"}[a["componentType"]]
        width = {"SCALAR": 1, "VEC2": 2, "VEC3": 3}[a["type"]]
        return np.frombuffer(
            original,
            dtype=dtype,
            count=a["count"] * width,
            offset=view.get("byteOffset", 0) + a.get("byteOffset", 0),
        ).reshape(-1, width)

    def write(array, target, bounds=False):
        array = np.ascontiguousarray(array)
        binary.extend(b"\0" * (-len(binary) % 4))
        view = {
            "buffer": 0,
            "byteOffset": len(binary),
            "byteLength": array.nbytes,
            "target": target,
        }
        binary.extend(array.tobytes())
        accessor = {
            "bufferView": len(output["bufferViews"]),
            "componentType": 5125 if array.dtype == np.dtype("<u4") else 5126,
            "count": len(array),
            "type": {1: "SCALAR", 2: "VEC2", 3: "VEC3"}[array.shape[1]],
        }
        if bounds:
            accessor.update(min=array.min(0).tolist(), max=array.max(0).tolist())
        output["bufferViews"].append(view)
        output["accessors"].append(accessor)
        return len(output["accessors"]) - 1

    triangles = 0
    for mesh_i, mesh in enumerate(cad["meshes"]):
        groups = {}
        for primitive in mesh["primitives"]:
            assert primitive.get("mode", 4) == 4
            assert set(primitive) <= {"attributes", "indices", "material", "mode"}
            attrs = tuple(sorted(primitive["attributes"]))
            assert attrs == ("NORMAL", "POSITION", "TEXCOORD_0")
            groups.setdefault(
                (primitive.get("material"), primitive.get("mode", 4), attrs), []
            ).append(primitive)
        primitives = []
        for (material, mode, attrs), items in groups.items():
            records, faces, offset = [], [], 0
            widths = [read(items[0]["attributes"][a]).shape[1] for a in attrs]
            for item in items:
                record = np.concatenate(
                    [read(item["attributes"][a]) for a in attrs], axis=1
                )
                indices = read(item["indices"]).ravel()
                assert len(indices) % 3 == 0 and indices.max() < len(record)
                records.append(record)
                faces.append(indices.astype(np.uint32) + offset)
                offset += len(record)
            records = np.ascontiguousarray(np.concatenate(records))
            faces = np.concatenate(faces)
            # Exact bytes, including normals and UV seams; no rounding or changed surface data.
            keys = records.view(
                np.dtype((np.void, records.shape[1] * records.dtype.itemsize))
            ).ravel()
            _, selected, inverse = np.unique(
                keys, return_index=True, return_inverse=True
            )
            shared = records[selected]
            remapped = inverse[faces].astype("<u4")
            assert np.array_equal(shared[inverse], records), "Vertex data changed"
            assert np.array_equal(
                shared[remapped], records[faces]
            ), "Triangle data changed"
            attributes, start = {}, 0
            for name, width in zip(attrs, widths):
                attributes[name] = write(
                    shared[:, start : start + width], 34962, name == "POSITION"
                )
                start += width
            primitive = {
                "attributes": attributes,
                "indices": write(remapped.reshape(-1, 1), 34963),
                "mode": mode,
            }
            if material is not None:
                primitive["material"] = material
            primitives.append(primitive)
            triangles += len(faces) // 3
        output["meshes"][mesh_i]["primitives"] = primitives
    output["buffers"] = [{"byteLength": len(binary)}]
    assert output["nodes"] == cad["nodes"]
    assert output["materials"] == cad["materials"]
    assert output["scenes"] == cad["scenes"]
    out = (
        Path(__file__).resolve().parents[2]
        / "visualization/advantagescope/Robot_SIMCITY_3464_2026"
    )
    out.mkdir(parents=True, exist_ok=True)
    # Apply only a whole-robot frame change in AdvantageScope; never move individual parts.
    # Shooter side (CAD -Y) is robot forward; intake (CAD +Y) is rearward.
    # Center and wheel floor are from the supplied chassis geometry.
    config = {
        "name": "SIMCITY-3464-2026",
        "disableSimplification": True,
        "rotations": [{"axis": "z", "degrees": 90}],
        "position": [
            0.000060558319091796875,
            -0.00013592839241027832,
            -0.011112872914083017,
        ],
        "cameras": [],
        "components": [],
    }
    # Explicit membership for this exact export, not a bounding-box part filter.
    # Aimer panels and curved gears move about the front shooter shaft. Side plates,
    # drive motors, flywheel shafts, and belts stay with the turret. The square frame,
    # base plate, support ring, and bearing stacks remain on the chassis.
    assert hashlib.sha256(source).hexdigest() == (
        "54a41c2c7f18c04bf9f78e3d0ab330776fd03462db19d6342286a2cdbe83d498"
    ), "New CAD export: re-identify joints and moving parts before building"
    hood = {242, 246, 262, 289}
    turret = {201} | {i for i in range(225, 324) if "mesh" in cad["nodes"][i]}
    turret -= hood
    all_meshes = {i for i, n in enumerate(cad["nodes"]) if "mesh" in n}
    chassis = all_meshes - turret - hood
    assert not (turret & hood) and chassis | turret | hood == all_meshes

    def matrix(i):
        # These occurrence matrices are already in assembly/world coordinates.
        return np.array(cad["nodes"][i]["matrix"]).reshape(4, 4, order="F")

    turret_cad = matrix(200)[:3, 3]  # Turntable gear center, vertical axis
    hood_cad = matrix(241)[:3, 3].copy()  # Aimer panel origin on front shaft
    shaft_axis = matrix(251)[:3, 2]
    hood_cad += shaft_axis * ((turret_cad[0] - hood_cad[0]) / shaft_axis[0])
    yaw = float(np.arctan2(matrix(229)[1, 0], matrix(229)[0, 0]))
    forward = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    offset = np.array(config["position"])
    turret_robot = forward @ turret_cad + offset
    hood_robot = forward @ hood_cad + offset
    unyaw = np.array(
        [
            [np.cos(yaw), np.sin(yaw), 0.0],
            [-np.sin(yaw), np.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ]
    )
    config["components"] = [
        {
            "zeroedRotations": [{"axis": "z", "degrees": 90}],
            "zeroedPosition": (-forward @ turret_cad).tolist(),
        },
        {
            "zeroedRotations": [{"axis": "z", "degrees": 90 - np.degrees(yaw)}],
            "zeroedPosition": (-unyaw @ forward @ hood_cad).tolist(),
        },
    ]
    asset_sizes = {}
    for filename, members in [
        ("model.glb", chassis),
        ("model_0.glb", turret),
        ("model_1.glb", hood),
    ]:
        asset_sizes[filename] = write_component(out / filename, output, binary, members)
    (out / "config.json").write_text(json.dumps(config, indent=2) + "\n")

    # Keep display-only pivots in sync with component zeroing. These CAD values must
    # not silently replace the hardware/aiming calibration in ShooterConstants.
    def vector(v):
        return ", ".join(f"{x:.12f}" for x in v)

    java = (
        Path(__file__).resolve().parents[2]
        / "visualization/src/main/java/frc/robot/visualization/RobotCadModelGeometry.java"
    )
    java.parent.mkdir(parents=True, exist_ok=True)
    java.write_text(
        "package frc.robot.visualization;\n\n"
        "import edu.wpi.first.math.geometry.Translation3d;\n\n"
        "/** Display-only CAD pivots. Generated by tools/robot-model/build_simcity_model.py. */\n"
        "public final class RobotCadModelGeometry {\n"
        "  private RobotCadModelGeometry() {}\n\n"
        "  public static final Translation3d TURRET_PIVOT =\n"
        f"      new Translation3d({vector(turret_robot)});\n"
        "  public static final Translation3d HOOD_OFFSET =\n"
        f"      new Translation3d({vector(hood_robot - turret_robot)});\n"
        f"  public static final double CAD_TURRET_YAW_RAD = {yaw:.12f};\n"
        "}\n"
    )
    metadata = {
        "source": args.source.name,
        "sha256": hashlib.sha256(source).hexdigest(),
        "mode": "articulated CAD; encoder-zero calibration pending",
        "robotForward": "shooter side (CAD -Y); intake is rearward",
        "nodes": len(cad["nodes"]),
        "meshInstances": sum("mesh" in n for n in cad["nodes"]),
        "uniqueMeshes": len(cad["meshes"]),
        "uniqueMeshTriangles": triangles,
        "materials": len(cad["materials"]),
        "geometrySimplified": False,
        "oldIntakeAdded": False,
        "assetBytes": asset_sizes,
        "componentMeshInstances": {
            "chassis": len(chassis),
            "turret": len(turret),
            "hood": len(hood),
        },
        "hoodMeshNodes": sorted(hood),
        "turretPivotRobot": turret_robot.tolist(),
        "hoodPivotRobot": hood_robot.tolist(),
        "cadTurretYawRad": yaw,
    }
    (out / "geometry.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()
