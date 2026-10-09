#!/usr/bin/env python3
"""Losslessly package 2026-ROBOT.gltf as the static SIMCITY-3464-2026 asset.

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
    header = json.dumps(output, separators=(",", ":")).encode()
    header += b" " * (-len(header) % 4)
    binary.extend(b"\0" * (-len(binary) % 4))
    glb = struct.pack("<III", 0x46546C67, 2, 28 + len(header) + len(binary))
    glb += struct.pack("<II", len(header), 0x4E4F534A) + header
    glb += struct.pack("<II", len(binary), 0x004E4942) + binary
    out = (
        Path(__file__).resolve().parents[2]
        / "visualization/advantagescope/Robot_SIMCITY_3464_2026"
    )
    out.mkdir(parents=True, exist_ok=True)
    (out / "model.glb").write_bytes(glb)
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
    (out / "config.json").write_text(json.dumps(config, indent=2) + "\n")
    metadata = {
        "source": args.source.name,
        "sha256": hashlib.sha256(source).hexdigest(),
        "mode": "static faithful CAD reference",
        "robotForward": "shooter side (CAD -Y); intake is rearward",
        "nodes": len(cad["nodes"]),
        "meshInstances": sum("mesh" in n for n in cad["nodes"]),
        "uniqueMeshes": len(cad["meshes"]),
        "uniqueMeshTriangles": triangles,
        "materials": len(cad["materials"]),
        "geometrySimplified": False,
        "oldIntakeAdded": False,
        "assetBytes": len(glb),
    }
    (out / "geometry.json").write_text(json.dumps(metadata, indent=2) + "\n")
    print(json.dumps(metadata, indent=2))


if __name__ == "__main__":
    main()
