# 3464 robot models

## SIMCITY-3464-2026 (full-resolution CAD reference)

This static model preserves all 409 mesh instances from `2026-ROBOT.gltf`,
including the original materials, normals, UVs, triangles, and assembly transforms.
[View the full-resolution preview](Robot_SIMCITY_3464_2026/preview.png). The export
assigns pure black to the large panels; this differs from their gray appearance
in the Onshape screenshot. Original exported materials are retained.

No parts are removed or simplified. The only placement change is one whole-robot
coordinate conversion: +X toward the shooter (front), intake toward the rear,
+Y left, and wheel contact at Z = 0.

1. In AdvantageScope, choose **Use Custom Assets Folder** and select this
   `visualization/advantagescope` directory.
2. Add `/RealOutputs/RobotState/EstimatedPose` as a **Robot** and choose
   **SIMCITY-3464-2026**.
3. Remove any **Component** data beneath this robot. This reference model is static;
   the earlier `SIMCITYComponents` animation and inferred pivots have been removed.

Automatic AdvantageScope simplification is disabled for this model. Its large
full-resolution mesh is intended for checking the CAD appearance first. Compare
it against the supplied CAD before creating articulated or reduced-detail models.
The old intake has not been added; compatibility with the new assembly is unconfirmed.
No robot control constants are changed.

Install the builder dependency in a Python virtual environment, then rebuild:

```sh
python -m pip install -r tools/robot-model/requirements.txt
python tools/robot-model/build_simcity_model.py '/path/to/2026-ROBOT.gltf'
```

The builder checks that batched triangles retain exactly the same vertex data,
including normals and UVs, and that nodes, materials, and scenes are unchanged.
The source hash and asset counts are recorded in `geometry.json`.
The generated `model.glb` exceeds GitHub's regular file-size limit, so it is kept
locally and ignored by Git. Regenerate it from the original export on another computer.
