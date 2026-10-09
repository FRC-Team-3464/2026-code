# 3464 robot models

## SIMCITY-3464-2026 (articulated CAD model)

This model preserves all 409 mesh instances from `2026-ROBOT.gltf`,
including the original materials, normals, UVs, triangles, and assembly transforms.
[View the full-resolution preview](Robot_SIMCITY_3464_2026/preview.png). The export
assigns pure black to the large panels; this differs from their gray appearance
in the Onshape screenshot. Original exported materials are retained.

No parts are removed or simplified. The default assembled pose uses one whole-robot
coordinate conversion: +X toward the shooter (front), intake toward the rear,
+Y left, and wheel contact at Z = 0.

1. In AdvantageScope, choose **Use Custom Assets Folder** and select this
   `visualization/advantagescope` directory.
2. Add `/RealOutputs/RobotState/EstimatedPose` as a **Robot** and choose
   **SIMCITY-3464-2026**.
3. Restart the robot simulation after rebuilding the Java code. Reload the custom assets
   (restart AdvantageScope if it still shows the previous static model).
4. Drag `/RealOutputs/Mechanism3d/Robot/Components` onto the robot entry in the
   3D Field poses list and select **Component**. Use the entire `Pose3d[]`, not its
   individual children. Array order is turret, then hood.
5. Run an auto and compare `Hood/PositionRad` with the hood movement. The animation
   uses measured feedback, not the target. Without Component data, the model keeps
   the original assembled CAD pose.

Automatic AdvantageScope simplification is disabled for this model. Its large
full-resolution mesh retains the original CAD detail. `model.glb` contains the
stationary chassis, `model_0.glb` the rotating turret, and `model_1.glb` the hood.
All 409 mesh instances appear exactly once across these files.
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
The generated GLBs are kept locally and ignored by Git. Regenerate all three from
the original export on another computer.

### Animation calibration

The turntable gear center and front shooter shaft define the visual pivots. The
hood group contains the two aimer panels and both exported curved gears. Motors,
side plates, shafts, and belts remain with the turret; the supporting frame and
bearing stacks remain stationary. The small CAD yaw offset is retained.

The builder generates
`visualization/src/main/java/frc/robot/visualization/RobotCadModelGeometry.java` from the same pivots used to zero
the model components. These are display-only constants, separate from aiming geometry. Gradle compiles this
visualization source directory because `RobotVisualizer` uses these values at runtime;
the CAD files themselves are not included in the robot deployment.
Turret display yaw negates `Turret/PositionRad` to match the existing tracking mapping.
Hood display pitch negates the reported angle so negative feedback raises the hood
about its front pivot. A zero reading restores the exported CAD pose.
**Hood zero, direction, and scale are provisional.** No travel limits or gear ratios
are invented to make the display look plausible; large reported angles can therefore
produce unrealistic motion. Photos of the physical hood, gear tooth counts, and paired
encoder readings at known positions are needed to calibrate that mapping.

This animation does not validate physical travel, collisions, or shot accuracy.
It changes no motor commands, control limits, or autonomous behavior.

Component setup follows the [AdvantageScope articulated-model format](https://docs.advantagescope.org/more-features/custom-assets/#articulated-components).
