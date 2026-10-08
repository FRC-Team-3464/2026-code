# 3464 robot model (CAD draft)

This simplified model retains the left shooter in the supplied rear-view photo,
removes the crossed-out right shooter, and turns the retained shooter toward the
robot's rear. +X points forward toward the intake, +Y left, +Z up. The moving hood
is orange so it is easy to identify. Small fasteners, electronics, the obsolete
climber and loose CAD parts are omitted. This is a display asset, not a physics model.

## Open in AdvantageScope

1. Choose **AdvantageScope → Use Custom Assets Folder** and select this
   `visualization/advantagescope` directory (the parent of `Robot_3464_2026`).
2. In a **3D Field** tab, add `/RealOutputs/RobotState/EstimatedPose` as a **Robot**
   and select **3464 2026 — CAD draft** as its model.
3. The model appears in its rear-facing reference pose without component data.
4. Run SIM from this branch. Add `/RealOutputs/Mechanism3d/Robot/Components` to
   that robot as **Component** data. The array order is turret, then hood.

`model.glb` is the fixed body; `model_0.glb` is the turret; `model_1.glb` is the hood.
The asset is placed using the shared code geometry, including its hood offset.
The component configuration removes that reference placement before applying
those logged poses. At zero turret angle the existing visualizer adds 180° yaw,
which makes this model face rearward. This is a display convention, not a verified
encoder calibration.

## What remains unconfirmed

`RobotVisualizer` uses `TurretConstants.kRobotToTurret` and
`HoodConstants.kTurretToHood` directly. The builder places the CAD pieces at those
same pivots. No separate display offsets or aiming changes are used.

The hood offset is now a direct CAD estimate: approximately 8.9 cm along the turret's
shooting direction and 5.3 cm upward (with a sub-millimetre sideways offset). It is
used only for visualization; the turret's robot-relative aiming position is unchanged.
Confirm the actual pivots and encoder-zero directions on the robot before treating
this model as calibrated. Agreement between the model and code is not independent
physical validation. Other old CAD details may also differ from the current robot.

## Rebuild

Source: the team's old Onshape assembly, document `85475e925eb3dcc4b391e6b2`,
workspace `2ff6485baafc6bd31b6c9523`, element `92778bb882b703fb96455458`.
The supplied `Assembly 1.gltf` contains 5,210 nodes with embedded geometry. Keep the
original export separately; the 548 MB source is intentionally not in this repository.

First build the robot jar, then export its actual shared constants (no robot is started):

```sh
./gradlew jar --offline
java -cp build/libs/2026-code.jar tools/robot-model/ExportGeometry.java > tools/robot-model/geometry.json
```

Use the WPILib JDK for these commands. Regenerate `geometry.json` whenever the
shared geometry changes; it is a generated snapshot, not a source of calibration.

In a Python virtual environment, install `tools/robot-model/requirements.txt`, then
run `python tools/robot-model/build_model.py '/path/to/Assembly 1.gltf'` from the
repository root. The script uses explicit part IDs from this export. Re-identify
parts before using a newer export. The checked-in GLBs need no Python installation.

Asset format: https://docs.advantagescope.org/more-features/custom-assets/
