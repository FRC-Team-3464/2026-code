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
   individual children. Array order is turret, hood, then the schematic intake.
5. Run an auto and compare `Hood/PositionRad` with the hood movement. The animation
   uses measured feedback, not the target. Without Component data, the model keeps
   the original assembled CAD pose.

Automatic AdvantageScope simplification is disabled for this model. Its large
full-resolution mesh retains the original CAD detail. `model.glb` contains the
stationary chassis, `model_0.glb` the rotating turret, and `model_1.glb` the hood.
All 409 mesh instances appear exactly once across these files.
The original CAD has no current intake assembly. `model_2.glb` adds an orange,
simplified sliding intake for SIM timing checks; it is not measured CAD.
No robot control constants are changed.

Install the builder dependency in a Python virtual environment, then rebuild:

```sh
python -m pip install -r tools/robot-model/requirements.txt
python tools/robot-model/build_simcity_model.py '/path/to/2026-ROBOT.gltf'
```

The builder checks that batched triangles retain exactly the same vertex data,
including normals and UVs, and that nodes, materials, and scenes are unchanged.
The source hash and asset counts are recorded in `geometry.json`.
The generated GLBs are kept locally and ignored by Git. Regenerate the CAD assets from
the original export on another computer. The builder also generates the schematic intake.

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

REAL and SIM use the shared bounds in `ShooterConstants`: turret −80° to +80°,
and hood −223.5° to 0° in the code's encoder coordinates. The hood range is not a
verified physical tilt range. AdvantageScope has no separate limits: it displays
measured feedback, including any motion beyond a software bound. No CAD rebuild
is needed for a control-limit change; rebuild and restart the robot code.

This animation does not validate physical travel, collisions, or shot accuracy.
It changes no motor commands, control limits, or autonomous behavior.

Component setup follows the [AdvantageScope articulated-model format](https://docs.advantagescope.org/more-features/custom-assets/#articulated-components).

### Intake timing visualization

If the three CAD GLBs are already installed, generate just the new intake asset
from the repository root (no extra Python packages needed):

```sh
python3 tools/robot-model/build_intake_model.py
```

Rebuild and restart SIM, then reload the AdvantageScope custom assets. Use the
entire `/RealOutputs/Mechanism3d/Robot/Components` array as described above. The
orange intake extends toward the rear of the robot while deployment is commanded
and retracts when retraction is commanded. Rollers alone do not move it.

The display assumes **30 cm of travel** and **2 seconds for full travel** at the
normal intake deployment duty cycle. These are provisional timing aids, not
measurements. A one-second command therefore moves roughly half the travel.
The model starts retracted, clamps at either end, and holds its position when the
command stops or SIM is disabled. Disabling also clears pending motor requests.
Position-control requests are logged but do not animate this open-loop model.

Graph `Intake/SimExtensionFraction` (0 = retracted, 1 = extended) alongside the
intake command outputs to check the auto timing. `Intake/SimExtensionAvailable`
identifies simulated travel. No physical position feedback is invented: the REAL
intake remains at the retracted display pose because its IO has no travel sensor.
This model does not simulate rollers, fuel movement, jams, collisions, or loads.

The display dimensions are generated by `build_intake_model.py`; the assumed
travel time is `FULL_TRAVEL_SECONDS` in `IntakeIOSim.java`. These values do not
change hardware controls or the autonomous command durations.


### Teleop passing

RB aims at the hub inside the home alliance zone, or at the nearer home-side
passing point outside it. This applies to the driver in single-controller mode
and the operator in dual-controller mode. The selection updates while RB is held,
including when crossing the zone boundary or the field centerline. Manual hood
and turret overrides retain their existing behavior. RT still feeds manually;
it does not wait for shooter readiness. Turn the robot toward the target so the
turret can reach it within its existing limits before feeding.

Autonomous routines continue using the hub-only target and original shooting
table. Unknown alliance or nonfinite pose stops teleop automatic turret/flywheel
tracking; valid data allows it to resume. Manual feeding is independent.

Passing settings are in `TrajectoryCalculator.passingTable`:

| Turret-to-target distance (m) | Flywheel RPM | Hood encoder angle (rad) | Estimated flight time (s) |
| --- | --- | --- | --- |
| 2 | 1175 | -0.5 | 0.81 |
| 3 | 1475 | -0.5 | 0.97 |
| 4 | 1725 | -0.5 | 1.11 |
| 6 | 2125 | -0.5 | 1.35 |
| 8 | 2475 | -0.5 | 1.55 |
| 10 | 2775 | -0.5 | 1.73 |

These are model-derived starting estimates, not measured passing results. The
[FIRST 2026 game manual](https://firstfrc.blob.core.windows.net/frc2026/Manual/HTML/2026GameManual.htm)
provides the nominal field size (16.54 × 8.07 m), hub opening height (72 in / 1.8288 m)
and FUEL diameter (5.91 in / 0.150114 m). Field coordinates and alliance mirroring
use the existing official AprilTag layout. The two passing points remain team-chosen
landing locations, not FIRST-defined scoring targets.

The calculation uses the successful 3 m / 1900 RPM hub shot as its reference.
It assumes that distance is measured from the shooter to the hub center, a
0.40 m ball-center release height, and a 55-degree physical launch angle at the
-0.5 rad hood encoder target. Those robot measurements are unconfirmed; FIRST's
field drawings cannot supply them. The 2.6 m shot uses a different hood target,
so it is not treated as a second measurement at the same launch angle.

For an [ideal ballistic trajectory](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/ballistic-flight-equations/),
with distance `d`, launch angle `theta`, and ball-center target height `h`:

```text
v(d, h) = sqrt(9.81 * d^2 / (2 * cos(theta)^2 * (d * tan(theta) - (h - 0.40))))
vReference = v(3.0, 1.8288 + 0.075057)
passingRPM(d) = 1900 * v(d, 0.075057) / vReference
flightTime(d) = d / (v(d, 0.075057) * cos(theta))
theta = 55 degrees
```

The reference models the ball center one radius above the hub opening; a pass ends
when the ball first touches the carpet. RPM is rounded to the nearest 25 and flight
time to 0.01 s. The ratio assumes ball exit speed is proportional to commanded RPM
with unchanged wheel contact. It does not model drag, compression, spin, bounce,
field-obstacle clearance or other robots. A successful hub shot alone does not
establish the launch angle or that conversion across all RPMs.

The fixed -0.5 rad hood target matches the applied encoder target for the 3 m hub
shot. It is used directly without the hub table's 0.5 multiplier. It does not
command a measured 55-degree launch angle. Neither the hub table nor autonomous
shooting settings are changed by these estimates.

Flight time is logged for tuning; stationary passing does not use it to compensate
for robot motion. Test while stationary, measure the landing distance, and adjust
the passing entries. Start with short passes before testing longer distances.
Values interpolate between entries and clamp beyond 2–10 m; a clamped value does
not mean a pass beyond that range will reach its destination.

In AdvantageScope, add `/RealOutputs/TeleopShooterTarget/Position` alongside the
robot pose. `TeleopShooterTarget/Type` reports HUB, PASS, or UNAVAILABLE; an
unavailable target has an empty pose array. These fields show the teleop selection
even during autonomous, when it is not used. The existing TargetField still shows
the hub. `Passing/RequestedRPM`, `HoodAngleRad`, `EstimatedFlightTimeSeconds`,
`DistanceMeters`, `DistanceClampedMeters`, and `InTableRange` update during passing
commands. They retain their last values afterward. The existing flywheel and hood
logs show actual mechanism commands and feedback.

In SIM, select blue and then red alliance stations and drive across the home-zone
boundary and between the upper and lower halves of the field with RB held. The
boundary is `FieldConstants.LinesVertical.allianceZone`. The passing destinations
are (2 m, 1 m) and (2 m, field width minus 1 m), mirrored for red. Confirm the turret,
hood and flywheel commands, then run the existing autos to confirm hub tracking.
SIM verifies selection and mechanism commands; it does not validate ball flight.
