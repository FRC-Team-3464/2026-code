# Phase 1 record: what we know before 2027 reuse work

This separates source facts already checked from information still needed for [Phase 1](DELIVERY_PLAN_2027.md#phase-1--establish-the-baseline-and-reuse-scope). It is not physical-robot acceptance.

## Facts verified now

| Item | Recorded result | Evidence and limit |
| --- | --- | --- |
| Short SIM startup | The desktop robot reached `Robot program startup complete`. | Startup alone does not verify controller bindings or mechanism behavior. [H1 SIM results](shooter-sim-lifecycle/RESULTS_2026-09-25.md) record a separate controller-driven check. |
| Runtime mode selection | [Constants.java](../src/main/java/frc/robot/Constants.java) chooses REAL on a roboRIO and SIM on a desktop by default. REPLAY is reserved but deliberately unsupported. | SIM startup and explicit REPLAY rejection were checked. REAL still requires the physical robot. |
| REAL construction path | [RealRobotWiring.java](../src/main/java/frc/robot/wiring/RealRobotWiring.java) selects the Pigeon 2, four Talon FX swerve modules, Talon FX indexer/intake, physical LED output, Spark MAX turret/hood, Talon FX flywheel, and two named Limelights. `RobotContainer` constructs the shared subsystems in the original order. | Source inspection. The physical robot was unavailable, so this does not confirm devices are present, connected, oriented correctly, or safe to move. |
| SIM construction path | [SimRobotWiring.java](../src/main/java/frc/robot/wiring/SimRobotWiring.java) selects a `GyroIOSim` based on measured wheel travel, four simulated swerve modules, a placeholder intake adapter, an indexer adapter that captures throat and tongue motor requests, WPILib-simulated LED output, and simulated shooter adapters. `RobotContainer` uses the same shared construction path but does not construct Vision. | Source inspection plus basic SIM startup. Intake and indexer do not model their physical effects; LED simulation exposes rendered frames but cannot verify the real strip. |
| REPLAY policy | [Robot.java](../src/main/java/frc/robot/Robot.java) rejects REPLAY before logger startup or subsystem construction because replay-safe wiring is not implemented. | An explicit desktop REPLAY launch produced the declared error instead of reaching bindings with uninitialized subsystems. |
| Current autonomous entry point | `RobotContainer.getAutonomousCommand()` composes shooter flywheel/hood tracking and indexer feeding after flywheel readiness. Its PathPlanner setup/chooser are not active. | Source inspection. No autonomous or stored route was executed in this record. |
| Configuration locations | Device IDs are in [Constants.java](../src/main/java/frc/robot/Constants.java) and generated swerve values in [DriveConstants.java](../src/main/java/frc/robot/subsystems/drive/DriveConstants.java); mechanism values are in [ShooterConstants.java](../src/main/java/frc/robot/subsystems/shooter/ShooterConstants.java) and related subsystem constants. Camera configuration is split between [VisionConstants.java](../src/main/java/frc/robot/subsystems/vision/VisionConstants.java), REAL Limelight names, and camera-side setup. Java PathPlanner config lists robot mass as `72.088` kg, while [editor settings](../src/main/deploy/pathplanner/settings.json) list `52.163` kg. | These are values *declared in files*, not measured robot specifications. Their disagreement is a configuration-reconciliation task, not permission to pick either value without evidence. |
| Logging available in source | REAL publishes via NetworkTables; the REAL `WPILOGWriter` line in `Robot.java` is commented out. | No persistent robot log was retrieved. P3.6/M4 covers reliable recording and retrieval. |

The [Architecture Review](ARCHITECTURE_REVIEW.md) and [priority overview](REUSE_RECOMMENDATIONS_2027.md#priority-overview) contain the existing source findings. They describe known code paths and suspected/confirmed implementation defects; they are not records of completed acceptance tests. Reuse those findings when assigning packages instead of making students repeat the whole code review.

## Required information or checks still open

| Needed item | Why it is needed | Current status and next owner |
| --- | --- | --- |
| Retained 2027 features, including moving shots and stored autos | Defines which H/M acceptance rows must actually pass and which features are explicitly unavailable | **Mentor decision open.** REAL and SIM are the declared runtime modes; REPLAY is explicitly deferred. The remaining feature scope still needs mentor approval. |
| Named student owners, reviewers, and hardware decision contacts | Prevents a package from being marked accepted by its implementer alone and identifies who can set physical limits | **Assignment open.** Mentor group names people at kickoff. |
| Current robot hardware revision, device type/ID/bus and wiring map | Required to compare REAL adapter configuration with the machine that will actually be tested | **Awaiting hardware.** Mechanical/electrical leads provide an as-built record when the robot is stable. Source IDs are a starting hypothesis only. |
| Encoder/gyro direction and zero, mechanism reference, travel bounds, gearing, output/current limits, and stopping margin | Required before powered movement and for H2/H3/H5/M3 physical acceptance | **Awaiting hardware and measurements.** Relevant mechanism leads and mentors set the method and safe values before testing. Do not copy simulator values as physical limits. |
| Camera mounting, camera-side configuration, field reference, timestamp/frame checks | Required for live H7 vision acceptance and camera-driven behavior | **Awaiting camera/robot access.** Vision/drive lead records the installed geometry and compares measured poses with an independent reference. |
| Physical mass, geometry, calibration, and authoritative PathPlanner model | Required to reconcile conflicting configured values and support a new robot | **Awaiting measurement/design decision.** Drive/mechanical leads select documented values for the actual robot revision. |
| Retained mechanism and autonomous measurements, including shot repeatability | Required for H6 and any physical route claim | **Awaiting hardware, safe references, and prerequisite software fixes.** Record criteria before trials; do not infer success from the 2026 source or a SIM startup. |
| Persistent robot telemetry or a representative prior log | Needed to compare before/after physical behavior and later test replay if retained | **Not available in this record.** Check whether the team already has a compatible saved log; otherwise P3.6/M4 must enable and demonstrate recording. |
| Controller-driven SIM behavior beyond H1 | Needed to verify other commands, interruption, output requests, and modeled feedback | **Partially exercised.** H1 tracking was checked in SIM; other packages need their own diagnostics. |
| Supported 2027 toolchain and target robot configuration | Required for H9/P6.4 port/release checks | **Future dependency.** Verify with the selected season toolchain and final hardware; this 2026 build cannot certify them. |

## Phase 1 disposition and smallest kickoff check

| Package | Already recorded here | Still needed to close it |
| --- | --- | --- |
| P1.1 | Existing code inventory and construction map | Confirm the mentor-approved retained/excluded scope. Full H9 extraction comes later. |
| P1.2 | REAL/SIM wiring map, unsupported-REPLAY policy, and short SIM startup observation | Name the evidence owners. REAL and broader command behavior remain unverified. |
| P1.3 | Sources of configured values and the known PathPlanner mass discrepancy | Obtain any prior log if available; assign owners of physical references and limits. Measurements can remain `Awaiting hardware` without pretending they have passed. |

**Phase 1 remains open:** confirm the retained feature scope, owners, and hardware information above. Reuse this source inventory unless construction changes.
