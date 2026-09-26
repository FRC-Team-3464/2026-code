# Architecture review: preparing the robot software for 2027

The project has a sound FRC foundation: WPILib commands and subsystems, replaceable IO adapters, WPILib geometry and estimation, and AdvantageKit logging. Keep it and repair the specific timing, ownership, and data-flow problems below. This is a source review of the 2026 code, not a physical-robot validation. [Mentor Recommendations](REUSE_RECOMMENDATIONS_2027.md) give the acceptance checks.

## What “following best practices” means here

There are three different sources of guidance:

| Basis | What it establishes | Example |
| --- | --- | --- |
| WPILib framework behavior and guidance | How its scheduler, subsystems, commands, and estimation APIs work | Commands declare the subsystems they control |
| AdvantageKit guidance | How to structure inputs and hardware access for this project's logging/replay approach | Read and log an IO input snapshot before using it |
| Engineering judgment for this team | Choices about maintainability, scope, teaching, and verification | Prefer a plain shooter coordinator; enforce one Java naming style |

WPILib presents a standard command-based structure while explicitly allowing other arrangements. Its template is a reference, not a requirement to reproduce every class name or helper. AdvantageKit is a separate FRC library with additional architectural guidance. A recommendation below should not be presented as an FRC competition rule or an endorsement by either project. [WPILib project structure](https://docs.wpilib.org/en/stable/docs/software/commandbased/structuring-command-based-project.html), [AdvantageKit overview](https://docs.advantagekit.org/).

The general robotics assessment uses concrete properties: predictable timing, clear hardware ownership, consistent units and coordinate frames, usable feedback, deliberate fault behavior, and observable results. It is an engineering assessment of this program, not a comparison against a universal robotics certification standard.

### Recognize the upstream starting point

Several choices discussed in the recommendations come directly from AdvantageKit's matching release. They should be evaluated with that context:

| Upstream example in `v26.0.1` | What it means for this review |
| --- | --- |
| Formatting during compilation, recursive file targets with build-directory exclusions, and automatic commits on `event` branches | These are template workflow choices. This branch now uses explicit formatting and narrower targets; the event commit task remains. The changes reflect team workflow preferences, not an FRC compliance correction. [Template build configuration](https://github.com/Mechanical-Advantage/AdvantageKit/blob/v26.0.1/template_projects/template/build.gradle) |
| Shared REV read-fault flag and bounded CTRE retries | The local helpers match the template. Existing sequential readers reset and consume the flag per refresh. More explicit status handling can improve diagnostics, but a cross-device fault has not been demonstrated. [Spark helper](https://github.com/Mechanical-Advantage/AdvantageKit/blob/v26.0.1/template_projects/sources/spark_swerve/src/main/java/frc/robot/util/SparkUtil.java), [Phoenix helper](https://github.com/Mechanical-Advantage/AdvantageKit/blob/v26.0.1/template_projects/sources/talonfx_swerve/src/main/java/frc/robot/util/PhoenixUtil.java) |
| Limelight parsing assumptions and consumption of both MegaTag streams | The local adapter closely follows the template. It now rejects malformed pose payloads and logs rejection reasons; a stream-correlation policy remains open. The separate uncertainty-handoff defect in `RobotState` has been repaired and checked on desktop; live camera validation remains open. [Limelight adapter](https://github.com/Mechanical-Advantage/AdvantageKit/blob/v26.0.1/template_projects/sources/vision/src/main/java/frc/robot/subsystems/vision/VisionIOLimelight.java) |

An upstream example is a useful starting point, not proof of suitability for every robot. Conversely, choosing a different implementation does not establish that the template or the students' use of it was wrong. Each proposed change needs a concrete benefit and a check that demonstrates it.

## Assessment by architectural area

“Aligned” means the approach is appropriate. It does not mean that every line has been tested. “Needs correction” identifies a specific implementation problem. The recommendation IDs link to the existing acceptance procedures.

| Area | Assessment | Local evidence and next step |
| --- | --- | --- |
| Application structure | Aligned overall | `Robot` handles lifecycle and scheduling; `RobotContainer` constructs mechanisms and configures bindings. The 50 Hz odometry submission now belongs to `Drive`; higher-rate integration remains open. [H2](REUSE_RECOMMENDATIONS_2027.md#acceptance-h2) |
| Commands and composition | Aligned approach; manual hood ownership repaired | Command factories, `sequence`, `parallel`, and `alongWith` are appropriate. Manual hood commands now require the hood; physical acceptance remains open. [H4](REUSE_RECOMMENDATIONS_2027.md#acceptance-h4) |
| Subsystem lifecycle | Fix ready for review | Duplicate shooter-child callbacks were removed; [H1 SIM evidence](IMPLEMENTATION_TRACKER_2027.md#p31--h1-one-shooter-child-update-per-robot-cycle) awaits mentor review. |
| Hardware abstraction | Strong foundation; contracts need correction | `FlywheelIO`, `ModuleIO`, and camera interfaces isolate hardware. Units and stop semantics differ between some adapters. [M3](REUSE_RECOMMENDATIONS_2027.md#acceptance-m3), [H8](REUSE_RECOMMENDATIONS_2027.md#acceptance-h8) |
| Pose estimation | Appropriate library; incomplete integration | Uses `SwerveDrivePoseEstimator`; the 50 Hz update ordering and per-measurement vision uncertainty handoff are ready for review. High-rate odometry and remaining vision validation stay open. [H2](REUSE_RECOMMENDATIONS_2027.md#acceptance-h2), [H7](REUSE_RECOMMENDATIONS_2027.md#acceptance-h7) |
| Coordinate frames and units | Partially aligned | Uses `Pose2d`, `Rotation2d`, and `ChassisSpeeds`; heading resets and RPM/RPS boundaries need repair. [H3](REUSE_RECOMMENDATIONS_2027.md#acceptance-h3), [H6](REUSE_RECOMMENDATIONS_2027.md#acceptance-h6) |
| Shared state and dependencies | Useful intent; responsibilities need separation | `RobotState` holds an estimator, mutable velocity, and season target selection. [M2](REUSE_RECOMMENDATIONS_2027.md#acceptance-m2), [H9](REUSE_RECOMMENDATIONS_2027.md#acceptance-h9) |
| Mechanism coordination | Needs correction | Shooter parts use different calculation paths; readiness can describe an old request. [H5](REUSE_RECOMMENDATIONS_2027.md#acceptance-h5), [H6](REUSE_RECOMMENDATIONS_2027.md#acceptance-h6) |
| Simulation and replay | Wiring separated; models incomplete | REAL and SIM select IO through separate wiring classes. REPLAY now fails clearly because replay-safe wiring is deferred. [H8](REUSE_RECOMMENDATIONS_2027.md#acceptance-h8) |
| Observability | Useful instrumentation; incomplete recording | Structured inputs and revision metadata exist; real-mode `WPILOGWriter` is disabled. [M4](REUSE_RECOMMENDATIONS_2027.md#acceptance-m4) |
| Autonomous integration | Incomplete | Active auto is a command composition. PathPlanner setup and chooser integration are disabled despite retained assets. [M5](REUSE_RECOMMENDATIONS_2027.md#acceptance-m5) |
| Season reuse | Needs separation | Field targets, calibrations, device configuration, and historical assets remain tied to 2026. [H9](REUSE_RECOMMENDATIONS_2027.md#acceptance-h9) |

## What the team should preserve

### A small Robot class and explicit construction

[Robot.java](../src/main/java/frc/robot/Robot.java) starts logging, creates the container, runs the scheduler, schedules autonomous, and cancels autonomous on entry to teleop. [RobotContainer.java](../src/main/java/frc/robot/RobotContainer.java) selects hardware implementations and installs bindings. This largely follows the responsibilities in the WPILib template. Keep mechanism-specific behavior outside `Robot`. [WPILib project structure](https://docs.wpilib.org/en/stable/docs/software/commandbased/structuring-command-based-project.html).

The control classes are a reasonable way to keep construction readable. Passing subsystem objects into their constructors makes dependencies visible. This is **dependency injection**: giving an object the collaborators it needs. No additional dependency-injection library is needed.

### IO interfaces and replaceable implementations

[Flywheel.java](../src/main/java/frc/robot/subsystems/shooter/flywheel/Flywheel.java) takes a `FlywheelIO`, updates an input object, logs it, and uses that snapshot. Real and simulated adapters implement the interface. This matches AdvantageKit's recommended separation between control logic and hardware access. Its input payloads intentionally use public mutable fields and generated logging support. [AdvantageKit IO interfaces](https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces/).

In design-pattern terms, a real IO implementation acts as an **adapter** between the team's API and the motor vendor's API. Selecting a real or simulated implementation supplies interchangeable behavior through the same interface. The practical benefit is that students can change a device adapter without rewriting controller bindings or shot logic.

Preserve that boundary. Improve method units and behavior before adding more interfaces. An interface is useful only when its implementations honor the same contract.

### Command factories and compositions

Methods returning `Command`, and the existing sequential and parallel compositions, fit WPILib's model. WPILib explicitly supports combining commands this way. A separate Java class for every button action is unnecessary. [WPILib command compositions](https://docs.wpilib.org/en/stable/docs/software/commandbased/command-compositions.html).

The improvement is to make the factories clear and complete: name the intended action, declare the controlled resources, and define termination behavior. For example, the current `trackAndShootAtTargetFullRealCommandLatestGoodUseThisOne()` coordinates aiming and spin-up but does not feed a ball. A name such as `trackTargetCommand()` makes its actual responsibility teachable.

### Existing math and estimation libraries

Using WPILib geometry, swerve kinematics, and a swerve pose estimator is appropriate. Pose means the robot's position and heading. The estimator combines wheel/gyro movement with delayed vision measurements; the team should correct its inputs and configuration rather than write a replacement estimator. [WPILib pose estimators](https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html).

## Where the implementation breaks the intended design

### One owner must control each lifecycle and motor resource

WPILib runs registered subsystem `periodic()` callbacks before polling triggers and executing scheduled commands. `SubsystemBase` registers itself. [Scheduler sequence](https://docs.wpilib.org/en/stable/docs/software/commandbased/command-scheduler.html), [subsystem registration](https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html).

Previously, [Shooter.java](../src/main/java/frc/robot/subsystems/shooter/Shooter.java) called `hood.periodic()`, `turret.periodic()`, and `flywheel.periodic()` even though those children were already registered with the scheduler. The extra calls have been removed and checked in SIM; see the [H1 tracker entry](IMPLEMENTATION_TRACKER_2027.md#p31--h1-one-shooter-child-update-per-robot-cycle). The plain `Module` helper objects in `Drive` still need their owner's explicit updates.

The earlier manual hood bindings in [DriverControls.java](../src/main/java/frc/robot/control/DriverControls.java) omitted the hood requirement. They now use a command factory in `Hood` that declares ownership, allowing manual hood control without cancelling turret tracking or flywheel spin-up. The scheduler arbitrates declared resources; it cannot infer ownership by inspecting a lambda. Read-only access to a measurement does not by itself require taking control of the mechanism. [WPILib subsystem resource management](https://docs.wpilib.org/en/stable/docs/software/commandbased/subsystems.html).

**Mentor recommendation:** keep the hood, turret, and flywheel as separately owned subsystems and make the shooter coordinator compose their commands. Each action must reserve the resources it actually controls. Do not assume that requiring the coordinator automatically reserves its children. Verify release, interruption, mode changes, and deliberate hold/stop behavior through H4.

### Estimation needs a coherent measurement path

The current call order is:

```text
Robot.robotPeriodic()
  CommandScheduler.run()
    Drive.periodic() refreshes drive measurements and updates RobotState
    triggers and commands run
  RobotContainer.updateDashboard() publishes the resulting pose
  FullSubsystem applies staged outputs
```

This 50 Hz ordering repair ensures commands can consume the pose updated from the current loop's ordinary drive inputs. The higher-frequency queue path in [Drive.java](../src/main/java/frc/robot/subsystems/drive/Drive.java) remains incomplete and is not part of this accepted subset.

**Remaining mentor recommendation:** preserve original timestamps when completing high-frequency odometry, update measured chassis velocity only with reviewed shooting behavior, and avoid relying on incidental registration order between unrelated subsystems. If vision/drive ordering needs coordination, define how timestamped observations are queued and incorporated.

[RobotState.java](../src/main/java/frc/robot/RobotState.java) now passes the `stdDevs` field of each vision measurement to the estimator. Desktop checks confirmed its effect on pose weighting and heading suppression; physical camera accuracy remains unverified. [WPILib PoseEstimator API](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/estimator/PoseEstimator.html).

H2, H3, and H7 provide the detailed checks. Source changes alone cannot establish physical localization accuracy.

### Custom output staging is a choice that needs a clear contract

[FullSubsystem.java](../src/main/java/frc/robot/util/FullSubsystem.java) adds a callback after the scheduler. The current shooter mechanisms use three different timings:

| Mechanism request | When it reaches IO |
| --- | --- |
| Flywheel velocity | `Flywheel.setVelocity()` calls IO immediately |
| Hood angle | `Hood.setAngle()` stores a goal; `Hood.periodic()` sends it to IO. A goal selected during command execution reaches IO on the next periodic update. Hood open-loop requests call IO directly. |
| Turret angle/output | Setters stage an output request; the post-scheduler callback applies it |

The extra stage is a project extension. Its existence is not itself a WPILib violation. It can make the sequence “read inputs, choose goals, apply outputs” explicit, provided registration, disabled behavior, and cancellation all have clear owners. The current mixture raises the amount a student must remember when tracing a command.

**Mentor recommendation:** start with the simplest output policy that meets the mechanisms' needs. If staging is retained, document exactly when output requests take effect and who clears or replaces them. Give the participants an explicit lifecycle. Do not add a second scheduler or background command loop. Choose this policy before refactoring all mechanisms around it.

### Simulation and replay must preserve the control contract

[FlywheelIOSim.java](../src/main/java/frc/robot/subsystems/shooter/flywheel/FlywheelIOSim.java) now compares RPS targets with RPS feedback and selects velocity, open-loop, or stopped output explicitly. Desktop checks cover these repairs; gearing and physical response still need verification.

WPILib simulation models advance from applied inputs to simulated sensor readings. An IO-based project can place that work inside its simulated adapter; moving everything into `simulationPeriodic()` is not required for this architecture. The essential review questions here are units, time step, and behavior at the interface. [WPILib physics simulation](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html), [AdvantageKit IO interfaces](https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces/).

REPLAY requires complete object construction as well as a saved input stream. The current code
rejects REPLAY before logger or subsystem construction because replay-safe wiring is not available.
AdvantageKit also requires deterministic, synchronized inputs and recommends checking replay
regularly before the team declares that mode supported. [AdvantageKit replay inputs](https://docs.advantagekit.org/getting-started/common-issues/non-deterministic-data-sources/).

**Mentor recommendation:** retain modest, trustworthy simulation coverage. Complete replay only if the team will use it; otherwise make its unsupported status explicit. Keep a single main-thread behavior pipeline. Any necessary background sensor sampling should hand synchronized input data to that pipeline. AdvantageKit documents this boundary and requires its logging calls on the main thread. [AdvantageKit multithreading](https://docs.advantagekit.org/getting-started/common-issues/multithreading/).

### State, targeting, and physical limits need distinct responsibilities

`RobotState` estimates location and chooses a 2026 hub target. `TrajectoryCalculator` reads shared robot state, while hood, turret, and flywheel tracking take different calculation paths. The resulting dependencies make it difficult to answer which exact pose and shot calculation produced a motor request.

**Mentor recommendation:** separate three responsibilities:

1. **State estimation:** current pose, measured velocity, measurement validity, and timestamps.
2. **Season strategy and shot calculation:** choose a target and calculate one `ShotSolution` from an explicit state snapshot and calibration.
3. **Mechanism control:** apply valid goals within physical bounds and report current readiness.

The mechanism must protect its limits regardless of whether a request originated from a button, autonomous routine, or calibration command. Readiness must describe the current request and current measurements. This is a team engineering requirement supported by the observed H5/H6 problems; WPILib cannot supply the physical travel limits or shot tolerances for this robot.

Shared state is reasonable when there is an explicit owner and controlled access. The singleton pattern is not automatically wrong. Here, constructor-supplied dependencies and defensive snapshots would make responsibilities clearer, particularly because `ChassisSpeeds` is mutable and currently returned directly. See M2 and H9.

## A practical target architecture

This is a proposed responsibility map, not a new framework or a claim that these classes already exist. Arrows show data or requests; `RobotContainer` constructs and connects the components.

```mermaid
flowchart TD
    B[Controller bindings and autonomous routines] --> C[WPILib commands and coordination]
    T[Season target selection] --> Q[Shot calculation]
    E[Pose and velocity estimation] --> Q
    Q -->|One shot solution| C
    C -->|Goals with subsystem requirements| S[Subsystem logic and limits]
    S -->|Output requests| I[Real, simulated, or replay IO]
    I -->|Measurements through logged input snapshots| S
    S -->|Timestamped drive and vision observations| E
    S -->|Current readiness| C
```

For the initial 2027 foundation, prefer small classes and constructor parameters. Introduce a dedicated state machine only where mechanism transitions need one, such as unreferenced, homing, ready, and fault states. A full state-machine framework is unnecessary for a simple intake action. Retain high-frequency odometry only with synchronized samples and a measured benefit; a correct main-loop pipeline is a reasonable intermediate step.

## Which recommendations are team preferences?

Keep these distinctions explicit when explaining the plan to students:

| Recommendation | Reason and authority |
| --- | --- |
| Declare requirements on motor-controlling commands | Needed for WPILib scheduler resource arbitration |
| Remove duplicate periodic calls | Correctness of this program's lifecycle and simulated timing |
| Preserve input timestamps and pass chosen vision uncertainty through | Correct use of the estimation design |
| Rename `kLoopPeriodSeconds` to `LOOP_PERIOD_SECONDS` | Team Java convention; WPILib examples also use `k` prefixes |
| Rename `TurretIO` to `TurretIo` | Team acronym policy; either spelling can implement the same architecture |
| Rename `Drive` to `Drivetrain` | Optional clarity improvement; `Drive` is already a valid Java class name |
| Prefer a plain shooter coordinator | Design recommendation for the current child-subsystem arrangement |
| Use Spotless and Checkstyle | Team enforcement workflow; neither establishes robot correctness |
| Document camera stream correlation or change the REV fault helper | Remaining hardening or maintainability choices; camera payload validation has already been implemented |
| Change automatic formatting, target patterns, or event commits | Team workflow tradeoffs; the matching AdvantageKit template contains the original patterns |
| Add a large unit-test suite | Not required by this reuse plan; acceptance evidence is still required |

The proposed casing follows Google Java conventions. WPILib's own examples demonstrate different naming choices, so standardizing team code should be explained as a consistency decision. Preserve dependency names and narrowly identified generated sources. [Google Java naming](https://google.github.io/styleguide/javaguide.html#s5-naming), [WPILib command examples](https://docs.wpilib.org/en/stable/docs/software/commandbased/command-compositions.html).

Import rules need the same distinction. WPILib recommends `import static edu.wpi.first.units.Units.*;` for its units library. A team may choose explicit imports, or allow this one scoped exception; a general wildcard ban is not a WPILib requirement. Decide this in the checked-in Checkstyle policy. [WPILib Java units](https://docs.wpilib.org/en/stable/docs/software/basic-programming/java-units.html).

## Recommended decision for the mentor

Keep the present architecture and make focused repairs before broad renaming or a season port. Demonstrate the repaired timing, command ownership, units, frames, stop behavior, and physical response using the [acceptance checks](REUSE_RECOMMENDATIONS_2027.md). Compare the carried code with the [matching AdvantageKit templates](https://docs.advantagekit.org/getting-started/template-projects/) when selecting the 2027 toolchain.
