# Mentor recommendations for the 2027 robot software

Original review: September 23, 2026. Revalidated: September 24, 2026.

**Source baseline:** `mentor-review` at [`477a8bf`](https://github.com/FRC-Team-3464/2026-code/tree/477a8bfc8be6f2bf33eba9ece365a21a50018a51). This revision includes the formatting/CI work merged after the original review. Current behavior and remaining work below refer to this baseline.

**Direction for preseason work:** keep the command-based structure, subsystem/IO separation, and structured logging. Correct the update timing, position estimation, and command ownership before carrying the affected code into the next robot. Adopt one Java naming standard and enforce it with **Spotless plus Checkstyle**.

Our goal is software that the next group of students can understand, operate, and maintain. A student should be able to follow a button press through a command to the motor request, explain which measurements it uses, and show what happens when the command stops. These recommendations focus on making that explanation reliable.

This is a preseason work proposal for mentor and student-lead adoption. The priorities, acceptance checks, and delivery plan describe the work expected before reuse. Each assigned task needs an owner, a reviewer, and a demonstration that its acceptance criteria have been met. Completed build configuration is identified below; acceptance of the broader recommendations remains pending unless a separate record documents completion.

A broad unit-test suite is not required for this plan. We will rely primarily on builds, automated style checks, recorded measurements, simulation, and controlled robot checks. A few small calculation checks are recommended where they would prevent difficult-to-diagnose mistakes.

The separate [Architecture Review](ARCHITECTURE_REVIEW.md) explains which parts follow WPILib and AdvantageKit guidance, which need correction, and which are team design choices. Use it when discussing the technical reasons for this plan.

## Contents

- [Expectations for the programming team](#expectations-for-the-programming-team)
- [Scope and priority definitions](#scope-and-priority-definitions)
- [Priority overview](#priority-overview)
- [Work already present in the baseline](#work-already-present-in-the-baseline)
- [How to run and record acceptance checks](#how-to-run-and-record-acceptance-checks)
- [High-priority recommendations](#high-priority-recommendations)
- [Medium-priority recommendations](#medium-priority-recommendations)
- [Low-priority recommendations](#low-priority-recommendations)
- [Java naming standard and concrete renames](#java-naming-standard-and-concrete-renames)
- [Formatting and static-analysis tooling](#formatting-and-static-analysis-tooling)
- [Verification without a unit-test program](#verification-without-a-unit-test-program)
- [Delivery plan](#delivery-plan)

## Expectations for the programming team

The following working agreement is recommended for this effort. Assign the roles at the preseason kickoff; the document does not assume that anyone has already accepted an assignment or approved a change.

- **The software mentor sets the release expectations with the student programming lead.** Confirmed high-priority defects must be resolved for retained capabilities. Agree on additional hardening and style requirements before assigning them; their priority is a team decision. An optional feature may be deferred by recording its exclusion and keeping it unavailable in the release.
- **The student lead assigns manageable work and coordinates shared changes.** Every package has one accountable owner and a reviewer. Students should be able to explain their implementation and demonstrate the acceptance procedure.
- **Technical alternatives are welcome when supported by evidence.** Bring the affected code, the reason for the alternative, and a way to compare results. The designated reviewer resolves routine implementation choices; the mentor resolves unresolved architecture, scope, and release decisions. Unanimous agreement is not a prerequisite for moving forward.
- **The checked-in coding standard applies to everyone.** Decide the rules during setup, then use the same formatter and checker on every contribution. Exceptions need a specific reason and reviewer approval.
- **Completion requires a repeatable result.** A successful build establishes that the code compiles. Behavior changes also need the applicable acceptance checks and saved results. The mentor responsible for robot bring-up reviews physical limits and hardware acceptance.

Review the priority overview together, then assign individual work packages. Each student can work from the relevant recommendation and acceptance section without reading the entire document at once. Use code reviews to explain units, ownership, and timing so that more students can maintain the robot independently.

## Scope and priority definitions

The review covers the tracked Java source, build and editor configuration, controller bindings, device adapters, simulation/replay wiring, utilities, and PathPlanner assets. It follows actual call paths and distinguishes active code from inactive helpers. The earlier [Technical Guide](TECHNICAL_GUIDE.md) explains the architecture in more introductory detail.

The inventory contains 75 tracked Java files, approximately 9,819 lines including comments and bundled helpers, 66 autonomous compositions, and 92 path files. Asset JSON parsing succeeded, and referenced path files exist. Running those routines on a robot still requires separate verification. The findings are based on source inspection and asset checks, with framework behavior checked against WPILib `2026.2.1` sources and AdvantageKit `v26.0.1` templates. No simulator run, replay, deployment, or physical measurement establishes behavioral acceptance here. Later formatting/build work does not close those checks.

**High:** address before accepting the affected feature into the reusable foundation or enabling it on the new robot. This covers confirmed control defects and proposed team release requirements. H10's coding standard is a team policy, not a robot correctness issue or WPILib requirement. For optional features such as replay, explicitly disabling or excluding the unfinished feature is an acceptable resolution.

**Medium:** planned preseason work to improve readability, diagnostics, and maintainability. These items usually allow an initial controlled bring-up, but need an assigned owner or an explicit deferral before release.

**Low:** cleanup to schedule after the foundation is stable, including improvements to currently unused features. If one of those features becomes necessary, move its verification ahead of its first use.

Priority reflects the consequence of carrying an issue forward. A source-level problem does not establish that it caused a past match failure. Physical geometry, gain quality, actual wiring, and 2027 compatibility still require measurements and checks with the applicable season toolchain.

Read priority together with the **basis** for a recommendation:

- **Confirmed defect:** the call path or calculation demonstrates a mismatch, such as running a registered subsystem twice or mixing RPM and RPS. This does not claim an observed match failure.
- **Hardening:** additional protection against invalid data, device faults, or operating conditions. State the scenario being protected against; do not imply it already occurred.
- **Team choice:** a naming, design, feature-scope, or workflow decision. Explain the benefit and tradeoff. A different choice can still follow WPILib and FRC practice.

Some sections contain more than one basis. Their acceptance procedures must follow the scope actually adopted. In particular, H7's uncertainty repair is high priority; extra camera guards and stream-selection changes can be separate hardening work. The [Architecture Review's upstream comparison](ARCHITECTURE_REVIEW.md#recognize-the-upstream-starting-point) identifies inherited template patterns so we do not mistake them for student mistakes.

## Priority overview

| ID | Priority | Recommendation | Scope | Basis |
| --- | --- | --- | --- | --- |
| H1 | High | Give each subsystem exactly one update per cycle | Robot loop and shooter | Confirmed defect |
| H2 | High | Establish one timestamp-consistent odometry pipeline | Drive and shared state | Confirmed defect |
| H3 | High | Define and verify heading/reset conventions | Gyro, driving, alliance handling | Confirmed behavior mismatch; team frame policy |
| H4 | High | Fix command ownership, mode gating, and termination behavior | Controls and autonomous | Confirmed defect; team operating policy |
| H5 | High | Establish valid mechanism references and travel limits | Turret, hood, intake | Confirmed limit behavior; hardening |
| H6 | High | Make shooting use one solution and explicit readiness policy | Shooter and feeder | Confirmed inconsistencies; team shot/feed policy |
| H7 | High | Apply vision uncertainty and validate camera observations | Vision and estimator | Confirmed defect; separate hardening |
| H8 | High | Make supported runtime modes complete and internally consistent | Real, sim, replay | Confirmed defects; team coverage choice |
| H9 | High | Separate reusable code from season/hardware assumptions | Configuration and migration | Team design choice; configuration reconciliation |
| H10 | High | Establish an enforceable Java standard and non-mutating CI checks | Build and style | Team style/workflow choice |
| M1 | Medium | Rename team-owned APIs systematically | Whole maintained source tree | Team naming choice |
| M2 | Medium | Reduce global mutable state and hidden construction effects | Constants, state, utilities | Team design choice; hardening |
| M3 | Medium | Make IO contracts explicit and failures observable | Hardware adapters | Hardening; team API choice |
| M4 | Medium | Restore persistent logging and improve diagnostics | Logging and dashboards | Team recording/diagnostics choice |
| M5 | Medium | Integrate only supported autonomous assets | Autonomous tooling | Inactive integration gaps; team scope choice |
| M6 | Medium | Simplify build, deployment, and IDE configuration | Developer workflow | Team workflow choice |
| M7 | Medium | Remove unfinished and obsolete code from the reusable core | Legacy mechanisms and helpers | Team scope/maintenance choice |
| M8 | Medium | Fix retained shared utility defects | Choosers, tuning, caching | Confirmed defects or risks in retained helpers |
| L1 | Low | Correct LED boundary and waveform behavior | LED utility | Confirmed defects |
| L2 | Low | Make geometric utilities and direction names unambiguous | Zones and directions | Team API/geometry policy |
| L3 | Low | Harden characterization commands before exposing them | Drive calibration helpers | Hardening of inactive helpers |
| L4 | Low | Improve comments and optimize only measured bottlenecks | Documentation and loop efficiency | Team maintenance choice |

## Work already present in the baseline

The following changes are implemented in the reviewed source. This is a status record, not a claim that every H10/M6 acceptance step has passed.

| Work | Baseline status | Remaining work |
| --- | --- | --- |
| Explicit formatting | `compileJava` no longer depends on `spotlessApply` | Keep the build free of source-rewriting verification steps |
| Formatting scope | Java, Gradle, JSON, and Markdown targets are scoped; generated `BuildConstants.java` is excluded | Review any additional generated/imported-code exceptions separately |
| Pinned formatter | Spotless `6.25.0`, google-java-format `1.21.0` | Change versions only with compatibility checks and a reviewed formatting diff |
| CI | Java 17 setup, `spotlessCheck`, then `build` | Verify a hosted run and remaining checker/report integration |
| Local hook | Versioned pre-commit hook checks staged whitespace and working-tree formatting | Each checkout must activate it; follow the [README](../README.md) and preserve existing hooks |
| Naming enforcement | Checkstyle is absent | Adopt rules, install the checker, migrate names, and demonstrate failures/passes |
| Deployment/editor workflow | `eventDeploy` and editor settings remain | Decide the event commit policy and complete M6 verification |

The formatter downgrade records a team version choice; Java 17 alone does not require changing from google-java-format `1.22.0` to `1.21.0`. The `1.22.0` sources include a Java 17 build profile. [Formatter build configuration](https://github.com/google/google-java-format/blob/v1.22.0/core/pom.xml). The local hook does not format, stage, or validate robotics behavior. With partial staging, its working-tree check can differ from the committed snapshot that CI checks.

## How to run and record acceptance checks

The procedures below describe verification to perform **after implementing the corresponding change**. They are not completed test results. Owners and reviewers remain unassigned; tooling packages have the partial implementation status recorded above. Use the recommendation IDs, such as H2 or M4, in changes and evidence records so the plan remains usable after Java names change.

### Environments and prerequisites

| Label | What it means | What it can establish |
| --- | --- | --- |
| Desktop review/build | Source inspection, Gradle tasks, debugger, or a small diagnostic runner | Naming, compilation, contracts, and known input/output cases |
| SIM | Desktop robot process with simulated/no-op IO | Scheduling, state transitions, and the behavior actually modeled |
| REPLAY | Recorded measurements processed by the updated robot logic | Repeatable behavior against known input; requires completed replay wiring |
| ROBOT | Controlled execution on the physical machine | Wiring, sensor frames, referencing, actual response, and calibration |

Run Gradle commands from the repository root; on Windows use `gradlew.bat` instead of `./gradlew`. `./gradlew simulateJava` starts the desktop application, but the current build disables the simulation GUI by default. Enable the needed GUI/Driver Station extensions through the team's launch configuration when implementing these checks. Select the intended desktop mode before launching. `checkstyleMain` is a **future** task until H10 installs it; do not treat “task not found” as a successful style check.

Existing telemetry includes `RobotState/EstimatedPose`, `Drive/Gyro`, `SwerveChassisSpeeds/Measured`, and mechanism inputs. Open the relevant live NetworkTables data or saved log in the team's viewer, such as AdvantageScope. Signals prefixed with `Acceptance/`, cycle counters, rejection reasons, solution IDs, and command-owner diagnostics below are **proposed instrumentation**, not promises that those dashboard entries already exist. Add them temporarily or as useful permanent diagnostics during implementation. Record where each signal appears.

For invalid sensor values, short camera arrays, or configuration failures, prefer a desktop diagnostic input or an IO stub that supplies controlled values. A small diagnostic runner is sufficient; these procedures do not require a JUnit suite. Do not unplug energized CAN devices or force a mechanism past a stop to reproduce a software condition. Physical runs follow the team's normal mechanism bring-up procedure; exercise one mechanism at a time before combined actions.

### Pass criteria and evidence

Distinguish exact software expectations from measured physical tolerances. One callback per loop and no duplicate sample submission are exact requirements. Position error, velocity error, settling time, and shot repeatability require team-selected limits based on the mechanism. Before a physical run, record the limit, units, measurement method, number of repetitions, and who selected it. An unset limit is **pending**, not an automatic pass. Example distances, durations, and speeds below are diagnostic inputs, not universal calibration targets or safe hardware limits.

Keep evidence in a proposed location such as `docs/verification/2027/<change-id>.md`, with large logs in the team's normal log storage. A record should contain:

```text
Recommendation / delivery task:
Code revision and any uncommitted diagnostic changes:
Implementer / verifier / date:
Environment, robot identity, runtime mode, relevant tool/firmware versions:
Prerequisites completed:
Inputs and numeric limits selected before the run:
Steps actually performed:
Expected result / observed result for each step:
Result: PASS / FAIL / BLOCKED / NOT APPLICABLE
Evidence: log location and timestamps, screenshot, or command output:
Follow-up issue and steps to repeat:
Temporary instrumentation removed or intentionally retained:
```

Use `BLOCKED` when required robot access, a prerequisite, or a numeric criterion is missing. `NOT APPLICABLE` requires a documented decision to exclude the feature from the supported foundation; it cannot stand in for an untested supported feature. Save failing observations as well as successful reruns. A reviewer should be able to repeat the check from the record without asking which button, goal, or coordinate convention was used.

For temporary negative checks, save the starting diff and remove only the deliberate probe afterward. Keep unrelated work intact. Capture before/after Git status for tooling checks; perform checks requiring an empty working tree in a dedicated clean checkout. Restore the normal desktop mode and approved runtime configuration when finishing diagnostics.

## High-priority recommendations

### H1. Give each subsystem exactly one update per cycle

**What the code does:** [Shooter.java](../src/main/java/frc/robot/subsystems/shooter/Shooter.java), `periodic()`, manually invokes the hood, turret, and flywheel periodic methods. Each child already extends `SubsystemBase`, directly or through `FullSubsystem`. The scheduler also invokes registered subsystem periodic methods, as described in the [WPILib scheduler documentation](https://docs.wpilib.org/en/stable/docs/software/commandbased/command-scheduler.html).

**Why this matters:** duplicate device reads and logging; simulated shooter models advance by 20 ms twice within a nominal 20 ms robot cycle. Timing-sensitive logic is harder to interpret.

**Recommended action:** remove child periodic calls from the coordinating shooter class. Use a plain command coordinator if it only combines child commands. Retain a subsystem only when it has its own scheduler-managed responsibility. Record who owns each lifecycle callback. Continue manually updating `Module` objects because those are not registered subsystems.

Document output timing for each mechanism. Immediate IO writes, applying a stored goal in `periodic()`, and a deliberate post-scheduler output stage can each work. Currently the flywheel writes immediately, hood angle requests wait for `periodic()`, and turret requests use the post-scheduler stage. Standardizing these is a design choice; removing duplicate registered-subsystem updates is the required repair. Do not introduce a second update path when changing the policy. Use one loop-period source for the robot loop and simulation steps instead of scattered `0.02` literals.

#### Acceptance H1

**Environment and prerequisites:** desktop/SIM; the lifecycle fix must be implemented. This check does not require accurate motor physics or a physical robot.

1. Add a robot-cycle counter incremented once at the start of `robotPeriodic()`. Count each hood/turret/flywheel `periodic()` call, IO `updateInputs()` call, and simulated model step independently. Publish a snapshot after all loop stages, for example under `Acceptance/Lifecycle`. Counting calls is essential: two writes to the same log key can conceal duplicate execution.
2. Start SIM disabled. Compare two end-of-cycle snapshots covering 100 complete robot cycles, after startup. Each child's periodic/input count must increase by exactly 100. Each modeled child's accumulated simulated time must increase by `100 * LOOP_PERIOD_SECONDS`, subject only to numeric rounding.
3. Repeat enabled with idle commands, then with shooter tracking scheduled, then after cancelling it. Also count the turret's post-scheduler output callback; it must run once per cycle when that stage is retained.
4. Inspect the call sites: no coordinator manually updates an already registered subsystem. Confirm plain `Module` objects still have one explicit owner. Document any deliberately multirate IO sampling separately from main-loop callbacks.

**Pass:** exact call counts match in every tested mode; a model using a 20 ms step advances 2.0 simulated seconds over 100 cycles. Do not compare this to wall-clock elapsed time in accelerated replay or a paused debugger. Unexpected doubles, zeros, or missed stages fail.

**Evidence and cleanup:** save counter snapshots, cycle range, and call-site review. Remove temporary counters, rebuild, and repeat a short startup/command smoke check on the final version.

### H2. Establish one timestamp-consistent odometry pipeline

**What the code does:** [Robot.java](../src/main/java/frc/robot/Robot.java) runs the container before the scheduler. [RobotContainer.java](../src/main/java/frc/robot/RobotContainer.java), `robotPeriodic()`, submits cached drive measurements with the current timestamp. [Drive.java](../src/main/java/frc/robot/subsystems/drive/Drive.java) refreshes them later. Its high-frequency estimator update and wheel-based gyro fallback are commented out. [GyroIOPigeon2.java](../src/main/java/frc/robot/subsystems/drive/GyroIOPigeon2.java) registers queues but leaves extraction and clearing commented out.

**Why this matters:** measurement age and timestamp differ by roughly a main-loop interval. Fast sampling adds complexity without supplying fast estimator updates. The missing heading fallback also prevents realistic simulated rotation.

**Recommended action:** start with a correct, explicit 50 Hz pipeline if necessary: refresh drive inputs, update pose using their measurement timestamp, publish measured chassis velocity, then let behavior consume the resulting state. Place this in one responsible owner, preferably the drive/state-estimation boundary. Do not simply move the container callback after the scheduler: commands would still consume the previous estimate, and ordering would remain implicit.

If high-frequency odometry is retained, submit synchronized module and gyro samples with their original timestamps and remove the duplicate low-frequency submission. Restore queue draining, validate array alignment, and handle missing samples deliberately. Bounded queues can drop samples when full; this is not an unbounded-memory leak, but dropped data should be observable. Put `odometryLock.unlock()` in `finally` so an exception during input refresh cannot leave the background thread blocked.

The intended contract is:

```text
fresh measurements + their timestamps
    → one pose/velocity update
    → commands consume current state
    → outputs applied
```

#### Acceptance H2

**Environment and prerequisites:** desktop/SIM for ordering and injected samples; ROBOT for measured accuracy. H1 must pass. Validate gyro simulation/fallback from H8 before using simulated rotation as evidence; validate H3's frame convention before judging physical headings.

1. Record which owner refreshes inputs and submits odometry. Instrument cycle ID, sample ID/timestamp, state-update timestamp, measured chassis speeds, and the state version read by a drive/aim command. For 50 Hz operation, verify commands consume the refresh from that cycle. For fast sampling, verify they consume the latest completed estimate and each retained sample is submitted once.
2. Inject a short known sample sequence in a desktop IO diagnostic. Confirm timestamps are preserved, increasing as expected, and not replaced with the time of later submission. Supply missing/misaligned arrays and verify they are rejected or handled by the documented policy without indexing exceptions. Check reset boundaries separately from normal samples.
3. If queues are retained, run long enough to observe repeated draining, then simulate a consumer delay. Confirm overflows/drops are counted and recovery does not repeatedly submit old samples. Review lock handling for `try/finally`; use a desktop injected read exception to verify the lock is released, independent of whether the outer application treats that exception as fatal.
4. Run forward, sideways, and rotation commands in SIM; compare signs of measured speed and pose change. With H3 passed and physical error limits recorded, drive a marked straight segment, a lateral segment, a turn, and a square on the robot. Measure start/end pose independently rather than using the estimator as its own reference.
5. Inject a disconnected gyro in SIM and verify the chosen fallback and alert; restore valid input and inspect recovery for unexpected heading jumps.

**Pass:** no double submission, stale-cycle substitution, unhandled mismatched arrays, or unreleased lock. Queue recovery matches the declared policy. Measured physical errors meet the team's recorded limits; simulator motion alone cannot satisfy that criterion.

**Evidence:** sample/cycle trace, dropped-sample counts, measured-versus-estimated displacement table, and log ranges for disconnection/recovery. Mark the ROBOT portion blocked if hardware is unavailable.

### H3. Define and verify heading/reset conventions

**What the code does:** `GyroIOPigeon2` initializes yaw to zero, but its `setYaw(angle)` adds 180°. `Drive.setYaw()` immediately stores the unadjusted argument until the next refresh. `Drive` calls the command factory `zeroYaw()` during construction without scheduling its result. Joystick drive uses raw gyro heading, while aiming uses estimator heading.

**Why this matters:** the heading can change meaning across startup, reset, sensor refresh, and vision correction. This is especially confusing when alliance flipping adds another 180° transformation.

**Recommended action:** document raw sensor heading, robot-forward heading, estimated field heading, and the driver's field-relative perspective separately. Apply a physical gyro mounting offset in one place. Provide a direct reset operation and a clearly named command factory for invoking it. Reset pose with current module positions and gyro information where required. A software pose reset and a hardware yaw reset should be deliberate, distinct operations.

Choose explicitly whether joystick driving follows an estimator heading or a gyro-derived heading with a driver offset. Both require consistent reset semantics; do not accidentally change driver feel as part of a naming cleanup. Keep one field-coordinate convention and transform alliance-specific targets and driver perspective at clear boundaries.

#### Acceptance H3

**Environment and prerequisites:** desktop input diagnostics followed by ROBOT; H2's measurement ordering must be established. Record the chosen sensor mounting offset, field origin, driver-forward convention, and what each reset operation promises to change.

1. Prepare an expected-value table for robot orientations forward, left, backward, and right relative to a marked field reference. In a standard robot-forward convention these correspond to 0°, +90°, ±180°, and -90° before any deliberate field/driver offset. Include expected raw gyro, corrected robot heading, estimated field heading, and driver offset separately.
2. Inject those headings on desktop and compare every transformed value with the table. Repeat with blue, red, and absent alliance. Field pose should retain the chosen field frame; alliance-dependent targets/driver perspective should change only where specified.
3. On the robot, align to each marked orientation and log the same values. Invoke the actual heading-reset control, then inspect the immediate result, the next input refresh, and several subsequent cycles. Repeat reset twice at the same orientation; no additional offset should accumulate.
4. Exercise an arbitrary pose reset with current wheel positions. Move afterward and confirm odometry continues from the new pose without a spurious wheel-distance jump. If vision is active, inspect the transition when a new camera observation arrives.
5. At low agreed speed, command the same driver-forward direction before/after reset and under both alliances. Compare with the documented driver-perspective behavior, including release/repress if required by the binding.

**Pass:** expected transformed angles match in desktop cases modulo whole turns, using numeric tolerance; hardware readings meet a separately chosen angular tolerance. No transient 180° snap, accumulating reset, or unexplained translation reversal is accepted. A deliberate change of driver perspective must match the table.

**Evidence:** the expected/observed table and reset timestamps, including at least the following sensor refresh. Document absent-alliance behavior explicitly.

Sources: [GyroIOPigeon2.java](../src/main/java/frc/robot/subsystems/drive/GyroIOPigeon2.java), [DriveCommands.java](../src/main/java/frc/robot/commands/DriveCommands.java), [AllianceFlipUtil.java](../src/main/java/frc/robot/util/AllianceFlipUtil.java).

### H4. Fix command ownership, mode gating, and termination behavior

**What the code does:** hood D-pad `StartEndCommand`s in [DriverControls.java](../src/main/java/frc/robot/control/DriverControls.java) omit a hood requirement, while the hood has an active default command. Intake roller and pivot commands all claim `Intake`. The active autonomous command claims neither drivetrain nor turret. Controller bindings and joystick defaults are not explicitly restricted to teleop. Some tracking commands rely on a later default to restore outputs.

**Recommended action:** require the correct subsystem on every mechanism command. Audit simultaneous button presses, not just each button independently. If roller and pivot must operate independently, split their scheduler ownership or supply an intentional combined command; do not remove requirements to bypass conflicts.

Gate normal driving and operator actions to enabled teleop, or document narrowly scoped exceptions. Give autonomous explicit drivetrain/turret behavior, including a stationary command when stationary is intended. Specify output behavior for cancellation, disabled entry, re-enable, and test mode. Hardware inhibition while disabled is not the same as clearing a retained software goal. `testInit().cancelAll()` alone does not stop bindings/defaults from rescheduling later.

`whileTrue` bindings also need practical verification after an interruption: a command interrupted while its button remains held may need a new trigger edge before restarting. A command conflict can therefore feel like a dead button even after the competing action finishes.

#### Acceptance H4

**Environment and prerequisites:** SIM with command scheduling and output-request diagnostics, then controlled ROBOT confirmation. H1 must pass. No-op intake/indexer IO needs output capture to establish scheduler behavior; unchanged sensor values are not evidence that output requests stopped.

1. Create a control matrix from the implemented bindings: input, commanded action, required subsystem(s), permitted modes, cancellation output, and intended resumption behavior. Log command initialize/end/interruption events and mechanism requested mode/output. Define neutral versus position-hold behavior per mechanism.
2. For each binding, press, hold for several cycles, and release. Confirm one intended owner controls the mechanism, its requested output is maintained correctly, and cancellation applies the declared idle behavior within the selected output-stage timing.
3. Hold automatic shooter tracking and request hood/turret manual movement. Repeat intake roller plus pivot, and opposing controls. Verify the declared conflict policy: cancellation, intentional composition, or rejection. Hold the original button through interruption, then release/repress; check whether resumption requires a new edge and that this matches operator instructions.
4. Disable while each representative action is held, re-enable with it still held, then release/repress. Inspect stored goals as well as output requests. Enter test mode and confirm normal bindings/defaults do not restart unless explicitly permitted.
5. Start the supported stationary autonomous routine and move driver sticks. Confirm it retains the declared stationary-drive ownership. Transition to teleop and verify auto ends and the teleop defaults take ownership. Check autonomous cancellation and any timeout separately.

**Pass:** each matrix entry behaves as specified; no hood default competes with manual control, no unintended movement is requested across mode transitions, and no stale goal resumes outside the chosen policy. Physical output inhibition while disabled does not by itself prove correct software state cleanup.

**Evidence:** completed control matrix and event/output log timestamps for conflicts and transitions. Repeat affected rows after later binding changes.

### H5. Establish valid mechanism references and travel limits

**What the code does:** turret and hood relative encoders are set to zero at startup without establishing a physical reference. Turret Spark position wrapping is enabled while configured mechanical travel is limited to -90°…210°, and hardware soft limits are disabled. Turret open-loop code blocks both motion directions after the measured angle is outside the range. Hood open-loop control bypasses angle clamping. Intake deployment is open-loop, with no end-position input in its IO record.

The source cannot establish whether the team already aligns the mechanisms physically before startup. Confirm that procedure with the operators before changing it. This section combines observable limit-handling behavior with proposed referencing and fault-response requirements; it does not prove the robot has been operated without a valid reference.

**Why this matters:** software limits are meaningful only if the encoder's zero is meaningful. A wrapped target also does not identify which mechanically reachable turn should be used.

**Recommended action:** define a startup referencing procedure for each retained mechanism: an absolute sensor, controlled homing with an appropriate sensor, or a documented physical alignment procedure. Track whether the mechanism is referenced and permit position-dependent behavior only when valid. Do not let a casually accessible “zero” button silently redefine travel limits at an arbitrary position.

For a limited-travel turret, represent physical position in an explicitly bounded, unwrapped coordinate and choose a reachable target angle. Avoid continuous-wrap control unless its behavior is proven to respect the actual permitted travel. Apply limits consistently to manual and automatic outputs, allow controlled movement back into range, and compute readiness against the feasible target. Establish intake endpoints and time/current bounds appropriate to the actual mechanism. Confirm motor current limits and signs from wiring and measurements rather than transferring values blindly.

#### Acceptance H5

**Environment and prerequisites:** desktop/SIM boundary inputs followed by ROBOT. H4 must establish ownership/stopping. Record physical reference procedure, coordinate signs, permitted travel, inward recovery direction, homing timeout if applicable, and hardware-specific current/output limits before running the mechanism.

1. Start with the mechanism reference declared invalid. Request manual and automatic actions and confirm only actions explicitly allowed during referencing can proceed. Complete the real referencing procedure; compare reported zero with the known physical mark/sensor. Power-cycle and repeat to establish startup behavior.
2. In a diagnostic adapter, supply measurements just inside, exactly at, and just outside each software boundary. Request both outward and inward movement. Outward requests must obey the limit policy; inward recovery must remain available when appropriate. Check zero output and disconnected-feedback cases too.
3. Supply reachable and unreachable automatic targets, including equivalent turret directions around a whole-turn boundary. Inspect the selected unwrapped target, applied/clamped target, travel path, and readiness result. A feasible target must not be reached by crossing forbidden travel, and an invalid request must not report ready.
4. On the robot, approach each limit using constrained output and a known reference. Verify stopping/holding before mechanical interference and controlled inward recovery. Test an unreachable target through software requests, not by pushing beyond a physical stop. Verify the encoder-reset control cannot casually invalidate the reference policy.
5. For intake endpoints or homing, exercise successful completion and a simulated missing-endpoint condition. Confirm timeout/current policy changes the request and reports why it stopped.

**Pass:** all desktop boundary cases meet the chosen policy. Physical position/reference accuracy and stopping margin meet limits established before the run. Failure to establish a valid reference remains visible and prevents reference-dependent operation. Any unmeasured mechanical margin leaves ROBOT acceptance pending.

**Evidence:** reference procedure, signed travel diagram, boundary input/output table, and physical position/current/output traces. Record what is software-verified versus physically verified.

Sources: [Turret.java](../src/main/java/frc/robot/subsystems/shooter/turret/Turret.java), [TurretIOSparkMax.java](../src/main/java/frc/robot/subsystems/shooter/turret/TurretIOSparkMax.java), [HoodIOSparkMax.java](../src/main/java/frc/robot/subsystems/shooter/hood/HoodIOSparkMax.java), [IntakeIO.java](../src/main/java/frc/robot/subsystems/intake/IntakeIO.java).

### H6. Make shooting use one solution and explicit readiness policy

**What the code does:** [TrajectoryCalculator.java](../src/main/java/frc/robot/subsystems/shooter/TrajectoryCalculator.java) has a full solution plus separate helpers returning `0.6 * RPM` and `0.5 * hoodAngle`. Active turret tracking uses separate geometry, active hood tracking uses the half-angle helper, and active flywheel tracking uses the full calculator. `RobotState.setRobotVelocity()` has no active callers. The one-time autonomous wait checks cached flywheel readiness; manual feed has no readiness check.

Flywheel tolerance `25.0` is compared in rad/s, approximately 239 RPM. Turret/hood readiness updates occur in setters and use falling-edge debounce. Flywheel open-loop cancellation leaves its old goal RPM stored. A stationary flywheel can have an at-goal result for its initial zero target before a new shot request is processed.

**Recommended action:** introduce one clearly named `ShotSolution` value with explicit units, feasible targets, validity, and an explanation when invalid. Calculate it once from a single state snapshot and apply its outputs consistently. Calibrate the combined behavior; removing the half-angle or sign adjustment without measuring could change a working physical shot.

Supply measured robot-relative velocity to shared state if motion compensation is retained. Until its entire output path is validated, label that feature unavailable. Bound valid distances by measured calibration and make out-of-range behavior explicit rather than only logging it.

Recompute readiness from current inputs, current command mode, connection/reference validity, and the applied target. Clear readiness on a new shot goal or stop. Choose settling/debounce timing deliberately, including what happens when a ball causes a brief RPM dip. Automatic feeding should follow the chosen readiness policy continuously if that is the intent; an explicit operator override can remain. Do not force every mechanism into an identical debounce policy without considering its purpose.

#### Acceptance H6

**Environment and prerequisites:** desktop/SIM for solution consistency and feeder logic; ROBOT for calibration. H1–H5 and the relevant H8 unit/mode corrections must pass for integrated simulation. Use fixed known poses until H7 vision is accepted. Record valid shot range, settling policy, readiness tolerances, and how an intentional feed override behaves.

1. Add a cycle/solution identifier and log the solution's pose/velocity snapshot, distance, validity/reason, and turret/hood/flywheel targets. Request known table distances and an interpolation midpoint. Compare the calculator output with independently calculated values from the retained calibration table; confirm all three consumers apply the same solution ID and declared units/sign transforms.
2. Exercise a known conversion on desktop: 2400 RPM corresponds to 40 RPS and approximately 251.33 rad/s. Check both the IO request and measured-value conversion. This numerical example is not a requirement to run a physical flywheel at that speed.
3. Start stopped, with measured speed zero. Request a nonzero shot and inspect readiness on the very first cycle and through settling. Change the target, cancel, and restart. Old zero-speed or old-target readiness must not authorize the new request. Verify applied/clamped targets are the ones used for readiness.
4. Supply ready/not-ready measurements, a brief and a sustained speed dip, invalid distance, and disconnected or unreferenced feedback in the diagnostic adapter. Verify feeder output follows the documented continuous/latched policy and delay thresholds. Exercise any explicit override separately; it must not disguise a not-ready condition.
5. On the robot, shoot from several measured distances within the proposed calibrated range. Record attempts and outcomes against shot-repeatability criteria chosen beforehand, along with actual speed/angles. Repeat goal changes and observe the dip from normal feeding; do not infer calibration quality solely from an at-goal flag.
6. If moving shots are retained, first verify nonzero measured velocity reaches the solution. Compare zero-velocity and controlled nonzero-velocity calculations, then perform separately planned physical trials. Otherwise mark moving-shot support excluded and verify it is not advertised/enabled.

**Pass:** one coherent solution drives the retained mechanisms, conversion cases agree numerically, readiness resets and feed transitions match the selected policy, and physical shot criteria pass. Passing stationary shots does not accept motion compensation.

**Evidence:** solution/consumer trace, readiness/feed timeline, calibration-distance/result table, and an explicit supported-shot-mode declaration.

### H7. Apply vision uncertainty and validate camera observations

**What the code does:** [Vision.java](../src/main/java/frc/robot/subsystems/vision/Vision.java) calculates measurement standard deviations, including an infinite MegaTag 2 heading standard deviation. [RobotState.java](../src/main/java/frc/robot/RobotState.java) discards them by calling the estimator overload with only pose and timestamp. [CameraIOLimelight.java](../src/main/java/frc/robot/subsystems/vision/CameraIOLimelight.java) checks only that raw arrays are nonempty before reading indices through 9. Numeric filters do not explicitly reject non-finite measurements.

**Scope:** dropping the calculated uncertainty is a confirmed integration defect. The parsing assumptions and use of both MegaTag streams also appear in the [AdvantageKit `v26.0.1` Limelight adapter](https://github.com/Mechanical-Advantage/AdvantageKit/blob/v26.0.1/template_projects/sources/vision/src/main/java/frc/robot/subsystems/vision/VisionIOLimelight.java). Additional input guards are hardening; no malformed live camera packet or resulting crash was observed in this review. Split that work from the uncertainty repair so it can be reviewed and accepted independently.

**Recommended action:** pass the uncertainty matrix into the estimator's corresponding overload; that API accepts per-measurement standard deviations. See the [WPILib pose-estimator API](https://github.wpilib.org/allwpilib/docs/release/java/edu/wpi/first/math/estimator/PoseEstimator.html).

For the hardening package, validate the expected base payload length and optional tag payload structure, finite pose coordinates, plausible tag count/distance, and usable timestamps before constructing observations. Reject stale, malformed, or impossible observations with logged reasons. Preserve deliberate infinite uncertainty where it expresses “do not trust this measurement component”; do not blanket-reject that configuration as invalid numeric input.

Both MegaTag 1 and MegaTag 2 streams currently feed the estimator. They may describe the same image and share information. Record whether to retain both with deliberate weighting or select a stream, then compare that policy using suitable logs. Processing both is not by itself a WPILib violation or a demonstrated localization failure. Keep camera and field configuration consistent between geometry and vision.

#### Acceptance H7

**Environment and prerequisites:** desktop diagnostic inputs; SIM or completed REPLAY for repeatable fusion; camera/ROBOT for live measurements. H2/H3 must provide trustworthy timing and frames before judging live fusion. Define the accepted payload schema and stale/future timestamp thresholds for the retained camera protocol when implementing hardening.

For the small uncertainty repair, use steps 3 and 4's matrix/heading checks with controlled estimator history; it need not wait for the full odometry refactor or parser work. Steps 1–2 cover the additional parser hardening. Step 4's stream-policy review is separate from passing through the matrix. Step 5 establishes live integration after the relevant timing/frame work. Record these results separately.

1. Build a small input matrix: empty array; short nonempty array; valid base payload; truncated optional tag data; zero/invalid tag count; non-finite pose/distance; stale/future timestamp; out-of-field pose; valid single-tag and multi-tag observations. Inject at the camera parsing boundary, before pose construction, so the check covers parsing as well as filtering.
2. For each case, record parser outcome, accepted/rejected reason, observation timestamp, and estimator-update count. Invalid cases must not call the estimator or throw an uncaught parsing exception. A malformed packet followed by a valid one must recover without requiring a process restart.
3. Compare uncertainty influence in two isolated runs with identical initial estimate, odometry history, valid timestamp, and pose residual. Submit the same pose once with low standard deviation and once with high standard deviation. Disable other camera inputs for this comparison. The higher-uncertainty measurement should make a smaller correction; changing values while simultaneously moving the robot is not a controlled comparison.
4. Exercise MegaTag 2 with the intentional untrusted-heading setting: a differing heading must not produce an independent heading correction, while usable translation can still contribute. Verify the matrix reaches the estimator and no `NaN` emerges from uncertainty calculations. Check the declared per-frame MegaTag 1/2 selection/correlation policy using observation IDs or timestamps.
5. In live operation, record a stationary known pose, a short move, tag loss/covering, and reacquisition. Inspect rejection reasons, pose continuity, and connection status. Compare location error with the team's selected measurement limits.

**Pass:** the uncertainty repair passes when the supplied matrix reaches the estimator, weighting changes the controlled result as expected, and the intentional untrusted-heading setting is preserved. The additional hardening passes when invalid inputs are rejected predictably, valid recovery succeeds, and stream handling matches the documented strategy. Live integration needs measured accuracy against recorded criteria. Pending hardening must not be reported as complete, but it does not invalidate independently demonstrated uncertainty repair. A camera connection indicator alone does not establish pose quality.

**Evidence:** input matrix, controlled uncertainty comparison traces, accepted/rejected counters, and live log timestamps. Restore normal camera inputs after the controlled comparison.

### H8. Make supported runtime modes complete and internally consistent

**What the code does:** the container has no `REPLAY` construction case. Selecting it leaves required subsystems null when bindings are configured. Intake/indexer simulation is effectively empty, the simulated gyro does not change yaw, and flywheel simulation compares an RPS setpoint with RPM measurement. Flywheel `updateInputs()` always overwrites voltage from its PID, including after open-loop or stop requests. Its simulated reduction of 300 is not mirrored in the real adapter's configuration.

**Recommended action:** use explicit construction for every advertised mode. For replay, construct real subsystem logic with appropriate no-op IO and let recorded inputs populate it. If replay is deferred, fail clearly at startup or remove it as a selectable supported mode. Do not advertise a partially initialized mode as operational.

For simulation, correct units and introduce explicit stopped/open-loop/closed-loop state where necessary. Ensure stop cannot be overwritten by a stale closed-loop target, and verify disabled behavior. Implement only the simulation fidelity needed for the team's workflow: a simple model or clearly named no-op is preferable to a misleading physical model. Complete gyro behavior through a chassis model or documented kinematic fallback. Reconcile mechanism reductions against the real hardware.

#### Acceptance H8

**Environment and prerequisites:** desktop for SIM/REPLAY, ROBOT for real-device initialization. Split this work: simulation units/output modes are prerequisites for mechanism simulation acceptance; complete replay can follow M4's saved logging. H1 establishes model timing. Record which mechanisms/modes are supported, modeled, intentionally no-op, or excluded.

1. Launch each supported desktop mode from a fresh process and inspect startup logs and constructors. Confirm every required subsystem has an implementation, bindings configure without null dereferences, and SIM/REPLAY do not instantiate real motor adapters. A deliberately unsupported mode must produce the declared clear startup rejection, not fail later in binding code.
2. In SIM, record requested control mode, target units, applied voltage, model speed, and accumulated model time. Exercise velocity control, open-loop output, stop, then velocity control again. For the flywheel, inspect the RPM-to-RPS conversion and the controller's feedback units. Verify model gearing and limits come from documented simulation configuration.
3. Keep a previous nonzero PID target, issue stop, and advance several cycles. Applied output must obey stopped mode; a physically coasting model may retain speed, but the controller must not reapply the stale target. Disable/re-enable and repeat. Run H1's timing check and H2's simulated rotation check on this corrected model.
4. Exercise intake/indexer commands. If modeled, inspect changing state; if intentionally no-op, verify output capture and visible unsupported status. Do not accept “the program starts” as evidence of functioning ball/mechanism physics.
5. If REPLAY is retained, use a saved compatible run containing known command/sensor transitions. Reproduce logged inputs and compare derived outputs for identified timestamps. An intentional code fix can legitimately change derived outputs; explain the expected difference instead of demanding equality with a buggy baseline. Confirm no hardware construction and that replay output does not overwrite the input log.
6. On the robot, check real-mode adapter selection and configuration/connection reports. Run only the mechanism checks already cleared for hardware execution; desktop construction checks cannot verify wiring.

**Pass:** every declared supported mode initializes and honors the same IO contracts. Stop/disabled policy persists across cycles, simulation units/time are consistent, and unsupported coverage is explicit. REPLAY may be accepted as intentionally excluded, but an advertised incomplete mode fails.

**Evidence:** mode/coverage matrix, startup output, mode-transition plots, and replay input/output filenames and timestamps. Restore the normal desktop selection after checking alternate modes.

Sources: [RobotContainer.java](../src/main/java/frc/robot/RobotContainer.java), [FlywheelIOSim.java](../src/main/java/frc/robot/subsystems/shooter/flywheel/FlywheelIOSim.java), [IntakeIOSim.java](../src/main/java/frc/robot/subsystems/intake/IntakeIOSim.java), [IndexerIOSim.java](../src/main/java/frc/robot/subsystems/indexer/IndexerIOSim.java).

### H9. Separate reusable code from season and hardware assumptions

**What the code does:** `RobotState` embeds hub target selection; `FieldConstants` and `VisionConstants` separately select the 2026 layout; shot tables, gear ratios, module offsets, CAN IDs, and auto assets belong to this robot. Java PathPlanner configuration uses 72.088 kg, while its editor settings use 52.163 kg. `Constants.kRobotConfig` is a separate mutable uninitialized field. Several configured values remain marked for tuning.

**Recommended action:** preserve the 2026 implementation in its existing repository/history. Create a small 2027 application from the supported season template when available, then port the corrected reusable pieces. Compare the drive/AdvantageKit code with the matching upstream template rather than carrying every old integration workaround forward.

Separate:

| Category | Examples | Migration policy |
| --- | --- | --- |
| Reusable behavior | Command patterns, IO contracts, swerve logic, diagnostics | Carry after fixing high-priority issues |
| Robot-specific configuration | IDs, buses, gearing, offsets, current limits, camera mounting | Measure and validate for the actual 2027 machine |
| Season logic | Field layout, target selection, hub timing, shot calibration, paths | Replace when 2027 requirements are known |
| Optional experiments | Alternate hardware adapters, caches, zone controls | Include only with a concrete use case |

Move target selection out of the generic pose store into season-level behavior. Share one chosen field layout between consumers. Choose one authoritative path-following robot model. Do not create an elaborate multi-season plugin framework: a few explicit configuration objects and season-level classes are enough for this project.

#### Acceptance H9

**Environment and prerequisites:** desktop configuration review and clean build now; ROBOT measurements and supported 2027 toolchain later. Complete the reuse inventory before extraction. H1–H8 apply only to features declared retained; exclusions must be deliberate and visible.

1. List every carried component and classify it as reusable logic, hardware configuration, season logic, or excluded experiment. For retained numeric configuration, record unit, source/measurement, robot identity, and validation owner. Reconcile conflicting mass/geometry values instead of copying both.
2. Search the proposed reusable packages for dependencies on `FieldConstants`, hub timing/target selection, shot tables, camera names, and 2026 path strings. Review matches individually: the season application may legitimately reference them, but generic drive/pose/IO code must receive those choices through an explicit boundary. Compilation alone does not detect inappropriate season coupling.
3. Run a minimal desktop application using the reusable core with a neutral/no-game target policy and no 2026 autonomous assets. Confirm generic initialization and drive/state behavior work. Keep the archived 2026 application separate so removing its assets from the new application does not erase history.
4. Compare Java path-following configuration, editor settings, and startup-reported values against the chosen authoritative model. Verify IDs with device type and bus, not bare numbers. Confirm active configuration and provenance are visible in diagnostics.
5. When the supported 2027 toolchain and hardware are available, record the actual Java, GradleRIO, vendor, and firmware versions; perform a clean build and the applicable startup/hardware acceptance checks again. Replace season targets and calibrations with measured 2027 data before enabling them.

**Pass:** the core has no hidden requirement for 2026 game data, each retained physical value has provenance, and one authoritative configuration is selected. Record “core extraction accepted” separately from “2027 robot accepted”; missing season toolchain or measurements blocks the latter.

**Evidence:** reuse/configuration inventory, reviewed dependency-search results, clean desktop build/startup output, and a later season-compatibility record.

Sources: [Constants.java](../src/main/java/frc/robot/Constants.java), [DriveConstants.java](../src/main/java/frc/robot/subsystems/drive/DriveConstants.java), [FieldConstants.java](../src/main/java/frc/robot/util/FieldConstants.java), [PathPlanner settings](../src/main/deploy/pathplanner/settings.json).

### H10. Establish an enforceable Java standard and non-mutating CI checks

**What the code does now:** the project mixes `kLoopPeriodSeconds`, `MIN_SHOOTING_DISTANCE`, mutable `kTuningMode`, and fields such as `FrontLeft`/`DrivetrainConstants`. [build.gradle](../build.gradle) uses Spotless `6.25.0` with google-java-format `1.21.0`. Compilation no longer triggers `spotlessApply`; CI explicitly checks formatting before building. There is no Checkstyle or SpotBugs configuration.

**Remaining action:** adopt the naming policy below for maintained team code, retain the existing explicit formatter workflow, and add a focused Checkstyle gate. H10 is a proposed team release policy, not a claim that `k` constants or uppercase `IO` violate WPILib guidance.

The earlier compile-time formatting and recursive targets with exclusions follow the matching AdvantageKit template. Explicit apply/check tasks and narrower targets fit our chosen review workflow: formatting changes are visible before commit and CI checks the committed input. Both targeting strategies are legitimate; the old exclusions were not evidence of an architectural defect. See the [upstream comparison](ARCHITECTURE_REVIEW.md#recognize-the-upstream-starting-point).

During migration, make existing naming violations visible and convert manageable groups. Before declaring the reusable foundation ready, require the documented team rules on all maintained team-owned code. Avoid a permanent “new lines only” exception that leaves two naming systems indefinitely.

#### Acceptance H10

**Environment and prerequisites:** desktop and CI only. Install the chosen checker/configuration; explicit formatting and the non-mutating compile dependency are already implemented. Complete M1 renames or record a temporary explicit migration baseline; final acceptance of the adopted team policy requires full enforcement on the documented maintained source set. This migration is not a prerequisite for correcting runtime defects.

1. Record the pinned formatter/checker versions, Java runtime, checked source paths, and justified exclusions. Run `./gradlew spotlessCheck checkstyleMain` on a known compliant revision. It must exit successfully with no unexplained suppressions. Confirm ignored generated sources and valid IO payloads do not produce spurious findings.
2. In a temporary team-owned Java file, introduce one violation at a time: formatter-visible spacing; `static final double kAcceptanceProbe = 1.0;`; then an ordinary field/type name that violates the team casing/acronym policy. Run the relevant task for each. Require nonzero exit and a diagnostic identifying the file and offending rule. Keep the file syntactically valid so compilation errors do not masquerade as successful style enforcement.
3. Fix/remove the probe and rerun. For the spacing case, verify `spotlessApply` explicitly repairs formatting; a subsequent `spotlessCheck` passes. Verify naming violations require a deliberate rename and are not silently rewritten by the formatter.
4. In a clean dedicated checkout, run `./gradlew build --dry-run` and inspect the graph for an unwanted `spotlessApply` dependency. Run the real build and checks, then `git diff --exit-code`. It must show no tracked source modifications. Inspect status for unexpected generated/untracked files separately; expected ignored build outputs are allowed.
5. Run the same negative and positive checks through the CI job in a temporary review branch/change. Confirm failure is not ignored and reports are retained. Finish with compliant source. Compare editor formatting with Spotless on the same sample.

**Pass:** valid code passes locally and in CI, deliberate violations fail for the intended reasons, exclusions are narrowly scoped, and checks/build do not repair or rewrite tracked input. A green job using `ignoreFailures` or unrestricted suppressions fails this acceptance.

**Evidence:** commands/exit codes, negative-check diagnostic excerpts, clean positive CI result, task graph, and before/after working-tree status. Remove all deliberate violations before accepting the change.

Tool configuration details and external references are in [Formatting and static-analysis tooling](#formatting-and-static-analysis-tooling).

## Medium-priority recommendations

### M1. Rename team-owned APIs systematically

Start with the rename map below and give each rename group one owner. Address names that obscure behavior or units first. For example, a method called `setFlywheelVelocity()` that returns a command is more misleading than the short class name `Drive`.

Apply IDE symbol refactoring for Java declarations and references. Compile after each coherent rename group. Do not use a global text replacement for `k`, `IO`, or `RPM`: that could damage vendor calls, generated class references, serialized keys, and calibration notation.

Command factories should look like actions that create scheduled work; direct setters should visibly describe immediate state changes. Keep telemetry keys stable during Java-only renames. Changing log keys, NetworkTables names, or PathPlanner named-command strings requires an explicit coordinated migration because those are external contracts.

#### Acceptance M1

**Environment/prerequisites:** desktop; H10 check tasks available. Work in one rename group at a time.

1. Keep a before/after symbol map. Use IDE reference search and refactoring, then search maintained source for the old names. Review legitimate matches in history/documentation/vendor code rather than replacing them indiscriminately.
2. Run `./gradlew spotlessCheck checkstyleMain build`. Confirm renamed IO input classes regenerate and editor references resolve.
3. Review the diff for accidental numeric, sign, unit, string-key, command-requirement, or control-flow changes. Separate any intentional behavioral change into its own work item. Compare the declared public units before/after.
4. Run the affected startup/binding smoke checks; for command-factory renames, confirm creation still requires scheduling and direct setters still act directly. Verify retained dashboard/path names still resolve.

**Pass/evidence:** no broken references or unexplained contract changes; clean build/check output, symbol map, focused diff review, and smoke-check notes. Compilation alone does not establish unchanged behavior.

### M2. Reduce global mutable state and hidden construction effects

**What the code does:** `RobotState` mixes eager singleton creation with a lazy null check, exposes a mutable `ChassisSpeeds` object, and combines generic estimation with season targeting. `Constants` holds mutable flags and configuration. `Leds` constructs hardware in a static singleton. `FullSubsystem` and `CachedSupplier` maintain static registries. Many references that never change are not declared `final`.

**Recommended action:** construct core objects in `RobotContainer`, pass narrow dependencies, and make stable references `private final`. A singleton can remain when it has a clear lifecycle, but avoid inconsistent eager/lazy patterns. Return defensive snapshots of mutable shared data where callers must not modify it. Records containing arrays or mutable objects are not automatically deeply immutable.

Separate startup configuration from runtime state. Make tuning enablement fixed at startup unless live toggling is deliberately supported. Avoid static construction of hardware so unsupported modes and future desktop tools do not accidentally initialize devices. If static registries remain, document lifecycle and prevent duplicate registration during reconstruction.

Keep the implementation small: constructor parameters and clear ownership are sufficient for this team.

#### Acceptance M2

**Environment/prerequisites:** desktop/SIM; H1 counters or equivalent lifecycle instrumentation available.

1. Review constructors/static initializers and map who owns each retained singleton, configuration, hardware object, and registry entry. Start each supported mode and confirm only its intended adapters are constructed.
2. In a desktop diagnostic, obtain the exposed velocity/array snapshot and attempt to modify it. If it is mutable, read shared state again: external edits must not change internal state. If the API is immutable, verify mutation is unavailable and inputs are copied where required.
3. Exercise supported reconfiguration/reconstruction paths and compare registry/callback counts. Do not demand full robot reconstruction in one process if unsupported; instead verify it is explicitly disallowed and normal startup registers once.

**Pass/evidence:** explicit ownership, no unintended hardware construction, no externally mutable shared state, and no duplicate callbacks. Save ownership notes, snapshot results, and counts.

### M3. Make IO contracts explicit and failures observable

**What the code does:** `setOpenLoop(double)` means volts for drive modules and duty cycle for mechanisms. Flywheel IO expects RPS while public mechanism methods accept RPM. Input payloads expose connection flags that are not consistently used in readiness decisions. `PhoenixUtil.tryUntilOk()` does not report failure after exhausting retries. Intake configuration calls are inconsistently checked. `SparkUtil` has a shared static fault flag used during reads.

The local [Spark helper](https://github.com/Mechanical-Advantage/AdvantageKit/blob/v26.0.1/template_projects/sources/spark_swerve/src/main/java/frc/robot/util/SparkUtil.java) and [Phoenix helper](https://github.com/Mechanical-Advantage/AdvantageKit/blob/v26.0.1/template_projects/sources/talonfx_swerve/src/main/java/frc/robot/util/PhoenixUtil.java) match AdvantageKit `v26.0.1`. Hood and turret each reset the REV scratch flag at the start of `updateInputs()` and consume it before returning. Their sequential main-thread calls do not demonstrate cross-device fault leakage. Clearer status handling is a maintainability/diagnostics recommendation, not a correction to a known upstream defect.

**Recommended action:** name interfaces by physical quantity: `setVoltage`, `setDutyCycle`, `setVelocityRpm`, or `setVelocityRadiansPerSecond`. Select one unit at each boundary and convert once. Use WPILib typed quantities at important boundaries where they improve clarity; explicit suffixes are sufficient for many scalar fields and logged payloads.

Make configuration retries return success/failure or a status and raise a useful alert naming the device/bus when exhausted. Keep connection, configuration, and measurement-validity conditions distinct. Define each mechanism's response to stale/disconnected feedback; do not assume every motor should receive the same response. Keep failed readings from making “at goal” appear valid.

Either retain the documented reset/read/consume discipline or use per-device/per-refresh status if it makes the code easier to maintain. Revisit the shared flag before adding concurrent readers; the present helper does not establish thread-safe use. Preserve simple no-op IO for replay, but distinguish deliberately absent hardware from a supposedly real adapter with empty methods.

#### Acceptance M3

**Environment/prerequisites:** desktop IO diagnostic, then ROBOT adapter checks; H8 unit/mode work for simulator comparisons.

1. Prepare an IO contract table: method, unit/range, sensor frame, stop semantics, invalid-input behavior. Trace one known value from caller through real and simulated adapters; compare conversion results.
2. Stub configuration calls to fail for all attempts, then fail briefly before succeeding. Count attempts and inspect returned status/alert. Exhaustion must identify the device/bus and must not look like success; recovery must follow the declared clearing policy.
3. Inject disconnected/stale readings for one device while another remains healthy. Confirm validity/readiness changes only as intended. If the shared REV flag is retained, verify reset/read/consume ordering for each refresh and confirm one device's failed read does not mislabel the next healthy device. Replacing the helper is not required to pass this check.
4. On real hardware, verify configured values/status reports and a constrained command/stop response. Do not intentionally damage configuration or disconnect energized wiring to create a fault.

**Pass/evidence:** contract table agrees with all adapters; failure categories are distinguishable and bounded retries behave correctly. Save diagnostic results and hardware trace; mark physical confirmation pending when unavailable.

Sources: [ModuleIO.java](../src/main/java/frc/robot/subsystems/drive/ModuleIO.java), [FlywheelIO.java](../src/main/java/frc/robot/subsystems/shooter/flywheel/FlywheelIO.java), [PhoenixUtil.java](../src/main/java/frc/robot/util/PhoenixUtil.java), [SparkUtil.java](../src/main/java/frc/robot/util/SparkUtil.java).

### M4. Restore persistent logging and improve diagnostics

The real-mode `WPILOGWriter` in [Robot.java](../src/main/java/frc/robot/Robot.java) is commented out; live NetworkTables publication alone does not create a persistent replay file. Make reliable recording central to our manual verification process. Verify file creation and retrieval rather than assuming a USB device is sufficient.

Record requested target, applied/clamped target, actual value, control mode, readiness reason, and connection/reference validity for important mechanisms. Add command start/end/interruption diagnostics and loop timing so control conflicts and overruns can be diagnosed. Keep build metadata, which already identifies the code revision.

Standardize log grouping and units prospectively, with documented changes to existing dashboard layouts. Fix misleading diagnostics such as `Turret Target Distance`, which computes `deltaField.getDistance(target)` rather than the target-vector length, and the gyro fallback alert that describes disabled code. Use distinct names for commanded chassis speed and measured chassis speed.

#### Acceptance M4

**Environment/prerequisites:** desktop logging checks and ROBOT for the selected real recording location. Full H8 replay support is not required to open and inspect a log.

1. Record a short run containing idle, one goal change, a deliberate command interruption, and a documented not-ready condition. Capture build revision, mode, input, requested/applied target, measurement, control mode, and readiness reason.
2. End the run normally, retrieve the file, and open it in the log viewer with the robot disconnected. Find each event by timestamp and explain the outputs from saved data alone.
3. Check a later run creates a distinct retrievable recording. In desktop configuration, exercise an unavailable recording destination and verify the documented alert/fallback rather than silently implying recording succeeded.
4. If replay is supported, use this file in H8. Otherwise label this acceptance as persistent recording/viewing, not replay validation.

**Pass/evidence:** saved data survives the run and explains the chosen events without temporary prints; metadata identifies the exact code. Retain log filenames, timestamps, and retrieval instructions.

### M5. Integrate only supported autonomous assets

AutoBuilder and chooser setup are disabled. Assets reference lowercase `shoot`, `intake`, and `climb`, whereas the uncalled registration method uses `Shoot`, `Index`, `Intake`, `DeployIntake`, and `RetractIntake`. No climber is constructed. Thirteen asset filenames have leading or trailing whitespace in the stem. These are integration and readability issues even though all referenced path files exist.

Keep 2026 routes with the 2026 application. For any retained tooling, use a small supported chooser with a defined default, explicit start pose and completion behavior, and exact named-command contracts. Add a lightweight asset-validation task that parses JSON and checks path names, registered action names, and duplicate/whitespace names. That is configuration validation, not a new JUnit suite.

Rename assets through the editor or update all references together. Document abbreviations such as `LT`, `RT`, and `SUTO`; use clear purpose-based names. Decide how removed deployment files are cleaned up because current deployment deliberately preserves old files on the robot.

#### Acceptance M5

**Environment/prerequisites:** desktop asset validator, corrected SIM, then ROBOT for exposed routes. H4 owns autonomous mode transitions; H2/H3 and the selected path model must pass before driving routes.

1. Run the implemented asset-validation task over the exact supported deployment set. Record its command/name in the verification record. Check referenced paths and case-sensitive named commands against actual registrations, not a separately stale list.
2. In temporary asset copies, introduce a missing path, unknown command, and whitespace/duplicate-name case. Each must fail with an actionable message. Remove the probes and rerun successfully.
3. Load every chooser-exposed auto, including the default/no-selection case. In SIM check start pose, command sequence, timeout/completion, interruption, and neutral outputs afterward.
4. On the robot, verify each retained route against the intended field/start position and measured tracking limits. Confirm removed/archived routes are absent from the chooser and follow the declared deployed-file cleanup policy.

**Pass/evidence:** validation rejects bad references, all exposed routines load and terminate as specified, and physical routes meet selected criteria. Preserve validator output and a per-auto result table; unsupported historical routes are explicitly excluded.

Sources: [RobotContainer.java](../src/main/java/frc/robot/RobotContainer.java), [PathPlanner assets](../src/main/deploy/pathplanner), [build.gradle](../build.gradle).

### M6. Simplify build, deployment, and IDE configuration

The deployment task runs `git add -A` and makes a commit on branches starting with `event` when a deploy task is requested. This follows the AdvantageKit template and can preserve the exact working changes used at an event. It can also include unrelated edits. Decide whether to retain it with an explicit event-branch workflow or move it to a separate opt-in task. In either case, make the staging behavior clear to the person deploying and preserve build revision metadata. This is a workflow tradeoff, not an FRC rule. See the [upstream comparison](ARCHITECTURE_REVIEW.md#recognize-the-upstream-starting-point).

The CI container is tagged `2024-22.04` while GradleRIO is 2026.2.1; the workflow now explicitly installs Java 17. Confirm compatibility on a clean environment. WPILib's current example uses the 2025 image for 2026, so a year mismatch alone is not proof of failure; that guidance does not verify our 2024 image. Select a supported toolchain during migration. Pin analyzer versions, formatter versions, wrapper version, and relevant tool runtimes. [WPILib CI guidance](https://docs.wpilib.org/en/stable/docs/software/advanced-gradlerio/robot-code-ci.html).

The editor configuration selects Spotless globally but overrides Java formatting with `redhat.java`; align the Java formatting path with the build. It also disables Gradle annotation processing import and requests a language-server maximum heap of `64G`. Confirm generated AdvantageKit symbols resolve in the intended WPILib editor setup, and replace that machine-sized heap override with a documented portable setting or remove it. Do not change the robot JVM heap based on this unrelated editor setting.

Spotless now uses scoped source/configuration/documentation targets, excluding build output and root simulator state. It intentionally includes `vendordeps/**/*.json` and team source, including copied helpers unless specifically excluded; do not describe all external files as exempt. Review further exclusions only where ownership warrants them. Recursive includes plus explicit exclusions are also valid; changing that strategy was a team choice. Keep optional checks out of a deployment path where their runtime cost would hinder field work, while requiring successful CI checks before accepting reusable-code changes.

#### Acceptance M6

**Environment/prerequisites:** a clean desktop checkout/CI and a scheduled ROBOT deployment check. The explicit formatter task wiring is already present; use H10's remaining checker configuration when verifying naming/editor integration.

1. Follow setup instructions in a clean environment using documented JDK/tool versions. Import, generate code, format a sample, check, and build. Confirm generated symbols resolve without undocumented local fixes and editor formatting matches the build formatter.
2. Record branch, commit, index, and working-tree status before/after a build. Inspect deployment task wiring for Git mutations. In an isolated checkout, exercise the selected event commit policy: either a separate opt-in helper or the documented automatic behavior on an `event` branch. Include an unrelated temporary file to make the scope of `git add -A` visible, then discard the isolated diagnostic checkout through the team's normal process.
3. During an authorized normal deployment, compare Git state before/after. For an explicit-helper policy, deployment must not stage or commit work. If automatic event commits are retained, confirm they happen only under the documented condition and include exactly the working changes reviewed before deployment. Record all-worktree staging as an accepted tradeoff; a dry-run alone does not verify task actions.
4. Inspect build/task timing and tool reports on the intended team laptop and CI. Confirm root simulator state/build output is outside formatter targets and generated/imported-code exclusions match the documented policy. Vendor JSON remains intentionally included.

**Pass/evidence:** reproducible setup/build, matching formatting, and Git behavior that matches the adopted policy. Keep environment versions, outputs, Git-state comparison, and measured task durations; deployment confirmation remains pending until exercised.

Sources: [build.gradle](../build.gradle), [CI workflow](../.github/workflows/build.yml), [editor settings](../.vscode/settings.json).

### M7. Remove unfinished and obsolete code from the reusable core

Examples include the unused `Guts` subsystem with CAN ID `-1`, empty real IO methods in `GutsIOTalonFX`, the uncalled single-controller binding method, empty `ZoneControls`, unused dual-shooter records, uncalled helper APIs, and commented-out implementation blocks. `intakeSignificantlyFaster()` currently uses the same speed as `intake()`.

Use Git history or a clearly identified reference directory/document for experiments. Keep active code focused on supported behavior. Retain alternate hardware adapters only if they have a plausible future use and a named maintainer; otherwise they increase apparent capability and yearly upgrade work without helping the robot.

Remove unused dependencies only after checking imports and retained adapters. For example, PhotonVision and Studica are referenced by alternative implementations even though those devices are not constructed. Preserve third-party attribution and license headers when moving copied code. A formatter or naming pass should not erase provenance.

#### Acceptance M7

**Environment/prerequisites:** desktop; H9 reuse inventory reviewed.

1. For each removed/archived item, record its destination or retained Git revision, callers/imports checked, and any dependency affected. Preserve attribution.
2. Search active source and construction paths for removed types, invalid placeholder IDs, and empty supposedly-real adapter methods. Review remaining no-op methods to confirm they are intentional replay/unsupported behavior.
3. Build all supported modes after dependency removal and open relevant configuration files in their normal tools. Confirm the README/coverage inventory lists only supported capabilities, with archived examples clearly identified.

**Pass/evidence:** retained code/dependencies compile and supported-mode startup remains functional; no required adapter was removed based only on lack of an active constructor. Save the disposition inventory, reviewed search results, and build/startup results.

### M8. Fix retained shared utility defects

These utilities are currently inactive or lightly connected, so they do not all merit blocking the active robot. However, a class placed in a reusable utility package should meet its advertised contract.

| Utility | Current behavior | Work required if retained |
| --- | --- | --- |
| `CachedSupplier` | Registration locks an instance while global invalidation locks the class; the static `HashSet` is not protected by one common lock. Global references also retain every instance. | Prefer explicitly owned per-cycle cached state; otherwise use one synchronization policy and lifecycle management. Do not advertise thread safety without establishing it. |
| `LoggedDashboardChooser` | A listener path calls `selectedValue.equals(...)` when selection can be null. Copying another chooser uses reflection on WPILib private fields. | Use null-safe comparison and define absent-selection behavior; construct options explicitly rather than inspecting library internals. Check whether the season's logging library already provides the needed chooser. |
| `LoggedTunableNumber` | Switching tuning from false to true after initialization can access an uninitialized `dashboardNumber`. `anyMatch` short-circuits `hasChanged`, leaving later values unexamined that cycle. | Fix tuning mode at startup or initialize safely; inspect every value before deciding whether to invoke the callback. |
| `FullSubsystem` | Global registration has no disposal/reset and makes output-stage ownership implicit. | Retain only with a clearly documented lifecycle, or let the container own the post-scheduler participants. |

#### Acceptance M8

**Environment/prerequisites:** desktop diagnostic runner/debugger; only retained utilities are in scope.

1. Chooser: exercise no options/selection, a default, switching between two values, and an absent selection with a listener registered. Verify null-safe results and callback behavior; inspect the implementation for dependence on private library fields.
2. Tunables: change two values before one polling cycle. Verify the callback sees both updated values once and the next unchanged poll does not produce a delayed duplicate. Exercise startup tuning enabled/disabled and any explicitly supported live transition.
3. Cache: call twice before invalidation and once after; count supplier evaluations. Review registry synchronization and lifecycle. If cross-thread support is claimed, exercise concurrent registration/read/invalidation and inspect locking ownership; a stress run without an exception alone does not prove thread safety.
4. Post-scheduler registry: verify one callback per retained participant and disposal/reconstruction behavior if supported. If a helper is removed, verify absence of active references instead.

**Pass/evidence:** documented scenarios work without null errors, stale updates, duplicate callbacks, or unjustified thread-safety claims. Save diagnostic inputs/results and lifecycle/locking review; mark removed helpers not applicable with the disposition reason.

Sources: [CachedSupplier.java](../src/main/java/frc/robot/util/CachedSupplier.java), [LoggedDashboardChooser.java](../src/main/java/frc/robot/util/LoggedDashboardChooser.java), [LoggedTunableNumber.java](../src/main/java/frc/robot/util/LoggedTunableNumber.java), [FullSubsystem.java](../src/main/java/frc/robot/util/FullSubsystem.java).

## Low-priority recommendations

### L1. Correct LED boundary and waveform behavior

`LedSection.ALL` ends at `length - 1`, but rendering loops use `i < end`, leaving the last configured LED untouched. Adopt an exclusive end index of `length` or an inclusive loop consistently. The unused `wave()` raises a possibly negative sine to the fractional exponent `0.4`, which can produce `NaN`; use a defined signed or nonnegative shaping function. If disabled indication should take precedence over autonomous selection, reorder that mode check.

Check every pixel on a 30-LED strip and the full waveform period. This is a good manual check; it does not require a dedicated unit-test suite.

#### Acceptance L1

**Environment:** desktop buffer inspection and, if retained, the actual strip.

1. Apply a solid pattern and inspect indices `0` through `length - 1`; all configured pixels, including the final pixel, must receive the requested value. Check a partial section leaves adjacent pixels unchanged.
2. Evaluate the waveform across a full period, including positive/negative sine regions and crossings. All computed channels must remain finite and within the LED API's expected range.
3. Select disabled/autonomous/teleop combinations and compare the visible pattern with the declared precedence.

**Pass/evidence:** correct section boundaries, no invalid numeric waveform values, and expected mode patterns. Save buffer values or strip photos plus mode settings; simulator-only evidence does not prove strip wiring.

Source: [Leds.java](../src/main/java/frc/robot/subsystems/leds/Leds.java).

### L2. Make geometric utilities and direction names unambiguous

`Direction` contains compass-style degree labels but separately maps signs into robot X/Y speeds. For example, its `EAST` angle is +90°, while its chassis motion is robot-right, or negative Y. Rename the robot-relative driving concept to `DriveDirection` with values such as `FORWARD` and `RIGHT`, or keep the separate angle convention explicitly named.

`Zone.contains(Supplier<Translation2d>)` returns a trigger rather than a boolean. Prefer a direct `contains(Translation2d)` predicate plus a separately named trigger factory. Document boundary inclusion: circles currently exclude the boundary, rectangles include it, and polygon behavior needs a policy. Correct the polygon Javadoc, which describes a different algorithm from the ray casting actually implemented.

#### Acceptance L2

**Environment:** desktop diagnostic values; no robot required for geometry.

1. Tabulate all eight direction vectors and angles in the declared frame. Check forward is +X, right is -Y, opposite pairs cancel, diagonal vector magnitude equals cardinal magnitude, and angular velocity is zero for translation directions.
2. For each retained zone shape, evaluate a clearly inside point, an outside point, and points on edges/vertices against the documented boundary policy. Include a concave polygon if supported.
3. Exercise union/intersection/difference with known points and compare the direct predicate with the trigger at the same supplied position.

**Pass/evidence:** no angle/vector naming contradiction, boundary cases are intentional, and predicate/trigger results agree. Keep the input/expected/observed table and corrected API documentation.

Sources: [Direction.java](../src/main/java/frc/robot/util/Direction.java), [Zone.java](../src/main/java/frc/robot/util/Zone.java).

### L3. Harden characterization commands before exposing them

The feedforward-fit helper can divide by a zero denominator with too few or uninformative samples. Wheel-radius characterization can divide by zero wheel movement. Neither helper should print a plausible-looking calibration value from invalid input. Add sample-count, denominator, finite-result, and minimum-motion checks; define output limits and cancellation cleanup explicitly.

These commands are not actively bound, which makes this low priority today. Treat their verification as a prerequisite if they become a calibration tool for 2027. Confirm that cancellation stops drivetrain output even when no default command immediately replaces them.

#### Acceptance L3

**Environment/prerequisites:** desktop synthetic samples, then ROBOT only if this calibration feature will be exposed; H4 stopping and H3 heading conventions established.

1. Supply zero samples, too few samples, identical velocities, zero wheel displacement, and non-finite values. Expect a clear invalid-result outcome, not an accepted `NaN`, infinity, or misleading calibration number.
2. Supply an independently constructed linear voltage/velocity dataset with known coefficients and a known angle/wheel-displacement example. Compare results within a selected numerical tolerance.
3. Cancel each calibration command before sampling, during motion, and during normal completion with no replacement default command. Capture output requests to prove cleanup stops active drive requests. Verify declared output/duration bounds.

**Pass/evidence:** invalid data is rejected, known valid data recovers expected values, and cancellation neutralizes outputs. Retain the dataset/result and cancellation trace. Physical calibration still needs measured robot validation before accepting new gains/radius.

Source: [DriveCommands.java](../src/main/java/frc/robot/commands/DriveCommands.java).

### L4. Improve comments and optimize only measured bottlenecks

Replace comments that merely narrate assignments with explanations of units, reference frames, calibration source, or non-obvious reasoning. Remove stale references to two shooters, unused chooser behavior, “don't start shooting until we're done aiming” where only flywheel readiness is checked, and the build comment claiming JUnit 4 while dependencies use Jupiter.

Convert vague `TODO: Tune` comments into tracked work with an acceptance criterion. Use brief class-level descriptions for subsystem, IO, and command ownership. Preserve useful teaching explanations in the Technical Guide so production code stays navigable.

Only optimize allocations, logging frequency, or camera flushes after recording loop duration and identifying a real bottleneck. Removing duplicate updates is justified by correctness; rewriting all collections or pooling all geometry objects without measurements is not.

#### Acceptance L4

**Environment:** source review; representative desktop/robot logs only if performance changes are proposed.

1. Have a reader trace one binding-to-IO path using the updated comments and guide. Check descriptions against actual units, frames, readiness conditions, and active call sites. Link remaining tuning work to a measurable completion criterion.
2. For comment-only edits, confirm the diff changes no executable logic or constants and run formatting checks.
3. For an optimization, record loop-time distribution and overruns before/after under comparable commands, logging settings, and environment. Repeat the affected behavior checks; compare against a preselected performance target rather than one favorable timing sample.

**Pass/evidence:** explanations match the source; any optimization has measured benefit without failed behavior acceptance. Save reader-review notes and, when applicable, comparable traces. Performance work is not required merely to close comment cleanup.

## Java naming standard and concrete renames

### Team naming policy

Adopt the following Java conventions for team-maintained code, using the google-java-format formatter already configured. Java itself allows other styles; these are the recommended team rules. Upper camel case for types, lower camel case for methods/variables, upper snake case for constants, and treating acronyms as words provide a consistent policy. The [Google Java Style Guide](https://google.github.io/styleguide/javaguide.html#s5-naming) is a useful reference for those conventions.

| Element | Team policy | Example |
| --- | --- | --- |
| Package | Lowercase | `frc.robot.subsystems.drive` |
| Class, interface, record, enum type | `UpperCamelCase`; no `I` interface prefix | `SwerveModule`, `TurretIo`, `ShotSolution` |
| Method, field, parameter, local | `lowerCamelCase` | `setVelocityRpm`, `robotRelativeSpeeds` |
| True constant / enum value | `UPPER_SNAKE_CASE` | `LOOP_PERIOD_SECONDS`, `CLOSED_LOOP` |
| Mutable configuration/state | `lowerCamelCase`; preferably privately owned | `tuningEnabled`, `robotConfig` |
| Boolean | Name a condition or capability | `isReadyToShoot`, `isConnected`, `hasValidReference` |
| Scalar physical quantity | Include unit when not evident from its type | `positionRadians`, `velocityRpm`, `timestampSeconds` |
| Coordinate-dependent value | Include frame where ambiguous | `fieldRelativeSpeeds`, `robotRelativeTargetAngle` |
| Command factory | Action name, optionally ending in `Command` | `trackTargetCommand`, `resetHeadingCommand` |
| Direct mutation | `set`, `reset`, `apply`, or `stop`, with explicit quantity | `setDutyCycle`, `resetEncoderPosition` |

This distinction concerns immutability, not just spelling. `static final` prevents reference reassignment; it does not make a mutable controller, array, map, or vendor configuration immutable. Prefer immutable configuration values and locally constructed mutable vendor objects. Use narrowly documented naming exceptions for retained static final mutable services rather than pretending they are immutable constants.

The `k` prefix is used in [WPILib examples](https://docs.wpilib.org/en/stable/docs/software/commandbased/command-compositions.html) and is not an architectural defect. Moving our constants to `UPPER_SNAKE_CASE` is a team consistency decision. Likewise, `Io` versus `IO` is a naming policy, not a requirement for correct AdvantageKit operation. Keep those distinctions clear when reviewing student code.

Do not rename externally owned symbols such as `Rotation2d.kZero`, `ControlType.kPosition`, `TalonFX`, or vendor methods `withKP()`. Those names belong to dependencies. Mathematical gain names also deserve interpretation: `kP` in a formula is a conventional coefficient, not necessarily a constant-prefix convention. Team-owned public configuration can use `proportionalGain` or a clearly scoped constant name instead.

### Constants and variables

| Current | Proposed | Reason |
| --- | --- | --- |
| `kLoopPeriodSeconds` | `LOOP_PERIOD_SECONDS` | True constant |
| `kSimMode` | `DESKTOP_MODE` | It can select simulation or replay |
| `kCurrentMode` | `RUNTIME_MODE` | Fixed runtime selection |
| `kDriverControllerPort` | `DRIVER_CONTROLLER_PORT` | Conventional constant |
| `kOperatorControllerPort` | `OPERATOR_CONTROLLER_PORT` | Conventional constant |
| `kTuningMode` | `tuningEnabled`, or `TUNING_ENABLED` if fixed | Choose mutability deliberately |
| `kDisableHAL` | `halDisabled`, or remove through explicit configuration | Mutable flag is not a constant |
| `kRobotConfig` | Remove if unused; otherwise `robotConfig` in its owner | Avoid nullable global configuration |
| `DeviceIDs` | `DeviceIds` | Acronym treated as a word |
| `kIndexerTounge` | `INDEXER_TONGUE_CAN_ID` | Correct spelling and identify the number's meaning |
| `toungeMotor` | `tongueMotor` | Correct spelling |
| `kIntakeDrive` | `INTAKE_ROLLER_CAN_ID` | Distinguish roller from drivetrain |
| `kGutsMotorSpeed` in indexer constants | `FEED_DUTY_CYCLE` | Describe active purpose and units |
| `kSpeedTolerance` in flywheel constants | `VELOCITY_TOLERANCE_RADIANS_PER_SECOND` | Preserve current units unless deliberately converting |
| `MIN_SHOOTING_DISTANCE` | `MIN_SHOOTING_DISTANCE_METERS` | Complete an already conventional name |
| `robotVelocity` | `robotRelativeSpeeds` | Clarify its frame and three components |
| `wheelRPM` record field | `wheelRpm` or `flywheelVelocityRpm` | Conventional acronym casing |
| `field2d` / `targetField2d` | `robotFieldDisplay` / `targetFieldDisplay` | Describe purpose |
| `io` array in `Vision` | `cameraIos` | Plural and identifies contained objects |
| `flModuleIO` etc. | `frontLeftModuleIo` etc. | Make constructor ordering easier to review |

Preserve numeric values during a pure naming pass. In particular, renaming a tolerance to “RPM” without converting it would change the intended contract even if the compiler accepts it.

### Classes, interfaces, and methods

| Current | Proposed | Notes |
| --- | --- | --- |
| `Drive` | `Drivetrain` or `SwerveDrive` | Select one project-wide term; optional clarity improvement |
| `Module` | `SwerveModule` | Identifies the physical component and avoids confusion with Java modules |
| `RobotState` | `RobotStateEstimator` after extracting target selection | Distinguishes it from WPILib's `RobotState` |
| `DriverControls` | `ControllerBindings` | It configures both driver and operator |
| `DefaultControls` | `DefaultCommands` | It installs subsystem defaults |
| `Configurable` | Remove if unnecessary, or `BindingConfigurator` | Current name is very broad for one method |
| `FullSubsystem` | `PostSchedulerSubsystem` if retained | Name its special lifecycle responsibility |
| `TrajectoryCalculator` | `ShotCalculator` | Avoid confusion with drivetrain trajectories |
| `ShooterCommand` record | `ShotSolution` | This is data, not a WPILib command |
| `TrajectoryParams` | `ShotCalibrationPoint` | Describes a table entry |
| `TurretIO` | `TurretIo` | Team-owned acronym casing |
| `TurretIOSparkMax` | `SparkMaxTurretIo` | Implementation technology plus implemented role |
| `FlywheelIOTalonFX` | `TalonFxFlywheelIo` | Underlying imported `TalonFX` keeps its vendor name |
| `CameraIOLimelight` | `LimelightCameraIo` | Same implementation naming pattern |
| `trackAndShootAtTargetFullRealCommandLatestGoodUseThisOne` | `trackTargetCommand` | Removes edit history and avoids claiming that it feeds balls |
| `shootAtTargetNoRotation` | `prepareShotWithoutTurretCommand` | It adjusts hood/flywheel; it does not feed |
| `setFlywheelVelocity` returning `Command` | `runFlywheelAtRpmCommand` | Clearly a command factory |
| `zeroYaw` returning `Command` | `resetHeadingCommand` | Separate from the direct reset operation |
| `Turret.zero` | `resetEncoderPosition` | Does not physically home the mechanism |
| `Hood.down` | `stowCommand` | Action-oriented command name |
| `Indexer.index` / `indexReverse` | `feedCommand` / `reverseFeedCommand` | Express the operation |
| `Intake.deployOpenLoop` | `deployCommand` with documented control mode | Keep IO details out of operator-level names |
| `intakeSignificantlyFaster` | Remove or give it a measurable distinct purpose | Currently duplicates normal intake |
| `getPosition` / `getVelocity` returning ambiguous doubles | `getPositionRadians` / `getVelocityRadiansPerSecond` | Make units visible |
| `readyToShoot` | `isReadyToShoot` | Boolean predicate |
| `calculateRPM` | `calculateFlywheelRpm`, or remove when consolidating | Prefer one canonical shot solution |

Names such as `Drive` are not Java violations; their renames are recommendations for clarity. Apply semantic changes only where they improve understanding. Do not add a `Subsystem` suffix to every class automatically or rename framework callbacks such as `periodic()`.

For IO renames, update the annotated input type and references to generated `*AutoLogged` classes, then regenerate them with a build. Verify the actual annotation-processor output rather than editing generated Java. Preserve known log keys deliberately.

### Generated and imported code

Adopt the standard across all maintained team-owned code, with explicit boundaries for external code:

- Exclude generated `BuildConstants.java` and generated `*AutoLogged` output from hand edits and team style rules.
- Keep original vendor helper APIs and attribution, such as `LimelightHelpers`, isolated and excluded narrowly if retained.
- The generated CTRE constants are currently embedded inside `DriveConstants.java`. Extract the regenerated block to a distinct generated/provenance-marked file and map it through team-owned configuration. Do not exempt all of `DriveConstants` or all drive code merely to avoid fixing team-owned names.
- If the team intentionally takes ownership of a generated block, rename it once and record that later regeneration requires a reviewed migration. Otherwise regeneration will undo the naming pass.

## Formatting and static-analysis tooling

### Recommended minimal stack

| Tool | Recommendation | Responsibility |
| --- | --- | --- |
| Spotless + google-java-format | Keep the explicit apply/check workflow and scoped targets already implemented | Formatting, import cleanup, whitespace |
| Checkstyle through Gradle | Add as the naming/structure gate | Naming, imports, braces, basic source conventions |
| SpotBugs through Gradle | Add later, selectively, if findings are useful | Bytecode patterns associated with bugs |
| Compiler warnings | Review useful warnings and resolve team-code findings | Compile-time issues and migration warnings |
| PMD, Error Prone, SonarQube, coverage tooling | Defer initially | Avoid multiple overlapping policy systems and setup burden |

Spotless separates checking from applying formatting and normally attaches its check to Gradle's verification lifecycle. It does not rename Java APIs for you. Keep the current formatter initially to limit unrelated churn; upgrade it separately with a reviewed diff. See the [Spotless Gradle documentation](https://github.com/diffplug/spotless/tree/main/plugin-gradle).

Checkstyle has a built-in Gradle plugin and can participate in `check`. It is a source-rule checker, not a proof that robotics logic is correct. Configure reports with actionable file/line locations and make real violations fail CI. See the [Gradle Checkstyle plugin documentation](https://docs.gradle.org/current/userguide/checkstyle_plugin.html).

SpotBugs analyzes compiled classes and has Gradle integration. It may identify null handling and other bug patterns, but it will not establish correct motor units, heading conventions, or a valid shot. Start with a small reviewed set of high-confidence findings; exclude generated and external classes. See the [SpotBugs Gradle documentation](https://spotbugs.readthedocs.io/en/stable/gradle.html) and [plugin compatibility guidance](https://github.com/spotbugs/spotbugs-gradle-plugin).

### Focused initial Checkstyle rules

Use a checked-in `config/checkstyle/checkstyle.xml` and an explicit, narrow suppression file. Review this initial rule set during Phase 2 and record the selected configuration before the naming migration.

| Rule family | Initial checks | Team decision |
| --- | --- | --- |
| Type/package names | `TypeName`, `PackageName` | Conventional type casing and lowercase packages |
| Member/static names | `MemberName`, `StaticVariableName` | Lower camel case; reject the `k` constant-prefix pattern for team configuration |
| Constants | `ConstantName` | Upper snake case with narrow documented mutable-service exceptions |
| Methods and variables | `MethodName`, `ParameterName`, `LocalVariableName`, `RecordComponentName` | Lower camel case, including record components |
| Acronyms | `AbbreviationAsWordInName` | Treat acronyms as words in team declarations; respect overrides and external boundaries |
| Imports | `AvoidStarImport`, `UnusedImports` | Explicit imports by default; decide whether to allow the WPILib units exception below; formatter owns ordering |
| Simple structure | `NeedBraces`, `EmptyStatement`, `ModifierOrder` | Prevent avoidable ambiguity and stray statements |
| Utility design | `HideUtilityClassConstructor` | Private constructor on static-only utility holders |

Available checks are documented in the [Checkstyle check catalog](https://checkstyle.org/checks/). Configure an acronym rule deliberately: ordinary `TypeName` matching alone will still accept many all-capital acronym runs. See [AbbreviationAsWordInName](https://checkstyle.org/checks/naming/abbreviationaswordinname.html).

WPILib specifically recommends `import static edu.wpi.first.units.Units.*;` for its units library. An explicit-import policy is a valid team preference, but a blanket wildcard ban should not be described as WPILib guidance. My recommendation is to permit this one static-import exception and enforce explicit imports elsewhere. Record the decision in Checkstyle and demonstrate both an allowed units import and a rejected unrelated wildcard in H10's negative/positive checks. [WPILib Java units](https://docs.wpilib.org/en/stable/docs/software/basic-programming/java-units.html).

A lower-camel-case regex alone still accepts `kLoopPeriodSeconds`. For fields that must reject that prefix, a proposed pattern is `^(?!k[A-Z])[a-z][a-zA-Z0-9]*$`. Apply it only to the intended team-owned field categories, not library calls or mathematical notation everywhere. The ordinary constant-name rule catches `k`-prefixed `static final` numeric constants by requiring upper snake case.

Checkstyle's `ConstantName` classifies fields primarily by modifiers, including `static final`, rather than proving deep immutability. Reconcile it with the team's mutable-service policy using narrow exceptions or moving services to instance ownership. Do not make all fields public or suppress all constant checking to quiet the rule. See [ConstantName](https://checkstyle.org/checks/naming/constantname.html).

Avoid enforcing two competing indentation/line-wrap policies: let Spotless own exact formatting. Initially defer strict method-length limits, complexity thresholds, and Javadoc requirements on every trivial method. These generate noise in command compositions and IO payloads without addressing the observed problems. Likewise, do not ban public fields indiscriminately: AdvantageKit input snapshots are intentionally simple mutable data carriers.

### Version and Java-runtime compatibility

The robot project currently targets Java 17 and pins Gradle 8.11. The JDK used to run a checker and the bytecode target of the robot are separate concerns. Do not upgrade the robot language level just to run a style tool.

At review time, Checkstyle documents Java 17+ for its 11.x/12.x releases and Java 21+ for 13.x/14.x. Pin a compatible reviewed release, or configure a separate checker launcher if the team accepts another tool JDK. Recheck compatibility with the eventual 2027 toolchain. See [Checkstyle runtime requirements](https://checkstyle.org/#JRE_and_JDK). No exact analyzer version is endorsed here as tested against this project.

### Proposed task workflow

The compile-time `spotlessApply` dependency has already been removed. Keep normal generation tasks needed for compilation. Today's workflow is explicit `spotlessApply` when needed, then `spotlessCheck` and `build`. **After Checkstyle is installed** and versions/exclusions are verified, add naming checks as follows:

```bash
# Explicit formatting operation; review its diff.
./gradlew spotlessApply

# Verify formatting and naming in the current working tree.
./gradlew spotlessCheck checkstyleMain

# Compile/package and run the configured verification lifecycle.
./gradlew build
```

In CI, perform checks without `spotlessApply` and retain reports on failure. If SpotBugs is later added, include its chosen source-set tasks in `check`. Confirm the actual Gradle task graph so compilation does not silently repair formatting before a check. A final `git diff --exit-code` can confirm that tracked inputs were not modified, but is not a substitute for the check tasks.

Roll out rules in two steps: inventory existing violations, then fix and enforce on the documented source set. Keep suppressions specific, justified, and reviewed. Make editor diagnostics use the same checked-in configuration; do not make an IDE extension the only source of enforcement.

Remaining tooling work adds Checkstyle configuration and integrates its reports/checks with Gradle, CI, and the editor as needed. Assign that work through Phase 2. Preserve the existing explicit formatting, scoped targets, Java 17 setup, and local hook rather than treating those changes as pending.

## Verification without a unit-test program

I do not recommend making a large JUnit suite, coverage percentage, mock hardware framework, or tests for getters/setters a requirement for this team. For this preseason effort, require repeatable acceptance checks and saved results for the work being changed.

### Minimum acceptance process

| Change type | Verification | Evidence to keep |
| --- | --- | --- |
| Naming-only refactor | Compile, formatting/naming checks, inspect diff for numeric/string changes | Clean checks and focused diff |
| IO/control-loop change | Known command, measured response, stop/restart, one-update counter | Short telemetry capture |
| Controller binding | Press/hold/release and overlap/mode transitions | Completed control checklist |
| Heading/odometry | Four orientations, reset, translation, rotation, square drive | Pose/gyro/module log and measured references |
| Mechanism bounds | Referencing, near-limit behavior, invalid target, recovery | Recorded setpoint/actual/output traces |
| Shooter | Stationary calibration points, new goal, RPM dip, cancellation, invalid input | Shot observations and readiness log |
| Vision | Tag visibility changes, bad/outdated observations, disconnection | Accepted/rejected measurements and estimator trace |
| Runtime mode | Startup and stop behavior in each supported mode | Mode-specific smoke-check record |
| Autonomous assets | JSON/reference validation plus controlled execution of exposed autos | Validation report and execution notes |
| Tooling | Known deliberate violations fail; normal code passes | CI result with reports |

Run simulator checks after correcting simulator timing and unit defects. Until then, a simulation pass can be misleading. Hardware checks should use the team's normal controlled bring-up procedure, with one mechanism or behavior changed at a time.

Persistent logs are particularly valuable in this workflow, because they make a field observation inspectable after the robot is unavailable. Manual checks must record what was exercised and what passed; “it seemed fine” is hard to reuse next season.

### Small calculation checks worth keeping

None is a reason to block the initial naming/tooling work. If the team retains and substantially rewrites advanced aiming or geometry, a tiny set of pure calculations has unusually high value:

1. **Unit boundaries:** a known 2400 RPM request corresponds to 40 RPS and about 251.33 rad/s. The current simulator contains exactly this class of mismatch.
2. **Limited-travel turret target selection:** representative targets around angle wrapping and both mechanical bounds. These cases are easy to miss by pointing at one target.
3. **Field/alliance transforms and shot interpolation:** known poses, a repeated alliance transform returning the original pose, interpolation midpoint, and out-of-range behavior.
4. **Camera parser input validation:** empty, short, malformed, non-finite, and valid payloads. These cases are awkward to reproduce reliably with a physical camera.

These can be small deterministic checks with no motors, HAL initialization, or vendor mocks. The mentor and student lead should select a practical way to retain these cases, either as small automated checks or as documented diagnostic procedures. Keep bounded-turret and unit-conversion cases whenever those algorithms remain in our reusable code. Static style tools cannot verify those behaviors.

## Delivery plan

Use the following work packages to assign the preseason effort and review progress. Priority remains unchanged: some medium-priority work, such as IO diagnostics, happens early because it helps verify high-priority fixes. At kickoff, the student lead and mentor should set dates from team availability, robot access, and toolchain availability. Review blocked work at each programming meeting.

**Status at the reviewed baseline:** P2.1 is partially implemented: explicit formatting, scoped targets, formatter pinning, and the CI/hook changes are present. Naming rules, checker installation, migration, and broader M6 decisions remain open. Other implementation packages have no recorded completion here; owner and reviewer are **unassigned**. The package owner supplies the implementation and results; the assigned reviewer records acceptance. A software package may be merged with hardware verification pending, but the affected capability must not be labelled robot-ready or enabled as an accepted feature until its required checks pass.

### Phase overview and dependencies

| Phase | Deliverable | Depends on | Robot access | Exit checkpoint |
| --- | --- | --- | --- | --- |
| 1. Baseline and scope | Reuse inventory, configuration provenance, evidence template, known baseline | None | Helpful for baseline; absence must be recorded | Mentor and student lead record scope and measurement needs |
| 2. Coding-standard tooling | Explicit formatter, focused checker, scoped exclusions, CI reports | Phase 1 ownership boundaries | None | Positive/negative tooling checks work; existing naming debt inventoried |
| 3. Runtime foundations | Single lifecycle, usable sim contracts, coherent pose/heading, command ownership, diagnostics | Phase 1 scope; current build workflow is sufficient to begin | Drive hardware for physical pose/heading gate | Core software checks pass; physical checks separately signed off |
| 4. Naming migration | Consistent maintained Java APIs and regenerated IO references | Phase 2 rules; coordinate files with runtime work | Usually none; runtime smoke checks use desktop | Full style gate passes; no unexplained behavior/string/value changes |
| 5. Mechanisms and vision | Valid references/limits, coherent shooting/readiness, robust vision | Phase 3 relevant checks | Required for retained mechanisms/cameras/calibration | Per-feature physical acceptance meets preselected criteria |
| 6. Reusable release | Season/config separation, supported modes/autos, retained utilities, reproducible release evidence | Relevant Phase 3/5 gates; supported season toolchain for 2027 release | Required for final hardware-supported release | All retained high-priority checks pass or feature is explicitly excluded |

Phases are review checkpoints, not a demand to leave people idle. While hardware verification is blocked, tooling, documentation, configuration review, parser diagnostics, and utility cleanup can proceed independently. Do not use that parallel progress to mark an unmet physical criterion passed.

**Begin with small runtime repairs.** After recording the baseline, assign separate changes for H1 duplicate callbacks, H4 missing hood requirements, H7 uncertainty forwarding, and H8 flywheel simulation units/control modes. Use the applicable acceptance steps for each change; complete the wider mode, mechanism, and localization checks as their prerequisites become available. None of these repairs depends on renaming constants, changing `IO` casing, or installing Checkstyle. Tooling and naming can proceed alongside them with coordinated file ownership.

### Phase 1 — establish the baseline and reuse scope

| Package | Deliverable and work | Recommendations / acceptance |
| --- | --- | --- |
| P1.1 | Record the baseline revision and working changes; preserve the 2026 application; inventory reusable, robot-specific, season-specific, and excluded code | H9; [inventory/provenance steps](#acceptance-h9) |
| P1.2 | Define supported modes/mechanisms, current defects, verification environments, evidence locations, and required hardware access | H8; [coverage matrix](#acceptance-h8), [evidence protocol](#how-to-run-and-record-acceptance-checks) |
| P1.3 | Record current configuration sources and collect available baseline telemetry; assign who will choose physical limits and reference procedures | H5/H6/M4; [reference criteria](#acceptance-h5), [shot criteria](#acceptance-h6), [logging procedure](#acceptance-m4) |

Record known failures in the baseline so the team can demonstrate what improves after a fix. If persistent logging is unavailable, preserve a documented live capture/screenshot or mark the baseline capture pending; M4 implementation follows in Phase 3. Do not change gains while collecting the baseline solely to make it appear healthy.

**Exit:** a reviewer can identify what will be reused, what will be excluded, what is currently unverified, and which measurements require the robot. Owners of physical criteria are assigned even if values must await measurements. No source behavior changes are required to complete this phase.

### Phase 2 — introduce coding-standard tooling

| Package | Deliverable and work | Recommendations / acceptance |
| --- | --- | --- |
| P2.1 | Preserve completed formatting/CI work; select naming/import rules and generated/vendor boundaries; pin compatible checker/runtime versions | H10/M6; partial implementation recorded above; [tooling checks](#acceptance-h10), [workflow checks](#acceptance-m6) |
| P2.2 | Add Checkstyle rules/reports and CI/editor integration; demonstrate valid and deliberately invalid examples; inventory existing violations | H10; [negative/positive probes](#acceptance-h10) |

Keep tool installation and generated-code boundary changes separate from broad symbol renames. Existing naming violations are expected at this stage: make them visible with an explicitly temporary migration approach, then enforce the complete maintained source set at Phase 4. Passing tool-configuration probes is only partial H10 acceptance; a permanently warning-only checker is not the final deliverable.

**Exit:** formatter/checker responsibilities are clear, deliberately invalid examples fail for the right reason, normal verification does not modify tracked code, exclusions are reviewed, and Phase 4 has a concrete list of remaining naming work. No robot session is needed.

### Phase 3 — repair runtime foundations and verification infrastructure

| Package | Deliverable and work | Recommendations / acceptance |
| --- | --- | --- |
| P3.1 | Remove duplicate callbacks; define update/output ownership and shared loop timing | H1; [cycle-count procedure](#acceptance-h1) |
| P3.2 | Correct simulation timing/units/control modes; add known input/output capture and declared gyro simulation/fallback support | H8/M3, initial subset; [sim contracts](#acceptance-h8), [IO conversion checks](#acceptance-m3) |
| P3.3 | Implement one odometry/velocity path and deliberate heading/reset semantics; handle queue/lock lifecycle if retained | H2/H3; [sample pipeline](#acceptance-h2), [heading matrix](#acceptance-h3) |
| P3.4 | Correct requirements, mode gating, interruption/stop behavior; expose device/configuration validity and command diagnostics | H4/M3; [control matrix](#acceptance-h4), [fault diagnostics](#acceptance-m3) |
| P3.5 | Enable reliable persistent recording, metadata, and event/readiness diagnostics; retrieve a representative run | M4; [recording acceptance](#acceptance-m4) |

Start with P3.1, then P3.2's basic model/output contracts. Develop P3.3's heading and odometry changes together as needed: controlled samples establish ordering/frames before joint physical drive checks. A kinematic gyro fallback may require the H2 and H8 changes in the same reviewed package. Avoid circular acceptance by distinguishing these software prerequisites from the final integrated motion checks.

P3.4's small hood-requirement repair can start alongside P3.1; use the lifecycle fix before its integrated acceptance. The broader ownership work can use output-capturing IO before physical mechanisms are cleared in Phase 5. Its physical mechanism rows remain pending until referencing/limits are verified; drivetrain checks use the team's existing controlled drive bring-up. P3.5 should be ready before Phase 5 calibration sessions so evidence is saved. Use early live captures for P3.1–P3.4 if persistent recording is still being completed.

**Exit:** H1 and the software parts of H2/H3/H4/H8/M3 pass on a named revision. Perform the physical drive/heading checks and record them separately; if unavailable, the core can be marked software-verified but not hardware-verified. No mechanism may inherit physical approval merely because its command scheduling passed with a stub.

### Phase 4 — migrate names without changing behavior

| Package | Deliverable and work | Recommendations / acceptance |
| --- | --- | --- |
| P4.1 | Rename constants/mutable configuration and ambiguous scalar fields while preserving values and units | M1; [rename checks](#acceptance-m1) |
| P4.2 | Rename command factories/direct operations and coordinator/control classes; migrate IO types and regenerate generated references | M1; [symbol/IO checks](#acceptance-m1) |
| P4.3 | Remove temporary naming migration exceptions, align documentation, and enable the full team style gate | H10/L4; [final tooling acceptance](#acceptance-h10), [comment review](#acceptance-l4) |

Use a small series of changes grouped by subsystem or API family. Prioritize misleading action/unit names over cosmetic casing. Coordinate with runtime fixes before renaming their shared APIs. Treat telemetry keys, camera names, serialized data, and named auto actions as separate contracts: preserve them in a Java-only rename. If a behavior defect is encountered, fix it in a separate focused change; it does not have to wait for the naming migration to finish.

**Exit:** clean build and full maintained-source style check; generated symbols resolve; no unexplained numeric, unit, sign, string-contract, requirement, or control-flow change. Naming acceptance does not establish runtime correctness; keep the separate runtime evidence.

### Phase 5 — validate retained mechanisms and vision

| Package | Deliverable and work | Recommendations / acceptance |
| --- | --- | --- |
| P5.1 | Establish references, feasible targets, boundary/recovery behavior, and relevant fault response for each retained mechanism | H5 plus physical H4/M3 rows; [boundary/reference procedure](#acceptance-h5), [control transitions](#acceptance-h4) |
| P5.2 | First repair discarded uncertainty; separately add adopted camera hardening and establish live pose quality | H7; [separate repair/hardening acceptance](#acceptance-h7) |
| P5.3 | Consolidate shot solution/readiness/feed policy; recalibrate stationary shooting, then separately validate any moving-shot support | H6; [solution and shot procedure](#acceptance-h6) |

P5.1 depends on command ownership and trustworthy IO. P5.2's uncertainty-forwarding repair and controlled estimator comparison can start during the first runtime fixes; parser hardening can also proceed independently. Live fusion acceptance depends on the Phase 3 pose pipeline. P5.3 can use a controlled known pose while vision work proceeds; camera-driven shooting acceptance requires the relevant P5.2 checks to pass. Preserve the distinction between no-ball control checks and actual fuel-shot calibration.

Schedule hardware sessions around explicit cases: referencing/limits first, control transitions second, then shot calibration. Set physical tolerances and success criteria before recording pass/fail. A blocked session should leave its cases pending and allow independent desktop work to continue. Do not choose new pass thresholds after seeing a failed result without documenting a reviewed requirement change and rerunning.

**Exit:** every retained mechanism has a reference/limit record, valid control behavior, and measured acceptance evidence. Vision meets its declared criteria. Shooting support states whether it includes stationary shots only or independently accepted moving shots. Mechanisms omitted from the new robot have documented exclusions rather than artificial completion claims.

### Phase 6 — extract and release the reusable foundation

| Package | Deliverable and work | Recommendations / acceptance |
| --- | --- | --- |
| P6.1 | Separate season/hardware configuration, clarify object ownership, and remove unsupported legacy code/dependencies | H9/M2/M7; [core extraction](#acceptance-h9), [ownership](#acceptance-m2), [disposition checks](#acceptance-m7) |
| P6.2 | Finish all advertised runtime modes, including replay if retained; validate supported autos and deployment/editor workflow | H8/M5/M6; [mode matrix](#acceptance-h8), [auto validation](#acceptance-m5), [clean workflow](#acceptance-m6) |
| P6.3 | Repair or remove retained helpers; complete optional utility/LED/calibration/comment cleanup | M8/L1–L4; [helpers](#acceptance-m8), [LEDs](#acceptance-l1), [geometry](#acceptance-l2), [characterization](#acceptance-l3), [comments/performance](#acceptance-l4) |
| P6.4 | Build with the supported season toolchain, confirm configuration provenance, and rerun affected acceptance on the release revision and actual robot | H9 and retained feature checks; [season acceptance](#acceptance-h9), [release checklist](#release-checklist) |

P6.1 inventory work starts in Phase 1; extraction uses corrected components rather than moving unreviewed code wholesale. P6.2 replay depends on a compatible saved log from P3.5. Auto loading/reference checks are independent of physical route validation; routes need accepted drive, field, and mechanism behavior. P6.3 work can proceed during hardware waits, but optional cleanup must not delay resolution of retained high-priority failures. Promote calibration-helper verification before using that helper to derive production values.

Do not wait for 2027 hardware to organize the core, but do wait for the actual supported toolchain and measurements before calling it a validated 2027 robot release. Porting to a new template or changing library versions can invalidate earlier evidence; rerun relevant lifecycle, mode, IO, and integration checks on the final combination. Results from the 2026 robot support the port but do not certify new mechanisms.

**Exit:** the [release checklist](#release-checklist) is satisfied for the declared scope. Optional deferred work remains explicitly tracked. Unsupported replay, moving shots, legacy routes, or adapters are unavailable or clearly labelled rather than implicitly trusted.

### Tracking each work package

Create an issue or checklist entry from this template when work starts; no project-management service is required.

```text
Package ID / title:
Recommendation IDs and acceptance links:
Owner: Unassigned
Reviewer/verifier: Unassigned
Status: Not started
Deliverable and explicit exclusions:
Predecessor packages / acceptance prerequisites:
Required robot/equipment access:
Implementation change/commit:
Desktop result and evidence:
Simulation/replay result and evidence, if applicable:
Hardware result and evidence, if applicable:
Open measurements, failures, or follow-up work:
Final reviewer decision and date:
Mentor hardware/release acceptance, when required:
```

Suggested statuses are `Not started`, `In progress`, `Ready for review`, `Awaiting hardware`, `Blocked`, and `Accepted`. Use `Excluded from scope` only with the feature-disposition decision. An accepted implementation review and accepted runtime behavior are different checkpoints; retain both results. Dates and time estimates belong on these entries once an owner and prerequisites are known.

### Review, regression, and rollback

Keep naming-only, tool-version, behavioral, and calibration changes separate where practical. Each change should describe its trigger/problem, resulting behavior, recommendation IDs, evidence, and remaining limits. Review the final diff and build after removing temporary probes so acceptance is associated with the version intended for use.

If a check fails, record the step, inputs, observed output, and log range. Fix the responsible package and rerun that procedure plus directly affected downstream checks: an H3 heading change can require H2 pose, H7 fusion, and H6 aiming checks; a Java-only private-field rename does not require every physical trial again. A port or configuration change needs broader relevant checks than a documentation edit.

Maintain a known accepted revision **with its matching configuration and calibration**. If a new candidate fails during bring-up, disable the affected behavior and return to that compatible revision through the team's normal review/deployment process. Do not mix older code with newly changed hardware or gains and assume the previous evidence still applies. Avoid destructive working-tree resets as an ad hoc rollback method.

### Release checklist

- [ ] Retained/excluded capabilities and modes are listed; confirmed high-priority defects are resolved and the adopted hardening/team-policy checks pass for retained features.
- [ ] The final revision builds and passes the team formatting/naming checks without modifying tracked source.
- [ ] Supported modes initialize correctly; simulation coverage and any replay limitation are explicit.
- [ ] Lifecycle, pose/heading, command transitions, and supported mechanism checks have evidence tied to compatible code/configuration.
- [ ] Physical criteria have values, measurement methods, and observed results; none is silently accepted while unset or awaiting hardware.
- [ ] IDs, buses, frames, gearing, limits, field layout, and calibration have reviewed provenance for the target robot.
- [ ] Logging files are retrievable and identify the code/configuration; named actions and exposed autonomous routines resolve.
- [ ] Temporary faults/probes are removed; diagnostic signals retained in production are intentional and documented.
- [ ] Remaining medium/low work is assigned or deliberately deferred; any defective optional feature required by the release has been promoted and verified.
- [ ] A compatible fallback revision/configuration and retrieval/deployment instructions are known.
- [ ] The operator-facing control description and Technical Guide match the released behavior.
- [ ] Core extraction acceptance and actual 2027 robot acceptance are recorded separately when the season toolchain/hardware is not yet available.
