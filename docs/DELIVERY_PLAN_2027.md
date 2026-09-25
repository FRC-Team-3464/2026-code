# 2027 robot software: mentor delivery proposal

**For the mentor group, September 2026.** This is a proposal for a small student team to prepare the reusable software before the physical robot is available. The [2027 Mentor Recommendations](REUSE_RECOMMENDATIONS_2027.md) remain the source of truth for the technical findings and acceptance procedures; this document owns schedule, staffing, and review decisions. The [Technical Guide](TECHNICAL_GUIDE.md) explains the existing code to students learning robotics.

## The decision we need from mentors

**End of November is not a credible completion date for this work.** If meetings begin the week of September 28, there are eight ordinary meeting weeks through November 20. The robot is expected only near the end of November, so physical acceptance cannot finish by then. Even the minimum software foundation below has more work than eight weeks of this team's available time. I propose a **November 20 progress and scope review**, followed by a forecast based on actual completed packages. If the first meeting starts later, move the review rather than compressing checks.

For planning, expect **roughly 15–25 active meeting weeks for the minimum software foundation**, including learning, review, and repair time. This range uses the already-recorded [Phase 1 source inventory](PHASE_1_BASELINE_2027.md) instead of asking students to repeat that inventory. Test the estimate after Weeks 2 and 5; it is not a promised completion date. School breaks, lost meetings, a difficult H2/H3 repair, or shifting students to robot bring-up extend calendar time. The full recommendation backlog, 2027 toolchain port, and physical calibration are additional work. A green Gradle build and SIM run cannot substitute for the missing robot.

| Mentor decision | Proposed answer for this block | Needed by |
| --- | --- | --- |
| Student team | Three named students; A owns lifecycle/drive/pose, B owns IO/SIM/wiring, C owns tooling/logging/evidence. Each change has a different student reviewer. | Week 1 |
| Mentor coverage | One software mentor at both session reviews, plus a mechanical/electrical contact for future hardware limits and configuration. | Week 1 |
| Supported modes | REAL and SIM. Treat REPLAY as unsupported until it has a complete reviewed implementation. | Week 1 |
| Feature scope | Retain only mechanisms and autonomous actions the team plans to operate in 2027; decide exclusions explicitly. | Week 1–2 |
| November checkpoint | Review completed packages and evidence, update the effort forecast, and decide the next funded scope. Do not label the foundation complete solely because the calendar reached November. | Week 8 |
| Physical access | Mechanical/electrical leads supply a stable robot revision, wiring map, reference procedures, and safe test limits before powered checks. | Before robot week R1 |

The three student lanes are assignments for this schedule, not permanent specialties. Rotate demos and reviews. One student owns each package through final review; the others should be able to explain it. Keep only one shared-runtime edit active at a time when `RobotContainer`, `Drive`, or shared constants are changing; independent tooling or mechanism work may proceed in parallel with named file owners.

## Capacity and session routine

Three students × two three-hour meetings equals **18 student contact-hours per week**. Reserve 15 minutes at the start of each meeting to agree on the goal, about 2 hours 15 minutes for focused work, and 30 minutes at the end for review, demonstration, and recording evidence. That gives roughly **13.5 focused team-hours per week**, or **108 focused hours over eight weeks** before absences and unfamiliar-code delays. These are estimates, not guaranteed velocity. The software mentor should plan roughly one to two additional hours a week for review/decisions, plus relevant mechanical/electrical mentor time when physical checks begin.

The following is an **initial effort estimate**, not measured team velocity. It counts focused student work for the narrow software scope, including diagnostics and code review; it excludes physical testing, full Java naming migration, H9's 2027 port, and optional features. Each row needs a demonstration, so a code edit by itself does not exhaust its estimate.

| Work group | Estimated focused student-hours | Why it may grow |
| --- | ---: | --- |
| P1 verify the recorded inventory and settle scope/owners | 2–4 | A changed construction path or unresolved mentor decisions |
| P2 checker policy and usable initial configuration | 12–20 | Generated-source exclusions and CI/editor differences |
| H1 plus narrow H4 hood and H7 uncertainty repairs | 15–25 | Building diagnostic inputs and reviewing scheduler/estimator behavior |
| P3.2 four staged REAL/SIM/REPLAY wiring commits | 30–45 | Constructor order, startup side effects, and mode smoke checks |
| P3.3 retained IO contracts and flywheel/SIM behavior | 20–35 | Ambiguous shaft units, no-op models, and control-mode transitions |
| P3.4 H2/H3 odometry, gyro, heading, and reset | 45–70 | Timestamp/queue behavior and conflicting frame assumptions |
| P3.5 retained command ownership and stop behavior | 15–25 | Operator policy decisions and multiple control modes |
| P3.6 retrievable logging and diagnostics | 12–20 | Recording destination and useful signal coverage |
| Integration, regression checks, review, and repair | 20–30 | Changes to shared loop/state code invalidate earlier observations |
| **Estimated total** | **171–269** | **Before absences and unplanned hardware/software defects** |

At 13.5 focused hours per active week, this estimate is about **13–20 weeks of work before schedule allowance**. Reserving time for absences, onboarding, and failed acceptance checks gives the **15–25 active-week planning band** above. Through November 20, the team has at most about 108 focused hours under perfect attendance: below even the optimistic 171-hour estimate. Parallel students help, but P3.2 must be reviewed before its shared construction path is used for P3.3, and H2/H3 needs trustworthy measurements. After Weeks 2 and 5, replace these estimates with the team's measured completed-package rate and remaining work.

Fifteen uninterrupted meeting weeks from September 28 would reach January 2027; school breaks and any robot sessions using the same students push the calendar later. The upper end of the band reaches into spring. Give mentors this range rather than a fabricated February promise. If physical testing starts before the software foundation is finished, reallocate student hours and publish a new forecast instead of pretending both tracks have full staffing.

In **Session 1**, the owner and reviewer identify the exact package/acceptance steps, agree on file ownership, implement one narrow change, and run the closest desktop diagnostic. In **Session 2**, finish the check, inspect the final diff, run `./gradlew spotlessCheck build`, capture observed behavior, and commit only if the package is reviewable and buildable. Keep temporary diagnostic code only if it will be maintained. A missed session moves work to the next week; it does not turn an unrun check into a pass.

Each weekly record should name the package ID, code revision, owner/reviewer, environment, inputs, expected and observed outputs, result, evidence location, and next blocker. Use the [work-package record](#tracking-each-work-package). Mark physical rows `Awaiting hardware`; mark unfinished software `In progress` or `Blocked` with a reason. The student owner demonstrates behavior, another student reviews it, and the mentor decides unresolved architecture, physical safety, and release scope.

## First eight weeks: the November checkpoint

Dates are **weeks beginning**, not assumed meeting days. The week of November 23 is a contingency/handoff week because school and holiday schedules may reduce attendance; the main **progress** review is November 20. These rows are a proposed order and an upper-bound workload, not guaranteed completions. If a gate takes longer, finish it before advancing the dependent task. Follow the linked P/H/M acceptance sections in the [phase package reference](#phase-based-work-packages) for detailed checks.

| Week | First three-hour session | Second three-hour session | Lead and review gate |
| --- | --- | --- | --- |
| **1 · Sep 28** | Kickoff. A/B compare the current source and REAL/SIM constructor order with the [Phase 1 record](PHASE_1_BASELINE_2027.md); C reviews the recorded configuration locations and open information needs. | Reuse the recorded inventory, resolve retained modes/features, name reviewers/hardware decision owners, and list any changed or missing baseline facts. | **C:** confirmed inventory, scope decision, and package list. **Gate:** mentors approve the narrow initial scope; physical rows remain open. |
| **2 · Oct 5** | A adds H1 cycle counters and removes duplicate registered-child `periodic()` calls; B maps flywheel IO units and no-op SIM devices; C drafts H10 naming/checker policy. | Demonstrate H1's one-update-per-loop counts. Record physical questions for H5/H6/H7 and measure actual package throughput against the estimate. | **A:** P3.1/H1 evidence and reviewed fix. **C:** P2.1 policy draft. **Forecast check 1:** adjust the remaining dates if H1/P1 consumed more than planned. |
| **3 · Oct 12** | A fixes the narrow H4 manual-hood requirement issue; B fixes H7's discarded uncertainty in a separate change; C sets up a focused Checkstyle draft. | Review command ownership and controlled low/high-uncertainty estimator output; run checker positive/negative probes. | **A/B:** narrow H4/H7 commits if their desktop checks pass. **C:** P2.2 initial evidence; no full naming claim. |
| **4 · Oct 19** | B implements P3.2a SIM wiring at existing constructor positions; A reviews module order; C finishes checker/CI setup or records its remaining blocker. | Run build, SIM startup, bindings, and constructor comparison before the P3.2a commit. | **B:** buildable SIM-selection commit. **Gate:** no real device constructed in SIM; no simulator-physics change hidden in the wiring diff. |
| **5 · Oct 26** | B implements P3.2b REAL wiring; A compares IDs, camera names, LEDs, and construction order with baseline; C records pending disabled robot startup. | Review desktop gates and commit P3.2b only if the graph matches. Measure P3.2 effort and update the end-to-end forecast. | **B:** buildable REAL-selection commit or a concrete discrepancy. **Forecast check 2:** extend wiring instead of borrowing time from its verification. |
| **6 · Nov 2** | A/B implement P3.2c explicit unsupported-REPLAY policy after P3.2b review; C checks that alternate startup fails clearly before bindings. | Review startup and complete a separate P3.2c commit. Prepare P3.2d's common-construction diff without merging it early. | **A:** supported-mode record; REPLAY rejection is deliberate and leaves REAL/SIM unchanged. |
| **7 · Nov 9** | B implements P3.2d common subsystem construction; A reviews the exact call order and begins an H2 timestamp/gyro map; C checks mode startup and telemetry keys. | Run final SIM startup/bindings and constructor-map comparison; commit P3.2d if the software gate passes. | **B:** P3.2 software gate; physical REAL startup remains `Awaiting hardware`. H2 work is analysis, not a claimed repair. |
| **8 · Nov 16** | Repair any P3.2 gate failures; C confirms tooling check and baseline log availability; A/B prepare P3.3 IO contracts and H2/H3 dependency notes. | Mentor review of exact accepted revisions, failed/pending rows, observed hours per package, and the next four weeks of work. | **C:** **Nov 20 progress report**, not a foundation-complete sign-off. Accept only demonstrated software rows and reforecast the remaining work. |

This is still an ambitious first block. If H1, checker setup, or wiring runs long, the Week 8 output may be fewer accepted packages. Do not combine unreviewed P3.2 commits to recover the calendar. H2/H3, complete flywheel SIM behavior, broad H4 transitions, and persistent logging are deliberately placed in the following block because they need additional diagnostic and review time.

## After November: sequence work by gates, not a guessed finish date

Once P3.2 passes its software gate, take the next packages in this order, revising the week labels after the November review:

| Next block | Two-session weekly focus | Gate before advancing |
| --- | --- | --- |
| P3.3, likely several weeks | Contract table; flywheel RPM/RPS and output-mode repair; known-output capture; only the SIM/gyro support needed for retained diagnostics | H8/M3 desktop traces demonstrate velocity, open-loop, stop, and unsupported no-op boundaries. |
| P3.4, likely several weeks | H2 measurement owner/timestamps/queues, then H3 frame/reset/alliance convention with controlled gyro/module inputs | No stale/double samples or unexplained heading transform in repeatable desktop cases. Physical accuracy remains open. |
| P3.5/P3.6 and integration | Retained H4 mode/interrupt/stop matrix, device validity, retrievable logs, and reruns of affected diagnostics | The software mentor and student reviewers can reproduce the accepted behavior on one named code/configuration revision. |

These blocks consume the remaining estimated hours; their actual week counts depend on the preceding gate. Prepare H5 mechanism-reference, H6 shot/readiness, and H7 camera/geometry procedures while hardware is absent. Their physical limits and outcomes remain `Awaiting hardware`. A focused Checkstyle configuration can start an explicit migration, but **full maintained-source naming migration (P4/H10)** remains separate; do not call H10 accepted while old team-owned code is exempt. H9 core extraction, P6.4 2027-toolchain acceptance, and optional M/L cleanup are follow-on work. Replay, moving shots, and legacy autonomous paths should be explicitly excluded unless mentors fund and verify them.

If mentors mean **all** reuse recommendations must be complete by late November, this staffing and hardware schedule cannot support that goal. Extend the date, reduce supported scope, or add sustained experienced capacity. Extra students alone do not remove the sequential P3.2 → P3.3 → H2/H3 dependencies or the need for physical access.

## Robot-access block after late November

Start **R1** only when the physical robot is available in a stable configuration; it is not automatically the week of November 30. **R1–R6 are check stages, not consecutive calendar weeks.** After disabled startup, independent mechanism and drive checks can be scheduled when their own prerequisites pass. R2 waits for P3.4's accepted drive/heading software path, mechanism movement waits for its safe reference and P3.5 control prerequisites, R5 waits for H6's coherent software shot policy, and R6 waits for accepted autonomous software/assets. The team may finish software between stages. These checks certify only the particular robot revision tested. If that is the modified 2026 robot, its results inform reuse but do not certify an eventual 2027 machine. The mechanical/electrical leads must provide the actual ID/bus map, gear ratios, inversion, reference positions, travel/current/output limits, and a safe one-mechanism-at-a-time procedure. A bench can verify only its own device/mechanism scope. Set numeric criteria, method, and repetitions before each run, then record robot and code/configuration revisions. Reopen affected physical checks after hardware changes.

| Robot week | First three-hour session | Second three-hour session | Acceptance boundary |
| --- | --- | --- | --- |
| **R1** | Reconcile REAL configuration with the built robot and start disabled. | Inspect device/camera/LED connections, firmware/configuration reports, and log retrieval. | P3.2 REAL startup and configuration only; no movement implied. |
| **R2** | At mentor-approved low output, check drive/module/gyro directions, reset, and a marked straight run. | Repeat lateral/turn cases and compare independent measurements with H2/H3 logs. | Drive pose/heading checks pass to declared tolerances or stay open with a fault record. |
| **R3** | Establish one retained mechanism's actual reference, travel direction, bounds, and safe stop/recovery. | Check bounded commands, interruption, and neutral output for that mechanism. | H5/H4/M3 physical checks for **that mechanism only**; repeat this week for others as needed. |
| **R4** | Finish remaining retained mechanism references and no-fuel readiness/interlock checks. | Verify live camera observations/pose correction against a known field reference. | Remaining H5 and H7 rows; camera or mechanism failures block their dependent features. |
| **R5** | Calibrate stationary shots and feed policy at approved settings after H5/H7 prerequisites. | Review repeatability, logs, and whether moving shots are excluded or separately verified. | Stationary H6 result only; no assumed moving-shot acceptance. |
| **R6** | Verify only supported autonomous/integrated behaviors whose drive and mechanism gates passed. | Re-run affected checks on final code/configuration and review the [release checklist](#release-checklist). | Mentor sign-off for the tested robot revision and declared scope; failing or unrun rows remain unavailable/pending. |

R3 and R4 are placeholders for **at least** the required mechanism sessions, not a guarantee that every mechanism can be cleared in two weeks. School breaks, missing field space, hardware rework, and failed checks extend this block. Do not schedule a robot-ready sign-off date before R1 confirms the machine's configuration. The team's existing 2026 code can be a software reference, but a 2027 toolchain/robot port requires a separate compatible build and affected rechecks.

## Mentor checkpoints and change control

At the end of Weeks 1, 4, 7, and 8, circulate a short status sheet: accepted revisions/packages, desktop evidence, `Awaiting hardware` rows, failures, next two sessions' owners, and decisions needed. Reforecast effort explicitly after Weeks 2 and 5 using actual package duration. The software mentor resolves scope and architecture decisions; the mechanical/electrical mentors own actual safe limits and hardware readiness. If one student misses two weeks, a dependency slips, or H2/H3 needs more work, reforecast at the next checkpoint. Do not compress acceptance to preserve the calendar.

The mentor group can approve this proposal by naming the three students, software reviewer/decision owner, hardware contact, retained features, and Week 1 meeting dates. The November checkpoint and physical acceptance are separate decisions. The detailed package catalog below remains available when mentors authorize the follow-on work.

## Phase-based work packages

The catalog below is the **full reuse backlog**, not the promise for November 20. The weekly table above identifies which portions fit the eight-week software block and which need later work or physical access. Use these package IDs to assign work and review progress. Priority remains unchanged: some medium-priority work, such as IO diagnostics, happens early because it helps verify high-priority fixes. Set follow-on dates from team availability, robot access, and toolchain availability. Review blocked work at each programming meeting.

**Status at the reviewed baseline:** P2.1 is partially implemented: explicit formatting, scoped targets, formatter pinning, and the CI/hook changes are present. Naming rules, checker installation, migration, and broader M6 decisions remain open. Other implementation packages have no recorded completion here; owner and reviewer are **unassigned**. The package owner supplies the implementation and results; the assigned reviewer records acceptance. A software package may be merged with hardware verification pending, but the affected capability must not be labelled robot-ready or enabled as an accepted feature until its required checks pass.

### Phase overview and dependencies

| Phase | Deliverable | Depends on | Robot access | Exit checkpoint |
| --- | --- | --- | --- | --- |
| 1. Baseline and scope | Reuse inventory, configuration provenance, evidence template, known baseline | None | Helpful for baseline; absence must be recorded | Mentor and student lead record scope and measurement needs |
| 2. Coding-standard tooling | Explicit formatter, focused checker, scoped exclusions, CI reports | Phase 1 ownership boundaries | None | Positive/negative tooling checks work; existing naming debt inventoried |
| 3. Runtime foundations | Single lifecycle, clear REAL/SIM wiring, usable sim contracts, coherent pose/heading, command ownership, diagnostics | Phase 1 scope; current build workflow is sufficient to begin | Disabled startup checks for wiring; drive hardware for physical pose/heading gate | Core software checks pass; physical checks separately signed off |
| 4. Naming migration | Consistent maintained Java APIs and regenerated IO references | Phase 2 rules; coordinate files with runtime work | Usually none; runtime smoke checks use desktop | Full style gate passes; no unexplained behavior/string/value changes |
| 5. Mechanisms and vision | Valid references/limits, coherent shooting/readiness, robust vision | Phase 3 relevant checks | Required for retained mechanisms/cameras/calibration | Per-feature physical acceptance meets preselected criteria |
| 6. Reusable release | Season/config separation, supported modes/autos, retained utilities, reproducible release evidence | Relevant Phase 3/5 gates; supported season toolchain for 2027 release | Required for final hardware-supported release | All retained high-priority checks pass or feature is explicitly excluded |

Phases are review checkpoints, not a demand to leave people idle. With the robot unavailable, complete and review the desktop portions of Phases 1–4, P5.2's uncertainty/parser work, and the software portions of other packages whose prerequisites are met. Phase 5's physical reference, vision, and shooting checks and Phase 6's robot release gate stay open. Record `Awaiting hardware` on those specific checks; do not hold up unrelated commits or mark an unmet physical criterion passed. Confirm the hardware configuration and repeat affected checks when the robot returns.

**Begin with small runtime repairs.** After confirming the [Phase 1 record](PHASE_1_BASELINE_2027.md) against the current construction path, assign separate changes for H1 duplicate callbacks, H4 missing hood requirements, H7 uncertainty forwarding, and H8 flywheel simulation units/control modes. Place the short REAL/SIM wiring sequence after that check and H1, before changing simulation models; this gives newer programmers one clear construction path to follow. Use the applicable acceptance steps for each change; complete the wider mode, mechanism, and localization checks as their prerequisites become available. None of these repairs depends on renaming constants, changing `IO` casing, or installing Checkstyle. Tooling and naming can proceed alongside them with coordinated file ownership.

### Phase 1 — establish the baseline and reuse scope

The [Phase 1 record](PHASE_1_BASELINE_2027.md) already captures the construction map, configuration locations, and a short SIM startup observation. Its missing-data table distinguishes mentor decisions, future desktop checks, and measurements that require the robot. Verify the construction map against the current code; do not re-create the whole inventory. Phase 1 remains open until the retained scope and responsible people are named.

| Package | Deliverable and work | Recommendations / acceptance |
| --- | --- | --- |
| P1.1 | Preserve the 2026 application; inventory reusable, robot-specific, season-specific, and excluded code | H9; [inventory/provenance steps](REUSE_RECOMMENDATIONS_2027.md#acceptance-h9) |
| P1.2 | Define supported modes/mechanisms, current defects, verification environments, evidence locations, and required hardware access; record the current REAL/SIM constructor map and order | H8; [coverage matrix](REUSE_RECOMMENDATIONS_2027.md#acceptance-h8), [evidence protocol](REUSE_RECOMMENDATIONS_2027.md#how-to-run-and-record-acceptance-checks) |
| P1.3 | Record current configuration sources and collect available baseline telemetry; assign who will choose physical limits and reference procedures | H5/H6/M4; [reference criteria](REUSE_RECOMMENDATIONS_2027.md#acceptance-h5), [shot criteria](REUSE_RECOMMENDATIONS_2027.md#acceptance-h6), [logging procedure](REUSE_RECOMMENDATIONS_2027.md#acceptance-m4) |

Keep the recorded startup warnings and known failures visible so the team can distinguish an existing condition from a regression. If persistent logging is unavailable, mark a robot log pending; M4 implementation follows in Phase 3. Do not change gains merely to make the starting record appear healthy.

**Exit:** a reviewer can identify what will be reused, what will be excluded, what is currently unverified, and which measurements require the robot. Owners of physical criteria are assigned even if values must await measurements. No source behavior changes are required to complete this phase.

### Phase 2 — introduce coding-standard tooling

| Package | Deliverable and work | Recommendations / acceptance |
| --- | --- | --- |
| P2.1 | Preserve completed formatting/CI work; select naming/import rules and generated/vendor boundaries; pin compatible checker/runtime versions | H10/M6; partial implementation recorded above; [tooling checks](REUSE_RECOMMENDATIONS_2027.md#acceptance-h10), [workflow checks](REUSE_RECOMMENDATIONS_2027.md#acceptance-m6) |
| P2.2 | Add Checkstyle rules/reports and CI/editor integration; demonstrate valid and deliberately invalid examples; inventory existing violations | H10; [negative/positive probes](REUSE_RECOMMENDATIONS_2027.md#acceptance-h10) |

Keep tool installation and generated-code boundary changes separate from broad symbol renames. Existing naming violations are expected at this stage: make them visible with an explicitly temporary migration approach, then enforce the complete maintained source set at Phase 4. Passing tool-configuration probes is only partial H10 acceptance; a permanently warning-only checker is not the final deliverable.

**Exit:** formatter/checker responsibilities are clear, deliberately invalid examples fail for the right reason, normal verification does not modify tracked code, exclusions are reviewed, and Phase 4 has a concrete list of remaining naming work. No robot session is needed.

### Phase 3 — repair runtime foundations and verification infrastructure

| Package | Deliverable and work | Recommendations / acceptance |
| --- | --- | --- |
| P3.1 | Remove duplicate callbacks; define update/output ownership and shared loop timing | H1; [cycle-count procedure](REUSE_RECOMMENDATIONS_2027.md#acceptance-h1) |
| P3.2 | Separate REAL/SIM IO selection in individually buildable commits; declare REPLAY policy; keep subsystem construction order | H8/M2; [mode matrix](REUSE_RECOMMENDATIONS_2027.md#acceptance-h8), [ownership](REUSE_RECOMMENDATIONS_2027.md#acceptance-m2), [commit steps](#p32-separate-real-and-simulation-wiring-in-small-commits) |
| P3.3 | Document IO units/modes; correct simulation timing/units/control modes; add known input/output capture and declared gyro simulation/fallback support | H8/M3, initial subset; [sim contracts](REUSE_RECOMMENDATIONS_2027.md#acceptance-h8), [IO conversion checks](REUSE_RECOMMENDATIONS_2027.md#acceptance-m3), [SIM-to-REAL guardrails](#guardrails-for-carrying-sim-behavior-to-the-robot) |
| P3.4 | Implement one odometry/velocity path and deliberate heading/reset semantics; handle queue/lock lifecycle if retained | H2/H3; [sample pipeline](REUSE_RECOMMENDATIONS_2027.md#acceptance-h2), [heading matrix](REUSE_RECOMMENDATIONS_2027.md#acceptance-h3) |
| P3.5 | Correct requirements, mode gating, interruption/stop behavior; expose device/configuration validity and command diagnostics | H4/M3; [control matrix](REUSE_RECOMMENDATIONS_2027.md#acceptance-h4), [fault diagnostics](REUSE_RECOMMENDATIONS_2027.md#acceptance-m3) |
| P3.6 | Enable reliable persistent recording, metadata, and event/readiness diagnostics; retrieve a representative run | M4; [recording acceptance](REUSE_RECOMMENDATIONS_2027.md#acceptance-m4) |

Start with P3.1, then P3.2's wiring commits, then P3.3's basic IO contracts and model/output fixes. Develop P3.4's heading and odometry changes together as needed: controlled samples establish ordering/frames before joint physical drive checks. A kinematic gyro fallback may require the H2 and H8 changes in the same reviewed package. Avoid circular acceptance by distinguishing these software prerequisites from the final integrated motion checks.

P3.5's small hood-requirement repair can start alongside P3.1; use the lifecycle fix before its integrated acceptance. The broader ownership work can use output-capturing IO before physical mechanisms are cleared in Phase 5. Its physical mechanism rows remain pending until referencing/limits are verified; drivetrain checks use the team's existing controlled drive bring-up. P3.6 should be ready before Phase 5 calibration sessions so evidence is saved. Use early live captures for P3.1–P3.5 if persistent recording is still being completed.

#### P3.2: separate REAL and simulation wiring in small commits

Keep `Robot` responsible for lifecycle and its logging-mode switch. Keep `RobotContainer` responsible for subsystem construction, bindings, and autonomous composition. Put concrete device choices in small `RealRobotWiring` and `SimRobotWiring` classes whose methods create IO adapters **when called**. An eager bundle would change the present construction order: `Drive` starts `PhoenixOdometryThread` during its constructor, before later mechanisms are built. The same subsystem logic should continue to receive the same IO interfaces in either mode. This is a readability change, not a simulator-accuracy fix. [WPILib project structure](https://docs.wpilib.org/en/stable/docs/software/commandbased/structuring-command-based-project.html), [AdvantageKit IO interfaces](https://docs.advantagekit.org/data-flow/recording-inputs/io-interfaces/).

For example, the **proposed** `createFrontLeftModule()` method would return `new ModuleIOSim(TunerConstants.FrontLeft)` in `SimRobotWiring` and `new ModuleIOTalonFX(TunerConstants.FrontLeft)` in `RealRobotWiring`. `RobotContainer` would call `wiring.createFrontLeftModule()` at the existing front-left argument position of `new Drive(...)`. Repeat this pattern for the other active adapters, retaining their order and current configuration values. These methods/classes do not exist yet; this example explains the change to implement, rather than describing current code. `RobotContainer` continues to create the `Drive` subsystem and install its commands.

Use one code commit per row. Complete the desktop checks and record any pending physical check before starting the next row. Do not mix these commits with renames, gain changes, command fixes, or simulator physics repairs.

| Commit | Small change | Gate before the next commit |
| --- | --- | --- |
| P3.2a — SIM selection | Introduce a narrow `RobotWiring` interface with creator methods for gyro, named module positions, indexer, intake, shooter components, and cameras. Add `SimRobotWiring`; use it only in the existing SIM branch and call each creator at the existing construction point. Keep absent SIM vision/LEDs absent. | `spotlessCheck`, `build`, desktop SIM startup, and a source comparison with the P1.2 mode map. SIM must create no real motor/camera/LED adapter; the four modules keep their order. Existing no-op models remain known limitations. |
| P3.2b — REAL selection | Add `RealRobotWiring` and use it only in the existing REAL branch. Keep adapter IDs and construction order, both Limelight names/order, the vision consumer's pose/timestamp/uncertainty forwarding, and REAL-only `Leds.getInstance()` unchanged. | Build/formatting and SIM startup pass. Compare each REAL constructor with the baseline. Start the robot disabled and inspect device configuration/connection reports, camera observations, and LED startup before calling this robot-verified. If hardware is unavailable, record that gate as pending. |
| P3.2c — REPLAY policy | Reject the currently incomplete REPLAY mode clearly before replay-source setup or container construction. Keep full no-op IO construction and known-log verification in P6.2 if the team later chooses to support replay. Do not silently use simulated sensors for replay. | REAL/SIM results remain unchanged. Explicitly selected REPLAY gives the declared startup message, never reaches bindings with null subsystems, and constructs no real hardware. Restore the normal desktop mode afterward. |
| P3.2d — common construction | Select wiring once in `RobotContainer`; construct `Drive`, `Indexer`, `Intake`, and `Shooter` once, calling creators in the original order. Keep LED creation after intake on the REAL path and construct `Vision` only when cameras are present, after the shooter. Configure bindings once. | Build, SIM startup, and a source comparison of the REAL/SIM subsystem graph, constructor order, camera/LED presence, bindings, autonomous command, and telemetry keys pass. Repeat disabled robot startup because the common constructor path changed; record physical verification as pending if unavailable. |

For each row, record the base and resulting revision, checks actually run, environment, observed results, and pending hardware evidence using the [acceptance record](REUSE_RECOMMENDATIONS_2027.md#how-to-run-and-record-acceptance-checks). `./gradlew simulateJava` starts the desktop program, but the current build does not enable the simulation GUI by default; configure the Driver Station extension when testing controller bindings. A build or SIM startup cannot prove a physical device is wired or configured correctly. If a row fails, repair or revert that row before continuing. Each commit must be buildable on its own.

After P3.2, document the IO contract before changing flywheel SIM units in P3.3: method, units/range, sensor frame, stop semantics, and whether the adapter models physics or is intentionally no-op. `IntakeIOSim` and `IndexerIOSim` currently do not model fuel/mechanism behavior. Update the Technical Guide's construction path once P3.2d is accepted. In P3.4, separately review `Drive`'s global-mode check for the missing-gyro alert and its unconditional Phoenix thread start; coordinate any thread move with the odometry queue/lock work and require real timestamp checks. Keep broader naming migration in Phase 4 and any 2027 hardware configuration changes in Phase 6.

**Exit:** P3.2's wiring gates, H1, and the software parts of H2/H3/H4/H8/M3 pass on a named revision. Perform the physical startup and drive/heading checks and record them separately; if unavailable, the core can be marked software-verified but not hardware-verified. No mechanism may inherit physical approval merely because its command scheduling passed with a stub.

#### Guardrails for carrying SIM behavior to the robot

Implement these alongside P3.2–P3.5, before treating SIM results as useful evidence for a retained feature. They are small checks at existing boundaries, not a second command implementation for REAL. The same command, subsystem, state transition, and stop logic should run in both modes; only `*IO` construction and device-specific behavior differ. If a command needs a mode check to work in SIM, first identify the missing sensor or output contract and fix that boundary. Document any deliberate mode exception.

1. **Reject an unintended mode before constructing devices.** P3.2's wiring selection must choose exactly one adapter family. REAL must never be selected by a desktop diagnostic, SIM must never be selected on a roboRIO, and unfinished REPLAY must exit clearly before bindings are created. Compare the selected mode with `RobotBase.isReal()` at startup and fail with a readable message for an impossible combination. After selection, keep the adapter constructors lazy so SIM startup creates no real CTRE/REV device, Limelight adapter, or LED hardware object. Inspect the constructor map in each P3.2 commit; do not claim this check proves physical wiring.
2. **Make every IO boundary explicit.** For each retained device, record output method, units/range, control mode, sensor frame, connection/validity meaning, and stop behavior. Use names or typed quantities that expose units. For the flywheel, trace a 2400 RPM caller request to 40 RPS at `FlywheelIO.setVelocity()` and compare it with the measured `velocityRadPerSec` only after converting to the same shaft and unit. Repair `FlywheelIOSim`'s RPM/RPS comparison and its overwritten open-loop/stop requests in P3.3; otherwise a passing SIM flywheel run does not predict the REAL request. Define a neutral-output response and diagnostic for a non-finite runtime target; select and document either rejection or clamping for an out-of-range duty-cycle request before it reaches either adapter. Choose physical mechanism limits from hardware evidence in P5.1, not from simulator behavior.
3. **Do not let missing sensors look ready.** Where a command depends on feedback, readiness must require a current connected measurement, any applicable valid reference, and a small numeric error. A disconnected or no-op adapter must report its real status instead of manufacturing success. `IntakeIOSim` and `IndexerIOSim` currently do not model mechanism motion: use output-capturing IO to check command requests and interruption/stop behavior, and label fuel movement or position feedback `Awaiting hardware` until a model or real measurement can support it. Never equate a scheduled command with a completed physical action.
4. **Compare requests and observations using the same scenario.** For each retained behavior, save a short desktop trace with the requested target, applied/clamped target, control mode, measured value, validity flag, and stop/interruption result. Include a known input, expected output, observed output, and revision. Run the scenario through the shared subsystem with SIM or controlled-input IO; review the REAL adapter's corresponding unit conversion and device request in source. When hardware returns, replay the procedure at approved low-risk settings and compare the *meaning* and direction of signals before tuning gains or increasing output. A desktop diagnostic cannot establish encoder polarity, physical stops, or response time.
5. **Keep configuration changes visible.** Put robot-specific IDs/buses, gear ratios, inversion, camera names, and mechanism limits in reviewed configuration with recorded provenance. Check for missing values and duplicate device addresses on the same bus before enabling affected hardware; preserve named module and camera order from P1.2. A hardware revision invalidates the affected physical evidence and triggers a REAL configuration review plus the corresponding desktop/robot checks. Do not change shared commands simply to compensate for an unverified device direction or ratio.

The software acceptance for these guardrails is a clean build/style check, mode-startup check, reviewed REAL/SIM constructor map, IO contract table, and saved controlled-input traces for the affected feature. Add small automated contract or startup checks only where they catch a real regression reliably; a broad unit-test suite is not required. Hardware acceptance remains separate until a stable robot or suitable device bench is available, with bench scope stated explicitly.

### Phase 4 — migrate names without changing behavior

| Package | Deliverable and work | Recommendations / acceptance |
| --- | --- | --- |
| P4.1 | Rename constants/mutable configuration and ambiguous scalar fields while preserving values and units | M1; [rename checks](REUSE_RECOMMENDATIONS_2027.md#acceptance-m1) |
| P4.2 | Rename command factories/direct operations and coordinator/control classes; migrate IO types and regenerate generated references | M1; [symbol/IO checks](REUSE_RECOMMENDATIONS_2027.md#acceptance-m1) |
| P4.3 | Remove temporary naming migration exceptions, align documentation, and enable the full team style gate | H10/L4; [final tooling acceptance](REUSE_RECOMMENDATIONS_2027.md#acceptance-h10), [comment review](REUSE_RECOMMENDATIONS_2027.md#acceptance-l4) |

Use a small series of changes grouped by subsystem or API family. Prioritize misleading action/unit names over cosmetic casing. Coordinate with runtime fixes before renaming their shared APIs. Treat telemetry keys, camera names, serialized data, and named auto actions as separate contracts: preserve them in a Java-only rename. If a behavior defect is encountered, fix it in a separate focused change; it does not have to wait for the naming migration to finish.

**Exit:** clean build and full maintained-source style check; generated symbols resolve; no unexplained numeric, unit, sign, string-contract, requirement, or control-flow change. Naming acceptance does not establish runtime correctness; keep the separate runtime evidence.

### Phase 5 — validate retained mechanisms and vision

| Package | Deliverable and work | Recommendations / acceptance |
| --- | --- | --- |
| P5.1 | Establish references, feasible targets, boundary/recovery behavior, and relevant fault response for each retained mechanism | H5 plus physical H4/M3 rows; [boundary/reference procedure](REUSE_RECOMMENDATIONS_2027.md#acceptance-h5), [control transitions](REUSE_RECOMMENDATIONS_2027.md#acceptance-h4) |
| P5.2 | First repair discarded uncertainty; separately add adopted camera hardening and establish live pose quality | H7; [separate repair/hardening acceptance](REUSE_RECOMMENDATIONS_2027.md#acceptance-h7) |
| P5.3 | Consolidate shot solution/readiness/feed policy; recalibrate stationary shooting, then separately validate any moving-shot support | H6; [solution and shot procedure](REUSE_RECOMMENDATIONS_2027.md#acceptance-h6) |

P5.1 depends on command ownership and trustworthy IO. Prepare its software boundary/recovery logic and planned physical procedure while the robot is unavailable, but do not invent a reference position or safe travel limit from code. P5.2's uncertainty-forwarding repair and controlled estimator comparison can start during the first runtime fixes; parser hardening can also proceed independently. Live fusion acceptance depends on the Phase 3 pose pipeline and physical camera evidence. P5.3 can use a controlled known pose while vision work proceeds; camera-driven shooting acceptance requires the relevant P5.2 checks to pass. Preserve the distinction between no-ball control checks and actual fuel-shot calibration.

Schedule hardware sessions around explicit cases: referencing/limits first, control transitions second, then shot calibration. Set physical tolerances and success criteria before recording pass/fail. A blocked session should leave its cases pending and allow independent desktop work to continue. Do not choose new pass thresholds after seeing a failed result without documenting a reviewed requirement change and rerunning.

**Exit:** every retained mechanism has a reference/limit record, valid control behavior, and measured acceptance evidence. Vision meets its declared criteria. Shooting support states whether it includes stationary shots only or independently accepted moving shots. Mechanisms omitted from the new robot have documented exclusions rather than artificial completion claims.

### Phase 6 — extract and release the reusable foundation

| Package | Deliverable and work | Recommendations / acceptance |
| --- | --- | --- |
| P6.1 | Separate season/hardware configuration, clarify object ownership, and remove unsupported legacy code/dependencies | H9/M2/M7; [core extraction](REUSE_RECOMMENDATIONS_2027.md#acceptance-h9), [ownership](REUSE_RECOMMENDATIONS_2027.md#acceptance-m2), [disposition checks](REUSE_RECOMMENDATIONS_2027.md#acceptance-m7) |
| P6.2 | Finish all advertised runtime modes, including replay if retained; validate supported autos and deployment/editor workflow | H8/M5/M6; [mode matrix](REUSE_RECOMMENDATIONS_2027.md#acceptance-h8), [auto validation](REUSE_RECOMMENDATIONS_2027.md#acceptance-m5), [clean workflow](REUSE_RECOMMENDATIONS_2027.md#acceptance-m6) |
| P6.3 | Repair or remove retained helpers; complete optional utility/LED/calibration/comment cleanup | M8/L1–L4; [helpers](REUSE_RECOMMENDATIONS_2027.md#acceptance-m8), [LEDs](REUSE_RECOMMENDATIONS_2027.md#acceptance-l1), [geometry](REUSE_RECOMMENDATIONS_2027.md#acceptance-l2), [characterization](REUSE_RECOMMENDATIONS_2027.md#acceptance-l3), [comments/performance](REUSE_RECOMMENDATIONS_2027.md#acceptance-l4) |
| P6.4 | Build with the supported season toolchain, confirm configuration provenance, and rerun affected acceptance on the release revision and actual robot | H9 and retained feature checks; [season acceptance](REUSE_RECOMMENDATIONS_2027.md#acceptance-h9), [release checklist](#release-checklist) |

P6.1 inventory work starts in Phase 1; extraction uses corrected components rather than moving unreviewed code wholesale. P6.2 replay depends on a compatible saved log from P3.6. Auto loading/reference checks are independent of physical route validation; routes need accepted drive, field, and mechanism behavior. P6.3 work can proceed during hardware waits, but optional cleanup must not delay resolution of retained high-priority failures. Promote calibration-helper verification before using that helper to derive production values.

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

For example, an owner starting P3.1 would write: **scope** = remove `Shooter.periodic()`'s calls to registered child subsystems; **out of scope** = shooter readiness and SIM motor tuning; **prerequisite** = P1.2 baseline recorded; **desktop gate** = H1's 100-cycle counts plus `spotlessCheck build`; **hardware gate** = none for this lifecycle correction; **status** = `In progress` until evidence exists. This is an example work card, not a completed result. The owner replaces the example text with the actual revision, observations, and reviewer decision.

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
