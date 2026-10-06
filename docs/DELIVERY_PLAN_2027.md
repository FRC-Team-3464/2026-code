# 2027 robot software delivery plan

The 2026 code is our starting point, not the finished 2027 robot. Before branching, fix clear
defects and reusable code. Prioritize changes we can review and test on the 2026 robot before the
[CyberKnight Invitational](https://team195.com/invitational) on October 17, 2026. The 2027 robot
will use Systemcore; that migration belongs in the 2027 work.

## How to read the status

**Done** means the planned software or document change has merged into `mentor-review`.
**In progress** means some work is done but the task is still open. **Not started** means no
implementation has begun. A merged change is not automatically verified on a physical robot;
robot checks are listed as separate tasks below. GitHub tracks review status, and the
[Implementation Tracker](IMPLEMENTATION_TRACKER_2027.md) holds evidence and detailed open checks.

## Phase-based work packages

| Phase | Task | Status | Next step |
| --- | --- | --- | --- |
| 1 · Baseline | Record the 2026 code and configuration baseline | Done | Recheck it when hardware or configuration changes. |
| 1 · Baseline | Decide which mechanisms and features to carry into 2027 | Not started | Decide after 2027 robot requirements are known. |
| 2 · Tooling | Apply consistent formatting and CI checks | Done | Keep the checks running. |
| 2 · Tooling | Agree on naming rules and add a checker | Not started | Choose a small, maintainable rule set. |
| 3 · Runtime | Update shooter children once per robot cycle | Done | Verify on the deployed 2026 robot. |
| 3 · Runtime | Separate REAL/SIM wiring and define REPLAY behavior | Done | Check disabled startup on the deployed robot. |
| 3 · Runtime | Correct known flywheel/indexer SIM behavior and simulated heading | Done | Revisit models only when needed for a retained feature. |
| 3 · Runtime | Complete drive odometry and heading validation | In progress | Check remaining sampling and physical accuracy questions. |
| 3 · Runtime | Make controls and shooter commands safe across modes | In progress | Finish shared readiness work, then check it on the robot. |
| 3 · Runtime | Save logs and expose useful hood telemetry | Done | Verify file retrieval and behavior on the robot. |
| 4 · Naming | Rename unclear APIs without changing behavior | Not started | Start after the naming rules are agreed. |
| 5 · Robot checks | Check references, limits, and stop behavior of retained mechanisms | Not started | Use the deployed robot and mechanical limits. |
| 5 · Vision | Validate camera data and estimator uncertainty | Done | Check live camera pose quality on the robot. |
| 5 · Shooting | Verify aiming, readiness, and shot calibration | In progress | Finish software guards, then measure real shots. |
| 6 · Autos | Restore and verify the selected 2026 URI autos | In progress | Test only supported routes on the deployed robot. |
| 6 · 2027 release | Choose 2027 auto strategy, extract reusable code, and port to Systemcore | Not started | Wait for 2027 requirements and supported toolchain. |

The detailed acceptance criteria remain in the
[2027 Mentor Recommendations](REUSE_RECOMMENDATIONS_2027.md). Update a row when its status changes;
keep test results in the tracker rather than expanding this plan.

### Phase 1 — establish the baseline and reuse scope

The [Phase 1 record](PHASE_1_BASELINE_2027.md) contains the existing inventory and open hardware
facts. The 2027 feature decision remains open.

### Phase 3 — repair runtime foundations and verification infrastructure

#### P3.2: separate REAL and simulation wiring in small commits

The wiring change has merged. `RobotContainer` constructs the shared subsystems, while
`RealRobotWiring` and `SimRobotWiring` select device adapters. Disabled startup on the physical
robot remains to be checked.

### Release checklist

- [ ] Select the features and modes the 2027 robot will support.
- [ ] Build and test with the supported 2027 Systemcore toolchain.
- [ ] Verify retained mechanisms, vision, and autos on the final robot and configuration.
- [ ] Confirm logs can be retrieved and a known working build is available.
