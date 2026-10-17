# FRC Team 3464 "Sim-City" 2026 Robot Code
This repository contains all the robot code used by Team 3464 "Sim-City" during the 2026 FRC season "REBUILT."
<br> <br>
The code has been extensively documented to be used as reference in training and for future seasons.

If you are new to robotics, start with the [illustrated, interactive Robot Parts and Control Map](docs/ROBOT_PARTS_AND_CONTROL_MAP.html), or read its [text version](docs/ROBOT_PARTS_AND_CONTROL_MAP.md). It shows what each mechanism does, which Java classes and libraries control it, and how driver input reaches the hardware or simulation.

For the swerve drivetrain specifically, see the [class and sequence diagrams](docs/SWERVE_DRIVE_DIAGRAMS.md).

For a detailed explanation of the architecture, robotics concepts, subsystems, controls, and current implementation limitations, read the [Technical Guide](docs/TECHNICAL_GUIDE.md).

For mentor recommendations and team expectations before reusing this code, including Java naming, analysis tools, and detailed acceptance procedures, read the [2027 Mentor Recommendations](docs/REUSE_RECOMMENDATIONS_2027.md). For the current task list and status, read the [2027 Delivery Plan](docs/DELIVERY_PLAN_2027.md).

Track implementation progress and verification separately in the [2027 Implementation Tracker](docs/IMPLEMENTATION_TRACKER_2027.md); the original recommendations remain unchanged.

The [Phase 1 Record](docs/PHASE_1_BASELINE_2027.md) separates facts checked against the current source and desktop run from decisions and robot measurements still needed before reuse.

For a separate assessment of how the design follows WPILib and AdvantageKit guidance, and which changes are team design choices, read the [Architecture Review](docs/ARCHITECTURE_REVIEW.md).

To check formatting, run `./gradlew spotlessCheck`. Run `./gradlew spotlessApply` when you want to format files, then review and stage the resulting changes yourself. CI runs `spotlessCheck` and `build`.

The `git-hooks/pre-commit` checks staged changes for whitespace errors and runs `spotlessCheck` before each commit. Spotless checks the working tree, including unstaged changes, so an unrelated unformatted file can also block a commit. The hook does not change or stage files. To enable it locally, first check whether you already have a custom hook path with `git config --local --get core.hooksPath`. If you do not, run `git config --local core.hooksPath git-hooks`. This Git setting affects only your local checkout; CI checks formatting for everyone even if they do not enable the hook.

## Saved logs for AdvantageScope

REAL saves `.wpilog` recordings to `/U/logs` on a FAT32 USB drive connected to the roboRIO. SIM saves them in the project's ignored `logs/` folder. Live NetworkTables viewing remains available in both modes.

After a run, copy the recording from the USB drive (after safely shutting down the robot) or from `logs/`, then open it in AdvantageScope. Open the Metadata tab for the build and runtime mode; compare mechanism targets, measured inputs, and Driver Station state along the timeline.

If the destination cannot be opened, AdvantageKit reports `Failed to open output log file`. Live telemetry is not proof that recording worked: verify a new file exists. Restore writable storage and restart the program; this setup has no alternate recording destination. Check available USB space before robot sessions.

Opening a saved log in AdvantageScope does not require this project's unsupported REPLAY mode.

## Structure

### Autonomous Routine Names

The 51 routines listed in the renamed FRC Autos Time Reference Sheet use matching filenames in `src/main/deploy/pathplanner/autos/`. Use uppercase tokens separated by hyphens, with the start position first. For example, `LT-ATW-DP`, `MS-LT-ATW-ASST`, and `RT-LT-MOVESHOOT-CLIMB`. Dual-shot variants append `-2SHOT`, including after `CLIMB`.

| Token | Meaning |
| --- | --- |
| LT / MS / RT | Left / middle / right start |
| ATW | Around the World |
| DP / OP | Depot / outpost |
| SUTO | Shoot position |
| REP | Repetitive |
| BUMP | Bump route |
| 2SHOT | Dual shot |
| HALF | Half Around the World |
| NOIN | Without intake |
| MOVESHOOT | MoveShot |
| LOCK | Locked auto |
| ASST / HOARD | Assist / hoard |
| CLIMB | Ends with a climb |

The URI routines use a `URI-` prefix. The autonomous chooser includes `URI-MS-SUTO`, `URI-MS-DP-SUTO-CYCLE-INTAKE`, `URI-MS-OP-SUTO-CYCLE-INTAKE`, `URI-LT-DP-SUTO`, and `URI-RT-OP-SUTO`, plus the left and right intake-retraction variants. `URI-MS-DP-SUTO-CYCLE-INTAKE` starts in the middle, deploys the intake while driving to the depot, collects for one additional second, then returns to the left shooting position and shoots for ten seconds. During shooting, it uses the same intake extension/retraction cycle as `URI-RT-OP-SUTO-CYCLE-INTAKE`. Its new `C to Depot` path is limited to 1 m/s and 1 m/s²; validate the route and deployed-intake clearance in simulation and on the field before competition use.

`URI-MS-OP-SUTO-CYCLE-INTAKE` starts in the middle, deploys the intake while following `C to Outpost`, waits four seconds at the outpost for loading, then follows `Outpost Int Ext to Right Tower`. It uses the same ten-second shooting and intake cycle as `URI-RT-OP-SUTO-CYCLE-INTAKE`. The new outbound path uses the existing outpost limits of 3 m/s and 3 m/s²; validate its route and deployed-intake clearance before competition use.

These names identify complete `.auto` routines. The shared `.path` filenames and command names inside each routine retain their existing names.

### Root Repository Structure
- `.github` - CI/CD workflows for GitHub
- `.vscode/` and `.wpilib` - Configurations and settings for VSCode and WPILib respectively
- `gradle/` - Gradle configuration
- `src/` -  All code for controlling the robot
- `vendordeps/` - Third-party software imports (REV Robotics/CTRE Hardware APIs, AdvantageKit, PathPlanner, etc.)
- Everything else - misc. configuration files, licenses, and other

### Robot Code Structure
After traveling down the `src/main/java/frc/robot` rabbit hole, you will find the main robot code. It is organized as follows:
- `commands/` - Robot-wide commands or commands so large they deserve their own files
- `control/` - Interfaces and classes for handling driver input and other bindings
- `subsystems/` - Pretty self-explanatory: contains all IO implementations, constants, and classes for each subsystem
- `util/` - Any misc. files that contain utility classes to make programming the robot a little easier
- `Constants.java` - Robot-wide constants
- `Main.java` - Program entry point (don't modify)
- `Robot.java` - Main robot code class
- `RobotContainer` - Holds all subsystem instances and some periodic functions
- `RobotState` - The single source of truth for robot pose and anything that depends on it
- `RobotVisualizer` - Logs mechanism poses so they can be visualized in AdvantageScope
