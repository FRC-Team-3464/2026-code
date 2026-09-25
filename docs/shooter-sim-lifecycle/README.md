# Shooter SIM lifecycle diagnostic: repeatable procedure

Use this folder when changing scheduler ownership, shooter coordination, or SIM IO timing. It supplies the complete temporary [counter helper](ShooterLifecycleProbe.java) and the exact edits needed for the September 2026 code layout. The original requirement is [the shooter lifecycle acceptance check](../REUSE_RECOMMENDATIONS_2027.md#acceptance-h1); the completed run is in the [September 2026 results](RESULTS_2026-09-25.md).

The shooter callback change is in classes used by **both REAL and SIM**. This procedure checks **how many times** the shared software stages run in SIM, where independent counters can also observe simulated motor steps. It does not test physical motor direction, limits, or shooting accuracy. Adapt the file names and loop period if the 2027 code changes. Remove the helper and every call to it after collecting evidence.

## 1. Prepare the temporary helper

1. Stop the running simulation and run `git status --short`. Note any existing edits so you do not remove a teammate's work during cleanup.
2. Copy [ShooterLifecycleProbe.java](ShooterLifecycleProbe.java) to `src/main/java/frc/robot/ShooterLifecycleProbe.java`. Keep its `package frc.robot;` line. This file is a **sample stored outside `src`**; it does nothing until copied and called.
3. Read its event names. Each counter is independent. `completeRobotCycle()` records a completed loop and prints one snapshot every 20 loops. `DS_MODE` distinguishes Disabled from Disconnected. `OP_RB` reports the operator right-bumper input, and `TRACKING_SCHEDULED` checks the exact command bound to that input.

On macOS or Linux, the copy command from the repository root is:

```sh
cp docs/shooter-sim-lifecycle/ShooterLifecycleProbe.java src/main/java/frc/robot/ShooterLifecycleProbe.java
```

## 2. Add one probe at each source location

Make only the insertions shown below. In each file outside package `frc.robot`, add `import frc.robot.ShooterLifecycleProbe;` alongside the existing imports. Do **not** add a second call to any `periodic()` method.

1. In [Robot.java](../../src/main/java/frc/robot/Robot.java), put this immediately after `RobotVisualizer.getInstance().log("Mechanism3d/Robot");`, as the last statement in `robotPeriodic()`:

   ```java
   ShooterLifecycleProbe.completeRobotCycle();
   ```

   This snapshot runs after `CommandScheduler.run()` and `FullSubsystem.runAllPeriodicAfterScheduler()`. If an earlier stage fails, it does not count as a completed loop.

2. In [Hood.java](../../src/main/java/frc/robot/subsystems/shooter/hood/Hood.java), add this after `RobotVisualizer.getInstance().setTurretHoodAngle(inputs.positionRad);`, before the closing brace of `periodic()`:

   ```java
   ShooterLifecycleProbe.record(ShooterLifecycleProbe.Event.HOOD_PERIODIC);
   ```

3. In [Turret.java](../../src/main/java/frc/robot/subsystems/shooter/turret/Turret.java), add one distinct counter at the end of **each** method:

   ```java
   // Last statement of periodic():
   ShooterLifecycleProbe.record(ShooterLifecycleProbe.Event.TURRET_PERIODIC);

   // Last statement of periodicAfterScheduler():
   ShooterLifecycleProbe.record(ShooterLifecycleProbe.Event.TURRET_POST_SCHEDULER);
   ```

   Keep these events separate. An earlier diagnostic accidentally recorded `TURRET_POST_SCHEDULER` in both methods; that produced a zero periodic count and a doubled post-scheduler count.

4. In [Flywheel.java](../../src/main/java/frc/robot/subsystems/shooter/flywheel/Flywheel.java), add this after `SmartDashboard.putNumber("Flywheel Setpoint", goalRPM);`, at the end of `periodic()`:

   ```java
   ShooterLifecycleProbe.record(ShooterLifecycleProbe.Event.FLYWHEEL_PERIODIC);
   ```

5. In [HoodIOSim.java](../../src/main/java/frc/robot/subsystems/shooter/hood/HoodIOSim.java), add the two `ShooterLifecycleProbe.record(...)` lines inside `updateInputs()`, at the stated positions. The other lines below are **existing context**; do not duplicate them:

   ```java
   sim.update(0.02); // Existing model step; keep the original line.
   ShooterLifecycleProbe.record(ShooterLifecycleProbe.Event.HOOD_SIM_STEP);

   // After the existing final input assignment:
   inputs.currentDrawAmps = sim.getCurrentDrawAmps();
   ShooterLifecycleProbe.record(ShooterLifecycleProbe.Event.HOOD_IO_UPDATE);
   ```

6. In [TurretIOSim.java](../../src/main/java/frc/robot/subsystems/shooter/turret/TurretIOSim.java), add only the two probe lines shown among the existing context:

   ```java
   sim.update(0.02); // Existing model step; keep the original line.
   ShooterLifecycleProbe.record(ShooterLifecycleProbe.Event.TURRET_SIM_STEP);

   // After the existing final input assignment:
   inputs.currentDrawAmps = sim.getCurrentDrawAmps();
   ShooterLifecycleProbe.record(ShooterLifecycleProbe.Event.TURRET_IO_UPDATE);
   ```

7. In [FlywheelIOSim.java](../../src/main/java/frc/robot/subsystems/shooter/flywheel/FlywheelIOSim.java), add only the two probe lines shown among the existing context:

   ```java
   sim.update(0.02); // Existing model step; keep the original line.
   ShooterLifecycleProbe.record(ShooterLifecycleProbe.Event.FLYWHEEL_SIM_STEP);

   // After the existing final input assignment:
   inputs.currentDrawAmps = sim.getCurrentDrawAmps();
   ShooterLifecycleProbe.record(ShooterLifecycleProbe.Event.FLYWHEEL_IO_UPDATE);
   ```

8. In [DriverControls.java](../../src/main/java/frc/robot/control/DriverControls.java), add imports for `edu.wpi.first.wpilibj2.command.Command` and `frc.robot.ShooterLifecycleProbe`. Inside `configureOperatorControls()`, replace **only** the existing operator right-bumper binding with this block:

   ```java
   Command trackingCommand =
       shooter.trackAndShootAtTargetFullRealCommandLatestGoodUseThisOne(
           () -> RobotState.getInstance().getShooterTarget());
   ShooterLifecycleProbe.setTrackingCommand(trackingCommand);
   operator.rightBumper().whileTrue(trackingCommand);
   ```

   The same `trackingCommand` object is passed to the trigger and the helper. Creating a second command solely for the probe would give an invalid scheduling result. Leave all other bindings unchanged.

## 3. Compile and run SIM

From the repository root, compile first. The repository uses the WPILib 2026 JDK; select that JDK in your IDE or run:

```sh
JAVA_HOME="$HOME/wpilib/2026/jdk" ./gradlew compileJava
```

On Windows, use `gradlew.bat compileJava` from a terminal configured with the WPILib JDK.

Launch **WPILib: Simulate Robot Code** in VS Code and choose **Sim GUI**, or run `./gradlew simulateJava` with the WPILib JDK. Do not use an ordinary Java `Main` launch configuration because it does not set up the WPILib simulator in the same way. Keep the simulation running while collecting all four cases. The `SHOOTER_LIFECYCLE` lines appear in the robot console. The [WPILib Simulation GUI guide](https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/simulation-gui.html) shows the mode controls and keyboard-joystick setup.

## 4. Capture four stable 100-cycle intervals

In the Simulation GUI's **Robot State** panel, click **Disabled**. If the panel says **Disconnected**, the count lines are *not* disabled evidence even though callbacks still run. The operator controller is port 1 in this code. Without a physical gamepad, drag **Keyboard 0** from **System Joysticks** onto **Joystick[1]**. Under **DS → Keyboard 0 Settings**, map a key to Xbox button 6, the right bumper.

For each row, wait until the new state has appeared in at least two `SHOOTER_LIFECYCLE` snapshots before choosing a starting snapshot. Save an ending snapshot whose `ROBOT_CYCLE` is exactly 100 greater. Keep the mode and input unchanged across the entire interval; exclude the cycles when you click a mode or press/release the key.

| Case | Simulation GUI action | Required fields on both snapshots |
| --- | --- | --- |
| Disabled | Select **Disabled**; do not press the bumper | `DS_MODE=DISABLED`, `TRACKING_SCHEDULED=false` |
| Enabled, idle | Select **Teleoperated**; do not press the bumper | `DS_MODE=TELEOP`, `OP_RB=false`, `TRACKING_SCHEDULED=false` |
| Tracking | Hold the mapped button-6 key; verify the button lights up on `Joystick[1]` | `DS_MODE=TELEOP`, `OP_RB=true`, `TRACKING_SCHEDULED=true` |
| After release | Release the key; verify the button goes dark | `DS_MODE=TELEOP`, `OP_RB=false`, `TRACKING_SCHEDULED=false` |

For every case, subtract the starting value from the ending value for **each of the eleven counters**. Each difference must be exactly 100. The three SIM models each advance `100 × 0.02 = 2.0` simulated seconds. Do not use wall-clock time or merely compare final values; the counts can have different starting values after startup. Repeated writes to a single telemetry key are not a substitute for these counters.

For example, a valid tracking interval in the September 2026 run began at `ROBOT_CYCLE=1640` and ended at `ROBOT_CYCLE=1740`. All eleven counter values rose from 1640 to 1740, and both snapshots showed `DS_MODE=TELEOP OP_RB=true TRACKING_SCHEDULED=true`. An interval with equal counts but `TRACKING_SCHEDULED=false` would not prove that tracking ran.

Record the two complete console lines, the mode and button state, the results, and any warnings in the implementation tracker. If `OP_RB=true` but `TRACKING_SCHEDULED=false`, check that the keyboard is assigned to `Joystick[1]`, Teleoperated is enabled, and the binding uses the same command object. Do not claim a tracking result until the scheduled flag is true. A missing driver controller on port 0 may produce a warning but does not itself invalidate the operator port-1 count check.

## 5. Remove the temporary code and check the final version

1. Delete `src/main/java/frc/robot/ShooterLifecycleProbe.java` after saving the snapshots. Remove every `ShooterLifecycleProbe` import and call inserted in step 2. Restore the original right-bumper binding in `DriverControls.java` without changing its command factory or requirements:

   ```java
   operator
       .rightBumper()
       .whileTrue(
           shooter.trackAndShootAtTargetFullRealCommandLatestGoodUseThisOne(
               () -> RobotState.getInstance().getShooterTarget()));
   ```

   Remove the temporary `Command` import if nothing else uses it. Review each diff; do not run `git restore` on files containing unrelated work.
2. Run `rg -n 'ShooterLifecycleProbe' src/main/java`. It should return no matches. Check that [Shooter.java](../../src/main/java/frc/robot/subsystems/shooter/Shooter.java) does not manually call registered child `periodic()` methods; the scheduler should own those callbacks. The plain drive `Module` helpers still have their explicit update owner.
3. Run the final checks:

   ```sh
   JAVA_HOME="$HOME/wpilib/2026/jdk" ./gradlew spotlessCheck build
   git diff --check
   ```

4. Restart SIM using the **probe-free** code. Confirm startup and, in Teleoperated, expand **NetworkTables → AdvantageKit → RealOutputs → Turret → Mode** in the Simulation GUI. The separate `AdvantageKit → Turret` branch contains sensor inputs such as `PositionRad` and `Connected`. Watch the output mode while holding and releasing the operator right bumper: idle `OPEN_LOOP`, tracking `CLOSED_LOOP`, then `OPEN_LOOP` after release. Record the observed values and warnings. This final smoke check is separate from the earlier counted run.
5. Review the candidate changes through Git or a pull request. Mark the implementation `Ready for review` only when the measured intervals, cleanup, final-version checks, and candidate changes are available. Git keeps the exact version history; do not copy commit IDs into this guide. A reviewer decides whether to mark it `Accepted`.

The sample helper remains in this documentation folder for future use. It is outside the Gradle Java source target and should not be copied into a 2027 robot deployment.
