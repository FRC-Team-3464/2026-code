// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.hood;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotVisualizer;
import frc.robot.subsystems.shooter.ShooterConstants.HoodConstants;
import frc.robot.subsystems.shooter.TrajectoryCalculator;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  // IO representation + inputs for the hood
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  private double targetAngleRad = 0.0;
  private boolean closedLoop = false;

  // Boolean representing if the hood is at its target RPM
  private boolean atGoal = false;
  private Debouncer atGoalDebouncer = new Debouncer(0.2, DebounceType.kRising);

  /** Creates a new Hood. */
  public Hood(HoodIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // Typical IO input cycle
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);

    // Readiness requires a valid position while actively controlling to a target. Passing false
    // during manual control or a sensor fault clears any previously true, stale result.
    atGoal =
        atGoalDebouncer.calculate(
            inputs.connected
                && closedLoop
                && Math.abs(targetAngleRad - inputs.positionRad) < HoodConstants.kAngleTolerance);
    if (closedLoop) {
      io.setAngle(targetAngleRad);
    }

    // The measured angle is already in Hood/PositionRad. Log the active target and its error here
    // every cycle; values recorded only by trackTarget() appeared frozen after that command ended.
    // No position target applies in open-loop mode; disconnected feedback cannot give a valid
    // error.
    Logger.recordOutput("Hood/Mode", closedLoop ? "CLOSED_LOOP" : "OPEN_LOOP");
    Logger.recordOutput("Hood/TargetAngleRad", closedLoop ? targetAngleRad : Double.NaN);
    Logger.recordOutput(
        "Hood/PositionErrorRad",
        closedLoop && inputs.connected ? targetAngleRad - inputs.positionRad : Double.NaN);

    // Log the exact mechanism position for visualization in AdvantageScope
    RobotVisualizer.getInstance().setTurretHoodAngle(inputs.positionRad);
  }

  public Command trackTarget(Supplier<Translation2d> targetSupplier) {
    // Run command: Continuously repeats until command ends
    return Commands.run(
        () -> {
          // Get the current target (typically the hub)
          Translation2d target = targetSupplier.get();
          // Get the current robot pose
          Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
          // Use the lookup table to get the goal angle based on distance to the target
          setAngle(TrajectoryCalculator.calculateHoodAngle(target, robotPose));
        },
        this); // Reference to current subsystem
  }

  /** Move the hood all the way down. */
  public Command down() {
    return Commands.run(() -> setAngle(0), this);
  }

  /**
   * Moves the hood manually while a signed output is requested, then holds its last measured angle.
   *
   * <p>The command requires this subsystem, so it cleanly interrupts any automatic hood command.
   * {@link #setManualOutput(double)} applies the configured travel limits on every scheduler cycle.
   *
   * @param output signed motor duty cycle, from -1.0 to 1.0
   */
  public Command manualControl(double output) {
    return Commands.runEnd(() -> setManualOutput(output), this::holdCurrentPosition, this);
  }

  /**
   * Sets the hood to the target angle.
   *
   * @param angle The target angle (in radians).
   */
  public void setAngle(double angle) {
    closedLoop = true;
    targetAngleRad = angle;
  }

  /** Run the hood motor at the specified open-loop value without position-limit checking. */
  public void setOpenLoop(double output) {
    closedLoop = false;
    io.setOpenLoop(output);
  }

  /**
   * Runs manual hood movement while preventing commands farther beyond the configured travel range.
   *
   * <p>This guard uses the relative encoder position, so it is valid only after the hood has been
   * placed at a known startup reference. If the hood is already outside a limit, this method still
   * permits movement back toward the allowed range.
   */
  public void setManualOutput(double output) {
    // Position limits are only meaningful when the latest encoder reading is valid. Stop instead of
    // moving from a stale or default position if the IO layer reports a disconnected sensor.
    if (!inputs.connected) {
      setOpenLoop(0.0);
      return;
    }

    boolean movingPastMaximum = output > 0.0 && inputs.positionRad >= HoodConstants.kMaxAngleRad;
    boolean movingPastMinimum = output < 0.0 && inputs.positionRad <= HoodConstants.kMinAngleRad;
    setOpenLoop(movingPastMaximum || movingPastMinimum ? 0.0 : output);
  }

  /**
   * Stops manual output and holds the most recently sampled hood angle.
   *
   * <p>The position was sampled by {@link #periodic()} near the beginning of the current robot
   * loop. Actual stopping accuracy still depends on motor braking, mechanism inertia, and
   * controller tuning, so it must be confirmed on the physical mechanism.
   */
  public void holdCurrentPosition() {
    // Do not turn a stale position into a closed-loop target. Remaining stopped is the only safe
    // behavior until position feedback becomes valid again.
    if (!inputs.connected) {
      setOpenLoop(0.0);
      return;
    }

    setOpenLoop(0);
    setAngle(inputs.positionRad);
  }

  public double getPosition() {
    return inputs.positionRad;
  }

  public double getVelocity() {
    return inputs.velocityRadPerSec;
  }

  public boolean atGoal() {
    return atGoal;
  }
}
