// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter.turret;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotState;
import frc.robot.RobotVisualizer;
import frc.robot.subsystems.shooter.ShooterConstants.TurretConstants;
import frc.robot.subsystems.shooter.turret.TurretIO.TurretIOOutputMode;
import frc.robot.subsystems.shooter.turret.TurretIO.TurretIOOutputs;
import frc.robot.util.FullSubsystem;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends FullSubsystem {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final TurretIOOutputs outputs = new TurretIOOutputs();

  private Rotation2d targetAngle = Rotation2d.kZero;

  private boolean atGoal = false;
  // Require 0.1 seconds continuously at the target before reporting ready, but clear readiness
  // immediately when the turret is no longer at the target.
  private final Debouncer atGoalDebouncer = new Debouncer(0.1, DebounceType.kRising);
  private boolean positionControlActive = false;

  /**
   * Creates a turret subsystem using the selected hardware or simulation adapter.
   *
   * @param io adapter that reads turret inputs and applies staged outputs
   */
  public Turret(TurretIO io) {
    this.io = io;
  }

  /** Refreshes turret inputs and updates readiness from the active position request. */
  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    // Readiness requires feedback that the IO adapter reports as connected and an active position
    // request. Rising-edge debounce requires the measured angle to remain within tolerance. Once
    // the IO adapter reports any of those conditions as invalid, readiness clears immediately.
    atGoal =
        atGoalDebouncer.calculate(positionControlActive && measurementMatchesTarget(targetAngle));
    Logger.recordOutput("Turret/AtGoal", atGoal);

    RobotVisualizer.getInstance().setTurretAzimuthAngle(Rotation2d.fromRadians(inputs.positionRad));
  }

  /** Applies the output selected by commands during the current scheduler cycle. */
  @Override
  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
    Logger.recordOutput("Turret/Mode", outputs.mode.toString());
    Logger.recordOutput(("Turret/TargetAngle"), targetAngle);
    Logger.recordOutput(("Turret/TargetAngleDegrees"), targetAngle.getDegrees());
    Logger.recordOutput(
        ("Turret/TargetOffsetDegrees"),
        targetAngle.minus(Rotation2d.fromRadians(inputs.positionRad)).getDegrees());
  }

  /**
   * Creates a command that continuously points the turret toward a field-relative target.
   *
   * @param targetSupplier supplier for the target translation in field coordinates
   * @return command requiring this turret subsystem
   */
  public Command trackTarget(Supplier<Translation2d> targetSupplier) {

    return Commands.run(
        () -> {
          Translation2d target = targetSupplier.get();
          Pose2d robotPose = RobotState.getInstance().getEstimatedPose();

          Translation2d turretOffset =
              TurretConstants.kRobotToTurret.getTranslation().toTranslation2d();

          // Turret position in field coordinates
          Translation2d turretFieldPos =
              robotPose.getTranslation().plus(turretOffset.rotateBy(robotPose.getRotation()));

          // Vector from turret -> target (field frame)
          Translation2d deltaField = target.minus(turretFieldPos);
          Logger.recordOutput("Turret Target Distance X", deltaField.getX());
          Logger.recordOutput("Turret Target Distance", deltaField.getDistance(target));
          Logger.recordOutput("Turret Target Norm", deltaField.getNorm());
          Logger.recordOutput("Turret Target Distance Y", deltaField.getY());
          Logger.recordOutput("Turret Target Angle?", deltaField.getAngle().getDegrees());

          // Convert to robot frame
          Translation2d deltaRobot = deltaField.rotateBy(robotPose.getRotation().unaryMinus());

          // Angle turret should point (robot-relative)
          Rotation2d targetAngle =
              Rotation2d.fromRadians(Math.atan2(deltaRobot.getY(), deltaRobot.getX()));

          setPosition(targetAngle.unaryMinus());
        },
        this);
  }

  /**
   * Stages a closed-loop turret angle for application after the scheduler finishes.
   *
   * @param position requested robot-relative turret angle
   */
  public void setPosition(Rotation2d position) {
    double targetChangeRad = Math.abs(position.getRadians() - targetAngle.getRadians());
    boolean measurementMatchesNewTarget = measurementMatchesTarget(position);

    // Tracking commands call this method every scheduler cycle. Preserve accumulated settling time
    // only when the repeated request still describes the same satisfied target. Starting position
    // control, moving the target beyond tolerance, or missing the requested angle begins a new
    // settling period.
    if (!positionControlActive
        || targetChangeRad >= TurretConstants.kAngleTolerance
        || !measurementMatchesNewTarget) {
      clearReadiness();
    }

    positionControlActive = true;
    targetAngle = position;

    outputs.mode = TurretIOOutputMode.CLOSED_LOOP;
    outputs.closedLoopTarget = position;
  }

  /**
   * Stages an open-loop turret output and clears position readiness.
   *
   * @param output requested motor duty cycle, clamped to {@code -1.0} through {@code 1.0}
   */
  public void setOpenLoop(double output) {
    positionControlActive = false;
    clearReadiness();
    outputs.mode = TurretIOOutputMode.OPEN_LOOP;
    if (inputs.positionRad > TurretConstants.kMaxTurretAngleRad
        || inputs.positionRad < TurretConstants.kMinTurretAngleRad) {
      outputs.openLoopOutput = 0.0;
    } else {
      outputs.openLoopOutput = MathUtil.clamp(output, -1.0, 1.0);
    }
  }

  /** Stages zero open-loop output and clears position readiness. */
  public void stop() {
    setOpenLoop(0.0);
  }

  /** Returns the latest measured turret angle in radians. */
  public double getPosition() {
    return inputs.positionRad;
  }

  /** Returns the latest measured turret angular velocity in radians per second. */
  public double getVelocity() {
    return inputs.velocityRadPerSec;
  }

  /**
   * Returns true when position control has held its target within tolerance while the IO adapter
   * reports connected feedback.
   */
  public boolean atGoal() {
    return atGoal;
  }

  /** Resets the IO adapter's relative turret position. */
  public void zero() {
    io.zero();
  }

  /** Clears both the published result and any partially accumulated debounce time. */
  private void clearReadiness() {
    atGoal = false;
    atGoalDebouncer.calculate(false);
  }

  /** Checks the latest connected position measurement against a requested target. */
  private boolean measurementMatchesTarget(Rotation2d position) {
    return inputs.connected
        && Math.abs(position.getRadians() - inputs.positionRad) < TurretConstants.kAngleTolerance;
  }
}
