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
  // IO representation + inputs and outputs for the turret
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final TurretIOOutputs outputs = new TurretIOOutputs();

  // Target angle for the turret to move to
  private Rotation2d targetAngle = Rotation2d.kZero;

  private boolean atGoal = false;
  private Debouncer atGoalDebouncer = new Debouncer(0.1, DebounceType.kFalling);

  /** Creates a new Turret. */
  public Turret(TurretIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    // Log the turret position for AdvantageScope visualization
    RobotVisualizer.getInstance().setTurretAzimuthAngle(Rotation2d.fromRadians(inputs.positionRad));
  }

  @Override
  public void periodicAfterScheduler() {
    // Apply the outputs (run the turret differently based on the control mode we
    // want to use, usually determined by driver inputs)
    io.applyOutputs(outputs);
    Logger.recordOutput("Turret/Mode", outputs.mode.toString());
    Logger.recordOutput(("Turret/TargetAngle"), targetAngle);
    Logger.recordOutput(("Turret/TargetAngleDegrees"), targetAngle.getDegrees());
    Logger.recordOutput(
        ("Turret/TargetOffsetDegrees"),
        targetAngle.minus(Rotation2d.fromRadians(inputs.positionRad)).getDegrees());
  }

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

          this.targetAngle = targetAngle;
          setPosition(targetAngle.unaryMinus());
        },
        this);
  }

  /**
   * Set the target angle for the turret.
   *
   * <p>This will clamp the angle to be within the maximum and minimum rotation of the turret.
   *
   * @param position A {@link Rotation2d} object representing the target position of the turret.
   */
  public void setPosition(Rotation2d position) {
    // Change the target angle to the specified position
    targetAngle = position;

    // Ensure we're using the target angle for closed-loop control
    outputs.mode = TurretIOOutputMode.CLOSED_LOOP;
    outputs.closedLoopTarget = position;

    atGoal =
        atGoalDebouncer.calculate(
            Math.abs(position.getRadians() - inputs.positionRad) < TurretConstants.kAngleTolerance);
  }

  public void setOpenLoop(double output) {
    // Ensure we're using open-loop control
    outputs.mode = TurretIOOutputMode.OPEN_LOOP;

    // Safety checks
    if (inputs.positionRad > TurretConstants.kMaxTurretAngleRad
        || inputs.positionRad < TurretConstants.kMinTurretAngleRad) {
      outputs.openLoopOutput = 0.0;
    } else {
      outputs.openLoopOutput = MathUtil.clamp(output, -1.0, 1.0);
    }
  }

  public void stop() {
    // Change to open loop output of 0 to truly stop the motor
    outputs.mode = TurretIOOutputMode.OPEN_LOOP;
    outputs.openLoopOutput = 0.0;
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

  public void zero() {
    // Tell the turret that the current position should be treated as "zero"
    // (perfectly straight forward)
    io.zero();
  }
}
